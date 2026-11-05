//! Equation-driven sketch curves: a `spline` entity whose points come from formulas.
//!
//! Wire: `{"type":"spline","id":"e1","pts":[..],"equation":{"x":"R*cos(t)","y":"R*sin(t)","t":[0,360],"tol":0.01}}`.
//! The formulas are read by the expression evaluator of dimension fields (`expr.rs`: trigonometry in
//! degrees, units, the part's variables); `t` is the curve's variable and hides a part variable of the
//! same name. The ends of `t` are numbers or formulas of the part's variables; `tol` (mm, default 0.01)
//! is the largest distance allowed between the formula's curve and the spline.
//!
//! Every regen evaluates the formulas again and replaces the entity's points, so the document stays
//! parametric and only the formula text needs to be written. The points are ordinary fixed sketch
//! points named `<id>_start`, `<id>_p1`.. and `<id>_end` (an open curve), so a line can end on
//! `<id>_end`; points of those names saved in the document are rebuilt, not trusted. A curve that ends
//! where it starts (within `tol`) becomes a closed (periodic) spline.
//!
//! The fit is the kernel's own interpolating spline (`spline::InterpSpline`, what the profile builds),
//! checked span by span: four formula points against the spline (two Gauss points, the middle and one
//! that roams from span to span, so a periodic formula sampled at whole periods is not taken for a
//! straight line) and the spline's mid-span point against the formula. Two placements are tried and the one with fewer points kept: the fewest evenly
//! spaced `t` that meet the tolerance (every count tried upward: the error is not monotone in the count,
//! so solidcraft #9411's search back down can miss the fewest), and splitting only the spans that miss
//! it (fewer points where the curve is flat).
//! Refusals carry a code and a plain message (`EqError`); a curve that crosses itself warns.

use super::spline::InterpSpline;
use super::{jstr, obj_set, Sketch, SketchEntity, SketchPoint};
use crate::expr::{self, Vars};
use ck_json::{Json, Obj};

pub const DEFAULT_TOL: f64 = 0.01;
pub const MIN_TOL: f64 = 1e-5;
pub const MAX_TOL: f64 = 10.0;
/// Most points one curve may take (the spline's moments are a dense solve).
pub const MAX_POINTS: usize = 200;
pub const MAX_FORMULA: usize = 1024;
/// Evenly spaced `t` evaluated before fitting, to tell a formula that cannot be read from one undefined somewhere.
const PROBE: usize = 64;

/// The `equation` object of a spline entity, as written (checked by `fit`).
#[derive(Debug, Clone, Default)]
pub struct Spec {
    pub x: Option<String>,
    pub y: Option<String>,
    pub t: Option<Vec<Json>>,
    pub tol: Option<Json>,
}

impl Spec {
    pub fn from_json(j: &Json) -> Spec {
        let s = |k: &str| match j.get(k) {
            Some(Json::Str(s)) => Some(s.clone()),
            Some(Json::Num(n)) => Some(ck_json::fmt_f64(*n)),
            _ => None,
        };
        Spec { x: s("x"), y: s("y"), t: j.get("t").and_then(|v| v.as_arr()).map(|a| a.to_vec()), tol: j.get("tol").cloned() }
    }
}

/// A non-fatal note about a fitted curve.
#[derive(Debug, Clone)]
pub struct Warning {
    pub code: &'static str,
    pub detail: String,
}

/// The fitted spline: its points (in order, a closed curve without the repeated end), the `t` of each
/// span end (a closed curve has one more, the end of `t`), the largest distance measured to the formula.
#[derive(Debug, Clone)]
pub struct Fit {
    pub pts: Vec<[f64; 2]>,
    pub ts: Vec<f64>,
    pub closed: bool,
    pub deviation: f64,
    pub tol: f64,
    pub warnings: Vec<Warning>,
}

/// Why an equation curve was refused.
#[derive(Debug, Clone, PartialEq)]
pub enum EqError {
    /// `equationFormula`: a formula that is missing, too long or cannot be read.
    Formula { axis: char, detail: String },
    /// `equationNotFinite`: the formula has no finite value at some `t` of the range.
    NotFinite { axis: char, t: f64 },
    /// `equationRange`: the range of `t` is missing, unreadable or empty.
    Range(String),
    /// `equationTolerance`: a tolerance out of bounds, or one the curve cannot meet in `MAX_POINTS`.
    Tolerance(String),
    /// `equationDegenerate`: the formulas give one point for every `t`.
    Degenerate,
}

impl EqError {
    pub fn code(&self) -> &'static str {
        match self {
            EqError::Formula { .. } => "equationFormula",
            EqError::NotFinite { .. } => "equationNotFinite",
            EqError::Range(_) => "equationRange",
            EqError::Tolerance(_) => "equationTolerance",
            EqError::Degenerate => "equationDegenerate",
        }
    }
    pub fn message(&self) -> String {
        match self {
            EqError::Formula { axis, detail } => format!("{axis}(t): {detail}"),
            EqError::NotFinite { axis, t } => format!(
                "{axis}(t) has no finite value at t = {}: keep the range of t where the formula is defined (no division by zero, no square root or logarithm of a negative number)",
                num(*t)
            ),
            EqError::Range(s) | EqError::Tolerance(s) => s.clone(),
            EqError::Degenerate => "x(t) and y(t) give the same point for every t of the range, so there is no curve to draw".into(),
        }
    }
}

fn num(v: f64) -> String {
    ck_json::fmt_f64((v * 1e6).round() / 1e6)
}

/// The two formulas with the part's variables and `t` bound (always the first variable).
struct Curve<'a> {
    x: &'a str,
    y: &'a str,
    vars: Vars,
}

impl<'a> Curve<'a> {
    fn new(x: &'a str, y: &'a str, vars: &Vars) -> Curve<'a> {
        let mut v = vars.clone();
        v.values.retain(|(n, _)| n != "t");
        v.values.insert(0, ("t".to_string(), 0.0));
        Curve { x, y, vars: v }
    }
    fn src(&self, axis: char) -> &'a str {
        if axis == 'x' {
            self.x
        } else {
            self.y
        }
    }
    fn axis(&mut self, axis: char, t: f64) -> Result<f64, ()> {
        self.vars.values[0].1 = t;
        expr::eval(self.src(axis), Some(&self.vars))
    }
    fn at(&mut self, t: f64) -> Result<[f64; 2], EqError> {
        let x = self.axis('x', t).map_err(|_| EqError::NotFinite { axis: 'x', t })?;
        let y = self.axis('y', t).map_err(|_| EqError::NotFinite { axis: 'y', t })?;
        Ok([x, y])
    }
}

/// A formula that reads at no probe `t` is unreadable (or names an unknown variable); one that fails at some is undefined there.
fn check_formula(c: &mut Curve, axis: char, probe: &[f64]) -> Result<(), EqError> {
    let src = c.src(axis);
    let bad = |detail: &str| EqError::Formula { axis, detail: detail.to_string() };
    if src.trim().trim_start_matches('=').trim().is_empty() {
        return Err(bad("the formula is empty: write it in terms of t, e.g. 20*cos(t)"));
    }
    if src.len() > MAX_FORMULA {
        return Err(bad("the formula is longer than 1024 characters"));
    }
    let mut first_bad = None;
    let mut any_ok = false;
    for &t in probe {
        match c.axis(axis, t) {
            Ok(_) => any_ok = true,
            Err(()) => {
                first_bad.get_or_insert(t);
            }
        }
    }
    if !any_ok {
        let mut unknown: Vec<String> = Vec::new();
        for id in expr::identifiers(src) {
            if id != "t" && !c.vars.has(&id) && !expr::is_builtin_word(&id) && !unknown.contains(&id) {
                unknown.push(id);
            }
        }
        if !unknown.is_empty() {
            let mut v = c.vars.clone();
            for u in &unknown {
                v.set(u, 1.0);
            }
            let reads = probe.iter().any(|&t| {
                v.values[0].1 = t;
                expr::eval(src, Some(&v)).is_ok()
            });
            if reads {
                let names = unknown.iter().map(|u| format!("'{u}'")).collect::<Vec<_>>().join(", ");
                return Err(bad(&format!("uses the unknown name {names}: the curve's variable is t, any other name must be a part variable")));
            }
        }
        return Err(bad("could not be read: check the brackets, the operators and the function names (trigonometry is in degrees: sin(t), cos(t))"));
    }
    match first_bad {
        Some(t) => Err(EqError::NotFinite { axis, t }),
        None => Ok(()),
    }
}

fn range_end(j: &Json, which: &str, vars: &Vars) -> Result<f64, EqError> {
    match j {
        Json::Num(n) if n.is_finite() => Ok(*n),
        Json::Str(s) => expr::eval(s, Some(vars)).map_err(|_| EqError::Range(format!("the {which} of t could not be read: {s:?}"))),
        _ => Err(EqError::Range(format!("the {which} of t must be a number or a formula"))),
    }
}

/// Everything checked before fitting.
struct Prep<'a> {
    curve: Curve<'a>,
    t0: f64,
    t1: f64,
    tol: f64,
    closed: bool,
}

fn prepare<'a>(spec: &'a Spec, vars: &Vars) -> Result<Prep<'a>, EqError> {
    let x = spec.x.as_deref().ok_or(EqError::Formula { axis: 'x', detail: "the formula is missing: give x and y as formulas of t".into() })?;
    let y = spec.y.as_deref().ok_or(EqError::Formula { axis: 'y', detail: "the formula is missing: give x and y as formulas of t".into() })?;
    let tr = spec
        .t
        .as_ref()
        .filter(|a| a.len() == 2)
        .ok_or_else(|| EqError::Range("t needs a start and an end, e.g. t: [0, 360] (angles are in degrees)".into()))?;
    let (t0, t1) = (range_end(&tr[0], "start", vars)?, range_end(&tr[1], "end", vars)?);
    if (t1 - t0).abs() <= 1e-12 * t0.abs().max(1.0) {
        return Err(EqError::Range(format!("t runs from {} to {}: the range is empty", num(t0), num(t1))));
    }
    let tol_bad = |v: String| EqError::Tolerance(format!("the tolerance must be between 0.00001 and 10 mm, got {v}"));
    let tol = match &spec.tol {
        None | Some(Json::Null) => DEFAULT_TOL,
        Some(Json::Num(n)) => *n,
        Some(Json::Str(s)) => expr::eval(s, Some(vars)).map_err(|_| tol_bad(format!("{s:?}")))?,
        Some(j) => return Err(tol_bad(ck_json::to_string(j))),
    };
    if !(tol >= MIN_TOL && tol <= MAX_TOL) {
        return Err(tol_bad(num(tol)));
    }
    let mut curve = Curve::new(x, y, vars);
    let probe: Vec<f64> = (0..=PROBE).map(|i| t0 + (t1 - t0) * i as f64 / PROBE as f64).collect();
    check_formula(&mut curve, 'x', &probe)?;
    check_formula(&mut curve, 'y', &probe)?;
    let pts = probe.iter().map(|&t| curve.at(t)).collect::<Result<Vec<_>, _>>()?;
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in &pts {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    if (hi[0] - lo[0]).hypot(hi[1] - lo[1]) <= 1e-9 {
        return Err(EqError::Degenerate);
    }
    let (a, b) = (pts[0], pts[pts.len() - 1]);
    let closed = (a[0] - b[0]).hypot(a[1] - b[1]) <= tol;
    Ok(Prep { curve, t0, t1, tol, closed })
}

/// One candidate: the samples, the `t` of every span end and the spline through the samples.
struct Trial {
    ts: Vec<f64>,
    pts: Vec<[f64; 2]>,
    sp: InterpSpline,
}

/// `ts` holds the span ends (a closed curve's last one is the end of `t`, not sampled again). Samples
/// closer than the spline's own 1e-9 merge (their spans join). `None` when too few distinct points remain.
fn build(p: &mut Prep, ts: &[f64]) -> Result<Option<Trial>, EqError> {
    let n = if p.closed { ts.len() - 1 } else { ts.len() };
    let (mut kt, mut kp): (Vec<f64>, Vec<[f64; 2]>) = (vec![], vec![]);
    for &t in &ts[..n] {
        let q = p.curve.at(t)?;
        if kp.last().map_or(true, |l: &[f64; 2]| (q[0] - l[0]).hypot(q[1] - l[1]) > 1e-9) {
            kt.push(t);
            kp.push(q);
        }
    }
    if p.closed {
        while kp.len() > 1 && {
            let (f, l) = (kp[0], kp[kp.len() - 1]);
            (f[0] - l[0]).hypot(f[1] - l[1]) <= 1e-9
        } {
            kp.pop();
            kt.pop();
        }
        kt.push(ts[ts.len() - 1]);
    } else if kt.last() != ts.last() {
        // the end sample merged into the one before: that span now reaches the end of t
        *kt.last_mut().unwrap() = ts[ts.len() - 1];
        if kt.len() == 1 {
            return Ok(None);
        }
    }
    if kp.len() < if p.closed { 3 } else { 2 } {
        return Ok(None);
    }
    let Some(sp) = InterpSpline::new(&kp, p.closed) else { return Ok(None) };
    if sp.p.len() != kp.len() {
        return Ok(None);
    }
    Ok(Some(Trial { ts: kt, pts: kp, sp }))
}

/// Smallest distance from `q` to `f` over `[lo, hi]`: 12 samples, then a golden-section search round the best.
fn min_dist(mut f: impl FnMut(f64) -> Result<[f64; 2], EqError>, q: [f64; 2], lo: f64, hi: f64) -> Result<f64, EqError> {
    let mut d = |s: f64| -> Result<f64, EqError> {
        let p = f(s)?;
        Ok((p[0] - q[0]).hypot(p[1] - q[1]))
    };
    let n = 12;
    let step = (hi - lo) / n as f64;
    let (mut best, mut bi) = (f64::INFINITY, 0);
    for i in 0..=n {
        let v = d(lo + step * i as f64)?;
        if v < best {
            best = v;
            bi = i;
        }
    }
    let (mut a, mut b) = (lo + step * (bi.max(1) - 1) as f64, lo + step * (bi + 1).min(n) as f64);
    let g = 0.5 * (5f64.sqrt() - 1.0);
    let (mut c, mut e) = (b - g * (b - a), a + g * (b - a));
    let (mut fc, mut fe) = (d(c)?, d(e)?);
    for _ in 0..30 {
        if fc < fe {
            b = e;
            e = c;
            fe = fc;
            c = b - g * (b - a);
            fc = d(c)?;
        } else {
            a = c;
            c = e;
            fc = fe;
            e = a + g * (b - a);
            fe = d(e)?;
        }
    }
    Ok(best.min(fc).min(fe))
}

/// Distance between the spline and the formula on every span; the list ends early at the first span over `stop`.
fn span_devs(p: &mut Prep, tr: &Trial, stop: f64) -> Result<Vec<f64>, EqError> {
    let len = tr.sp.length();
    let (tmin, tmax) = (p.t0.min(p.t1), p.t0.max(p.t1));
    let mut out = Vec::with_capacity(tr.ts.len() - 1);
    for i in 0..tr.ts.len() - 1 {
        let (ta, tb) = (tr.ts[i], tr.ts[i + 1]);
        let (sa, sb) = (tr.sp.knots[i], tr.sp.knots[i + 1]);
        let hs = sb - sa;
        let (mut lo, mut hi) = (sa - 0.5 * hs, sb + 0.5 * hs);
        if !p.closed {
            lo = lo.max(0.0);
            hi = hi.min(len);
        }
        let mut d = 0.0f64;
        // Gauss points, the middle and a point that moves along the golden ratio from span to span, so samples of a
        // periodic formula taken at a whole number of periods (sin(40*t) at every 9 degrees) do not all look straight
        let roam = 0.1 + 0.8 * ((i as f64 * 0.618_033_988_749_895).fract());
        for f in [0.211_324_865_405_187, 0.5, 0.788_675_134_594_813, roam] {
            let q = p.curve.at(ta + f * (tb - ta))?;
            d = d.max(min_dist(|s| Ok(tr.sp.eval(s)), q, lo, hi)?);
        }
        let q = tr.sp.eval(sa + 0.5 * hs);
        let dt = tb - ta;
        let (l, h) = ((ta - 0.5 * dt).clamp(tmin, tmax), (tb + 0.5 * dt).clamp(tmin, tmax));
        d = d.max(min_dist(|t| p.curve.at(t), q, l, h)?);
        out.push(d);
        if d > stop {
            break;
        }
    }
    Ok(out)
}

fn even_ts(p: &Prep, n: usize) -> Vec<f64> {
    let k = if p.closed { n } else { n - 1 };
    (0..=k).map(|i| p.t0 + (p.t1 - p.t0) * i as f64 / k as f64).collect()
}

/// The trial of `n` evenly spaced samples and its deviation (infinite when the samples collapse); past `stop`
/// the deviation is only known to exceed it.
fn even_trial(p: &mut Prep, n: usize, stop: f64) -> Result<(Option<Trial>, f64), EqError> {
    if n < if p.closed { 3 } else { 2 } {
        return Ok((None, f64::INFINITY));
    }
    let ts = even_ts(p, n);
    match build(p, &ts)? {
        Some(tr) => {
            let d = span_devs(p, &tr, stop)?.into_iter().fold(0.0, f64::max);
            Ok((Some(tr), d))
        }
        None => Ok((None, f64::INFINITY)),
    }
}

/// Deviation of `n` evenly spaced samples (for tests: is the fit's count the fewest even one?).
pub fn even_deviation(spec: &Spec, vars: &Vars, n: usize) -> Result<f64, EqError> {
    let mut p = prepare(spec, vars)?;
    Ok(even_trial(&mut p, n, f64::INFINITY)?.1)
}

/// The fewest evenly spaced samples within the tolerance. The error is not monotone in the count (the
/// samples fall in and out of step with a periodic formula), so every count is tried upward; a check
/// stops at the first span that misses, which keeps the failing counts cheap.
fn fit_even(p: &mut Prep) -> Result<Option<(Trial, f64)>, EqError> {
    let tol = p.tol;
    for n in (if p.closed { 3 } else { 2 })..=MAX_POINTS {
        if let (Some(tr), d) = even_trial(p, n, tol)? {
            if d <= tol {
                return Ok(Some((tr, d)));
            }
        }
    }
    Ok(None)
}

/// Split only the spans that miss the tolerance, round after round.
fn fit_split(p: &mut Prep) -> Result<Option<(Trial, f64)>, EqError> {
    let mut ts = even_ts(p, if p.closed { 4 } else { 3 });
    for _ in 0..40 {
        let tr = build(p, &ts)?;
        let devs = match &tr {
            Some(tr) => span_devs(p, tr, f64::INFINITY)?,
            None => vec![f64::INFINITY; ts.len() - 1],
        };
        if let Some(tr) = tr.filter(|_| devs.iter().all(|d| *d <= p.tol)) {
            let d = devs.into_iter().fold(0.0, f64::max);
            return Ok(Some((tr, d)));
        }
        let mut next = vec![ts[0]];
        for i in 0..ts.len() - 1 {
            if devs.get(i).map_or(true, |d| *d > p.tol) {
                next.push(0.5 * (ts[i] + ts[i + 1]));
            }
            next.push(ts[i + 1]);
        }
        ts = next;
        if ts.len() - usize::from(p.closed) > MAX_POINTS {
            return Ok(None);
        }
    }
    Ok(None)
}

/// A proper crossing of two segments that share no end of the polyline (touching does not count).
fn crossing(poly: &[[f64; 2]], closed: bool) -> Option<[f64; 2]> {
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let m = poly.len() - 1;
    for i in 0..m {
        let (a, b) = (poly[i], poly[i + 1]);
        for j in i + 2..m {
            if closed && i == 0 && j == m - 1 {
                continue;
            }
            let (c, d) = (poly[j], poly[j + 1]);
            if cross(a, b, c) * cross(a, b, d) < 0.0 && cross(c, d, a) * cross(c, d, b) < 0.0 {
                return Some([0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])]);
            }
        }
    }
    None
}

/// Fit the spline with the fewest points that stays within the tolerance of the formulas.
pub fn fit(spec: &Spec, vars: &Vars) -> Result<Fit, EqError> {
    let mut p = prepare(spec, vars)?;
    let even = fit_even(&mut p)?;
    let split = fit_split(&mut p)?;
    let chosen = match (even, split) {
        (Some(e), Some(s)) => Some(if s.0.pts.len() < e.0.pts.len() { s } else { e }),
        (e, s) => e.or(s),
    };
    let Some((tr, deviation)) = chosen else {
        let d = even_trial(&mut p, MAX_POINTS, f64::INFINITY)?.1;
        let best = if d.is_finite() { format!(" (with {MAX_POINTS} it is {} mm away)", num(d)) } else { String::new() };
        return Err(EqError::Tolerance(format!(
            "the curve needs more than {MAX_POINTS} points to stay within {} mm of the formulas{best}; loosen the tolerance or split the range of t into parts",
            num(p.tol)
        )));
    };
    let mut warnings = vec![];
    let spans = tr.ts.len() - 1;
    let k = (800 / spans).clamp(1, 4);
    let len = tr.sp.length();
    let poly: Vec<[f64; 2]> = (0..=spans * k).map(|i| tr.sp.eval(len * i as f64 / (spans * k) as f64)).collect();
    if let Some(at) = crossing(&poly, p.closed) {
        warnings.push(Warning {
            code: "equationSelfIntersects",
            detail: format!(
                "the curve crosses itself near ({}, {}); an outline that crosses itself does not make one region to extrude",
                num(at[0]),
                num(at[1])
            ),
        });
    }
    Ok(Fit { pts: tr.pts, ts: tr.ts, closed: p.closed, deviation, tol: p.tol, warnings })
}

/// Point ids of an equation curve of `n` points: `<id>_start`, `<id>_p1`.., and `<id>_end` last when open.
pub fn point_ids(eid: &str, n: usize, closed: bool) -> Vec<String> {
    (0..n)
        .map(|i| match i {
            0 => format!("{eid}_start"),
            _ if !closed && i == n - 1 => format!("{eid}_end"),
            _ => format!("{eid}_p{i}"),
        })
        .collect()
}

fn is_generated(eid: &str, id: &str) -> bool {
    let Some(rest) = id.strip_prefix(eid).and_then(|r| r.strip_prefix('_')) else { return false };
    rest == "start" || rest == "end" || rest.strip_prefix('p').map_or(false, |d| !d.is_empty() && d.bytes().all(|b| b.is_ascii_digit()))
}

/// What `apply` did: whether any generated point moved (the sketch then needs a solve) and the warnings per entity.
#[derive(Debug, Default)]
pub struct Applied {
    pub changed: bool,
    pub warnings: Vec<(String, Warning)>,
}

/// Rebuild the points of every equation curve of the sketch from its formulas (the typed sketch and its echoed DOM).
pub fn apply(sk: &mut Sketch, vars: &Vars) -> Result<Applied, (String, EqError)> {
    let mut out = Applied::default();
    let Some(raw_ents) = sk.raw.get("entities").and_then(|v| v.as_arr()).map(|a| a.to_vec()) else { return Ok(out) };
    let specs: Vec<(usize, String, Spec)> = raw_ents
        .iter()
        .enumerate()
        .filter(|(_, e)| e.get("type").and_then(|t| t.as_str()) == Some("spline"))
        .filter_map(|(i, e)| match e.get("equation") {
            Some(q @ Json::Obj(_)) => Some((i, jstr(e, "id"), Spec::from_json(q))),
            _ => None,
        })
        .collect();
    if specs.is_empty() {
        return Ok(out);
    }
    let mut raw_ents = raw_ents;
    let mut raw_pts: Vec<Json> = sk.raw.get("points").and_then(|v| v.as_arr()).map(|a| a.to_vec()).unwrap_or_default();
    raw_pts.resize(sk.points.len().max(raw_pts.len()), Json::Obj(vec![]));
    for (ri, eid, spec) in specs {
        let f = fit(&spec, vars).map_err(|e| (eid.clone(), e))?;
        let ids = point_ids(&eid, f.pts.len(), f.closed);
        let old: Vec<(String, f64, f64)> = sk.points.iter().filter(|p| is_generated(&eid, &p.id)).map(|p| (p.id.clone(), p.x, p.y)).collect();
        let (mut pts, mut raws) = (Vec::with_capacity(sk.points.len()), Vec::with_capacity(sk.points.len()));
        for (i, p) in sk.points.drain(..).enumerate() {
            if !is_generated(&eid, &p.id) {
                raws.push(raw_pts.get(i).cloned().unwrap_or(Json::Obj(vec![])));
                pts.push(p);
            }
        }
        for (id, q) in ids.iter().zip(&f.pts) {
            pts.push(SketchPoint { id: id.clone(), x: q[0], y: q[1], fixed: true });
            raws.push(Json::Obj(vec![
                ("id".into(), Json::Str(id.clone())),
                ("x".into(), Json::Num(q[0])),
                ("y".into(), Json::Num(q[1])),
                ("fixed".into(), Json::Bool(true)),
            ]));
        }
        let same = old.len() == ids.len()
            && old.iter().zip(ids.iter().zip(&f.pts)).all(|((oid, x, y), (id, q))| oid == id && (x - q[0]).abs() <= 1e-12 && (y - q[1]).abs() <= 1e-12);
        out.changed |= !same;
        sk.points = pts;
        raw_pts = raws;
        for e in sk.entities.iter_mut() {
            if let SketchEntity::Spline { id, pts, closed, .. } = e {
                if *id == eid {
                    *pts = ids.clone();
                    *closed = f.closed;
                }
            }
        }
        obj_set(&mut raw_ents[ri], "pts", Json::Arr(ids.iter().map(|s| Json::Str(s.clone())).collect()));
        obj_set(&mut raw_ents[ri], "closed", Json::Bool(f.closed));
        out.warnings.extend(f.warnings.into_iter().map(|w| (eid.clone(), w)));
    }
    obj_set(&mut sk.raw, "points", Json::Arr(raw_pts));
    obj_set(&mut sk.raw, "entities", Json::Arr(raw_ents));
    Ok(out)
}

/// `{type:"equationCurve", x, y, t:[start, end], tol?, variables?:[{name, expr}]}`: the fitted points
/// without a part (the sketch editor's preview). Stateless. Error: `"<code>: <message>"`.
pub fn reply(msg: &Json) -> Result<Json, String> {
    let spec = Spec::from_json(msg);
    let vars = expr::resolve_variables(&expr::parse_variables(msg.get("variables"))).values;
    let f = fit(&spec, &vars).map_err(|e| format!("{}: {}", e.code(), e.message()))?;
    let pt = |q: &[f64; 2]| Json::Arr(vec![Json::Num(q[0]), Json::Num(q[1])]);
    Ok(Obj::new()
        .put("pts", Json::Arr(f.pts.iter().map(pt).collect()))
        .put("closed", f.closed)
        .put("count", f.pts.len() as f64)
        .put("deviation", f.deviation)
        .put("tol", f.tol)
        .put(
            "warnings",
            Json::Arr(f.warnings.iter().map(|w| Obj::new().put("code", w.code).put("detail", w.detail.clone()).build()).collect()),
        )
        .build())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn generated_ids_are_recognised_exactly() {
        assert_eq!(point_ids("e1", 3, false), vec!["e1_start", "e1_p1", "e1_end"]);
        assert_eq!(point_ids("e1", 3, true), vec!["e1_start", "e1_p1", "e1_p2"]);
        for id in ["e1_start", "e1_end", "e1_p1", "e1_p20"] {
            assert!(is_generated("e1", id), "{id}");
        }
        for id in ["e1", "e1_p", "e1_px", "e10_p1", "e1_start2", "p1", "e1_q1"] {
            assert!(!is_generated("e1", id), "{id}");
        }
    }

    #[test]
    fn builtin_words_are_not_unknown_names() {
        for w in ["pi", "PI", "sin", "Cos", "max", "atan2", "mm", "deg"] {
            assert!(expr::is_builtin_word(w), "{w}");
        }
        for w in ["q", "Max", "r"] {
            assert!(!expr::is_builtin_word(w), "{w}");
        }
    }
}
