//! Thin-wall option of `extrude` (add / cut) and `revolve`: `"thin": {thickness, side, ends,
//! cornerStyle, thickness2?}`.
//!
//! The sketch profile curves (open chains OR closed loops) are offset in the sketch plane and
//! the region between the curve and its offset (or between two offsets) is swept in place of
//! the sketch regions, with the feature's usual end condition, draft and direction rules.
//!
//! * `side`: `inside` = the left normal of the curve oriented so that it turns counter-clockwise
//!   (closed loops: the interior; an open bend: the inside of the bend; a straight chain: left of
//!   the sketch direction), `outside` = the other side, `both` = `thickness` to the left and
//!   `thickness2` (default `thickness`) to the right, `mid` = `thickness` centred on the curve.
//! * `ends` (open chains): `flat` | `round` end caps. Closed loops give a hollow wall (outer and
//!   inner loop, no end caps).
//! * `cornerStyle`: `mitre` (outer corners are extended to their intersection, very sharp ones
//!   fall back to a bevel) | `round` (an arc about the corner point). Inner corners are trimmed.
//! * Wall faces `S.<entity>.W0` (the offset side; with `both` the left offset) and
//!   `S.<entity>.W1` (the original curve; with `both` the right offset); corner pieces
//!   `<entity>.W0r` (round) / `.W0m` / `.W0b` (mitre connector / bevel); open end caps
//!   `<first entity>.E0` / `<last entity>.E1`.
//! * A thickness the profile cannot take (past a radius of curvature, a short segment eaten by an
//!   inner corner) is `thinTooThick {limit, minRadius, maxThickness, thickness, side}`.

use super::extrude::{profile_seg, sketch_axis, REGION_NONE, REGION_UNKNOWN};
use super::FeatureKind;
use crate::regen::{codes, FeatureError, RegenState, SketchResult};
use crate::sketch::profile::{open_chains, Seg};
use crate::Feature;
use ck_geom::curve2::Curve2;
use ck_geom::fit::{interpolate, Param};
use ck_geom::{Vec2, Vec3};
use ck_json::Json;
use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};

pub const THIN_TOO_THICK: &str = "thinTooThick";
/// Mitre length (from the corner point) in thicknesses beyond which a corner is bevelled.
const MITRE_LIMIT: f64 = 4.0;
/// Share of a radius of curvature the offset may reach.
const RADIUS_MARGIN: f64 = 0.02;

#[derive(Debug, Clone, Copy)]
pub struct Thin {
    /// Wall thickness on the left / right of the (oriented) curve.
    pub tl: f64,
    pub tr: f64,
    /// The `thickness` field as given (reported back in errors).
    pub thickness: f64,
    pub round_ends: bool,
    pub round_corners: bool,
}

/// The parsed `thin` option (`None`: absent, `null`, `false` or `enabled:false`).
pub fn parse(state: &RegenState, f: &Feature) -> Result<Option<Thin>, FeatureError> {
    let Some(t) = f.raw.get("thin") else { return Ok(None) };
    if matches!(t, Json::Null | Json::Bool(false)) || matches!(t.get("enabled"), Some(Json::Bool(false))) {
        return Ok(None);
    }
    let bad = |field: &str| FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(format!("thin.{field}")));
    let num = |k: &str, d: f64| -> Result<f64, FeatureError> { crate::expr::eval_field(t.get(k), &state.vars, d).map_err(FeatureError::new) };
    let thickness = num("thickness", 1.0)?;
    if !(thickness > 0.0) || !thickness.is_finite() {
        return Err(bad("thickness"));
    }
    let t2 = num("thickness2", thickness)?;
    if !(t2 >= 0.0) || !t2.is_finite() {
        return Err(bad("thickness2"));
    }
    let side = t.get("side").and_then(|s| s.as_str()).unwrap_or("inside");
    let (tl, tr) = match side {
        "inside" => (thickness, 0.0),
        "outside" => (0.0, thickness),
        "both" => (thickness, t2),
        "mid" => (thickness / 2.0, thickness / 2.0),
        _ => return Err(bad("side")),
    };
    let round_ends = match t.get("ends").and_then(|s| s.as_str()).unwrap_or("flat") {
        "flat" => false,
        "round" => true,
        _ => return Err(bad("ends")),
    };
    let round_corners = match t.get("cornerStyle").and_then(|s| s.as_str()).unwrap_or("mitre") {
        "mitre" | "miter" => false,
        "round" => true,
        _ => return Err(bad("cornerStyle")),
    };
    Ok(Some(Thin { tl, tr, thickness, round_ends, round_corners }))
}

/// A profile chain in travel order.
#[derive(Clone)]
struct Chain {
    segs: Vec<ProfileSeg>,
    closed: bool,
}

enum Fail {
    /// The offset reaches a centre of curvature / eats a segment / crosses itself.
    Thick,
    Err(FeatureError),
}
impl From<FeatureError> for Fail {
    fn from(e: FeatureError) -> Fail {
        Fail::Err(e)
    }
}

fn floor_hundredth(x: f64) -> f64 {
    (x * 100.0).floor() / 100.0
}

// ---------------------------------------------------------------- chains of the sketch

fn loop_key(l: &[Seg]) -> Vec<String> {
    let mut k: Vec<String> = l.iter().map(|s| s.tag.clone()).collect();
    k.sort_unstable();
    k
}

fn reverse_segs(v: &[ProfileSeg]) -> Vec<ProfileSeg> {
    v.iter().rev().map(|s| s.reversed()).collect()
}

/// Net turning of a chain (radians, counter-clockwise positive), corners included.
fn turning(segs: &[ProfileSeg]) -> f64 {
    let mut total = 0.0;
    let mut prev: Option<Vec2> = None;
    for s in segs {
        let n = 24;
        for k in 0..=n {
            let t = s.t_start() + (s.t_end() - s.t_start()) * k as f64 / n as f64;
            let tg = s.travel_tangent(t);
            if let Some(p) = prev {
                total += p.cross(tg).atan2(p.dot(tg));
            }
            prev = Some(tg);
        }
    }
    total
}

/// The candidate chains of a sketch: the closed region loops (outer and hole loops, once each),
/// then the open chains made of entities that bound no region.
fn candidates(sk: &SketchResult) -> Result<Vec<(Chain, Vec<String>)>, FeatureError> {
    let mut out: Vec<(Chain, Vec<String>)> = vec![];
    let mut seen: Vec<Vec<String>> = vec![];
    let mut region_tags: Vec<String> = vec![];
    let mut add_loop = |l: &Vec<Seg>, out: &mut Vec<(Chain, Vec<String>)>| -> Result<(), FeatureError> {
        let key = loop_key(l);
        if seen.contains(&key) {
            return Ok(());
        }
        seen.push(key.clone());
        let segs: Vec<ProfileSeg> = l.iter().map(profile_seg).collect::<Result<_, _>>()?;
        out.push((Chain { segs, closed: true }, key));
        Ok(())
    };
    for r in &sk.profiles.regions {
        add_loop(&r.outer, &mut out)?;
        for h in &r.holes {
            add_loop(h, &mut out)?;
        }
        for l in std::iter::once(&r.outer).chain(r.holes.iter()) {
            region_tags.extend(l.iter().map(|s| s.tag.clone()));
        }
    }
    for ch in open_chains(&sk.sketch) {
        if ch.iter().any(|s| region_tags.contains(&s.tag)) {
            continue;
        }
        let segs: Vec<ProfileSeg> = ch.iter().map(profile_seg).collect::<Result<_, _>>()?;
        let closed = segs[0].start().dist(segs[segs.len() - 1].end()) < 1e-6;
        out.push((Chain { segs, closed }, loop_key(&ch)));
    }
    Ok(out)
}

fn selected_chains(sk: &SketchResult, f: &Feature) -> Result<Vec<Chain>, FeatureError> {
    let all = candidates(sk)?;
    let ids: Option<Vec<String>> = match f.raw.get("regions") {
        None => None,
        Some(sel) => Some(
            sel.as_arr()
                .and_then(|a| a.iter().map(|v| v.as_str().map(str::to_string)).collect::<Option<_>>())
                .ok_or_else(|| FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("regions".into())))?,
        ),
    };
    let picked: Vec<Chain> = match &ids {
        None => all.into_iter().map(|c| c.0).collect(),
        Some(ids) => {
            let unknown: Vec<Json> = ids.iter().filter(|i| sk.sketch.entity(i).is_none()).map(|i| Json::Str(i.clone())).collect();
            if !unknown.is_empty() {
                return Err(FeatureError::new(REGION_UNKNOWN).with("ids", Json::Arr(unknown)));
            }
            let picked: Vec<Chain> = all.into_iter().filter(|(_, tags)| tags.iter().any(|t| ids.contains(t))).map(|c| c.0).collect();
            if picked.is_empty() {
                return Err(FeatureError::new(REGION_NONE).with("regions", Json::Arr(ids.iter().cloned().map(Json::Str).collect())));
            }
            picked
        }
    };
    if picked.is_empty() {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    Ok(picked)
}

/// Orient: closed loops counter-clockwise (left = interior), open chains turning
/// counter-clockwise (left = inside of the bend).
fn oriented(mut c: Chain) -> Chain {
    let flip = if c.closed { ProfileLoop::new(c.segs.clone()).signed_area() < 0.0 } else { turning(&c.segs) < -1e-9 };
    if flip {
        c.segs = reverse_segs(&c.segs);
    }
    c
}

// ---------------------------------------------------------------- offsetting

fn retag(s: &ProfileSeg, suffix: &str) -> ProfileSeg {
    ProfileSeg { tag: format!("{}.{}", s.tag, suffix), ..s.clone() }
}

/// Curvature of `c` at `t` (positive = turns left of the entity direction).
fn curvature(c: &Curve2, t: f64) -> f64 {
    let d = c.derivs(t, 2);
    let s = d[1].len();
    if s < 1e-300 {
        return 0.0;
    }
    d[1].cross(d[2]) / (s * s * s)
}

/// Offset one segment by `d` (> 0: left of its travel direction).
fn offset_seg(s: &ProfileSeg, d: f64, w: &str) -> Result<ProfileSeg, Fail> {
    // signed offset along the left normal of the ENTITY direction
    let e = if s.rev { -d } else { d };
    let tag = format!("{}.{}", s.tag, w);
    let curve = match &s.curve {
        Curve2::Line { o, d: dir } => {
            let n = dir.normalize().perp();
            Curve2::Line { o: *o + n * e, d: *dir }
        }
        Curve2::Circle { c, x, r, ccw } => {
            // the left of a counter-clockwise circle is its centre
            let r2 = if *ccw { r - e } else { r + e };
            if r2 <= *r * RADIUS_MARGIN || r2 <= 1e-9 {
                return Err(Fail::Thick);
            }
            Curve2::Circle { c: *c, x: *x, r: r2, ccw: *ccw }
        }
        other => return fit_offset(other, s, e, tag),
    };
    Ok(ProfileSeg { tag, curve, t0: s.t0, t1: s.t1, rev: s.rev })
}

/// Offset of an ellipse / B-spline piece: a cubic B-spline interpolating samples of the exact
/// offset, refined until it follows it within 1e-5.
fn fit_offset(curve: &Curve2, s: &ProfileSeg, e: f64, tag: String) -> Result<ProfileSeg, Fail> {
    let (t0, t1) = (s.t0, s.t1);
    let base = match curve {
        Curve2::BSpline(b) => (b.ctrl.len() * 16).max(48),
        _ => ((96.0 * (t1 - t0).abs() / std::f64::consts::TAU).ceil() as usize).max(24),
    };
    let off_pt = |t: f64| -> Vec2 {
        let d = curve.derivs(t, 1);
        d[0] + d[1].normalize().perp() * e
    };
    let mut n = base;
    for round in 0..4 {
        let ts: Vec<f64> = (0..=n).map(|i| t0 + (t1 - t0) * i as f64 / n as f64).collect();
        for &t in &ts {
            if 1.0 - e * curvature(curve, t) < RADIUS_MARGIN * 5.0 {
                return Err(Fail::Thick);
            }
        }
        let pts: Vec<Vec2> = ts.iter().map(|&t| off_pt(t)).collect();
        let fit = interpolate(&pts, 3, Param::ChordLength).map_err(|m| Fail::Err(FeatureError::new("offsetFitFailed").with("detail", Json::Str(m))))?;
        let dev = (0..n)
            .map(|i| {
                let q = off_pt((ts[i] + ts[i + 1]) / 2.0);
                fit.closest_point(q).1.dist(q)
            })
            .fold(0.0, f64::max);
        if dev < 1e-5 || round == 3 {
            if dev > 1e-3 {
                return Err(Fail::Err(FeatureError::new("offsetFitFailed")));
            }
            let (a, b) = fit.domain();
            return Ok(ProfileSeg { tag, curve: Curve2::BSpline(fit), t0: a, t1: b, rev: s.rev });
        }
        n *= 2;
    }
    Err(Fail::Thick)
}

fn set_end(s: &mut ProfileSeg, t: f64) {
    if s.rev {
        s.t0 = t
    } else {
        s.t1 = t
    }
}
fn set_start(s: &mut ProfileSeg, t: f64) {
    if s.rev {
        s.t1 = t
    } else {
        s.t0 = t
    }
}
/// Travel length in parameter units (signed: negative = the trims crossed).
fn span(s: &ProfileSeg) -> f64 {
    (s.t_end() - s.t_start()) * if s.rev { -1.0 } else { 1.0 }
}

/// Polyline of a segment in travel order with the parameter of every point.
fn polyline(s: &ProfileSeg, n: usize) -> Vec<(f64, Vec2)> {
    let n = match s.curve {
        Curve2::Line { .. } => 1,
        _ => n,
    };
    (0..=n)
        .map(|k| {
            let t = s.t_start() + (s.t_end() - s.t_start()) * k as f64 / n as f64;
            (t, s.curve.eval(t))
        })
        .collect()
}

fn seg_cross(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Option<(f64, f64)> {
    let r = b - a;
    let q = d - c;
    let den = r.cross(q);
    if den.abs() < 1e-300 {
        return None;
    }
    let t = (c - a).cross(q) / den;
    let u = (c - a).cross(r) / den;
    ((-1e-12..=1.0 + 1e-12).contains(&t) && (-1e-12..=1.0 + 1e-12).contains(&u)).then_some((t, u))
}

/// Intersection of two consecutive offset pieces near their shared corner: parameters
/// `(on a, on b)`. Prefers the crossing latest along `a`.
fn intersect_pair(a: &ProfileSeg, b: &ProfileSeg) -> Option<(f64, f64)> {
    let (pa, pb) = (polyline(a, 96), polyline(b, 96));
    let mut best: Option<(f64, f64, f64)> = None; // (a fraction, ta, tb)
    for i in 0..pa.len() - 1 {
        for j in 0..pb.len() - 1 {
            if let Some((t, u)) = seg_cross(pa[i].1, pa[i + 1].1, pb[j].1, pb[j + 1].1) {
                let ta = pa[i].0 + (pa[i + 1].0 - pa[i].0) * t;
                let tb = pb[j].0 + (pb[j + 1].0 - pb[j].0) * u;
                let frac = (i as f64 + t) / (pa.len() - 1) as f64;
                if best.map_or(true, |b| frac > b.0) {
                    best = Some((frac, ta, tb));
                }
            }
        }
    }
    let (_, mut s, mut u) = best?;
    // Newton on a(s) = b(u)
    for _ in 0..8 {
        let (da, db) = (a.curve.derivs(s, 1), b.curve.derivs(u, 1));
        let f = da[0] - db[0];
        if f.len() < 1e-13 {
            break;
        }
        let det = da[1].cross(-db[1]);
        if det.abs() < 1e-14 {
            break;
        }
        // J [ds du]^T = -f, J = [a', -b']
        let ds = (-f).cross(-db[1]) / det;
        let du = da[1].cross(-f) / det;
        let (s2, u2) = (s + ds, u + du);
        let inside = |t: f64, g: &ProfileSeg| t >= g.t0 - 1e-9 && t <= g.t1 + 1e-9 || matches!(g.curve, Curve2::Line { .. } | Curve2::Circle { .. });
        if !inside(s2, a) || !inside(u2, b) {
            break;
        }
        s = s2;
        u = u2;
    }
    Some((s, u))
}

fn connector(tag: String, a: Vec2, b: Vec2) -> ProfileSeg {
    ProfileSeg::line(&tag, a, b)
}

/// The offset of a chain by `d` (> 0 left of travel) with its corner joins, as a travel-ordered
/// list of segments tagged `<entity>.<w>`.
fn offset_chain(src: &[ProfileSeg], d: f64, closed: bool, th: &Thin, w: &str) -> Result<Vec<ProfileSeg>, Fail> {
    let n = src.len();
    let mut offs: Vec<ProfileSeg> = src.iter().map(|s| offset_seg(s, d, w)).collect::<Result<_, _>>()?;
    let mut conn: Vec<Vec<ProfileSeg>> = vec![vec![]; n];
    let joins = if n == 1 { 0 } else if closed { n } else { n - 1 };
    for i in 0..joins {
        let j = (i + 1) % n;
        let ta = src[i].travel_tangent(src[i].t_end());
        let tb = src[j].travel_tangent(src[j].t_start());
        let cr = ta.cross(tb);
        let ang = cr.atan2(ta.dot(tb));
        if ang.abs() < 1e-7 {
            continue;
        }
        let v = src[i].end();
        let (pa, pb) = (offs[i].end(), offs[j].start());
        if cr * d < 0.0 {
            // the offset curves leave a gap on the outer side of the turn
            let tagc = |sfx: &str| format!("{}.{}{}", src[i].tag, w, sfx);
            if th.round_corners {
                let arc = if cr > 0.0 { ProfileSeg::arc(&tagc("r"), v, pa, pb) } else { ProfileSeg::arc(&tagc("r"), v, pb, pa).reversed() };
                conn[i].push(arc);
                continue;
            }
            let den = ta.cross(tb);
            let s = (pb - pa).cross(tb) / den;
            let m = pa + ta * s;
            if s <= 0.0 || m.dist(v) > MITRE_LIMIT * d.abs() {
                conn[i].push(connector(tagc("b"), pa, pb));
                continue;
            }
            // extend a straight piece, or bridge a curved one with a tangent line
            let line_t = |g: &ProfileSeg, p: Vec2| match &g.curve {
                Curve2::Line { o, d } => Some((p - *o).dot(*d) / d.len2()),
                _ => None,
            };
            match line_t(&offs[i], m) {
                Some(t) => {
                    let g = &mut offs[i];
                    set_end(g, t);
                }
                None => conn[i].push(connector(tagc("m"), pa, m)),
            }
            match line_t(&offs[j], m) {
                Some(t) => {
                    let g = &mut offs[j];
                    set_start(g, t);
                }
                None => conn[i].push(connector(format!("{}.{}m", src[j].tag, w), m, pb)),
            }
        } else {
            // inner side: both offsets overlap, trim them at their crossing
            let (s, u) = intersect_pair(&offs[i], &offs[j]).ok_or(Fail::Thick)?;
            set_end(&mut offs[i], s);
            set_start(&mut offs[j], u);
        }
    }
    if offs.iter().any(|s| span(s) < 1e-9) {
        return Err(Fail::Thick);
    }
    let mut out = vec![];
    for (o, c) in offs.into_iter().zip(conn) {
        out.push(o);
        out.extend(c);
    }
    Ok(out)
}

// ---------------------------------------------------------------- the wall region

fn poly_of(segs: &[ProfileSeg]) -> Vec<Vec2> {
    let mut pts = vec![];
    for s in segs {
        let pl = polyline(s, 40);
        pts.extend(pl[..pl.len() - 1].iter().map(|p| p.1));
    }
    pts
}

/// Does the closed polygon cross itself (non-adjacent edges)?
fn self_crosses(p: &[Vec2]) -> bool {
    let n = p.len();
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        for j in (i + 2)..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            let (c, d) = (p[j], p[(j + 1) % n]);
            if let Some((t, u)) = seg_cross(a, b, c, d) {
                if t > 1e-7 && t < 1.0 - 1e-7 && u > 1e-7 && u < 1.0 - 1e-7 {
                    return true;
                }
            }
        }
    }
    false
}

fn crosses_between(p: &[Vec2], q: &[Vec2]) -> bool {
    for i in 0..p.len() {
        let (a, b) = (p[i], p[(i + 1) % p.len()]);
        for j in 0..q.len() {
            let (c, d) = (q[j], q[(j + 1) % q.len()]);
            if seg_cross(a, b, c, d).is_some() {
                return true;
            }
        }
    }
    false
}

fn inside_poly(p: &[Vec2], q: Vec2) -> bool {
    let mut c = false;
    let n = p.len();
    for i in 0..n {
        let (a, b) = (p[i], p[(i + 1) % n]);
        if (a.y > q.y) != (b.y > q.y) && q.x < (b.x - a.x) * (q.y - a.y) / (b.y - a.y) + a.x {
            c = !c;
        }
    }
    c
}

fn tagged(segs: &[ProfileSeg], w: &str) -> Vec<ProfileSeg> {
    segs.iter().map(|s| retag(s, w)).collect()
}

/// The wall region of one chain for the thicknesses `(tl, tr)`.
fn wall(chain: &Chain, th: &Thin, tl: f64, tr: f64) -> Result<ProfileRegion, Fail> {
    let c = &chain.segs;
    // boundary list: (offset d, tag suffix). W0 is the offset side (left one with `both`)
    let (a, b): (Vec<ProfileSeg>, Vec<ProfileSeg>) = match (tl > 0.0, tr > 0.0) {
        (true, true) => (offset_chain(c, tl, chain.closed, th, "W0")?, offset_chain(c, -tr, chain.closed, th, "W1")?),
        (true, false) => (offset_chain(c, tl, chain.closed, th, "W0")?, tagged(c, "W1")),
        (false, true) => (offset_chain(c, -tr, chain.closed, th, "W0")?, tagged(c, "W1")),
        (false, false) => return Err(Fail::Err(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("thin.thickness".into())))),
    };
    // left / right boundary in travel direction
    let (left, right) = match (tl > 0.0, tr > 0.0) {
        (true, true) => (a, b),
        (true, false) => (a, b),
        _ => (b, a),
    };
    if chain.closed {
        let (pa, pb) = (poly_of(&left), poly_of(&right));
        let (la, lb) = (ProfileLoop::new(left.clone()).signed_area(), ProfileLoop::new(right.clone()).signed_area());
        // the loops must keep their orientation (an offset past the centre inverts a loop)
        let orig = ProfileLoop::new(c.clone()).signed_area();
        if (tl > 0.0 && la * orig <= 0.0) || (tr > 0.0 && lb * orig <= 0.0) {
            return Err(Fail::Thick);
        }
        if self_crosses(&pa) || self_crosses(&pb) || crosses_between(&pa, &pb) {
            return Err(Fail::Thick);
        }
        let (outer, hole) = if la.abs() >= lb.abs() { (left, right) } else { (right, left) };
        let (mut outer, mut hole) = (ProfileLoop::new(outer), ProfileLoop::new(hole));
        if outer.signed_area() < 0.0 {
            outer = outer.reversed();
        }
        if hole.signed_area() > 0.0 {
            hole = hole.reversed();
        }
        return Ok(ProfileRegion { outer, holes: vec![hole] });
    }
    // open: right boundary forward, end cap, left boundary backward, start cap
    let (r0, r1) = (right[0].start(), right[right.len() - 1].end());
    let (l0, l1) = (left[0].start(), left[left.len() - 1].end());
    let (first, last) = (&c[0].tag, &c[c.len() - 1].tag);
    let (cap_e, cap_s) = if th.round_ends {
        let (mid_e, mid_s) = ((r1 + l1) * 0.5, (l0 + r0) * 0.5);
        if r1.dist(l1) < 1e-9 {
            return Err(Fail::Err(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str("thin.thickness".into()))));
        }
        (ProfileSeg::arc(&format!("{last}.E1"), mid_e, r1, l1), ProfileSeg::arc(&format!("{first}.E0"), mid_s, l0, r0))
    } else {
        (ProfileSeg::line(&format!("{last}.E1"), r1, l1), ProfileSeg::line(&format!("{first}.E0"), l0, r0))
    };
    let mut segs = right;
    segs.push(cap_e);
    segs.extend(reverse_segs(&left));
    segs.push(cap_s);
    let mut outer = ProfileLoop::new(segs);
    if self_crosses(&poly_of(&outer.segs)) {
        return Err(Fail::Thick);
    }
    if outer.signed_area() < 0.0 {
        outer = ProfileLoop::new(reverse_segs(&outer.segs));
    }
    Ok(ProfileRegion { outer, holes: vec![] })
}

/// Smallest radius of curvature of the chain on the side the offset `d` moves towards.
fn min_concave_radius(c: &[ProfileSeg], d: f64) -> f64 {
    let mut best = f64::INFINITY;
    for s in c {
        let e = if s.rev { -d } else { d };
        for k in 0..=64 {
            let t = s.t0 + (s.t1 - s.t0) * k as f64 / 64.0;
            let kap = curvature(&s.curve, t);
            if kap * e > 1e-9 {
                best = best.min(1.0 / kap.abs());
            }
        }
    }
    best
}

fn too_thick(chain: &Chain, th: &Thin) -> FeatureError {
    // bisect the scale of the thickness that still builds
    let (mut lo, mut hi) = (0.0f64, 1.0f64);
    for _ in 0..16 {
        let k = (lo + hi) / 2.0;
        if wall(chain, th, th.tl * k, th.tr * k).is_ok() {
            lo = k;
        } else {
            hi = k;
        }
    }
    let mut r = f64::INFINITY;
    if th.tl > 0.0 {
        r = r.min(min_concave_radius(&chain.segs, th.tl));
    }
    if th.tr > 0.0 {
        r = r.min(min_concave_radius(&chain.segs, -th.tr));
    }
    let reach = th.tl.max(th.tr);
    let (limit, min_r) = if r.is_finite() && r <= reach * 1.5 {
        ("curvature", r)
    } else {
        let shortest = chain.segs.iter().map(|s| s.curve.length_between(s.t0, s.t1)).fold(f64::INFINITY, f64::min);
        ("corner", if r.is_finite() { r } else { shortest })
    };
    FeatureError::new(THIN_TOO_THICK)
        .with("limit", Json::Str(limit.into()))
        .with("minRadius", Json::Num((min_r * 1000.0).round() / 1000.0))
        .with("maxThickness", Json::Num(floor_hundredth(lo * th.thickness * 0.98)))
        .with("thickness", Json::Num(th.thickness))
        .with("face", Json::Str(chain.segs[0].tag.clone()))
}

/// The chains left after dropping the segments that lie on the revolve axis `(o, d)` (`d` unit):
/// the strip between an on-axis segment and its offset would revolve into a solid rod down the
/// middle of the hollow. A closed loop that loses segments becomes one open chain (its longest
/// kept run, rotated to start at its first segment); an open chain that loses a middle segment
/// splits into the runs. Lines only: no other curve can lie on a line.
fn drop_on_axis(chain: Chain, sk: &SketchResult, o: Vec3, d: Vec3) -> Vec<Chain> {
    let fr = &sk.frame;
    let w = |p: Vec2| fr.o + fr.x * p.x + fr.y * p.y;
    let on_axis = |s: &ProfileSeg| -> bool {
        matches!(s.curve, Curve2::Line { .. }) && [w(s.start()), w(s.end())].iter().all(|p| (*p - o).cross(d).len() < 1e-6)
    };
    let keep: Vec<bool> = chain.segs.iter().map(|s| !on_axis(s)).collect();
    if keep.iter().all(|&k| k) {
        return vec![chain];
    }
    let n = chain.segs.len();
    if chain.closed {
        let mut best: Option<(usize, usize)> = None; // (start, length) of the longest kept run
        for s0 in 0..n {
            if !keep[s0] {
                continue;
            }
            let mut len = 0;
            while len < n && keep[(s0 + len) % n] {
                len += 1;
            }
            if best.map_or(true, |(_, l)| len > l) {
                best = Some((s0, len));
            }
        }
        let Some((s0, len)) = best else { return vec![] };
        let segs = (0..len).map(|k| chain.segs[(s0 + k) % n].clone()).collect();
        return vec![Chain { segs, closed: false }];
    }
    let mut out: Vec<Chain> = vec![];
    let mut cur: Vec<ProfileSeg> = vec![];
    for (s, &k) in chain.segs.iter().zip(&keep) {
        if k {
            cur.push(s.clone());
        } else if !cur.is_empty() {
            out.push(Chain { segs: std::mem::take(&mut cur), closed: false });
        }
    }
    if !cur.is_empty() {
        out.push(Chain { segs: cur, closed: false });
    }
    out
}

/// The thin-wall profile of the sketch for the feature's `thin` option.
pub fn thin_profile(state: &RegenState, sk: &SketchResult, f: &Feature, th: &Thin) -> Result<Profile, FeatureError> {
    // a revolve's wall ignores segments lying on its axis (a closed loop that loses them
    // becomes an open chain): the strip between an on-axis segment and its offset would
    // revolve into a solid rod on the axis
    let axis = match &f.kind {
        FeatureKind::Revolve { .. } => f.raw.get("axis").and_then(|a| sketch_axis(state, sk, a)),
        _ => None,
    };
    let mut chains: Vec<Chain> = vec![];
    for chain in selected_chains(sk, f)? {
        match axis {
            Some((o, d)) => chains.extend(drop_on_axis(chain, sk, o, d)),
            None => chains.push(chain),
        }
    }
    if chains.is_empty() {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    let mut regions = vec![];
    for chain in chains {
        let chain = oriented(chain);
        match wall(&chain, th, th.tl, th.tr) {
            Ok(r) => regions.push(r),
            Err(Fail::Thick) => return Err(too_thick(&chain, th)),
            Err(Fail::Err(e)) => return Err(e),
        }
    }
    Ok(Profile::new(sk.frame, regions))
}

/// The profile an extrude / revolve sweeps: the thin wall when `thin` is on, else the sketch regions.
pub fn feature_profile(state: &RegenState, sk: &SketchResult, f: &Feature) -> Result<(Profile, bool), FeatureError> {
    match parse(state, f)? {
        Some(th) => Ok((thin_profile(state, sk, f, &th)?, true)),
        None => Ok((super::extrude::selected_profile(sk, f)?, false)),
    }
}
