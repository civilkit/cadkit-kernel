//! Sheet metal features, stage 1: `sheetRules`, `baseFlange`, `edgeFlange` (docs/sheet-metal.md).
//!
//! ```json
//! {"type":"sheetRules","id":"F1","name":"Steel2","thickness":2,"kFactor":0.44,"bendRadius":2}
//! {"type":"baseFlange","id":"F3","sketch":"F2","rule":"Steel2","flip":false}
//! {"type":"edgeFlange","id":"F4","edges":[[50,0,2],[50,60,2]],"height":20,"angle":90,"heightFrom":"outer","position":"inside"}
//! ```
//!
//! The sheet model (`sheet.rs`) lives in [`Brep::sheet`]; the folded solid is rebuilt from it after every sheet feature as the
//! union of the plate, one bend and one leg per flange (each an extrusion of a section with true arcs), so the bend faces are
//! cylinders and the cut-outs of a corner relief fall out of the extents. The regen result gains `sheetMetal`: one entry per
//! sheet feature with the rule and the flat pattern (outline, bend lines, bend table).
//!
//! Errors: `nothingToCut` (no sheet to flange), `missingSketch` / `noClosedProfile`, `invalidValue {field, detail, hint}`,
//! `flangeHeightTooLow {minHeight, detail, hint}`, `sheetMetalUnsupported {detail, hint}` (outside stage 1), and the boolean codes.

use super::extrude::{self, Combine};
use super::sheet::*;
use super::sheet_contour::Contour;
use super::sheet_fold::{self, zone_of, Fold, FoldPos};
use super::sheet_roles::{self, cyl_role, plane_role, Geo, RoleGeo};
use super::sheet_tree::{self, Flat3, FlangeReq, FoldReq, Tree};
use super::sheet_tree3d;
use super::Brep;
use crate::expr::{self, Vars};
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::{Frame, Mat3, Plane, Vec3};
use ck_core::xform::Xform;
use ck_geom::curve2::Curve2;
use ck_geom::Vec2;
use ck_json::{Json, Obj};
use ck_topo::body::Body;
use ck_topo::make::{ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

pub const UNSUPPORTED: &str = "sheetMetalUnsupported";
pub const HEIGHT_TOO_LOW: &str = "flangeHeightTooLow";

/// What a part has accumulated of sheet metal: the rules and the sheet.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SheetState {
    pub rules: Vec<Rule>,
    pub active: Option<usize>,
    pub model: Option<Model>,
}

/// The sheet and what the solid is built from.
#[derive(Debug, Clone, PartialEq)]
pub struct Model {
    pub sheet: Sheet,
    /// Frame of the base sketch.
    pub frame: Frame,
    /// `+1`: the base is extruded along the sketch normal, `-1`: against it. `N = frame.z * dir` points from the face the
    /// base started on to its cap face.
    pub dir: f64,
    /// The profile of the base as sketched (normalised: outer loop CCW), for the plate before any flange.
    pub profile: Profile,
    /// Faces and volume of the body this model made; another feature changing the body is noticed by it.
    pub sig: (usize, i64),
    pub batches: usize,
    /// The base is a contour flange: the chain and how it is extruded (`profile` is then its section).
    pub contour: Option<Contour>,
    /// The body is the flat sheet (`unfold`); `refold` rebuilds the folded one from the model.
    pub unfolded: bool,
    /// The named faces of the folded solid, for the role query.
    pub roles: Vec<RoleGeo>,
    /// The panel tree: once a sheet needs more than the legacy forms (flanges beside folds, flanges on any panel, corner
    /// reliefs) the tree is the model and `sheet` is only kept for the rule.
    pub tree: Option<Tree>,
}

impl Model {
    /// Where the flat layout of the legacy model sits in space.
    pub fn flat3(&self) -> Flat3 {
        match &self.contour {
            Some(c) => {
                let f = &self.frame;
                let d0 = c.dir(0);
                let m0 = c.mat(0);
                let s = if c.right { 1.0 } else { -1.0 };
                Flat3 {
                    o: f.o + f.x * c.pts[0][0] + f.y * c.pts[0][1],
                    x: f.x * d0[0] + f.y * d0[1],
                    y: f.z * s,
                    n: f.x * m0[0] + f.y * m0[1],
                }
            }
            None => Flat3 { o: self.frame.o, x: self.frame.x, y: self.frame.y, n: self.frame.z * self.dir },
        }
    }
}

/// The tree of a model: its own, or the exact conversion of a legacy sheet.
fn tree_of(m: &Model) -> Result<Tree, FeatureError> {
    if let Some(t) = &m.tree {
        return Ok(t.clone());
    }
    match &m.contour {
        Some(c) => Tree::from_contour(c, &m.sheet.rule, m.flat3()),
        None => Tree::from_sheet(&m.sheet, m.flat3()),
    }
    .map_err(|d| unsupported(d, "this sheet cannot be turned into a panel tree"))
}

pub(super) fn invalid(field: &str, detail: impl Into<String>, hint: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE)
        .with("field", Json::Str(field.into()))
        .with("detail", Json::Str(detail.into()))
        .with("hint", Json::Str(hint.into()))
}

pub(super) fn unsupported(detail: impl Into<String>, hint: &str) -> FeatureError {
    FeatureError::new(UNSUPPORTED).with("detail", Json::Str(detail.into())).with("hint", Json::Str(hint.into()))
}

/// A numeric field: a number, `"=expr"`, a plain number with an optional `mm` / `deg` unit, or an expression in the
/// variables (`Thickness * 4`). `None` when the field is absent or null.
fn num(raw: &Json, key: &str, vars: &Vars) -> Result<Option<f64>, FeatureError> {
    let bad = |d: String| invalid(key, d, "a number, or an expression such as \"=Thickness*4\"");
    let v = match raw.get(key) {
        None | Some(Json::Null) => return Ok(None),
        Some(Json::Num(n)) => *n,
        Some(Json::Str(s)) => {
            let t = s.trim();
            let t = t.strip_suffix("mm").or_else(|| t.strip_suffix("deg")).unwrap_or(t).trim();
            match t.parse::<f64>() {
                Ok(v) => v,
                Err(_) => expr::eval(t, Some(vars)).map_err(|_| bad(format!("`{s}` is not a number or an expression")))?,
            }
        }
        Some(_) => return Err(bad(format!("`{key}` must be a number"))),
    };
    if v.is_finite() {
        Ok(Some(v))
    } else {
        Err(bad(format!("`{key}` is not finite")))
    }
}

fn flag(raw: &Json, key: &str) -> bool {
    crate::sketch::jtruthy(raw.get(key))
}

fn choice<T>(raw: &Json, key: &str, parse: fn(&str) -> Option<T>, options: &str) -> Result<Option<T>, FeatureError> {
    match raw.get(key) {
        None | Some(Json::Null) => Ok(None),
        Some(Json::Str(s)) => parse(s).map(Some).ok_or_else(|| invalid(key, format!("`{s}` is not one of {options}"), options)),
        Some(_) => Err(invalid(key, format!("`{key}` must be text"), options)),
    }
}

/// `sheetRules`.
pub fn exec_sheet_rules(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let mut vars = state.vars.clone();
    let t = num(&f.raw, "thickness", &vars)?.unwrap_or(DEFAULT_THICKNESS);
    if t <= 0.0 {
        return Err(invalid("thickness", "the thickness must be above 0", "give the stock thickness in mm"));
    }
    vars.set("Thickness", t);
    let name = f.raw.get_str("name").filter(|n| !n.is_empty()).unwrap_or(&f.id).to_string();
    let mut rule = Rule::new(&name, t);
    let mut get = |key: &str, slot: &mut f64, min: f64, max: f64| -> Result<(), FeatureError> {
        if let Some(v) = num(&f.raw, key, &vars)? {
            if !(min..=max).contains(&v) {
                return Err(invalid(key, format!("`{key}` {v} is outside {min}..{max}"), "check the sheet rule"));
            }
            *slot = v;
        }
        Ok(())
    };
    get("kFactor", &mut rule.k_factor, 0.0, 1.0)?;
    get("bendRadius", &mut rule.bend_radius, 1e-6, 1e6)?;
    get("reliefWidth", &mut rule.relief_width, 0.0, 1e6)?;
    get("reliefDepth", &mut rule.relief_depth, 0.0, 1e6)?;
    get("cornerRelief", &mut rule.corner_relief, 0.0, 1e6)?;
    get("hemGap", &mut rule.hem_gap, 0.0, 1e6)?;
    get("gap", &mut rule.gap, 0.0, 1e6)?;
    // the defaults that follow the thickness also follow a given bend radius? No: only the thickness (`Thickness` in a field).
    if let Some(s) = choice(&f.raw, "cornerReliefShape", ReliefShape::parse, "notch, round, rectangular, tear")? {
        rule.corner_relief_shape = s;
    }
    let st = &mut br.sheet;
    let at = match st.rules.iter().position(|r| r.name == rule.name) {
        Some(k) => {
            st.rules[k] = rule.clone();
            k
        }
        None => {
            st.rules.push(rule.clone());
            st.rules.len() - 1
        }
    };
    if f.raw.get("active").map_or(true, |a| crate::sketch::jtruthy(Some(a))) {
        st.active = Some(at);
    }
    state.result.put_sheet_metal(&f.id, Obj::new().put("id", f.id.as_str()).put("type", "sheetRules").put("rule", rule.to_json()).build());
    Ok(())
}

fn body_sig(b: &Body) -> (usize, i64) {
    let faces: Vec<_> = b.face_ids().collect();
    (b.num_faces(), (ck_topo::mass::signed_volume(b, &faces) * 1e4).round() as i64)
}

fn v2(p: P) -> Vec2 {
    Vec2::new(p[0], p[1])
}

fn sample(l: &ProfileLoop) -> (Vec<P>, Vec<String>, Vec<bool>) {
    let (mut pts, mut tags, mut line) = (vec![], vec![], vec![]);
    for s in &l.segs {
        let is_line = matches!(s.curve, Curve2::Line { .. });
        let n = if is_line { 1 } else { 32 };
        let (ta, tb) = (s.t_start(), s.t_end());
        for k in 0..n {
            let p = s.curve.eval(ta + (tb - ta) * k as f64 / n as f64);
            pts.push([p.x, p.y]);
            tags.push(s.tag.clone());
            line.push(is_line);
        }
    }
    (pts, tags, line)
}

/// `baseFlange`: the sketched region extruded by the rule's thickness along the sketch normal.
pub fn exec_base_flange(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_some() || br.sheet.model.is_some() {
        return Err(unsupported("a part has one base flange and it comes first", "make the sheet body from a base flange in a part of its own"));
    }
    let sk = f.raw.get_str("sketch").and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let rule = rule_of(br, f)?;
    let t = rule.thickness;
    let profile = extrude::selected_profile(sk, f)?.normalized();
    if profile.regions.len() != 1 {
        return Err(invalid("sketch", format!("the base flange needs one closed region, the sketch has {}", profile.regions.len()), "sketch a single closed outline (holes inside it are allowed)"));
    }
    let dir = if flag(&f.raw, "flip") { -1.0 } else { 1.0 };
    let opts = if dir < 0.0 { ExtrudeOpts::flipped(t) } else { ExtrudeOpts::blind(t) };
    let region = &profile.regions[0];
    let (outline, tags, line) = sample(&region.outer);
    let holes: Vec<Vec<P>> = region.holes.iter().map(|h| sample(h).0).collect();
    let body = ck_topo::make::extrude(&f.id, &profile, opts).map_err(|e| FeatureError::new(&e))?;
    br.combine(&body, Combine::Add, false)?;
    let sheet = Sheet { base: f.id.clone(), rule, outline, tags, line, holes, flanges: vec![], folds: vec![], fold_root: None };
    let sig = br.body.as_ref().map(body_sig).unwrap_or((0, 0));
    let mut model = Model { sheet, frame: sk.frame, dir, profile, sig, batches: 0, contour: None, unfolded: false, roles: vec![], tree: None };
    let mut roles = vec![];
    plate_roles(&model, &mut roles);
    model.roles = roles;
    br.sheet.model = Some(model);
    report(br, state, f, "baseFlange");
    Ok(())
}

fn flat_of(m: &Model) -> Result<FlatPattern, FeatureError> {
    if let Some(t) = &m.tree {
        return Ok(t.flat_pattern(&m.sheet.rule, m.unfolded));
    }
    let mut fp = match &m.contour {
        Some(c) => {
            let plan = c.plan(&m.sheet.rule).map_err(|d| unsupported(d, "make the segments longer, or the radius smaller"))?;
            c.flat_pattern(&m.sheet.rule, &plan)
        }
        None => m.sheet.flat_pattern(),
    };
    fp.unfolded = m.unfolded;
    Ok(fp)
}

fn report(br: &Brep, state: &mut RegenState, f: &Feature, kind: &str) {
    let Some(m) = &br.sheet.model else { return };
    let Ok(fp) = flat_of(m) else { return };
    state.result.put_sheet_metal(
        &f.id,
        Obj::new()
            .put("id", f.id.as_str())
            .put("type", kind)
            .put("rule", m.sheet.rule.to_json())
            .put("flatPattern", fp.to_json())
            .put("roles", sheet_roles::roles_json(br.body.as_ref(), &m.roles))
            .build(),
    );
}

/// A pick: `[x,y,z]`, `{"at":[x,y,z]}`, `{"edge": n}` (index into the outline), `{"role": "P0.side.e2"}` /
/// `{"curve": "e2"}` (a wall by its sketch curve) or a role string, with optional per-edge overrides.
struct Pick<'a> {
    at: Option<[f64; 3]>,
    index: Option<usize>,
    role: Option<String>,
    /// The panel an index pick counts the edges of (default 0).
    panel: Option<usize>,
    over: Option<&'a Json>,
}

fn point(j: &Json) -> Option<[f64; 3]> {
    let a = j.as_arr()?;
    if a.len() == 3 {
        Some([a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?])
    } else {
        None
    }
}

fn picks(raw: &Json) -> Result<Vec<Pick<'_>>, FeatureError> {
    let hint = "edges: [[x,y,z], ...] a point on each edge, {\"edge\":n}, or {\"role\":\"P0.side.e2\"} a wall by role";
    let a = raw.get_arr("edges").filter(|a| !a.is_empty()).ok_or_else(|| invalid("edges", "name the edges to flange", hint))?;
    a.iter()
        .map(|j| {
            if let Some(p) = point(j) {
                return Ok(Pick { at: Some(p), index: None, role: None, panel: None, over: None });
            }
            if let Json::Str(s) = j {
                return Ok(Pick { at: None, index: None, role: Some(s.clone()), panel: None, over: None });
            }
            let at = j.get("at").and_then(point);
            let index = j.get("edge").and_then(|v| v.as_f64()).filter(|v| *v >= 0.0).map(|v| v as usize);
            let role = j.get_str("role").map(str::to_string).or_else(|| j.get_str("curve").map(|c| format!("P{}.side.{c}", j.get("panel").and_then(|v| v.as_f64()).unwrap_or(0.0) as usize)));
            if at.is_none() && index.is_none() && role.is_none() {
                return Err(invalid("edges", "an edge pick is [x,y,z], {\"at\":[x,y,z]}, {\"edge\":n} or {\"role\":\"P0.side.<curve>\"}", hint));
            }
            let panel = j.get("panel").and_then(|v| v.as_f64()).filter(|v| *v >= 0.0).map(|v| v as usize);
            Ok(Pick { at, index, role, panel, over: Some(j) })
        })
        .collect()
}

/// The outline edge a pick names and whether the pick is on the cap side of the base.
fn resolve_pick(pk: &Pick, model: &Model, t: f64) -> Result<(usize, bool), FeatureError> {
    let sh = &model.sheet;
    let nrm = model.frame.z * model.dir;
    if let Some(role) = &pk.role {
        let er = sheet_roles::parse_edge_role(role).map_err(|d| invalid("edges", d, "a wall by role: P0.side.<curve>, or an edge of it: P0.back|P0.side.<curve>"))?;
        if er.panel != 0 {
            return Err(unsupported(format!("`{role}`: only the walls of the base panel P0 take a flange in this stage"), "pick a wall of P0"));
        }
        let edge = sh.tags.iter().position(|tg| *tg == er.side).ok_or_else(|| {
            invalid("edges", format!("`{role}`: the base has no wall along the curve `{}` (its curves: {})", er.side, sh.tags.join(", ")), "name a sketch curve of the base outline")
        })?;
        return Ok((edge, er.back.unwrap_or(true)));
    }
    match (pk.index, pk.at) {
        (Some(i), _) if i < sh.n_edges() => Ok((i, true)),
        (Some(i), _) => Err(invalid("edges", format!("the outline has {} edges, there is no edge {i}", sh.n_edges()), "edge: an index into the outline")),
        (None, Some(p)) => {
            let r = Vec3::new(p[0], p[1], p[2]) - model.frame.o;
            let q = [r.dot(model.frame.x), r.dot(model.frame.y)];
            let mut best = (f64::MAX, 0usize);
            for i in (0..sh.n_edges()).filter(|&i| sh.line[i]) {
                let e = sh.edge(i);
                let u = ((q[0] - e.a[0]) * e.d[0] + (q[1] - e.a[1]) * e.d[1]).clamp(0.0, e.len);
                let c = [e.a[0] + e.d[0] * u, e.a[1] + e.d[1] * u];
                let dist = (q[0] - c[0]).hypot(q[1] - c[1]);
                if dist < best.0 - 1e-9 {
                    best = (dist, i);
                }
            }
            Ok((best.1, r.dot(nrm) >= t * 0.5 - 1e-9))
        }
        _ => unreachable!(),
    }
}

/// The sheet model of the part, or the error for a flange feature that cannot go on it.
fn live_model(br: &Brep, what: &str) -> Result<Model, FeatureError> {
    let Some(model) = br.sheet.model.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    if br.body.as_ref().map(body_sig) != Some(model.sig) {
        return Err(unsupported("another feature changed the sheet body", "sheet features must follow the base flange and other sheet features directly (cutting across bends comes later)"));
    }
    if model.unfolded {
        return Err(unsupported(format!("the sheet is unfolded, {what} needs it folded"), "add a refold feature first"));
    }
    let _ = what;
    Ok(model)
}

fn height_error(j: usize, min: f64) -> FeatureError {
    FeatureError::new(HEIGHT_TOO_LOW)
        .with("minHeight", Json::Num(min))
        .with("bend", Json::Num((j + 1) as f64))
        .with("detail", Json::Str(format!("the flange height must be at least {min:.3} mm for this bend")))
        .with("hint", Json::Str("raise the height, or reduce the radius or the angle".into()))
}

/// `edgeFlange`.
pub fn exec_edge_flange(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    exec_flange(br, state, f, false)
}

/// `hem`: a 180 degree bend of the rule's hem gap as inside radius, `length` to the outer face of the hem.
pub fn exec_hem(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    exec_flange(br, state, f, true)
}

/// Is the outline of the base convex (counter-clockwise, no reflex corner)?
fn convex_outline(o: &[P]) -> bool {
    let n = o.len();
    (0..n).all(|i| {
        let (a, b, c) = (o[i], o[(i + 1) % n], o[(i + 2) % n]);
        (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) >= -1e-9
    })
}

/// Does the feature ask for a partial-width or shaped flange (`start`, `length`, `from`, `to`, `profile`, `relief`, on the feature or on
/// one of its picks)? Those run on the panel tree.
fn partial_keys(raw: &Json, hem: bool) -> bool {
    // (a hem's `length` is its own: the length of the hem)
    const KEYS: [&str; 7] = ["start", "length", "from", "to", "profile", "holes", "relief"];
    let has = |j: &Json| KEYS.iter().any(|k| !(hem && *k == "length") && j.get(k).is_some());
    has(raw) || raw.get_arr("edges").map_or(false, |a| a.iter().any(|e| matches!(e, Json::Obj(_)) && has(e)))
}

/// A `[v, h]` pair of a profile.
fn pair(j: &Json) -> Option<P> {
    let a = j.as_arr()?;
    if a.len() == 2 {
        Some([a[0].as_f64()?, a[1].as_f64()?])
    } else {
        None
    }
}

/// Samples of an arc about `c` from `from` through `sweep` radians (the last point is not included).
fn arc_samples(c: P, from: P, sweep: f64) -> Vec<P> {
    let r = (from[0] - c[0]).hypot(from[1] - c[1]);
    let a0 = (from[1] - c[1]).atan2(from[0] - c[0]);
    let n = ((sweep.abs() / (std::f64::consts::PI / 32.0)).ceil() as usize).clamp(4, 128);
    (0..n)
        .map(|i| {
            let a = a0 + sweep * i as f64 / n as f64;
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}

/// The `profile` of a shaped flange: an inline list of segments `{"line":[[v,h],[v,h]]}` / `{"arc":{"center","from","to","ccw"}}`
/// forming one closed loop, or `{"sketch": id}` (the outer loop of the sketch's region; the lower left corner of its box is the
/// origin). Points `(v, h)` with their edge tags: lines `p<k>`, arcs `ra<k>` (a run of samples of one arc).
fn parse_profile(j: &Json, state: &RegenState, f: &Feature) -> Result<(Vec<(P, String)>, Vec<Vec<P>>), FeatureError> {
    let hint = "profile: [{\"line\":[[v,h],[v,h]]},{\"arc\":{\"center\":[v,h],\"from\":[v,h],\"to\":[v,h],\"ccw\":true}}, ...] a closed loop, v along the flange from its start, h from the height reference; or {\"sketch\":\"S2\"}";
    if let Some(id) = j.get_str("sketch") {
        let sk = state.sketch(id).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
        let profile = extrude::selected_profile(sk, f)?.normalized();
        let region = profile.regions.first().ok_or_else(|| invalid("profile", "the sketch has no closed region", hint))?;
        let (pts, tags, line) = sample(&region.outer);
        let (x0, y0) = (pts.iter().map(|p| p[0]).fold(f64::MAX, f64::min), pts.iter().map(|p| p[1]).fold(f64::MAX, f64::min));
        let shift = |q: P| [q[0] - x0, q[1] - y0];
        let outer = (0..pts.len()).map(|i| (shift(pts[i]), if line[i] { format!("p{}", tags[i]) } else { format!("ra{}", tags[i]) })).collect();
        let holes = region.holes.iter().map(|h| sample(h).0.into_iter().map(shift).collect()).collect();
        return Ok((outer, holes));
    }
    Ok((parse_loop(j, hint)?, vec![]))
}

/// One closed loop of `[v, h]` segments: lines `p<k>`, arcs sampled and tagged `ra<k>`.
fn parse_loop(j: &Json, hint: &str) -> Result<Vec<(P, String)>, FeatureError> {
    let segs = j.as_arr().filter(|a| a.len() >= 3).ok_or_else(|| invalid("profile", "a profile is a list of at least three segments", hint))?;
    let mut out: Vec<(P, String)> = vec![];
    let mut first: Option<P> = None;
    let mut prev_end: Option<P> = None;
    for (k, sg) in segs.iter().enumerate() {
        let (start, end, kind) = if let Some(l) = sg.get_arr("line") {
            let (a, b) = (l.first().and_then(pair), l.get(1).and_then(pair));
            (a, b, None)
        } else if let Some(ar) = sg.get("arc") {
            let (c, a, b) = (ar.get("center").and_then(pair), ar.get("from").and_then(pair), ar.get("to").and_then(pair));
            (a, b, c.map(|c| (c, ar.get("ccw").map_or(true, |v| crate::sketch::jtruthy(Some(v))))))
        } else {
            return Err(invalid("profile", format!("segment {k} is neither a line nor an arc"), hint));
        };
        let (Some(a), Some(b)) = (start, end) else { return Err(invalid("profile", format!("segment {k} needs two [v,h] points"), hint)) };
        if let Some(pe) = prev_end {
            if (pe[0] - a[0]).hypot(pe[1] - a[1]) > 1e-6 {
                return Err(invalid("profile", format!("segment {k} does not start where segment {} ends", k - 1), "the segments follow each other end to start"));
            }
        }
        first.get_or_insert(a);
        match kind {
            None => out.push((a, format!("p{k}"))),
            Some((c, ccw)) => {
                let (a0, a1) = ((a[1] - c[1]).atan2(a[0] - c[0]), (b[1] - c[1]).atan2(b[0] - c[0]));
                let tau = std::f64::consts::TAU;
                let sweep = if ccw { (a1 - a0).rem_euclid(tau) } else { -((a0 - a1).rem_euclid(tau)) };
                if sweep.abs() < 1e-9 {
                    return Err(invalid("profile", format!("arc {k} has no sweep (split a full circle in two arcs)"), hint));
                }
                for q in arc_samples(c, a, sweep) {
                    out.push((q, format!("ra{k}")));
                }
            }
        }
        prev_end = Some(b);
    }
    if let (Some(fi), Some(pe)) = (first, prev_end) {
        if (fi[0] - pe[0]).hypot(fi[1] - pe[1]) > 1e-6 {
            return Err(invalid("profile", "the profile is not closed", "the last segment ends where the first starts"));
        }
    }
    Ok(out)
}

/// The `holes` of a shaped flange: each `{"circle":{"center":[v,h],"radius":r}}` or a closed loop of segments as for the profile
/// (a loop is faceted: only circles are true cylinders).
fn parse_holes(j: &Json) -> Result<Vec<Vec<P>>, FeatureError> {
    let hint = "holes: [{\"circle\":{\"center\":[v,h],\"radius\":r}}, [segments...], ...]";
    let a = j.as_arr().ok_or_else(|| invalid("holes", "holes is a list", hint))?;
    a.iter()
        .map(|h| {
            if let Some(c) = h.get("circle") {
                let (Some(ctr), Some(r)) = (c.get("center").and_then(pair), c.get_f64("radius").filter(|r| *r > 0.0)) else {
                    return Err(invalid("holes", "a circle needs a center [v,h] and a radius above 0", hint));
                };
                return Ok((0..64).map(|i| {
                    let t = std::f64::consts::TAU * i as f64 / 64.0;
                    [ctr[0] + r * t.cos(), ctr[1] + r * t.sin()]
                }).collect());
            }
            Ok(parse_loop(h, hint)?.into_iter().map(|p| p.0).collect())
        })
        .collect()
}

fn exec_flange(br: &mut Brep, state: &mut RegenState, f: &Feature, hem: bool) -> Result<(), FeatureError> {
    let mut model = live_model(br, if hem { "a hem" } else { "an edge flange" })?;
    // the legacy forms: the plate of the base with flanges on its edges. Anything else is a panel tree.
    let off_plate = picks(&f.raw)?.iter().any(|pk| {
        pk.role.as_deref().and_then(|r| sheet_roles::parse_edge_role(r).ok()).map_or(pk.panel.unwrap_or(0), |er| er.panel) != 0
    });
    // a point pick above the plate (the free end of a flange, a wall of a leg) is not an edge of the base outline: the plate form would
    // snap it to the edge underneath and refuse it ("already has a flange"); the panel tree finds the wall the point is on
    let t0 = model.sheet.rule.thickness;
    let nrm0 = model.frame.z * model.dir;
    let above_plate = picks(&f.raw)?.iter().any(|pk| pk.role.is_none() && pk.index.is_none() && pk.at.map_or(false, |p| (Vec3::new(p[0], p[1], p[2]) - model.frame.o).dot(nrm0) > t0 + 0.25 * t0.max(0.5) + 1e-6));
    if model.tree.is_some()
        || model.contour.is_some()
        || above_plate
        || partial_keys(&f.raw, hem)
        || !model.sheet.straight()
        || !model.sheet.folds.is_empty()
        || model.sheet.rule.corner_relief_shape != ReliefShape::Notch
        || !convex_outline(&model.sheet.outline)
        || off_plate
    {
        return exec_flange_tree(br, state, f, hem, model);
    }
    let sh = &model.sheet;
    let t = sh.rule.thickness;
    let mut vars = state.vars.clone();
    vars.set("Thickness", t);
    let raw = &f.raw;
    let d_height = if hem { num(raw, "length", &vars)? } else { num(raw, "height", &vars)? };
    let hname = if hem { "length" } else { "height" };
    let d_angle = if hem { 180.0 } else { num(raw, "angle", &vars)?.unwrap_or(90.0) };
    let d_radius = if hem { Some(num(raw, "gap", &vars)?.unwrap_or(sh.rule.hem_gap)) } else { num(raw, "radius", &vars)? };
    let d_from = if hem { HeightFrom::Outer } else { choice(raw, "heightFrom", HeightFrom::parse, "outer, inner, tangent")?.unwrap_or(HeightFrom::Outer) };
    let d_pos = if hem { Position::Outside } else { choice(raw, "position", Position::parse, "inside, outside, middle")?.unwrap_or(Position::Inside) };
    let d_flip = flag(raw, "flip");
    let then = if hem { vec![] } else { parse_then(raw, &vars)? };
    let batch = model.batches + 1;
    let mut added: Vec<Flange> = vec![];
    for pk in picks(raw)? {
        let o = pk.over;
        let ov = |key: &str| -> Result<Option<f64>, FeatureError> { o.map_or(Ok(None), |j| num(j, key, &vars)) };
        let height = ov(hname)?.or(d_height).ok_or_else(|| invalid(hname, if hem { "the hem needs a length" } else { "the flange needs a height" }, if hem { "length: to the outer face of the hem, in mm" } else { "height: the flange height in mm" }))?;
        let angle = if hem { 180.0 } else { ov("angle")?.unwrap_or(d_angle) };
        let radius = if hem { ov("gap")?.or(d_radius).unwrap_or(sh.rule.hem_gap) } else { ov("radius")?.or(d_radius).unwrap_or(sh.rule.bend_radius) };
        let from = o.map_or(Ok(None), |j| choice(j, "heightFrom", HeightFrom::parse, "outer, inner, tangent"))?.unwrap_or(d_from);
        let pos = o.map_or(Ok(None), |j| choice(j, "position", Position::parse, "inside, outside, middle"))?.unwrap_or(d_pos);
        let flip = o.and_then(|j| j.get("flip")).map_or(d_flip, |v| crate::sketch::jtruthy(Some(v)));
        if !(height.is_finite() && height > 0.0) {
            return Err(invalid(hname, format!("the {hname} must be above 0"), if hem { "length: to the outer face of the hem, in mm" } else { "height: the flange height in mm" }));
        }
        if !(angle > 0.0 && angle < 179.5) && !hem {
            return Err(invalid("angle", format!("the flange angle {angle} must be between 0 and 180 degrees"), "angle: degrees the leg turns through, 90 for a right angle"));
        }
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(invalid(if hem { "gap" } else { "radius" }, "the bend radius must be above 0", if hem { "gap: the inside radius of the hem in mm" } else { "radius: the inside bend radius in mm" }));
        }
        let (edge, cap) = resolve_pick(&pk, &model, t)?;
        if !sh.line[edge] {
            return Err(invalid("edges", "that edge is curved", "flange a straight edge"));
        }
        if added.iter().any(|a| a.edge == edge) {
            continue;
        }
        if model.sheet.flange_on(edge).is_some() {
            return Err(unsupported(format!("edge {edge} already has a flange"), "one flange per edge in this stage"));
        }
        // a hem sits against the plate beside it: no neighbouring flange (and a flange next to a hem collides with it)
        let n = sh.n_edges();
        for nb in [(edge + n - 1) % n, (edge + 1) % n] {
            let other = model.sheet.flange_on(nb).or_else(|| added.iter().find(|a| a.edge == nb));
            if other.map_or(false, |o| o.hem || hem) {
                return Err(unsupported(format!("a hem on edge {edge} next to the flange on edge {nb}"), "hems go on edges that have no flange beside them"));
            }
        }
        let angle_r = if hem { std::f64::consts::PI } else { angle.to_radians() };
        chain_of(t, sh.rule.k_factor, radius, angle_r, height, from, pos, hem, &then).map_err(|(j, min)| height_error(j, min))?;
        added.push(Flange {
            feature: f.id.clone(),
            edge,
            height,
            angle: angle_r,
            radius,
            height_from: from,
            position: pos,
            up: if hem { !cap ^ flip } else { cap ^ flip },
            batch,
            leg: [0.0, 0.0],
            hem,
            then: then.clone(),
        });
    }
    model.sheet.flanges.extend(added);
    model.batches = batch;
    model.sheet.settle_batch(batch);
    // the flanges must leave something of every edge
    let q = model.sheet.straight_part();
    for fl in &model.sheet.flanges {
        let [x0, x1] = model.sheet.zone(fl.edge, &q);
        if x1 - x0 < 1e-6 || area(&q) < 1e-6 {
            return Err(unsupported(format!("the flange on edge {} leaves no width", fl.edge), "make the base larger or the bends smaller"));
        }
    }
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, if hem { "hem" } else { "edgeFlange" });
    Ok(())
}

/// `then`: further bends chained on the far edge of the flange.
fn parse_then(raw: &Json, vars: &Vars) -> Result<Vec<Next>, FeatureError> {
    let hint = "then: [{\"angle\":90,\"height\":10,\"flip\":false}, ...] at most 16 bends";
    let a = match raw.get("then") {
        None | Some(Json::Null) => return Ok(vec![]),
        Some(j) => j.as_arr().ok_or_else(|| invalid("then", "`then` must be a list of bends", hint))?,
    };
    if a.len() > MAX_THEN {
        return Err(invalid("then", format!("`then` may list at most {MAX_THEN} bends"), hint));
    }
    a.iter()
        .map(|j| {
            let angle = num(j, "angle", vars)?.unwrap_or(90.0);
            let height = num(j, "height", vars)?.ok_or_else(|| invalid("then", "every further bend needs a height", hint))?;
            if !(angle > 0.0 && angle < 179.5) {
                return Err(invalid("then", format!("the angle {angle} must be between 0 and 180 degrees"), hint));
            }
            if !(height.is_finite() && height > 0.0) {
                return Err(invalid("then", "the height must be above 0", hint));
            }
            Ok(Next { angle: angle.to_radians(), height, flip: flag(j, "flip") })
        })
        .collect()
}

pub(super) fn rule_of(br: &Brep, f: &Feature) -> Result<Rule, FeatureError> {
    Ok(match f.raw.get_str("rule") {
        Some(n) => br
            .sheet
            .rules
            .iter()
            .find(|r| r.name == n)
            .cloned()
            .ok_or_else(|| invalid("rule", format!("no sheet rule is named `{n}`"), "add a sheetRules feature with that name before the base flange"))?,
        None => br.sheet.active.and_then(|k| br.sheet.rules.get(k)).cloned().unwrap_or_else(|| Rule::new("Default", DEFAULT_THICKNESS)),
    })
}

/// The open chain of line segments a contour flange takes: the listed sketch curves (or every line of the sketch) joined end to
/// end. Returns the vertices and the id of each segment.
fn chain_from_sketch(sk: &crate::regen::SketchResult, curves: Option<Vec<String>>) -> Result<(Vec<P>, Vec<String>), FeatureError> {
    use crate::sketch::SketchEntity;
    let hint = "curves: the ids of the sketch lines, an open chain";
    let ids: Vec<String> = match curves {
        Some(c) if !c.is_empty() => c,
        _ => sk.sketch.entities.iter().filter(|e| matches!(e, SketchEntity::Line { .. }) && !e.construction()).map(|e| e.id().to_string()).collect(),
    };
    let mut segs: Vec<(String, P, P)> = vec![];
    for id in &ids {
        match sk.sketch.entity(id) {
            Some(SketchEntity::Line { p1, p2, .. }) => {
                let (a, b) = (sk.sketch.point(p1), sk.sketch.point(p2));
                match (a, b) {
                    (Some(a), Some(b)) => segs.push((id.clone(), [a.x, a.y], [b.x, b.y])),
                    _ => return Err(invalid("curves", format!("the line `{id}` has no end points"), hint)),
                }
            }
            Some(_) => return Err(invalid("curves", format!("`{id}` is not a line: a contour flange takes lines (arcs come later)"), hint)),
            None => return Err(invalid("curves", format!("the sketch has no curve `{id}`"), hint)),
        }
    }
    if segs.is_empty() {
        return Err(invalid("curves", "the sketch has no lines for a contour", hint));
    }
    let same = |a: P, b: P| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-6;
    let shared = |p: P, me: usize| segs.iter().enumerate().any(|(i, s)| i != me && (same(s.1, p) || same(s.2, p)));
    // the first segment with a free end starts the chain
    let start = segs.iter().enumerate().find_map(|(i, s)| {
        if !shared(s.1, i) {
            Some((i, false))
        } else if !shared(s.2, i) {
            Some((i, true))
        } else {
            None
        }
    });
    let Some((s0, rev)) = start else { return Err(invalid("curves", "the lines form a closed loop: a contour flange needs an open chain", hint)) };
    let mut used = vec![false; segs.len()];
    let first = if rev { (segs[s0].0.clone(), segs[s0].2, segs[s0].1) } else { segs[s0].clone() };
    used[s0] = true;
    let mut pts = vec![first.1, first.2];
    let mut tags = vec![first.0];
    loop {
        let end = *pts.last().unwrap();
        let next = segs.iter().enumerate().find(|(i, s)| !used[*i] && (same(s.1, end) || same(s.2, end)));
        match next {
            Some((i, s)) => {
                used[i] = true;
                let (a, b) = if same(s.1, end) { (s.1, s.2) } else { (s.2, s.1) };
                let _ = a;
                pts.push(b);
                tags.push(s.0.clone());
            }
            None => break,
        }
    }
    if used.iter().any(|u| !u) {
        return Err(invalid("curves", "the lines do not form one open chain (a branch, or a gap between two lines)", hint));
    }
    Ok((pts, tags))
}

/// `contourFlange`: the sheet made from an open chain of sketch lines extruded by `distance`.
pub fn exec_contour_flange(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    if br.body.is_some() || br.sheet.model.is_some() {
        return Err(unsupported("a part has one base flange and it comes first", "make the sheet body from a base flange in a part of its own"));
    }
    let sk = f.raw.get_str("sketch").and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let rule = rule_of(br, f)?;
    let mut vars = state.vars.clone();
    vars.set("Thickness", rule.thickness);
    let distance = num(&f.raw, "distance", &vars)?.ok_or_else(|| invalid("distance", "the contour flange needs a distance", "distance: how far the chain is extruded, in mm"))?;
    if !(distance > 0.0) {
        return Err(invalid("distance", "the distance must be above 0", "distance: how far the chain is extruded, in mm"));
    }
    let curves = f.raw.get_arr("curves").map(|a| a.iter().filter_map(|j| j.as_str().map(str::to_string)).collect::<Vec<_>>());
    let (pts, tags) = chain_from_sketch(sk, curves)?;
    let contour = Contour { feature: f.id.clone(), pts, tags, right: !flag(&f.raw, "flip"), reverse: flag(&f.raw, "reverse"), distance };
    let plan = contour.plan(&rule).map_err(|d| unsupported(d, "make the segments longer, or the radius smaller"))?;
    let section = contour.section(&rule, &plan);
    let profile = Profile::new(sk.frame, vec![ProfileRegion { outer: section, holes: vec![] }]).normalized();
    let dir = if contour.reverse { -1.0 } else { 1.0 };
    let sheet = Sheet {
        base: f.id.clone(),
        rule,
        outline: vec![],
        tags: vec![],
        line: vec![],
        holes: vec![],
        flanges: vec![],
        folds: vec![],
        fold_root: None,
    };
    let mut model = Model { sheet, frame: sk.frame, dir, profile, sig: (0, 0), batches: 0, contour: Some(contour), unfolded: false, roles: vec![], tree: None };
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "contourFlange");
    Ok(())
}

fn to_flat(m: &Model, p: [f64; 3]) -> P {
    let r = Vec3::new(p[0], p[1], p[2]) - m.frame.o;
    [r.dot(m.frame.x), r.dot(m.frame.y)]
}

/// The fold line in the plane of the base: two world points, or a sketch line (`"line":{"sketch":"S2","curve":"e5"}`) which the
/// fold follows when the sketch changes.
fn fold_line(state: &RegenState, raw: &Json, m: &Model) -> Result<(P, P), FeatureError> {
    use crate::sketch::SketchEntity;
    let hint = "line: {\"sketch\":\"S2\",\"curve\":\"e5\"} or points: [[x,y,z],[x,y,z]]";
    if let Some(l) = raw.get("line") {
        let sk = l.get_str("sketch").and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
        let curve = l.get_str("curve").ok_or_else(|| invalid("line", "name the sketch curve", hint))?;
        let Some(SketchEntity::Line { p1, p2, .. }) = sk.sketch.entity(curve) else { return Err(invalid("line", format!("`{curve}` is not a line of the sketch"), hint)) };
        let w = |id: &str| -> Result<P, FeatureError> {
            let p = sk.sketch.point(id).ok_or_else(|| invalid("line", "the line has no end point", hint))?;
            let v = sk.frame.o + sk.frame.x * p.x + sk.frame.y * p.y;
            Ok(to_flat(m, [v.x, v.y, v.z]))
        };
        return Ok((w(p1)?, w(p2)?));
    }
    let a = raw.get_arr("points").filter(|a| a.len() == 2).ok_or_else(|| invalid("points", "the fold needs a line", hint))?;
    let p = |j: &Json| point(j).map(|q| to_flat(m, q)).ok_or_else(|| invalid("points", "a point is [x,y,z]", hint));
    Ok((p(&a[0])?, p(&a[1])?))
}

/// `fold`: bend the base along a line. The plain forms (a convex base, square folds, no flanges) are the legacy fold; everything else
/// (oblique folds, concave panels, folds beside flanges, folds on any panel) is a fold of the panel tree.
pub fn exec_fold(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let model = live_model(br, "a fold")?;
    if model.tree.is_none() && model.contour.is_none() && model.sheet.flanges.is_empty() {
        match exec_fold_legacy(br, state, f) {
            Err(e) if e.code == UNSUPPORTED => {}
            other => return other,
        }
    }
    exec_fold_tree(br, state, f, model)
}

fn exec_fold_legacy(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let mut model = live_model(br, "a fold")?;
    if !model.sheet.flanges.is_empty() {
        return Err(unsupported("the base has flanges", "folds across a base with flanges come later: fold before flanging"));
    }
    let (pa, pb) = fold_line(state, &f.raw, &model)?;
    let t = model.sheet.rule.thickness;
    let mut vars = state.vars.clone();
    vars.set("Thickness", t);
    let raw = &f.raw;
    let angle = num(raw, "angle", &vars)?.unwrap_or(90.0);
    if !(angle > 0.0 && angle < 180.0) {
        return Err(invalid("angle", format!("the fold angle {angle} must be between 0 and 180 degrees"), "angle: degrees the moving side turns through"));
    }
    let radius = num(raw, "radius", &vars)?.unwrap_or(model.sheet.rule.bend_radius);
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(invalid("radius", "the bend radius must be above 0", "radius: the inside bend radius in mm"));
    }
    let position = choice(raw, "position", FoldPos::parse, "centerline, start, end, mould")?.unwrap_or(FoldPos::Centerline);
    let dv = [pb[0] - pa[0], pb[1] - pa[1]];
    let dl = dv[0].hypot(dv[1]);
    if dl < 1e-9 {
        return Err(invalid("points", "the fold line has no length", "give two different points"));
    }
    let d = [dv[0] / dl, dv[1] / dl];
    let nu0 = [-d[1], d[0]];
    let u0 = |p: P| (p[0] - pa[0]) * nu0[0] + (p[1] - pa[1]) * nu0[1];
    // the fixed side: a point or a wall by role on the side that stays
    let fixed: Option<P> = match raw.get("fixed") {
        None | Some(Json::Null) => None,
        Some(j) => {
            if let Some(p) = point(j) {
                Some(to_flat(&model, p))
            } else if let Some(r) = j.as_str().or_else(|| j.get_str("role")) {
                let er = sheet_roles::parse_edge_role(r).map_err(|d| invalid("fixed", d, "fixed: a point [x,y,z] on the side that stays, or a wall P0.side.<curve>"))?;
                let i = model.sheet.tags.iter().position(|tg| *tg == er.side).ok_or_else(|| invalid("fixed", format!("the base has no wall along `{}`", er.side), "name a sketch curve of the base outline"))?;
                let e = model.sheet.edge(i);
                Some([(e.a[0] + e.b[0]) * 0.5, (e.a[1] + e.b[1]) * 0.5])
            } else {
                return Err(invalid("fixed", "`fixed` is a point [x,y,z] or a wall by role", "fixed: a point [x,y,z] on the side that stays"));
            }
        }
    };
    let root: P = match model.sheet.fold_root {
        Some(r) => {
            if let Some(p) = fixed {
                if u0(p) * u0(r) < 0.0 {
                    return Err(unsupported("the fixed side of a further fold must keep the first fixed panel", "the panel the first fold left fixed stays; fold the other side"));
                }
            }
            r
        }
        None => {
            let (_, a_pos, c_pos) = sheet_fold::half(&model.sheet.outline, pa, nu0, true);
            let (_, a_neg, c_neg) = sheet_fold::half(&model.sheet.outline, pa, nu0, false);
            if a_pos < 1e-9 || a_neg < 1e-9 {
                return Err(invalid("points", "the fold line does not cross the base", "draw the line across the base"));
            }
            let pos_fixed = match fixed {
                Some(p) => u0(p) > 0.0,
                None => a_pos > a_neg + 1e-9,
            };
            if pos_fixed {
                c_pos
            } else {
                c_neg
            }
        }
    };
    let nu = if u0(root) > 0.0 { [-nu0[0], -nu0[1]] } else { nu0 };
    let allowance = (radius + model.sheet.rule.k_factor * t) * angle.to_radians();
    let setback = (radius + t) * (angle.to_radians() / 2.0).tan();
    let up = !flag(raw, "flip");
    model.sheet.folds.push(Fold {
        feature: f.id.clone(),
        o: pa,
        d,
        nu,
        angle: angle.to_radians(),
        radius,
        up,
        position,
        z: zone_of(position, allowance, setback),
        allowance,
        setback,
    });
    model.sheet.fold_root = Some(root);
    sheet_fold::check(&model.sheet).map_err(|d| unsupported(d, "folds need a straight convex base, bend zones that meet its edges at right angles and miss its holes"))?;
    sheet_fold::layout(&model.sheet, root).map_err(|d| unsupported(d, "fold across the base so that the fixed side is wider than the bend"))?;
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "fold");
    Ok(())
}

/// `unfold`: the body becomes the flat sheet.
pub fn exec_unfold(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(mut model) = br.sheet.model.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    if br.body.as_ref().map(body_sig) != Some(model.sig) {
        return Err(unsupported("another feature changed the sheet body", "unfold directly after the sheet features"));
    }
    if model.unfolded {
        return Err(unsupported("the sheet is already unfolded", "add a refold feature to fold it again"));
    }
    model.unfolded = true;
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "unfold");
    Ok(())
}

/// `refold`: the folded sheet again, rebuilt from the bend rows of the model.
pub fn exec_refold(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(mut model) = br.sheet.model.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    if !model.unfolded {
        return Err(unsupported("there is no unfolded sheet to refold", "unfold the sheet first"));
    }
    if br.body.as_ref().map(body_sig) != Some(model.sig) {
        return Err(unsupported("another feature changed the sheet body", "refold directly after the unfold"));
    }
    model.unfolded = false;
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "refold");
    Ok(())
}

// ---------------------------------------------------------------- the folded solid

fn world(m: &Model, p: P, h_n: f64) -> Vec3 {
    m.frame.o + m.frame.x * p[0] + m.frame.y * p[1] + m.frame.z * (m.dir * h_n)
}

fn ext_opts(m: &Model) -> ExtrudeOpts {
    let t = m.sheet.rule.thickness;
    if m.dir < 0.0 {
        ExtrudeOpts::flipped(t)
    } else {
        ExtrudeOpts::blind(t)
    }
}

fn line_loop(poly: &[(P, String)]) -> ProfileLoop {
    let n = poly.len();
    ProfileLoop::new((0..n).map(|i| ProfileSeg::line(&poly[i].1, v2(poly[i].0), v2(poly[(i + 1) % n].0))).collect())
}

/// The plate: the base flange with its flanged edges moved in.
fn plate(m: &Model, id: &str) -> Result<Body, FeatureError> {
    let sh = &m.sheet;
    let profile = if sh.flanges.is_empty() {
        m.profile.clone()
    } else {
        let q = sh.straight_part();
        let n = q.len();
        let segs = (0..n).map(|i| ProfileSeg::line(&sh.tags[i], v2(q[i]), v2(q[(i + 1) % n]))).collect();
        let reg = ProfileRegion { outer: ProfileLoop::new(segs), holes: m.profile.regions[0].holes.clone() };
        Profile::new(m.frame, vec![reg]).normalized()
    };
    ck_topo::make::extrude(id, &profile, ext_opts(m)).map_err(|e| FeatureError::new(&e))
}

/// A section plane of a strip: origin, the two in-plane axes and the direction of the line the section is extruded along.
pub(super) struct Sec {
    pub(super) o: Vec3,
    pub(super) xs: Vec3,
    pub(super) ys: Vec3,
    pub(super) d3: Vec3,
}

impl Sec {
    pub(super) fn zs(&self) -> Vec3 {
        self.xs.cross(self.ys)
    }
    pub(super) fn pt(&self, p: Vec2, along: f64) -> Vec3 {
        self.o + self.xs * p.x + self.ys * p.y + self.d3 * along
    }
    pub(super) fn frame(&self) -> Frame {
        Frame { o: self.o, x: self.xs, y: self.ys, z: self.zs() }
    }
    pub(super) fn span(&self, lo: f64, hi: f64) -> ExtrudeOpts {
        if self.zs().dot(self.d3) > 0.0 {
            ExtrudeOpts { s0: lo, s1: hi, dir: None, draft: 0.0 }
        } else {
            ExtrudeOpts { s0: -hi, s1: -lo, dir: None, draft: 0.0 }
        }
    }
}

/// One bend of a strip with the straight leg after it; `along` is where the pieces run on the line.
pub(super) struct Step {
    pub(super) angle: f64,
    /// Turns the other way from the first bend.
    pub(super) flip: bool,
    pub(super) leg: f64,
    pub(super) bend_along: [f64; 2],
    pub(super) leg_along: [f64; 2],
}

/// Where a strip stands after its last bend and leg, in section coordinates.
pub(super) struct Pose {
    p: Vec2,
    w: Vec2,
    n: Vec2,
}

fn travel(tag: &str, c: Vec2, from: Vec2, to: Vec2, ccw: bool) -> ProfileSeg {
    if ccw {
        ProfileSeg::arc(tag, c, from, to)
    } else {
        ProfileSeg::arc(tag, c, to, from).reversed()
    }
}

/// The bends and legs of a strip as solids, with their roles. Section coordinates: `x` runs back over the plate, `y` is the
/// side the first bend turns to; the strip leaves the plate at `x = b` along the line `y = 0` (surface A; B is `t` above it).
/// `bend0` / `panel0` number the first bend and leg for the roles; `a_front`: surface A is the front face of the sheet.
#[allow(clippy::too_many_arguments)]
pub(super) fn strip(sec: &Sec, id: &dyn Fn(char, usize) -> String, b: f64, r: f64, t: f64, steps: &[Step], bend0: usize, panel0: usize, a_front: bool, roles: &mut Vec<RoleGeo>) -> Result<(Vec<Body>, Pose), FeatureError> {
    let (na, nb) = if a_front { ("front", "back") } else { ("back", "front") };
    let mut pose = Pose { p: Vec2::new(b, 0.0), w: Vec2::new(-1.0, 0.0), n: Vec2::new(0.0, 1.0) };
    let mut out = vec![];
    for (j, st) in steps.iter().enumerate() {
        let sense = if st.flip { -1.0 } else { 1.0 };
        let (rho_a, rho_b) = if sense > 0.0 { (r + t, r) } else { (r, r + t) };
        let c = pose.p + pose.n * (if sense > 0.0 { r + t } else { -r });
        let (s, co) = st.angle.sin_cos();
        let w2 = pose.w * co + pose.n * (sense * s);
        let n2 = pose.n * co - pose.w * (sense * s);
        let (a_s, b_s) = (pose.p, pose.p + pose.n * t);
        let a_e = c + n2 * (-sense * rho_a);
        let b_e = c + n2 * (-sense * rho_b);
        let ccw = pose.w.cross(pose.n * sense) > 0.0;
        let hn = {
            let (s2, c2) = (st.angle * 0.5).sin_cos();
            pose.n * c2 - pose.w * (sense * s2)
        };
        let a_m = c + hn * (-sense * rho_a);
        let b_m = c + hn * (-sense * rho_b);
        let bend_loop = vec![travel("o", c, a_s, a_e, ccw), ProfileSeg::line("e1", a_e, b_e), travel("i", c, b_e, b_s, !ccw), ProfileSeg::line("e0", b_s, a_s)];
        let frame = sec.frame();
        let prof = |segs| Profile::new(frame, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]).normalized();
        let ext = |k: char, p: &Profile, o: ExtrudeOpts| ck_topo::make::extrude(&id(k, j), p, o).map_err(|e| FeatureError::new(&e));
        let [lo, hi] = st.bend_along;
        out.push(ext('b', &prof(bend_loop), sec.span(lo, hi))?);
        let bn = bend0 + j;
        let axis_pt = sec.pt(c, 0.0);
        for (name, rho, e0, em, e1) in [(na, rho_a, a_s, a_m, a_e), (nb, rho_b, b_s, b_m, b_e)] {
            let pts: Vec<Vec3> = [lo, hi].iter().flat_map(|&al| [sec.pt(e0, al), sec.pt(em, al), sec.pt(e1, al)]).collect();
            roles.push(cyl_role(&format!("B{bn}.{name}"), axis_pt, sec.d3, rho, &pts));
        }
        for (name, al) in [("start", lo), ("end", hi)] {
            let pts: Vec<Vec3> = [a_s, a_m, a_e, b_e, b_m, b_s].iter().map(|p| sec.pt(*p, al)).collect();
            roles.extend(plane_role(&format!("B{bn}.side.{name}"), &pts));
        }
        pose = Pose { p: a_e, w: w2, n: n2 };
        if st.leg > 1e-9 {
            let w = Vec2::new(w2.x, w2.y);
            let (a1, b1) = (a_e + w * st.leg, b_e + w * st.leg);
            let leg_loop = vec![ProfileSeg::line("a", a_e, a1), ProfileSeg::line("b", a1, b1), ProfileSeg::line("c", b1, b_e), ProfileSeg::line("d", b_e, a_e)];
            let [l0, l1] = st.leg_along;
            out.push(ext('l', &prof(leg_loop), sec.span(l0, l1))?);
            let pn = panel0 + j;
            for (name, p, q) in [(na, a_e, a1), (nb, b_e, b1), ("tip", a1, b1)] {
                let pts = [sec.pt(p, l0), sec.pt(q, l0), sec.pt(p, l1), sec.pt(q, l1)];
                roles.extend(plane_role(&format!("P{pn}.{name}"), &pts));
            }
            for (name, al) in [("start", l0), ("end", l1)] {
                let pts = [sec.pt(a_e, al), sec.pt(a1, al), sec.pt(b1, al), sec.pt(b_e, al)];
                roles.extend(plane_role(&format!("P{pn}.side.{name}"), &pts));
            }
            pose = Pose { p: a1, w: w2, n: n2 };
        }
    }
    Ok((out, pose))
}

/// The bends and legs of one edge flange (its own bend first, then the chained ones).
fn flange_pieces(m: &Model, fl: &Flange, bend0: usize, panel0: usize, roles: &mut Vec<RoleGeo>) -> Result<Vec<Body>, FeatureError> {
    let sh = &m.sheet;
    let (t, r) = (sh.rule.thickness, fl.radius);
    let ch = sh.chain(fl);
    let e = sh.edge(fl.edge);
    let q = sh.straight_part();
    let [x0, x1] = sh.zone(fl.edge, &q);
    // section frame: x = inward normal of the edge, y = towards the bend side, origin on the outline edge at the face the
    // bend leaves from
    let h_axis = m.frame.z * (m.dir * if fl.up { 1.0 } else { -1.0 });
    let xs = m.frame.x * e.m[0] + m.frame.y * e.m[1];
    let d3 = m.frame.x * e.d[0] + m.frame.y * e.d[1];
    let o = world(m, e.a, if fl.up { 0.0 } else { t });
    let sec = Sec { o, xs, ys: h_axis, d3 };
    let steps: Vec<Step> = ch
        .iter()
        .enumerate()
        .map(|(j, bd)| Step {
            angle: if j == 0 { fl.angle } else { fl.then[j - 1].angle },
            flip: j > 0 && fl.then[j - 1].flip,
            leg: bd.leg,
            bend_along: if j == 0 { [x0, x1] } else { fl.leg },
            leg_along: fl.leg,
        })
        .collect();
    let id = |k: char, j: usize| if j == 0 { format!("{}.{}{}", fl.feature, k, fl.edge) } else { format!("{}.{}{}_{}", fl.feature, k, fl.edge, j) };
    Ok(strip(&sec, &id, ch[0].b, r, t, &steps, bend0, panel0, fl.up, roles)?.0)
}

/// Roles of the plate (base panel): front and back and the walls of the unflanged edges.
fn plate_roles(m: &Model, roles: &mut Vec<RoleGeo>) {
    let sh = &m.sheet;
    let t = sh.rule.thickness;
    let q = if sh.flanges.is_empty() { sh.outline.clone() } else { sh.straight_part() };
    let n = q.len();
    for (name, h) in [("front", 0.0), ("back", t)] {
        let pts: Vec<Vec3> = q.iter().map(|p| world(m, *p, h)).collect();
        roles.extend(plane_role(&format!("P0.{name}"), &pts));
    }
    for i in 0..n {
        if sh.flange_on(i).is_none() {
            let pts = [world(m, q[i], 0.0), world(m, q[(i + 1) % n], 0.0), world(m, q[(i + 1) % n], t), world(m, q[i], t)];
            roles.extend(plane_role(&format!("P0.side.{}", sh.tags[i]), &pts));
        }
    }
}

fn xf_pt(x: &Xform, p: Vec3) -> Vec3 {
    x.apply_point(p)
}

/// The rigid motion that takes the flat far side of fold `k` to its folded place, in the frame of the base laid flat.
fn bend_xform(m: &Model, k: usize) -> (Xform, Sec) {
    let f = &m.sheet.folds[k];
    let t = m.sheet.rule.thickness;
    let nu3 = m.frame.x * f.nu[0] + m.frame.y * f.nu[1];
    let d3 = m.frame.x * f.d[0] + m.frame.y * f.d[1];
    let h_axis = m.frame.z * (m.dir * if f.up { 1.0 } else { -1.0 });
    let o3 = world(m, f.o, if f.up { 0.0 } else { t });
    let sec = Sec { o: o3, xs: -nu3, ys: h_axis, d3 };
    let (r, a) = (f.radius, f.angle);
    let b = -f.z[0];
    let c = Vec2::new(b, r + t);
    let (s, co) = a.sin_cos();
    let e_end = c + Vec2::new(-s, -co) * (r + t);
    let (w2, n2) = (Vec2::new(-co, s), Vec2::new(s, co));
    let zs = sec.zs();
    let dir3 = |v: Vec2| sec.xs * v.x + sec.ys * v.y;
    let p0 = sec.pt(Vec2::new(-f.z[1], 0.0), 0.0);
    let e3 = sec.pt(e_end, 0.0);
    let flat = Mat3::from_cols(dir3(Vec2::new(-1.0, 0.0)), dir3(Vec2::new(0.0, 1.0)), zs);
    let folded = Mat3::from_cols(dir3(w2), dir3(n2), zs);
    let rot = folded.mul_mat(&flat.transpose());
    (Xform::new(rot, e3 - rot.mul_vec(p0)), sec)
}

/// The solid of a folded base: the root panel, then for every bend of the walk the bend and the panel behind it.
fn fold_parts(m: &Model, roles: &mut Vec<RoleGeo>) -> Result<Vec<Body>, FeatureError> {
    let sh = &m.sheet;
    let t = sh.rule.thickness;
    let root = sh.fold_root.ok_or_else(|| unsupported("a folded base without a fixed panel", "fold again"))?;
    let lay = sheet_fold::layout(sh, root).map_err(|d| unsupported(d, "check the fold lines"))?;
    let mut xf: Vec<Xform> = vec![Xform::IDENTITY; lay.cells.len()];
    let mut out = vec![];
    for (i, cell) in lay.cells.iter().enumerate() {
        if let Some((pi, k)) = lay.parent[i] {
            let (bx, sec) = bend_xform(m, k);
            xf[i] = xf[pi].compose(&bx);
            let f = &sh.folds[k];
            let [v0, v1] = sheet_fold::chord(sh, k).map_err(|d| unsupported(d, "fold square to the edges"))?;
            let step = Step { angle: f.angle, flip: false, leg: 0.0, bend_along: [v0, v1], leg_along: [v0, v1] };
            let mut sub_roles = vec![];
            let id = |k2: char, _| format!("{}.{}{}", f.feature, k2, i);
            let (mut bodies, _) = strip(&sec, &id, -f.z[0], f.radius, t, &[step], k + 1, 0, f.up, &mut sub_roles)?;
            for bd in bodies.iter_mut() {
                bd.transform(&xf[pi]);
            }
            for r in sub_roles {
                roles.push(xform_role(&xf[pi], r));
            }
            out.append(&mut bodies);
        }
        // the panel
        let holes: Vec<ProfileLoop> = m
            .profile
            .regions
            .first()
            .map(|reg| reg.holes.iter().zip(&sh.holes).filter(|(_, poly)| poly.iter().all(|p| point_in(&cell.poly, *p))).map(|(h, _)| h.clone()).collect())
            .unwrap_or_default();
        let reg = ProfileRegion { outer: line_loop(&cell.poly), holes };
        let prof = Profile::new(m.frame, vec![reg]).normalized();
        let mut body = ck_topo::make::extrude(&format!("{}.p{}", sh.base, i), &prof, ext_opts(m)).map_err(|e| FeatureError::new(&e))?;
        body.transform(&xf[i]);
        out.push(body);
        // roles of the panel
        let nn = cell.poly.len();
        for (name, h) in [("front", 0.0), ("back", t)] {
            let pts: Vec<Vec3> = cell.poly.iter().map(|(p, _)| xf_pt(&xf[i], world(m, *p, h))).collect();
            roles.extend(plane_role(&format!("P{i}.{name}"), &pts));
        }
        for j in 0..nn {
            let tag = &cell.poly[j].1;
            if tag.starts_with('z') && tag[1..].chars().next().map_or(false, |c| c.is_ascii_digit()) {
                continue;
            }
            let (a, b2) = (cell.poly[j].0, cell.poly[(j + 1) % nn].0);
            let pts = [world(m, a, 0.0), world(m, b2, 0.0), world(m, b2, t), world(m, a, t)].map(|p| xf_pt(&xf[i], p));
            roles.extend(plane_role(&format!("P{i}.side.{tag}"), &pts));
        }
    }
    Ok(out)
}

pub(super) fn xform_role(x: &Xform, r: RoleGeo) -> RoleGeo {
    let map_box = |lo: Vec3, hi: Vec3| {
        let mut nlo = x.apply_point(lo);
        let mut nhi = nlo;
        for k in 0..8 {
            let p = Vec3::new(if k & 1 == 0 { lo.x } else { hi.x }, if k & 2 == 0 { lo.y } else { hi.y }, if k & 4 == 0 { lo.z } else { hi.z });
            let q = x.apply_point(p);
            nlo = nlo.min(q);
            nhi = nhi.max(q);
        }
        (nlo, nhi)
    };
    let geo = match r.geo {
        Geo::Plane { o, n, lo, hi } => {
            let (lo, hi) = map_box(lo, hi);
            Geo::Plane { o: x.apply_point(o), n: x.apply_vec(n), lo, hi }
        }
        Geo::Cyl { o, axis, r: rad, lo, hi } => {
            let (lo, hi) = map_box(lo, hi);
            Geo::Cyl { o: x.apply_point(o), axis: x.apply_vec(axis), r: rad, lo, hi }
        }
    };
    RoleGeo { role: r.role, geo }
}

fn point_in(poly: &[(P, String)], p: P) -> bool {
    let n = poly.len();
    (0..n).all(|i| {
        let (a, b) = (poly[i].0, poly[(i + 1) % n].0);
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -1e-7
    })
}

/// The contour flange as one extrusion of its section.
fn contour_parts(m: &Model, roles: &mut Vec<RoleGeo>) -> Result<Vec<Body>, FeatureError> {
    let c = m.contour.as_ref().expect("contour");
    let t = m.sheet.rule.thickness;
    let plan = c.plan(&m.sheet.rule).map_err(|d| unsupported(d, "make the segments longer, or the radius smaller"))?;
    let opts = if m.dir < 0.0 { ExtrudeOpts::flipped(c.distance) } else { ExtrudeOpts::blind(c.distance) };
    let body = ck_topo::make::extrude(&m.sheet.base, &m.profile, opts).map_err(|e| FeatureError::new(&e))?;
    // roles: the chain's face is `front`, the other face `back`; the extrusion ends are the sides
    let lift = |p: P, al: f64| m.frame.o + m.frame.x * p[0] + m.frame.y * p[1] + m.frame.z * (m.dir * al);
    let n = c.segments();
    let (z0, z1) = (0.0, c.distance);
    let off = |p: P, i: usize| [p[0] + c.mat(i)[0] * t, p[1] + c.mat(i)[1] * t];
    let mut panel = 0;
    let mut bend = 0;
    let mut cur = c.pts[0];
    for i in 0..n {
        let end = plan.bends.iter().find(|b| b.vertex == i + 1).map_or(c.pts[i + 1], |b| b.ta);
        if plan.legs[i] > 1e-9 {
            for (name, a, b) in [("front", cur, end), ("back", off(cur, i), off(end, i))] {
                let pts = [lift(a, z0), lift(b, z0), lift(a, z1), lift(b, z1)];
                roles.extend(plane_role(&format!("P{panel}.{name}"), &pts));
            }
            for (name, al) in [("start", z0), ("end", z1)] {
                let pts = [lift(cur, al), lift(end, al), lift(off(end, i), al), lift(off(cur, i), al)];
                roles.extend(plane_role(&format!("P{panel}.side.{name}"), &pts));
            }
        }
        panel += 1;
        if let Some(b) = plan.bends.iter().find(|b| b.vertex == i + 1) {
            bend += 1;
            let axis = lift(b.center, 0.0);
            let ax = m.frame.z;
            let r_o = if b.line_outer { b.r_line - t } else { b.r_line + t };
            let (ra, rb) = (b.r_line, r_o);
            let tb_o = off(b.tb, i + 1);
            let ta_o = off(b.ta, i);
            let mid = |p: P, q: P, rad: f64| {
                let (u, v) = ([p[0] - b.center[0], p[1] - b.center[1]], [q[0] - b.center[0], q[1] - b.center[1]]);
                let s = [u[0] + v[0], u[1] + v[1]];
                let l = s[0].hypot(s[1]).max(1e-12);
                [b.center[0] + s[0] / l * rad, b.center[1] + s[1] / l * rad]
            };
            let pts_a: Vec<Vec3> = [z0, z1].iter().flat_map(|&al| [lift(b.ta, al), lift(mid(b.ta, b.tb, ra), al), lift(b.tb, al)]).collect();
            let pts_b: Vec<Vec3> = [z0, z1].iter().flat_map(|&al| [lift(ta_o, al), lift(mid(ta_o, tb_o, rb), al), lift(tb_o, al)]).collect();
            roles.push(cyl_role(&format!("B{bend}.front"), axis, ax, ra, &pts_a));
            roles.push(cyl_role(&format!("B{bend}.back"), axis, ax, rb, &pts_b));
            for (name, al) in [("start", z0), ("end", z1)] {
                let pts: Vec<Vec3> = [b.ta, mid(b.ta, b.tb, ra), b.tb, tb_o, mid(ta_o, tb_o, rb), ta_o].iter().map(|p| lift(*p, al)).collect();
                roles.extend(plane_role(&format!("B{bend}.side.{name}"), &pts));
            }
            cur = b.tb;
        } else {
            cur = c.pts[i + 1];
        }
    }
    Ok(vec![body])
}

/// The flat sheet: the flat pattern extruded by the thickness where the base was.
fn slab(m: &Model) -> Result<Body, FeatureError> {
    let fp = flat_of(m)?;
    let t = m.sheet.rule.thickness;
    if let Some(c) = &m.contour {
        let len = fp.outline[0].iter().map(|p| p[0]).fold(0.0, f64::max);
        let d0 = c.dir(0);
        let m0 = c.mat(0);
        let lift = |p: P| m.frame.o + m.frame.x * p[0] + m.frame.y * p[1];
        let x = lift([d0[0], d0[1]]) - m.frame.o;
        let y = lift([m0[0], m0[1]]) - m.frame.o;
        let frame = Frame { o: lift(c.pts[0]), x, y, z: x.cross(y) };
        let along = m.frame.z * m.dir;
        let rect = vec![ProfileSeg::line("u0", v2([0.0, 0.0]), v2([len, 0.0])), ProfileSeg::line("u1", v2([len, 0.0]), v2([len, t])), ProfileSeg::line("u2", v2([len, t]), v2([0.0, t])), ProfileSeg::line("u3", v2([0.0, t]), v2([0.0, 0.0]))];
        let prof = Profile::new(frame, vec![ProfileRegion { outer: ProfileLoop::new(rect), holes: vec![] }]).normalized();
        let o = if frame.z.dot(along) > 0.0 { ExtrudeOpts::blind(c.distance) } else { ExtrudeOpts::flipped(c.distance) };
        return ck_topo::make::extrude(&format!("{}.flat", m.sheet.base), &prof, o).map_err(|e| FeatureError::new(&e));
    }
    let loops: Vec<ProfileLoop> = fp
        .outline
        .iter()
        .enumerate()
        .map(|(k, l)| {
            let n = l.len();
            ProfileLoop::new((0..n).map(|i| ProfileSeg::line(&format!("u{k}_{i}"), v2(l[i]), v2(l[(i + 1) % n]))).collect())
        })
        .collect();
    let mut it = loops.into_iter();
    let outer = it.next().ok_or_else(|| unsupported("empty flat pattern", "check the sheet"))?;
    let reg = ProfileRegion { outer, holes: it.collect() };
    let prof = Profile::new(m.frame, vec![reg]).normalized();
    ck_topo::make::extrude(&format!("{}.flat", m.sheet.base), &prof, ext_opts(m)).map_err(|e| FeatureError::new(&e))
}

/// The pieces of the solid in the order they are united, with their roles.
fn build_parts(m: &Model, roles: &mut Vec<RoleGeo>) -> Result<Vec<Body>, FeatureError> {
    if let Some(t) = &m.tree {
        if m.unfolded {
            return Ok(vec![sheet_tree3d::slab(m, t, &m.sheet.rule)?]);
        }
        return sheet_tree3d::parts(m, t, roles);
    }
    if m.unfolded {
        return Ok(vec![slab(m)?]);
    }
    if m.contour.is_some() {
        return contour_parts(m, roles);
    }
    if !m.sheet.folds.is_empty() {
        return fold_parts(m, roles);
    }
    let mut parts = vec![plate(m, &m.sheet.base)?];
    plate_roles(m, roles);
    let (mut bend_no, mut panel_no) = (1, 1);
    for fl in &m.sheet.flanges {
        parts.extend(flange_pieces(m, fl, bend_no, panel_no, roles)?);
        let nb = 1 + fl.then.len();
        bend_no += nb;
        panel_no += nb;
    }
    Ok(parts)
}

/// Rebuild the solid from the model: the plate, then every flange's bends and legs united in.
fn rebuild(br: &mut Brep, m: &mut Model) -> Result<(), FeatureError> {
    let before = br.body.take();
    let mut roles = vec![];
    let mut run = || -> Result<(), FeatureError> {
        br.body = None;
        for p in build_parts(m, &mut roles)? {
            br.combine(&p, Combine::Add, false)?;
        }
        if let (Some(t), false) = (&m.tree, m.unfolded) {
            for tool in sheet_tree3d::cut_tools(m, t)? {
                br.combine(&tool, Combine::Cut, false)?;
            }
        }
        Ok(())
    };
    if let Err(e) = run() {
        br.body = before;
        return Err(e);
    }
    m.roles = roles;
    m.sig = br.body.as_ref().map(body_sig).unwrap_or((0, 0));
    Ok(())
}

/// The roles of the sheet body of a part and the faces they name (the query behind `sheetMetal[].roles`).
pub fn sheet_roles(br: &Brep) -> Json {
    match &br.sheet.model {
        Some(m) => sheet_roles::roles_json(br.body.as_ref(), &m.roles),
        None => Obj::new().put("faces", Json::Arr(vec![])).put("edges", Json::Arr(vec![])).build(),
    }
}

// ---------------------------------------------------------------- the panel tree (stage 3)

/// A tree operation's refusal as a feature error.
fn tree_error(e: String) -> FeatureError {
    if let Some(rest) = e.strip_prefix("flangeHeightTooLow:") {
        let mut it = rest.split(':');
        let j: usize = it.next().and_then(|v| v.parse().ok()).unwrap_or(0);
        let min: f64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(0.0);
        return height_error(j, min);
    }
    unsupported(e, "check the picks, the heights and the radii; see docs/sheet-metal.md")
}

/// The panel, the edge of it and the side (cap or front) a pick names on the tree.
fn resolve_tree_pick(pk: &Pick, tree: &Tree, xfs: &[Xform], t: f64) -> Result<(usize, usize, bool), FeatureError> {
    if let Some(role) = &pk.role {
        let er = sheet_roles::parse_edge_role(role).map_err(|d| invalid("edges", d, "a wall by role: P<n>.side.<name>, or an edge of it: P<n>.back|P<n>.side.<name>"))?;
        let panel = tree.panels.get(er.panel).ok_or_else(|| invalid("edges", format!("`{role}`: the sheet has {} panels, there is no P{}", tree.panels.len(), er.panel), "name an existing panel"))?;
        let names: Vec<&str> = panel.poly.iter().map(|v| v.1.as_str()).filter(|g| !Tree::is_zone_tag(g)).collect();
        let edge = panel.edge_with(&er.side).ok_or_else(|| invalid("edges", format!("`{role}`: panel P{} has no wall `{}` (its walls: {})", er.panel, er.side, names.join(", ")), "name a wall of the panel"))?;
        return Ok((er.panel, edge, er.back.unwrap_or(true)));
    }
    match (pk.index, pk.at) {
        (Some(i), _) => {
            let p = pk.panel.unwrap_or(0);
            let panel = tree.panels.get(p).ok_or_else(|| invalid("edges", format!("the sheet has {} panels, there is no P{p}", tree.panels.len()), "panel: an index of a panel"))?;
            if i >= panel.poly.len() {
                return Err(invalid("edges", format!("panel P{p} has {} edges, there is no edge {i}", panel.poly.len()), "edge: an index into the ring of the panel"));
            }
            Ok((p, i, true))
        }
        (None, Some(p)) => {
            let w = Vec3::new(p[0], p[1], p[2]);
            sheet_tree3d::nearest_wall(tree, xfs, t, w).ok_or_else(|| invalid("edges", "the sheet has no wall near that point", "pick a point on the edge of a panel"))
        }
        _ => unreachable!(),
    }
}

fn exec_flange_tree(br: &mut Brep, state: &mut RegenState, f: &Feature, hem: bool, mut model: Model) -> Result<(), FeatureError> {
    let rule = model.sheet.rule.clone();
    let t = rule.thickness;
    let mut tree = tree_of(&model)?;
    let xfs = sheet_tree3d::xforms(&tree, t);
    let mut vars = state.vars.clone();
    vars.set("Thickness", t);
    let raw = &f.raw;
    let d_height = if hem { num(raw, "length", &vars)? } else { num(raw, "height", &vars)? };
    let hname = if hem { "length" } else { "height" };
    let d_angle = if hem { 180.0 } else { num(raw, "angle", &vars)?.unwrap_or(90.0) };
    let d_radius = if hem { Some(num(raw, "gap", &vars)?.unwrap_or(rule.hem_gap)) } else { num(raw, "radius", &vars)? };
    let d_from = if hem { HeightFrom::Outer } else { choice(raw, "heightFrom", HeightFrom::parse, "outer, inner, tangent")?.unwrap_or(HeightFrom::Outer) };
    let d_pos = if hem { Position::Outside } else { choice(raw, "position", Position::parse, "inside, outside, middle")?.unwrap_or(Position::Inside) };
    let d_flip = flag(raw, "flip");
    let then = if hem { vec![] } else { parse_then(raw, &vars)? };
    let batch = model.batches + 1;
    let mut reqs: Vec<FlangeReq> = vec![];
    for pk in picks(raw)? {
        let o = pk.over;
        let ov = |key: &str| -> Result<Option<f64>, FeatureError> { o.map_or(Ok(None), |j| num(j, key, &vars)) };
        // partial and shaped flanges: a key of the pick, else of the feature
        let get = |key: &str| -> Option<&Json> { o.and_then(|j| j.get(key)).or_else(|| raw.get(key)) };
        let nv = |key: &str| -> Result<Option<f64>, FeatureError> {
            match o.and_then(|j| j.get(key)) {
                Some(_) => num(o.expect("checked"), key, &vars),
                None => num(raw, key, &vars),
            }
        };
        let profile_raw = if hem { None } else { get("profile").filter(|j| !matches!(j, Json::Null)) };
        let hint = if hem { "length: to the outer face of the hem, in mm" } else { "height: the flange height in mm" };
        let (profile_pts, mut hole_pts) = match profile_raw {
            Some(j) => {
                let (p, h) = parse_profile(j, state, f)?;
                (Some(p), h)
            }
            None => (None, vec![]),
        };
        if profile_pts.is_some() {
            if let Some(hj) = get("holes").filter(|j| !matches!(j, Json::Null)) {
                hole_pts.extend(parse_holes(hj)?);
            }
        }
        let height = match &profile_pts {
            Some(pr) => Some(pr.iter().map(|p| p.0[1]).fold(f64::MIN, f64::max)),
            None => ov(hname)?.or(d_height),
        }
        .ok_or_else(|| invalid(hname, if hem { "the hem needs a length" } else { "the flange needs a height" }, hint))?;
        let angle = if hem { 180.0 } else { ov("angle")?.unwrap_or(d_angle) };
        let radius = if hem { ov("gap")?.or(d_radius).unwrap_or(rule.hem_gap) } else { ov("radius")?.or(d_radius).unwrap_or(rule.bend_radius) };
        let from = o.map_or(Ok(None), |j| choice(j, "heightFrom", HeightFrom::parse, "outer, inner, tangent"))?.unwrap_or(d_from);
        let pos = o.map_or(Ok(None), |j| choice(j, "position", Position::parse, "inside, outside, middle"))?.unwrap_or(d_pos);
        let flip = o.and_then(|j| j.get("flip")).map_or(d_flip, |v| crate::sketch::jtruthy(Some(v)));
        if !(height.is_finite() && height > 0.0) {
            return Err(invalid(hname, format!("the {hname} must be above 0"), hint));
        }
        if !(angle > 0.0 && angle < 179.5) && !hem {
            return Err(invalid("angle", format!("the flange angle {angle} must be between 0 and 180 degrees"), "angle: degrees the leg turns through, 90 for a right angle"));
        }
        if !(radius > 0.0 && radius.is_finite()) {
            return Err(invalid(if hem { "gap" } else { "radius" }, "the bend radius must be above 0", if hem { "gap: the inside radius of the hem in mm" } else { "radius: the inside bend radius in mm" }));
        }
        let (panel, edge, cap) = resolve_tree_pick(&pk, &tree, &xfs, t)?;
        // where along the edge: `start` / `length` (from the start of the edge), or the world points `from` / `to` on it
        let (ring_a, ring_b) = {
            let poly = &tree.panels[panel].poly;
            (poly[edge].0, poly[(edge + 1) % poly.len()].0)
        };
        let edge_len = ((ring_b[0] - ring_a[0]).hypot(ring_b[1] - ring_a[1])).max(1e-12);
        let dir = [(ring_b[0] - ring_a[0]) / edge_len, (ring_b[1] - ring_a[1]) / edge_len];
        let along = |w: [f64; 3]| -> f64 {
            let p = sheet_tree3d::flat_on(&tree, &xfs, panel, Vec3::new(w[0], w[1], w[2]));
            (p[0] - ring_a[0]) * dir[0] + (p[1] - ring_a[1]) * dir[1]
        };
        let (start, length) = (nv("start")?, if hem { None } else { nv("length")? });
        let (mut origin, mut forward, mut span) = (start.unwrap_or(0.0), true, None);
        match (get("from").and_then(point), get("to").and_then(point)) {
            (Some(fp), Some(tp)) => {
                let (s0, s1) = (along(fp), along(tp));
                forward = s1 >= s0;
                origin = s0;
                span = Some([s0.min(s1), s0.max(s1)]);
            }
            (None, None) => {
                if length.is_some() || (start.is_some() && profile_pts.is_none()) {
                    let s0 = start.unwrap_or(0.0);
                    span = Some([s0, length.map_or(edge_len, |l| s0 + l)]);
                }
            }
            _ => return Err(invalid("from", "`from` and `to` come as a pair (two points on the edge)", "from, to: [x,y,z] world points along the edge that bound the flange")),
        }
        if let Some(l) = length {
            if !(l > 0.0) {
                return Err(invalid("length", "the length of a partial flange must be above 0", "length: how far along the edge the flange runs, in mm"));
            }
        }
        let to_edge = |q: P| [if forward { origin + q[0] } else { origin - q[0] }, q[1]];
        let profile: Option<sheet_tree::Poly> = profile_pts.map(|pr| pr.into_iter().map(|(q, tg)| (to_edge(q), tg)).collect());
        let profile_holes: Vec<Vec<P>> = hole_pts.into_iter().map(|h| h.into_iter().map(to_edge).collect()).collect();
        let relief = match get("relief") {
            None | Some(Json::Null) => None,
            Some(Json::Str(s)) if s == "none" => None,
            Some(Json::Str(s)) if s == "rectangular" => Some((rule.relief_width, rule.relief_depth)),
            Some(j @ Json::Obj(_)) => Some((num(j, "width", &vars)?.unwrap_or(rule.relief_width), num(j, "depth", &vars)?.unwrap_or(rule.relief_depth))),
            Some(_) => return Err(invalid("relief", "relief is \"none\", \"rectangular\" or {\"width\":w,\"depth\":d}", "relief: the notch beside a partial flange that stops inside its edge")),
        };
        if reqs.iter().any(|r| r.panel == panel && r.edge == edge && r.span.is_none() && r.profile.is_none()) && span.is_none() && profile.is_none() {
            continue;
        }
        let angle_r = if hem { std::f64::consts::PI } else { angle.to_radians() };
        chain_of(t, rule.k_factor, radius, angle_r, height, from, pos, hem, &then).map_err(|(j, min)| height_error(j, min))?;
        if profile.is_some() && !then.is_empty() {
            return Err(unsupported("`then` on a flange with a profile", "chain the bends as flanges on the edges of the leg"));
        }
        reqs.push(FlangeReq { panel, edge, height, angle: angle_r, radius, from, pos, up: if hem { !cap ^ flip } else { cap ^ flip }, hem, then: then.clone(), span, profile, profile_holes, relief });
    }
    // a hem sits against the plate beside it: no neighbouring flange (and a flange next to a hem collides with it)
    for r in &reqs {
        let poly = &tree.panels[r.panel].poly;
        let n = poly.len();
        for nb in [(r.edge + n - 1) % n, (r.edge + 1) % n] {
            let tag = &poly[nb].1;
            let other_hem = tree.bend_on_edge(r.panel, tag).map(|k| tree.bends[k].hem).or_else(|| reqs.iter().find(|q| q.panel == r.panel && q.edge == nb).map(|q| q.hem));
            let other_flange = tree.bend_on_edge(r.panel, tag).is_some() || reqs.iter().any(|q| q.panel == r.panel && q.edge == nb);
            if other_flange && (r.hem || other_hem.unwrap_or(false)) {
                return Err(unsupported(format!("a hem on edge {} next to the flange on edge {nb}", r.edge), "hems go on edges that have no flange beside them"));
            }
        }
    }
    tree.add_flanges(&rule, &f.id, batch, &reqs).map_err(tree_error)?;
    model.tree = Some(tree);
    model.batches = batch;
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, if hem { "hem" } else { "edgeFlange" });
    Ok(())
}

/// The fold line in world coordinates: two points, or a sketch line.
fn fold_line_world(state: &RegenState, raw: &Json) -> Result<(Vec3, Vec3), FeatureError> {
    use crate::sketch::SketchEntity;
    let hint = "line: {\"sketch\":\"S2\",\"curve\":\"e5\"} or points: [[x,y,z],[x,y,z]]";
    if let Some(l) = raw.get("line") {
        let sk = l.get_str("sketch").and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
        let curve = l.get_str("curve").ok_or_else(|| invalid("line", "name the sketch curve", hint))?;
        let Some(SketchEntity::Line { p1, p2, .. }) = sk.sketch.entity(curve) else { return Err(invalid("line", format!("`{curve}` is not a line of the sketch"), hint)) };
        let w = |id: &str| -> Result<Vec3, FeatureError> {
            let p = sk.sketch.point(id).ok_or_else(|| invalid("line", "the line has no end point", hint))?;
            Ok(sk.frame.o + sk.frame.x * p.x + sk.frame.y * p.y)
        };
        return Ok((w(p1)?, w(p2)?));
    }
    let a = raw.get_arr("points").filter(|a| a.len() == 2).ok_or_else(|| invalid("points", "the fold needs a line", hint))?;
    let p = |j: &Json| point(j).map(|q| Vec3::new(q[0], q[1], q[2])).ok_or_else(|| invalid("points", "a point is [x,y,z]", hint));
    Ok((p(&a[0])?, p(&a[1])?))
}

/// Length of the infinite line `o + s d` inside a polygon.
fn chord_length(poly: &sheet_tree::Poly, o: P, d: P) -> f64 {
    let n = poly.len();
    let mut ss: Vec<f64> = vec![];
    for i in 0..n {
        let (a, b) = (poly[i].0, poly[(i + 1) % n].0);
        let e = [b[0] - a[0], b[1] - a[1]];
        let den = d[0] * e[1] - d[1] * e[0];
        if den.abs() < 1e-12 {
            continue;
        }
        let ao = [a[0] - o[0], a[1] - o[1]];
        let s = (ao[0] * e[1] - ao[1] * e[0]) / den;
        let u = (ao[0] * d[1] - ao[1] * d[0]) / den;
        if (0.0..1.0).contains(&u) {
            ss.push(s);
        }
    }
    ss.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    ss.chunks(2).filter(|c| c.len() == 2).map(|c| c[1] - c[0]).sum()
}

fn exec_fold_tree(br: &mut Brep, state: &mut RegenState, f: &Feature, mut model: Model) -> Result<(), FeatureError> {
    let rule = model.sheet.rule.clone();
    let t = rule.thickness;
    let mut tree = tree_of(&model)?;
    let xfs = sheet_tree3d::xforms(&tree, t);
    let mut vars = state.vars.clone();
    vars.set("Thickness", t);
    let raw = &f.raw;
    let angle = num(raw, "angle", &vars)?.unwrap_or(90.0);
    if !(angle > 0.0 && angle < 180.0) {
        return Err(invalid("angle", format!("the fold angle {angle} must be between 0 and 180 degrees"), "angle: degrees the moving side turns through"));
    }
    let radius = num(raw, "radius", &vars)?.unwrap_or(rule.bend_radius);
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(invalid("radius", "the bend radius must be above 0", "radius: the inside bend radius in mm"));
    }
    let position = choice(raw, "position", FoldPos::parse, "centerline, start, end, mould")?.unwrap_or(FoldPos::Centerline);
    let (wa, wb) = fold_line_world(state, raw)?;
    let hint_panel = raw.get("panel").and_then(|v| v.as_f64()).filter(|v| *v >= 0.0).map(|v| v as usize);
    if let Some(h) = hint_panel {
        if h >= tree.panels.len() {
            return Err(invalid("panel", format!("the sheet has {} panels, there is no P{h}", tree.panels.len()), "panel: the index of the panel to fold"));
        }
    }
    let located = sheet_tree3d::locate(&tree, &xfs, t, wa).or_else(|| sheet_tree3d::locate(&tree, &xfs, t, wb));
    let to_flat = |panel: Option<usize>, w: Vec3| -> P {
        match panel {
            Some(p) => tree.flat.flat_of(xfs[p].inverse_rigid().apply_point(w)).0,
            None => tree.flat.flat_of(w).0,
        }
    };
    let mut panel = hint_panel.or(located.map(|l| l.0));
    let (mut pa, mut pb) = (to_flat(panel, wa), to_flat(panel, wb));
    let dv = [pb[0] - pa[0], pb[1] - pa[1]];
    let dl = dv[0].hypot(dv[1]);
    if dl < 1e-9 {
        return Err(invalid("points", "the fold line has no length", "give two different points"));
    }
    let d = [dv[0] / dl, dv[1] / dl];
    if panel.is_none() {
        // the line in the flat layout: the panel it runs through longest
        let best = tree.panels.iter().enumerate().map(|(i, p)| (chord_length(&p.poly, pa, d), i)).fold((0.0, usize::MAX), |a, b| if b.0 > a.0 + 1e-9 { b } else { a });
        if best.1 == usize::MAX {
            return Err(unsupported("the fold line does not cross a panel (it is off the sheet or inside a bend zone)", "draw the line across a panel, clear of the bends"));
        }
        panel = Some(best.1);
    }
    let panel = panel.unwrap_or(0);
    if hint_panel.is_some() || located.is_some() {
        // points given in the folded state were mapped through the panel they lie on; make sure both ends use the same panel
        pa = to_flat(Some(panel), wa);
        pb = to_flat(Some(panel), wb);
    }
    let _ = pb;
    let fixed: Option<P> = match raw.get("fixed") {
        None | Some(Json::Null) => None,
        Some(j) => {
            if let Some(p) = point(j) {
                Some(to_flat(Some(panel), Vec3::new(p[0], p[1], p[2])))
            } else if let Some(r) = j.as_str().or_else(|| j.get_str("role")) {
                let er = sheet_roles::parse_edge_role(r).map_err(|d| invalid("fixed", d, "fixed: a point [x,y,z] on the side that stays, or a wall P<n>.side.<name>"))?;
                let pn = tree.panels.get(er.panel).ok_or_else(|| invalid("fixed", format!("no panel P{}", er.panel), "name an existing panel"))?;
                let e = pn.edge_with(&er.side).ok_or_else(|| invalid("fixed", format!("panel P{} has no wall `{}`", er.panel, er.side), "name a wall of the panel"))?;
                let (a, b) = (pn.poly[e].0, pn.poly[(e + 1) % pn.poly.len()].0);
                let mid = tree.flat.point([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5], t * 0.5);
                Some(to_flat(Some(panel), xfs[er.panel].apply_point(mid)))
            } else {
                return Err(invalid("fixed", "`fixed` is a point [x,y,z] or a wall by role", "fixed: a point [x,y,z] on the side that stays"));
            }
        }
    };
    let req = FoldReq { panel, o: pa, d, angle: angle.to_radians(), radius, up: !flag(raw, "flip"), position, fixed };
    tree.add_fold(&rule, &f.id, &req).map_err(tree_error)?;
    model.tree = Some(tree);
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "fold");
    Ok(())
}

/// Turn the legacy model of the part into its panel tree and rebuild the solid from the tree. The solid, the flat pattern and the
/// bend table do not change (the conversion is exact); features made afterwards work on the tree. For tests and tools.
pub fn convert_to_tree(br: &mut Brep) -> Result<(), FeatureError> {
    let Some(mut model) = br.sheet.model.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    model.tree = Some(tree_of(&model)?);
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    Ok(())
}

/// `mirror` of the whole body of a part that is a live sheet: the part stays a sheet.
///
/// * `replace`: the mirror image takes the place of the sheet. Its panel tree is the reflected tree (every panel, bend, cut-out and
///   the rule as they were; the layout frame is reflected, so the bend sides and the inside and outside of every bend stay correct),
///   which unfolds to the same flat pattern as the original. Any plane will do.
/// * otherwise the image is joined to the sheet, which is one sheet only when the two meet along a straight edge of the base plate
///   that carries no bend, with the plane square to the plate and nothing else of the sheet touching it. The joined panel tree
///   unfolds as a whole. Everything else (a plane across a face, apart from the sheet, through a bend, tilted) cannot be one sheet
///   and is refused with the reason, since a part holds one sheet.
///
/// `Ok(false)`: the part is not a live sheet (another feature changed the body, or it is unfolded); the caller mirrors the solid.
pub fn mirror_sheet(br: &mut Brep, pl: &Plane, replace: bool) -> Result<bool, FeatureError> {
    let Some(mut model) = br.sheet.model.clone() else { return Ok(false) };
    if model.unfolded || br.body.as_ref().map(body_sig) != Some(model.sig) {
        return Ok(false);
    }
    let tree = tree_of(&model)?;
    let fl = tree.flat;
    let t = model.sheet.rule.thickness;
    let hint_part = "mirror inside a part of its own and insert both, or pick a plane square to the base plate through one of its straight edges that carries no bend";
    if replace {
        let m = Xform::mirror(pl);
        let fl2 = Flat3 { o: m.apply_point(fl.o), x: m.apply_vec(fl.x), y: -m.apply_vec(fl.y), n: m.apply_vec(fl.n) };
        model.tree = Some(tree.reflected(fl2, |p| [p[0], -p[1]], |v| [v[0], -v[1]]));
        // the layout frame of the model, as `flat3` reads it back
        let z = fl2.x.cross(fl2.y);
        model.frame = Frame { o: fl2.o, x: fl2.x, y: fl2.y, z };
        model.dir = if z.dot(fl2.n) >= 0.0 { 1.0 } else { -1.0 };
        model.contour = None;
        rebuild(br, &mut model)?;
        br.sheet.model = Some(model);
        return Ok(true);
    }
    let across = pl.n.dot(fl.n).abs();
    if across > 1.0 - 1e-9 {
        let face = |h: f64| pl.signed_dist(fl.point([0.0, 0.0], h)).abs() < 1e-6;
        let why = if face(0.0) || face(t) {
            "the mirror plane is a face of the sheet: the two halves would touch over the whole face and the wall would double in thickness"
        } else {
            "the mirror plane is parallel to the sheet: the image lies over or across the sheet, not along its edge"
        };
        return Err(unsupported(why, hint_part));
    }
    if across > 1e-9 {
        return Err(unsupported("the mirror plane is neither square to the sheet nor parallel to it, so the image does not continue the base plate", hint_part));
    }
    let mut m2 = [pl.n.dot(fl.x), pl.n.dot(fl.y)];
    let l = (m2[0] * m2[0] + m2[1] * m2[1]).sqrt();
    m2 = [m2[0] / l, m2[1] / l];
    let c = fl.flat_of(pl.o).0;
    let d0 = m2[0] * c[0] + m2[1] * c[1];
    let line_dist = |p: P| m2[0] * p[0] + m2[1] * p[1] - d0;
    let refl = |p: P| {
        let k = 2.0 * line_dist(p);
        [p[0] - k * m2[0], p[1] - k * m2[1]]
    };
    let refl_v = |v: P| {
        let k = 2.0 * (m2[0] * v[0] + m2[1] * v[1]);
        [v[0] - k * m2[0], v[1] - k * m2[1]]
    };
    let image = tree.reflected(fl, refl, refl_v);
    let joined = tree.joined_mirror(&image, c, m2).map_err(|why| unsupported(format!("the mirror image cannot be joined to the sheet as one sheet: {why}"), hint_part))?;
    // the solids may meet along the seam face only: one side of the plane, four corners on it
    if let Some(b) = &br.body {
        let ds: Vec<f64> = b.vertices.iter().map(|v| pl.signed_dist(v.p)).collect();
        let (lo, hi) = ds.iter().fold((f64::MAX, f64::MIN), |a, d| (a.0.min(*d), a.1.max(*d)));
        let on = ds.iter().filter(|d| d.abs() <= 1e-6).count();
        if lo < -1e-6 && hi > 1e-6 {
            return Err(unsupported("the mirror plane cuts through the sheet, so the image overlaps it", hint_part));
        }
        if on != 4 {
            return Err(unsupported("the sheet touches the mirror plane at more than the end face of the base plate (a bend or flange reaches it), so the halves would fuse along more than a seam", hint_part));
        }
    }
    model.tree = Some(joined);
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    Ok(true)
}

pub(super) fn report_mirror(br: &Brep, state: &mut RegenState, f: &Feature) {
    report(br, state, f, "mirror")
}

/// `sheetCorner`: round or chamfer corners of the panels (free walls only).
/// `{"type":"sheetCorner","corners":[[x,y,z] | {"panel":k,"vertex":i}],"mode":"round"|"chamfer","radius":r | "distance":d}`
pub fn exec_sheet_corner(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let mut model = live_model(br, "a corner")?;
    let t = model.sheet.rule.thickness;
    let mut tree = tree_of(&model)?;
    let xfs = sheet_tree3d::xforms(&tree, t);
    let mut vars = state.vars.clone();
    vars.set("Thickness", t);
    let raw = &f.raw;
    let hint = "mode: \"round\" with radius, or \"chamfer\" with distance";
    let chamfer = match raw.get_str("mode").unwrap_or("round") {
        "round" | "fillet" => false,
        "chamfer" => true,
        other => return Err(invalid("mode", format!("`{other}` is not round or chamfer"), hint)),
    };
    let size = if chamfer { num(raw, "distance", &vars)? } else { num(raw, "radius", &vars)? }
        .or(num(raw, "size", &vars)?)
        .ok_or_else(|| invalid(if chamfer { "distance" } else { "radius" }, "the corner needs a size", hint))?;
    if !(size > 0.0 && size.is_finite()) {
        return Err(invalid(if chamfer { "distance" } else { "radius" }, "the size must be above 0", hint));
    }
    let list = raw.get_arr("corners").filter(|a| !a.is_empty()).ok_or_else(|| invalid("corners", "name the corners", "corners: [[x,y,z], {\"panel\":1,\"vertex\":2}]"))?;
    let mut todo: Vec<(usize, usize)> = vec![];
    for j in list {
        if let Some(p) = point(j) {
            let w = Vec3::new(p[0], p[1], p[2]);
            let mut best: Option<(f64, usize, usize)> = None;
            for (i, pn) in tree.panels.iter().enumerate().filter(|(_, p)| p.area() > 1e-9) {
                for (k, v) in pn.poly.iter().enumerate() {
                    let d = xfs[i].apply_point(tree.flat.point(v.0, t * 0.5)).dist(w);
                    if best.map_or(true, |b| d < b.0 - 1e-9) {
                        best = Some((d, i, k));
                    }
                }
            }
            let b = best.ok_or_else(|| invalid("corners", "the sheet has no corner", hint))?;
            todo.push((b.1, b.2));
        } else {
            let pnl = j.get("panel").and_then(|v| v.as_f64()).unwrap_or(0.0) as usize;
            let vx = j.get("vertex").and_then(|v| v.as_f64()).filter(|v| *v >= 0.0).ok_or_else(|| invalid("corners", "a corner is [x,y,z] or {\"panel\":k,\"vertex\":i}", hint))? as usize;
            if pnl >= tree.panels.len() {
                return Err(invalid("corners", format!("no panel P{pnl}"), hint));
            }
            todo.push((pnl, vx));
        }
    }
    todo.sort_unstable();
    todo.dedup();
    for (pn, vx) in todo.into_iter().rev() {
        tree.round_corner(pn, vx, chamfer, size).map_err(tree_error)?;
    }
    tree.refresh();
    model.tree = Some(tree);
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "sheetCorner");
    Ok(())
}

/// Is this plain `extrude` cut a cut through the thickness of the live sheet, so that it is a sheet cut (the part stays a sheet
/// and unfolds)? It is when the sheet is folded and untouched since its last sheet feature, the cut goes all the way through
/// (through all, both ways, or a blind depth of at least the thickness), has no draft or start offset, and the sketch lies
/// parallel to the layout of the sheet (the cut then crosses bends straight through the thickness, as a sheet cut does).
pub fn plain_cut_is_sheet_cut(br: &Brep, state: &RegenState, f: &Feature) -> bool {
    let super::FeatureKind::Extrude { op: super::Op::Cut, sketch: Some(sk_id), end, depth, draft: None, .. } = &f.kind else { return false };
    if super::has_start(f) {
        return false;
    }
    let Some(model) = &br.sheet.model else { return false };
    if model.unfolded || br.body.as_ref().map(body_sig) != Some(model.sig) {
        return false;
    }
    match end {
        super::End::Through | super::End::ThroughBoth => {}
        super::End::Blind => match depth.as_num() {
            Some(d) if d >= model.sheet.rule.thickness - 1e-9 => {}
            _ => return false,
        },
        _ => return false,
    }
    let Some(sk) = state.sketch(sk_id.as_str()) else { return false };
    sk.frame.z.dot(model.flat3().n).abs() > 1.0 - 1e-9
}

/// Warning: a plain extrude cut of a folded sheet was tried as a sheet cut, which refused, so it ran as an ordinary cut of the solid and
/// the part is no longer an unfoldable sheet. Params: `detail`, `hint`, `reason` (the code of the sheet-cut refusal) and `reasonDetail` (its `detail`, when it has one).
pub const SHEET_CUT_FALLBACK: &str = "sheetCutFallback";

pub fn sheet_cut_fallback_warning(refusal: &FeatureError) -> FeatureError {
    let mut w = FeatureError::new(SHEET_CUT_FALLBACK)
        .with("detail", Json::Str("this cut was applied as a plain cut of the solid; the part is no longer an unfoldable sheet".into()))
        .with("hint", Json::Str("check that the slot or sketch is aligned with the sheet; a cut a few nanometres off axis leaves a sliver the sheet cut refuses".into()))
        .with("reason", Json::Str(refusal.code.clone()));
    if let Some((_, d)) = refusal.params.iter().find(|(k, _)| k == "detail") {
        w = w.with("reasonDetail", d.clone());
    }
    w
}

/// `sheetCut`: cut the closed regions of a sketch out of the sheet, through the flat pattern. The sketch is read in the flat layout
/// (a sketch on the plane of the base, or parallel to it): its regions are removed from every panel they meet and from the bend
/// zones they cross (the cut is mapped through the bend, so it runs across bends).
/// `{"type":"sheetCut","sketch":"S2","regions":[...]}`
pub fn exec_sheet_cut(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let mut model = live_model(br, "a cut")?;
    let sk = f.raw.get_str("sketch").and_then(|s| state.sketch(s)).ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let profile = extrude::selected_profile(sk, f)?.normalized();
    if profile.regions.is_empty() {
        return Err(invalid("sketch", "the sketch has no closed region to cut", "draw closed profiles (circles, rectangles, slots)"));
    }
    let mut tree = tree_of(&model)?;
    for reg in &profile.regions {
        if !reg.holes.is_empty() {
            return Err(unsupported("a cut region with an island inside", "cut the regions one by one"));
        }
        let (pts, _, _) = sample(&reg.outer);
        let flat: Vec<P> = pts
            .iter()
            .map(|p| {
                let w = sk.frame.o + sk.frame.x * p[0] + sk.frame.y * p[1];
                tree.flat.flat_of(w).0
            })
            .collect();
        tree.cut(&flat).map_err(tree_error)?;
    }
    model.tree = Some(tree);
    rebuild(br, &mut model)?;
    br.sheet.model = Some(model);
    report(br, state, f, "sheetCut");
    Ok(())
}

/// Rebuild the solid of a model made from a body (`sheetConvert`) and check its volume against the original `vol0`; on a
/// mismatch the original body stays and the feature is refused.
pub(super) fn rebuild_checked(br: &mut Brep, m: &mut Model, vol0: f64) -> Result<(), FeatureError> {
    let before = br.body.clone();
    rebuild(br, m)?;
    let v = br.body.as_ref().map(|b| ck_topo::mass::mass_props(b).volume).unwrap_or(0.0);
    if (v - vol0).abs() > 0.01 * vol0.abs() {
        br.body = before;
        return Err(unsupported(format!("the sheet rebuilt from the body has volume {v:.3} against {vol0:.3} of the body"), "the body is not a plain bent plate (reliefs, corner cut-outs, embossed features are not recognised)"));
    }
    Ok(())
}

pub(super) fn report_pub(br: &Brep, state: &mut RegenState, f: &Feature, kind: &str) {
    report(br, state, f, kind)
}
