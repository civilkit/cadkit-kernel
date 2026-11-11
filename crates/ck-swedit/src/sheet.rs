//! Sheet metal parts: the `Sheet-Metal` rules feature and the `Base-Flange` of a SolidWorks sheet metal part.
//!
//! * `Sheet-Metal` (the feature that makes the part a sheet metal part) lists the bend radius (`D1`), the k-factor
//!   (`D2`, a plain number) and the `Thickness`, and made no faces: it becomes our `sheetRules` feature. The later
//!   `Sheet-MetalN` entries (one after every sheet metal operation, no dimensions), `Flat-Pattern` and `Cut list`
//!   hold no geometry and are skipped.
//! * `Base-Flange` is an extrusion of a closed sketch by the sheet thickness: its faces are read back like a plain
//!   boss extrude (`profile::extrude`: caps and sides, the outline from the cap, with the corner rounds later features
//!   made), and the extrude is then written as our `baseFlange` on the rules (the thickness must equal the extrude's
//!   depth, else the step is unsupported).
//!
//! * `Edge-Flange` (with the `EdgeBend` feature after it, which holds its bend faces) is read back as our `edgeFlange`: the
//!   bend cylinders give the radius, the angle, the side it turns to and the bend line (the span along the parent edge), the
//!   flange's inner cap face gives the leg's outline (lines and arcs, `profile`); see [`edge_flange`].
//!
//! Everything else of a sheet metal tree (`Hem`, `Jog`, `Sketched-Bend`, `Miter-Flange`, ...) is not read back: the step stays
//! unsupported and the file keeps the editable steps before it and the imported remainder (`prefix.rs`), or its plain solid.

use crate::{profile, Cx};
use ck_json::{Json, Obj};
use ck_recog::emit::n;
use ck_sldprt::meta::SwFeature;
use ck_recog::model::Kind;
use ck_topo::body::FaceId;
use std::collections::HashSet;

/// The plate of the base flange just read.
#[derive(Clone)]
struct Plate {
    normal: ck_geom::Vec3,
    caps: HashSet<FaceId>,
    /// Edges of the caps' outer loops (the outline the base flange sketch has) and of their inner loops (holes).
    outer: HashSet<ck_topo::body::EdgeId>,
    inner: HashSet<ck_topo::body::EdgeId>,
}

thread_local! {
    /// The base flange just read: the normal of its plate and its two cap faces (for the corner rounds that follow).
    static PLATE: std::cell::RefCell<Option<Plate>> = const { std::cell::RefCell::new(None) };
}

/// A new read starts: no plate yet.
pub fn reset() {
    PLATE.with(|p| *p.borrow_mut() = None);
}

/// Is `f` the sheet metal rules feature (the one with a `Thickness` dimension)?
pub fn is_rules(f: &SwFeature) -> bool {
    f.kind == "Sheet-Metal" && f.dims.iter().any(|d| d.name == "Thickness" && d.value.is_some())
}

/// The features that carry no geometry of a sheet metal part: the state entry after each operation, the flat pattern,
/// the cut list and the bend lines.
pub fn is_inert(f: &SwFeature) -> bool {
    matches!(f.kind.as_str(), "Flat-Pattern" | "Cut list" | "Cut-List-Folder") || (f.kind == "Sheet-Metal" && !is_rules(f))
}

pub fn is_base_flange(f: &SwFeature) -> bool {
    f.kind == "Base-Flange"
}

/// A raw (unscaled) dimension by name.
fn raw(f: &SwFeature, name: &str) -> Option<f64> {
    f.dims.iter().find(|d| d.name == name).and_then(|d| d.value)
}

/// The rule's name in our document.
fn rule_name(f: &SwFeature) -> String {
    let s: String = f.name.chars().map(|c| if c.is_alphanumeric() || c == '_' || c == '-' { c } else { '_' }).collect();
    if s.is_empty() {
        "Sheet".to_string()
    } else {
        s
    }
}

/// `Sheet-Metal` -> `sheetRules` (thickness and bend radius in the document's length unit, the k-factor as stored).
pub fn rules(cx: &mut Cx, f: &SwFeature) -> Result<String, String> {
    let t = raw(f, "Thickness").ok_or("no Thickness dimension")? * cx.unit_mm;
    if !(t > 1e-9) {
        return Err(format!("thickness {t}"));
    }
    let r = raw(f, "D1").map(|v| v * cx.unit_mm);
    let k = raw(f, "D2").filter(|k| (0.0..=1.0).contains(k));
    let id = cx.tree.ids.next();
    let mut o = Obj::new().put("type", "sheetRules").put("id", id.as_str()).put("name", rule_name(f).as_str()).put("thickness", n(t));
    if let Some(r) = r {
        o = o.put("bendRadius", n(r));
    }
    if let Some(k) = k {
        o = o.put("kFactor", n(k));
    }
    cx.tree.push(o.build());
    Ok(format!(
        "sheetRules: thickness {t}{}{}",
        r.map_or(String::new(), |r| format!(", bend radius {r}")),
        k.map_or(", k-factor not stored (default)".to_string(), |k| format!(", k-factor {k}"))
    ))
}

/// The last `sheetRules` feature emitted: (name, thickness).
fn last_rule(cx: &Cx) -> Option<(String, f64)> {
    cx.tree.feats.iter().rev().find(|j| j.get_str("type") == Some("sheetRules")).map(|j| (j.get_str("name").unwrap_or("").to_string(), j.get("thickness").and_then(|v| v.as_f64()).unwrap_or(0.0)))
}

/// `Base-Flange` -> `baseFlange` on the last rules.
pub fn base_flange(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Result<String, String> {
    let (rule, t) = last_rule(cx).ok_or("no Sheet-Metal rules before the base flange")?;
    let first = cx.tree.feats.len();
    // (the outline as the later corner rounds and rim cuts left it first, else the plain reading)
    profile::OUTER_ONLY.with(|x| x.set(true));
    let (ids, notes, names) = (cx.tree.ids.0, cx.notes.len(), cx.names.clone());
    profile::CAP_FIRST.with(|x| x.set(true));
    let mut note = profile::extrude(cx, f, faces, false);
    profile::CAP_FIRST.with(|x| x.set(false));
    if note.is_err() {
        cx.tree.feats.truncate(first);
        cx.tree.ids.0 = ids;
        cx.notes.truncate(notes);
        cx.names = names;
        note = profile::extrude(cx, f, faces, false);
    }
    profile::OUTER_ONLY.with(|x| x.set(false));
    let note = note?;
    // the extrude it emitted is the last feature; a base flange is a plain blind boss of the sheet thickness
    let Some(last) = cx.tree.feats.last().cloned() else { return Err("no feature emitted".into()) };
    let bad = |cx: &mut Cx, why: String| -> Result<String, String> {
        cx.tree.feats.truncate(first);
        Err(why)
    };
    if last.get_str("type") != Some("extrude") || last.get_str("op") != Some("add") || last.get_str("end") != Some("blind") {
        return bad(cx, "the base flange did not read back as a plain boss".into());
    }
    let depth = last.get("depth").and_then(|v| v.as_f64()).unwrap_or(0.0);
    if (depth - t).abs() > 1e-6 * (1.0 + t) {
        return bad(cx, format!("the plate is {depth} thick, the rule's thickness is {t}"));
    }
    let id = last.get_str("id").unwrap_or("").to_string();
    let sketch = last.get_str("sketch").unwrap_or("").to_string();
    let mut o = Obj::new().put("type", "baseFlange").put("id", id.as_str()).put("sketch", sketch.as_str()).put("rule", rule.as_str());
    if last.get("flip").and_then(|v| v.as_bool()) == Some(true) {
        o = o.put("flip", true);
    }
    let k = cx.tree.feats.len() - 1;
    cx.tree.feats[k] = o.build();
    let caps: HashSet<FaceId> = faces.iter().copied().filter(|&g| cx.is_cap(g, f.id)).collect();
    let normal = caps.iter().find_map(|&g| match cx.m.kind(g) {
        Kind::Plane { n, .. } => Some(*n),
        _ => None,
    });
    let (mut outer, mut inner) = (HashSet::new(), HashSet::new());
    for &g in &caps {
        let b = cx.m.b;
        let fc = b.face(g);
        let area = |l: ck_topo::body::LoopId| -> f64 {
            let pts: Vec<ck_geom::Vec3> = b.lp(l).coedges.iter().map(|&c| b.vertex(b.coedge_start(c)).p).collect();
            let c0 = pts.first().copied().unwrap_or_default();
            let mut a = ck_geom::Vec3::default();
            for i in 1..pts.len().saturating_sub(1) {
                a = a + (pts[i] - c0).cross(pts[i + 1] - c0);
            }
            a.len()
        };
        let big = fc.loops.iter().copied().max_by(|&x, &y| area(x).total_cmp(&area(y)));
        for &l in &fc.loops {
            let set = if Some(l) == big { &mut outer } else { &mut inner };
            set.extend(b.lp(l).coedges.iter().map(|&c| b.coedge(c).edge));
        }
    }
    PLATE.with(|p| *p.borrow_mut() = normal.map(|normal| Plate { normal, caps, outer, inner }));
    Ok(format!("baseFlange of the sheet thickness {t}; {note}"))
}

/// A `Fillet` whose faces are all round corners of the plate (cylinders about the plate's normal, each meeting a cap of
/// the base flange): the base flange's outline was read from the caps as the file has them, corner rounds included, so
/// the round is already part of the sketch and the step adds no feature of its own.
pub fn fold_round(cx: &Cx, faces: &[FaceId], features: &[SwFeature]) -> Option<String> {
    PLATE.with(|p| p.borrow().clone())?;
    if faces.is_empty() {
        return None;
    }
    // a cap of the base flange or of an edge flange (by the file's tags, so it does not matter which of the two steps comes first
    // in the list: a corner round of a leg may carry a lower id than the flange it rounds)
    let is_plate_cap = |h: FaceId| cx.tags.get(h.idx()).is_some_and(|v| v.iter().any(|&(id, kind)| kind == crate::KIND_CAP && features.iter().any(|g| g.id == id && (is_base_flange(g) || is_edge_flange(g)))));
    for &g in faces {
        let Kind::Cyl { a, .. } = cx.m.kind(g) else { return None };
        // a cylinder about the normal of a plate (the base or the leg of a flange) that meets a cap of it
        let on_plate = cx.m.edges(g).iter().any(|&e| {
            cx.m.across(e, g).is_some_and(|h| match cx.m.kind(h) {
                Kind::Plane { n, .. } => a.dot(*n).abs() >= 1.0 - 1e-9 && is_plate_cap(h),
                _ => false,
            })
        });
        if !on_plate {
            return None;
        }
    }
    Some(format!("corner round of the plate ({} face(s)): part of the outline of the base flange or of an edge flange leg", faces.len()))
}

/// Is the face a wall square to the plate (a plane containing the plate's normal, a cylinder about it)?
fn square_wall(cx: &Cx, g: FaceId, nrm: ck_geom::Vec3) -> bool {
    match cx.m.kind(g) {
        Kind::Plane { n, .. } => n.dot(nrm).abs() < 1e-9,
        Kind::Cyl { a, .. } => a.dot(nrm).abs() > 1.0 - 1e-9,
        _ => false,
    }
}

/// A cut square to the plate (`Cut-Extrude` of a sheet metal part, before any flange).
///
/// * a cut into the rim of the plate: its walls are on the outline the base flange sketch has (read from the caps as the
///   file has them), so the cut is already part of that sketch and adds no feature;
/// * a cut inside the plate (a hole, a slot): the cut's sketch, as `sheetCut`.
///
/// `None` when the part has no plate yet or the cut's faces are not square to it.
pub fn cut(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Option<Result<String, String>> {
    if let Some(note) = rim_cut(cx, faces) {
        return Some(Ok(note));
    }
    let plate = PLATE.with(|p| p.borrow().clone())?;
    if faces.is_empty() || !faces.iter().all(|&g| square_wall(cx, g, plate.normal) || plate.caps.contains(&g)) {
        return None;
    }
    let first = cx.tree.feats.len();
    let ids = cx.tree.ids.0;
    let r = profile::extrude(cx, f, faces, true);
    let fail = |cx: &mut Cx, why: String| -> Option<Result<String, String>> {
        cx.tree.feats.truncate(first);
        cx.tree.ids.0 = ids;
        Some(Err(why))
    };
    let note = match r {
        Ok(n) => n,
        Err(e) => return fail(cx, e),
    };
    let Some(last) = cx.tree.feats.last().cloned() else { return fail(cx, "no feature emitted".into()) };
    let t = last_rule(cx).map_or(0.0, |r| r.1);
    let through = last.get_str("end") == Some("through") || last.get("depth").and_then(|v| v.as_f64()).is_some_and(|d| d >= t - 1e-6);
    if last.get_str("type") != Some("extrude") || last.get_str("op") != Some("cut") || !through {
        return fail(cx, "the cut does not read back as a cut through the sheet".into());
    }
    let o = Obj::new().put("type", "sheetCut").put("id", last.get_str("id").unwrap_or("")).put("sketch", last.get_str("sketch").unwrap_or(""));
    let k = cx.tree.feats.len() - 1;
    cx.tree.feats[k] = o.build();
    Some(Ok(format!("sheetCut through the sheet; {note}")))
}

/// A cut square to the plate whose walls are all on the rim of the plate (the outline the base flange has already holds it):
/// no feature of its own. `None` for any other cut.
pub fn rim_cut(cx: &Cx, faces: &[FaceId]) -> Option<String> {
    let plate = PLATE.with(|p| p.borrow().clone())?;
    if faces.is_empty() || !faces.iter().all(|&g| square_wall(cx, g, plate.normal) || plate.caps.contains(&g)) {
        return None;
    }
    let walls: Vec<FaceId> = faces.iter().copied().filter(|g| !plate.caps.contains(g)).collect();
    let mut on_outer = false;
    let mut on_inner = false;
    for &g in &walls {
        for e in cx.m.edges(g) {
            on_outer |= plate.outer.contains(&e);
            on_inner |= plate.inner.contains(&e);
        }
    }
    (on_outer && !on_inner).then(|| format!("cut into the plate's rim ({} wall(s)): part of the base flange outline", walls.len()))
}

/// A round hole (or several) through the plate square to it: `sheetCut` of its sketch. `None` unless every wall is a cylinder about
/// the plate's normal.
pub fn round_hole_cut(cx: &mut Cx, f: &SwFeature, faces: &[FaceId]) -> Option<Result<String, String>> {
    let plate = PLATE.with(|p| p.borrow().clone())?;
    let walls: Vec<FaceId> = faces.iter().copied().filter(|g| !plate.caps.contains(g)).collect();
    if walls.is_empty() || !walls.iter().all(|&g| matches!(cx.m.kind(g), Kind::Cyl { a, .. } if a.dot(plate.normal).abs() > 1.0 - 1e-9)) {
        return None;
    }
    cut(cx, f, faces)
}

/// Why a sheet metal feature is not read back (the step stays unsupported: the editable steps before it and the file's
/// own body as the remainder, `prefix.rs`).
pub fn unsupported_note(kind: &str) -> Option<String> {
    let why = match kind {
        "Edge-Flange" | "EdgeFlange" => "sheet metal edge flange: not read back",
        "Miter-Flange" | "MiterFlange" | "Hem" | "Jog" | "Sketched-Bend" | "SketchedBend" | "Lofted-Bend" | "Closed-Corner" | "Break-Corner" | "Bend" | "Forming-Tool" | "Unfold" | "Fold" | "Flatten-Bends" | "No-Bends" => {
            return Some(format!("sheet metal feature {kind}: not read back"));
        }
        _ => return None,
    };
    Some(why.to_string())
}

/// Is `f` the edge flange of a sheet metal part?
pub fn is_edge_flange(f: &SwFeature) -> bool {
    f.kind == "Edge-Flange"
}

/// The `EdgeBend` feature after an edge flange holds its bend faces (two cylinders and the two end planes): no step of its own.
pub fn is_edge_bend(f: &SwFeature) -> bool {
    f.kind == "EdgeBend"
}

type V3 = ck_geom::Vec3;

/// A plane (unit normal `n`, offset `d`: `n . p = d`) of the cap faces tagged `feat`, as (face, n, d).
fn cap_planes(cx: &Cx, faces: &[FaceId], feat: i64) -> Vec<(FaceId, V3, f64)> {
    faces
        .iter()
        .filter(|&&g| cx.is_cap(g, feat))
        .filter_map(|&g| match cx.m.kind(g) {
            Kind::Plane { n, d } => Some((g, *n, *d)),
            _ => None,
        })
        .collect()
}

/// The tangent lines of an inner bend cylinder (its straight boundary edges): the one on the leg (the neighbour across it is a
/// face of the flange) and the one on the parent, each as (point, direction from the cylinder's axis, perpendicular to it).
fn bend_lines(cx: &Cx, cyl: FaceId, flange: i64, c: V3, a: V3) -> Result<((V3, V3), (V3, V3)), String> {
    let b = cx.m.b;
    let mut leg = None;
    let mut parent = None;
    for e in cx.m.edges(cyl) {
        if !matches!(b.edge(e).curve, ck_geom::curve::Curve::Line { .. }) {
            continue;
        }
        let p = cx.m.mid(e);
        let rad = (p - c) - a * (p - c).dot(a);
        if rad.len() < 1e-9 {
            continue;
        }
        let entry = (p, rad.normalize());
        let on_leg = cx.m.across(e, cyl).is_some_and(|h| cx.tags.get(h.idx()).is_some_and(|v| v.iter().any(|t| t.0 == flange)));
        if on_leg {
            if leg.is_some() {
                return Err("the bend meets the leg along more than one edge".into());
            }
            leg = Some(entry);
        } else {
            if parent.is_some() {
                return Err("the bend meets its parent along more than one edge".into());
            }
            parent = Some(entry);
        }
    }
    match (parent, leg) {
        (Some(p), Some(l)) => Ok((p, l)),
        _ => Err("the bend cylinder has no tangent line on the parent and on the leg".into()),
    }
}

/// `Edge-Flange` (+ its `EdgeBend`) -> `edgeFlange`: a flange of any width and outline on an edge of the sheet.
///
/// From the bend (the `EdgeBend` feature's two cylinders about one axis `a` through `c`, radii `r` < `r + t`):
/// * the angle the leg turns through is the angle between the radius vectors of the inner cylinder's two tangent lines;
/// * the flange turns towards the cylinder's axis (`s`, from the parent plate to the axis), which tells which side of the
///   parent it bends to;
/// * the bend starts on the parent's mid-plane at `c - d_p (r + t/2)`; its extent along the axis is the span the flange covers
///   along the parent edge (`from` / `to`, the position of the pick on the parent).
/// The leg is the outline of the flange's inner cap face (the face tangent to the inner cylinder), as the `profile`: `v` along
/// the axis from the start of the span, `h` along the leg from the outer virtual sharp. Everything is read from the final
/// body, so later round corners and cuts into the rim of the leg are part of the outline (`fold_round`).
pub fn edge_flange(cx: &mut Cx, f: &SwFeature, faces: &[FaceId], features: &[SwFeature]) -> Result<String, String> {
    let (rule, t) = last_rule(cx).ok_or("no Sheet-Metal rules before the flange")?;
    if PLATE.with(|p| p.borrow().is_none()) {
        return Err("no base flange before the edge flange".into());
    }
    let pos = features.iter().position(|g| g.id == f.id).ok_or("feature not in the list")?;
    let bend_f = features[pos + 1..].iter().take(3).find(|g| is_edge_bend(g)).ok_or("the edge flange has no EdgeBend feature after it")?;
    let bend: Vec<FaceId> = cx.m.faces.iter().copied().filter(|g| cx.tags.get(g.idx()).is_some_and(|v| v.iter().any(|x| x.0 == bend_f.id))).collect();
    let cyls: Vec<(FaceId, V3, V3, f64)> = bend
        .iter()
        .filter_map(|&g| match cx.m.kind(g) {
            Kind::Cyl { o, a, r, .. } => Some((g, *o, *a, *r)),
            _ => None,
        })
        .collect();
    if cyls.len() != 2 {
        return Err(format!("the bend has {} cylinder face(s), a flange bend has two", cyls.len()));
    }
    let (inner, outer) = if cyls[0].3 < cyls[1].3 { (cyls[0], cyls[1]) } else { (cyls[1], cyls[0]) };
    let (_, c, a, r) = inner;
    let a = a.normalize();
    if (outer.2.normalize().cross(a)).len() > 1e-9 || ((outer.1 - c) - a * (outer.1 - c).dot(a)).len() > 1e-6 {
        return Err("the two bend cylinders are not about one axis".into());
    }
    if (outer.3 - r - t).abs() > 1e-4 * (1.0 + t) {
        return Err(format!("the bend's radii {r} and {} are not a thickness {t} apart", outer.3));
    }
    let ((_, d_p), (_, d_l)) = bend_lines(cx, inner.0, f.id, c, a)?;
    let cosang = d_p.dot(d_l).clamp(-1.0, 1.0);
    let theta = cosang.acos();
    if !(theta > 1e-3 && theta < std::f64::consts::PI - 1e-2) {
        return Err(format!("bend angle {:.3} deg out of range", theta.to_degrees()));
    }
    // the inner cap face of the leg: the plane tangent to the inner cylinder along the leg's tangent line
    let caps = cap_planes(cx, faces, f.id);
    let inner_leg: Vec<&(FaceId, V3, f64)> = caps.iter().filter(|(_, n, d)| ((n.dot(c) - d).abs() - r).abs() < 1e-4 * (1.0 + r) && n.cross(d_l).len() < 1e-6).collect();
    if inner_leg.len() != 1 {
        return Err(format!("{} inner cap face(s) of the leg (expected one)", inner_leg.len()));
    }
    let (leg_face, leg_n, _) = *inner_leg[0];
    // the leg runs away from the bend along `u_l`: in the plane of the leg, perpendicular to the axis
    let mut u_l = a.cross(leg_n).normalize();
    let t_l = c + d_l * r;
    let pts = face_pts(cx, leg_face);
    let cen = pts.iter().fold(V3::ZERO, |acc, &p| acc + p) * (1.0 / pts.len().max(1) as f64);
    if (cen - t_l).dot(u_l) < 0.0 {
        u_l = -u_l;
    }
    // the extent of the bend along the axis
    let mut lo = f64::MAX;
    let mut hi = f64::MIN;
    for &g in &bend {
        for p in face_pts(cx, g) {
            lo = lo.min(p.dot(a));
            hi = hi.max(p.dot(a));
        }
    }
    if !(hi - lo > 1e-6) {
        return Err("the bend has no width".into());
    }
    let ca = c.dot(a);
    let mid_line = |s: f64| c + d_p * (r + t * 0.5) + a * (s - ca);
    let s_dir = -d_p;
    let (from, to) = (mid_line(lo), mid_line(hi));
    let pick = (from + to) * 0.5 + s_dir * (t * 0.25);
    // the profile
    let reach = (r + t) * (theta / 2.0).tan();
    let ez = a.cross(u_l);
    let loc = |p: V3| -> [f64; 2] { [p.dot(a) - lo, (p - t_l).dot(u_l) + reach] };
    let b = cx.m.b;
    let area = |l: ck_topo::body::LoopId| -> f64 {
        let q: Vec<[f64; 2]> = b.lp(l).coedges.iter().map(|&c| loc(b.vertex(b.coedge_start(c)).p)).collect();
        (0..q.len()).map(|i| q[i][0] * q[(i + 1) % q.len()][1] - q[(i + 1) % q.len()][0] * q[i][1]).sum::<f64>().abs()
    };
    let outer_loop = b.face(leg_face).loops.iter().copied().max_by(|&x, &y| area(x).total_cmp(&area(y))).ok_or("the leg face has no loop")?;
    let mut segs: Vec<Json> = vec![];
    let pt = |q: [f64; 2]| Json::Arr(vec![n(q[0]), n(q[1])]);
    for &co in &b.lp(outer_loop).coedges {
        let ce = b.coedge(co);
        let (va, vb) = (b.coedge_start(co), b.coedge_end(co));
        let (pa, pb) = (loc(b.vertex(va).p), loc(b.vertex(vb).p));
        match &b.edge(ce.edge).curve {
            ck_geom::curve::Curve::Line { .. } => {
                if (pa[0] - pb[0]).hypot(pa[1] - pb[1]) > 1e-9 {
                    segs.push(Obj::new().put("line", Json::Arr(vec![pt(pa), pt(pb)])).build());
                }
            }
            ck_geom::curve::Curve::Circle { frame, r: cr } => {
                if va == vb || b.vertex(va).p.dist(b.vertex(vb).p) < 1e-9 {
                    return Err("the leg outline has a full circle".into());
                }
                if frame.z.cross(leg_n).len() > 1e-6 || *cr < 1e-6 {
                    return Err("an arc of the leg outline is off the plane of the leg".into());
                }
                let ccw = !ce.rev == (frame.z.dot(ez) > 0.0);
                let arc = Obj::new().put("center", pt(loc(frame.o))).put("from", pt(pa)).put("to", pt(pb)).put("ccw", ccw).build();
                segs.push(Obj::new().put("arc", arc).build());
            }
            _ => return Err("the leg outline has a free curve".into()),
        }
    }
    if segs.len() < 3 {
        return Err("the leg outline has fewer than three edges".into());
    }
    // holes of the flange's own sketch (the other loops of the cap): circles, or polygons of lines
    let mut holes: Vec<Json> = vec![];
    for &l in b.face(leg_face).loops.iter().filter(|&&l| l != outer_loop) {
        let cos = &b.lp(l).coedges;
        if cos.len() == 1 {
            if let ck_geom::curve::Curve::Circle { frame, r: cr } = &b.edge(b.coedge(cos[0]).edge).curve {
                if frame.z.cross(leg_n).len() < 1e-6 {
                    let circle = Obj::new().put("center", pt(loc(frame.o))).put("radius", n(*cr)).build();
                    holes.push(Obj::new().put("circle", circle).build());
                    continue;
                }
            }
            return Err("a hole of the leg is not a plain circle".into());
        }
        let mut hs: Vec<Json> = vec![];
        for &co in cos {
            if !matches!(b.edge(b.coedge(co).edge).curve, ck_geom::curve::Curve::Line { .. }) {
                return Err("a hole of the leg has curved sides other than a circle (not read back exactly)".into());
            }
            let (pa, pb) = (loc(b.vertex(b.coedge_start(co)).p), loc(b.vertex(b.coedge_end(co)).p));
            hs.push(Obj::new().put("line", Json::Arr(vec![pt(pa), pt(pb)])).build());
        }
        holes.push(Json::Arr(hs));
    }
    let id = cx.tree.ids.next();
    let world = |p: V3| ck_recog::emit::vec3(p);
    let edge = Obj::new().put("at", world(pick)).put("from", world(from)).put("to", world(to)).build();
    let o = Obj::new()
        .put("type", "edgeFlange")
        .put("id", id.as_str())
        .put("edges", Json::Arr(vec![edge]))
        .put("angle", n(theta.to_degrees()))
        .put("radius", n(r))
        .put("position", "outside")
        .put("profile", Json::Arr(segs.clone()));
    let o = if holes.is_empty() { o } else { o.put("holes", Json::Arr(holes.clone())) };
    cx.tree.push(o.build());
    Ok(format!("edgeFlange: {:.3} deg bend of radius {r} (rule {rule}), flange width {:.3}, outline of {} edge(s) and {} hole(s) from the inner cap face", theta.to_degrees(), hi - lo, segs.len(), holes.len()))
}

/// Vertices and edge mid points of a face.
fn face_pts(cx: &Cx, g: FaceId) -> Vec<V3> {
    crate::face_points(cx.m, g)
}

/// The order in which the steps are read: the file's, except that a `Fillet` that only rounds corners of the leg of an edge flange
/// whose id is higher than the fillet's comes after that flange (and its `EdgeBend`): the fillet cannot have been made before the
/// leg it rounds, so the ids are not the history's order there. Indices into `features`.
pub fn leg_round_order(m: &ck_recog::model::Model, tags: &[Vec<(i64, i64)>], features: &[SwFeature]) -> Vec<usize> {
    let n = features.len();
    let tagged = |g: FaceId, id: i64| tags.get(g.idx()).is_some_and(|v| v.iter().any(|t| t.0 == id));
    // fillet index -> the position after which it goes
    let mut moved: Vec<(usize, usize)> = vec![];
    for (i, f) in features.iter().enumerate() {
        if f.kind != "Fillet" {
            continue;
        }
        let faces: Vec<FaceId> = m.faces.iter().copied().filter(|&g| tagged(g, f.id)).collect();
        if faces.is_empty() {
            continue;
        }
        let mut flanges: Vec<i64> = vec![];
        let mut all = true;
        for &g in &faces {
            let Kind::Cyl { a, .. } = m.kind(g) else {
                all = false;
                break;
            };
            let owner = m.edges(g).iter().find_map(|&e| {
                let h = m.across(e, g)?;
                let Kind::Plane { n, .. } = m.kind(h) else { return None };
                if a.dot(*n).abs() < 1.0 - 1e-9 {
                    return None;
                }
                tags.get(h.idx())?.iter().find(|&&(id, kind)| kind == crate::KIND_CAP && features.iter().any(|g| g.id == id && is_edge_flange(g))).map(|t| t.0)
            });
            match owner {
                Some(id) => flanges.push(id),
                None => {
                    all = false;
                    break;
                }
            }
        }
        if !all {
            continue;
        }
        let mut after: Option<usize> = None;
        for id in flanges {
            if id <= f.id {
                continue;
            }
            let Some(fi) = features.iter().position(|g| g.id == id) else { continue };
            let end = features[fi + 1..].iter().take(3).position(|g| is_edge_bend(g)).map_or(fi, |k| fi + 1 + k);
            after = Some(after.map_or(end, |a| a.max(end)));
        }
        if let Some(a) = after {
            if a > i {
                moved.push((i, a));
            }
        }
    }
    let mut order: Vec<usize> = (0..n).filter(|i| !moved.iter().any(|m| m.0 == *i)).collect();
    for (i, a) in moved {
        let at = order.iter().position(|&x| x == a).map_or(order.len(), |p| p + 1);
        // after the other fillets already put behind the same step
        let mut at = at;
        while at < order.len() && moved_after(&order, at, features) {
            at += 1;
        }
        order.insert(at, i);
    }
    order
}

/// Is the feature at `order[at]` a fillet that was moved (a later insertion keeps their relative order)?
fn moved_after(order: &[usize], at: usize, features: &[SwFeature]) -> bool {
    at > 0 && at < order.len() && order[at] < order[at - 1] && features[order[at]].kind == "Fillet"
}
