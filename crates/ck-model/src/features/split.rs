//! `split` and `lipJoint`: cut a body by a plane into two bodies, and give the two halves a tongue and a groove.
//!
//! ```json
//! {"type":"split","id":"F6","plane":{"kind":"feature","id":"P1"},"names":["Base","Lid"]}
//! {"type":"lipJoint","id":"F7","plane":{"kind":"feature","id":"P1"},"height":1.5,"widthRatio":0.5,"clearance":0.2,
//!  "tongue":"below","names":["Base","Lid"]}
//! ```
//!
//! `split`:
//! * `plane` (required): a standard plane, a reference plane feature (`{kind:"feature", id}`) or a planar face
//!   (`{kind:"face", id}`). `offset` (mm, along the plane normal) moves it; `flip` reverses its normal.
//! * `target` (optional, `{body: name}` or the name): the body to split. Without it the part must hold one body.
//! * `names` (optional): `[below, above]`, the names of the two halves; "below" is the half on the side the plane
//!   normal points away from. Default: the part keeps the target's name and the upper half gets `<name> 2`.
//! * `keep`: `both` (default), `below` or `above`: the other half is dropped.
//! * Made as `combine` op `split` with a half-space tool, so the cut face of each half is a planar face of the
//!   tool feature and every other face keeps its name. A plane that misses the body is `cutNoEffect`.
//!
//! `lipJoint`: a `split` that also makes a tongue on one half and the matching groove on the other, both following
//! the outer boundary of the cut face (the wall of a shelled body) at `inset` from the outer wall.
//! * `height` (required, mm): how far the tongue stands out of its half.
//! * `width` (mm) or `widthRatio` (fraction of the wall thickness, default 0.5): the width of the tongue.
//! * `inset` (mm, default: centred in the wall): distance from the outer wall to the tongue.
//! * `clearance` (mm, default 0.2): the groove is wider by `clearance` on each side and deeper by `clearance`.
//! * `tongue`: `below` (default) or `above`: the half that carries the tongue; the other half gets the groove.
//! * The wall thickness is measured on the cut face (outer boundary to inner boundary, the narrowest place). A cut
//!   face without an inner boundary (a solid body) needs `width` and `inset` in mm.
//! * Each half is made with the `lip` feature (`inset` / `clearance` fields), so a tongue or groove follows the same
//!   rules and errors as there (planar cut face, closed outer boundary).
//! * Errors: `invalidValue {field, detail}` (nothing fits the wall), `cutNoEffect`, `missingReference`, and the
//!   `lip` errors with `step: "tongue" | "groove"`.

use super::{body_box, Brep};
use crate::regen::{codes, FeatureError, RegenState};
use crate::{Feature, GeomRef};
use ck_core::linalg::Frame;
use ck_geom::{Vec2, Vec3};
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId, LumpId};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

fn invalid(field: &str, detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into()))
}

struct Pl {
    o: Vec3,
    n: Vec3,
}

impl Pl {
    fn dist(&self, p: Vec3) -> f64 {
        (p - self.o).dot(self.n)
    }
}

fn plane_of(br: &Brep, state: &RegenState, f: &Feature) -> Result<Pl, FeatureError> {
    let r = f.raw.get("plane").and_then(GeomRef::from_json).ok_or_else(|| invalid("plane", "a plane is a standard plane, a reference plane or a planar face"))?;
    let fr = super::pattern::plane_frame(br, state, &r).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let mut n = fr.z.normalize();
    if crate::sketch::jtruthy(f.raw.get("flip")) {
        n = n * -1.0;
    }
    let offset = state.num(f, "offset", 0.0)?;
    if !offset.is_finite() {
        return Err(invalid("offset", "offset is a finite number"));
    }
    Ok(Pl { o: fr.o + n * offset, n })
}

fn target_name(raw: &Json) -> Option<String> {
    match raw.get("target") {
        Some(Json::Str(s)) if !s.is_empty() => Some(s.clone()),
        Some(t) => t.get_str("body").map(|s| s.to_string()),
        None => None,
    }
}

fn names_of(raw: &Json) -> Result<Option<[String; 2]>, FeatureError> {
    match raw.get("names") {
        None | Some(Json::Null) => Ok(None),
        Some(Json::Arr(a)) if a.len() == 2 => match (a[0].as_str(), a[1].as_str()) {
            (Some(x), Some(y)) if !x.trim().is_empty() && !y.trim().is_empty() && x != y => Ok(Some([x.to_string(), y.to_string()])),
            _ => Err(invalid("names", "names is two different non-empty names: [below, above]")),
        },
        Some(_) => Err(invalid("names", "names is two names: [below, above]")),
    }
}

/// A box on the normal side of the plane, big enough to cover `body` there.
fn half_space(body: &Body, pl: &Pl, fid: &str) -> Result<Body, FeatureError> {
    let (lo, hi) = body_box(body);
    let big = (hi - lo).len() + 1.0;
    let c = (lo + hi) * 0.5;
    let p = c - pl.n * pl.dist(c);
    let ex = if pl.n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (ex - pl.n * ex.dot(pl.n)).normalize();
    let frame = Frame { o: p, x, y: pl.n.cross(x), z: pl.n };
    let q = |a: f64, b: f64| Vec2::new(a * big, b * big);
    let pts = [q(-1.0, -1.0), q(1.0, -1.0), q(1.0, 1.0), q(-1.0, 1.0)];
    let segs: Vec<ProfileSeg> = (0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), pts[i], pts[(i + 1) % 4])).collect();
    let profile = Profile::new(frame, vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]);
    extrude(fid, &profile, ExtrudeOpts::blind(big)).map_err(|e| FeatureError::new(&e))
}

/// Which side of the plane a lump lies on: the mean signed distance of its vertices (negative: below).
fn side(b: &Body, l: LumpId, pl: &Pl) -> f64 {
    let (mut sum, mut n) = (0.0, 0.0);
    for f in ck_topo::mass::lump_faces(b, l) {
        for e in b.face_edges(f) {
            sum += pl.dist(b.vertex(b.edge(e).v0).p);
            n += 1.0;
        }
    }
    if n > 0.0 {
        sum / n
    } else {
        0.0
    }
}

/// `(below, above)` lump of a body made of two lumps.
fn halves(b: &Body, pl: &Pl) -> Option<(LumpId, LumpId)> {
    let mut ls: Vec<(LumpId, f64)> = b.lump_ids().map(|l| (l, side(b, l, pl))).collect();
    let (below, above): (Vec<_>, Vec<_>) = ls.drain(..).partition(|x| x.1 < 0.0);
    // the split leaves the other lumps of the part where they are: the halves are the ones touching the plane
    let touches = |l: LumpId| {
        ck_topo::mass::lump_faces(b, l).into_iter().any(|f| plane_face(b, f, pl))
    };
    let pick = |v: Vec<(LumpId, f64)>| v.into_iter().map(|x| x.0).find(|&l| touches(l));
    Some((pick(below)?, pick(above)?))
}

/// A planar face lying in the plane.
fn plane_face(b: &Body, f: FaceId, pl: &Pl) -> bool {
    if !matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. }) {
        return false;
    }
    if b.planar_normal(f).dot(pl.n).abs() < 1.0 - 1e-9 {
        return false;
    }
    b.face_edges(f).first().is_some_and(|&e| pl.dist(b.vertex(b.edge(e).v0).p).abs() < 1e-6)
}

struct Split {
    body: Body,
    spans: Vec<super::Span>,
    tolerance_used: Option<f64>,
}

fn split_body(br: &Brep, body: &Body, pl: &Pl, f: &Feature) -> Result<Split, FeatureError> {
    let tool = half_space(body, pl, &f.id)?;
    // The target is one lump, cut whole: a hollow body (a closed shell) has its cavity as a second shell of the same
    // lump, which a cut by lump components (`combine`) would leave out.
    let lump = match target_name(&f.raw) {
        Some(n) => {
            let ls = super::bodies::lumps_named(Some(body), &n)?;
            match ls.as_slice() {
                [l] => *l,
                _ => {
                    let l: Vec<LumpId> = ls.clone();
                    return Err(super::bodies::ambiguous(body, &n, &l));
                }
            }
        }
        None if body.num_lumps() == 1 => body.lump_ids().next().ok_or_else(|| FeatureError::new(codes::NOTHING_TO_CUT))?,
        None => {
            return Err(FeatureError::new(codes::INVALID_VALUE)
                .with("field", Json::Str("target".into()))
                .with("detail", Json::Str(format!("the part has {} bodies: name the one to split", body.num_lumps())))
                .with("hint", Json::Str("set target to {\"body\": \"<name>\"}".into())))
        }
    };
    let target = ck_ops::boolean::assemble::rebuild_in_order(body, &ck_topo::mass::lump_faces(body, lump));
    let mut s = split_whole(br, &target, &tool)?;
    // the other lumps stay as they are, ahead of the halves
    let rest: Vec<FaceId> = body.lump_ids().filter(|&l| l != lump).flat_map(|l| ck_topo::mass::lump_faces(body, l)).collect();
    if !rest.is_empty() {
        let mut out = ck_ops::boolean::assemble::rebuild_in_order(body, &rest);
        let by_face: std::collections::HashMap<String, String> =
            body.face_ids().filter(|&x| body.face_lump(x) != lump).filter_map(|x| Some((body.face_id_str(x), body.lump(body.face_lump(x)).name.clone()?))).collect();
        for l in out.lump_ids().collect::<Vec<_>>() {
            let n = ck_topo::mass::lump_faces(&out, l).into_iter().find_map(|x| by_face.get(&out.face_id_str(x)).cloned());
            out.lumps[l.idx()].name = n;
        }
        out.append(&s.body);
        super::renumber_duplicates(&mut out);
        s.body = out;
    }
    Ok(s)
}

/// The single lump of `body` cut by `tool` into the part outside it (below) and inside it (above).
fn split_whole(br: &Brep, body: &Body, tool: &Body) -> Result<Split, FeatureError> {
    use ck_ops::boolean::Op as BOp;
    let mut spans = vec![];
    let mut used: Option<f64> = None;
    let mut run = |op: BOp| -> Result<ck_ops::boolean::BoolOut, FeatureError> {
        let (r, u) = super::fuzzy_boolean(body, tool, op, super::TOL, br.fuzz, br.auto_fuzz);
        if u.is_some() {
            used = u;
        }
        match r {
            Ok(out) => {
                spans.push(super::boolean_span(op, body.num_faces(), tool.num_faces(), out.body.num_faces()));
                match &out.mismatch {
                    Some(m) => Err(super::volume_mismatch(m)),
                    None => Ok(out),
                }
            }
            Err(e) => Err(FeatureError::new(&e)),
        }
    };
    let below = run(BOp::Cut)?;
    let above = run(BOp::Intersect)?;
    if below.no_effect || above.no_effect || below.body.num_faces() == 0 || above.body.num_faces() == 0 {
        return Err(FeatureError::new(super::CUT_NO_EFFECT));
    }
    let mut out = Body::new();
    out.append(&below.body);
    out.append(&above.body);
    super::renumber_duplicates(&mut out);
    Ok(Split { body: out, spans, tolerance_used: used })
}

/// Name the two halves and refuse a name a lump outside the split already has.
fn name_halves(b: &mut Body, pl: &Pl, names: &Option<[String; 2]>, target: Option<&str>, fid: &str) -> Result<(), FeatureError> {
    let Some((below, above)) = halves(b, pl) else { return Ok(()) };
    let (nb, na) = match (names, target) {
        (Some([x, y]), _) => (Some(x.clone()), Some(y.clone())),
        (None, Some(t)) => (Some(t.to_string()), Some(format!("{t} 2"))),
        (None, None) => (b.lump(below).name.clone(), None),
    };
    for l in b.lump_ids().collect::<Vec<_>>() {
        if l == below || l == above {
            continue;
        }
        if let (Some(n), true) = (b.lump(l).name.as_deref(), true) {
            if Some(n) == nb.as_deref() || Some(n) == na.as_deref() {
                return Err(FeatureError::new(super::bodies::BODY_NAME_CONFLICT).with("name", Json::Str(n.into())).with("features", Json::Arr(vec![Json::Str(fid.into())])));
            }
        }
    }
    b.lumps[below.idx()].name = nb;
    b.lumps[above.idx()].name = na;
    Ok(())
}

fn keep_of(raw: &Json) -> Result<&'static str, FeatureError> {
    match raw.get("keep").and_then(|k| k.as_str()) {
        None | Some("both") => Ok("both"),
        Some("below") => Ok("below"),
        Some("above") => Ok("above"),
        Some(_) => Err(invalid("keep", "keep is both, below or above")),
    }
}

fn target_lump_name(body: &Body, f: &Feature) -> Option<String> {
    match target_name(&f.raw) {
        Some(n) => Some(n),
        None => body.lump_ids().next().and_then(|l| body.lump(l).name.clone()).filter(|_| body.num_lumps() == 1),
    }
}

pub fn exec_split(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(body) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let pl = plane_of(br, state, f)?;
    let keep = keep_of(&f.raw)?;
    let names = names_of(&f.raw)?;
    let target = target_lump_name(&body, f);
    let out = split_body(br, &body, &pl, f)?;
    br.spans.extend(out.spans);
    if out.tolerance_used.is_some() {
        br.fuzz_used = out.tolerance_used;
    }
    let mut nb = out.body;
    name_halves(&mut nb, &pl, &names, target.as_deref(), &f.id)?;
    if keep != "both" {
        let Some((below, above)) = halves(&nb, &pl) else { return Err(FeatureError::new(codes::MISSING_REFERENCE)) };
        let drop = if keep == "below" { above } else { below };
        let names: Vec<(LumpId, Option<String>)> = nb.lump_ids().filter(|&l| l != drop).map(|l| (l, nb.lump(l).name.clone())).collect();
        let faces: Vec<FaceId> = names.iter().flat_map(|(l, _)| ck_topo::mass::lump_faces(&nb, *l)).collect();
        let mut kept = ck_ops::boolean::assemble::rebuild_in_order(&nb, &faces);
        // the rebuilt lumps come in the order of the faces handed over
        let order: Vec<Option<String>> = names.into_iter().map(|x| x.1).collect();
        for (l, n) in kept.lump_ids().collect::<Vec<_>>().into_iter().zip(order) {
            kept.lumps[l.idx()].name = n;
        }
        nb = kept;
    }
    super::renumber_duplicates(&mut nb);
    br.body = Some(nb);
    Ok(())
}

/// The cut faces of a lump: planar faces in the plane.
fn rim_faces(b: &Body, l: LumpId, pl: &Pl) -> Vec<FaceId> {
    ck_topo::mass::lump_faces(b, l).into_iter().filter(|&f| plane_face(b, f, pl)).collect()
}

/// Loops of a face, outer first: the loop of the largest area in the plane.
fn loops_outer_first(b: &Body, f: FaceId, pl: &Pl) -> Vec<Vec<ck_topo::body::EdgeId>> {
    let ex = if pl.n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (ex - pl.n * ex.dot(pl.n)).normalize();
    let y = pl.n.cross(x);
    let mut v: Vec<(f64, Vec<ck_topo::body::EdgeId>)> = b
        .face(f)
        .loops
        .iter()
        .filter(|&&l| b.lp(l).alive)
        .map(|&l| {
            let cs = &b.lp(l).coedges;
            let pts: Vec<Vec2> = cs.iter().map(|&c| b.vertex(b.coedge_start(c)).p).map(|p| Vec2::new((p - pl.o).dot(x), (p - pl.o).dot(y))).collect();
            let a: f64 = (0..pts.len()).map(|i| pts[i].cross(pts[(i + 1) % pts.len()])).sum::<f64>().abs();
            (a, cs.iter().map(|&c| b.coedge(c).edge).collect())
        })
        .collect();
    v.sort_by(|p, q| q.0.total_cmp(&p.0));
    v.into_iter().map(|x| x.1).collect()
}

/// The narrowest wall of a cut face: the distance from the outer boundary (edge midpoints) to the inner ones.
fn wall_thickness(b: &Body, faces: &[FaceId], pl: &Pl) -> Option<f64> {
    let mut t: Option<f64> = None;
    for &f in faces {
        let ls = loops_outer_first(b, f, pl);
        if ls.len() < 2 {
            return None;
        }
        for &e in &ls[0] {
            let m = b.edge_mid(e);
            for inner in &ls[1..] {
                for &ie in inner {
                    let ed = b.edge(ie);
                    let d = ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), m).1;
                    t = Some(t.map_or(d, |x: f64| x.min(d)));
                }
            }
        }
    }
    t
}

fn run_lip(br: &mut Brep, state: &mut RegenState, f: &Feature, half: Body, pl: &Pl, kind: &str, p: &Params) -> Result<Body, FeatureError> {
    let rims: Vec<FaceId> = half.face_ids().filter(|&x| plane_face(&half, x, pl)).collect();
    if rims.is_empty() {
        return Err(FeatureError::new(codes::MISSING_REFERENCE));
    }
    let mut faces = vec![];
    let mut edges = vec![];
    for &r in &rims {
        let rn = half.face_id_str(r);
        faces.push(Json::Str(rn.clone()));
        let outer = loops_outer_first(&half, r, pl).into_iter().next().unwrap_or_default();
        for e in outer {
            let other = half.edge_faces(e).into_iter().find(|&x| x != r).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
            let m = half.edge_mid(e);
            edges.push(
                Obj::new()
                    .put("faces", Json::Arr(vec![Json::Str(rn.clone()), Json::Str(half.face_id_str(other))]))
                    .put("at", Json::Arr(vec![Json::Num(m.x), Json::Num(m.y), Json::Num(m.z)]))
                    .build(),
            );
        }
    }
    let mut o = Obj::new()
        .put("type", "lip")
        .put("id", f.id.clone())
        .put("kind", kind)
        .put("faces", Json::Arr(faces))
        .put("edges", Json::Arr(edges))
        .put("width", p.width)
        .put("height", p.height)
        .put("inset", p.inset);
    if kind == "groove" {
        o = o.put("clearance", p.clearance);
    }
    let lf = Feature::from_json(&o.build());
    let saved = br.body.take();
    br.body = Some(half);
    let r = super::lip::exec_lip(br, state, &lf);
    let res = br.body.take();
    br.body = saved;
    match r {
        Ok(()) => res.ok_or_else(|| FeatureError::new(codes::NOTHING_TO_CUT)),
        Err(e) => Err(e.with("step", Json::Str(if kind == "lip" { "tongue" } else { "groove" }.into()))),
    }
}

struct Params {
    height: f64,
    width: f64,
    inset: f64,
    clearance: f64,
}

pub fn exec_lip_joint(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(body) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let pl = plane_of(br, state, f)?;
    let names = names_of(&f.raw)?;
    let target = target_lump_name(&body, f);
    let height = state.num(f, "height", 0.0)?;
    let clearance = state.num(f, "clearance", 0.2)?;
    if !(height > 0.0) || !height.is_finite() {
        return Err(invalid("height", "height is above 0"));
    }
    if !(clearance >= 0.0) || !clearance.is_finite() {
        return Err(invalid("clearance", "clearance is 0 or more"));
    }
    let tongue_below = match f.raw.get("tongue").and_then(|t| t.as_str()) {
        None | Some("below") => true,
        Some("above") => false,
        Some(_) => return Err(invalid("tongue", "tongue is below or above")),
    };
    let out = split_body(br, &body, &pl, f)?;
    br.spans.extend(out.spans);
    if out.tolerance_used.is_some() {
        br.fuzz_used = out.tolerance_used;
    }
    let nb = out.body;
    let Some((below, above)) = halves(&nb, &pl) else { return Err(FeatureError::new(codes::MISSING_REFERENCE)) };
    let extract = |l: LumpId| ck_ops::boolean::assemble::rebuild_in_order(&nb, &ck_topo::mass::lump_faces(&nb, l));
    let (lo, hi) = (extract(below), extract(above));
    // wall thickness from the cut face of the half that carries the tongue
    let (tg, gv) = if tongue_below { (&lo, &hi) } else { (&hi, &lo) };
    let rims: Vec<FaceId> = tg.face_ids().filter(|&x| plane_face(tg, x, &pl)).collect();
    if rims.is_empty() {
        return Err(FeatureError::new(codes::MISSING_REFERENCE));
    }
    let wall = wall_thickness(tg, &rims, &pl);
    let width_mm = state.num(f, "width", f64::NAN)?;
    let ratio = state.num(f, "widthRatio", 0.5)?;
    let inset_mm = state.num(f, "inset", f64::NAN)?;
    let need_wall = || invalid("width", "the cut face has no inner boundary (a solid body): give width and inset in mm");
    let width = if width_mm.is_nan() {
        if !(ratio > 0.0 && ratio < 1.0) {
            return Err(invalid("widthRatio", "widthRatio is between 0 and 1"));
        }
        wall.ok_or_else(need_wall)? * ratio
    } else {
        width_mm
    };
    if !(width > 0.0) || !width.is_finite() {
        return Err(invalid("width", "width is above 0"));
    }
    let inset = if inset_mm.is_nan() { (wall.ok_or_else(need_wall)? - width) / 2.0 } else { inset_mm };
    if !(inset >= 0.0) || !inset.is_finite() {
        return Err(invalid("inset", "inset is 0 or more (a tongue wider than the wall leaves a negative inset)").with("hint", Json::Str("use a narrower width".into())));
    }
    if let Some(t) = wall {
        if inset + width + clearance > t + 1e-9 {
            return Err(FeatureError::new(codes::INVALID_VALUE)
                .with("field", Json::Str("width".into()))
                .with("detail", Json::Str(format!("inset {inset:.3} + width {width:.3} + clearance {clearance:.3} is more than the wall {t:.3}")))
                .with("hint", Json::Str("use a narrower width or a smaller clearance".into())));
        }
    }
    if clearance > inset + 1e-9 && inset > 0.0 {
        return Err(invalid("clearance", "the clearance is larger than the inset: the groove would cut through the outer wall"));
    }
    let p = Params { height, width, inset, clearance };
    let tongue_half = run_lip(br, state, f, tg.clone(), &pl, "lip", &p)?;
    let groove_half = run_lip(br, state, f, gv.clone(), &pl, "groove", &p)?;
    let (new_lo, new_hi) = if tongue_below { (tongue_half, groove_half) } else { (groove_half, tongue_half) };
    // the lumps the split did not touch stay as they are
    let rest: Vec<FaceId> = nb.lump_ids().filter(|&l| l != below && l != above).flat_map(|l| ck_topo::mass::lump_faces(&nb, l)).collect();
    let mut out = if rest.is_empty() { Body::new() } else { ck_ops::boolean::assemble::rebuild_in_order(&nb, &rest) };
    if !rest.is_empty() {
        let by_face: std::collections::HashMap<String, String> =
            nb.face_ids().filter(|&x| nb.face_lump(x) != below && nb.face_lump(x) != above).filter_map(|x| Some((nb.face_id_str(x), nb.lump(nb.face_lump(x)).name.clone()?))).collect();
        for l in out.lump_ids().collect::<Vec<_>>() {
            let n = ck_topo::mass::lump_faces(&out, l).into_iter().find_map(|x| by_face.get(&out.face_id_str(x)).cloned());
            out.lumps[l.idx()].name = n;
        }
    }
    out.append(&new_lo);
    out.append(&new_hi);
    super::renumber_duplicates(&mut out);
    name_halves(&mut out, &pl, &names, target.as_deref(), &f.id)?;
    br.body = Some(out);
    Ok(())
}
