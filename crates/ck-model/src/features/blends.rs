//! Fillet (and later chamfer) features -> ck-ops blends.
//!
//! Pinned by oracle probes (bracket F11/F12):
//! * trace child span `"F11: Fillet(5.0, Circular) of 4 edges"` (`1 edge`
//!   singular; radius printed with Rust `{:?}` float formatting);
//! * edge selections `{faces:[a,b], at:[x,y,z]}` resolve to the edge between
//!   the two named faces nearest `at` (ck-ops `blend::build::find_edge`);
//! * errors `invalidValue` (radius <= 0), `missingEdge`, `blendOverflow`
//!   (`params.detail`), body unchanged.

use super::{Brep, ChamferType, FeatureKind, Span};
use crate::regen::{FeatureError, RegenState};
use crate::Feature;
use ck_geom::Vec3;
use ck_json::Json;

/// `Fillet(<r>, Circular) of <n> edge(s)`.
pub fn fillet_label(id: &str, r: f64, n: usize) -> String {
    format!("{id}: Fillet({r:?}, Circular) of {n} edge{}", if n == 1 { "" } else { "s" })
}

pub fn blend_error(e: ck_ops::blend::BlendError) -> FeatureError {
    let mut fe = FeatureError::new(&e.code);
    if let Some(d) = e.detail {
        fe = fe.with("detail", Json::Str(d));
    }
    for (k, v) in e.params {
        fe = fe.with(&k, Json::Str(v));
    }
    fe
}

/// `filletType: "fullRound"` (`side1`, `center`, `side2` faces): the ball tangent to all three planar
/// faces rolls between the sides and the center face vanishes. Built as the two circular fillets of the
/// edges `side1 | center` and `side2 | center` whose radius is the ball's. Oracle (box probes):
/// 5 faces / 9 edges, volume 6283.18531 for sides 20 apart on a 10 high box (= two r = 10 fillets,
/// ours merges the two cylinder halves into one face: 4 / 8); a missing face is `missingReference`; the
/// same face twice, or a center face not adjacent to both sides: `fullRoundUnsupported`; a ball that does
/// not fit: `blendFailed {detail: "<id>.B0: spring ends out of order"}`.
fn exec_full_round(br: &mut Brep, f: &Feature) -> Result<(), FeatureError> {
    use ck_geom::surface::Surface;
    let miss = || FeatureError::new("missingReference");
    let unsup = || FeatureError::new("fullRoundUnsupported");
    let name = |k: &str| f.raw.get(k).and_then(|v| v.as_str()).map(str::to_string).ok_or_else(miss);
    let (s1, c, s2) = (name("side1")?, name("center")?, name("side2")?);
    let body = br.body.as_ref().ok_or_else(miss)?;
    let fid = |n: &str| body.find_face(n).ok_or_else(miss);
    let (f1, fc, f2) = (fid(&s1)?, fid(&c)?, fid(&s2)?);
    if f1 == f2 || f1 == fc || f2 == fc {
        return Err(unsup());
    }
    for x in [f1, fc, f2] {
        if !matches!(body.face(x).surface, Surface::Plane { .. }) {
            return Err(unsup());
        }
    }
    // the two edges the ball rolls over
    let shared = |a: ck_topo::body::FaceId| -> Result<ck_topo::body::EdgeId, FeatureError> {
        body.edge_ids().find(|&e| matches!(body.edge(e).curve, ck_geom::curve::Curve::Line { .. }) && {
            let fs = body.edge_faces(e);
            fs.len() == 2 && fs.contains(&a) && fs.contains(&fc)
        }).ok_or_else(unsup)
    };
    let (e1, e2) = (shared(f1)?, shared(f2)?);
    // radius: the ball centre c has distance R inside all three planes: (c - o_i) . n_i = -R
    let dir = {
        let ed = body.edge(e1);
        (body.vertex(ed.v1).p - body.vertex(ed.v0).p).normalize()
    };
    let plane_of = |x: ck_topo::body::FaceId| -> (Vec3, Vec3) {
        let Surface::Plane { frame } = &body.face(x).surface else { unreachable!() };
        (frame.o, body.planar_normal(x))
    };
    let rows: Vec<[f64; 4]> = [f1, fc, f2]
        .iter()
        .map(|&x| {
            let (o, n) = plane_of(x);
            [n.x, n.y, n.z, n.dot(o)]
        })
        .collect();
    // unknowns: centre c (3, the component along the edge is free: fix it at 0) and R:
    //   n_i . c + R = n_i . o_i   (i = 1..3); the edge direction gets a fourth equation d . c = 0
    let mut a = [[0.0f64; 5]; 4];
    for (i, r) in rows.iter().enumerate() {
        a[i] = [r[0], r[1], r[2], 1.0, r[3]];
    }
    a[3] = [dir.x, dir.y, dir.z, 0.0, 0.0];
    for col in 0..4 {
        let piv = (col..4).max_by(|&x, &y| a[x][col].abs().partial_cmp(&a[y][col].abs()).unwrap()).unwrap();
        if a[piv][col].abs() < 1e-12 {
            return Err(unsup());
        }
        a.swap(col, piv);
        for r in col + 1..4 {
            let k = a[r][col] / a[col][col];
            for cc in col..5 {
                a[r][cc] -= k * a[col][cc];
            }
        }
    }
    let mut x = [0.0f64; 4];
    for r in (0..4).rev() {
        let mut v = a[r][4];
        for cc in r + 1..4 {
            v -= a[r][cc] * x[cc];
        }
        x[r] = v / a[r][r];
    }
    let rad = x[3];
    if !(rad > 1e-9) || !rad.is_finite() {
        return Err(unsup());
    }
    let mid = |e: ck_topo::body::EdgeId| (body.vertex(body.edge(e).v0).p + body.vertex(body.edge(e).v1).p) * 0.5;
    let sel = vec![(s1.clone(), c.clone(), mid(e1)), (s2.clone(), c.clone(), mid(e2))];
    br.spans.push(Span::leaf(fillet_label(&f.id, rad, 2)));
    let out = ck_ops::blend::build::fillet(body, &sel, rad, &f.id, super::TOL).map_err(|e| {
        if e.code == "blendOverflow" {
            FeatureError::new("blendFailed").with("detail", Json::Str(format!("{}.B0: spring ends out of order", f.id)))
        } else {
            blend_error(e)
        }
    })?;
    br.body = Some(out);
    Ok(())
}

/// `filletType: "variable"`: `radii: [near, far]` (the first at the end of each picked edge nearer its
/// pick point; for `chord` the widths), `profile` as for constant fillets, `rhos: [a, b]` for conic.
/// Oracle: radii missing / not two positive numbers -> `invalidValue`; `blendTooLarge` when a contact
/// line leaves its face.
fn exec_variable_fillet(br: &mut Brep, state: &mut RegenState, f: &Feature, edges: &[crate::EdgeRef]) -> Result<(), FeatureError> {
    use ck_ops::blend::build::FilletProfile as P;
    let radii: Vec<f64> = match f.raw.get("radii").and_then(|a| a.as_arr()) {
        Some(a) if a.len() == 2 => a.iter().map(|x| crate::expr::eval_field(Some(x), &state.vars, f64::NAN).unwrap_or(f64::NAN)).collect(),
        _ => return Err(FeatureError::new("invalidValue")),
    };
    if radii.iter().any(|r| !(*r > 0.0) || !r.is_finite()) {
        return Err(FeatureError::new("invalidValue"));
    }
    let profile = fillet_profile(state, f)?;
    let rhos = match f.raw.get("rhos").and_then(|a| a.as_arr()) {
        Some(a) if a.len() == 2 => {
            let v: Vec<f64> = a.iter().map(|x| crate::expr::eval_field(Some(x), &state.vars, 0.5).unwrap_or(0.5)).collect();
            [v[0], v[1]]
        }
        _ => {
            let r = state.num(f, "rho", 0.5)?;
            [r, r]
        }
    };
    if matches!(profile, P::Conic(_)) && rhos.iter().any(|r| !(*r > 0.0 && *r < 1.0)) {
        return Err(FeatureError::new("filletRho"));
    }
    let sel = select_edges(br, edges)?;
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    br.spans.push(Span::leaf(fillet_label(&f.id, radii[0], sel.len())));
    let spec = ck_ops::blend::interp::VarSpec { radii: [radii[0], radii[1]], profile, rhos };
    let out = ck_ops::blend::interp::variable_fillet(body, &sel, spec, &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}

/// `fillet.profile`: `circular` (default), `conic` (`rho` in (0, 1), default 0.5: `filletRho`),
/// `chord` (the radius is the chord width) or `curvature` (`shape` in [0.1, 1], default 0.5:
/// `filletShape`; `g3` is accepted). Anything else answers `unsupported`.
fn fillet_profile(state: &RegenState, f: &Feature) -> Result<ck_ops::blend::build::FilletProfile, FeatureError> {
    use ck_ops::blend::build::FilletProfile as P;
    match f.raw.get("profile").and_then(|p| p.as_str()) {
        None | Some("circular") => Ok(P::Circular),
        Some("chord") => Ok(P::Chord),
        Some("conic") => {
            let rho = state.num(f, "rho", 0.5)?;
            if !(rho > 0.0 && rho < 1.0) {
                return Err(FeatureError::new("filletRho"));
            }
            Ok(P::Conic(rho))
        }
        Some("curvature") => {
            let shape = state.num(f, "shape", 0.5)?;
            if !(shape >= ck_ops::blend::g2::SHAPE_MIN - 1e-12 && shape <= ck_ops::blend::g2::SHAPE_MAX) {
                return Err(FeatureError::new("filletShape"));
            }
            Ok(P::Curvature(shape))
        }
        Some(other) => Err(FeatureError::new("unsupported").with("what", Json::Str(format!("fillet profile {other}")))),
    }
}

/// `Chamfer(<d>, <d>) of <n> edge(s)` (equal-distance chamfer).
pub fn chamfer_label(id: &str, d: f64, n: usize) -> String {
    format!("{id}: Chamfer({d:?}, {d:?}) of {n} edge{}", if n == 1 { "" } else { "s" })
}

/// `chamfer`. Oracle (box probes): `equal` (also the default) is the only
/// supported edge chamfer; `distanceDistance` / `angleDistance` / unknown
/// types fail `unsupported {what:"chamfer <type>"}` before anything else.
pub fn exec_chamfer(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Chamfer { chamfer_type, edges, .. } = &f.kind else { unreachable!() };
    let ty = f.raw.get("chamferType").and_then(|v| v.as_str()).unwrap_or("equal").to_string();
    if matches!(ty.as_str(), "distances" | "angle") {
        return exec_chamfer_legs(br, state, f, edges, &ty);
    }
    match chamfer_type {
        ChamferType::Equal => {}
        ChamferType::Vertex => return exec_vertex_chamfer(br, state, f, edges),
        _ => return Err(FeatureError::new("unsupported").with("what", Json::Str(format!("chamfer {ty}")))),
    }
    let d = state.num(f, "distance", 0.0)?;
    if !(d > 0.0) || !d.is_finite() {
        return Err(FeatureError::new("invalidValue"));
    }
    let sel = select_edges(br, edges)?;
    br.spans.push(Span::leaf(chamfer_label(&f.id, d, sel.len())));
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    let out = ck_ops::blend::build::chamfer(body, &sel, d, &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}

/// `chamferType: "distances"` (`distance` on the first face of each pick, `distance2` on the second)
/// or `"angle"` (`distance` on the first face, `angle` degrees between the chamfer and that face).
/// Oracle (box probes): `distance` 2 / `distance2` 3 cuts 2 on the first face and 3 on the second
/// (volume 7880); `distance` 2, `angle` 30 cuts 2 and 2 tan 30 (7953.81). The first face is the first
/// of the pick's `faces`; picks whose faces the kernel lists the other way round are not supported
/// must all agree (mixed orders are `unsupported`).
fn exec_chamfer_legs(br: &mut Brep, state: &mut RegenState, f: &Feature, edges: &[crate::EdgeRef], ty: &str) -> Result<(), FeatureError> {
    use ck_ops::blend::build::ChamferLeg;
    let d1 = state.num(f, "distance", 0.0)?;
    if !(d1 > 0.0) || !d1.is_finite() {
        return Err(FeatureError::new("invalidValue"));
    }
    let leg = if ty == "distances" {
        let d2 = if f.raw.get("distance2").is_some() { state.num(f, "distance2", d1)? } else { d1 };
        if !(d2 > 0.0) || !d2.is_finite() {
            return Err(FeatureError::new("invalidValue"));
        }
        ChamferLeg::Dist(d2)
    } else {
        let a = state.num(f, "angle", 45.0)?.to_radians();
        if !(a > 0.0 && a < std::f64::consts::PI) {
            return Err(FeatureError::new("invalidValue"));
        }
        ChamferLeg::Angle(a)
    };
    let sel = select_edges(br, edges)?;
    let swapped: Vec<bool> = sel.iter().zip(edges).map(|(s, e)| e.faces.first().map_or(false, |a| *a != s.0)).collect();
    let (d1, leg) = if swapped.iter().all(|x| !x) {
        (d1, leg)
    } else if swapped.iter().all(|x| *x) {
        match leg {
            ChamferLeg::Dist(d2) => (d2, ChamferLeg::Dist(d1)),
            ChamferLeg::Angle(a) | ChamferLeg::AngleRev(a) => (d1, ChamferLeg::AngleRev(a)),
        }
    } else {
        return Err(FeatureError::new("unsupported").with("what", Json::Str(format!("chamfer {ty} with mixed face order"))));
    };
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    br.spans.push(Span::leaf(chamfer_label(&f.id, d1, sel.len())));
    let out = ck_ops::blend::build::chamfer_legs(body, &sel, d1, leg, &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}

fn select_edges(br: &Brep, edges: &[crate::EdgeRef]) -> Result<Vec<(String, String, Vec3)>, FeatureError> {
    // GUESS: no body -> missingEdge (nothing can be selected).
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new("missingEdge")) };
    let mut sel = vec![];
    for e in edges {
        let (Some(a), Some(b)) = (e.faces.first(), e.faces.get(1)) else { return Err(FeatureError::new("missingEdge")) };
        let at = match e.at {
            Some(p) => Vec3::new(p[0], p[1], p[2]),
            // GUESS: without a pick point the first edge between the faces
            None => first_edge_mid(body, a, b).ok_or_else(|| FeatureError::new("missingEdge"))?,
        };
        if ck_ops::blend::build::find_edge(body, a, b, at).is_none() {
            return Err(FeatureError::new("missingEdge"));
        }
        sel.push(forward_first(body, a, b, at));
    }
    if sel.is_empty() {
        return Err(FeatureError::new("missingEdge"));
    }
    Ok(sel)
}

/// `fillet`. Oracle (box probes): any `filletType` string (even
/// `"circular"`) fails `unsupported {what:"fillet <type>"}` first; then
/// `radius <= 0` -> `invalidValue`; then unresolved / no edges ->
/// `missingEdge`; none of these open the `Fillet(..)` span.
pub fn exec_fillet(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Fillet { edges, .. } = &f.kind else { unreachable!() };
    if let Some(t) = f.raw.get("filletType").and_then(|v| v.as_str()) {
        if t == "variable" {
            return exec_variable_fillet(br, state, f, edges);
        }
        if t == "fullRound" {
            return exec_full_round(br, f);
        }
        return Err(FeatureError::new("unsupported").with("what", Json::Str(format!("fillet {t}"))));
    }
    let r = state.num(f, "radius", 0.0)?;
    if !(r > 0.0) || !r.is_finite() {
        return Err(FeatureError::new("invalidValue"));
    }
    let profile = fillet_profile(state, f)?;
    let sel = select_edges(br, edges)?;
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    br.spans.push(Span::leaf(fillet_label(&f.id, r, sel.len())));
    let out = ck_ops::blend::build::blend_edges(body, &sel, ck_ops::blend::build::Spec::Fillet(r, profile), &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}

/// Oracle (box probes, bracket F12): the blend's touched faces list the
/// edge's two faces with the one whose loop runs the edge FORWARD first,
/// whatever the selection order (`["F2.S.e8","F2.S.e9"]` -> e9, e8).
/// ck-ops keeps the selection order, so normalise it here.
fn forward_first(b: &ck_topo::body::Body, fa: &str, fb: &str, at: Vec3) -> (String, String, Vec3) {
    let keep = (fa.to_string(), fb.to_string(), at);
    let Some(e) = ck_ops::blend::build::find_edge(b, fa, fb, at) else { return keep };
    let fwd = b.edge_coedges(e).into_iter().find(|&c| !b.coedge(c).rev).map(|c| b.coedge_face(c));
    match (fwd, b.find_face(fb)) {
        (Some(f), Some(g)) if f == g => (fb.to_string(), fa.to_string(), at),
        _ => keep,
    }
}

fn first_edge_mid(b: &ck_topo::body::Body, fa: &str, fb: &str) -> Option<Vec3> {
    let (f, g) = (b.find_face(fa)?, b.find_face(fb)?);
    b.edge_ids()
        .find(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && fs.contains(&f) && fs.contains(&g)
        })
        .map(|e| b.edge_mid(e))
}

/// `chamferType: "vertex"`: the corner nearest `at` (feature `at`, else the
/// first edge's pick point) is cut by the plane through the points at
/// `distance` / `distance2` / `distance3` (defaults: `distance`) along the
/// vertex's edges in arena order. No `Chamfer(..)` span (oracle trace).
fn exec_vertex_chamfer(br: &mut Brep, state: &mut RegenState, f: &Feature, edges: &[crate::EdgeRef]) -> Result<(), FeatureError> {
    let d1 = state.num(f, "distance", 0.0)?;
    let d2 = if f.raw.get("distance2").is_some() { state.num(f, "distance2", d1)? } else { d1 };
    let d3 = if f.raw.get("distance3").is_some() { state.num(f, "distance3", d1)? } else { d1 };
    let at = f
        .raw
        .get("at")
        .and_then(|a| a.as_arr())
        .and_then(|a| (a.len() == 3).then(|| Vec3::new(a[0].as_f64().unwrap_or(0.0), a[1].as_f64().unwrap_or(0.0), a[2].as_f64().unwrap_or(0.0))))
        .or_else(|| edges.first().and_then(|e| e.at).map(|p| Vec3::new(p[0], p[1], p[2])));
    let Some(at) = at else { return Err(FeatureError::new("missingReference")) };
    if [d1, d2, d3].iter().any(|d| !(*d > 0.0) || !d.is_finite()) {
        return Err(FeatureError::new("invalidValue"));
    }
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new("missingReference")) };
    let out = ck_ops::blend::vertex::chamfer_vertex(body, at, [d1, d2, d3], &f.id, super::TOL).map_err(blend_error)?;
    br.body = Some(out);
    Ok(())
}
