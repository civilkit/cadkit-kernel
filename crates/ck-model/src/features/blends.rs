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

/// A blend refusal; when it is a size one (`blendFailed` / `blendOverflow` / `blendUnsupported`) and the asked radius
/// is positive, search the same builder for the largest radius that builds and name it in `maxSize` (see max_size.rs).
/// A smaller radius that does not build leaves the refusal as it was: no size is offered. Off unless
/// [`super::max_size::set_blend_search`] turned it on.
fn size_refusal(
    e: ck_ops::blend::BlendError,
    asked: f64,
    before: usize,
    mut build: impl FnMut(f64) -> Result<ck_topo::body::Body, ck_ops::blend::BlendError>,
) -> FeatureError {
    let size = matches!(e.code.as_str(), "blendFailed" | "blendOverflow" | "blendUnsupported");
    let mut fe = blend_error(e);
    if size && super::max_size::blend_search_enabled() && asked > 0.0 && asked.is_finite() {
        let ok = |v: f64| build(v).is_ok_and(|b| ck_check::validate(&b, super::TOL).len() <= before);
        if let Some(m) = super::max_size::largest_that_builds(asked, None, ok) {
            fe = super::max_size::with_max_size(fe, "radius", m, "mm");
        }
    }
    fe
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
    // (ours, not in the oracle) one end may be 0: the round closes to the corner. See the retry below.
    if radii.iter().any(|r| !(*r >= 0.0) || !r.is_finite()) || (radii[0] == 0.0 && radii[1] == 0.0) {
        return Err(FeatureError::new("invalidValue")
            .with("field", Json::Str("radii".into()))
            .with("detail", Json::Str("a variable fillet needs radii [near, far], two finite numbers >= 0 that are not both 0".into()))
            .with("hint", Json::Str("give a positive radius at least at one end, e.g. radii [4, 1]".into())));
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
    // a pick may carry its own `radii: [near, far]`; the feature's pair is the default
    let picks = f.raw.get("edges").and_then(|a| a.as_arr());
    let per_pick: Vec<[f64; 2]> = (0..sel.len())
        .map(|i| {
            let own = picks.and_then(|a| a.get(i)).and_then(|p| p.get("radii")).and_then(|r| r.as_arr()).filter(|r| r.len() == 2);
            match own {
                Some(r) => {
                    let g = |x: &Json| crate::expr::eval_field(Some(x), &state.vars, f64::NAN).unwrap_or(f64::NAN);
                    [g(&r[0]), g(&r[1])]
                }
                None => [radii[0], radii[1]],
            }
        })
        .collect();
    // several edges with one constant radius: the constant-radius builder (sphere patches at corners)
    if sel.len() > 1 && profile == P::Circular && per_pick.iter().all(|p| (p[0] - p[1]).abs() < 1e-12 && (p[0] - per_pick[0][0]).abs() < 1e-12) {
        let out = ck_ops::blend::build::blend_edges(body, &sel, ck_ops::blend::build::Spec::Fillet(per_pick[0][0], profile), &f.id, super::TOL).map_err(blend_error)?;
        br.body = Some(out);
        return Ok(());
    }
    // (ours) a radius of 0 at one end, where the round closes to the corner, is not built exactly. Try the smallest
    // positive end radius the builder accepts (a fraction of the larger radius) and say which one was used.
    if per_pick.iter().any(|p| p[0] == 0.0 || p[1] == 0.0) {
        let big = per_pick.iter().flat_map(|p| p.iter().copied()).fold(0.0f64, f64::max);
        let mut last = None;
        let mut tried = vec![];
        for frac in ZERO_END_FRACTIONS {
            let eps = frac * big;
            tried.push(eps);
            let fix = |r: f64| if r == 0.0 { eps } else { r };
            let pp: Vec<[f64; 2]> = per_pick.iter().map(|p| [fix(p[0]), fix(p[1])]).collect();
            let sp = ck_ops::blend::interp::VarSpec { radii: [fix(radii[0]), fix(radii[1])], profile, rhos };
            match ck_ops::blend::interp::variable_fillet_picks(body, &sel, &pp, sp, &f.id, super::TOL) {
                Ok(out) => {
                    corner_warnings(state, &f.id, &out);
                    state.result.warnings.push(crate::regen::RegenError {
                        feature: f.id.clone(),
                        error: FeatureError::new("filletRadiusApproximated")
                            .with("requested", Json::Str("end radius 0".into()))
                            .with("used", Json::Num(eps))
                            .with("detail", Json::Str(format!("a fillet radius of 0 at one end is not built exactly; the end radius {eps} (the smallest the builder accepted) was used")))
                            .with("hint", Json::Str("give a small positive end radius yourself to control the result".into())),
                    });
                    br.body = Some(out);
                    return Ok(());
                }
                Err(e) => last = Some(e),
            }
        }
        let e = blend_error(last.expect("at least one attempt"));
        return Err(e
            .with("detail", Json::Str(format!("a radius of 0 at one end could not be built, nor an end radius of {:?}", tried)))
            .with("hint", Json::Str("use a positive end radius, or a constant radius fillet".into())));
    }
    let out = ck_ops::blend::interp::variable_fillet_picks(body, &sel, &per_pick, spec, &f.id, super::TOL).map_err(blend_error)?;
    corner_warnings(state, &f.id, &out);
    br.body = Some(out);
    Ok(())
}

/// End radii tried for a variable fillet that closes to 0, as fractions of the larger radius, smallest first.
const ZERO_END_FRACTIONS: [f64; 6] = [0.05, 0.1, 0.15, 0.25, 0.4, 0.6];

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
    // `distanceDistance` / `angleDistance` are the long names of `distances` / `angle`
    let ty = match ty.as_str() {
        "distanceDistance" => "distances".to_string(),
        "angleDistance" => "angle".to_string(),
        _ => ty,
    };
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
    let sel = if propagates(f) { propagate_tangent(br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?, sel) } else { sel };
    let _chain = ck_ops::blend::build::TangentChainGuard::new(propagates(f));
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
/// of the pick's `faces`, unless the feature names a `face` (a face id, or `{kind:"face", id}`):
/// then `distance` is measured on that face for every pick (each pick must border it, else
/// `missingEdge`), whatever order the pick lists its faces in. `flip: true` swaps the two legs
/// (distance on the other face; for `angle` the angle is then taken to the other face). The long
/// names `distanceDistance` / `angleDistance` are accepted for `distances` / `angle`. The picks of
/// one feature must agree on which of their faces is the first (mixed orders are `unsupported`;
/// `face` makes a ring of edges around one face agree).
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
    let mut first_face: Option<String> = f.raw.get("face").and_then(|v| v.as_str().map(str::to_string).or_else(|| v.get("id").and_then(|i| i.as_str()).map(str::to_string)));
    let sel = if propagates(f) {
        // the chain shares one face: without `face`, the first face of the first pick is the first face of all
        if first_face.is_none() {
            first_face = edges.first().and_then(|e| e.faces.first().cloned());
        }
        propagate_tangent(br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?, sel)
    } else {
        sel
    };
    let _chain = ck_ops::blend::build::TangentChainGuard::new(propagates(f));
    if let Some(ff) = &first_face {
        // every pick must border the named face (by the family name, `~k` pieces ignored)
        let base = super::lip::base_id(ff);
        if !sel.iter().all(|s| super::lip::base_id(&s.0) == base || super::lip::base_id(&s.1) == base) {
            return Err(FeatureError::new("missingEdge"));
        }
    }
    let flip = f.raw.get("flip").and_then(|v| v.as_bool()).unwrap_or(false);
    if let Some(ff) = &first_face {
        // a named face: each edge decides on its own which of its sides is the first
        let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
        br.spans.push(Span::leaf(chamfer_label(&f.id, d1, sel.len())));
        let out = ck_ops::blend::build::chamfer_legs_on_face(body, &sel, d1, leg, ff, flip, &f.id, super::TOL).map_err(blend_error)?;
        br.body = Some(out);
        return Ok(());
    }
    let swapped: Vec<bool> = sel
        .iter()
        .enumerate()
        .map(|(i, s)| {
            let first = match &first_face {
                Some(ff) => Some(ff.as_str()),
                None => edges.get(i).and_then(|e| e.faces.first()).map(|a| a.as_str()),
            };
            first.map_or(false, |a| super::lip::base_id(a) != super::lip::base_id(&s.0)) ^ flip
        })
        .collect();
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
    use ck_topo::body::EdgeId;
    // GUESS: no body -> missingEdge (nothing can be selected).
    let Some(body) = br.body.as_ref() else { return Err(FeatureError::new("missingEdge")) };
    let missing = || FeatureError::new("missingEdge");
    // pass 1: the recorded name pair, else the candidates between the same face FAMILIES
    // (`~k` suffixes ignored). The `~k` numbers of split faces (inner / outer of a shell, pieces
    // of a blend) come from the order the boolean walks them, which can change when the
    // geometry is edited: the recorded pair may then name two faces that no longer meet.
    enum Pick {
        Exact(String, String, Vec3),
        Family(Vec3, Vec<(EdgeId, f64)>),
    }
    let mut picks = vec![];
    for e in edges {
        let (Some(a), Some(b)) = (e.faces.first(), e.faces.get(1)) else { return Err(missing()) };
        let at = match e.at {
            Some(p) => Vec3::new(p[0], p[1], p[2]),
            // GUESS: without a pick point the first edge between the faces
            None => first_edge_mid(body, a, b).ok_or_else(missing)?,
        };
        if ck_ops::blend::build::find_edge(body, a, b, at).is_some() {
            picks.push(Pick::Exact(a.clone(), b.clone(), at));
        } else {
            let (ba, bb) = (super::lip::base_id(a), super::lip::base_id(b));
            let cands: Vec<(EdgeId, f64)> = body
                .edge_ids()
                .filter(|&e| {
                    let fs = body.edge_faces(e);
                    fs.len() == 2 && {
                        let (x, y) = (super::lip::base_id(&body.face_id_str(fs[0])), super::lip::base_id(&body.face_id_str(fs[1])));
                        (x == ba && y == bb) || (x == bb && y == ba)
                    }
                })
                .map(|e| {
                    let ed = body.edge(e);
                    (e, ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1)
                })
                .collect();
            if cands.is_empty() {
                // no edge between the two families: the pick point may still sit on an edge of the same two
                // features (a wall number that shifted when a boolean merged the halves of a cone)
                let Some(e) = ck_ops::blend::build::find_edge_by_position(body, a, b, at) else { return Err(missing()) };
                let fs = body.edge_faces(e);
                picks.push(Pick::Exact(body.face_id_str(fs[0]), body.face_id_str(fs[1]), at));
                continue;
            }
            picks.push(Pick::Family(at, cands));
        }
    }
    // pass 2: the stale pick point alone cannot tell the outer rim of a shell from the inner one
    // once the part is edited (the rim moved further than the wall is thick). The picks of one
    // feature lie on one chain of edges, so take the connected set of candidate edges that the
    // picks are closest to as a whole and choose within it.
    let mut comp: std::collections::HashMap<EdgeId, EdgeId> = Default::default();
    fn root(c: &mut std::collections::HashMap<EdgeId, EdgeId>, e: EdgeId) -> EdgeId {
        let p = *c.entry(e).or_insert(e);
        if p == e {
            e
        } else {
            let r = root(c, p);
            c.insert(e, r);
            r
        }
    }
    let mut all: Vec<EdgeId> = vec![];
    for p in &picks {
        match p {
            Pick::Exact(a, b, at) => all.extend(ck_ops::blend::build::find_edge(body, a, b, *at)),
            Pick::Family(_, c) => all.extend(c.iter().map(|x| x.0)),
        }
    }
    for (i, &e) in all.iter().enumerate() {
        for &g in &all[i + 1..] {
            let (x, y) = (body.edge(e), body.edge(g));
            if [x.v0, x.v1].iter().any(|v| *v == y.v0 || *v == y.v1) {
                let (r1, r2) = (root(&mut comp, e), root(&mut comp, g));
                comp.insert(r1, r2);
            }
        }
    }
    let mut score: std::collections::BTreeMap<usize, f64> = Default::default();
    for p in &picks {
        if let Pick::Family(_, c) = p {
            let mut best: std::collections::BTreeMap<usize, f64> = Default::default();
            for &(e, d) in c {
                let r = root(&mut comp, e).idx();
                let v = best.entry(r).or_insert(f64::INFINITY);
                *v = v.min(d);
            }
            // components this pick has no candidate in cost it the worst distance
            let worst = best.values().cloned().fold(0.0f64, f64::max);
            for &r in &score.keys().cloned().collect::<Vec<_>>() {
                best.entry(r).or_insert(worst * 2.0 + 1.0);
            }
            for (r, d) in best {
                *score.entry(r).or_insert(0.0) += d;
            }
        }
    }
    let chosen_comp = score.iter().min_by(|a, b| a.1.total_cmp(b.1)).map(|(r, _)| *r);
    let mut sel = vec![];
    for p in picks {
        let (a, b, at) = match p {
            Pick::Exact(a, b, at) => (a, b, at),
            Pick::Family(at, c) => {
                let roots: Vec<usize> = c.iter().map(|x| root(&mut comp, x.0).idx()).collect();
                let best = |only: bool| {
                    c.iter()
                        .zip(&roots)
                        .filter(|(_, r)| !only || chosen_comp.map_or(true, |k| **r == k))
                        .min_by(|x, y| x.0 .1.total_cmp(&y.0 .1))
                        .map(|x| x.0 .0)
                };
                let e = best(true).or_else(|| best(false)).ok_or_else(missing)?;
                let fs = body.edge_faces(e);
                (body.face_id_str(fs[0]), body.face_id_str(fs[1]), at)
            }
        };
        sel.push(forward_first(body, &a, &b, at));
    }
    if sel.is_empty() {
        return Err(missing());
    }
    Ok(sel)
}

/// With several lumps (bodies), a blend acts on the lump that holds the faces its picks name; another lump, a coincident copy
/// for one, takes no part in it (a fillet beside an overlapping lump fails with `blendFailed`). The other lumps are put back
/// unchanged after the blended lump. With one lump, or picks spread over several, `run` sees the whole body.
pub fn on_lump_of_picks(br: &mut Brep, edges: &[crate::EdgeRef], run: impl FnOnce(&mut Brep) -> Result<(), FeatureError>) -> Result<(), FeatureError> {
    let names: Vec<&String> = edges.iter().flat_map(|e| e.faces.iter()).collect();
    match lump_split(br, edges.is_empty(), &names) {
        Some(split) => run_on_split(br, split, run),
        None => run(br),
    }
}

/// The lump holding all of `names` (faces) split from the other lumps, as (that lump, the rest); `None` when the body has one lump,
/// the names are empty or spread over several lumps, an assembly indexes the body, or a name is not a face of it.
fn lump_split(br: &Brep, none: bool, names: &[&String]) -> Option<Split> {
    let body = br.body.as_ref()?;
    // component spans of an assembly index the body; they stay valid only when the body is not rebuilt
    if body.lump_ids().count() < 2 || none || !br.asm.components.is_empty() {
        return None;
    }
    let mut lumps = vec![];
    for n in names {
        lumps.push(body.face_lump(body.find_face(n)?));
    }
    lumps.sort_unstable();
    lumps.dedup();
    if lumps.len() != 1 {
        return None;
    }
    let mine = ck_topo::mass::lump_faces(body, lumps[0]);
    let rest: Vec<_> = body.face_ids().filter(|g| !mine.contains(g)).collect();
    let mut sub = ck_ops::boolean::assemble::rebuild_in_order(body, &mine);
    ck_topo::region::rebuild_lumps(&mut sub);
    let mut others = ck_ops::boolean::assemble::rebuild_in_order(body, &rest);
    ck_topo::region::rebuild_lumps(&mut others);
    let name = body.lump(lumps[0]).name.clone();
    Some((sub, others, name))
}

/// The lump that was split off, the other lumps, and the body name of the lump that was split off.
type Split = (ck_topo::body::Body, ck_topo::body::Body, Option<String>);

fn run_on_split(br: &mut Brep, split: Split, run: impl FnOnce(&mut Brep) -> Result<(), FeatureError>) -> Result<(), FeatureError> {
    let (sub, others, name) = split;
    let whole = br.body.replace(sub);
    match run(br) {
        Ok(()) => {
            let done = br.body.take();
            br.body = Some(others);
            if let Some(mut d) = done {
                // the body keeps its name through the edit
                if let (Some(n), 1) = (name, d.lump_ids().count()) {
                    let l = d.lump_ids().next().unwrap();
                    if d.lump(l).name.is_none() {
                        d.lumps[l.idx()].name = Some(n);
                    }
                }
                br.append_body(&d);
            }
            Ok(())
        }
        Err(e) => {
            br.body = whole;
            Err(e)
        }
    }
}

/// Run `run` on the one lump that holds `face` while the other lumps stay as they are. `None` when that does not apply (see
/// [`lump_split`]): `run` has not been called then.
pub fn on_lump_of_face(br: &mut Brep, face: &str, run: impl FnOnce(&mut Brep) -> Result<(), FeatureError>) -> Option<Result<(), FeatureError>> {
    let name = face.to_string();
    let split = lump_split(br, false, &[&name])?;
    Some(run_on_split(br, split, run))
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
    let sel = if propagates(f) { propagate_tangent(br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?, sel) } else { sel };
    // a pick may carry its own `radius`; the feature's radius is the default
    let own: Vec<Option<f64>> = (0..sel.len())
        .map(|i| {
            f.raw.get("edges").and_then(|a| a.as_arr()).and_then(|a| a.get(i)).and_then(|p| p.get("radius")).map(|x| crate::expr::eval_field(Some(x), &state.vars, f64::NAN).unwrap_or(f64::NAN))
        })
        .collect();
    let radii: Vec<f64> = own.iter().map(|o| o.unwrap_or(r)).collect();
    if own.iter().any(Option::is_some) {
        if radii.iter().any(|x| !(*x > 0.0) || !x.is_finite()) {
            return Err(FeatureError::new("invalidValue"));
        }
        if radii.iter().any(|x| (x - radii[0]).abs() > 1e-12 * (1.0 + x.abs())) {
            if profile != ck_ops::blend::build::FilletProfile::Circular {
                return Err(FeatureError::new("unsupported").with("what", Json::Str("unequal radii with a fillet profile".into())));
            }
            let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
            br.spans.push(Span::leaf(fillet_label(&f.id, radii.iter().cloned().fold(0.0, f64::max), sel.len())));
            // mixed vertices first (a wall radius and a rim radius), else the per-edge builder
            let mixed = if profile == ck_ops::blend::build::FilletProfile::Circular { ck_ops::blend::mixed::fillet_mixed(body, &sel, &radii, &f.id, super::TOL) } else { None };
            let out = match mixed {
                Some(res) => res.map_err(blend_error)?,
                None => ck_ops::blend::build::fillet_unequal(body, &sel, &radii, &f.id, super::TOL).map_err(blend_error)?,
            };
            corner_warnings(state, &f.id, &out);
            br.body = Some(out);
            return Ok(());
        }
    }
    let r = radii.first().copied().unwrap_or(r);
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    br.spans.push(Span::leaf(fillet_label(&f.id, r, sel.len())));
    // two convex edges and a concave one at a vertex: the concave corner is filled first, the rim rolls round it
    if profile == ck_ops::blend::build::FilletProfile::Circular {
        if let Some(res) = ck_ops::blend::mixed::fillet_mixed(body, &sel, &vec![r; sel.len()], &f.id, super::TOL) {
            match res {
                Ok(out) => {
                    br.body = Some(out);
                    return Ok(());
                }
                Err(e) => {
                    let build = |radius: f64| {
                        ck_ops::blend::mixed::fillet_mixed(body, &sel, &vec![radius; sel.len()], &f.id, super::TOL)
                            .unwrap_or_else(|| Err(ck_ops::blend::BlendError::code("blendUnsupported")))
                    };
                    return Err(size_refusal(e, r, ck_check::validate(body, super::TOL).len(), build));
                }
            }
        }
    }
    let build = |radius: f64| {
        let spec = ck_ops::blend::build::Spec::Fillet(radius, profile);
        if propagates(f) {
            ck_ops::blend::build::blend_edges_tangent_chain(body, &sel, spec, &f.id, super::TOL)
        } else {
            ck_ops::blend::build::blend_edges(body, &sel, spec, &f.id, super::TOL)
        }
    };
    let out = build(r).map_err(|e| size_refusal(e, r, ck_check::validate(body, super::TOL).len(), build))?;
    br.body = Some(out);
    Ok(())
}

/// `cornerNotTangent` warning: two blend faces of feature `id` that meet along an edge (the mitre of
/// blends that end at a shared vertex with different radii) with a normal jump of more than half a
/// degree; `deviation` is the largest jump in degrees.
fn corner_warnings(state: &mut RegenState, id: &str, out: &ck_topo::body::Body) {
    let prefix = format!("{id}.B");
    let mut worst = 0.0f64;
    for e in out.edge_ids() {
        let fs = out.edge_faces(e);
        if fs.len() != 2 || !fs.iter().all(|&x| out.face_id_str(x).starts_with(&prefix)) {
            continue;
        }
        let ed = out.edge(e);
        for t in [0.25, 0.5, 0.75] {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * t);
            let n: Vec<Vec3> = fs
                .iter()
                .map(|&x| {
                    let (u, v) = out.face(x).surface.closest_param(p);
                    out.outward_normal(x, u, v).normalize()
                })
                .collect();
            worst = worst.max(n[0].dot(n[1]).clamp(-1.0, 1.0).acos().to_degrees());
        }
    }
    state.result.warnings.retain(|w| !(w.feature == id && w.error.code == "cornerNotTangent"));
    if worst > 0.5 {
        let e = FeatureError::new("cornerNotTangent").with("deviation", Json::Num((worst * 100.0).round() / 100.0));
        state.result.warnings.push(crate::regen::RegenError { feature: id.to_string(), error: e });
    }
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

/// `propagate: true` on a fillet or chamfer: the selection grows along edges that continue the picked
/// ones tangentially (OCCT does this for every selected edge).
fn propagates(f: &Feature) -> bool {
    f.raw.get("propagate").and_then(|v| v.as_bool()).unwrap_or(false)
}

/// Close `sel` under tangent continuation: at a vertex of a selected edge, an unselected edge that
/// shares exactly one face with it and leaves in the same direction (parallel tangents) joins the
/// selection. New picks are `(face a, face b, midpoint)` in the order the edge lists its faces.
fn propagate_tangent(body: &ck_topo::body::Body, sel: Vec<(String, String, Vec3)>) -> Vec<(String, String, Vec3)> {
    use ck_topo::body::EdgeId;
    let mut out = sel.clone();
    let mut chain: Vec<EdgeId> = sel.iter().filter_map(|(a, b, at)| ck_ops::blend::build::find_edge(body, a, b, *at)).collect();
    let tangent_at = |e: EdgeId, v: ck_topo::body::VertexId| -> Option<Vec3> {
        let ed = body.edge(e);
        if ed.v0 == ed.v1 {
            return None;
        }
        let t = if ed.v0 == v { ed.t0 } else { ed.t1 };
        let d = ed.curve.tangent(t);
        (d.len() > 1e-12).then(|| d.normalize())
    };
    let mut i = 0;
    while i < chain.len() {
        let e = chain[i];
        i += 1;
        let ed = body.edge(e);
        let fe = body.edge_faces(e);
        for v in [ed.v0, ed.v1] {
            let Some(t0) = tangent_at(e, v) else { continue };
            let cands: Vec<EdgeId> = body.edge_ids().filter(|&x| x != e && !chain.contains(&x) && (body.edge(x).v0 == v || body.edge(x).v1 == v)).collect();
            for e2 in cands {
                let f2 = body.edge_faces(e2);
                if f2.len() != 2 || f2.iter().filter(|f| fe.contains(f)).count() != 1 {
                    continue;
                }
                let Some(t1) = tangent_at(e2, v) else { continue };
                if t0.cross(t1).len() > 1e-6 {
                    continue;
                }
                let ed2 = body.edge(e2);
                let mid = ed2.curve.eval(0.5 * (ed2.t0 + ed2.t1));
                out.push((body.face_id_str(f2[0]), body.face_id_str(f2[1]), mid));
                chain.push(e2);
            }
        }
    }
    out
}
