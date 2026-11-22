//! Procedural surface features (ck-model/src/features/proc_surface.rs): `sweepSurface` and `blendSurface`. Each makes
//! a sheet (a surface body under the feature id, [`Brep::surfs`]) whose face lies on a ck-geom procedural surface and
//! keeps it as its definition: `Surface::Sweep` (a profile carried along a path) and `Surface::Blend` (a rolling ball
//! between two supports). The face is one patch over the surface's parameter rectangle; its boundary edges are the
//! exact iso lines of the definition (`Curve::Iso`), or the closed form where there is one (the profile at the path
//! start and, moved rigidly, at its end; the path itself, translated, as a rail of a translation sweep; the end arcs of
//! a blend as circles). Each coedge carries the line of its pcurve, so the mass properties, the mesher and the
//! validator read the patch without projecting.
//!
//! The definition is converted to a B-spline within `tolerance` (`Surface::to_nurbs_within`) when the feature runs: a
//! definition that does not convert is refused, and the marker reports the B-spline (`degree`, `count`, `deviation`)
//! the STEP export and the operations that need one form take. The face, being the definition, is what the body codec
//! stores, the regen cache snapshots and the mesher evaluates.
//!
//! `sweepSurface`: `profile` and `path`, each a curve input (a string: an earlier curve feature, else a sketch whose
//! curves form one chain; an object: one curve reference; an array: curve references in chain order). `frame`:
//! `"tangent"` (default; the frame of the path tangent and the `up` direction) or `"translate"`. `caps: true` closes a
//! closed planar profile swept along an open path with plane faces at both ends. Faces `<id>.W`, caps `<id>.C0` (start)
//! and `<id>.C1` (end). Errors: sweepSurfaceNoInput, sweepSurfaceFrame, sweepSurfaceDegenerate, sweepSurfaceFold,
//! sweepSurfaceCaps, sweepSurfaceFailed (and the curve input's missingReference / missingSketch / Gap / NotChain).
//!
//! `blendSurface`: `faces` (the two supports), `radius` (a number, or `[start, end]` for a linear law along the
//! spine), `sides` (`[s1, s2]`, +1 when the ball lies on the side the face's outward normal points to, -1 against it;
//! default: each toward the other face), and the span: `from` / `to` (the spine is the line between them), else the
//! edge the faces share (nearest `at` when there are several). Face `<id>.R`. Errors: blendSurfaceNoInput,
//! blendSurfaceSpan, blendSurfaceSides, blendSurfaceNoContact, blendSurfaceRadius {radius, maxRadius},
//! blendSurfaceFailed, invalidValue, missingReference.

use super::proc_curve::{field_curve, num_field};
use super::project_curve::plane_normal;
use super::surfaces::Slot;
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::procsurf::{RollingBall, SweepLaw, SweepSurface};
use ck_geom::surface::Surface;
use ck_geom::Vec2;
use ck_json::Json;
use ck_ops::boolean::classify::{point_in_face_cached, LoopCache, Where};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, VertexId};

const DEFAULT_GAP: f64 = 1e-5;

fn v3(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(p.x), Json::Num(p.y), Json::Num(p.z)])
}

fn hint(e: FeatureError, h: &str) -> FeatureError {
    e.with("hint", Json::Str(h.into()))
}

fn invalid(field: &str, detail: &str, h: &str) -> FeatureError {
    hint(FeatureError::new(codes::INVALID_VALUE).with("field", Json::Str(field.into())).with("detail", Json::Str(detail.into())), h)
}

fn json_vec(j: &Json) -> Option<Vec3> {
    let a = j.as_arr()?;
    (a.len() == 3).then(|| Some(Vec3::new(a[0].as_f64()?, a[1].as_f64()?, a[2].as_f64()?)))?
}

fn tolerance(state: &RegenState, f: &Feature) -> Result<f64, FeatureError> {
    let tol = num_field(state, f, "tolerance", 0.001)?;
    if !(tol.is_finite() && tol > 0.0) {
        return Err(invalid("tolerance", "tolerance is the largest distance (mm) of the B-spline from the definition, above 0", "leave it out for 0.001 mm"));
    }
    Ok(tol)
}

// ---- the patch ----------------------------------------------------------------------------------------------

/// A boundary curve offered for one side of the patch: `curve` over `[t0, t1]`, running with the surface parameter
/// along that side (its pcurve is the linear map of `[t0, t1]` onto the side). Taken only when it lies on the
/// surface; otherwise the side is the surface's iso line.
struct Offer {
    curve: Curve,
    t0: f64,
    t1: f64,
}

/// The sides of the patch over `[u0, u1] x [v0, v1]`, in loop order: v = v0 (along u), u = u1 (along v), v = v1
/// (along u), u = u0 (along v).
#[derive(Default)]
struct Offers {
    bottom: Option<Offer>,
    right: Option<Offer>,
    top: Option<Offer>,
    left: Option<Offer>,
}

/// The patch as built: the body and its edges (`None` where a side is the seam of a closed direction).
struct Patch {
    body: Body,
    face: FaceId,
    left: EdgeId,
    right: EdgeId,
}

fn size_of(s: &Surface) -> f64 {
    let ((u0, u1), (v0, v1)) = s.domain();
    let mut m = 0.0f64;
    for i in 0..=4 {
        for j in 0..=4 {
            let p = s.eval(u0 + (u1 - u0) * i as f64 / 4.0, v0 + (v1 - v0) * j as f64 / 4.0);
            m = m.max(p.len());
        }
    }
    1.0 + m
}

/// Does the surface close on itself across u (S(u0, v) = S(u1, v)) or across v?
fn closes(s: &Surface, across_u: bool, tol: f64) -> bool {
    let ((u0, u1), (v0, v1)) = s.domain();
    (0..=8).all(|k| {
        let w = k as f64 / 8.0;
        if across_u {
            let v = v0 + (v1 - v0) * w;
            s.eval(u0, v).dist(s.eval(u1, v)) <= tol
        } else {
            let u = u0 + (u1 - u0) * w;
            s.eval(u, v0).dist(s.eval(u, v1)) <= tol
        }
    })
}

/// The edge of one side: the offered curve when it lies on the surface along the side, else the iso line.
/// `fixed_u`: the side is u = `at` (it runs along v over `range`); else v = `at` (along u).
fn side_edge(b: &mut Body, s: &Surface, offer: Option<Offer>, fixed_u: bool, at: f64, range: (f64, f64), va: VertexId, vb: VertexId, tol: f64) -> (EdgeId, Curve2) {
    let uv = |w: f64| if fixed_u { Vec2::new(at, w) } else { Vec2::new(w, at) };
    if let Some(o) = offer {
        let k = (range.1 - range.0) / (o.t1 - o.t0);
        let on = (0..=8).all(|i| {
            let t = o.t0 + (o.t1 - o.t0) * i as f64 / 8.0;
            let q = uv(range.0 + (t - o.t0) * k);
            o.curve.eval(t).dist(s.eval(q.x, q.y)) <= tol
        });
        if on && o.t1 > o.t0 {
            let d = if fixed_u { Vec2::new(0.0, k) } else { Vec2::new(k, 0.0) };
            let pc = Curve2::Line { o: uv(range.0 - o.t0 * k), d };
            return (b.add_edge(o.curve, o.t0, o.t1, va, vb), pc);
        }
    }
    let d = if fixed_u { Vec2::new(0.0, 1.0) } else { Vec2::new(1.0, 0.0) };
    let pc = Curve2::Line { o: uv(0.0), d };
    (b.add_edge(Curve::iso(s.clone(), fixed_u, at), range.0, range.1, va, vb), pc)
}

/// One face on `s` over its whole domain, named `<feature>.<role>`, its sense the natural normal; a direction in
/// which the surface closes on itself gets a seam edge (two coedges of the face) instead of two sides.
fn patch(feature: &str, role: &str, s: &Surface, mut offers: Offers) -> Patch {
    let ((u0, u1), (v0, v1)) = s.domain();
    let tol = 1e-9 * size_of(s);
    let (cu, cv) = (closes(s, true, 1e-7 * size_of(s)), closes(s, false, 1e-7 * size_of(s)));
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    let c00 = b.add_vertex(s.eval(u0, v0));
    let c10 = if cu { c00 } else { b.add_vertex(s.eval(u1, v0)) };
    let c01 = if cv { c00 } else { b.add_vertex(s.eval(u0, v1)) };
    let c11 = match (cu, cv) {
        (true, _) => c01,
        (_, true) => c10,
        _ => b.add_vertex(s.eval(u1, v1)),
    };
    let (bottom, pb) = side_edge(&mut b, s, offers.bottom.take(), false, v0, (u0, u1), c00, c10, tol);
    let (right, pr) = side_edge(&mut b, s, offers.right.take(), true, u1, (v0, v1), c10, c11, tol);
    let (top, pt) = if cv {
        (bottom, shift(&pb, Vec2::new(0.0, v1 - v0)))
    } else {
        side_edge(&mut b, s, offers.top.take(), false, v1, (u0, u1), c01, c11, tol)
    };
    let (left, pl) = if cu {
        (right, shift(&pr, Vec2::new(u0 - u1, 0.0)))
    } else {
        side_edge(&mut b, s, offers.left.take(), true, u0, (v0, v1), c00, c01, tol)
    };
    let face = b.add_face(shell, s.clone(), true, FaceName::other(feature, role));
    let lp = b.add_loop(face);
    b.add_coedge(lp, bottom, false, Some(pb));
    b.add_coedge(lp, right, false, Some(pr));
    b.add_coedge(lp, top, true, Some(pt));
    b.add_coedge(lp, left, true, Some(pl));
    Patch { body: b, face, left, right }
}

/// The pcurve of the second use of a seam: the first use's line moved across the closed direction.
fn shift(first: &Curve2, by: Vec2) -> Curve2 {
    match first {
        Curve2::Line { o, d } => Curve2::Line { o: *o + by, d: *d },
        c => c.clone(),
    }
}

/// Is the patch closed in u and in v (seams instead of sides)?
fn seams(p: &Patch) -> (bool, bool) {
    let b = &p.body;
    let lp = b.face(p.face).loops[0];
    let cs = &b.lp(lp).coedges;
    let e = |i: usize| b.coedge(cs[i]).edge;
    (e(1) == e(3), e(0) == e(2))
}

fn face_area(b: &Body, f: FaceId) -> f64 {
    let sg = if b.face(f).sense { 1.0 } else { -1.0 };
    sg * ck_topo::mass::face_integral::<1>(b, f, 2, &|_, n| [n.len()])[0]
}

/// Certify the definition and publish the sheet and its marker.
fn publish(br: &mut Brep, state: &mut RegenState, f: &Feature, body: Body, main: FaceId, prefix: &str, tol: f64, mut extra: Vec<(String, Json)>) -> Result<(), FeatureError> {
    let s = body.face(main).surface.clone();
    let (ur, vr) = s.domain();
    let (bs, bound) = s.to_nurbs_within(ur, vr, tol).map_err(|e| {
        hint(
            FeatureError::new(&format!("{prefix}Failed")).with("detail", Json::Str(e)).with("tolerance", Json::Num(tol)),
            "the surface could not be converted to a B-spline within `tolerance`: raise `tolerance`, or check that the definition is smooth and has a solution over the whole span",
        )
    })?;
    let faces: Vec<Json> = body.face_ids().map(|x| Json::Str(body.face_id_str(x))).collect();
    let mut m = vec![
        ("kind".to_string(), Json::Str(if prefix == "sweepSurface" { "sweep" } else { "blend" }.into())),
        ("faces".into(), Json::Arr(faces)),
        ("area".into(), Json::Num(face_area(&body, main))),
    ];
    m.append(&mut extra);
    m.extend([
        ("degree".to_string(), Json::Arr(vec![Json::Num(bs.deg_u as f64), Json::Num(bs.deg_v as f64)])),
        ("count".into(), Json::Arr(vec![Json::Num(bs.nu as f64), Json::Num(bs.nv as f64)])),
        ("deviation".into(), Json::Num(bound)),
    ]);
    let id = f.id.clone();
    state.result.markers.retain(|x| x.as_arr().and_then(|a| a.first()).and_then(|x| x.as_str()) != Some(id.as_str()));
    state.result.markers.push(Json::Arr(vec![Json::Str(id.clone()), Json::Obj(m)]));
    br.set_surface(&id, body);
    Ok(())
}

// ---- sweepSurface -------------------------------------------------------------------------------------------

/// Columns of the sweep's moving frame at `u` (the frame `SweepSurface` uses).
fn frame_at(path: &Curve, law: &SweepLaw, u: f64) -> [Vec3; 3] {
    match law {
        SweepLaw::Translate => [Vec3::X, Vec3::Y, Vec3::Z],
        SweepLaw::Up(up) => {
            let t = path.deriv(u).normalize();
            let y = up.cross(t).normalize();
            [t, y, t.cross(y)]
        }
    }
}

/// `sweepSurface`: a profile swept along a path, one face on the `Surface::Sweep`.
pub fn exec_sweep_surface(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let gap = num_field(state, f, "gapTolerance", DEFAULT_GAP)?;
    if !(gap.is_finite() && gap >= 0.0) {
        return Err(invalid("gapTolerance", "gapTolerance is a distance (mm), 0 or more", "leave it out for 1e-5 mm"));
    }
    let no_input = |what: &str| {
        hint(
            FeatureError::new("sweepSurfaceNoInput").with("field", Json::Str(what.into())),
            "give `profile` and `path`: each a sketch or curve feature id, a curve reference ({kind: sketchCurve | edge | curve}) or a list of them in chain order",
        )
    };
    let Some((profile, v0, v1, prof_normal)) = field_curve(br, state, f.raw.get("profile"), gap, "sweepSurface")? else { return Err(no_input("profile")) };
    let Some((path, u0, u1, _)) = field_curve(br, state, f.raw.get("path"), gap, "sweepSurface")? else { return Err(no_input("path")) };
    let tol = tolerance(state, f)?;
    let frame = match f.raw.get("frame") {
        None | Some(Json::Null) => "tangent",
        Some(Json::Str(s)) if s == "tangent" || s == "translate" => s.as_str(),
        _ => return Err(invalid("frame", "frame is \"tangent\" or \"translate\"", "\"tangent\" turns the profile with the path, \"translate\" carries it without turning")),
    };
    let scale = 1.0 + profile.sample(v0, v1, 16).iter().chain(path.sample(u0, u1, 16).iter()).map(|p| p.len()).fold(0.0, f64::max);
    // the path must have a direction everywhere
    for i in 0..=64 {
        let u = u0 + (u1 - u0) * i as f64 / 64.0;
        if !(path.deriv(u).len() > 1e-12) {
            return Err(hint(
                FeatureError::new("sweepSurfaceDegenerate").with("detail", Json::Str(format!("the path has no direction at parameter {u}"))).with("at", v3(path.eval(u))),
                "the path must have a tangent everywhere: remove the zero-length piece or the cusp",
            ));
        }
    }
    let law = if frame == "translate" {
        SweepLaw::Translate
    } else {
        let up = match f.raw.get("up") {
            None | Some(Json::Null) => {
                // the normal of a plane path (the frame then turns only about it); a straight path: the world axis
                // furthest from it; any other path: +z
                let pts = path.sample(u0, u1, 64);
                match plane_normal(&pts) {
                    Some(n) => n,
                    None => {
                        let d = path.eval(u1) - path.eval(u0);
                        let straight = pts.iter().all(|q| (*q - path.eval(u0)).cross(d).len() <= 1e-9 * scale * (1.0 + d.len()));
                        if straight && d.len() > 0.0 {
                            let a = d.normalize().abs();
                            if a.x <= a.y && a.x <= a.z {
                                Vec3::X
                            } else if a.y <= a.z {
                                Vec3::Y
                            } else {
                                Vec3::Z
                            }
                        } else {
                            Vec3::Z
                        }
                    }
                }
            }
            Some(j) => json_vec(j).filter(|v| v.len() > 1e-12).ok_or_else(|| invalid("up", "up is a non-zero direction [x, y, z]", "give three numbers, for example [0, 0, 1]"))?,
        };
        SweepLaw::Up(up)
    };
    let law_copy = law.clone();
    let sw = SweepSurface::new(profile.clone(), path.clone(), law, (u0, u1), (v0, v1)).map_err(|e| {
        let at = (0..=32).map(|i| u0 + (u1 - u0) * i as f64 / 32.0).find(|&u| matches!(&law_copy, SweepLaw::Up(up) if up.normalize().cross(path.tangent(u)).len() < 1e-6));
        let mut fe = FeatureError::new("sweepSurfaceFrame").with("detail", Json::Str(e));
        if let Some(u) = at {
            fe = fe.with("at", v3(path.eval(u)));
        }
        if let SweepLaw::Up(up) = &law_copy {
            fe = fe.with("up", v3(up.normalize()));
        }
        hint(fe, "the up direction must never be along the path tangent: give another `up`, or use frame \"translate\"")
    })?;
    let s = Surface::Sweep(Box::new(sw));
    // no zero-area point and, turning with the path, no fold where the profile reaches past the centre of curvature
    let n = 24;
    for i in 0..=n {
        let u = u0 + (u1 - u0) * i as f64 / n as f64;
        let t = path.tangent(u);
        for j in 0..=n {
            let v = v0 + (v1 - v0) * j as f64 / n as f64;
            let d = s.derivs2(u, v);
            if matches!(law_copy, SweepLaw::Up(_)) && !(d.du.dot(t) > 1e-9 * d.du.len().max(1e-300)) {
                let c = path.derivs(u, 2);
                let k = c[1].cross(c[2]).len() / c[1].len().powi(3);
                let mut e = FeatureError::new("sweepSurfaceFold")
                    .with("detail", Json::Str(format!("the profile reaches past the path's centre of curvature near path parameter {u}: the surface folds over itself")))
                    .with("at", v3(d.p))
                    .with("distance", Json::Num(d.p.dist(path.eval(u))));
                if k > 1e-300 {
                    e = e.with("radius", Json::Num(1.0 / k));
                }
                return Err(hint(e, "make the profile smaller than the path's smallest radius of curvature on the side it bends to, or open the path's bend"));
            }
            if !(d.du.cross(d.dv).len() > 1e-9 * d.du.len() * d.dv.len()) {
                return Err(hint(
                    FeatureError::new("sweepSurfaceDegenerate")
                        .with("detail", Json::Str(format!("the surface has no area at path parameter {u}, profile parameter {v}: the profile runs along the sweep direction there")))
                        .with("at", v3(d.p)),
                    "the profile must cross the direction it is swept in: turn the profile, or sweep it along another path",
                ));
            }
        }
    }
    let ends_frame = (frame_at(&path, &law_copy, u0), frame_at(&path, &law_copy, u1));
    let (o0, o1) = (path.eval(u0), path.eval(u1));
    // the profile moved rigidly to the path end: q -> o1 + M1 · M0^T (q - o0)
    let lin = |m0: [Vec3; 3], m1: [Vec3; 3], d: Vec3| m1[0] * d.dot(m0[0]) + m1[1] * d.dot(m0[1]) + m1[2] * d.dot(m0[2]);
    let (m0, m1) = ends_frame;
    let end_profile = profile.transform_dyn(&|q| o1 + lin(m0, m1, q - o0), &|d| lin(m0, m1, d));
    let mut offers = Offers {
        left: Some(Offer { curve: profile.clone(), t0: v0, t1: v1 }),
        right: Some(Offer { curve: end_profile, t0: v0, t1: v1 }),
        ..Default::default()
    };
    if matches!(law_copy, SweepLaw::Translate) {
        let rail = |v: f64| {
            let off = profile.eval(v) - o0;
            Offer { curve: path.transform_dyn(&|q| q + off, &|d| d), t0: u0, t1: u1 }
        };
        offers.bottom = Some(rail(v0));
        offers.top = Some(rail(v1));
    }
    let mut p = patch(&f.id, "W", &s, offers);
    let (closed_u, closed_v) = seams(&p);
    let want_caps = crate::sketch::jtruthy(f.raw.get("caps"));
    let mut caps = false;
    if want_caps && !closed_v {
        return Err(hint(
            FeatureError::new("sweepSurfaceCaps").with("detail", Json::Str("the profile is open: there is no end loop to cap".into())),
            "close the profile, or leave `caps` out for an open sheet",
        ));
    }
    // a closed path closes the tube by itself: nothing to cap
    if want_caps && !closed_u {
        let n0 = prof_normal.or_else(|| plane_normal(&profile.sample(v0, v1, 64)));
        let Some(n0) = n0.filter(|n| profile.sample(v0, v1, 64).iter().all(|q| (*q - profile.eval(v0)).dot(*n).abs() <= 1e-7 * scale)) else {
            return Err(hint(
                FeatureError::new("sweepSurfaceCaps").with("detail", Json::Str("the profile does not lie in one plane: its ends cannot be capped with plane faces".into())),
                "use a planar profile, or leave `caps` out and close the ends with `fill`",
            ));
        };
        add_caps(&mut p, &f.id, n0, lin(m0, m1, n0));
        caps = true;
    }
    let b = &mut p.body;
    if closed_u && closed_v || caps {
        // a closed shell: outward normals
        let faces: Vec<FaceId> = b.face_ids().collect();
        if ck_topo::mass::signed_volume(b, &faces) < 0.0 {
            for x in faces {
                ck_topo::orient::flip_face(b, x);
            }
        }
    }
    let length: f64 = path.length_between(u0, u1);
    let mut extra = vec![
        ("frame".to_string(), Json::Str(frame.into())),
        ("closed".into(), Json::Arr(vec![Json::Bool(closed_u), Json::Bool(closed_v)])),
        ("caps".into(), Json::Bool(caps)),
        ("pathLength".into(), Json::Num(length)),
    ];
    if let SweepLaw::Up(up) = &law_copy {
        extra.push(("up".into(), v3(up.normalize())));
    }
    if caps {
        let faces: Vec<FaceId> = p.body.face_ids().collect();
        extra.push(("volume".into(), Json::Num(ck_topo::mass::signed_volume(&p.body, &faces))));
    }
    let main = p.face;
    publish(br, state, f, p.body, main, "sweepSurface", tol, extra)
}

/// Plane faces on the start (`left`) and end (`right`) loops of a sweep closed across its profile.
fn add_caps(p: &mut Patch, feature: &str, n0: Vec3, n1: Vec3) {
    let b = &mut p.body;
    let shell = b.face(p.face).shell;
    for (k, e, n) in [(0u32, p.left, n0), (1, p.right, n1)] {
        let ed = b.edge(e).clone();
        let o = ed.curve.eval(ed.t0);
        let fr = ck_geom::frame_from(o, n, ck_geom::any_perp(n));
        let face = b.add_face(shell, Surface::Plane { frame: fr }, true, FaceName::cap(feature, k, None));
        let lp = b.add_loop(face);
        // opposite to the sweep face's use of the edge
        let used = b.edge_coedges(e).iter().map(|&c| b.coedge(c).rev).next().unwrap_or(false);
        b.add_coedge(lp, e, !used, None);
        if ck_topo::orient::loop_uv_area(b, lp) < 0.0 {
            b.faces[face.idx()].sense = false;
        }
    }
}

// ---- blendSurface -------------------------------------------------------------------------------------------

/// One support face: its body slot, id and surface.
struct Support {
    name: String,
    slot: Slot,
    fid: FaceId,
    surf: Surface,
    sense: bool,
}

fn support(br: &Brep, name: &str) -> Result<Support, FeatureError> {
    let slot = br.slot_of_face(name).ok_or_else(|| br.missing_face(name))?;
    let body = br.slot_body(slot).ok_or_else(|| FeatureError::new(codes::MISSING_REFERENCE))?;
    let fid = body.find_face(name).ok_or_else(|| crate::refs::missing_face([body], name))?;
    let fc = body.face(fid);
    Ok(Support { name: name.into(), slot, fid, surf: fc.surface.clone(), sense: fc.sense })
}

/// Mean of a face's vertices (a point of the face away from an edge it shares).
fn face_mean(br: &Brep, s: &Support) -> Vec3 {
    let b = br.slot_body(s.slot).unwrap();
    let mut vs: Vec<VertexId> = vec![];
    for &l in &b.face(s.fid).loops {
        for &c in &b.lp(l).coedges {
            let v = b.coedge_start(c);
            if !vs.contains(&v) {
                vs.push(v);
            }
        }
    }
    let n = vs.len().max(1) as f64;
    vs.iter().fold(Vec3::ZERO, |a, &v| a + b.vertex(v).p) / n
}

/// The outward normal of a support at the foot of `q`.
fn outward_at(s: &Support, q: Vec3) -> (Vec3, Vec3) {
    let (u, v) = s.surf.closest_param(q);
    let n = s.surf.normal(u, v);
    (s.surf.eval(u, v), if s.sense { n } else { -n })
}

/// Does a ball of `radius` (law) roll along the whole spine touching both supports inside their faces? `Err((u, at,
/// detail))` names the first station that fails.
fn rolls(br: &Brep, sup: &[Support; 2], sides: (f64, f64), radius: (f64, f64), spine: &Curve, range: (f64, f64)) -> Result<RollingBall, (f64, Vec3, String)> {
    let rb = RollingBall::new(sup[0].surf.clone(), sup[1].surf.clone(), sides.0, sides.1, radius, spine.clone(), range).map_err(|e| (range.0, spine.eval(range.0), e))?;
    let mut caches = [LoopCache::new(), LoopCache::new()];
    let n = 32;
    for i in 0..=n {
        let u = range.0 + (range.1 - range.0) * i as f64 / n as f64;
        let st = rb.station(u);
        let r = rb.radius_at(u);
        if !(st.residual <= 1e-9 * (1.0 + r + st.centre.len())) {
            return Err((u, spine.eval(u), format!("no ball of radius {r} touches both supports here (residual {:.3e})", st.residual)));
        }
        for (k, p) in [st.p1, st.p2].into_iter().enumerate() {
            let b = br.slot_body(sup[k].slot).unwrap();
            if point_in_face_cached(b, sup[k].fid, p, 1e-5, &mut caches[k]) == Where::Out {
                return Err((u, p, format!("the ball of radius {r} touches the surface of {} outside the face", sup[k].name)));
            }
        }
    }
    Ok(rb)
}

/// The radius value or `[start, end]`.
fn radius_law(state: &RegenState, f: &Feature) -> Result<(f64, f64), FeatureError> {
    let bad = || invalid("radius", "radius is a positive number (mm), or [start, end] for a radius that varies linearly along the span", "give for example 3, or [2, 4]");
    let one = |j: &Json| crate::expr::eval_field(Some(j), &state.vars, 0.0).map_err(FeatureError::new);
    let r = match f.raw.get("radius") {
        None | Some(Json::Null) => return Err(invalid("radius", "radius is required", "give the ball radius in mm, or [start, end]")),
        Some(Json::Arr(a)) if a.len() == 2 => (one(&a[0])?, one(&a[1])?),
        Some(Json::Arr(_)) => return Err(bad()),
        Some(j) => {
            let r = one(j)?;
            (r, r)
        }
    };
    if !(r.0.is_finite() && r.1.is_finite() && r.0 > 0.0 && r.1 > 0.0) {
        return Err(bad());
    }
    Ok(r)
}

/// `blendSurface`: the rolling-ball blend of two faces, one face on the `Surface::Blend`.
pub fn exec_blend_surface(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let names: Vec<&str> = f.raw.get("faces").and_then(|a| a.as_arr()).map(|a| a.iter().filter_map(|x| x.as_str()).collect()).unwrap_or_default();
    if names.is_empty() {
        return Err(hint(FeatureError::new("blendSurfaceNoInput"), "give `faces`: the two faces the ball rolls on, and `radius`"));
    }
    if names.len() != 2 || names[0] == names[1] {
        return Err(invalid("faces", &format!("a blend rolls on two different faces; {} given", names.len()), "name exactly two faces in `faces`"));
    }
    let sup = [support(br, names[0])?, support(br, names[1])?];
    let radius = radius_law(state, f)?;
    let tol = tolerance(state, f)?;
    // the span: from / to, else the shared edge
    let pt = |k: &str| -> Result<Option<Vec3>, FeatureError> {
        match f.raw.get(k) {
            None | Some(Json::Null) => Ok(None),
            Some(j) => json_vec(j).map(Some).ok_or_else(|| invalid(k, &format!("{k} is a point [x, y, z]"), "give three numbers")),
        }
    };
    let (from, to, at) = (pt("from")?, pt("to")?, pt("at")?);
    let (spine, range, span) = match (from, to) {
        (Some(a), Some(b)) => {
            if !(a.dist(b) > 1e-9) {
                return Err(hint(FeatureError::new("blendSurfaceSpan").with("detail", Json::Str("from and to are the same point".into())).with("at", v3(a)), "give two different end points"));
            }
            (Curve::line(a, b - a), (0.0, a.dist(b)), "points")
        }
        (Some(_), None) | (None, Some(_)) => return Err(hint(FeatureError::new("blendSurfaceSpan").with("detail", Json::Str("only one of from / to".into())), "give both `from` and `to`, or neither to use the edge the faces share")),
        (None, None) => {
            let shared: Vec<EdgeId> = if sup[0].slot == sup[1].slot {
                let b = br.slot_body(sup[0].slot).unwrap();
                b.edge_ids().filter(|&e| {
                    let fs = b.edge_faces(e);
                    fs.contains(&sup[0].fid) && fs.contains(&sup[1].fid)
                }).collect()
            } else {
                vec![]
            };
            let b = br.slot_body(sup[0].slot).unwrap();
            let dist = |e: EdgeId| at.map_or(0.0, |p| {
                let ed = b.edge(e);
                ck_ops::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1
            });
            let Some(e) = shared.into_iter().min_by(|&x, &y| dist(x).total_cmp(&dist(y))) else {
                return Err(hint(
                    FeatureError::new("blendSurfaceSpan").with("detail", Json::Str(format!("{} and {} share no edge", sup[0].name, sup[1].name))),
                    "give the span with `from` and `to` (the spine runs between them), or pick two faces that meet in an edge",
                ));
            };
            let ed = b.edge(e);
            (ed.curve.clone(), (ed.t0, ed.t1), "edge")
        }
    };
    // sides, relative to each face's outward normal
    let sides = match f.raw.get("sides") {
        None | Some(Json::Null) => {
            let mut s = [0.0; 2];
            for k in 0..2 {
                let m = face_mean(br, &sup[1 - k]);
                let (foot, n) = outward_at(&sup[k], m);
                let d = (m - foot).dot(n);
                if !(d.abs() > 1e-9 * (1.0 + m.len())) {
                    return Err(hint(
                        FeatureError::new("blendSurfaceSides").with("face", Json::Str(sup[k].name.clone())).with("detail", Json::Str(format!("{} lies in the surface of {}: no side faces it", sup[1 - k].name, sup[k].name))),
                        "give `sides` [s1, s2]: +1 puts the ball on the side a face's outward normal points to, -1 against it",
                    ));
                }
                s[k] = d.signum();
            }
            (s[0], s[1])
        }
        Some(Json::Arr(a)) if a.len() == 2 && a.iter().all(|x| matches!(x.as_f64(), Some(v) if v == 1.0 || v == -1.0)) => (a[0].as_f64().unwrap(), a[1].as_f64().unwrap()),
        Some(_) => return Err(invalid("sides", "sides is [s1, s2], each 1 or -1", "+1 puts the ball on the side a face's outward normal points to, -1 against it")),
    };
    // to the sides of the surfaces' natural normals
    let natural = (if sup[0].sense { sides.0 } else { -sides.0 }, if sup[1].sense { sides.1 } else { -sides.1 });
    let rb = match rolls(br, &sup, natural, radius, &spine, range) {
        Ok(rb) => rb,
        Err((u, at, detail)) => {
            // does a smaller ball roll? then the radius is too large; else there is no contact at all
            let ok = |k: f64| rolls(br, &sup, natural, (radius.0 * k, radius.1 * k), &spine, range).is_ok();
            let rmax = radius.0.max(radius.1);
            let smallest = 1e-3;
            if ok(smallest) {
                let (mut lo, mut hi) = (smallest, 1.0);
                for _ in 0..40 {
                    let m = 0.5 * (lo + hi);
                    if ok(m) {
                        lo = m;
                    } else {
                        hi = m;
                    }
                }
                let r = if radius.0 == radius.1 { Json::Num(radius.0) } else { Json::Arr(vec![Json::Num(radius.0), Json::Num(radius.1)]) };
                return Err(hint(
                    FeatureError::new("blendSurfaceRadius").with("radius", r).with("maxRadius", Json::Num(lo * rmax)).with("at", v3(at)).with("detail", Json::Str(detail)),
                    "use a radius no larger than maxRadius (a radius law scaled down to it), or extend the faces the ball rolls on",
                ));
            }
            return Err(hint(
                FeatureError::new("blendSurfaceNoContact").with("at", v3(at)).with("u", Json::Num(u)).with("detail", Json::Str(detail)),
                "no ball rolls on both faces along the span: check `sides` (which side of each face the ball is on) and the span (`from` / `to` or the shared edge)",
            ));
        }
    };
    let s = Surface::Blend(Box::new(rb.clone()));
    // the end arcs as circles (angle t = v·θ)
    let arc = |u: f64| -> Option<Offer> {
        let st = rb.station(u);
        let (e1, e2) = (st.p1 - st.centre, st.p2 - st.centre);
        let th = e1.cross(e2).len().atan2(e1.dot(e2));
        let n = e1.cross(e2).try_normalize()?;
        (th > 1e-9).then(|| Offer { curve: Curve::circle_x(st.centre, n, e1, e1.len()), t0: 0.0, t1: th })
    };
    let offers = Offers { left: arc(range.0), right: arc(range.1), ..Default::default() };
    let mut p = patch(&f.id, "R", &s, offers);
    // outward like the first support where the ball touches it: away from the ball centre when the ball is inside
    let um = 0.5 * (range.0 + range.1);
    let st = rb.station(um);
    let d = s.derivs2(um, 0.0);
    let want = (st.p1 - st.centre) * (-sides.0);
    if d.du.cross(d.dv).dot(want) < 0.0 {
        ck_topo::orient::flip_face(&mut p.body, p.face);
    }
    let (closed_u, _) = seams(&p);
    let length = spine.length_between(range.0, range.1);
    let extra = vec![
        ("radius".to_string(), Json::Arr(vec![Json::Num(radius.0), Json::Num(radius.1)])),
        ("sides".into(), Json::Arr(vec![Json::Num(sides.0), Json::Num(sides.1)])),
        ("span".into(), Json::Str(span.into())),
        ("spineLength".into(), Json::Num(length)),
        ("closed".into(), Json::Bool(closed_u)),
    ];
    let main = p.face;
    publish(br, state, f, p.body, main, "blendSurface", tol, extra)
}
