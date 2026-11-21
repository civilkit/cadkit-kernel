//! Solids built straight from numbers, with no sketch: `torus`, `pipe`, `coil`, `boundingSolid` and `dome`.
//!
//! ```json
//! {"type":"torus","id":"F1","op":"add","center":[0,0,0],"axis":"z","majorRadius":20,"minorRadius":4}
//! {"type":"pipe","id":"F2","op":"add","path":{"kind":"sketch","id":"F1"},"radius":5,"wall":1}
//! {"type":"coil","id":"F3","op":"add","center":[0,0,0],"axis":"z","radius":10,"wireRadius":1,"pitch":5,"revolutions":8,"leftHand":false}
//! {"type":"boundingSolid","id":"F4","bodies":[{"body":"Part"}],"padding":0}
//! {"type":"dome","id":"F5","face":"F1.C1","height":5,"elliptical":false}
//! ```
//!
//! * `torus`: a ring about `axis` (`"x"|"y"|"z"` or a vector, default `z`) through `center` (default the origin);
//!   `majorRadius` (also `radius`) is the distance from the axis to the middle of the tube, `minorRadius`
//!   (also `tubeRadius`) the tube radius, below the major radius. One toroidal face `<id>.S.e1`.
//! * `pipe`: a tube of circular section along `path` (`{kind:"sketch"|"curve", id}`, an open chain, as for a sweep):
//!   `radius` is the outer radius, `wall` (optional, below the radius) makes it hollow with that wall thickness.
//!   The section is built at the start of the path, normal to it; the sweep is the `sweep` code.
//! * `coil`: a wire of circular section (`wireRadius`) wound as a helix of `radius` (the distance from the axis to the
//!   middle of the wire) about `axis` through `center`. `pitch` and `revolutions` (or `height`, which gives the
//!   revolutions) define it; `leftHand`. The pitch must exceed the wire diameter (turns would overlap) and the
//!   radius must exceed the wire radius. At most 500 revolutions.
//! * `boundingSolid`: the axis-aligned box around the selected bodies (`bodies`, selected as for `moveBody`; none
//!   given: every solid body), grown by `padding` (default 0) on each side. Always a new body (`op`, when given,
//!   must be `newBody`). A box with no thickness on an axis needs a `padding`.
//! * `dome`: a cap on a planar face bounded by one circle: `face` (or `faces: [one]`), `height` above the face
//!   (negative: a dimple cut into the body), spherical; `elliptical: true` would be a quarter ellipse of semi-axes
//!   radius and height (not built yet: `notImplemented {what: "elliptical dome"}`). Any other face is `invalidValue`.
//!
//! The first four take `op` (`add`, `cut`, `newBody`; `boundingSolid` and `dome` as described). Errors:
//! `invalidValue {detail}`, `missingPath`, `missingReference`, `nothingToCut`.

use super::extrude::{Combine, Recipe, Tool};
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_json::Json;
use ck_ops::sweep::PathSeg;
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

/// Most revolutions a coil takes (each turn is a screw surface patch row).
pub const MAX_COIL_REVOLUTIONS: f64 = 500.0;

fn bad(detail: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into()))
}

fn topo_err(e: String) -> FeatureError {
    FeatureError::new(&e)
}

fn pos(v: f64) -> bool {
    v.is_finite() && v > 0.0
}

/// A field that is the first present of `keys`, as a length (expressions allowed).
fn len(state: &RegenState, f: &Feature, keys: &[&str]) -> Result<f64, FeatureError> {
    let key = keys.iter().find(|k| f.raw.get(k).is_some()).copied().unwrap_or(keys[0]);
    state.size(f, key, 0.0)
}

fn combine_of(f: &Feature, has_body: bool) -> Result<Combine, FeatureError> {
    let c = Combine::of(super::op(&f.raw));
    if c == Combine::Cut && !has_body {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    Ok(c)
}

/// The axis (`"x"|"y"|"z"`, `{kind:"axis", axis}` or a vector), default `z`.
fn axis_of(state: &RegenState, f: &Feature) -> Result<Vec3, FeatureError> {
    let Some(j) = f.raw.get("axis").filter(|a| !matches!(a, Json::Null)) else { return Ok(Vec3::Z) };
    if let Some(v) = super::move_body::vec3(state, Some(j))? {
        return v.try_normalize().ok_or_else(|| bad("the axis is not a zero vector"));
    }
    match j.as_str().or_else(|| j.get_str("axis")) {
        Some("x") => Ok(Vec3::X),
        Some("y") => Ok(Vec3::Y),
        Some("z") => Ok(Vec3::Z),
        _ => Err(bad("axis is x, y, z or a vector of three numbers")),
    }
}

/// A frame at `o` with `y` along `a` and `x` a unit vector perpendicular to it (so the profile plane holds the axis).
fn axial_frame(o: Vec3, a: Vec3) -> Frame {
    let r = if a.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let x = (r - a * r.dot(a)).normalize();
    Frame { o, x, y: a, z: x.cross(a) }
}

fn one_region(frame: Frame, outer: ProfileLoop, holes: Vec<ProfileLoop>) -> Profile {
    Profile::new(frame, vec![ProfileRegion { outer, holes }])
}

pub fn exec_torus(state: &RegenState, f: &Feature, has_body: bool) -> Result<Tool, FeatureError> {
    let c = super::move_body::vec3(state, f.raw.get("center"))?.unwrap_or(Vec3::ZERO);
    let a = axis_of(state, f)?;
    let major = len(state, f, &["majorRadius", "radius"])?;
    let minor = len(state, f, &["minorRadius", "tubeRadius"])?;
    if !pos(major) || !pos(minor) {
        return Err(bad("a torus needs majorRadius and minorRadius above 0"));
    }
    if minor >= major {
        return Err(bad("the minor radius is below the major radius (a ring torus; a larger tube crosses the axis)"));
    }
    let combine = combine_of(f, has_body)?;
    let frame = axial_frame(c, a);
    let outer = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(major, 0.0), minor)]);
    let profile = one_region(frame, outer, vec![]);
    let body = revolve(&f.id, &profile, c, a, 0.0, TAU).map_err(topo_err)?;
    Ok(Tool { body, combine, recipe: Some(Recipe::Revolve { profile, o: c, d: a, a0: 0.0, a1: TAU }) })
}

/// The circular (or annular) section of radius `r` / wall `wall` in the plane through `o` normal to `n`.
fn round_section(o: Vec3, n: Vec3, x: Vec3, r: f64, wall: Option<f64>) -> Profile {
    let frame = Frame { o, x, y: n.cross(x), z: n };
    let outer = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::ZERO, r)]);
    let holes = match wall {
        Some(w) => vec![ProfileLoop::new(vec![ProfileSeg::circle("h1", Vec2::ZERO, r - w).reversed()])],
        None => vec![],
    };
    one_region(frame, outer, holes)
}

fn wall_of(state: &RegenState, f: &Feature, r: f64) -> Result<Option<f64>, FeatureError> {
    if f.raw.get("wall").is_none_or(|w| matches!(w, Json::Null)) {
        return Ok(None);
    }
    let w = state.size(f, "wall", 0.0)?;
    if !pos(w) || w >= r {
        return Err(bad("the wall is above 0 and below the radius"));
    }
    Ok(Some(w))
}

fn perp(d: Vec3) -> Vec3 {
    let r = if d.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    (r - d * r.dot(d)).normalize()
}

pub fn exec_pipe(br: &Brep, state: &RegenState, f: &Feature) -> Result<Tool, FeatureError> {
    let path = f.raw.get("path").and_then(crate::GeomRef::from_json);
    let r = len(state, f, &["radius"])?;
    if !pos(r) {
        return Err(bad("the radius of a pipe is above 0"));
    }
    let wall = wall_of(state, f, r)?;
    let segs = super::sweep::path_segs(br, state, path.as_ref())?;
    let combine = combine_of(f, br.body.is_some())?;
    let first = segs.first().ok_or_else(|| FeatureError::new(super::sweep::PATH_INVALID))?;
    let d = first.start_dir().ok_or_else(|| FeatureError::new(super::sweep::PATH_INVALID))?;
    let profile = round_section(first.start(), d, perp(d), r, wall);
    let body = ck_ops::sweep::sweep_joined(&f.id, &profile, &segs, 0.0, super::sweep::path_join_tol(&segs)).map_err(|e| FeatureError::new(&e))?;
    Ok(Tool { body, combine, recipe: None })
}

pub fn exec_coil(state: &RegenState, f: &Feature, has_body: bool) -> Result<Tool, FeatureError> {
    let c = super::move_body::vec3(state, f.raw.get("center"))?.unwrap_or(Vec3::ZERO);
    let a = axis_of(state, f)?;
    let radius = len(state, f, &["radius"])?;
    let wire = len(state, f, &["wireRadius"])?;
    let pitch = len(state, f, &["pitch"])?;
    if !pos(radius) || !pos(wire) || !pos(pitch) {
        return Err(bad("a coil needs radius, wireRadius and pitch above 0"));
    }
    let revs = if f.raw.get("revolutions").is_some() {
        state.num(f, "revolutions", 0.0)?
    } else if f.raw.get("height").is_some() {
        state.size(f, "height", 0.0)? / pitch
    } else {
        return Err(bad("a coil needs revolutions or height"));
    };
    if !pos(revs) || revs > MAX_COIL_REVOLUTIONS {
        return Err(bad(&format!("the revolutions of a coil are above 0 and at most {MAX_COIL_REVOLUTIONS}")));
    }
    if radius <= wire {
        return Err(bad("the coil radius exceeds the wire radius (the wire would cross the axis)"));
    }
    if pitch <= 2.0 * wire {
        return Err(bad("the pitch exceeds the wire diameter (the turns would overlap)"));
    }
    let combine = combine_of(f, has_body)?;
    let x = perp(a);
    let frame = Frame { o: c, x, y: a.cross(x), z: a };
    let hp = ck_ops::helix::HelixParams {
        defined_by: ck_ops::helix::DefinedBy::PitchRevs,
        pitch,
        revolutions: revs,
        height: 0.0,
        start_angle: 0.0,
        left_hand: crate::sketch::jtruthy(f.raw.get("leftHand")),
        reverse: false,
    };
    let h = ck_ops::helix::helix(&frame, c, radius, &hp).map_err(|e| FeatureError::new(&e))?;
    let seg = PathSeg::Helix { h, rev: false };
    let d = seg.start_dir().ok_or_else(|| bad("degenerate helix"))?;
    // the wire section lies in the axial plane at the start of the helix
    let profile = round_section(seg.start(), d, perp_in(d, a), wire, None);
    let body = ck_ops::sweep::sweep(&f.id, &profile, &[seg], 0.0).map_err(|e| FeatureError::new(&e))?;
    Ok(Tool { body, combine, recipe: None })
}

/// A unit vector perpendicular to `d`, the nearest to `a` (the axis lies in the section plane).
fn perp_in(d: Vec3, a: Vec3) -> Vec3 {
    (a - d * a.dot(d)).try_normalize().unwrap_or_else(|| perp(d))
}

/// Box (corners `lo`, `hi`) as a tool: the extrude of its bottom rectangle, faces named as the `box` feature names them.
fn box_tool(id: &str, lo: Vec3, hi: Vec3, combine: Combine) -> Result<Tool, FeatureError> {
    let frame = Frame { o: Vec3::new((lo.x + hi.x) / 2.0, (lo.y + hi.y) / 2.0, lo.z), x: Vec3::X, y: Vec3::Y, z: Vec3::Z };
    let (hx, hy) = ((hi.x - lo.x) / 2.0, (hi.y - lo.y) / 2.0);
    let p = [Vec2::new(-hx, -hy), Vec2::new(hx, -hy), Vec2::new(hx, hy), Vec2::new(-hx, hy)];
    let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let profile = one_region(frame, outer, vec![]);
    let opts = ExtrudeOpts { s0: 0.0, s1: hi.z - lo.z, dir: None, draft: 0.0 };
    let body = extrude(id, &profile, opts.clone()).map_err(topo_err)?;
    Ok(Tool { body, combine, recipe: Some(Recipe::Extrude { profile, opts }) })
}

/// Bounds of `faces`: their edges, plus a grid over each curved face (a sphere bulges beyond its edges).
fn faces_box(b: &ck_topo::body::Body, faces: &[ck_topo::body::FaceId]) -> (Vec3, Vec3) {
    let (mut lo, mut hi) = b.bbox_of_faces(faces);
    for &fid in faces {
        let face = b.face(fid);
        if matches!(face.surface, Surface::Plane { .. }) {
            continue;
        }
        let (mut a, mut z) = (Vec2::new(f64::INFINITY, f64::INFINITY), Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
        for &l in &face.loops {
            for &c in &b.lp(l).coedges {
                let (t0, t1) = b.coedge_params(c);
                for k in 0..=8 {
                    let uv = b.coedge_uv(c, t0 + (t1 - t0) * k as f64 / 8.0, None);
                    a = Vec2::new(a.x.min(uv.x), a.y.min(uv.y));
                    z = Vec2::new(z.x.max(uv.x), z.y.max(uv.y));
                }
            }
        }
        if !(a.x.is_finite() && z.x.is_finite()) {
            continue;
        }
        let n = 32;
        let us: Vec<f64> = (0..=n).map(|i| a.x + (z.x - a.x) * i as f64 / n as f64).collect();
        let vs: Vec<f64> = (0..=n).map(|j| a.y + (z.y - a.y) * j as f64 / n as f64).collect();
        for p in face.surface.eval_grid(&us, &vs) {
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    (lo, hi)
}

pub fn exec_bounding_solid(br: &Brep, state: &RegenState, f: &Feature) -> Result<Tool, FeatureError> {
    if let Some(o) = f.raw.get("op").and_then(|o| o.as_str()) {
        if !super::bodies::is_new_body_op(o) {
            return Err(bad("a bounding solid is always a new body; leave op out or use newBody"));
        }
    }
    let padding = if f.raw.get("padding").is_some() { state.num(f, "padding", 0.0)? } else { 0.0 };
    if !(padding.is_finite() && padding >= 0.0) {
        return Err(bad("the padding is 0 or more"));
    }
    if br.body.is_none() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let (faces, surf) = super::move_body::select(br, f)?;
    if !surf.is_empty() {
        return Err(bad("a bounding solid is taken around solid bodies, not surface bodies"));
    }
    let b = br.body.as_ref().unwrap();
    if faces.is_empty() {
        return Err(FeatureError::new(codes::MISSING_REFERENCE));
    }
    let (lo, hi) = faces_box(b, &faces);
    let (lo, hi) = (lo - Vec3::new(padding, padding, padding), hi + Vec3::new(padding, padding, padding));
    let d = hi - lo;
    if !(d.x > 1e-9 && d.y > 1e-9 && d.z > 1e-9) {
        return Err(bad("the bodies have no thickness along an axis; give a padding"));
    }
    box_tool(&f.id, lo, hi, Combine::Separate)
}

pub fn exec_dome(br: &Brep, state: &RegenState, f: &Feature) -> Result<Tool, FeatureError> {
    let name = f
        .raw
        .get("face")
        .and_then(|v| v.as_str())
        .or_else(|| f.raw.get("faces").and_then(|a| a.as_arr()).and_then(|a| (a.len() == 1).then(|| a[0].as_str()).flatten()));
    let Some(b) = br.body.as_ref() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let Some(name) = name else {
        return Err(if f.raw.get("face").is_none() && f.raw.get("faces").is_none() { FeatureError::new(codes::NOTHING_TO_CUT) } else { bad("a dome takes one face") });
    };
    let h = state.num(f, "height", 0.0)?;
    if !h.is_finite() || h == 0.0 {
        return Err(bad("the height of a dome is not 0"));
    }
    let (fid, _) = super::extrude::resolve_face(b, name, None).ok_or_else(|| crate::refs::missing_face([b], name))?;
    let face = b.face(fid);
    let not_circle = || bad("a dome goes on a planar face bounded by one circle (no holes)");
    if !matches!(face.surface, Surface::Plane { .. }) || face.loops.len() != 1 {
        return Err(not_circle());
    }
    let edges = b.face_edges(fid);
    let [e] = edges[..] else { return Err(not_circle()) };
    let Curve::Circle { frame: cf, r } = b.edge(e).curve.clone() else { return Err(not_circle()) };
    let (a, c) = (r, cf.o);
    let outward = b.planar_normal(fid);
    // a dimple is the cap built on the inside and cut
    let (n, hh, combine) = if h > 0.0 { (outward, h, Combine::Add) } else { (-outward, -h, Combine::Cut) };
    let max = 1e6 * a;
    if hh > max {
        return Err(bad("the height of a dome is below a million times its radius"));
    }
    if crate::sketch::jtruthy(f.raw.get("elliptical")) {
        // the quarter ellipse meets the wall tangentially and the boolean with a generic surface of revolution does not close there
        return Err(FeatureError::new(&super::extrude::todo("elliptical dome")));
    }
    let frame = axial_frame(c, n);
    let base = |t: &str, p: Vec2, q: Vec2| ProfileSeg::line(t, p, q);
    let rr = (a * a + hh * hh) / (2.0 * hh);
    let cap = ProfileSeg::arc("e2", Vec2::new(0.0, hh - rr), Vec2::new(a, 0.0), Vec2::new(0.0, hh));
    let outer = ProfileLoop::new(vec![base("e1", Vec2::ZERO, Vec2::new(a, 0.0)), cap, base("e3", Vec2::new(0.0, hh), Vec2::ZERO)]);
    let profile = one_region(frame, outer, vec![]);
    let body = revolve(&f.id, &profile, c, n, 0.0, TAU).map_err(topo_err)?;
    Ok(Tool { body, combine, recipe: Some(Recipe::Revolve { profile, o: c, d: n, a0: 0.0, a1: TAU }) })
}
