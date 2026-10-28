//! Sketch-driven surface features: `surfacePlanar`, `surfaceRevolve`,
//! `surfaceExtrude` (RECONSTRUCTION from oracle probes).
//!
//! Pinned behaviour (probe parts in `tools/brep_cases.mjs`, `surf*`):
//! * the sheet is the solid sweep of the same profile with the cap faces
//!   removed: edge list = rails, then per segment its C0 / C1 edge, in the
//!   solid's order (open edges stay in place); faces `F.S.<entity>`;
//! * an OPEN chain is swept as the closed loop `chain + closing chord`;
//!   the chord's faces and edges are not part of the sheet;
//! * `surfacePlanar` is one face `F.P` (plane normal = sketch normal) with
//!   the profile's edges in entity order.

use super::extrude::{profile_seg, sketch_axis, sketch_profile, MISSING_AXIS, MIN_DEPTH};
use super::{Brep, End, FeatureKind};
use crate::regen::{codes, FeatureError, RegenState, SketchResult};
use crate::sketch::profile::open_chains;
use crate::Feature;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceId, FaceName, Role};
use ck_topo::make::revolve::revolve;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::f64::consts::TAU;

const CHORD: &str = "~chord";

fn sketch_of<'a>(state: &'a RegenState, f: &Feature) -> Result<&'a SketchResult, FeatureError> {
    f.raw
        .get("sketch")
        .and_then(|s| s.as_str())
        .and_then(|s| state.sketch(s))
        .ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))
}

/// Profile of the sketch: its regions, or the open chains closed by a chord.
fn sheet_profile(sk: &SketchResult, ccw: bool) -> Result<Profile, FeatureError> {
    if !sk.profiles.regions.is_empty() {
        return sketch_profile(sk);
    }
    let chains = open_chains(&sk.sketch);
    if chains.is_empty() {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    let mut regions = vec![];
    for ch in chains {
        let mut segs: Vec<ProfileSeg> = ch.iter().map(profile_seg).collect::<Result<_, _>>()?;
        let (a, b) = (segs[segs.len() - 1].end(), segs[0].start());
        if a.dist(b) > 1e-9 {
            segs.push(ProfileSeg::line(CHORD, a, b));
        }
        let mut outer = ProfileLoop::new(segs);
        // pinned: a clockwise chain + chord loop is reversed to CCW as a
        // plain reverse (`[chord', last', .., first']`, no rotation)
        if ccw && outer.signed_area() < 0.0 {
            outer = ProfileLoop::new(outer.segs.iter().rev().map(|s| s.reversed()).collect());
        }
        regions.push(ProfileRegion { outer, holes: vec![] });
    }
    Ok(Profile::new(sk.frame, regions))
}

/// The side faces of a swept solid (all but the caps and the chord).
fn side_faces(b: &Body) -> Vec<FaceId> {
    b.face_ids()
        .filter(|&f| matches!(&b.face(f).name.role, Role::Side(t) if t != CHORD))
        .collect()
}

fn topo_err(e: String) -> FeatureError {
    FeatureError::new(&e)
}

pub fn exec_surface_extrude(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let sk = sketch_of(state, f).map_err(|_| FeatureError::new(codes::NO_CLOSED_PROFILE))?;
    let depth = state.num(f, "depth", 10.0)?.max(MIN_DEPTH);
    let flip = crate::sketch::jtruthy(f.raw.get("flip"));
    let end = f.raw.get("end").and_then(|e| e.as_str()).unwrap_or("blind");
    let profile = sheet_profile(sk, false)?;
    let o = match end {
        "mid" => ExtrudeOpts::mid(depth),
        _ if flip => ExtrudeOpts::flipped(depth),
        _ => ExtrudeOpts::blind(depth),
    };
    let solid = extrude(&f.id, &profile, o).map_err(topo_err)?;
    let sheet = ck_ops::surfacing::sheet::sheet(&solid, &side_faces(&solid));
    br.set_surface(&f.id, sheet);
    Ok(())
}

/// Error code of a profile that straddles the revolve axis.
pub const PROFILE_CROSSES_AXIS: &str = "profileCrossesAxis";

/// Does any profile segment lie on both sides of the revolve axis?
/// (oracle: a region or chain with points on both sides of the axis line.)
pub fn crosses_axis(profile: &Profile, o: Vec3, d: Vec3) -> bool {
    let fr = &profile.frame;
    let (mut pos, mut neg) = (false, false);
    for reg in &profile.regions {
        for l in reg.loops() {
            for s in &l.segs {
                for k in 0..=8 {
                    let t = s.t0 + (s.t1 - s.t0) * k as f64 / 8.0;
                    let q = s.curve.eval(t);
                    let p = fr.o + fr.x * q.x + fr.y * q.y;
                    let side = (p - o).cross(d).dot(fr.z);
                    pos |= side > 1e-9;
                    neg |= side < -1e-9;
                }
            }
        }
    }
    pos && neg
}

pub fn exec_surface_revolve(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::SurfaceRevolve { end, flip, .. } = &f.kind else { unreachable!() };
    let sk = sketch_of(state, f)?;
    let angle = state.num(f, "angle", 360.0)?;
    let (o, d) = f
        .raw
        .get("axis")
        .and_then(|a| sketch_axis(state, sk, a))
        .ok_or_else(|| FeatureError::new(MISSING_AXIS))?;
    let profile = sheet_profile(sk, true)?;
    if crosses_axis(&profile, o, d) {
        // pinned: closed region -> empty detail, open chain -> the projection text
        let detail = if sk.profiles.regions.is_empty() { "the profile and its projection onto the axis cross" } else { "" };
        return Err(FeatureError::new(PROFILE_CROSSES_AXIS).with("detail", ck_json::Json::Str(detail.into())));
    }
    if angle.abs() < 1e-9 {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    let mut a = angle.to_radians();
    if a.abs() >= TAU {
        a = TAU * a.signum();
    }
    if *flip {
        a = -a;
    }
    let (a0, a1) = if *end == End::Mid { (-a / 2.0, a / 2.0) } else { (0.0, a) };
    let (d, a0, a1) = if a1 < a0 { (-d, -a0, -a1) } else { (d, a0, a1) };
    let (a0, a1) = (a0.min(a1), a0.max(a1));
    let solid = revolve(&f.id, &profile, o, d, a0, a1).map_err(topo_err)?;
    let sheet = ck_ops::surfacing::sheet::sheet(&solid, &side_faces(&solid));
    br.set_surface(&f.id, sheet);
    Ok(())
}

pub fn exec_surface_planar(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let sk = sketch_of(state, f)?;
    if sk.profiles.regions.is_empty() {
        return Err(FeatureError::new(codes::NO_CLOSED_PROFILE));
    }
    let profile = sketch_profile(sk)?;
    let solid = extrude(&f.id, &profile, ExtrudeOpts::blind(1.0)).map_err(topo_err)?;
    let caps: Vec<FaceId> = solid.face_ids().filter(|&x| matches!(solid.face(x).name.role, Role::Cap(0))).collect();
    let mut sheet = ck_ops::surfacing::sheet::sheet(&solid, &caps);
    let multi = caps.len() > 1;
    let fids: Vec<FaceId> = sheet.face_ids().collect();
    for (k, fid) in fids.into_iter().enumerate() {
        let mut name = FaceName::other(&f.id, "P");
        if multi {
            name.region = Some(k as u32 + 1);
        }
        sheet.faces[fid.idx()].name = name;
        flip_face(&mut sheet, fid);
    }
    br.set_surface(&f.id, sheet);
    Ok(())
}

/// Reverse a face's orientation (sense and every loop's travel).
pub fn flip_face(b: &mut Body, f: FaceId) {
    ck_topo::orient::flip_face(b, f);
}
#[allow(dead_code)]
fn flip_face_old(b: &mut Body, f: FaceId) {
    b.faces[f.idx()].sense = !b.faces[f.idx()].sense;
    for l in b.face(f).loops.clone() {
        let mut cs = b.lp(l).coedges.clone();
        cs.reverse();
        for &c in &cs {
            let co = &mut b.coedges[c.idx()];
            co.rev = !co.rev;
        }
        b.loops[l.idx()].coedges = cs;
    }
}
