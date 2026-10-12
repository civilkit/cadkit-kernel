//! `rib` (RECONSTRUCTION from oracle probes, `tools/brep_cases.mjs` `rib*`).
//!
//! Pinned behaviour (hollow 40x20x10 box, line sketch on an offset plane):
//! * checks: no body -> `nothingToCut`; no / unknown sketch -> `missingSketch`;
//!   `thickness <= 0` -> `invalidValue`; a sketch with a closed region ->
//!   `ribNeedsOpenSketch`; no enclosed rib piece -> `ribNoContact`;
//! * the rib is a thin wall of total `thickness`, centred on the sketch plane
//!   (faces `C0`/`C1` at -t/2 / +t/2 along the sketch normal); the open chain
//!   is its FREE edge (face `S.<entity>r`) and the wall fills the space
//!   between the chain and the body: it is extended past both chain ends and
//!   away from the chain on the side that is enclosed by the body (`flip`
//!   does not change a successful result), then united with the body;
//! * the chain extension / far faces never survive (they sit outside the
//!   body), so only `C0`, `C1` and `S.<entity>r` of the profile remain.
//!
//! Construction here: profile = chain + straight extensions + far side
//! (tagged `x0..x3` / `<entity>l`), extruded mid by `thickness`, cut by the
//! body; the pieces without far faces are the rib; GUESS: the side tried
//! first is the left of the chain direction (right when `flip`).

use super::extrude::{profile_seg, Combine};
use super::{Brep, FeatureKind};
use crate::regen::{codes, FeatureError, RegenState};
use crate::sketch::profile::open_chains;
use crate::Feature;
use ck_geom::Vec2;
use ck_topo::body::{Body, FaceId, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

pub const RIB_NEEDS_OPEN_SKETCH: &str = "ribNeedsOpenSketch";
pub const RIB_NO_CONTACT: &str = "ribNoContact";

fn is_far(tag: &str) -> bool {
    tag.starts_with('x') || tag.ends_with('l')
}

pub fn exec_rib(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let FeatureKind::Rib { sketch, flip, .. } = &f.kind else { unreachable!() };
    let Some(body) = br.body.clone() else { return Err(FeatureError::new(codes::NOTHING_TO_CUT)) };
    let sk = sketch
        .as_ref()
        .and_then(|r| state.sketch(r.as_str()))
        .ok_or_else(|| FeatureError::new(codes::MISSING_SKETCH))?;
    let t = state.num(f, "thickness", 2.0)?;
    if !(t > 0.0) {
        return Err(FeatureError::new(codes::INVALID_VALUE));
    }
    if !sk.profiles.regions.is_empty() {
        return Err(FeatureError::new(RIB_NEEDS_OPEN_SKETCH));
    }
    let chains = open_chains(&sk.sketch);
    let Some(chain) = chains.into_iter().next() else { return Err(FeatureError::new(RIB_NEEDS_OPEN_SKETCH)) };
    let segs: Vec<ProfileSeg> = chain
        .iter()
        .map(|s| {
            let mut p = profile_seg(s)?;
            p.tag = format!("{}r", p.tag);
            Ok(p)
        })
        .collect::<Result<_, FeatureError>>()?;
    let (a, b) = (segs[0].start(), segs[segs.len() - 1].end());
    if a.dist(b) < 1e-9 {
        return Err(FeatureError::new(RIB_NEEDS_OPEN_SKETCH));
    }
    let (lo, hi) = super::body_box(&body);
    let big = 4.0 * lo.dist(hi) + 4.0 * a.dist(b) + 100.0;
    let tan_a = segs[0].travel_tangent(segs[0].t_start()).normalize();
    let tan_b = segs[segs.len() - 1].travel_tangent(segs[segs.len() - 1].t_end()).normalize();
    let left = (b - a).normalize().perp();
    let last_tag = chain.last().map(|s| s.tag.clone()).unwrap_or_default();
    let far_tag = format!("{last_tag}l");
    let sides: [f64; 2] = if *flip { [-1.0, 1.0] } else { [1.0, -1.0] };
    for s in sides {
        let n: Vec2 = left * s;
        let (a0, b0) = (a - tan_a * big, b + tan_b * big);
        // a line at either chain end is itself extended (one coplanar face
        // `S.<entity>r` wall to wall); curved ends get a tangent extension
        let mut mid = segs.clone();
        let is_line = |p: &ProfileSeg| matches!(p.curve, ck_geom::curve2::Curve2::Line { .. });
        let mut loop_segs = vec![];
        if is_line(&mid[0]) {
            let tag = mid[0].tag.clone();
            let e = mid[0].end();
            mid[0] = ProfileSeg::line(&tag, a0, e);
        } else {
            loop_segs.push(ProfileSeg::line("x0", a0, a));
        }
        let n_mid = mid.len();
        let tail_line = is_line(&mid[n_mid - 1]);
        if tail_line {
            let tag = mid[n_mid - 1].tag.clone();
            let st = mid[n_mid - 1].start();
            mid[n_mid - 1] = ProfileSeg::line(&tag, st, b0);
        }
        loop_segs.extend(mid.into_iter());
        if !tail_line {
            loop_segs.push(ProfileSeg::line("x1", b, b0));
        }
        loop_segs.push(ProfileSeg::line("x2", b0, b0 + n * big));
        loop_segs.push(ProfileSeg::line(&far_tag, b0 + n * big, a0 + n * big));
        loop_segs.push(ProfileSeg::line("x3", a0 + n * big, a0));
        let mut outer = ProfileLoop::new(loop_segs);
        if outer.signed_area() < 0.0 {
            outer = ProfileLoop::new(outer.segs.iter().rev().map(|x| x.reversed()).collect());
        }
        let profile = Profile::new(sk.frame, vec![ProfileRegion { outer, holes: vec![] }]);
        let Ok(tool) = extrude(&f.id, &profile, ExtrudeOpts::mid(t)) else { continue };
        let cut = match ck_ops::boolean::boolean(&tool, &body, ck_ops::boolean::Op::Cut, super::TOL) {
            Ok(c) if !c.no_effect => c.body,
            r => {
                if std::env::var_os("CK_RIB_DBG").is_some() {
                    eprintln!("RIB side {s}: boolean {:?}", r.map(|c| c.no_effect).map_err(|e| format!("{e:?}")));
                }
                continue;
            }
        };
        if std::env::var_os("CK_RIB_DBG").is_some() {
            let comps = ck_topo::region::components(&cut);
            eprintln!("RIB side {s}: cut faces {} comps {:?} issues {:?}", cut.num_faces(), comps.iter().map(|c| c.len()).collect::<Vec<_>>(), ck_topo::check::check(&cut, 1e-6).iter().take(6).collect::<Vec<_>>());
        }
        // pieces that never reach a far face are enclosed by the body
        let mut keep: Vec<FaceId> = vec![];
        for comp in ck_topo::region::components(&cut) {
            let far = comp.iter().any(|&fid| {
                let n = &cut.face(fid).name;
                n.feature == f.id && matches!(&n.role, Role::Side(tag) if is_far(tag))
            });
            let own = comp.iter().any(|&fid| cut.face(fid).name.feature == f.id);
            if !far && own {
                keep.extend(comp);
            }
        }
        if keep.is_empty() {
            continue;
        }
        let order: Vec<FaceId> = cut.face_ids().filter(|x| keep.contains(x)).collect();
        let mut piece: Body = ck_ops::boolean::assemble::rebuild_in_order(&cut, &order);
        // the `~k` of the intermediate cut belong to its other pieces
        for fc in piece.faces.iter_mut().filter(|x| x.alive && x.name.feature == f.id) {
            fc.name.region = None;
        }
        // contact: part of the chain itself must border the rib piece
        // (pinned: a chain outside the body whose line would reach it
        // is `ribNoContact`)
        let fr = sk.frame;
        let inplane = fr.x * n.x + fr.y * n.y;
        let cont = ck_ops::contain::Containment::new(&piece, 1e-7);
        let touches = segs.iter().any(|sg| {
            (0..=8).any(|k| {
                let tt = sg.t0 + (sg.t1 - sg.t0) * k as f64 / 8.0;
                let q = sg.curve.eval(tt);
                let p = fr.o + fr.x * q.x + fr.y * q.y + inplane * 1e-5;
                matches!(cont.classify_point(p, 1e-7), ck_ops::contain::Loc::In | ck_ops::contain::Loc::On)
            })
        });
        if !touches {
            continue;
        }
        br.combine(&piece, Combine::Add, false)?;
        if let Some(b) = br.body.as_mut() {
            renumber_duplicates(b);
        }
        return Ok(());
    }
    Err(FeatureError::new(RIB_NO_CONTACT))
}

/// A face split by the union (the shell floor cut by the rib) keeps its old
/// `~k` on both pieces; pinned: the whole family is re-ranked by control-box
/// centre (`F2.C0~1` outer, `~2`/`~3` the floor pieces).
fn renumber_duplicates(b: &mut Body) {
    use std::collections::BTreeMap;
    let mut fam: BTreeMap<(String, String, String), Vec<FaceId>> = BTreeMap::new();
    for fid in b.face_ids() {
        let n = &b.face(fid).name;
        let base = ck_topo::body::FaceName { region: None, ..n.clone() };
        fam.entry((base.id(), String::new(), String::new())).or_default().push(fid);
    }
    for (_, fs) in fam {
        let ids: Vec<String> = fs.iter().map(|&x| b.face_id_str(x)).collect();
        let dup = ids.iter().enumerate().any(|(i, a)| ids[..i].contains(a));
        if dup {
            for &x in &fs {
                b.faces[x.idx()].name.region = None;
            }
            b.number_split_faces();
        }
    }
}
