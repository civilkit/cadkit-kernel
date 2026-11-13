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
//! Options (all optional, defaults give the behaviour above):
//! * `side` `both` (default) | `right` | `left`: the thickness is centred on the sketch plane, or all on the side the
//!   sketch normal points to (`right`: `C0` on the plane, `C1` at +thickness) or against it (`left`);
//! * `draft` (degrees, 0 to 45): the two side faces taper toward the free edge, the wall being `thickness` thick at the
//!   chain and wider by `2 * tan(draft)` per unit of distance toward the body (one straight line only: the tool is then
//!   a prism of the trapezoid cross-section, not the flat profile's mid extrude);
//! * `fillet` (radius, below half the thickness): the edges between the free face and the two sides are rounded after
//!   the rib is united (the kernel fillet);
//! * `target`: the body (or bodies) the rib extends to, like extrude's `target`; the other bodies are left alone.
//!
//! The extension of the chain stops at the outer wall: a piece of the tool enclosed in a void of the body (the bore of
//! a boss the line runs through) never counts as the rib: only the pieces that border the chain itself do.
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
use ck_json::Json;
use ck_topo::body::{Body, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

pub const RIB_NEEDS_OPEN_SKETCH: &str = "ribNeedsOpenSketch";
pub const RIB_NO_CONTACT: &str = "ribNoContact";

fn is_far(tag: &str) -> bool {
    tag.starts_with('x') || tag.ends_with('l')
}

fn invalid(detail: &str, hint: &str) -> FeatureError {
    FeatureError::new(codes::INVALID_VALUE).with("detail", Json::Str(detail.into())).with("hint", Json::Str(hint.into()))
}

pub fn exec_rib(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let Some(t) = f.raw.get("target").filter(|t| !matches!(t, Json::Null)) else { return rib_on(br, state, f) };
    if br.body.is_none() {
        return Err(FeatureError::new(codes::NOTHING_TO_CUT));
    }
    let faces = br.target_faces(t)?;
    let before = br.body.clone();
    let (mut changed, mut res) = (false, Ok(()));
    for face in &faces {
        let one = match super::blends::on_lump_of_face(br, face, |b| rib_on(b, state, f)) {
            Some(r) => r,
            None => rib_on(br, state, f),
        };
        match one {
            Ok(()) => changed = true,
            Err(e) if e.code == RIB_NO_CONTACT && faces.len() > 1 => {}
            Err(e) => {
                res = Err(e);
                break;
            }
        }
    }
    if res.is_ok() && !changed {
        res = Err(FeatureError::new(RIB_NO_CONTACT));
    }
    if res.is_err() {
        br.body = before;
    }
    res
}

fn rib_on(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
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
    let side = match f.raw.get("side") {
        None | Some(Json::Null) => 0,
        Some(v) => match v.as_str() {
            Some("both") => 0,
            Some("right") => 1,
            Some("left") => -1,
            _ => return Err(invalid("side is both, left or right", "set side to \"both\" (centred on the sketch plane), \"right\" (along the sketch normal) or \"left\" (against it)")),
        },
    };
    let draft = state.num(f, "draft", 0.0)?;
    if !draft.is_finite() || !(0.0..=45.0).contains(&draft) {
        return Err(invalid("draft is an angle in degrees from 0 to 45", "give the taper of the two side faces as an angle between 0 and 45 degrees, 0 for none"));
    }
    let fillet = state.num(f, "fillet", 0.0)?;
    if !fillet.is_finite() || fillet < 0.0 || (fillet > 0.0 && fillet >= t / 2.0 - 1e-9) {
        return Err(invalid("fillet is a radius below half the thickness", "give a radius above 0 and below half the rib thickness, 0 for none"));
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
    let drafted = draft > 0.0;
    if drafted && !(segs.len() == 1 && matches!(segs[0].curve, ck_geom::curve2::Curve2::Line { .. })) {
        return Err(FeatureError::new("draftUnsupported")
            .with("detail", Json::Str("a drafted rib needs a sketch that is one straight line".into()))
            .with("hint", Json::Str("draw the rib as one straight line, or leave draft at 0".into())));
    }
    // the thickness range across the sketch plane (the sketch normal is +): centred, or all on one side
    let (ylo, yhi) = match side {
        0 => (-t / 2.0, t / 2.0),
        1 => (0.0, t),
        _ => (-t, 0.0),
    };
    let (lo, hi) = super::body_box(&body);
    let big = 4.0 * lo.dist(hi) + 4.0 * a.dist(b) + 100.0;
    let tan_a = segs[0].travel_tangent(segs[0].t_start()).normalize();
    let tan_b = segs[segs.len() - 1].travel_tangent(segs[segs.len() - 1].t_end()).normalize();
    let left = (b - a).normalize().perp();
    let last_tag = chain.last().map(|s| s.tag.clone()).unwrap_or_default();
    let far_tag = format!("{last_tag}l");
    let sides: [f64; 2] = if *flip { [-1.0, 1.0] } else { [1.0, -1.0] };
    // a chain end that meets an edge of the body exactly (a line drawn to a corner) leaves the enclosed piece joined
    // to the outside along that edge: the end is then tried with its extension turned a little, either way
    let rot = |v: Vec2, th: f64| Vec2::new(v.x * th.cos() - v.y * th.sin(), v.x * th.sin() + v.y * th.cos());
    const EPS: f64 = 2e-3;
    let all_tilts: [(f64, f64); 9] = [(0.0, 0.0), (0.0, EPS), (0.0, -EPS), (EPS, 0.0), (-EPS, 0.0), (EPS, EPS), (EPS, -EPS), (-EPS, EPS), (-EPS, -EPS)];
    // a drafted rib is one prism along the line: no tilted attempts
    let tilts: &[(f64, f64)] = if drafted { &all_tilts[..1] } else { &all_tilts };
    for ((ta, tb), s) in tilts.iter().copied().flat_map(|t| sides.into_iter().map(move |s| (t, s))) {
        let n: Vec2 = left * s;
        let (tan_a, tan_b) = (if ta != 0.0 { rot(tan_a, ta) } else { tan_a }, if tb != 0.0 { rot(tan_b, tb) } else { tan_b });
        let tool = if drafted {
            let Ok(tool) = drafted_tool(&f.id, sk.frame, (a, b), n, segs[0].tag.clone(), (ylo, yhi), draft.to_radians().tan(), big) else { continue };
            tool
        } else {
            let (a0, b0) = (a - tan_a * big, b + tan_b * big);
            // a line at either chain end is itself extended (one coplanar face
            // `S.<entity>r` wall to wall); curved ends get a tangent extension
            let mut mid = segs.clone();
            let is_line = |p: &ProfileSeg| matches!(p.curve, ck_geom::curve2::Curve2::Line { .. });
            let mut loop_segs = vec![];
            if is_line(&mid[0]) && ta == 0.0 {
                let tag = mid[0].tag.clone();
                let e = mid[0].end();
                mid[0] = ProfileSeg::line(&tag, a0, e);
            } else {
                loop_segs.push(ProfileSeg::line("x0", a0, a));
            }
            let n_mid = mid.len();
            let tail_line = is_line(&mid[n_mid - 1]) && tb == 0.0;
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
            let opts = ExtrudeOpts { s0: ylo, s1: yhi, dir: None, draft: 0.0 };
            let Ok(tool) = extrude(&f.id, &profile, opts) else { continue };
            tool
        };
        let cut = match ck_ops::boolean::boolean(&tool, &body, ck_ops::boolean::Op::Cut, super::TOL) {
            Ok(c) if !c.no_effect && c.mismatch.is_none() => c.body,
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
        let fr = sk.frame;
        let inplane = fr.x * n.x + fr.y * n.y;
        // contact: part of the chain itself must border the rib piece
        // (pinned: a chain outside the body whose line would reach it
        // is `ribNoContact`)
        let touches = |piece: &Body| -> bool {
            let cont = ck_ops::contain::Containment::new(piece, 1e-7);
            segs.iter().any(|sg| {
                (0..=8).any(|k| {
                    let tt = sg.t0 + (sg.t1 - sg.t0) * k as f64 / 8.0;
                    let q = sg.curve.eval(tt);
                    let p = fr.o + fr.x * q.x + fr.y * q.y + inplane * 1e-5;
                    matches!(cont.classify_point(p, 1e-7), ck_ops::contain::Loc::In | ck_ops::contain::Loc::On)
                })
            })
        };
        let mut cands: Vec<Vec<FaceId>> = vec![];
        for comp in ck_topo::region::components(&cut) {
            let far = comp.iter().any(|&fid| {
                let n = &cut.face(fid).name;
                n.feature == f.id && matches!(&n.role, Role::Side(tag) if is_far(tag))
            });
            let own = comp.iter().any(|&fid| cut.face(fid).name.feature == f.id);
            if !far && own {
                cands.push(comp);
            }
        }
        // the tool also fills a void the line runs through (the bore of a boss it ends in): such a piece lies away from
        // the chain, so with more than one candidate only those that border the chain are the rib
        if cands.len() > 1 {
            cands.retain(|c| {
                let order: Vec<FaceId> = cut.face_ids().filter(|x| c.contains(x)).collect();
                touches(&ck_ops::boolean::assemble::rebuild_in_order(&cut, &order))
            });
        }
        let keep: Vec<FaceId> = cands.into_iter().flatten().collect();
        if keep.is_empty() {
            continue;
        }
        let order: Vec<FaceId> = cut.face_ids().filter(|x| keep.contains(x)).collect();
        let mut piece: Body = ck_ops::boolean::assemble::rebuild_in_order(&cut, &order);
        // the `~k` of the intermediate cut belong to its other pieces
        for fc in piece.faces.iter_mut().filter(|x| x.alive && x.name.feature == f.id) {
            fc.name.region = None;
        }
        if !touches(&piece) {
            continue;
        }
        // the round is made on the rib piece, before it joins the body: its free edges end on the faces the rib stands on, and a
        // round that is carried on after the union would cut a notch into those faces
        if fillet > 0.0 {
            let mut tmp = Brep::new();
            tmp.body = Some(piece);
            round_free_edge(&mut tmp, &f.id, fillet)?;
            br.spans.extend(tmp.spans.drain(..));
            piece = tmp.body.take().expect("the round keeps a body");
        }
        br.combine(&piece, Combine::Add, false)?;
        if let Some(b) = br.body.as_mut() {
            super::renumber_duplicates(b);
        }
        return Ok(());
    }
    Err(FeatureError::new(RIB_NO_CONTACT))
}

/// The tool of a drafted rib: a prism along the (straight) chain whose cross-section is the trapezoid from the free edge
/// (`ylo..yhi` across the sketch plane) out to `big` toward the body, each side leaving at `tan_a`. The free face keeps the
/// chain's tag; the far end and the two ends of the prism are named as far faces (they never survive), the two sloped
/// faces take the names of the sides, `C0` (low) and `C1` (high).
#[allow(clippy::too_many_arguments)]
fn drafted_tool(feature: &str, fr: ck_core::linalg::Frame, (a, b): (Vec2, Vec2), n: Vec2, tag: String, (ylo, yhi): (f64, f64), tan_a: f64, big: f64) -> Result<Body, String> {
    let mid = (a + b) * 0.5;
    let n3 = fr.x * n.x + fr.y * n.y;
    let o = fr.o + fr.x * mid.x + fr.y * mid.y;
    let pf = ck_core::linalg::Frame { o, x: n3, y: fr.z, z: n3.cross(fr.z) };
    let (p0, p1) = (Vec2::new(0.0, ylo), Vec2::new(0.0, yhi));
    let (p2, p3) = (Vec2::new(big, yhi + big * tan_a), Vec2::new(big, ylo - big * tan_a));
    let mut outer = ProfileLoop::new(vec![ProfileSeg::line(&tag, p0, p1), ProfileSeg::line("d1", p1, p2), ProfileSeg::line("x2", p2, p3), ProfileSeg::line("d0", p3, p0)]);
    if outer.signed_area() < 0.0 {
        outer = ProfileLoop::new(outer.segs.iter().rev().map(|x| x.reversed()).collect());
    }
    let profile = Profile::new(pf, vec![ProfileRegion { outer, holes: vec![] }]);
    let half = a.dist(b) / 2.0 + big;
    let mut tool = extrude(feature, &profile, ExtrudeOpts { s0: -half, s1: half, dir: None, draft: 0.0 })?;
    for fc in tool.faces.iter_mut().filter(|x| x.alive) {
        let role = fc.name.role.clone();
        fc.name = match role {
            Role::Cap(k) => FaceName::side(feature, &format!("x{}", 4 + k)),
            Role::Side(t) if t == "d0" => FaceName::cap(feature, 0, None),
            Role::Side(t) if t == "d1" => FaceName::cap(feature, 1, None),
            _ => continue,
        };
    }
    Ok(tool)
}

/// The fillet of the rib's free edge: the edges between a free face (`S.<entity>r`) and a side (`C0`, `C1`).
fn round_free_edge(br: &mut Brep, id: &str, r: f64) -> Result<(), FeatureError> {
    let body = br.body.as_ref().ok_or_else(|| FeatureError::new("missingEdge"))?;
    let free = |fid: FaceId| matches!(&body.face(fid).name, n if n.feature == id && matches!(&n.role, Role::Side(t) if t.ends_with('r')));
    let side = |fid: FaceId| matches!(&body.face(fid).name, n if n.feature == id && matches!(n.role, Role::Cap(_)));
    let mut sel = vec![];
    for e in body.edge_ids() {
        let fs = body.edge_faces(e);
        if fs.len() != 2 || !((free(fs[0]) && side(fs[1])) || (free(fs[1]) && side(fs[0]))) {
            continue;
        }
        let ed = body.edge(e);
        let at = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        sel.push(super::blends::forward_first(body, &body.face_id_str(fs[0]), &body.face_id_str(fs[1]), at));
    }
    if sel.is_empty() {
        return Err(FeatureError::new("missingEdge"));
    }
    super::blends::fillet_picks(br, id, &sel, r)
}
