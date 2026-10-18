//! Blend construction (ck-ops/src/blend/build.rs) — FIRST PASS: the
//! simplest case only, a constant-radius fillet along a straight edge
//! between two planar faces (convex or concave), built as a boolean with
//! a "corner prism" tool whose arc face becomes the blend face `B<k>`.
//!
//! Oracle evidence (probes on box F2, fillet feature F3):
//! - `fillet r=2` on `F2.C1|F2.S.e1`: faces `C0, S.e3` (untouched, original
//!   order), `C1, S.e1` (the edge's faces, ref order), `S.e4, S.e2` (end
//!   faces at the edge's start, end vertex), `F3.B0` (cylinder, meta
//!   `{feature:"F3"}`); end-face edges are circle arcs c = axis point,
//!   n = edge direction (sign from the end face), r = 2.
//! - two edges → `B0`, `B1` in ref order; touched faces in first-touch order.
//! - errors: radius 0 → `invalidValue`; no / unknown edge → `missingEdge`;
//!   radius 30 → `blendOverflow` with detail
//!   `"F3.B0: spring leaves its face away from its edges"`; the body is
//!   left unchanged in all error cases.
//! - GUESS/known mismatch: the oracle blends IN PLACE (blend/inplace.rs):
//!   surviving edges keep their arena number in the wire key `#n`, split
//!   pieces and new edges are appended (fillet probe: #0..#4, #6, #8..#11,
//!   #14, #16..#19). This boolean-based pass renumbers edges in face-walk
//!   order; face ids, order, geometry and massProps match.

use super::BlendError;
use crate::boolean::assemble::{build, select};
use crate::boolean::classify::{point_in_face, Where};
use crate::boolean::imprint::imprint;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

/// Faces named `id`; a face an earlier tool split into regions (`F4.C1~1`, `~2`) is still
/// named by its base id in the selection.
fn faces_named(b: &Body, id: &str) -> Vec<FaceId> {
    if let Some(f) = b.find_face(id) {
        return vec![f];
    }
    let Some(mut want) = FaceName::parse(id) else { return vec![] };
    want.region = None;
    b.face_ids()
        .filter(|&f| {
            let mut n = b.face(f).name.clone();
            n.region = None;
            n == want
        })
        .collect()
}

/// Edge between the two named faces closest to `at` (fillet `edges[]` ref).
///
/// A selection whose face names no longer fit the geometry (a saved document naming a wall that is now the
/// other half of a split rim, so the numbers shifted) falls back, when the named pair does border an edge but
/// not near `at`, to the edge through `at` between faces of the same two features.
pub fn find_edge(b: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    let named = |id: &str| faces_named(b, id);
    let best = find_edge_named(b, fa, fb, at, &named);
    let dist = |e: EdgeId| {
        let ed = b.edge(e);
        crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
    };
    if best.is_some_and(|e| dist(e) > 1e-4 * (1.0 + at.len())) {
        if let Some(e) = find_edge_by_position(b, fa, fb, at) {
            return Some(e);
        }
    }
    best
}

/// The id without its region suffix (`F4.S.e1~2` -> `F4.S.e1`).
fn family_id(id: &str) -> &str {
    match id.rfind('~') {
        Some(i) if i + 1 < id.len() && id[i + 1..].bytes().all(|c| c.is_ascii_digit()) && !id.contains('@') => &id[..i],
        _ => id,
    }
}

/// The edge through `at` between faces of the two features the names belong to. For a saved selection whose
/// face names no longer fit (the `~k` or wall numbers shifted when a boolean merged the pieces of a cone, or an
/// earlier kernel numbered them differently) but whose pick point still sits on the edge. Both names must still
/// belong to a face (or a face's family, or an absorbed name), so a misspelt face stays `missingEdge`.
pub fn find_edge_by_position(b: &Body, fa: &str, fb: &str, at: Vec3) -> Option<EdgeId> {
    let exists = |id: &str| {
        b.find_face(id).is_some()
            || b.face_ids().any(|f| family_id(&b.face_id_str(f)) == family_id(id) || b.face(f).absorbed.iter().any(|a| family_id(a) == family_id(id)))
    };
    if !exists(fa) || !exists(fb) {
        return None;
    }
    let (ha, hb) = (FaceName::parse(fa)?, FaceName::parse(fb)?);
    let scale = 1.0 + at.len();
    let by_feature = |f: FaceId, feat: &str| b.face(f).name.feature == feat;
    let dist = |e: EdgeId| {
        let ed = b.edge(e);
        crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
    };
    b.edge_ids()
        .filter(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2
                && ((by_feature(fs[0], &ha.feature) && by_feature(fs[1], &hb.feature)) || (by_feature(fs[1], &ha.feature) && by_feature(fs[0], &hb.feature)))
                && dist(e) <= 1e-6 * scale
        })
        .min_by(|&x, &y| dist(x).partial_cmp(&dist(y)).unwrap())
}

fn find_edge_named(b: &Body, fa: &str, fb: &str, at: Vec3, named: &dyn Fn(&str) -> Vec<FaceId>) -> Option<EdgeId> {
    let (fs_a, fs_b) = (named(fa), named(fb));
    b.edge_ids()
        .filter(|&e| {
            let fs = b.edge_faces(e);
            fs.len() == 2 && ((fs_a.contains(&fs[0]) && fs_b.contains(&fs[1])) || (fs_a.contains(&fs[1]) && fs_b.contains(&fs[0])))
        })
        .min_by(|&x, &y| {
            let d = |e: EdgeId| {
                let ed = b.edge(e);
                crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), at).1
            };
            d(x).partial_cmp(&d(y)).unwrap()
        })
}

/// Constant-radius fillets of `edges` (face-name pairs + pick point, in
/// ref order), blend faces `B0, B1, …` of `feature`.
///
/// Each pair is first normalised so that the face whose loop runs the edge
/// FORWARD comes first (oracle box probes, bracket F12: selection
/// `["F2.S.e8","F2.S.e9"]` lists `F12.B0|F2.S.e9` before `…|F2.S.e8`, and
/// the touched faces come e9, e8). The result is written in the original's
/// IN-PLACE arena layout ([`in_place_layout`]), so wire `#n` keep the
/// input's numbers.
pub fn fillet(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Fillet(r, FilletProfile::Circular), feature, tol)
}

/// Equal-distance chamfers (`Chamfer(d, d)` in the trace) of `edges`.
pub fn chamfer(b: &Body, edges: &[(String, String, Vec3)], d: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Chamfer(d, ChamferLeg::Dist(d)), feature, tol)
}

/// Chamfer with distance `d1` on the FIRST face of every edge pick and a second leg given by a
/// distance or by the angle between the chamfer and the first face (oracle `distances` / `angle`).
pub fn chamfer_legs(b: &Body, edges: &[(String, String, Vec3)], d1: f64, leg: ChamferLeg, feature: &str, tol: f64) -> Result<Body, BlendError> {
    blend_edges(b, edges, Spec::Chamfer(d1, leg), feature, tol)
}

/// What one blend cuts: a rolling-ball fillet of radius `r` or an
/// equal-distance chamfer of leg `d`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Spec {
    Fillet(f64, FilletProfile),
    Chamfer(f64, ChamferLeg),
}

/// Second leg of a chamfer: a distance on the second face, or the angle (radians) between the
/// chamfer face and the first face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChamferLeg {
    Dist(f64),
    Angle(f64),
    /// [`ChamferLeg::Angle`] with the pick's faces listed the other way round: the angle is to
    /// the second face, the distance on it.
    AngleRev(f64),
}

/// Section of a fillet (`fillet.profile`): a circular arc, a conic (rational
/// quadratic through the circular fillet's contact points, `rho` in (0, 1)),
/// a circular arc of the given CHORD width, or the curvature-continuous
/// spiral - arc - spiral of `blend::g2` (`shape` in [0.1, 1]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilletProfile {
    Circular,
    Conic(f64),
    Chord,
    Curvature(f64),
}

impl Spec {
    fn size(self) -> f64 {
        match self {
            Spec::Fillet(r, _) => r,
            Spec::Chamfer(a, ChamferLeg::Dist(b)) => a.max(b),
            Spec::Chamfer(a, ChamferLeg::Angle(_) | ChamferLeg::AngleRev(_)) => a,
        }
    }
}

/// Shared fillet / chamfer driver (see [`fillet`]).
pub fn blend_edges(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    let first = blend_edges_inner(b, edges, spec, feature, tol);
    // A selection that is a chain of tangent edges (a rounded outline picked edge by edge) fails when a tool runs on past
    // the junction into the arc's band: retry with the ends of tangent-continuing edges stopping flush. Selections that
    // worked keep their result.
    if first.is_err() && edges.len() > 1 && !TANGENT_FLUSH.with(|c| c.get()) {
        TANGENT_FLUSH.with(|c| c.set(true));
        let second = blend_edges_inner(b, edges, spec, feature, tol);
        TANGENT_FLUSH.with(|c| c.set(false));
        if second.is_ok() {
            return second;
        }
    }
    first
}

fn blend_edges_inner(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    let r = spec.size();
    if !(r > 0.0) || !r.is_finite() {
        return Err(BlendError::code("invalidValue"));
    }
    if edges.is_empty() {
        return Err(BlendError::code("missingEdge"));
    }
    if matches!(spec, Spec::Fillet(..)) {
        let free = edges.iter().any(|(fa, fb, at)| find_edge(b, fa, fb, *at).is_some_and(|e| super::freeform::needs_freeform(b, e)));
        if free {
            return super::freeform::fillet_freeform(b, edges, r, feature, tol);
        }
    }
    // a circle split in arcs by a seam (a lofted cone's rim is two arcs on one surface) is one smooth edge: selecting
    // one arc selects the whole ring, so the blend runs all the way round instead of stopping at the seam vertex
    let expanded;
    let edges = if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && EDGE_RADII.with(|c| c.borrow().is_none()) {
        expanded = with_ring_siblings(b, edges);
        &expanded[..]
    } else {
        edges
    };
    if edges.len() > 1 {
        // Oracle (box probes): the selection runs in ROUNDS, each round one
        // blend op (untouched faces, touched in first-touch order, new
        // faces) numbering its edges in place; blend names stay
        // selection-indexed. An edge joins the round unless it conflicts
        // with an edge already in it or with an earlier-selected edge that
        // was deferred. Chamfers conflict with any edge sharing an end
        // vertex; fillets only with one they PRECEDE in the chain
        // (v1 -> v0 links; positions relative to the chain's first
        // selected edge, so a closed ring is a single op).
        let fwd: Vec<(String, String, Vec3)> = edges.iter().map(|(fa, fb, at)| forward_first(b, fa, fb, *at)).collect();
        let eids: Vec<EdgeId> = fwd
            .iter()
            .map(|(fa, fb, at)| find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge")))
            .collect::<Result<_, BlendError>>()?;
        let ends: Vec<[usize; 2]> = eids.iter().map(|&e| [b.edge(e).v0.idx(), b.edge(e).v1.idx()]).collect();
        let n = edges.len();
        // two spline sections (conic / curvature) meeting at a vertex need a corner patch: the
        // spline x spline intersection is not closed by the marcher (documented gap)
        if matches!(spec, Spec::Fillet(_, FilletProfile::Conic(_) | FilletProfile::Curvature(_)))
            && (0..n).any(|i| (i + 1..n).any(|j| ends[i].iter().any(|v| ends[j].contains(v))))
        {
            return Err(BlendError::detail("blendUnsupported", "adjacent edges with a conic or curvature profile"));
        }
        let rounds: Vec<Vec<usize>> = match spec {
            Spec::Chamfer(..) => {
                let mut done = vec![false; n];
                let mut rounds = vec![];
                // G1 (tangent) edges go first: their sliver cut is only
                // stable on a body no other chamfer has touched yet. Blend
                // names stay selection-indexed.
                let tang: Vec<bool> = eids.iter().map(|&e| is_tangent_edge(b, e)).collect();
                let mut order: Vec<usize> = (0..n).collect();
                order.sort_by_key(|&i| !tang[i]);
                while done.iter().any(|d| !d) {
                    // an edge joins the round unless it shares a vertex with an
                    // edge in it or with an earlier-selected deferred edge
                    let mut round: Vec<usize> = vec![];
                    let mut blocked: Vec<usize> = vec![];
                    for &i in &order {
                        if done[i] {
                            continue;
                        }
                        if blocked.iter().any(|&j| ends[i].iter().any(|v| ends[j].contains(v))) {
                            blocked.push(i);
                        } else {
                            round.push(i);
                            blocked.push(i);
                        }
                    }
                    for &i in &round {
                        done[i] = true;
                    }
                    rounds.push(round);
                }
                rounds
            }
            Spec::Fillet(..) => {
                // consecutive groups of the selection: an edge joins the
                // current group when it is not in the group's chain, or lies
                // after (v1 -> v0 links; positions relative to the chain's
                // first selected edge) at least one member of it
                let pos = chain_positions(&ends);
                // the three edges of a vertex blend are one op
                let mates = |i: usize| -> Vec<usize> {
                    let mut out = vec![];
                    for v in ends[i] {
                        let at: Vec<usize> = (0..n).filter(|&j| ends[j].contains(&v)).collect();
                        if at.len() == 3 {
                            out.extend(at.into_iter().filter(|&j| j != i));
                        }
                    }
                    out
                };
                let mut rounds: Vec<Vec<usize>> = vec![];
                let mut cur: Vec<usize> = vec![];
                for i in 0..n {
                    let same: Vec<i64> = cur.iter().filter(|&&j| pos[j].0 == pos[i].0).map(|&j| pos[j].1).collect();
                    if cur.is_empty() || same.is_empty() || pos[i].1 > *same.iter().min().unwrap() || mates(i).iter().any(|j| cur.contains(j)) {
                        cur.push(i);
                    } else {
                        rounds.push(std::mem::take(&mut cur));
                        cur.push(i);
                    }
                }
                rounds.push(cur);
                rounds
            }
        };
        // junction vertices of the whole selection (an earlier round shortens the edges that follow)
        let mut junc = if matches!(spec, Spec::Fillet(..)) {
            junction_vertices(b, &eids, tol)
        } else if TANGENT_FLUSH.with(|c| c.get()) {
            Junc { ext: vec![], flush: tangent_flush_vertices(b, &eids), mitre: vec![] }
        } else {
            Junc::default()
        };
        junc.mitre = mitre_pairs(b, &eids);
        let mut out = b.clone();
        let mut earlier: Vec<String> = vec![];
        // chamfer: arcs of one split ring (a lofted cap rim cut in two at the
        // seam) are ONE revolved cut, named for the first arc
        let (ring_lead, ring_skip) = if chamfer_only(spec) || matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) { split_rings(b, &eids) } else { (vec![false; n], vec![false; n]) };
        for round in rounds {
            let round: Vec<usize> = round.into_iter().filter(|&i| !ring_skip[i]).collect();
            if round.is_empty() {
                continue;
            }
            let rings: Vec<bool> = round.iter().map(|&i| ring_lead[i]).collect();
            let idx: Vec<u32> = round.iter().map(|&i| i as u32).collect();
            let sel: Vec<(String, String, Vec3)> = round.iter().map(|&i| fwd[i].clone()).collect();
            let (raw, round_refs, touched) = blend_edges_at(&out, &sel, spec, feature, tol, &idx, &earlier, &rings, &junc)?;
            if std::env::var_os("CK_BLEND_DEBUG").is_some() {
                eprintln!("round {:?}: touched {:?} -> faces {:?}", round, touched, raw.face_ids().map(|f| raw.face_id_str(f)).collect::<Vec<_>>());
            }
            for t in touched {
                if !earlier.contains(&t) {
                    earlier.push(t);
                }
            }
            if matches!(spec, Spec::Chamfer(..)) {
                // chamfers number their edges round by round
                out = in_place_layout(&out, &raw, &round_refs, tol);
            } else {
                // fillets: one global in-place numbering (junction layout)
                out = raw;
            }
        }
        if matches!(spec, Spec::Fillet(..)) {
            // three fillets meeting at a convex planar corner: the rolling
            // ball's sphere patch `V<k>` replaces the miter
            let mut corners: Vec<Vec3> = vec![];
            for corner in super::corner::find_corner(b, &eids) {
                // per-edge radii: the ball patch needs one radius at the corner
                let rc = EDGE_RADII.with(|c| {
                    c.borrow().as_ref().map(|v| {
                        let rs: Vec<f64> = corner.edges.iter().filter_map(|e| eids.iter().position(|x| x == e)).map(|i| v[i]).collect();
                        if rs.len() == 3 && rs.iter().all(|x| (x - rs[0]).abs() <= 1e-12 * (1.0 + rs[0])) { Some(rs[0]) } else { None }
                    })
                });
                let r = match rc {
                    Some(Some(x)) => x,
                    Some(None) => continue,
                    None => r,
                };
                let name = FaceName::other(feature, &format!("V{}", corners.len()));
                let Ok(tool) = super::corner::sphere_corner_tool(b, &corner, r, feature, &name, tol) else { continue };
                let op = if corner.concave { Op::Union } else { Op::Cut };
                let Ok((mut cut, _)) = apply_tool(&out, &tool, op, &name, vec![], tol) else { continue };
                super::corner::fix_sphere_pcurves(&mut cut, &name.id());
                // faces: the input order, then the new vertex blends
                let order: Vec<FaceId> = {
                    let key = |f: FaceId| -> usize {
                        let n = cut.face_id_str(f);
                        out.face_ids().position(|g| out.face_id_str(g) == n).unwrap_or(usize::MAX)
                    };
                    let mut o: Vec<FaceId> = cut.face_ids().collect();
                    o.sort_by_key(|&f| key(f));
                    o
                };
                out = crate::boolean::assemble::rebuild_in_order(&cut, &order);
                corners.push(b.vertex(corner.vertex).p);
            }
            let mut all: Vec<BlendRef> = vec![];
            for (k, (fa, fb, at)) in fwd.iter().enumerate() {
                let e = find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
                let ed = b.edge(e);
                all.push(BlendRef { name: FaceName::blend(feature, k as u32).id(), faces: [fa.clone(), fb.clone()], ends: [b.vertex(ed.v0).p, b.vertex(ed.v1).p] });
            }
            out = in_place_layout_opts(b, &out, &all, tol, &LayoutOpts { corners, ..Default::default() });
        }
        return Ok(out);
    }
    let idx: Vec<u32> = (0..edges.len() as u32).collect();
    let (raw, refs, _) = blend_edges_at(b, edges, spec, feature, tol, &idx, &[], &[], &Junc::default())?;
    Ok(in_place_layout(b, &raw, &refs, tol))
}

thread_local! {
    /// Radius of the fillet of selection index `k` (see [`fillet_unequal`]).
    static EDGE_RADII: std::cell::RefCell<Option<Vec<f64>>> = const { std::cell::RefCell::new(None) };
}

/// Circular fillets with a radius per edge (`radii[k]` for `edges[k]`). Every edge is built as the
/// ordinary constant-radius blend (analytic cylinders, the same rounds and junction rules); where
/// edges of different radii meet, the blend faces end in a mitre, which is not tangent-continuous.
/// Three convex or concave edges that share a radius at their vertex still get the sphere patch.
pub fn fillet_unequal(b: &Body, edges: &[(String, String, Vec3)], radii: &[f64], feature: &str, tol: f64) -> Result<Body, BlendError> {
    if radii.len() != edges.len() || !radii.iter().all(|r| *r > 0.0 && r.is_finite()) {
        return Err(BlendError::code("invalidValue"));
    }
    let rmax = radii.iter().cloned().fold(0.0, f64::max);
    EDGE_RADII.with(|c| *c.borrow_mut() = Some(radii.to_vec()));
    let r = blend_edges(b, edges, Spec::Fillet(rmax, FilletProfile::Circular), feature, tol);
    EDGE_RADII.with(|c| *c.borrow_mut() = None);
    r
}

thread_local! {
    /// The face a chamfer's first leg lies on (and whether to take the other one instead), see
    /// [`chamfer_legs_on_face`].
    static CHAMFER_FIRST: std::cell::RefCell<Option<(FaceName, bool)>> = const { std::cell::RefCell::new(None) };
}

/// [`chamfer_legs`] with the first leg (`d1`, or the distance of the angle form) on the face named
/// `first` for every edge, whichever side of the pick it is on; `flip` takes the other face of each
/// edge instead. `first` is a face id, `~k` region suffix ignored.
pub fn chamfer_legs_on_face(b: &Body, edges: &[(String, String, Vec3)], d1: f64, leg: ChamferLeg, first: &str, flip: bool, feature: &str, tol: f64) -> Result<Body, BlendError> {
    let name = FaceName::parse(first).map(|mut n| {
        n.region = None;
        n
    });
    let Some(name) = name else { return Err(BlendError::code("missingEdge")) };
    CHAMFER_FIRST.with(|c| *c.borrow_mut() = Some((name, flip)));
    let r = blend_edges(b, edges, Spec::Chamfer(d1, leg), feature, tol);
    CHAMFER_FIRST.with(|c| *c.borrow_mut() = None);
    r
}

thread_local! {
    /// While set, the ends of selected edges that continue each other tangentially stop flush (see
    /// [`junction_vertices`]); used by [`super::mixed`] for the chain rim - arc - rim.
    pub(super) static TANGENT_FLUSH: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Sets [`TANGENT_FLUSH`] for its lifetime (a selection that is a chain of tangent edges).
pub struct TangentChainGuard(bool);

impl TangentChainGuard {
    pub fn new(on: bool) -> TangentChainGuard {
        let old = TANGENT_FLUSH.with(|c| c.replace(on || c.get()));
        TangentChainGuard(old)
    }
}

impl Drop for TangentChainGuard {
    fn drop(&mut self) {
        TANGENT_FLUSH.with(|c| c.set(self.0));
    }
}

/// [`blend_edges`] with the ends of tangent-continuing selected edges stopping flush (a selection that is
/// a chain of tangent edges: the rolling ball runs on from one edge into the next, so no tool may reach
/// past the junction).
pub fn blend_edges_tangent_chain(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    TANGENT_FLUSH.with(|c| c.set(true));
    let r = blend_edges(b, edges, spec, feature, tol);
    TANGENT_FLUSH.with(|c| c.set(false));
    r
}

/// Per-selection end information gathered on the original body.
#[derive(Default)]
struct Junc {
    /// vertices where a concave tool runs on to (`true`: material behind the end face)
    ext: Vec<(Vec3, bool)>,
    /// end vertices whose end face leans toward the material: a convex tool stops flush there
    flush: Vec<Vec3>,
    /// `(vertex, direction away from it along this edge, along the partner edge)`: two selected convex
    /// edges that meet at a reflex corner of their common face. Both tools run on past the vertex and
    /// each keeps its own side of the bisector plane.
    mitre: Vec<(Vec3, Vec3, Vec3)>,
}

/// Reflex-corner pairs of the selection, see [`Junc::mitre`]: two selected straight convex edges of
/// planar faces that share a vertex and exactly one face, each of whose end face at the vertex leans
/// toward the material (the other edge's wall).
fn mitre_pairs(b: &Body, sel: &[EdgeId]) -> Vec<(Vec3, Vec3, Vec3)> {
    let convex_line = |e: EdgeId| -> bool {
        super::mixed::line_edge_concave(b, e) == Some(false)
    };
    let away = |e: EdgeId, v: ck_topo::body::VertexId| -> Vec3 {
        let ed = b.edge(e);
        let other = if ed.v0 == v { ed.v1 } else { ed.v0 };
        (b.vertex(other).p - b.vertex(v).p).normalize()
    };
    // the end face of `e` at `v`: the one face at the vertex not on the edge; it must lean toward the material
    let leans = |e: EdgeId, v: ck_topo::body::VertexId| -> bool {
        let efs = b.edge_faces(e);
        let mut ends: Vec<FaceId> = vec![];
        for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
            for f in b.edge_faces(x) {
                if !efs.contains(&f) && !ends.contains(&f) {
                    ends.push(f);
                }
            }
        }
        ends.len() == 1 && matches!(b.face(ends[0]).surface, Surface::Plane { .. }) && b.planar_normal(ends[0]).dot(-away(e, v)) < -1e-9
    };
    let concave_line = |e: EdgeId| -> bool { super::mixed::line_edge_concave(b, e) == Some(true) };
    let mut out = vec![];
    for (i, &a) in sel.iter().enumerate() {
        for &c in &sel[i + 1..] {
            let both_convex = convex_line(a) && convex_line(c);
            let both_concave = concave_line(a) && concave_line(c);
            if a == c || !(both_convex || both_concave) {
                continue;
            }
            let (ea, ec) = (b.edge(a), b.edge(c));
            for v in [ea.v0, ea.v1] {
                if ea.v0 == ea.v1 || !(ec.v0 == v || ec.v1 == v) {
                    continue;
                }
                let (fa, fc) = (b.edge_faces(a), b.edge_faces(c));
                if fa.iter().filter(|f| fc.contains(f)).count() != 1 {
                    continue;
                }
                if both_convex {
                    if !leans(a, v) || !leans(c, v) {
                        continue;
                    }
                } else {
                    // two concave edges (a cove round the foot of a post) that meet at an OUTSIDE corner: the
                    // third edge of the vertex, between the two walls, is convex and not filleted
                    let others: Vec<EdgeId> = b.edge_ids().filter(|&x| x != a && x != c && (b.edge(x).v0 == v || b.edge(x).v1 == v)).collect();
                    if others.len() != 1 || sel.contains(&others[0]) || !convex_line(others[0]) {
                        continue;
                    }
                }
                let (da, dc) = (away(a, v), away(c, v));
                let p = b.vertex(v).p;
                out.push((p, da, dc));
                out.push((p, dc, da));
            }
        }
    }
    out
}

/// `tool` cut down to the half-space `(p - o) . n >= 0`.
pub(super) fn clip_halfspace(tool: &Body, o: Vec3, n: Vec3, tol: f64) -> Result<Body, BlendError> {
    let (lo, hi) = tool.bbox();
    let big = 4.0 * (hi - lo).len() + 10.0;
    let u = if n.x.abs() < 0.9 { Vec3::X.cross(n) } else { Vec3::Y.cross(n) }.normalize();
    let w = n.cross(u);
    let pt = |a: f64, b: f64, c: f64| o + n * (a * big) + u * (b * big) + w * (c * big);
    // (corner loop, outward normal)
    let quads: [([Vec3; 4], Vec3); 6] = [
        ([pt(0., -1., -1.), pt(0., 1., -1.), pt(0., 1., 1.), pt(0., -1., 1.)], -n),
        ([pt(1., -1., -1.), pt(1., 1., -1.), pt(1., 1., 1.), pt(1., -1., 1.)], n),
        ([pt(0., -1., -1.), pt(1., -1., -1.), pt(1., -1., 1.), pt(0., -1., 1.)], -u),
        ([pt(0., 1., -1.), pt(1., 1., -1.), pt(1., 1., 1.), pt(0., 1., 1.)], u),
        ([pt(0., -1., -1.), pt(1., -1., -1.), pt(1., 1., -1.), pt(0., 1., -1.)], -w),
        ([pt(0., -1., 1.), pt(1., -1., 1.), pt(1., 1., 1.), pt(0., 1., 1.)], w),
    ];
    let polys: Vec<(Vec<Vec3>, FaceName)> = quads
        .iter()
        .enumerate()
        .map(|(i, (q, out))| {
            let nw = (q[1] - q[0]).cross(q[2] - q[0]);
            let mut v = q.to_vec();
            if nw.dot(*out) < 0.0 {
                v.reverse();
            }
            (v, FaceName::other("clip", &format!("k{i}")))
        })
        .collect();
    let block = super::vertex::polyhedron(&polys);
    crate::boolean::boolean(tool, &block, Op::Intersect, tol).map(|r| r.body).map_err(|_| BlendError::code("blendFailed"))
}

/// Vertices where two of the selected edges continue each other tangentially (see [`TANGENT_FLUSH`]).
fn tangent_flush_vertices(b: &Body, sel: &[EdgeId]) -> Vec<Vec3> {
    let mut flush: Vec<Vec3> = vec![];
    {
        for &e in sel {
            let ed = b.edge(e);
            if ed.v0 == ed.v1 {
                continue;
            }
            for v in [ed.v0, ed.v1] {
                let d = ed.curve.tangent(if ed.v0 == v { ed.t0 } else { ed.t1 });
                if d.len() < 1e-12 {
                    continue;
                }
                let tangent_mate = sel.iter().copied().filter(|&x| x != e).any(|x| {
                    let ex = b.edge(x);
                    if ex.v0 != v && ex.v1 != v {
                        return false;
                    }
                    let dx = ex.curve.tangent(if ex.v0 == v { ex.t0 } else { ex.t1 });
                    dx.len() > 1e-12 && d.normalize().cross(dx.normalize()).len() < 1e-6
                });
                let p = b.vertex(v).p;
                if tangent_mate && !flush.iter().any(|q| q.dist(p) < 1e-9) {
                    flush.push(p);
                }
            }
        }
    }
    flush
}

/// The original vertices (with `true` when material lies behind the end face, so
/// a tool may run on past it) where two or more of the `sel` edges meet and not
/// all of them are concave straight edges of planar faces (three of those get the
/// sphere patch). An earlier tool of the feature shortens the later edge there; a
/// concave tool runs on to the original vertex.
fn junction_vertices(b: &Body, sel: &[EdgeId], tol: f64) -> Junc {
    let concave = |e: EdgeId| -> bool {
        let ed = b.edge(e);
        if !matches!(ed.curve, Curve::Line { .. }) {
            return false;
        }
        let fs = b.edge_faces(e);
        if fs.len() != 2 || fs.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
            return false;
        }
        let d = (b.vertex(ed.v1).p - b.vertex(ed.v0).p).normalize();
        let Some(co) = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0]) else { return false };
        let tdir = if b.coedge(co).rev { -d } else { d };
        b.planar_normal(fs[0]).cross(tdir).dot(b.planar_normal(fs[1])) > 1e-9
    };
    let mut out: Vec<(Vec3, bool)> = vec![];
    let mut flush: Vec<Vec3> = vec![];
    if TANGENT_FLUSH.with(|c| c.get()) {
        flush = tangent_flush_vertices(b, sel);
    }
    for &e in sel {
        let ed = b.edge(e);
        if matches!(ed.curve, Curve::Line { .. }) {
            let efs = b.edge_faces(e);
            for (v, other) in [(ed.v0, ed.v1), (ed.v1, ed.v0)] {
                let outd = (b.vertex(v).p - b.vertex(other).p).normalize();
                let mut ends: Vec<FaceId> = vec![];
                for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                    for f in b.edge_faces(x) {
                        if !efs.contains(&f) && !ends.contains(&f) {
                            ends.push(f);
                        }
                    }
                }
                if ends.len() == 1 && matches!(b.face(ends[0]).surface, Surface::Plane { .. }) && b.planar_normal(ends[0]).dot(outd) < -1e-9 {
                    flush.push(b.vertex(v).p);
                }
            }
        }
        for v in [ed.v0, ed.v1] {
            let at: Vec<EdgeId> = sel.iter().copied().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v).collect();
            let p = b.vertex(v).p;
            if at.len() < 2 || (at.len() == 3 && at.iter().all(|&x| concave(x))) || out.iter().any(|(q, _)| q.dist(p) <= tol) {
                continue;
            }
            // the end face of the concave edge here: the face at the vertex not on the edge
            let Some(&ce) = at.iter().find(|&&x| concave(x)) else { continue };
            let cd = b.edge(ce);
            let other = if cd.v0 == v { b.vertex(cd.v1).p } else { b.vertex(cd.v0).p };
            let out_dir = (p - other).normalize();
            let on_edge = b.edge_faces(ce);
            let mut ends: Vec<FaceId> = vec![];
            for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                for f in b.edge_faces(x) {
                    if !on_edge.contains(&f) && !ends.contains(&f) {
                        ends.push(f);
                    }
                }
            }
            // the faces other than the concave edge's own: the plane perpendicular to it decides
            let behind = ends.iter().filter(|&&f| matches!(b.face(f).surface, Surface::Plane { .. })).any(|&f| {
                let n = b.planar_normal(f);
                n.cross(out_dir).len() < 1e-6 && n.dot(out_dir) < 0.0
            });
            out.push((p, behind));
        }
    }
    Junc { ext: out, flush, mitre: vec![] }
}

/// Both faces border a blend face of `feature` in `b` (their shared edge was
/// consumed by a tool of this feature).
fn blended_by_earlier_tool(b: &Body, fa: &str, fb: &str, feature: &str) -> bool {
    let (Some(f1), Some(f2)) = (b.find_face(fa), b.find_face(fb)) else { return false };
    b.face_ids().any(|g| {
        let n = &b.face(g).name;
        n.feature == feature && matches!(n.role, Role::Blend(_)) && {
            let nb = b.face_neighbors(g);
            nb.contains(&f1) && nb.contains(&f2)
        }
    })
}

/// Chain (component id, position) of each selected edge: components follow
/// v1 -> v0 links from the first selected edge of the component, forwards
/// (+1) then backwards (-1); a closed ring stops where it returns.
fn chain_positions(ends: &[[usize; 2]]) -> Vec<(usize, i64)> {
    let n = ends.len();
    let mut pos: Vec<Option<(usize, i64)>> = vec![None; n];
    let mut comp = 0;
    for s0 in 0..n {
        if pos[s0].is_some() {
            continue;
        }
        pos[s0] = Some((comp, 0));
        let mut cur = s0;
        let mut p = 0;
        while let Some(nx) = (0..n).find(|&j| pos[j].is_none() && ends[cur][1] == ends[j][0]) {
            p += 1;
            pos[nx] = Some((comp, p));
            cur = nx;
        }
        let mut cur = s0;
        let mut p = 0;
        while let Some(pv) = (0..n).find(|&j| pos[j].is_none() && ends[j][1] == ends[cur][0]) {
            p -= 1;
            pos[pv] = Some((comp, p));
            cur = pv;
        }
        comp += 1;
    }
    pos.into_iter().map(|p| p.unwrap()).collect()
}

/// One blend op over `edges`; `idx[k]` is the blend number (`B<idx>`) of edge k.
fn blend_edges_at(
    b: &Body,
    edges: &[(String, String, Vec3)],
    spec: Spec,
    feature: &str,
    tol: f64,
    idx: &[u32],
    skip_end: &[String],
    rings: &[bool],
    junc: &Junc,
) -> Result<(Body, Vec<BlendRef>, Vec<String>), BlendError> {
    let edges: Vec<(String, String, Vec3)> = edges.iter().map(|(fa, fb, at)| forward_first(b, fa, fb, *at)).collect();
    // blend references on the INPUT body (for the in-place numbering)
    let mut refs: Vec<BlendRef> = vec![];
    let mut sel_edges: Vec<EdgeId> = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let e = find_edge(b, fa, fb, *at).ok_or_else(|| BlendError::code("missingEdge"))?;
        sel_edges.push(e);
        let ed = b.edge(e);
        // a tolerant edge (imported geometry that did not fit exactly) has no exact face intersection to roll along
        if ed.tol > 0.0 {
            return Err(BlendError::detail("blendTolerantEdge", &format!("edge#{} between {fa} and {fb} carries a tolerance of {:.3e} mm (imported geometry); the blend needs an exact edge", e.idx(), ed.tol)));
        }
        refs.push(BlendRef {
            name: FaceName::blend(feature, idx[k]).id(),
            faces: [fa.clone(), fb.clone()],
            ends: [b.vertex(ed.v0).p, b.vertex(ed.v1).p],
        });
    }
    let mut out = b.clone();
    let mut touched: Vec<String> = vec![];
    for (k, (fa, fb, at)) in edges.iter().enumerate() {
        let Some(e) = find_edge(&out, fa, fb, *at) else {
            // an earlier tool of this feature already blended it: collinear edges
            // separated by a notch (a hole centred on the edge line) are cut by one
            // tool, whose margin runs on over the notch; the edge is gone but both of
            // its faces now border that feature's blend face
            if blended_by_earlier_tool(&out, fa, fb, feature) {
                dbg_fail(&format!("edge {k} ({fa} | {fb}) was blended by an earlier tool"));
                continue;
            }
            dbg_fail(&format!("edge {k} ({fa} | {fb} at {at:?}) not found after earlier blends"));
            return Err(BlendError::code("missingEdge"));
        };
        let own = out.edge_faces(e);
        let (f1, f2) = match (out.find_face(fa), out.find_face(fb)) {
            // the names must still be the edge's faces (a saved selection can name a face the edge found by
            // position no longer borders)
            (Some(f1), Some(f2)) if own.len() == 2 && own.contains(&f1) && own.contains(&f2) => (f1, f2),
            _ => {
                // a face split into regions by an earlier tool: the edge's own faces, in selection order
                let fs = out.edge_faces(e);
                let base = |f: FaceId| {
                    let mut n = out.face(f).name.clone();
                    n.region = None;
                    n.id()
                };
                if base(fs[0]) == *fa || base(fs[1]) == *fb { (fs[0], fs[1]) } else { (fs[1], fs[0]) }
            }
        };
        let spec_k = match spec {
            Spec::Fillet(_, p) => EDGE_RADII.with(|c| c.borrow().as_ref().and_then(|v| v.get(idx[k] as usize).copied())).map_or(spec, |r| Spec::Fillet(r, p)),
            _ => spec,
        };
        let (body, hint) = blend_edge_hint(&out, e, [f1, f2], spec_k, feature, idx[k], tol, rings.get(k).copied().unwrap_or(false), junc)?;
        // end faces at a junction with another selected edge are that
        // edge's own faces, not extra touches (oracle: the ring's face
        // order is e1 e2 e3 e4, the open chain's C1 e1 e4 e2 e3)
        let junction_faces: Vec<&String> = (0..edges.len())
            .filter(|&j| j != k && refs[j].ends.iter().any(|&q| refs[k].ends.iter().any(|&p| p.dist(q) <= tol * 10.0)))
            .flat_map(|j| refs[j].faces.iter())
            .collect();
        for (n, h) in hint.into_iter().enumerate() {
            if n >= 2 && junction_faces.iter().any(|f| **f == h) {
                continue;
            }
            // fillets: an end face an earlier round already touched stays put
            if n >= 2 && matches!(spec, Spec::Fillet(..)) && skip_end.contains(&h) {
                continue;
            }
            if !touched.contains(&h) {
                touched.push(h);
            }
        }
        out = body;
    }
    let earlier_blends = b.face_ids().any(|f| {
        let n = &b.face(f).name;
        n.feature == feature && matches!(n.role, Role::Blend(_))
    });
    if edges.len() == 1 && !earlier_blends {
        return Ok((out, refs, touched));
    }
    // two edges leaving the same vertex (a bottom edge and the rail on its
    // start, probes b<k>,v<k>): the face they share goes after the rest
    let mut touched = touched;
    for i in 0..refs.len() {
        for j in i + 1..refs.len() {
            let full_corner = (0..refs.len()).filter(|&k| refs[k].ends.iter().any(|q| q.dist(refs[i].ends[0]) <= tol * 10.0)).count() >= 3;
            if !full_corner && refs[i].ends[0].dist(refs[j].ends[0]) <= tol * 10.0 {
                for f in refs[i].faces.iter().filter(|f| refs[j].faces.contains(f)) {
                    if let Some(p) = touched.iter().position(|t| t == f) {
                        let t = touched.remove(p);
                        touched.push(t);
                    }
                }
            }
        }
    }
    // a face whose whole boundary is blended (the top of a filleted ring)
    // is rebuilt rather than trimmed: oracle order puts it after the other
    // touched faces
    let replaced = |id: &String| {
        b.find_face(id).is_some_and(|f| {
            let es = b.face_edges(f);
            !es.is_empty() && es.iter().all(|e| sel_edges.contains(e))
        })
    };
    let (rep, mut keep): (Vec<String>, Vec<String>) = touched.iter().cloned().partition(|t| replaced(t));
    keep.extend(rep);
    let touched = keep;
    // one blend op over all edges: untouched faces, touched faces in
    // first-touch order, blend faces (probe fil2)
    let base = |id: &str| FaceName::parse(id).map(|mut n| {
        n.region = None;
        n.id()
    });
    let rank = |f: FaceId| -> (usize, usize) {
        let name = &out.face(f).name;
        if name.feature == feature {
            if let Role::Blend(i) = name.role {
                // chamfer blends of earlier rounds are ordinary faces; fillet
                // blends always sit at the end, by blend number
                if idx.contains(&i) || matches!(spec, Spec::Fillet(..)) {
                    return (2, i as usize);
                }
            }
        }
        match touched.iter().position(|t| base(t) == base(&name.id())) {
            Some(p) => (1, p),
            // untouched faces keep the INPUT body's order (a face the cut
            // modified but the oracle did not count as touched sits at its
            // old place)
            None => (0, b.face_ids().position(|g| base(&b.face_id_str(g)) == base(&name.id())).unwrap_or(usize::MAX)),
        }
    };
    let mut order: Vec<FaceId> = out.face_ids().collect();
    order.sort_by_key(|&f| rank(f)); // stable: keeps relative order within ties
    let out = crate::boolean::assemble::rebuild_in_order(&out, &order);
    Ok((out, refs, touched))
}

/// `(fa, fb)` with the face whose coedge runs the selected edge forward
/// first (unchanged when the edge is not found).
pub fn forward_first(b: &Body, fa: &str, fb: &str, at: Vec3) -> (String, String, Vec3) {
    let keep = (fa.to_string(), fb.to_string(), at);
    let Some(e) = find_edge(b, fa, fb, at) else { return keep };
    let fwd = b.edge_coedges(e).into_iter().find(|&c| !b.coedge(c).rev).map(|c| b.coedge_face(c));
    match (fwd, b.find_face(fb)) {
        (Some(f), Some(g)) if f == g => (fb.to_string(), fa.to_string(), at),
        _ => keep,
    }
}

/// One selected blend: its face id, the edge's two faces (forward face
/// first) and end points (v0, v1) on the input body.
#[derive(Debug, Clone)]
pub struct BlendRef {
    pub name: String,
    pub faces: [String; 2],
    pub ends: [Vec3; 2],
}

/// Re-lay `out` (the blended body) in the original's in-place arena, so
/// [`Body::wire_numbered`] gives the oracle `#n` (bracket F11 / F12):
/// - an input edge that survives unchanged keeps its slot;
/// - an input edge the blend trims is split in parameter order (in
///   ascending input slot order): its first piece keeps the slot, each
///   further piece takes the next new slot; pieces the blend removed stay
///   as dead slots (F11: C0|S.e9 #2 trimmed at its end keeps #2, C0|S.e8
///   #1 trimmed at its start dies and its kept piece is #76, S.e10|S.e9 #6
///   trimmed at both ends -> #80 kept, #81 dead);
/// - removed input edges stay dead slots;
/// - then per blend (selection order) its edges: spring on the forward
///   face, spring on the other face, end arc at v0, end arc at v1.
pub fn in_place_layout(input: &Body, out: &Body, refs: &[BlendRef], tol: f64) -> Body {
    in_place_layout_opts(input, out, refs, tol, &LayoutOpts::default())
}

/// Overrides of the in-place numbering (vertex chamfer: three slots are
/// reserved right after the input's and handed to the trimmed edges by their
/// vertex-edge order).
#[derive(Debug, Clone, Default)]
pub struct LayoutOpts {
    /// Dead / reserved slots after the input's (None: two per junction).
    pub reserve: Option<usize>,
    /// (input slot, slot of its kept piece when the edge is trimmed at its start).
    pub piece_slots: Vec<(usize, usize)>,
    /// Vertex-blend corners (three fillets meeting in a sphere patch): no
    /// junction (miter / deferred spring / slot) there.
    pub corners: Vec<Vec3>,
}

pub fn in_place_layout_opts(input: &Body, out: &Body, refs: &[BlendRef], tol: f64, opts: &LayoutOpts) -> Body {
    let (in_slots, n_in) = input.wire_numbered();
    let out_chains = out.wire_chains();
    let names = |b: &Body, e: EdgeId| {
        let mut v: Vec<String> = b.edge_faces(e).iter().map(|&f| b.face_id_str(f)).collect();
        v.sort();
        v
    };
    let ends = |b: &Body, ch: &[EdgeId]| (b.vertex(b.edge(ch[0]).v0).p, b.vertex(b.edge(*ch.last().unwrap()).v1).p);
    let near = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let on = |b: &Body, ch: &[EdgeId], p: Vec3| {
        ch.iter().any(|&e| {
            let ed = b.edge(e);
            crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), p).1 <= tol * 10.0
        })
    };
    let mut slot_of: Vec<Option<usize>> = vec![None; out_chains.len()];
    let mut used_in = vec![false; in_slots.len()];
    // 1. unchanged edges
    for (k, ch) in out_chains.iter().enumerate() {
        let (p, q) = ends(out, ch);
        let nm = names(out, ch[0]);
        let mid = out.chain_mid(ch);
        for (i, (s, ich)) in in_slots.iter().enumerate() {
            if used_in[i] || names(input, ich[0]) != nm {
                continue;
            }
            let (a, b2) = ends(input, ich);
            if ((near(a, p) && near(b2, q)) || (near(a, q) && near(b2, p))) && near(input.chain_mid(ich), mid) {
                slot_of[k] = Some(*s);
                used_in[i] = true;
                break;
            }
        }
    }
    // 2. trimmed edges, split in parameter order
    // Oracle (box fillet chains): the new slots follow the touched input
    // edges in ascending slot order; a selected edge uses one slot per end
    // it shares with another selected edge (a junction), a trimmed
    // neighbour one per window it opens (a rail trimmed at one end: one, an
    // edge trimmed at both ends: two).
    let near_pt = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let at_corner = |p: Vec3| opts.corners.iter().any(|&q| near_pt(p, q));
    let junction_ends = |k: usize| -> usize {
        (0..2)
            .filter(|&e| !at_corner(refs[k].ends[e]) && (0..refs.len()).any(|j| j != k && refs[j].ends.iter().any(|&q| near_pt(refs[k].ends[e], q))))
            .count()
    };
    let mut sel_cost: Vec<usize> = vec![0; in_slots.len()];
    for (k, r) in refs.iter().enumerate() {
        let mut want = r.faces.to_vec();
        want.sort();
        for (i, (_, ich)) in in_slots.iter().enumerate() {
            let (a, b2) = ends(input, ich);
            if names(input, ich[0]) == want
                && ((near(a, r.ends[0]) && near(b2, r.ends[1])) || (near(a, r.ends[1]) && near(b2, r.ends[0])))
            {
                sel_cost[i] = junction_ends(k);
                break;
            }
        }
    }
    let mut next = n_in + opts.reserve.unwrap_or(0);
    // ascending input slot order (oracle: the new slots follow it)
    let mut slot_order: Vec<usize> = (0..in_slots.len()).collect();
    slot_order.sort_by_key(|&i| in_slots[i].0);
    for i in slot_order {
        let (s, ich) = &in_slots[i];
        if opts.reserve.is_none() {
            next += sel_cost[i];
        }
        if used_in[i] || ich.len() != 1 {
            continue;
        }
        let ie = input.edge(ich[0]);
        let nm = names(input, ich[0]);
        let pieces: Vec<usize> = (0..out_chains.len())
            .filter(|&k| slot_of[k].is_none() && names(out, out_chains[k][0]) == nm && {
                let (p, q) = ends(out, &out_chains[k]);
                on(input, ich, p) && on(input, ich, q) && on(input, ich, out.chain_mid(&out_chains[k]))
            })
            .collect();
        if pieces.is_empty() {
            continue;
        }
        let param = |p: Vec3| crate::boolean::imprint::dist_in_range(&ie.curve, (ie.t0, ie.t1), p).0;
        let mut cuts: Vec<f64> = vec![];
        for &k in &pieces {
            let (p, q) = ends(out, &out_chains[k]);
            for t in [param(p), param(q)] {
                let slack = crate::intersect::param_slack(&ie.curve, t, tol * 10.0);
                if t > ie.t0 + slack && t < ie.t1 - slack && !cuts.iter().any(|&c| (c - t).abs() <= slack) {
                    cuts.push(t);
                }
            }
        }
        cuts.sort_by(|a, b| a.total_cmp(b));
        let mut bounds = vec![ie.t0];
        bounds.extend(cuts);
        bounds.push(ie.t1);
        for (j, w) in bounds.windows(2).enumerate() {
            let slot = if j == 0 {
                *s
            } else if let Some(&(_, forced)) = opts.piece_slots.iter().find(|(is, _)| is == s) {
                forced
            } else {
                next += 1;
                next - 1
            };
            let tm = 0.5 * (w[0] + w[1]);
            if let Some(&k) = pieces.iter().find(|&&k| {
                let t = param(out.chain_mid(&out_chains[k]));
                t > w[0] && t < w[1] && slot_of[k].is_none()
            }) {
                let _ = tm;
                slot_of[k] = Some(slot);
            }
        }
        used_in[i] = true;
    }
    // 3. blend edges. Oracle (chains of box fillets, probes filletChain3 /
    // filletTop4): per blend in selection order its edges [spring on the
    // forward face, spring on the other face, end arc at v0, end arc at v1]
    // - except that where another selected edge meets the blend's v0 the
    // spring on the COMMON face is deferred; the junctions follow, by
    // blend pair (i<j) lexicographically: the miter edge B<i>|B<j>, then
    // the deferred spring of the blend that starts at the junction.
    let near_pt = |p: Vec3, q: Vec3| p.dist(q) <= tol * 10.0;
    let meets = |a: usize, ea: usize, b2: usize| !at_corner(refs[a].ends[ea]) && refs[b2].ends.iter().any(|&q| near_pt(refs[a].ends[ea], q));
    let common_face = |a: usize, b2: usize| refs[a].faces.iter().find(|f| refs[b2].faces.contains(f)).cloned();
    // deferred spring face of blend a (junction at its v0)
    let deferred: Vec<Option<String>> = (0..refs.len())
        .map(|a| (0..refs.len()).filter(|&o| o != a && meets(a, 0, o)).find_map(|o| common_face(a, o)))
        .collect();
    let is_blend = |n: &String| refs.iter().any(|r| &r.name == n);
    for (ri, r) in refs.iter().enumerate() {
        let mut mine: Vec<(usize, usize, f64)> = vec![];
        for (k, ch) in out_chains.iter().enumerate() {
            if slot_of[k].is_some() {
                continue;
            }
            let nm = names(out, ch[0]);
            if !nm.contains(&r.name) {
                continue;
            }
            let other = nm.iter().find(|n| **n != r.name).cloned().unwrap_or_default();
            if is_blend(&other) || deferred[ri].as_deref() == Some(other.as_str()) {
                continue; // miter / deferred spring: the junction pass
            }
            let (p, q) = ends(out, ch);
            let d0 = p.dist(r.ends[0]).min(q.dist(r.ends[0]));
            let d1 = p.dist(r.ends[1]).min(q.dist(r.ends[1]));
            let rank = if other == r.faces[0] {
                0
            } else if other == r.faces[1] {
                1
            } else if d0 <= d1 {
                2
            } else {
                3
            };
            mine.push((rank, k, d0.min(d1)));
        }
        mine.sort_by(|a, b| a.0.cmp(&b.0).then(a.2.total_cmp(&b.2)));
        for (_, k, _) in mine {
            slot_of[k] = Some(next);
            next += 1;
        }
    }
    for i in 0..refs.len() {
        for j in i + 1..refs.len() {
            if !(meets(i, 0, j) || meets(i, 1, j)) {
                continue;
            }
            let find = |x: &str, y: &str, slot_of: &Vec<Option<usize>>| {
                (0..out_chains.len()).find(|&k| {
                    slot_of[k].is_none() && {
                        let nm = names(out, out_chains[k][0]);
                        nm.len() == 2 && nm.iter().any(|n| n == x) && nm.iter().any(|n| n == y)
                    }
                })
            };
            if let Some(k) = find(&refs[i].name, &refs[j].name, &slot_of) {
                slot_of[k] = Some(next);
                next += 1;
            }
            let owner = if meets(i, 0, j) { Some(i) } else if meets(j, 0, i) { Some(j) } else { None };
            if let Some(o) = owner {
                if let Some(f) = &deferred[o] {
                    if let Some(k) = find(&refs[o].name, f, &slot_of) {
                        slot_of[k] = Some(next);
                        next += 1;
                    }
                }
            }
        }
    }
    // 4. anything else, in out order
    for k in 0..out_chains.len() {
        if slot_of[k].is_none() {
            slot_of[k] = Some(next);
            next += 1;
        }
    }
    // ---- write the arena: slot order, dead placeholders, seams last
    let mut res = Body::new();
    let lump = res.add_lump();
    let shell = res.add_shell(lump);
    let mut vmap: Vec<Option<ck_topo::body::VertexId>> = vec![None; out.vertices.len()];
    for v in out.vertex_ids() {
        vmap[v.idx()] = Some(res.add_vertex(out.vertex(v).p));
    }
    let mut by_slot: Vec<Option<usize>> = vec![None; next];
    for (k, s) in slot_of.iter().enumerate() {
        by_slot[s.unwrap()] = Some(k);
    }
    let mut emap: Vec<Option<EdgeId>> = vec![None; out.edges.len()];
    let dummy = res.add_vertex(Vec3::ZERO);
    res.vertices[dummy.idx()].alive = false;
    for s in 0..next {
        match by_slot[s] {
            Some(k) => {
                for &e in &out_chains[k] {
                    let ed = out.edge(e);
                    emap[e.idx()] =
                        Some(res.add_edge(ed.curve.clone(), ed.t0, ed.t1, vmap[ed.v0.idx()].unwrap(), vmap[ed.v1.idx()].unwrap()));
                }
            }
            None => {
                let d = res.add_edge(Curve::Line { o: Vec3::ZERO, d: Vec3::X }, 0.0, 0.0, dummy, dummy);
                res.edges[d.idx()].alive = false;
            }
        }
    }
    for e in out.edge_ids() {
        if emap[e.idx()].is_none() {
            let ed = out.edge(e);
            emap[e.idx()] = Some(res.add_edge(ed.curve.clone(), ed.t0, ed.t1, vmap[ed.v0.idx()].unwrap(), vmap[ed.v1.idx()].unwrap()));
        }
    }
    for f in out.face_ids() {
        let face = out.face(f);
        let nf = res.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
        for &l in &face.loops {
            let nl = res.add_loop(nf);
            for &c in &out.lp(l).coedges {
                let ce = out.coedge(c).clone();
                res.add_coedge(nl, emap[ce.edge.idx()].unwrap(), ce.rev, ce.pcurve);
            }
        }
    }
    ck_topo::region::rebuild_lumps(&mut res);
    res
}

thread_local! {
    static LAST_FAIL: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
}

/// The reason the last blend step gave up (see [`dbg_fail`]); empty when none did.
pub(super) fn last_fail() -> String {
    LAST_FAIL.with(|c| c.borrow().clone())
}

/// `CK_BLEND_DEBUG=1` prints why a blend gave up (native runs only).
fn dbg_fail(m: &str) {
    LAST_FAIL.with(|c| *c.borrow_mut() = m.chars().take(400).collect());
    if std::env::var_os("CK_BLEND_DEBUG").is_some() {
        eprintln!("blend fail: {m}");
    }
}

fn chamfer_only(spec: Spec) -> bool {
    matches!(spec, Spec::Chamfer(..))
}

/// Split rings in a chamfer selection: partial arcs (circle edges, or spline arcs, e.g. the
/// halves of a lofted cap rim) on one circle sharing a face
/// whose spans add up to a full turn. Returns `(leader, skipped)` flags per
/// selected edge: the first arc cuts the whole ring, the rest are absorbed.
fn split_rings(b: &Body, eids: &[EdgeId]) -> (Vec<bool>, Vec<bool>) {
    let n = eids.len();
    let (mut lead, mut skip) = (vec![false; n], vec![false; n]);
    let info: Vec<Option<(Vec3, Vec3, f64, f64, Vec<FaceId>)>> = eids
        .iter()
        .map(|&e| {
            let ed = b.edge(e);
            if matches!(ed.curve, Curve::Line { .. }) {
                return None;
            }
            let (c, a, r, sp) = match &ed.curve {
                Curve::Circle { frame, r } => (frame.o, frame.z.normalize(), *r, (ed.t1 - ed.t0).abs()),
                other => fit_circle(other, ed.t0.min(ed.t1), ed.t0.max(ed.t1))?,
            };
            let mut fs = b.edge_faces(e);
            fs.sort_by_key(|f| f.idx());
            Some((c, a, r, sp, fs))
        })
        .collect();
    let mut used = vec![false; n];
    for i in 0..n {
        let Some((c, a, r, _, fs)) = &info[i] else { continue };
        if used[i] {
            continue;
        }
        let grp: Vec<usize> = (0..n)
            .filter(|&j| {
                !used[j]
                    && info[j].as_ref().map_or(false, |(c2, a2, r2, _, f2)| {
                        f2.iter().any(|f| fs.contains(f)) && (*c2 - *c).len() < 1e-7 * (1.0 + r) && a2.cross(*a).len() < 1e-7 && (r2 - r).abs() < 1e-7 * (1.0 + r)
                    })
            })
            .collect();
        let total: f64 = grp.iter().map(|&j| info[j].as_ref().unwrap().3).sum();
        if grp.len() >= 2 && (total - std::f64::consts::TAU).abs() < 1e-6 {
            lead[grp[0]] = true;
            for &j in &grp[1..] {
                skip[j] = true;
            }
        }
        for &j in &grp {
            used[j] = true;
        }
    }
    (lead, skip)
}

/// The other arcs of a split ring: edges on the circle of `e` that share a face with it, when together they close
/// the circle (spans add up to a full turn). Empty for a single edge or an open arc.
fn ring_siblings(b: &Body, e: EdgeId) -> Vec<EdgeId> {
    let circle_of = |x: EdgeId| -> Option<(Vec3, Vec3, f64, f64)> {
        let ed = b.edge(x);
        if matches!(ed.curve, Curve::Line { .. }) {
            return None;
        }
        match &ed.curve {
            Curve::Circle { frame, r } => Some((frame.o, frame.z.normalize(), *r, (ed.t1 - ed.t0).abs())),
            other => fit_circle(other, ed.t0.min(ed.t1), ed.t0.max(ed.t1)),
        }
    };
    let Some((c, a, r, sp)) = circle_of(e) else { return vec![] };
    let fs = b.edge_faces(e);
    let mut group = vec![];
    let mut total = sp;
    for x in b.edge_ids().filter(|&x| x != e) {
        let Some((c2, a2, r2, sp2)) = circle_of(x) else { continue };
        if (c2 - c).len() < 1e-7 * (1.0 + r) && a2.cross(a).len() < 1e-7 && (r2 - r).abs() < 1e-7 * (1.0 + r) && b.edge_faces(x).iter().any(|f| fs.contains(f)) {
            group.push(x);
            total += sp2;
        }
    }
    if !group.is_empty() && (total - std::f64::consts::TAU).abs() < 1e-6 { group } else { vec![] }
}

/// `edges` plus, for every selected arc of a split ring, the rest of its arcs (picked at their midpoints).
fn with_ring_siblings(b: &Body, edges: &[(String, String, Vec3)]) -> Vec<(String, String, Vec3)> {
    let mut out = edges.to_vec();
    let mut have: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    for e in have.clone() {
        for s in ring_siblings(b, e) {
            if have.contains(&s) {
                continue;
            }
            let fs = b.edge_faces(s);
            if fs.len() != 2 {
                continue;
            }
            let ed = b.edge(s);
            let at = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
            let (fa, fb) = (b.face_id_str(fs[0]), b.face_id_str(fs[1]));
            if find_edge(b, &fa, &fb, at) == Some(s) {
                have.push(s);
                out.push((fa, fb, at));
            }
        }
    }
    out
}

/// One straight edge between two planar faces.
pub fn fillet_line_edge(b: &Body, e: EdgeId, faces: [FaceId; 2], r: f64, feature: &str, k: u32, tol: f64) -> Result<Body, BlendError> {
    blend_edge_hint(b, e, faces, Spec::Fillet(r, FilletProfile::Circular), feature, k, tol, false, &Junc::default()).map(|x| x.0)
}

/// A planar face the circle edge `e` stands on: the plane contains the circle, is normal to its axis, and the
/// face covers the edge's start point in its interior (a wall that only touches a plate along a line: the edge
/// is the foot of the wall, not a free rim). Not one of the edge's own faces.
fn foot_plane(b: &Body, e: EdgeId, faces: [FaceId; 2], tol: f64) -> Option<FaceId> {
    let ed = b.edge(e);
    let (c, axis, rad) = ed.curve.circle_info()?;
    let axis = axis.normalize();
    let p0 = ed.curve.eval(ed.t0.min(ed.t1));
    b.face_ids().find(|&g| {
        if faces.contains(&g) {
            return false;
        }
        let Surface::Plane { frame } = &b.face(g).surface else { return false };
        frame.z.cross(axis).len() < 1e-9
            && (p0 - frame.o).dot(frame.z).abs() <= 1e-7 * (1.0 + rad)
            && (c - p0).dot(frame.z).abs() <= 1e-7 * (1.0 + rad)
            && point_in_face(b, g, p0, tol) != Where::Out
    })
}

/// As [`fillet_line_edge`], also returning the touched-face order hint
/// (edge faces, then end faces).
///
/// A circular fillet on the foot of a wall that stands on a plane is the concave fillet between the wall and that
/// plane (tangent to both, adding material), not a round-over of the wall's own base edge: tried first with each of
/// the edge's faces in turn; the ordinary blend of the edge's two faces is the fallback.
fn blend_edge_hint(
    b: &Body,
    e: EdgeId,
    faces: [FaceId; 2],
    spec: Spec,
    feature: &str,
    k: u32,
    tol: f64,
    ring: bool,
    junc: &Junc,
) -> Result<(Body, Vec<String>), BlendError> {
    if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) {
        if let Some(plane) = foot_plane(b, e, faces, tol) {
            for cand in [faces[0], faces[1]] {
                if !matches!(b.face(cand).surface, Surface::Cone { .. } | Surface::Cylinder { .. }) {
                    continue;
                }
                match blend_edge_hint_inner(b, e, [cand, plane], spec, feature, k, tol, ring, junc) {
                    Ok(x) => {
                        if std::env::var_os("CK_BLEND_DEBUG").is_some() {
                            eprintln!("foot blend on {} and {} ok", b.face_id_str(cand), b.face_id_str(plane));
                        }
                        return Ok(x);
                    }
                    Err(err) => dbg_fail(&format!("foot blend on {} and {} failed: {}", b.face_id_str(cand), b.face_id_str(plane), err.code)),
                }
            }
        }
    }
    blend_edge_hint_inner(b, e, faces, spec, feature, k, tol, ring, junc)
}

fn blend_edge_hint_inner(
    b: &Body,
    e: EdgeId,
    faces: [FaceId; 2],
    spec: Spec,
    feature: &str,
    k: u32,
    tol: f64,
    ring: bool,
    junc: &Junc,
) -> Result<(Body, Vec<String>), BlendError> {
    let ed = b.edge(e).clone();
    let r = spec.size();
    let chamfer = matches!(spec, Spec::Chamfer(..));
    // `ext`: chamfer through curved geometry (a cylinder / lofted cone face,
    // a tangent edge, a spline edge that is a circle). The tool then reaches
    // OUT of the part past both legs, so no tool face is coincident with a
    // body face of a different surface type.
    let mut ext = false;
    // angle swept by a circle edge (from its start point, ccw about the axis)
    let mut sweep = 0.0f64;
    // Geometry at the working point P0 (line: edge midpoint; circle: its
    // start vertex, which is also the faces' seam point).
    let (p0, p1, dir, n1, n2, circ): (Vec3, Vec3, Vec3, Vec3, Vec3, Option<(Vec3, Vec3)>) = match ed.curve {
        Curve::Line { .. } => {
            let p0 = b.vertex(ed.v0).p;
            let p1 = b.vertex(ed.v1).p;
            let dir = (p1 - p0) / p0.dist(p1);
            let pm = (p0 + p1) * 0.5;
            for f in faces {
                match &b.face(f).surface {
                    Surface::Plane { .. } => {}
                    Surface::Cylinder { frame, .. } if chamfer && frame.z.cross(dir).len() < 1e-9 => ext = true,
                    other => {
                        dbg_fail(&format!("line edge on {} face", other.kind()));
                        return Err(BlendError::code("blendUnsupported"));
                    }
                }
            }
            let nrm = |f: FaceId| match &b.face(f).surface {
                Surface::Plane { .. } => b.planar_normal(f),
                sf => {
                    let (u, v) = sf.closest_param(pm);
                    b.outward_normal(f, u, v).normalize()
                }
            };
            (p0, p1, dir, nrm(faces[0]), nrm(faces[1]), None)
        }
        _ => {
            // circle between coaxial plane / cylinder / cone faces
            let (c, axis, rad, span) = match ed.curve.circle_info() {
                Some((c, axis, rad)) => (c, axis.normalize(), rad, (ed.t1 - ed.t0).abs()),
                None => match chamfer.then(|| fit_circle(&ed.curve, ed.t0.min(ed.t1), ed.t0.max(ed.t1))).flatten() {
                    Some(x) => {
                        ext = true;
                        x
                    }
                    None => {
                        dbg_fail(&format!("non-circle edge {}", ed.curve.kind()));
                        return Err(BlendError::code("blendUnsupported"));
                    }
                },
            };
            sweep = if ring { std::f64::consts::TAU } else { span };
            let t_start = ed.t0.min(ed.t1);
            let p0 = ed.curve.eval(t_start);
            let mut spline_face = false;
            let mut coaxial = |f: FaceId| match &b.face(f).surface {
                Surface::Plane { frame } => frame.z.cross(axis).len() < 1e-9,
                Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => {
                    frame.z.cross(axis).len() < 1e-9 && (frame.o - c).cross(axis).len() < 1e-7 * (1.0 + rad)
                }
                Surface::BSpline(_) if chamfer => {
                    let ok = bspline_coaxial_cone(&b.face(f).surface, c, axis);
                    spline_face |= ok;
                    ok
                }
                _ => false,
            };
            if !coaxial(faces[0]) || !coaxial(faces[1]) || !(rad > 0.0) {
                dbg_fail(&format!("circle edge not coaxial: {} {}", b.face(faces[0]).surface.kind(), b.face(faces[1]).surface.kind()));
                return Err(BlendError::code("blendUnsupported"));
            }
            ext |= spline_face;
            let nrm = |f: FaceId| {
                let (u, v) = b.face(f).surface.closest_param(p0);
                b.outward_normal(f, u, v).normalize()
            };
            let dir = ed.curve.tangent(t_start).normalize();
            (p0, p0, dir, nrm(faces[0]), nrm(faces[1]), Some((c, axis)))
        }
    };
    let len = p0.dist(p1);
    let c12 = n1.dot(n2);
    let tangent = (1.0 - c12).abs() < 1e-9;
    // turning angle of the edge; a CHORD fillet is the circular fillet whose
    // arc has that chord width
    let phi = c12.clamp(-1.0, 1.0).acos();
    let (spec, r) = match spec {
        Spec::Fillet(w, FilletProfile::Chord) if phi > 1e-9 && phi < std::f64::consts::PI - 1e-9 => {
            let rr = w / (2.0 * (0.5 * phi).sin());
            (Spec::Fillet(rr, FilletProfile::Circular), rr)
        }
        _ => (spec, r),
    };
    // `chamfer_legs_on_face`: the first leg belongs to the named face, whichever side of the pick it is on
    let swap_first = CHAMFER_FIRST.with(|c| {
        c.borrow().as_ref().map_or(false, |(name, flip)| {
            let base = |f: FaceId| {
                let mut n = b.face(f).name.clone();
                n.region = None;
                n
            };
            (base(faces[1]) == *name) ^ *flip
        })
    });
    // chamfer legs on the first / second face
    let (ch1, ch2) = match spec {
        Spec::Chamfer(a, ChamferLeg::Dist(b)) => if swap_first { (b, a) } else { (a, b) },
        Spec::Chamfer(a, ChamferLeg::Angle(al) | ChamferLeg::AngleRev(al)) => {
            // triangle corner / first contact / second contact: law of sines with the interior angle
            let psi = std::f64::consts::PI - phi;
            let den = (al + psi).sin();
            if !(al > 0.0) {
                return Err(BlendError::code("invalidValue"));
            }
            if !(den > 1e-9) {
                return Err(BlendError::detail("blendSize", &format!("{}: no blend fits between the faces", FaceName::blend(feature, k).id())));
            }
            let x = a * al.sin() / den;
            if matches!(spec, Spec::Chamfer(_, ChamferLeg::AngleRev(_))) != swap_first {
                (x, a)
            } else {
                (a, x)
            }
        }
        _ => (r, r),
    };
    let r = r.max(ch1).max(ch2);
    // curvature-continuous profile: the contact points lie `g2d` from the edge
    let g2d = match spec {
        Spec::Fillet(_, FilletProfile::Curvature(sh)) => super::g2::tangent_distance(r, sh, phi).ok_or_else(|| BlendError::code("blendUnsupported"))?,
        _ => 0.0,
    };
    if (1.0 + c12).abs() < 1e-9 || (tangent && !(ext && circ.is_none())) {
        dbg_fail("tangent / folded faces");
        return Err(BlendError::code("blendUnsupported")); // tangent / folded faces
    }
    // convex iff the direction into F1 (left of its coedge about the
    // outward normal) points against n2
    let pm = if circ.is_some() { p0 } else { (p0 + p1) * 0.5 };
    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == faces[0]).ok_or_else(|| BlendError::code("missingEdge"))?;
    let tdir = if b.coedge(co).rev { -dir } else { dir };
    let up = |f: FaceId, q: Vec3| -> Vec3 {
        // on-face point nearest `q` (planes are exact already)
        match &b.face(f).surface {
            Surface::Plane { .. } => q,
            sf => sf.closest_point(q).2,
        }
    };
    let (leg1, leg2) = (n1.cross(tdir).normalize(), -n2.cross(tdir).normalize());
    let convex = if tangent {
        // G1 edge: convex iff a curved face falls away from the shared
        // tangent plane on the leg side (the cut then removes a sliver)
        let away = |f: FaceId, n: Vec3, leg: Vec3| {
            let q = pm + leg * r;
            !matches!(b.face(f).surface, Surface::Plane { .. }) && (q - up(f, q)).dot(n) > 1e-12 * (1.0 + r)
        };
        let cv = away(faces[0], n1, leg1) || away(faces[1], n2, leg2);
        if !cv {
            dbg_fail("concave tangent edge");
            return Err(BlendError::code("blendUnsupported"));
        }
        true
    } else {
        n1.cross(tdir).dot(n2) < 0.0
    };
    let s = if convex { -1.0 } else { 1.0 };
    // contact points on the two faces, the ball centre (fillet) and the
    // chamfer legs (in-face directions away from the edge: n x coedge dir)
    let off = if tangent { Vec3::ZERO } else { (n1 + n2) * (s * r / (1.0 + c12)) };
    let (ta, tb) = match spec {
        Spec::Fillet(_, FilletProfile::Curvature(_)) => (up(faces[0], pm + leg1 * g2d), up(faces[1], pm + leg2 * g2d)),
        Spec::Fillet(..) => (pm + off - n1 * (s * r), pm + off - n2 * (s * r)),
        Spec::Chamfer(..) => (up(faces[0], pm + leg1 * ch1), up(faces[1], pm + leg2 * ch2)),
    };
    // contact (spring) lines must stay on their faces
    let name = FaceName::blend(feature, k);
    // (a contact point that runs along an end face which is perpendicular to the
    // edge up to the data precision -- sketch coordinates carry 4 decimals, so an
    // "exactly radial" end line of a clip is off by 1e-5 -- lies on that face's
    // boundary line within `ptol`, not outside the face)
    let ptol = (10.0 * tol).max(2e-5 * (b.bbox().1 - b.bbox().0).len()).min(0.25 * r);
    for (f, t) in [(faces[0], ta), (faces[1], tb)] {
        let on_boundary = || {
            b.face_edges(f).into_iter().any(|e2| {
                let x = b.edge(e2);
                crate::boolean::imprint::near_in_range(&x.curve, (x.t0, x.t1), t, ptol)
            })
        };
        if point_in_face(b, f, t, tol) == Where::Out && !on_boundary() {
            if let Spec::Fillet(_, FilletProfile::Curvature(sh)) = spec {
                // the largest radius whose contact points still lie on their faces
                let fits = |rr: f64| -> bool {
                    let Some(d) = super::g2::tangent_distance(rr, sh, phi) else { return false };
                    [(faces[0], leg1), (faces[1], leg2)].iter().all(|&(f, leg)| point_in_face(b, f, up(f, pm + leg * d), tol) != Where::Out)
                };
                let (mut lo, mut hi) = (0.0, r);
                for _ in 0..40 {
                    let mid = 0.5 * (lo + hi);
                    if fits(mid) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                let mut e = BlendError::detail("blendG2TooLargeMax", &format!("{}: spring ends out of order", name.id()));
                e.params = vec![("max".into(), format!("{lo:.2}")), ("shape".into(), format!("{sh}"))];
                return Err(e);
            }
            dbg_fail(&format!("spring leaves face {} at {:?} (edge {:?} t=({:.4},{:.4}) corev {}, p0 {:?}, dir {:?}, leg1 {:?} leg2 {:?}, circ {})", b.face_id_str(f), t, e, ed.t0, ed.t1, b.coedge(co).rev, p0, dir, leg1, leg2, circ.is_some()));
            return Err(BlendError::detail("blendOverflow", &format!("{}: spring leaves its face away from its edges", name.id())));
        }
    }
    // tool profile in the plane perpendicular to dir through p0 - dir*margin
    // (line) or in the meridian plane through P0 (circle: revolved about the
    // axis). convex: the tool pierces the end faces (the cut trims it);
    // concave: GUESS end faces perpendicular to the edge, the tool ends
    // flush with them
    let margin = if convex && circ.is_none() { r.max(len * 0.05) + 1.0 } else { 0.0 };
    // concave: an end at a junction vertex runs on past the shortened edge end
    // to just inside the wall behind the original vertex
    let (mut m0, mut m1) = (margin, margin);
    if convex && circ.is_none() && !chamfer {
        // the tool pierces an end only if the body ends there; an end face that
        // leans toward the material (the edge runs into a reflex corner, e.g. a
        // pocket rim edge into the pocket wall) bounds the tool flush
        let flush_end = |v: ck_topo::body::VertexId, out: Vec3| -> bool {
            let mut ends: Vec<FaceId> = vec![];
            for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                for f in b.edge_faces(x) {
                    if !faces.contains(&f) && !ends.contains(&f) {
                        ends.push(f);
                    }
                }
            }
            ends.len() == 1 && matches!(b.face(ends[0]).surface, Surface::Plane { .. }) && b.planar_normal(ends[0]).dot(out) < -1e-9
        };
        let near = |q: Vec3, p: Vec3, o: Vec3| (q - p).cross(dir).len() <= 1e-6 && q.dist(p) < q.dist(o);
        if flush_end(ed.v0, -dir) || junc.flush.iter().any(|&q| near(q, p0, p1)) {
            m0 = 0.0;
        }
        if flush_end(ed.v1, dir) || junc.flush.iter().any(|&q| near(q, p1, p0)) {
            m1 = 0.0;
        }
    } else if convex && circ.is_none() && !junc.flush.is_empty() {
        // chamfer: only the ends where a tangent chain goes on stop flush
        let near = |q: Vec3, p: Vec3, o: Vec3| (q - p).cross(dir).len() <= 1e-6 && q.dist(p) < q.dist(o);
        if junc.flush.iter().any(|&q| near(q, p0, p1)) {
            m0 = 0.0;
        }
        if junc.flush.iter().any(|&q| near(q, p1, p0)) {
            m1 = 0.0;
        }
    }
    // an end that runs into the blend faces of an earlier feature (the convex rim of a tower after its wall coves were rounded):
    // the tool must not reach past the plane of the wall those blends are tangent to
    if convex && circ.is_none() {
        for (v, od, which) in [(ed.v0, -dir, 0usize), (ed.v1, dir, 1usize)] {
            let cur = if which == 0 { m0 } else { m1 };
            if cur == 0.0 {
                continue;
            }
            let pv = b.vertex(v).p;
            let mut end_faces: Vec<FaceId> = vec![];
            for x in b.edge_ids().filter(|&x| b.edge(x).v0 == v || b.edge(x).v1 == v) {
                for f in b.edge_faces(x) {
                    if !faces.contains(&f) && !end_faces.contains(&f) {
                        end_faces.push(f);
                    }
                }
            }
            if end_faces.is_empty() || !end_faces.iter().all(|&f| matches!(b.face(f).name.role, Role::Blend(_))) {
                continue;
            }
            let mut best: Option<f64> = None;
            for &f in &end_faces {
                for g in b.face_neighbors(f) {
                    if let Surface::Plane { frame } = &b.face(g).surface {
                        let n = b.planar_normal(g);
                        if n.cross(od).len() < 1e-6 {
                            let t = (frame.o - pv).dot(od);
                            if t > -1e-9 && t < cur && best.map_or(true, |x| t < x) {
                                best = Some(t.max(0.0));
                            }
                        }
                    }
                }
            }
            if let Some(t) = best {
                if which == 0 { m0 = t } else { m1 = t }
            }
        }
    }
    // reflex corner shared with another selected edge: run on past the vertex, clip to the bisector plane
    let mut clips: Vec<(Vec3, Vec3)> = vec![];
    if circ.is_none() {
        let mm = if convex { margin } else { r.max(len * 0.05) + 1.0 };
        for &(q, da, db) in &junc.mitre {
            let n = da - db;
            if n.len() < 1e-9 {
                continue;
            }
            if q.dist(p0) < 1e-6 && (da - dir).len() < 1e-6 {
                m0 = mm;
                clips.push((q, n.normalize()));
            } else if q.dist(p1) < 1e-6 && (da + dir).len() < 1e-6 {
                m1 = mm;
                clips.push((q, n.normalize()));
            }
        }
    }
    if !convex && circ.is_none() && !chamfer {
        for &(q, behind) in &junc.ext {
            if (q - p0).cross(dir).len() > 1e-6 {
                continue;
            }
            let (t0, t1) = ((p0 - q).dot(dir), (q - p1).dot(dir));
            let more = if behind { 0.25 * r } else { 0.0 };
            if t0 > 1e-6 {
                m0 = m0.max(t0 + more);
            }
            if t1 > 1e-6 {
                m1 = m1.max(t1 + more);
            }
        }
    }
    let o = p0 - dir * m0;
    let fr = match circ {
        None => Frame::from_normal(o, dir),
        Some((c, axis)) => {
            let rad = (p0 - c).normalize();
            ck_geom::frame_from(c, rad.cross(axis), rad)
        }
    };
    let uv = |p: Vec3| {
        let q = fr.world_to_uv(p);
        Vec2::new(q.x, q.y)
    };
    let (cp, t1, t2) = (uv(p0), uv(ta), uv(tb));
    let cc = uv(p0 + off);
    let lp = if ext {
        // chord t1-t2 plus an outer rim a margin beyond both faces
        let k = r;
        let dn = |n: Vec3| uv(p0 + n) - cp;
        let (d1, d2) = (dn(n1), dn(n2));
        let mut pts = vec![t1, t2, t2 + d2 * k];
        if !tangent {
            pts.push(cp + (d1 + d2).normalize() * (k * 1.5));
        }
        pts.push(t1 + d1 * k);
        let tags = ["arc", "o1", "o2", "o3", "o4"];
        let n = pts.len();
        ProfileLoop::new((0..n).map(|i| ProfileSeg::line(tags[i], pts[i], pts[(i + 1) % n])).collect())
    } else if let Some(seg) = match spec {
        Spec::Fillet(_, FilletProfile::Conic(rho)) => super::g2::conic_segment(rho, t1, cp, t2),
        Spec::Fillet(_, FilletProfile::Curvature(sh)) => super::g2::g2_segment(r, sh, phi, t1, cp, t2),
        _ => None,
    } {
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), seg])
    } else if matches!(spec, Spec::Chamfer(..)) {
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), ProfileSeg::line("arc", t1, t2)])
    } else {
        let short_ccw = (t1 - cc).cross(t2 - cc) > 0.0;
        let arc = if short_ccw {
            ProfileSeg::arc("arc", cc, t1, t2)
        } else if !convex {
            // concave: the tool's cap arcs ARE the end arcs; the oracle runs
            // them from the forward face's spring to the other one, n = (s1 -
            // c) x (s2 - c) (bracket F12 on S.e8|S.e9 along +x: n = -x)
            let x = (t1 - cc).normalize();
            let (a, b) = (t1 - cc, t2 - cc);
            let sweep = -(a.cross(b)).atan2(a.dot(b));
            ProfileSeg { tag: "arc".into(), curve: ck_geom::curve2::Curve2::Circle { c: cc, x, r: cc.dist(t1), ccw: false }, t0: 0.0, t1: sweep, rev: false }
        } else {
            ProfileSeg::arc("arc", cc, t2, t1).reversed()
        };
        ProfileLoop::new(vec![ProfileSeg::line("l2", t2, cp), ProfileSeg::line("l1", cp, t1), arc])
    };
    let prof = Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]).normalized();
    let made = match circ {
        None => extrude(feature, &prof, ExtrudeOpts { s0: 0.0, s1: len + m0 + m1, dir: None, draft: 0.0 }),
        Some((c, axis)) => {
            // a partial arc sweeps only its own angle (convex: a margin more at
            // both ends so the tool pierces the adjacent faces; concave:
            // flush, like the straight edge); a full circle is one turn
            use std::f64::consts::TAU;
            let span = sweep;
            let rad = (p0 - c).len();
            let (a0, a1) = if span >= TAU - 1e-9 {
                (0.0, TAU)
            } else {
                let m = if convex { (r.max(span * rad * 0.05) + 1.0) / rad } else { 0.0 };
                // an end that runs on into another arc of the same circle (a
                // split ring) stops flush: that arc is cut by its own tool
                let ends = [p0, ed.curve.eval(ed.t0.max(ed.t1))];
                let flush = |q: Vec3| (ext && (continues_circle(b, e, q, c, axis, rad) || meets_own_blend(b, e, q, feature, tol))) || junc.flush.iter().any(|f| f.dist(q) < 1e-6 * (1.0 + rad));
                let (m0, m1) = (if flush(ends[0]) { 0.0 } else { m }, if flush(ends[1]) { 0.0 } else { m });
                if span + m0 + m1 >= TAU { (0.0, TAU) } else { (-m0, span + m1) }
            };
            ck_topo::make::revolve::revolve(feature, &prof, c, axis, a0, a1)
        }
    };
    let mut tool = made.map_err(|e| {
        dbg_fail(&format!("tool: {e}"));
        BlendError::code("blendFailed")
    })?;
    for f in &mut tool.faces {
        if f.name.role == Role::Side("arc".into()) {
            f.name = name.clone();
        }
    }
    for (o, n) in clips {
        tool = clip_halfspace(&tool, o, n, tol)?;
    }
    let op = if convex { Op::Cut } else { Op::Union };
    // order: untouched; edge faces (ref order), end faces (start, end);
    // other touched faces; blend
    let ends: Vec<FaceId> = [ed.v0, ed.v1]
        .iter()
        .flat_map(|&v| {
            b.edge_ids()
                .filter(move |&x| b.edge(x).v0 == v || b.edge(x).v1 == v)
                .flat_map(|x| b.edge_faces(x))
                .filter(|f| !faces.contains(f))
                .collect::<Vec<_>>()
        })
        .collect();
    let mut hint: Vec<String> = faces.iter().map(|&f| b.face_id_str(f)).collect();
    for f in ends {
        let id = b.face_id_str(f);
        if !hint.contains(&id) {
            hint.push(id);
        }
    }
    if matches!(spec, Spec::Fillet(_, FilletProfile::Conic(_) | FilletProfile::Curvature(_))) {
        // the spline section touches the two faces tangentially along its end rulings (their
        // contact lines are the edges it shares with the tool's flat legs): no marching
        let blend_f = tool.face_ids().find(|&f| tool.face(f).name == name);
        let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> { (Some(g) == blend_f && faces.contains(&f)).then(Vec::new) };
        return apply_tool_hooked(b, &tool, op, &name, hint, tol, Some(&hook));
    }
    apply_tool(b, &tool, op, &name, hint, tol)
}

/// Boolean `b op tool`, faces ordered untouched / `hint` (touched, in that
/// order) / the tool's new faces; returns the body and the hint.
pub(super) fn apply_tool(b: &Body, tool: &Body, op: Op, name: &FaceName, hint: Vec<String>, tol: f64) -> Result<(Body, Vec<String>), BlendError> {
    apply_tool_hooked(b, tool, op, name, hint, tol, None)
}

/// [`apply_tool`] with a per-face-pair SSI override (see
/// [`crate::boolean::imprint::PairHook`]).
///
/// Marched intersection curves meet their neighbours only to ~1e-6, so a
/// corner whose cut curve ends 1.5e-6 short of an existing vertex (the
/// vertical fillet at one of four symmetric chamfered corners) leaves the
/// arrangement open at `tol`. Like `boolean::boolean`, a failure is retried
/// with the welding tolerance raised 10x and 100x; a result that closes but
/// falls outside the volume bounds the operands allow (`boolean::sanity`)
/// counts as a failure too.
pub(super) fn apply_tool_hooked(
    b: &Body,
    tool: &Body,
    op: Op,
    name: &FaceName,
    hint: Vec<String>,
    tol: f64,
    hook: Option<crate::boolean::imprint::PairHook>,
) -> Result<(Body, Vec<String>), BlendError> {
    let mut first = None;
    // a tangent-contact (hooked) tool also gets a last try with a tight weld: a body with sliver faces thinner than the
    // loose weld (bands stacked a few thousandths apart) collapses them and does not assemble
    let ks: &[f64] = if hook.is_some() { &[1.0, 10.0, 100.0, 0.1] } else { &[1.0, 10.0, 100.0] };
    for &k in ks {
        match apply_tool_once(b, tool, op, name, hint.clone(), tol * k, hook) {
            Ok(r) => match crate::boolean::sanity::check_result(b, tool, op, &r.0) {
                None => return Ok(r),
                Some(m) => {
                    dbg_fail(&format!("sanity at tol {:e}: {} {} outside [{}, {}]", tol * k, m.what, m.got, m.expected_min, m.expected_max));
                    first.get_or_insert(BlendError::code("blendFailed"));
                }
            },
            Err(e) => {
                first.get_or_insert(e);
            }
        }
    }
    Err(first.unwrap_or_else(|| BlendError::code("blendFailed")))
}

fn apply_tool_once(
    b: &Body,
    tool: &Body,
    op: Op,
    name: &FaceName,
    hint: Vec<String>,
    tol: f64,
    hook: Option<crate::boolean::imprint::PairHook>,
) -> Result<(Body, Vec<String>), BlendError> {
    let hook_given = hook.is_some();
    let dbg_t = std::env::var_os("CK_BLEND_DEBUG").is_some().then(std::time::Instant::now);
    let arr = crate::boolean::imprint::imprint_hooked(b, tool, tol, hook);
    if let Some(t) = dbg_t {
        eprintln!("apply_tool: imprint {:.1}s", t.elapsed().as_secs_f64());
    }
    let dbg_t = std::env::var_os("CK_BLEND_DEBUG").is_some().then(std::time::Instant::now);
    // a hooked (tangent contact) tool is imprinted at a loose welding tolerance; classify against it tightly
    let class_tol = if hook_given { tol.min(1e-5) } else { tol };
    let mut faces_out = crate::boolean::assemble::select_class_tol(&arr, [b, tool], op, class_tol).map_err(|e| {
        dbg_fail(&format!("select: {e:?}"));
        BlendError::code("blendFailed")
    })?;
    if let Some(t) = dbg_t {
        eprintln!("apply_tool: select {:.1}s", t.elapsed().as_secs_f64());
    }
    for f in &mut faces_out {
        if f.group == 1 {
            let mut base = f.name.clone();
            base.region = None;
            let pos = hint.iter().position(|h| FaceName::parse(h).map(|mut x| {
                x.region = None;
                x
            }) == Some(base.clone()));
            f.key = (1, pos.unwrap_or(hint.len()), f.key.1 * 1000 + f.key.2);
        } else if f.group == 4 {
            f.key = (1, hint.len() + 1, f.key.1);
        }
        // walk items follow the face key (edge numbering = face-walk order)
        let key = f.key;
        for w in &mut f.walk {
            w.0 = key;
        }
    }
    faces_out.sort_by_key(|f| f.key);
    // the blend face may have merged into the coincident blend face of the
    // ring's previous arc (a split ring chamfered edge by edge: one cone)
    let merged_into_earlier = faces_out.iter().any(|f| (f.group == 0 || f.group == 4) && f.name.feature == name.feature && matches!(f.name.role, Role::Blend(_)));
    if !faces_out.iter().any(|f| { let mut n = f.name.clone(); n.region = None; n == *name }) && !merged_into_earlier {
        dbg_fail("no blend face in result");
        return Err(BlendError::code("blendFailed"));
    }
    let mut body = build(&arr, &faces_out);
    super::freeform::drop_zero_edges(&mut body, 1e-4);
    let bad = ck_topo::check::check(&body, tol)
        .into_iter()
        .any(|i| matches!(i, ck_topo::check::Issue::OpenEdge(_) | ck_topo::check::Issue::BrokenLoop(..)));
    if bad {
        dbg_fail(&format!("check: {:?}", ck_topo::check::check(&body, tol)));
        if std::env::var_os("CK_BLEND_DEBUG").is_some() {
            eprintln!("  in : {:?}", b.face_ids().map(|f| b.face_id_str(f)).collect::<Vec<_>>());
            eprintln!("  out: {:?}", body.face_ids().map(|f| body.face_id_str(f)).collect::<Vec<_>>());
            for i in ck_topo::check::check(&body, tol) {
                if let ck_topo::check::Issue::OpenEdge(e) = i {
                    let ed = body.edge(e);
                    eprintln!(
                        "  open {:?} {} t=({:.6},{:.6}) a={:?} b={:?} faces={:?}",
                        e, ed.curve.kind(), ed.t0, ed.t1, body.vertex(ed.v0).p, body.vertex(ed.v1).p,
                        body.edge_faces(e).iter().map(|&f| body.face_id_str(f)).collect::<Vec<_>>()
                    );
                }
            }
        }
        return Err(BlendError::code("blendFailed"));
    }
    Ok((body, hint))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;
    use ck_topo::mass::mass_props;

    fn the_box() -> Body {
        box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0)
    }
    fn ids(b: &Body) -> Vec<String> {
        b.face_ids().map(|f| b.face_id_str(f)).collect()
    }
    fn mass(b: &Body, vol: f64, area: f64, com: [f64; 3]) {
        let m = mass_props(b);
        assert!((m.volume - vol).abs() < 1e-9 * vol, "{} vs {vol}", m.volume);
        assert!((m.area - area).abs() < 1e-9 * area, "{} vs {area}", m.area);
        assert!((m.com - Vec3::new(com[0], com[1], com[2])).len() < 1e-8, "{:?}", m.com);
    }
    fn e(a: &str, b: &str, at: [f64; 3]) -> (String, String, Vec3) {
        (a.into(), b.into(), Vec3::new(at[0], at[1], at[2]))
    }

    #[test]
    fn oracle_fillet_one_edge() {
        let r = fillet(&the_box(), &[e("F2.C1", "F2.S.e1", [20., 10., 0.])], 2.0, "F3", 1e-6).unwrap();
        assert!(ck_topo::check::check(&r, 1e-6).is_empty());
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e3", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F3.B0"]);
        mass(&r, 7965.6637061435895, 2763.946891450771, [20.000000000000004, 4.980372983260395, -10.041179705330975]);
        let b0 = r.find_face("F3.B0").unwrap();
        assert_eq!(r.face(b0).surface.kind(), "cylinder");
        // end-face arcs: c = (0,8,-2) / (40,8,-2), r = 2
        let arcs: Vec<(Vec3, f64)> = r
            .face_edges(b0)
            .into_iter()
            .filter_map(|x| r.edge(x).curve.circle_info().map(|(c, _, rr)| (c, rr)))
            .collect();
        assert_eq!(arcs.len(), 2);
        for (c, rr) in arcs {
            assert!((rr - 2.0).abs() < 1e-12);
            assert!((c.y - 8.0).abs() < 1e-12 && (c.z + 2.0).abs() < 1e-12);
        }
    }

    #[test]
    fn oracle_fillet_two_edges_and_vertical() {
        let r = fillet(
            &the_box(),
            &[e("F2.C1", "F2.S.e1", [20., 10., 0.]), e("F2.C1", "F2.S.e3", [20., 10., -20.])],
            2.0,
            "F3",
            1e-6,
        )
        .unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F2.S.e3", "F3.B0", "F3.B1"]);
        mass(&r, 7931.327412287184, 2727.8937829015417, [20., 4.960576028002486, -10.]);
        let r = fillet(&the_box(), &[e("F2.S.e1", "F2.S.e2", [40., 5., 0.])], 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.S.e3", "F2.S.e4", "F2.S.e1", "F2.S.e2", "F2.C0", "F2.C1", "F3.B0"]);
        mass(&r, 7991.4159265358985, 2789.6991118430774, [19.978996631231546, 5., -10.010261751083915]);
    }

    #[test]
    fn cylinder_top_circle() {
        // oracle volumes (tools/brep_cases.mjs filletCylTop / chamferCylTop)
        let cyl = crate::boolean::tests::cyl_body("F2", 5.0, 3.0, 10.0, 0.0, 10.0);
        let top = cyl.face_ids().find(|&f| cyl.face_id_str(f) == "F2.C1").unwrap();
        let side = cyl.face_ids().find(|&f| matches!(cyl.face(f).surface, Surface::Cylinder { .. })).unwrap();
        let (a, bb) = (cyl.face_id_str(top), cyl.face_id_str(side));
        let at = Vec3::new(15., 10., -3.);
        let r = fillet(&cyl, &[(a.clone(), bb.clone(), at)], 2.0, "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - 3090.0668137679036).abs() < 1e-6);
        assert_eq!(r.face(r.find_face("F3.B0").unwrap()).surface.kind(), "torus");
        let r = chamfer(&cyl, &[(a, bb, at)], 1.0, "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - 3111.223924605093).abs() < 1e-6);
        assert_eq!(r.face(r.find_face("F3.B0").unwrap()).surface.kind(), "cone");
    }

    #[test]
    fn oracle_chamfers_on_the_box() {
        // chamferOne / chamferTwo / chamferRail / chamferBig (oracle)
        let b = the_box();
        let one = [e("F2.C1", "F2.S.e1", [20., 10., 0.])];
        let r = chamfer(&b, &one, 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e3", "F2.C1", "F2.S.e1", "F2.S.e4", "F2.S.e2", "F3.B0"]);
        assert!((mass_props(&r).volume - 7920.0).abs() < 1e-9);
        let two = [one[0].clone(), e("F2.C1", "F2.S.e3", [20., 10., -20.])];
        assert!((mass_props(&chamfer(&b, &two, 2.0, "F3", 1e-6).unwrap()).volume - 7840.0).abs() < 1e-9);
        assert_eq!(
            chamfer(&b, &one, 30.0, "F3", 1e-6).unwrap_err(),
            BlendError::detail("blendOverflow", "F3.B0: spring leaves its face away from its edges")
        );
        // a ring of four runs in rounds; blend names stay selection-indexed
        let ring = [
            e("F2.C1", "F2.S.e1", [20., 10., 0.]),
            e("F2.C1", "F2.S.e2", [40., 10., -10.]),
            e("F2.C1", "F2.S.e3", [20., 10., -20.]),
            e("F2.C1", "F2.S.e4", [0., 10., -10.]),
        ];
        let r = chamfer(&b, &ring, 2.0, "F3", 1e-6).unwrap();
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e1", "F2.S.e2", "F2.S.e3", "F3.B1", "F2.C1", "F2.S.e4", "F3.B2", "F3.B0", "F3.B3"]);
    }

    #[test]
    fn vertex_chamfer_on_the_box() {
        use crate::blend::vertex::chamfer_vertex;
        let b = the_box();
        // distances 2 / 3 / 4 along the vertex edges in arena order (rail, +x, -z)
        let r = chamfer_vertex(&b, Vec3::new(0., 10., 0.), [2., 3., 4.], "F3", 1e-6).unwrap();
        assert!((mass_props(&r).volume - (8000.0 - 2.0 * 3.0 * 4.0 / 6.0)).abs() < 1e-9);
        assert_eq!(ids(&r), ["F2.C0", "F2.S.e2", "F2.S.e3", "F2.S.e1", "F2.C1", "F2.S.e4", "F3.B0"]);
        // oracle normal (-0.512, 0.768, 0.384)
        let n = r.planar_normal(r.find_face("F3.B0").unwrap());
        assert!((n - Vec3::new(-4., 6., 3.).normalize()).len() < 1e-12, "{n:?}");
        assert_eq!(chamfer_vertex(&b, Vec3::new(0., 10., 0.), [10., 1., 1.], "F3", 1e-6).unwrap_err(), BlendError::code("blendTooLarge"));
        assert_eq!(chamfer_vertex(&b, Vec3::new(0., 10., 0.), [1., 0., 1.], "F3", 1e-6).unwrap_err(), BlendError::code("invalidValue"));
    }

    #[test]
    fn oracle_fillet_errors() {
        let b = the_box();
        let one = [e("F2.C1", "F2.S.e1", [20., 10., 0.])];
        assert_eq!(fillet(&b, &one, 0.0, "F3", 1e-6).unwrap_err(), BlendError::code("invalidValue"));
        assert_eq!(fillet(&b, &[], 2.0, "F3", 1e-6).unwrap_err(), BlendError::code("missingEdge"));
        assert_eq!(
            fillet(&b, &[e("F2.C1", "F2.S.e9", [20., 10., 0.])], 2.0, "F3", 1e-6).unwrap_err(),
            BlendError::code("missingEdge")
        );
        assert_eq!(
            fillet(&b, &one, 30.0, "F3", 1e-6).unwrap_err(),
            BlendError::detail("blendOverflow", "F3.B0: spring leaves its face away from its edges")
        );
    }

    #[test]
    fn concave_fillet_adds_material() {
        // L-shape: box with a step cut → concave edge at the step
        let a = the_box();
        let tool = box_body("F4", 0.0, 0.0, 20.0, 20.0, 5.0, 11.0);
        let l = crate::boolean::boolean(&a, &tool, Op::Cut, 1e-6).unwrap().body;
        let v0 = mass_props(&l).volume;
        // concave edge between the step floor (y=5) and the step wall (x=20)
        let floor = l.face_ids().find(|&f| l.face_id_str(f).starts_with("F4.C0")).unwrap();
        let wall = l.face_ids().find(|&f| l.face_id_str(f) == "F4.S.e2").unwrap();
        let (fa, fb) = (l.face_id_str(floor), l.face_id_str(wall));
        let r = fillet(&l, &[(fa, fb, Vec3::new(20., 5., -10.))], 1.0, "F5", 1e-6).unwrap();
        let v = mass_props(&r).volume;
        let add = (1.0 - std::f64::consts::PI / 4.0) * 20.0;
        assert!((v - v0 - add).abs() < 1e-9 * v, "{v} {v0}");
    }
}

/// Circle through a curve's samples on `[t0, t1]`: `(centre, axis, radius,
/// swept angle)`, the axis oriented so increasing `t` runs counter-clockwise.
/// `None` unless every sample lies on that circle (1e-6 relative).
fn fit_circle(c: &Curve, t0: f64, t1: f64) -> Option<(Vec3, Vec3, f64, f64)> {
    const N: usize = 48;
    let p = |i: usize| c.eval(t0 + (t1 - t0) * i as f64 / N as f64);
    let (a, m, d) = (p(0), p(N / 3), p(2 * N / 3));
    let (u, v) = (m - a, d - a);
    let w = u.cross(v);
    let w2 = w.dot(w);
    if w2 < 1e-24 {
        return None;
    }
    let ctr = a + (v.cross(w) * u.dot(u) + w.cross(u) * v.dot(v)) / (2.0 * w2);
    let axis = w.normalize();
    let rad = a.dist(ctr);
    if !(rad > 0.0) {
        return None;
    }
    let (mut ang, mut prev) = (0.0, a - ctr);
    for i in 1..=N {
        let q = p(i) - ctr;
        if (q.len() - rad).abs() > 1e-6 * rad || q.dot(axis).abs() > 1e-6 * rad {
            return None;
        }
        ang += prev.cross(q).dot(axis).atan2(prev.dot(q));
        prev = q;
    }
    (ang > 1e-6).then_some((ctr, axis, rad, ang.min(std::f64::consts::TAU)))
}

/// Is this B-spline face a cone / cylinder about the axis `(c, axis)`
/// (radius linear in height over a sample grid, 1e-6 relative)? A lofted
/// side between two coaxial circles is one.
fn bspline_coaxial_cone(sf: &Surface, c: Vec3, axis: Vec3) -> bool {
    let ((u0, u1), (v0, v1)) = sf.domain();
    if ![u0, u1, v0, v1].iter().all(|x| x.is_finite()) {
        return false;
    }
    let mut hr: Vec<(f64, f64)> = vec![];
    for i in 0..=8 {
        for j in 0..=8 {
            let q = sf.eval(u0 + (u1 - u0) * i as f64 / 8.0, v0 + (v1 - v0) * j as f64 / 8.0) - c;
            let h = q.dot(axis);
            hr.push((h, (q - axis * h).len()));
        }
    }
    let (lo, hi) = hr.iter().fold((f64::MAX, f64::MIN), |(l, h), x| (l.min(x.0), h.max(x.0)));
    let scale = hr.iter().map(|x| x.1.max(x.0.abs())).fold(0.0, f64::max);
    if !(hi - lo > 1e-6 * scale) {
        return false;
    }
    let a = hr.iter().find(|x| x.0 == lo).unwrap();
    let z = hr.iter().find(|x| x.0 == hi).unwrap();
    let slope = (z.1 - a.1) / (z.0 - a.0);
    hr.iter().all(|x| (a.1 + slope * (x.0 - a.0) - x.1).abs() < 1e-6 * scale)
}

/// Do the two faces of `e` meet tangentially (G1) along it?
fn is_tangent_edge(b: &Body, e: EdgeId) -> bool {
    let fs = b.edge_faces(e);
    if fs.len() != 2 {
        return false;
    }
    let ed = b.edge(e);
    let pm = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
    let n = |f: FaceId| {
        let (u, v) = b.face(f).surface.closest_param(pm);
        b.outward_normal(f, u, v).normalize()
    };
    (1.0 - n(fs[0]).dot(n(fs[1]))).abs() < 1e-9
}

/// Does another edge of `b` leave the point `q` on the same circle
/// `(c, axis, rad)` as edge `e` (a ring split in several edges)?
fn continues_circle(b: &Body, e: EdgeId, q: Vec3, c: Vec3, axis: Vec3, rad: f64) -> bool {
    b.edge_ids().filter(|&x| x != e).any(|x| {
        let ed = b.edge(x);
        let (t0, t1) = (ed.t0.min(ed.t1), ed.t0.max(ed.t1));
        if ed.curve.eval(t0).dist(q) > 1e-7 * (1.0 + rad) && ed.curve.eval(t1).dist(q) > 1e-7 * (1.0 + rad) {
            return false;
        }
        let fit = ed.curve.circle_info().map(|(c2, a2, r2)| (c2, a2.normalize(), r2)).or_else(|| fit_circle(&ed.curve, t0, t1).map(|x| (x.0, x.1, x.2)));
        fit.map_or(false, |(c2, a2, r2)| (c2 - c).len() < 1e-7 * (1.0 + rad) && a2.cross(axis).len() < 1e-7 && (r2 - rad).abs() < 1e-7 * (1.0 + rad))
    })
}

/// Does another edge of `b` leave `q` along a face blended by `feature`
/// earlier in this feature (the ring's previous arc, already chamfered: its blend
/// face or the flush end wall the tool left)?
fn meets_own_blend(b: &Body, e: EdgeId, q: Vec3, feature: &str, tol: f64) -> bool {
    b.edge_ids().filter(|&x| x != e).any(|x| {
        let ed = b.edge(x);
        (b.vertex(ed.v0).p.dist(q) <= tol.max(1e-7) * 10.0 || b.vertex(ed.v1).p.dist(q) <= tol.max(1e-7) * 10.0)
            && b.edge_faces(x).iter().any(|&f| {
                let n = &b.face(f).name;
                n.feature == feature && matches!(n.role, Role::Blend(_) | Role::Cap(_))
            })
    })
}
