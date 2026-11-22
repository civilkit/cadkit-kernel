//! The selection driver: tangent-chain retries, rounds of non-conflicting edges, one blend op per round.

use super::BlendError;
use crate::boolean::Op;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, Role};
use super::{analytic::*, apply::*, chain::*, diag::*, ends::*, find::*, layout::*, model::*};

/// Shared fillet / chamfer driver (see [`fillet`]).
pub fn blend_edges(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    // prototype: a concave vertical edge that ends against a rim fillet, built in place (CK_BLEND_INPLACE=1)
    if super::inplace::enabled() && matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) {
        if let Some(res) = super::inplace::try_pick(b, edges, spec.size(), feature) {
            let res = res.and_then(|out| super::validity::check(b, out, edges, spec, tol));
            if res.is_ok() {
                return res;
            }
        }
    }
    // a fillet on a procedural (sweep or blend) face is made on its B-spline, so that the guard judges the result against it too
    if matches!(spec, Spec::Fillet(..)) {
        if let Some(c) = super::freeform::procedural_to_bspline(b, edges) {
            return blend_edges(&c, edges, spec, feature, tol);
        }
    }
    // a plane-plane-cylinder corner whose patch does not fit at this radius is refused up front, in every pick order, with its
    // reason: a retry (the free-form strip for the vertical edge) would otherwise build the same selection as a mitre in some orders
    if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && EDGE_RADII.with(|c| c.borrow().is_none()) {
        let eids: Vec<EdgeId> = with_ring_siblings(b, edges).iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
        for corner in super::corner::find_corner(b, &eids).into_iter().filter(|c| c.curved) {
            if let Err(mut e) = super::corner::cyl_corner_geom(b, &corner, spec.size()) {
                e.params.push(("hint".into(), "use a smaller radius, or round the rim and the wall edges in separate features".into()));
                return Err(e);
            }
        }
    }
    // Every attempt is judged by the validity gate: a result that is not a closed valid solid, or that changed the volume
    // by more than the blends can explain, is an error (and lets the next attempt run).
    let rejected: std::cell::RefCell<Option<BlendError>> = std::cell::RefCell::new(None);
    let attempt = || {
        let out = blend_edges_inner(b, edges, spec, feature, tol)?;
        super::validity::check(b, out, edges, spec, tol).map_err(|e| {
            rejected.borrow_mut().get_or_insert_with(|| e.clone());
            e
        })
    };
    // A fillet or chamfer along a chain of edges that continue each other tangentially (a line into an arc of a rounded outline) must
    // stop every tool flush at the tangent vertex: a tool that runs on past it (a straight prism into the arc, a revolved
    // tool into the straight run) leaves a planar step between the two blend faces. Tried first; the ordinary ends are the
    // fallback.
    // (the arcs of a split ring that a picked arc brings in count: `blend_edges_inner` rounds them with it, and the seam between
    // them must end flush, or it leaves slivers)
    let mates_in = |e: &[(String, String, Vec3)]| if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) { has_tangent_mates(b, &with_ring_siblings(b, e)) } else { has_tangent_mates(b, e) };
    if edges.len() > 1 && !TANGENT_FLUSH.with(|c| c.get()) && mates_in(edges) {
        TANGENT_FLUSH.with(|c| c.set(true));
        let flush = attempt();
        TANGENT_FLUSH.with(|c| c.set(false));
        if flush.is_ok() {
            return flush;
        }
    }
    let first = attempt();
    // A selection that is a chain of tangent edges (a rounded outline picked edge by edge) fails when a tool runs on past
    // the junction into the arc's band: retry with the ends of tangent-continuing edges stopping flush. Selections that
    // worked keep their result.
    if first.is_err() && edges.len() > 1 && !TANGENT_FLUSH.with(|c| c.get()) {
        TANGENT_FLUSH.with(|c| c.set(true));
        let second = attempt();
        TANGENT_FLUSH.with(|c| c.set(false));
        if second.is_ok() {
            return second;
        }
    }
    // chamfers: a vertex where edges of both convexities meet closes in some round orders only; the first valid result
    // of concave-first, then convex-first wins (the selection's own order keeps priority when it works)
    if first.is_err() && edges.len() > 1 && matches!(spec, Spec::Chamfer(..)) {
        for alt in [1u8, 2] {
            CHAMFER_ORDER.with(|c| c.set(alt));
            let r = attempt();
            CHAMFER_ORDER.with(|c| c.set(0));
            if r.is_ok() {
                return r;
            }
        }
    }
    // a selection with wall lines the analytic builder refused: the free-form strips, as before that builder took them
    if first.is_err() && edges.iter().any(|(fa, fb, at)| find_edge(b, fa, fb, *at).is_some_and(|e| super::freeform::needs_freeform(b, e) && super::torus_corner::analytic_wall_edge(b, e))) {
        if let Ok(x) = super::torus_corner::free_walls(attempt) {
            return Ok(x);
        }
    }
    // when every attempt failed and the gate refused one of them, that refusal says more than a later attempt's failure
    let res = match (first, rejected.into_inner()) {
        (Err(_), Some(gate)) => Err(gate),
        (r, _) => r,
    };
    // the edges of strip faces whose rounds are wider together than the strip: built as crossing rounds (cross.rs),
    // only when the ordinary build refused it as an overflow
    if res.as_ref().is_err_and(|e| e.code == "blendOverflow") && matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && edges.len() >= 2 {
        let radii = EDGE_RADII.with(|c| c.borrow().clone()).filter(|v| v.len() == edges.len()).unwrap_or_else(|| vec![spec.size(); edges.len()]);
        if edges.len() == 2 {
            if let Some(Ok(out)) = super::cross::try_crossing(b, edges, [radii[0], radii[1]], feature, tol) {
                return Ok(out);
            }
        }
        // several walls at once (cross_multi.rs)
        if let Some(Ok(out)) = super::cross_multi::try_crossing_multi(b, edges, &radii, feature, tol) {
            return Ok(out);
        }
    }
    if res.is_err() && matches!(spec, Spec::Fillet(..)) {
        if let Some(e) = curved_corner_refusal(b, edges, spec.size()) {
            return Err(e);
        }
    }
    res
}

/// A refused fillet whose selection has all three edges of a vertex, where a face or an edge there is curved (the rim arc,
/// the wall-top edge and a vertical wall edge of a stepped disc). Two planes and a cylinder have a vertex patch
/// (`corner::cyl_corner_tool`); the reason it cannot be built at this radius is the refusal's detail. Any other curved corner
/// has no patch (`corner.rs` takes planar faces only), and the tools of its three edges leave it open: say so, instead of the
/// open-edge check or a size hint that does not hold (such a corner builds at some sizes and not at others).
fn curved_corner_refusal(b: &Body, edges: &[(String, String, Vec3)], r: f64) -> Option<BlendError> {
    let eids: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    for v in b.vertex_ids() {
        let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
        if inc.len() != 3 || !inc.iter().all(|e| eids.contains(e)) {
            continue;
        }
        let straight = inc.iter().all(|&e| matches!(b.edge(e).curve, ck_geom::curve::Curve::Line { .. }));
        let planar = inc.iter().flat_map(|&e| b.edge_faces(e)).all(|f| matches!(b.face(f).surface, ck_geom::surface::Surface::Plane { .. }));
        if straight && planar {
            continue;
        }
        let p = b.vertex(v).p;
        if let Some(c) = super::corner::find_corner(b, &eids).into_iter().find(|c| c.curved && c.vertex == v) {
            let mut e = match super::corner::cyl_corner_geom(b, &c, r) {
                Err(e) => e,
                Ok(_) => BlendError::detail(
                    "blendUnsupported",
                    &format!("the corner of two planes and a cylinder at ({:.3}, {:.3}, {:.3}) could not be built at radius {r}: its three rounds do not join the corner patch cleanly", p.x, p.y, p.z),
                ),
            };
            e.params.push(("hint".into(), "use a smaller radius, or round the rim and the wall edges in separate features".into()));
            return Some(e);
        }
        let mut e = BlendError::detail(
            "blendUnsupported",
            &format!("three rounded edges meet at a corner with a curved face or edge ({:.3}, {:.3}, {:.3}); the corner patch is only built where all three faces are planar, or two planes and a cylinder", p.x, p.y, p.z),
        );
        e.params.push(("hint".into(), "round the rim and the wall edges in separate features, or leave one of the three edges at that corner out of the selection".into()));
        return Some(e);
    }
    None
}

/// Two of the selected edges continue each other tangentially at a shared vertex.
pub(super) fn has_tangent_mates(b: &Body, edges: &[(String, String, Vec3)]) -> bool {
    let eids: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    eids.len() > 1 && !tangent_flush_vertices(b, &eids).is_empty()
}

/// Chamfer of a selection that holds overhang corners: stage one the picks at the corners (`at_corner`), stage two
/// the others on the result, blend names following the selection. `None` when either stage fails (the caller
/// builds the selection in one pass, the corners then ending in planar steps).
fn chamfer_corners_first(b: &Body, edges: &[(String, String, Vec3)], at_corner: &[bool], spec: Spec, feature: &str, tol: f64) -> Option<Body> {
    let idx1: Vec<usize> = (0..edges.len()).filter(|&i| at_corner[i]).collect();
    let idx2: Vec<usize> = (0..edges.len()).filter(|&i| !at_corner[i]).collect();
    let sel1: Vec<_> = idx1.iter().map(|&i| edges[i].clone()).collect();
    let sel2: Vec<_> = idx2.iter().map(|&i| edges[i].clone()).collect();
    let mut body1 = blend_edges_inner(b, &sel1, spec, feature, tol).ok()?;
    let renum = |body: &mut Body, from: &str, map: &[usize], spare: &mut u32| {
        for f in body.face_ids().collect::<Vec<_>>() {
            let nm = body.face(f).name.clone();
            if nm.feature != from {
                continue;
            }
            body.faces[f.idx()].name.feature = feature.to_string();
            if let Role::Blend(k) = nm.role {
                let new = map.get(k as usize).map_or_else(|| { *spare += 1; *spare - 1 }, |&i| i as u32);
                body.faces[f.idx()].name.role = Role::Blend(new);
            }
        }
    };
    // stage one already carries the feature's name: renumber its blend faces to the selection's numbers first
    let mut spare = edges.len() as u32;
    for f in body1.face_ids().collect::<Vec<_>>() {
        let nm = body1.face(f).name.clone();
        if nm.feature == feature {
            if let Role::Blend(k) = nm.role {
                if let Some(&i) = idx1.get(k as usize) {
                    body1.faces[f.idx()].name.role = Role::Blend(i as u32);
                }
            }
        }
    }
    let tmp = format!("{feature}_b");
    let mut body2 = blend_edges_inner(&body1, &sel2, spec, &tmp, tol).ok()?;
    renum(&mut body2, &tmp, &idx2, &mut spare);
    Some(body2)
}

pub(super) fn blend_edges_inner(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
    SWEPT.with(|w| w.borrow_mut().clear());
    CYL_CORNER.with(|c| c.set(false));
    ORIGINAL.with(|o| *o.borrow_mut() = Some(b.clone()));
    let r = spec.size();
    if !(r > 0.0) || !r.is_finite() {
        return Err(BlendError::code("invalidValue"));
    }
    if edges.is_empty() {
        return Err(BlendError::code("missingEdge"));
    }
    if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) {
        // a concave vertex of three equal fillets with a free-form edge: staged build with a sphere patch
        if let Some(Ok(done)) = super::concave_vertex::try_fillet(b, edges, r, feature, tol) {
            return Ok(done);
        }
    }
    if matches!(spec, Spec::Fillet(..)) {
        let free = edges.iter().any(|(fa, fb, at)| find_edge(b, fa, fb, *at).is_some_and(|e| super::freeform::needs_freeform(b, e) && !super::torus_corner::analytic_wall_edge(b, e)));
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
        // overhang corners: four selected edges as two collinear pairs that flip convex/concave;
        // their stripes are trimmed by the zone swap after the rounds (see super::overhang).
        // A structural corner with skewed pair directions cannot take the patch (the zone box and
        // the boundary chains are built for perpendicular pairs): refuse it with a hint instead of
        // silently leaving the stepped stripes, and keep the ordinary path for every other shape.
        // per-pair radii come from the selection (`EDGE_RADII`, set by fillet_unequal);
        // a constant selection is one radius for every edge
        let eir: Vec<f64> = EDGE_RADII
            .with(|c| c.borrow().clone())
            .filter(|v| v.len() == eids.len())
            .unwrap_or_else(|| vec![r; eids.len()]);
        // chamfers of equal distance take the same corner (its zone and hexagon are those of a fillet of radius d);
        // any other chamfer at such a corner is refused below
        let overhangs_all: Vec<super::overhang::Corner> = match spec {
            Spec::Fillet(..) => super::overhang::detect(b, &eids, &eir),
            Spec::Chamfer(d, ChamferLeg::Dist(d2)) if (d - d2).abs() < 1e-12 => super::overhang::detect(b, &eids, &vec![d; n])
                .into_iter()
                .map(|mut c| {
                    c.chamfer = true;
                    c
                })
                .collect(),
            Spec::Chamfer(..) => {
                // structural test only (the radius does not matter for it)
                if let Some(c) = super::overhang::detect(b, &eids, &vec![1.0; n]).first() {
                    return Err(BlendError {
                        detail: Some(format!(
                            "a chamfer with two different distances or an angle cannot meet the corner at ({:.3}, {:.3}, {:.3}) where a convex and a concave edge pair all four selected: the corner patch is a mitre through the ends of the four strips, and with unequal legs the strips of the concave pair and of the convex pair stop at different distances on the shared faces, so their ends do not meet and the strips would stop in planar steps",
                            c.v.x, c.v.y, c.v.z
                        )),
                        params: vec![(
                            "hint".into(),
                            "use an equal-distance chamfer for these four edges, chamfer the concave pair and the convex pair in separate features, or leave the four edges at this vertex out of the selection".into(),
                        )],
                        ..BlendError::code("blendCorner")
                    });
                }
                vec![]
            }
        };
        if let Some(c) = overhangs_all.iter().find(|c| !c.patch_ok) {
            return Err(c.refusal(None));
        }
        let overhangs: Vec<super::overhang::Corner> = overhangs_all;
        // chamfers: the edges of the overhang corners are built first, on their own (each corner's zone is rebuilt from
        // its four strips), and the rest of the selection is chamfered on that body. Built in one pass the other chamfers
        // (a vertex of mixed convexity's, say) reshape the body about a corner and its patch cannot be joined.
        if chamfer_only(spec) && !overhangs.is_empty() {
            let at_corner: Vec<bool> = eids.iter().map(|e| overhangs.iter().any(|c| c.edges.contains(e))).collect();
            if at_corner.iter().any(|&c| !c) {
                if let Some(done) = chamfer_corners_first(b, edges, &at_corner, spec, feature, tol) {
                    return Ok(done);
                }
            }
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
                let mut alt = CHAMFER_ORDER.with(|c| c.get());
                let concave: Vec<bool> = eids.iter().map(|&e| super::mixed::line_edge_concave(b, e).unwrap_or(false)).collect();
                // a convex and a concave pick that share a vertex whose other edge is not picked: chamfered in the order of the
                // selection the first strip's end is cut by the second tool in one order (a tool face or a planar step is left
                // in the part) and ends in a clean cap in the other, so the result depended on the pick order. The convex pick
                // goes first, whatever the selection's order (a stable sort: the other picks keep theirs)
                if alt == 0 && has_mixed_pair(b, &eids, &concave) {
                    alt = 2;
                }
                order.sort_by_key(|&i| (!tang[i], match alt { 1 => !concave[i], 2 => concave[i], _ => false }));
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
        let mut rounds = rounds;
        // (a plane-plane-cylinder corner is built from one round, whatever order the edges were picked in: see `seq` in
        // `blend_edges_at`; the trims are set just below)
        let curved_corner = matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && EDGE_RADII.with(|c| c.borrow().is_none()) && super::corner::find_corner(b, &eids).iter().any(|c| c.curved);
        if curved_corner {
            rounds = vec![(0..n).collect()];
        }
        if !overhangs.is_empty() && !chamfer_only(spec) {
            // the four edges of a corner are one round whatever their chain links say
            let oh: Vec<usize> = overhangs.iter().flat_map(|c| c.edges.iter().filter_map(|&e| eids.iter().position(|&x| x == e))).collect();
            if let Some(first) = rounds.iter().position(|rd| rd.iter().any(|i| oh.contains(i))) {
                let mut moved: Vec<usize> = vec![];
                for (ri, rd) in rounds.iter_mut().enumerate() {
                    if ri == first {
                        continue;
                    }
                    rd.retain(|i| {
                        if oh.contains(i) {
                            moved.push(*i);
                            false
                        } else {
                            true
                        }
                    });
                }
                moved.sort_unstable();
                for i in moved {
                    if !rounds[first].contains(&i) {
                        rounds[first].push(i);
                    }
                }
                rounds[first].sort_unstable();
            }
        }
        // a convex corner of two planes and a cylinder with all three edges selected: the tools end on the planes through the
        // ball's centre (they would otherwise cross each other in a curve the booleans cannot follow) and the vertex blend
        // (`corner::cyl_corner_tool`) joins them after the rounds
        let _trims = TrimGuard;
        if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && EDGE_RADII.with(|c| c.borrow().is_none()) {
            for corner in super::corner::find_corner(b, &eids).into_iter().filter(|c| c.curved) {
                // (a corner the patch cannot be built at is refused with its own reason, see `curved_corner_refusal`)
                let g = super::corner::cyl_corner_geom(b, &corner, r)?;
                CYL_CORNER.with(|c| c.set(true));
                let vp = b.vertex(corner.vertex).p;
                CORNER_TRIMS.with(|t| {
                    for i in 0..3 {
                        t.borrow_mut().push(CornerTrim { p: vp, arc: i == 0, dir: g.u[i], amount: g.along[i], centre: None });
                    }
                });
            }
        }
        // a convex corner of three planes at other than right angles (the vertical edge of a trapezoid prism between two
        // slanted walls and its two top edges): the same trims, see `corner::oblique_corner_centre`
        if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && EDGE_RADII.with(|c| c.borrow().is_none()) {
            for corner in super::corner::find_corner(b, &eids) {
                if let Some((c, u)) = super::corner::oblique_corner_centre(b, &corner, r) {
                    let vp = b.vertex(corner.vertex).p;
                    CORNER_TRIMS.with(|t| {
                        for i in 0..3 {
                            t.borrow_mut().push(CornerTrim { p: vp, arc: false, dir: u[i], amount: 0.0, centre: Some(c) });
                        }
                    });
                }
            }
        }
        // junction vertices of the whole selection (an earlier round shortens the edges that follow)
        let mut junc = if matches!(spec, Spec::Fillet(..)) {
            let mut j = junction_vertices(b, &eids, tol);
            j.flush.extend(open_tangent_ends(b, &eids));
            j
        } else {
            // chamfer: the ends of a tangent chain stop flush, and so do the ends that run on tangentially into an edge
            // that is not selected (a tool that runs on there cuts into the continuing edge's material)
            let mut flush = if TANGENT_FLUSH.with(|c| c.get()) { tangent_flush_vertices(b, &eids) } else { vec![] };
            for p in open_tangent_ends(b, &eids) {
                if !flush.iter().any(|q| q.dist(p) < 1e-9) {
                    flush.push(p);
                }
            }
            EndConditions { ext: vec![], flush, mitre: vec![], patch: vec![], shared: super::ends::shared_ends(b, &eids) }
        };
        junc.mitre = mitre_pairs(b, &eids, matches!(spec, Spec::Chamfer(d, ChamferLeg::Dist(d2)) if (d - d2).abs() < 1e-12));
        junc.patch.extend(overhangs.iter().map(|c| c.v));
        // reflex rim corners of a curved outline: both rim tools stop flush there, a wedge cut joins them afterwards
        let reflex = if matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) && EDGE_RADII.with(|c| c.borrow().is_none()) { super::rim_reflex::corners(b, &eids) } else { vec![] };
        junc.flush.extend(reflex.iter().map(|c| c.v));
        let mut out = b.clone();
        let mut earlier: Vec<String> = vec![];
        // chamfer: arcs of one split ring (a lofted cap rim cut in two at the
        // seam) are ONE revolved cut, named for the first arc
        let (ring_lead, ring_skip) = if chamfer_only(spec) || matches!(spec, Spec::Fillet(_, FilletProfile::Circular)) { split_rings(b, &eids) } else { (vec![false; n], vec![false; n]) };
        for round in rounds {
            // (an edge an earlier round's tool already blended is gone: the other arc of a concave arc split by a seam,
            // which that tool swept over, see analytic.rs)
            let round: Vec<usize> = round
                .into_iter()
                .filter(|&i| !ring_skip[i])
                .filter(|&i| {
                    let (fa, fb, at) = &fwd[i];
                    find_edge(&out, fa, fb, *at).is_some() || !(blended_by_earlier_tool(&out, fa, fb, feature) || swept_pick(*at))
                })
                .collect();
            if round.is_empty() {
                continue;
            }
            // a pick of this round that an earlier round's tool took away (the selection had it: `eids`): an overflow
            if let Some(&i) = round.iter().find(|&&i| find_edge(&out, &fwd[i].0, &fwd[i].1, fwd[i].2).is_none()) {
                return Err(super::diag::consumed_by_earlier(feature, i as u32, &fwd[i].0, &fwd[i].1));
            }
            let rings: Vec<bool> = round.iter().map(|&i| ring_lead[i]).collect();
            let idx: Vec<u32> = round.iter().map(|&i| i as u32).collect();
            let sel: Vec<(String, String, Vec3)> = round.iter().map(|&i| fwd[i].clone()).collect();
            let (raw, round_refs, touched) = blend_edges_at(&out, &sel, spec, feature, tol, &idx, &earlier, &rings, &junc)?;
            if super::diag::on() {
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
        for c in &reflex {
            // the wedge must join as a valid solid; where its boolean does not close (a corner zone a few hundredths of a
            // millimetre long at a small radius) the two rounds keep their flush ends, as long as the step between them
            // is small (the zone reaches less than r / 2 past the corner); else the corner is refused
            let joined = super::rim_reflex::cut_wedge(&out, c, r, feature, c.line as u32, c.arc as u32, tol).and_then(|w| super::validity::check_solid(&out, &w, tol).map(|_| w));
            match joined {
                Ok(w) => out = w,
                Err(e) if c.zone_reach(r) <= 0.5 * r => {
                    dbg_fail(&format!("reflex rim corner at ({:.3},{:.3},{:.3}): the wedge did not join ({} {:?}); the rounds end flush", c.v.x, c.v.y, c.v.z, e.code, e.detail));
                }
                Err(e) => {
                    return Err(BlendError {
                        code: "blendCorner".into(),
                        detail: Some(format!(
                            "the rounds of a straight edge and an arc that meet at ({:.3}, {:.3}, {:.3}), where the wall edge between them is left sharp, could not be joined ({})",
                            c.v.x, c.v.y, c.v.z,
                            e.detail.unwrap_or(e.code)
                        )),
                        params: vec![("hint".into(), "use a different radius, round the sketch corner before the cut (or round the wall edge too), or round the two edges in separate features".into())],
                    });
                }
            }
        }
        // overhang corners: replace the zone box about the vertex with the corner patch solid
        // (a splice that fails is refused: the striped body would end each stripe in a planar step)
        for (i, oh) in overhangs.iter().enumerate() {
            match super::overhang::splice(&out, oh, feature, i, tol) {
                Ok(spliced) => out = spliced,
                Err(e) => {
                    dbg_fail(&format!("overhang patch at ({:.3},{:.3},{:.3}): {} {:?}", oh.v.x, oh.v.y, oh.v.z, e.code, e.detail));
                    let why = e.detail.as_deref().map(|d| d.trim_start_matches("overhang corner: ")).unwrap_or(&e.code).to_string();
                    return Err(oh.refusal(Some(&why)));
                }
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
                let tool = match super::corner::sphere_corner_tool(b, &corner, r, feature, &name, tol) {
                    Ok(t) => t,
                    Err(e) => {
                        dbg_fail(&format!("corner tool at {:?}: {} {:?}", b.vertex(corner.vertex).p, e.code, e.detail));
                        continue;
                    }
                };
                let op = if corner.concave { Op::Union } else { Op::Cut };
                let mut cut = match apply_tool(&out, &tool, op, &name, vec![], tol) {
                    Ok((c, _)) => c,
                    Err(e) => {
                        dbg_fail(&format!("corner cut at {:?}: {} {:?}", b.vertex(corner.vertex).p, e.code, e.detail));
                        continue;
                    }
                };
                if !corner.curved {
                    super::corner::fix_sphere_pcurves(&mut cut, &name.id());
                }
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
    let mut junc = EndConditions::default();
    {
        let eids: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
        junc.flush = open_tangent_ends(b, &eids);
    }
    let (raw, refs, _) = blend_edges_at(b, edges, spec, feature, tol, &idx, &[], &[], &junc)?;
    Ok(in_place_layout(b, &raw, &refs, tol))
}

/// The edge of a corner whose two planar faces meet at the smallest turn, when it is the only one (`None` for a corner
/// whose edges all have the same dihedral, such as a box corner).
fn flattest_edge(b: &Body, c: &super::corner::Corner) -> Option<EdgeId> {
    let dots: Vec<f64> = c.edges.iter().map(|&e| {
        let fs = b.edge_faces(e);
        if fs.len() == 2 { b.planar_normal(fs[0]).normalize().dot(b.planar_normal(fs[1]).normalize()) } else { f64::NAN }
    }).collect();
    if dots.iter().any(|d| !d.is_finite()) {
        return None;
    }
    let i = (0..3).max_by(|&i, &j| dots[i].total_cmp(&dots[j]))?;
    (0..3).all(|j| j == i || dots[i] > dots[j] + 1e-6).then_some(c.edges[i])
}

thread_local! {
    /// Radius of the fillet of selection index `k` (see [`fillet_unequal`]).
    pub(super) static EDGE_RADII: std::cell::RefCell<Option<Vec<f64>>> = const { std::cell::RefCell::new(None) };
}

thread_local! {
    /// The face a chamfer's first leg lies on (and whether to take the other one instead), see
    /// [`chamfer_legs_on_face`].
    pub(super) static CHAMFER_FIRST: std::cell::RefCell<Option<(FaceName, bool)>> = const { std::cell::RefCell::new(None) };
}

thread_local! {
    /// While set, the ends of selected edges that continue each other tangentially stop flush (see
    /// [`junction_vertices`]); used by [`super::mixed`] for the chain rim - arc - rim.
    pub(super) static TANGENT_FLUSH: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// Chamfer round order of a retry: 0 = the selection's order, 1 = concave edges first, 2 = convex edges first (a chamfer
    /// built edge by edge cuts a body the earlier ones changed, and where three edges of mixed convexity meet at a vertex
    /// only some orders close).
    pub(super) static CHAMFER_ORDER: std::cell::Cell<u8> = const { std::cell::Cell::new(0) };
}

/// One blend op over `edges`; `idx[k]` is the blend number (`B<idx>`) of edge k.
pub(super) fn blend_edges_at(
    b: &Body,
    edges: &[(String, String, Vec3)],
    spec: Spec,
    feature: &str,
    tol: f64,
    idx: &[u32],
    skip_end: &[String],
    rings: &[bool],
    junc: &EndConditions,
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
    // At a convex corner of three selected edges whose dihedrals differ (the vertical edge of an octagonal boss between
    // two rim edges), the flattest edge's tool is built first: built after a rim tool has cut into the corner, it ends
    // short of the ball centre and leaves a sliver of the mitre the corner's sphere patch does not remove.
    let mut seq: Vec<usize> = (0..edges.len()).collect();
    if matches!(spec, Spec::Fillet(..)) {
        let first: Vec<EdgeId> = super::corner::find_corner(b, &sel_edges).iter().filter(|c| !c.concave).filter_map(|c| flattest_edge(b, c)).collect();
        seq.sort_by_key(|&k| !first.contains(&sel_edges[k]));
        // a plane-plane-cylinder corner (CORNER_TRIMS): the straight tools first, the rim arcs last, whatever order the edges
        // were picked in. Each tool is the same solid either way (they end on the planes through the ball's centre), but the
        // booleans of the rim tool first meet the end of the wall-top edge at the far wall corner in a coincidence that only
        // the other order survives when the split plane is a hair off the centre
        if !CORNER_TRIMS.with(|t| t.borrow().is_empty()) {
            seq.sort_by_key(|&k| (matches!(b.edge(sel_edges[k]).curve, ck_geom::curve::Curve::Circle { .. }), sel_edges[k].idx()));
        }
    }
    for k in seq {
        let (fa, fb, at) = &edges[k];
        let Some(e) = find_edge(&out, fa, fb, *at) else {
            // an earlier tool of this feature already blended it: collinear edges
            // separated by a notch (a hole centred on the edge line) are cut by one
            // tool, whose margin runs on over the notch; the edge is gone but both of
            // its faces now border that feature's blend face
            if blended_by_earlier_tool(&out, fa, fb, feature) || swept_pick(*at) {
                // only a pick whose face pair another pick shares is one tool's run on (the notch's two halves). A pick on
                // another pair of faces that an earlier round's tool swallowed whole (a round wider than the wall it
                // runs over, the concave edge beside it) was not blended: building the part without it would drop it
                // silently, so it is refused as overlapping
                // convex or concave in the input: a swallowed pick of the same sense as another pick is one tool's run on
                // (a split edge), one of the opposite sense (a concave round beside a convex one) is not
                let convex_in = |ga: &str, gb: &str, p: Vec3| -> Option<bool> {
                    let e = find_edge(b, ga, gb, p)?;
                    let fs = b.edge_faces(e);
                    if fs.len() != 2 {
                        return None;
                    }
                    let ed = b.edge(e);
                    let d = b.vertex(ed.v1).p - b.vertex(ed.v0).p;
                    if !(d.len() > 1e-9) {
                        return None;
                    }
                    let n = |f: FaceId| match &b.face(f).surface {
                        ck_geom::surface::Surface::Plane { .. } => Some(b.planar_normal(f).normalize()),
                        _ => None,
                    };
                    let (n1, n2) = (n(fs[0])?, n(fs[1])?);
                    let co = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == fs[0])?;
                    let t = if b.coedge(co).rev { -d } else { d }.normalize();
                    Some(n1.cross(t).dot(n2) < 0.0)
                };
                let mine = convex_in(fa, fb, *at);
                let opposite = mine.is_some() && edges.iter().enumerate().any(|(j, (ga, gb, p))| j != k && convex_in(ga, gb, *p).is_some_and(|c| Some(!c) == mine));
                if opposite && !swept_pick(*at) {
                    dbg_fail(&format!("edge {k} ({fa} | {fb}) was taken by the blend of another edge"));
                    return Err(BlendError::detail("blendOverflow", &format!("{feature}.B{}: blends overlap: the blend of another edge covers the edge between {fa} and {fb}", idx[k])));
                }
                dbg_fail(&format!("edge {k} ({fa} | {fb}) was blended by an earlier tool"));
                continue;
            }
            // the edge was found on this round's input (`refs` above): an earlier tool of the round took it away
            return Err(super::diag::consumed_by_earlier(feature, idx[k], fa, fb));
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
        if super::diag::on() {
            eprintln!("tool B{} ({fa} | {fb}) -> faces {:?}", idx[k], body.face_ids().map(|f| format!("{}:{}", body.face_id_str(f), body.face(f).surface.kind())).collect::<Vec<_>>());
        }
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

/// A vertex where a selected straight convex edge and a selected straight concave edge meet (the picks of a corner that is
/// not fully selected: the third edge of a 3-valent vertex, or the others of a 4-valent one, are left sharp).
fn has_mixed_pair(b: &Body, eids: &[EdgeId], concave: &[bool]) -> bool {
    let lines: Vec<bool> = eids.iter().map(|&e| super::mixed::line_edge_concave(b, e).is_some()).collect();
    for (i, &a) in eids.iter().enumerate() {
        for (j, &c) in eids.iter().enumerate().skip(i + 1) {
            if !(lines[i] && lines[j]) || concave[i] == concave[j] {
                continue;
            }
            let (ea, ec) = (b.edge(a), b.edge(c));
            if [ea.v0, ea.v1].iter().any(|v| *v == ec.v0 || *v == ec.v1) {
                return true;
            }
        }
    }
    false
}
