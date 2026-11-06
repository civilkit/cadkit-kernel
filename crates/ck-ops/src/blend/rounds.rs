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
    if edges.len() > 1 && !TANGENT_FLUSH.with(|c| c.get()) && has_tangent_mates(b, edges) {
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
    // a selection with wall lines the analytic builder refused: the free-form strips, as before that builder took them
    if first.is_err() && edges.iter().any(|(fa, fb, at)| find_edge(b, fa, fb, *at).is_some_and(|e| super::freeform::needs_freeform(b, e) && super::torus_corner::analytic_wall_edge(b, e))) {
        if let Ok(x) = super::torus_corner::free_walls(attempt) {
            return Ok(x);
        }
    }
    // when every attempt failed and the gate refused one of them, that refusal says more than a later attempt's failure
    match (first, rejected.into_inner()) {
        (Err(_), Some(gate)) => Err(gate),
        (r, _) => r,
    }
}

/// Two of the selected edges continue each other tangentially at a shared vertex.
pub(super) fn has_tangent_mates(b: &Body, edges: &[(String, String, Vec3)]) -> bool {
    let eids: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    eids.len() > 1 && !tangent_flush_vertices(b, &eids).is_empty()
}

pub(super) fn blend_edges_inner(b: &Body, edges: &[(String, String, Vec3)], spec: Spec, feature: &str, tol: f64) -> Result<Body, BlendError> {
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
        let overhangs_all: Vec<super::overhang::Corner> = if matches!(spec, Spec::Fillet(..)) { super::overhang::detect(b, &eids, &eir) } else { vec![] };
        if let Some(c) = overhangs_all.iter().find(|c| !c.patch_ok) {
            let deg = c.a12.abs().clamp(-1.0, 1.0).acos().to_degrees();
            return Err(BlendError {
                detail: Some(format!(
                    "a convex and a concave blended edge meet at ({:.3}, {:.3}, {:.3}) and their two edge directions are {:.0} degrees apart, not perpendicular",
                    c.v.x, c.v.y, c.v.z, deg
                )),
                params: vec![(
                    "hint".into(),
                    "the corner patch is only built where the two edge directions are perpendicular: fillet the concave edges and the convex edges in separate features, or leave the four edges at this vertex out of the selection".into(),
                )],
                ..BlendError::code("blendCorner")
            });
        }
        let overhangs: Vec<super::overhang::Corner> = overhangs_all;
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
        let mut rounds = rounds;
        if !overhangs.is_empty() {
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
            EndConditions { ext: vec![], flush, mitre: vec![], patch: vec![] }
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
            let round: Vec<usize> = round.into_iter().filter(|&i| !ring_skip[i]).collect();
            if round.is_empty() {
                continue;
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
            out = super::rim_reflex::cut_wedge(&out, c, r, feature, c.line as u32, c.arc as u32, tol)?;
        }
        // overhang corners: replace the zone box about the vertex with the corner patch solid
        // (an Err keeps the striped body: a valid, nearly right solid, never a refusal)
        for (i, oh) in overhangs.iter().enumerate() {
            match super::overhang::splice(&out, oh, feature, i, tol) {
                Ok(spliced) => out = spliced,
                Err(e) => dbg_fail(&format!("overhang patch at ({:.3},{:.3},{:.3}): {} {:?}", oh.v.x, oh.v.y, oh.v.z, e.code, e.detail.unwrap_or_default())),
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
    let i = (0..3).max_by(|&i, &j| dots[i].partial_cmp(&dots[j]).unwrap())?;
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
    }
    for k in seq {
        let (fa, fb, at) = &edges[k];
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
