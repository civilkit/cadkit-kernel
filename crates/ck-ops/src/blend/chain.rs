//! Chain positions of a selection and ring handling (circles split by a seam, arcs of one circle).

use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_topo::body::{Body, EdgeId, FaceId};
use super::{analytic::*, find::*, model::*};

/// Chain (component id, position) of each selected edge: components follow
/// v1 -> v0 links from the first selected edge of the component, forwards
/// (+1) then backwards (-1); a closed ring stops where it returns.
pub(super) fn chain_positions(ends: &[[usize; 2]]) -> Vec<ChainPos> {
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

/// Split rings in a chamfer selection: partial arcs (circle edges, or spline arcs, e.g. the
/// halves of a lofted cap rim) on one circle sharing a face
/// whose spans add up to a full turn. Returns `(leader, skipped)` flags per
/// selected edge: the first arc cuts the whole ring, the rest are absorbed.
pub(super) fn split_rings(b: &Body, eids: &[EdgeId]) -> (Vec<bool>, Vec<bool>) {
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
pub(super) fn ring_siblings(b: &Body, e: EdgeId) -> Vec<EdgeId> {
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

/// The arcs that continue edge `e` along the same circle between the same two faces: one smooth edge that a seam of
/// the cylinder (or cone) it lies on split in several B-rep edges, though together they do not close the circle.
pub(super) fn arc_chain_siblings(b: &Body, e: EdgeId) -> Vec<EdgeId> {
    let circle_of = |x: EdgeId| -> Option<(Vec3, Vec3, f64)> {
        let ed = b.edge(x);
        match &ed.curve {
            Curve::Line { .. } => None,
            Curve::Circle { frame, r } => Some((frame.o, frame.z.normalize(), *r)),
            other => fit_circle(other, ed.t0.min(ed.t1), ed.t0.max(ed.t1)).map(|(c, a, r, _)| (c, a, r)),
        }
    };
    let Some((c, a, r)) = circle_of(e) else { return vec![] };
    let mut fs = b.edge_faces(e);
    fs.sort_by_key(|f| f.idx());
    let mut chain = vec![e];
    loop {
        let before = chain.len();
        for x in b.edge_ids().filter(|x| !chain.contains(x)).collect::<Vec<_>>() {
            let Some((c2, a2, r2)) = circle_of(x) else { continue };
            let mut f2 = b.edge_faces(x);
            f2.sort_by_key(|f| f.idx());
            let same = f2 == fs && (c2 - c).len() < 1e-7 * (1.0 + r) && a2.cross(a).len() < 1e-7 && (r2 - r).abs() < 1e-7 * (1.0 + r);
            let (xv0, xv1) = (b.edge(x).v0, b.edge(x).v1);
            if same && xv0 != xv1 && chain.iter().any(|&m| [b.edge(m).v0, b.edge(m).v1].iter().any(|v| *v == xv0 || *v == xv1)) {
                chain.push(x);
            }
        }
        if chain.len() == before {
            break;
        }
    }
    chain.into_iter().skip(1).collect()
}

/// `edges` plus, for every selected arc of a split ring, the rest of its arcs (picked at their midpoints).
pub(super) fn with_ring_siblings(b: &Body, edges: &[(String, String, Vec3)]) -> Vec<(String, String, Vec3)> {
    let mut out = edges.to_vec();
    let mut have: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    for e in have.clone() {
        let mut sibs = ring_siblings(b, e);
        for x in arc_chain_siblings(b, e) {
            if !sibs.contains(&x) {
                sibs.push(x);
            }
        }
        for s in sibs {
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


/// Picks that are the later pieces of a straight run the sketch split in collinear pieces (a mirrored sketch, a split
/// line, a vertex on the line): two straight edges that meet at a vertex, on one line, with their faces in the same two
/// planes. The first pick of a run is its leader: its tool runs over the whole run (the end conditions let the tool of
/// a piece go on past the vertex it shares with a collinear pick), so the other pieces get no tool of their own; the
/// caller checks afterwards that a blend face really runs along each of them.
pub(super) fn collinear_run_followers(b: &Body, eids: &[EdgeId]) -> Vec<bool> {
    use ck_geom::surface::Surface;
    let dir = |e: EdgeId| -> Option<Vec3> {
        let ed = b.edge(e);
        if !matches!(ed.curve, Curve::Line { .. }) || ed.v0 == ed.v1 {
            return None;
        }
        let d = b.vertex(ed.v1).p - b.vertex(ed.v0).p;
        (d.len() > 1e-12).then(|| d.normalize())
    };
    let same_plane = |f: FaceId, g: FaceId| {
        f == g
            || (matches!(b.face(f).surface, Surface::Plane { .. })
                && matches!(b.face(g).surface, Surface::Plane { .. })
                && b.planar_normal(f).normalize().dot(b.planar_normal(g).normalize()) > 1.0 - 1e-9)
    };
    let next_to = |e: EdgeId, x: EdgeId| -> bool {
        let (ea, ex) = (b.edge(e), b.edge(x));
        if e == x || ![ea.v0, ea.v1].iter().any(|v| *v == ex.v0 || *v == ex.v1) {
            return false;
        }
        let (Some(d), Some(dx)) = (dir(e), dir(x)) else { return false };
        let (fe, fx) = (b.edge_faces(e), b.edge_faces(x));
        fe.len() == 2 && fx.len() == 2 && d.cross(dx).len() < 1e-9 && ((same_plane(fe[0], fx[0]) && same_plane(fe[1], fx[1])) || (same_plane(fe[0], fx[1]) && same_plane(fe[1], fx[0])))
    };
    // a pick follows when an earlier pick of its run exists (the run is walked through its neighbours, picks or not
    // being irrelevant: only picks are in `eids`, so adjacency is between picks)
    let n = eids.len();
    let mut follower = vec![false; n];
    for i in 0..n {
        follower[i] = (0..i).any(|j| eids[j] != eids[i] && next_to(eids[j], eids[i]));
    }
    follower
}
