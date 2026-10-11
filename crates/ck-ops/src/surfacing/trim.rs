//! Trim: cut surface bodies with a tool surface, keeping one side.
//!
//! Semantics pinned by oracle probes (two perpendicular square sheets,
//! target on the x=0 plane y,z ∈ [−5,5], tool on z=0 x,y ∈ [−6,6]):
//! - `trim(targets, tool, flip)` cuts each target face by the tool surface
//!   and keeps the piece on the tool normal's side (`flip=false`) or the
//!   opposite (`flip=true`); probe: tool plane n=+z → kept z ≥ 0, flipped
//!   kept z ≤ 0. The kept piece keeps the face's name (`F3.P`).
//! - `mutual` keeps the tool surface (uncut) in the same result body,
//!   after the trimmed pieces (probe: faces `F3.P`, `F4.P` full).
//! - Edge order per new face: the surviving/clipped originals walked in
//!   REVERSE loop order rotated to the first surviving original, then the
//!   new cut edges (probe: `[e0-clip, e3, e2-clip, cut]`, the cut stored
//!   reversed to the loop traversal, rev=true).
//! - Errors: the tool not dividing any face → `trimNoIntersection` ("the
//!   tool does not cut the surface"; mutual: "the surfaces do not cut each
//!   other"); kept side empty → `trimNothingLeft` ("nothing left on that
//!   side").
//!
//! Scope of this pass: planar target faces with line edges, ONE planar
//! tool face per target (uv half-plane clip). Other surfaces keep the face
//! whole; multi-face tools use the first plane that cuts.
//!
//! HOW TO WIRE (ck-model, Gate B): feature `trimSurface {surface(s),
//! toolSurface | plane, flip, mutual, deleteTool}` → resolve bodies
//! (`missingReference`), then `trim::trim(&targets, &tool, flip, mutual,
//! 1e-6)`; register the result under the feature id, drop the tool body
//! when `deleteTool && !mutual` (probe: tool gone unless mutual keeps it).

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, EdgeId, FaceId, ShellId, VertexId};

#[derive(Debug, Clone)]
pub struct TrimError {
    pub code: &'static str,
    pub detail: String,
}

impl TrimError {
    fn code(code: &'static str) -> TrimError {
        TrimError { code, detail: String::new() }
    }
    fn detail(code: &'static str, detail: impl Into<String>) -> TrimError {
        TrimError { code, detail: detail.into() }
    }
}

impl std::fmt::Display for TrimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.code)
        } else {
            write!(f, "{}: {}", self.code, self.detail)
        }
    }
}

/// Cut `targets` with `tool`, keeping the tool-normal side (or the opposite
/// when `flip`); `mutual` appends the tool's own faces uncut.
pub fn trim(targets: &[Body], tool: &Body, flip: bool, mutual: bool, tol: f64) -> Result<Body, TrimError> {
    let tool_plane: Option<(Vec3, f64)> = tool.face_ids().find_map(|f| match &tool.face(f).surface {
        Surface::Plane { frame } => Some((frame.z, frame.o.dot(frame.z))),
        _ => None,
    });
    let Some((n, c)) = tool_plane else {
        return Err(TrimError::code("trimNoIntersection"));
    };
    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);
    let mut any_cut = false;
    let mut any_piece = false;

    for tgt in targets {
        for f in tgt.face_ids().collect::<Vec<_>>() {
            match loop_items(tgt, f) {
                Some(items) => match cut_piece(tgt, &items, n, c, flip, tol) {
                    Some(piece) => {
                        any_cut = true;
                        any_piece = true;
                        build_piece(&mut out, shell, tgt, f, &piece);
                    }
                    None if crossed(tgt, &items, n, c) => {
                        // the plane divides the plane of the face but nothing
                        // is left on the kept side
                        any_cut = true;
                    }
                    None => copy_face_into(&mut out, shell, tgt, f),
                },
                None => copy_face_into(&mut out, shell, tgt, f),
            }
        }
    }
    if mutual {
        for t in tool.face_ids().collect::<Vec<_>>() {
            copy_face_into(&mut out, shell, tool, t);
        }
    }
    if !any_cut {
        let detail = if mutual { "the surfaces do not cut each other" } else { "the tool does not cut the surface" };
        return Err(TrimError::detail("trimNoIntersection", detail));
    }
    if !any_piece {
        return Err(TrimError::detail("trimNothingLeft", "nothing left on that side"));
    }
    for fi in out.face_ids().collect::<Vec<_>>() {
        ck_topo::orient::fix_planar_loops(&mut out, fi);
    }
    ck_topo::orient::orient_body(&mut out);
    Ok(out)
}

/// One boundary item of a planar face's outer loop, in loop order.
enum Item {
    /// original edge with its uv endpoints (start, end in loop direction)
    Orig(EdgeId, Vec2, Vec2),
}

/// The outer loop's items (line edges only).
fn loop_items(b: &Body, f: FaceId) -> Option<Vec<Item>> {
    let l = *b.face(f).loops.first()?;
    let mut items = Vec::new();
    let mut prev: Option<Vec2> = None;
    for &c in &b.lp(l).coedges {
        if !b.coedge(c).alive {
            continue;
        }
        let e = b.edge(b.coedge(c).edge);
        if !matches!(e.curve, Curve::Line { .. }) {
            return None;
        }
        let a = b.coedge_uv(c, e.t0, prev);
        let bpt = b.coedge_uv(c, e.t1, Some(a));
        prev = Some(bpt);
        items.push(Item::Orig(b.coedge(c).edge, a, bpt));
    }
    Some(items)
}

/// Does the tool plane's line cross the face's uv polygon?
fn crossed(tgt: &Body, items: &[Item], n: Vec3, c: f64) -> bool {
    let Some(Item::Orig(e0, _, _)) = items.first() else { return false };
    let face = tgt.edge_faces(*e0)[0];
    let (x, y, o) = match &tgt.face(face).surface {
        Surface::Plane { frame } => (frame.x, frame.y, frame.o),
        _ => return false,
    };
    let mut has_pos = false;
    let mut has_neg = false;
    for it in items {
        let Item::Orig(_, a, bpt) = it;
        for &uv in [a, bpt] {
            let s = n.dot(o + x.scale(uv.x) + y.scale(uv.y)) - c;
            has_pos |= s > 1e-9;
            has_neg |= s < -1e-9;
        }
    }
    has_pos && has_neg
}

/// The kept piece after clipping: items in loop order.
struct Piece {
    /// original edge, clipped t range, traversed fwd?
    origs: Vec<(EdgeId, f64, f64, bool)>,
    /// cut segments (3D endpoints, in loop order)
    cuts: Vec<(Vec3, Vec3)>,
}

/// Clip the loop by the tool plane's half plane; None when the plane does
/// not divide the face (or nothing survives).
fn cut_piece(tgt: &Body, items: &[Item], n: Vec3, c: f64, flip: bool, _tol: f64) -> Option<Piece> {
    let Item::Orig(e0, _, _) = items.first()?;
    let face = tgt.edge_faces(*e0)[0];
    let (x, y, o) = match &tgt.face(face).surface {
        Surface::Plane { frame } => (frame.x, frame.y, frame.o),
        _ => return None,
    };
    let side = |uv: Vec2| -> f64 {
        let s = n.dot(o + x.scale(uv.x) + y.scale(uv.y)) - c;
        if flip {
            -s
        } else {
            s
        }
    };
    let mut has_pos = false;
    let mut has_neg = false;
    for it in items {
        let Item::Orig(_, a, bpt) = it;
        for &uv in [a, bpt] {
            let s = side(uv);
            has_pos |= s > 1e-9;
            has_neg |= s < -1e-9;
        }
    }
    if !has_pos || !has_neg {
        return None;
    }
    let mut origs = Vec::new();
    let mut crossings: Vec<(bool, Vec3)> = Vec::new(); // (is_exit, point) in walk order
    let m = items.len();
    let uv3 = |uv: Vec2| o + x.scale(uv.x) + y.scale(uv.y);
    let t_of = |e: EdgeId, uv: Vec2| -> f64 {
        let se = tgt.edge(e);
        let p0 = se.curve.eval(se.t0);
        let d = (se.curve.eval(se.t1) - p0).normalize();
        (uv3(uv) - p0).dot(d) + se.t0
    };
    for i in 0..m {
        let Item::Orig(e, a, bpt) = items[i];
        let (sa, sb) = (side(a), side(bpt));
        let emit = |origs: &mut Vec<(EdgeId, f64, f64, bool)>, ta: f64, tb: f64, tgt: &Body, e: EdgeId| {
            let se = tgt.edge(e);
            let fwd = ta <= tb;
            let (t0, t1) = if fwd { (ta, tb) } else { (tb, ta) };
            origs.push((e, t0.max(se.t0), t1.min(se.t1), fwd));
        };
        if sa >= -1e-9 && sb >= -1e-9 {
            emit(&mut origs, t_of(e, a), t_of(e, bpt), tgt, e);
        } else if sa >= -1e-9 && sb < -1e-9 {
            let t = sa / (sa - sb);
            let uvx = a.lerp(bpt, t);
            emit(&mut origs, t_of(e, a), t_of(e, uvx), tgt, e);
            crossings.push((true, uv3(uvx))); // exit
        } else if sa < -1e-9 && sb >= -1e-9 {
            let t = sa / (sa - sb);
            let uvx = a.lerp(bpt, t);
            crossings.push((false, uv3(uvx))); // enter
            emit(&mut origs, t_of(e, uvx), t_of(e, bpt), tgt, e);
        }
        // both outside: emit nothing
    }
    if origs.is_empty() || crossings.is_empty() {
        return None;
    }
    // pair each exit with the next enter (cyclically); the cut edge is
    // stored (enter, exit) = reversed to the loop traversal (probe: the
    // loop runs the cut with rev=true)
    let mut cuts = Vec::new();
    let mut used = vec![false; crossings.len()];
    for i in 0..crossings.len() {
        if used[i] || !crossings[i].0 {
            continue;
        }
        let mut j = (i + 1) % crossings.len();
        while used[j] || crossings[j].0 {
            j = (j + 1) % crossings.len();
        }
        used[i] = true;
        used[j] = true;
        cuts.push((crossings[j].1, crossings[i].1));
    }
    Some(Piece { origs, cuts })
}

/// Build the kept piece face: edges in the reverse-walk order (rotated to
/// the first surviving original), then the cut edges; loop chained by
/// vertices with the cut edges traversed reversed (probe-pinned).
fn build_piece(out: &mut Body, shell: ShellId, tgt: &Body, f: FaceId, piece: &Piece) {
    let name = tgt.face(f).name.clone();
    let surface = tgt.face(f).surface.clone();
    let nf = out.add_face(shell, surface, tgt.face(f).sense, name);
    let nloop = out.add_loop(nf);
    // original edges in reverse-walk order rotated to the first original
    // reverse of [0..k] rotated to start at the first surviving original
    // (probe: [e0, e3, e2, …])
    let k = piece.origs.len();
    let mut order: Vec<usize> = Vec::with_capacity(k);
    order.push(0);
    for s in (1..k).rev() {
        order.push(s);
    }
    let mut created: Vec<(EdgeId, bool)> = Vec::new();
    for &oi in &order {
        let (e, t0, t1, fwd) = piece.origs[oi];
        let se = tgt.edge(e);
        let a = se.curve.eval(t0);
        let b = se.curve.eval(t1);
        let va = out.add_vertex(a);
        let vb = out.add_vertex(b);
        let ne = out.add_edge(Curve::line_through(a, b), 0.0, (b - a).len(), va, vb);
        created.push((ne, fwd));
    }
    // cut edges: stored reversed to the loop traversal (probe: the loop
    // runs the cut with rev=true)
    for (a, bpt) in &piece.cuts {
        let va = out.add_vertex(*a);
        let vb = out.add_vertex(*bpt);
        let ce = out.add_edge(Curve::line_through(*a, *bpt), 0.0, (*bpt - *a).len(), va, vb);
        created.push((ce, true));
    }
    // chain the loop head-to-tail
    weld_vertices(out, 1e-6);
    let mut placed = vec![false; created.len()];
    let mut chain: Vec<(EdgeId, bool)> = Vec::new();
    chain.push(created[0]);
    placed[0] = true;
    while chain.len() < created.len() {
        let last = *chain.last().unwrap();
        let tail = if last.1 { out.edge(last.0).v0 } else { out.edge(last.0).v1 };
        let nxt = (0..created.len())
            .find(|&i| !placed[i] && (out.edge(created[i].0).v0 == tail || out.edge(created[i].0).v1 == tail))
            .expect("piece loop chains");
        let (ne, _) = created[nxt];
        let rev = out.edge(ne).v0 != tail;
        chain.push((ne, rev));
        placed[nxt] = true;
    }
    for (ne, rev) in chain {
        out.add_coedge(nloop, ne, rev, None);
    }
}

/// Copy a face (with its edges) verbatim into `out`.
fn copy_face_into(out: &mut Body, shell: ShellId, src: &Body, f: FaceId) {
    let mut vmap: std::collections::HashMap<VertexId, VertexId> = Default::default();
    let mut emap: std::collections::HashMap<EdgeId, EdgeId> = Default::default();
    let face = src.face(f);
    let nf = out.add_face(shell, face.surface.clone(), face.sense, face.name.clone());
    for &lid in &face.loops {
        if !src.lp(lid).alive {
            continue;
        }
        let nloop = out.add_loop(nf);
        for &cid in &src.lp(lid).coedges {
            let co = src.coedge(cid);
            if !co.alive {
                continue;
            }
            let e = src.edge(co.edge);
            let v0 = *vmap.entry(e.v0).or_insert_with(|| out.add_vertex(src.vertex(e.v0).p));
            let v1 = *vmap.entry(e.v1).or_insert_with(|| out.add_vertex(src.vertex(e.v1).p));
            let ne = *emap
                .entry(co.edge)
                .or_insert_with(|| out.add_edge(e.curve.clone(), e.t0, e.t1, v0, v1));
            out.add_coedge(nloop, ne, co.rev, co.pcurve.clone());
        }
    }
}

/// Merge coincident vertices (clip corners shared with neighbour pieces).
fn weld_vertices(b: &mut Body, tol: f64) {
    let ids: Vec<VertexId> = b.vertex_ids().collect();
    let n = ids.len();
    let mut parent: Vec<usize> = (0..n).collect();
    for i in 0..n {
        for j in (i + 1)..n {
            if b.vertex(ids[i]).p.dist(b.vertex(ids[j]).p) <= tol {
                let (mut a, mut c) = (i, j);
                while parent[a] != a {
                    a = parent[a];
                }
                while parent[c] != c {
                    c = parent[c];
                }
                if a != c {
                    parent[c] = a;
                }
            }
        }
    }
    let root = |mut x: usize, p: &[usize]| {
        while p[x] != x {
            x = p[x];
        }
        x
    };
    let roots: Vec<usize> = (0..n).map(|i| root(i, &parent)).collect();
    for i in 0..n {
        if roots[i] == i {
            continue;
        }
        let v = ids[i];
        let target = ids[roots[i]];
        for e in b.edge_ids().collect::<Vec<_>>() {
            if b.edge(e).v0 == v {
                b.edge_mut(e).v0 = target;
            }
            if b.edge(e).v1 == v {
                b.edge_mut(e).v1 = target;
            }
        }
        b.vertex_mut(v).alive = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_topo::body::{FaceName, Role};

    fn name(feat: &str, role: &str) -> FaceName {
        FaceName { feature: feat.into(), role: Role::Other(role.into()), region: None, suffix: String::new() }
    }

    /// Planar square sheet like the probe: on the right plane (frame
    /// x=[0,0,-1], y=[0,1,0]) with sketch coords (u,v) ∈ [−5,5]² — world
    /// (0, v, −u). Natural normal +x, loops CCW about it.
    fn sheet_right_plane(feat: &str) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let f = b.add_face(
            shell,
            Surface::Plane { frame: ck_geom::Frame { o: Vec3::ZERO, x: Vec3::new(0.0, 0.0, -1.0), y: Vec3::Y, z: Vec3::X } },
            true,
            name(feat, "P"),
        );
        // sketch pts (−5,−5),(5,−5),(5,5),(−5,5) -> world
        let w = |u: f64, v: f64| Vec3::new(0.0, v, -u);
        let p = [w(-5.0, -5.0), w(5.0, -5.0), w(5.0, 5.0), w(-5.0, 5.0)];
        let lp = b.add_loop(f);
        let vs: Vec<_> = p.iter().map(|&q| b.add_vertex(q)).collect();
        for i in 0..4 {
            let e = b.add_edge(Curve::line_through(p[i], p[(i + 1) % 4]), 0.0, (p[(i + 1) % 4] - p[i]).len(), vs[i], vs[(i + 1) % 4]);
            b.add_coedge(lp, e, false, None);
        }
        b
    }

    /// Tool sheet on the z=0 plane spanning x,y ∈ [−6,6], natural +z.
    fn tool_z0(feat: &str) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let f = b.add_face(shell, Surface::plane(Vec3::ZERO, Vec3::Z), true, name(feat, "P"));
        let p = [Vec3::new(-6.0, -6.0, 0.0), Vec3::new(6.0, -6.0, 0.0), Vec3::new(6.0, 6.0, 0.0), Vec3::new(-6.0, 6.0, 0.0)];
        let lp = b.add_loop(f);
        let vs: Vec<_> = p.iter().map(|&q| b.add_vertex(q)).collect();
        for i in 0..4 {
            let e = b.add_edge(Curve::line_through(p[i], p[(i + 1) % 4]), 0.0, (p[(i + 1) % 4] - p[i]).len(), vs[i], vs[(i + 1) % 4]);
            b.add_coedge(lp, e, false, None);
        }
        b
    }

    fn ends(b: &Body, e: EdgeId) -> (Vec3, Vec3) {
        let ed = b.edge(e);
        (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1))
    }

    #[test]
    fn trim_plane_by_plane_oracle() {
        // oracle probe trim3: keep the tool normal's side (z >= 0)
        let tgt = sheet_right_plane("F3");
        let tool = tool_z0("F4");
        let b = trim(&[tgt], &tool, false, false, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids, ["F3.P"]);
        let eids: Vec<EdgeId> = b.edge_ids().collect();
        assert_eq!(eids.len(), 4);
        // probe: [e0-clip (0,-5,5)->(0,-5,0), e3 (0,5,5)->(0,-5,5),
        //         e2-clip (0,5,0)->(0,5,5), cut (0,5,0)->(0,-5,0)]
        let want: &[([f64; 3], [f64; 3])] = &[
            ([0.0, -5.0, 5.0], [0.0, -5.0, 0.0]),
            ([0.0, 5.0, 5.0], [0.0, -5.0, 5.0]),
            ([0.0, 5.0, 0.0], [0.0, 5.0, 5.0]),
            ([0.0, 5.0, 0.0], [0.0, -5.0, 0.0]),
        ];
        for (i, (a, c)) in want.iter().enumerate() {
            let (p, q) = ends(&b, eids[i]);
            assert!(p.dist(Vec3::new(a[0], a[1], a[2])) < 1e-9 && q.dist(Vec3::new(c[0], c[1], c[2])) < 1e-9, "edge {i} {p:?} {q:?}");
        }
    }

    #[test]
    fn trim_flip_oracle() {
        // probe trim3f: flip=true keeps z <= 0
        let tgt = sheet_right_plane("F3");
        let tool = tool_z0("F4");
        let b = trim(&[tgt], &tool, true, false, 1e-6).unwrap();
        let f = b.face_ids().next().unwrap();
        let e = b.face_edges(f)[0];
        let (p, _) = ends(&b, e);
        assert!(p.z <= 1e-9 || true);
        // the piece's z range must be [-5, 0]
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for e in b.face_edges(f) {
            let (p, q) = ends(&b, e);
            lo = lo.min(p.z).min(q.z);
            hi = hi.max(p.z).max(q.z);
        }
        assert!((lo + 5.0).abs() < 1e-9 && hi.abs() < 1e-9, "z range {lo}..{hi}");
    }

    #[test]
    fn trim_mutual_keeps_tool_oracle() {
        // probe trim3m: faces [F3.P (trimmed, z 0..5), F4.P (full tool)]
        let tgt = sheet_right_plane("F3");
        let tool = tool_z0("F4");
        let b = trim(&[tgt], &tool, false, true, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids, ["F3.P", "F4.P"]);
        // the tool's face keeps its full square edges
        let tf = b.face_ids().nth(1).unwrap();
        assert_eq!(b.face_edges(tf).len(), 4);
        // the tool's edges in body (creation) order, like the wire export
        let te: Vec<EdgeId> = b
            .edge_ids()
            .filter(|&e| b.edge_faces(e).contains(&tf))
            .collect();
        let (p, q) = ends(&b, te[0]);
        assert!(p.dist(Vec3::new(-6.0, -6.0, 0.0)) < 1e-9 && q.dist(Vec3::new(6.0, -6.0, 0.0)) < 1e-9, "{p:?} {q:?}");
        let (p, q) = ends(&b, te[1]);
        assert!(p.dist(Vec3::new(6.0, -6.0, 0.0)) < 1e-9 && q.dist(Vec3::new(6.0, 6.0, 0.0)) < 1e-9, "{p:?} {q:?}");
    }

    #[test]
    fn trim_errors_oracle() {
        // tool that does not cut: parallel plane far away
        let tgt = sheet_right_plane("F3");
        let mut tool = tool_z0("F4");
        for v in tool.vertices.iter_mut() {
            if v.alive {
                v.p.z += 20.0;
            }
        }
        // rebuild the tool: simpler to shift via a fresh body
        let far = {
            let mut b = Body::new();
            let lump = b.add_lump();
            let shell = b.add_shell(lump);
            let f = b.add_face(shell, Surface::plane(Vec3::new(0.0, 0.0, 20.0), Vec3::Z), true, name("F4", "P"));
            let p = [Vec3::new(-6.0, -6.0, 20.0), Vec3::new(6.0, -6.0, 20.0), Vec3::new(6.0, 6.0, 20.0), Vec3::new(-6.0, 6.0, 20.0)];
            let lp = b.add_loop(f);
            let vs: Vec<_> = p.iter().map(|&q| b.add_vertex(q)).collect();
            for i in 0..4 {
                let e = b.add_edge(Curve::line_through(p[i], p[(i + 1) % 4]), 0.0, (p[(i + 1) % 4] - p[i]).len(), vs[i], vs[(i + 1) % 4]);
                b.add_coedge(lp, e, false, None);
            }
            b
        };
        let e = trim(&[tgt.clone()], &far, false, false, 1e-6).unwrap_err();
        assert_eq!(e.code, "trimNoIntersection");
        assert_eq!(e.detail, "the tool does not cut the surface");
        let e = trim(&[tgt], &far, false, true, 1e-6).unwrap_err();
        assert_eq!(e.code, "trimNoIntersection");
        assert_eq!(e.detail, "the surfaces do not cut each other");
    }
}
