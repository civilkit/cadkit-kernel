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
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, ShellId, VertexId};

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
///
/// All-planar inputs take the pinned planar path (uv half-plane clip);
/// anything else goes through [`trim_curved`] (surface x surface SSI).
pub fn trim(targets: &[Body], tool: &Body, flip: bool, mutual: bool, tol: f64) -> Result<Body, TrimError> {
    let planar = |b: &Body| b.face_ids().all(|x| matches!(b.face(x).surface, Surface::Plane { .. }));
    if planar(tool) && targets.iter().all(planar) {
        return trim_planar(targets, tool, flip, mutual, tol);
    }
    trim_curved(targets, tool, flip, mutual, tol)
}

fn trim_planar(targets: &[Body], tool: &Body, flip: bool, mutual: bool, tol: f64) -> Result<Body, TrimError> {
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
        // pinned: the uncut tool is its own lump (`lump: 1` in the wire faces)
        let tool_lump = out.add_lump();
        let tool_shell = out.add_shell(tool_lump);
        for t in tool.face_ids().collect::<Vec<_>>() {
            copy_face_into(&mut out, tool_shell, tool, t);
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
    // volume-based outward orientation only means something for closed shells
    // (pinned: mutual trim of two sheets keeps both sheets' normals)
    let closed = out.edge_ids().all(|e| out.edge_coedges(e).iter().filter(|&&c| out.coedge(c).alive).count() == 2);
    if closed {
        ck_topo::orient::orient_body(&mut out);
    }
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

// ------------------------------------------------------------------ curved
//
// Surface x surface trim (large sample F14: loft x barrel revolve, mutual).
// The SSI curves come from `ck_ops::intersect::ss` (read-only) and are
// clipped HERE — sampled onto the face-pair region, longest-run + bisection
// — never inside ss.rs. Mutual keeps the region of each side that does NOT
// project inside the other side's faces; `flip` is ignored for mutual
// (pinned by the oracle: both non-overlapping regions survive). Non-mutual
// keeps the target piece on the (flip-adjusted) tool-normal side (GUESS for
// curved tools; planar tools keep the planar path above).
//
// Result shape: one lump/shell; cut edges are SHARED between the two sides
// (so a later knit sews along them); faces keep their original names.
//
// NOTE: `ck_ops::boolean::imprint` (and its `edit::split_face`) were checked
// as a candidate for the face-split step — they are NOT used here. imprint()
// never mutates the input Bodies: it builds a side-car `Arrangement` and
// `assemble::build` emits a *fresh* Body from fragments, which would lose the
// "shared cut edge + keep face names" contract trim needs for the later knit.
// `edit::split_face` is dead code and requires the boundary split first
// anyway. This path therefore mutates faces in place via `endpoint_vertex`
// (split_edge) + `add_edge` + `rebuild_loop`.

use std::collections::{HashMap, HashSet};
use ck_geom::curve::Curve as Curve3;
use ck_geom::Vec2 as Uv2;

/// A clipped SSI curve placed between two out-body faces.
struct Placed {
    e: EdgeId,
    v0: VertexId,
    v1: VertexId,
    fa: FaceId,
    fb: FaceId,
}

fn trim_curved(targets: &[Body], tool: &Body, flip: bool, mutual: bool, tol: f64) -> Result<Body, TrimError> {
    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);
    // copy all faces into ONE shell with per-source maps so shared seams
    // stay shared (mutual: tool faces sewn to the targets along the cuts)
    let mut tgt_faces = Vec::new();
    for tgt in targets {
        let mut vm: HashMap<VertexId, VertexId> = Default::default();
        let mut em: HashMap<EdgeId, EdgeId> = Default::default();
        for f in tgt.face_ids().collect::<Vec<_>>() {
            tgt_faces.push(copy_face_mapped(&mut out, shell, tgt, f, &mut vm, &mut em));
        }
    }
    let mut tool_faces = Vec::new();
    {
        let mut vm: HashMap<VertexId, VertexId> = Default::default();
        let mut em: HashMap<EdgeId, EdgeId> = Default::default();
        for f in tool.face_ids().collect::<Vec<_>>() {
            tool_faces.push(copy_face_mapped(&mut out, shell, tool, f, &mut vm, &mut em));
        }
    }

    // SSI per (target face, tool face) pair, clipped to the face region
    let mut cuts: Vec<(Curve3, f64, f64, Vec3, Vec3, FaceId, FaceId)> = Vec::new();
    for &fa in &tgt_faces {
        for &fb in &tool_faces {
            let (mut lo, mut hi) = face_pts_box(&out, fa);
            let (lo2, hi2) = face_pts_box(&out, fb);
            lo = lo.min(lo2);
            hi = hi.max(hi2);
            let (sa, sb) = (out.face(fa).surface.clone(), out.face(fb).surface.clone());
            let res = crate::intersect::ss::intersect(&sa, &sb, Some(crate::intersect::ss::Bounds { lo, hi }), tol);
            for c in res.curves() {
                let clipped = clip_to_face_pair(&out, fa, fb, c.curve(), tol);
                if let Some((ta, tb)) = clipped {
                    cuts.push((c.curve().clone(), ta, tb, c.curve().eval(ta), c.curve().eval(tb), fa, fb));
                }
            }
        }
    }
    if cuts.is_empty() {
        if std::env::var("TRIM_DEBUG").is_ok() {
            eprintln!("trim_curved: no clipped SSI cuts ({} face pairs)", tgt_faces.len() * tool_faces.len());
        }
        let detail = if mutual { "the surfaces do not cut each other" } else { "the tool does not cut the surface" };
        return Err(TrimError::detail("trimNoIntersection", detail));
    }
    if std::env::var("TRIM_DEBUG").is_ok() {
        eprintln!("trim_curved: {} clipped cuts", cuts.len());
        for (_, _, _, p0, p1, fa, fb) in &cuts {
            eprintln!("  cut {:?} -> {:?} ({:?} x {:?})", p0, p1, out.face(*fa).name.id(), out.face(*fb).name.id());
        }
    }

    // place the cut edges (one shared edge per curve) + split the faces'
    // boundary edges at the cut endpoints (welding across the pair)
    let mut placed: Vec<Placed> = Vec::new();
    let dbg = std::env::var("TRIM_DEBUG").is_ok();
    for (cv, ta, tb, p0, p1, fa, fb) in &cuts {
        let v0 = match endpoint_vertex(&mut out, *fa, *p0, tol) {
            Some(v) => Some(v),
            None => endpoint_vertex(&mut out, *fb, *p0, tol),
        };
        let v1 = match endpoint_vertex(&mut out, *fa, *p1, tol) {
            Some(v) => Some(v),
            None => endpoint_vertex(&mut out, *fb, *p1, tol),
        };
        if dbg {
            eprintln!(
                "  place cut {:?}->{:?} ({} x {}): v0={:?} v1={:?}",
                p0,
                p1,
                out.face(*fa).name.id(),
                out.face(*fb).name.id(),
                v0.map(|v| out.vertex(v).p),
                v1.map(|v| out.vertex(v).p)
            );
        }
        let (Some(v0), Some(v1)) = (v0, v1) else { continue };
        // tool-side boundary splits: only for true feet (the endpoint sits
        // on BOTH faces' boundaries — e.g. the arch feet on the barrel's
        // bottom spline). Seam-region endpoints are interior to the tool
        // face and must NOT split its boundary. The split vertex is merged
        // into the fa-side vertex so the cut edge endpoint lies on both
        // faces' boundaries.
        let fb_split = |out: &mut Body, v: VertexId| {
            let q = out.vertex(v).p;
            let (blo, bhi) = face_pts_box(out, *fb);
            let fbd = (bhi - blo).len().max(1.0);
            let tight = 0.01 * fbd;
            let best = nearest_on_boundary(out, *fb, q);
            let Some((d, e, t)) = best else { return };
            if d > tight {
                if dbg {
                    eprintln!("  fb-split skip ({:?}, {:?}): d={d:.4} > tight={tight:.4}", out.face(*fb).name.id(), q);
                }
                return;
            }
            let ed = out.edge(e);
            let vb = if (t - ed.t0).abs() <= 1e-12 * (1.0 + ed.t0.abs()) {
                ed.v0
            } else if (ed.t1 - t).abs() <= 1e-12 * (1.0 + ed.t1.abs()) {
                ed.v1
            } else {
                ck_topo::edit::split_edge(out, e, t).0
            };
            if vb != v {
                // merge vb into v (the cut endpoint)
                for e2 in out.edge_ids().collect::<Vec<_>>() {
                    if out.edge(e2).v0 == vb {
                        out.edge_mut(e2).v0 = v;
                    }
                    if out.edge(e2).v1 == vb {
                        out.edge_mut(e2).v1 = v;
                    }
                }
                out.vertex_mut(vb).alive = false;
            }
            if dbg {
                eprintln!("  fb-split ({:?}, {:?}): merged into {:?}", out.face(*fb).name.id(), q, out.vertex(v).p);
            }
        };
        fb_split(&mut out, v0);
        fb_split(&mut out, v1);
        let e = out.add_edge(cv.clone(), *ta, *tb, v0, v1);
        placed.push(Placed { e, v0, v1, fa: *fa, fb: *fb });
    }
    // safety net: merge any remaining near-coincident vertices (the two
    // pair-fits of one arch foot can differ by ~1e-2) so the cut chains
    // share endpoints and the loops close
    {
        let (blo, bhi) = {
            let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
            let mut hi = -lo;
            for v in out.vertex_ids() {
                lo = lo.min(out.vertex(v).p);
                hi = hi.max(out.vertex(v).p);
            }
            (lo, hi)
        };
        let weld = 1e-3 * (bhi - blo).len().max(1.0);
        weld_vertices(&mut out, weld);
    }
    // The fitted SSI curves of two pairs that share an arch foot land on the
    // same underlying edge curve at slightly different points (a 0.63 gap here
    // on e7's top spline): each pair's endpoint_vertex splits that edge, so a
    // cut-endpoint pair ends up 0.6 apart and one cut orphans its loop. Weld
    // those cut-endpoint pairs (only cut endpoints — a general weld at this
    // radius would swallow the whole loop) back onto one vertex. The partner
    // is the nearest cut endpoint within 1.0 regardless of which boundary edge
    // it snapped to: the arch-top corner is the true vertex of the shared
    // vertex-vertex edge (e4|e7) that both the e4 and e7 strips must reach.
    {
        // candidate weld targets: cut endpoints + existing boundary vertices
        let mut targets: Vec<(VertexId, Vec3)> = placed
            .iter()
            .flat_map(|p| [(p.v0, out.vertex(p.v0).p), (p.v1, out.vertex(p.v1).p)])
            .collect();
        for v in out.vertex_ids() {
            if out.vertex(v).alive {
                let p = out.vertex(v).p;
                if targets.iter().all(|&(_, q)| q.dist(p) > 1e-9) {
                    targets.push((v, p));
                }
            }
        }
        let mut done: HashSet<VertexId> = HashSet::new();
        for &(v, q) in &placed.iter().flat_map(|p| [(p.v0, out.vertex(p.v0).p), (p.v1, out.vertex(p.v1).p)]).collect::<Vec<_>>() {
            if done.contains(&v) || !out.vertex(v).alive {
                continue;
            }
            let mut best: Option<(f64, VertexId, Vec3)> = None;
            for &(w, wq) in &targets {
                if w == v || done.contains(&w) || !out.vertex(w).alive {
                    continue;
                }
                let d = wq.dist(q);
                if d <= 1.0 && best.map(|(bd, _, _)| d < bd).unwrap_or(true) {
                    best = Some((d, w, wq));
                }
            }
            if let Some((d, w, wq)) = best {
                // keep the earlier-created (lower-id) vertex so the first cut's
                // loop topology wins; repoint every edge + Placed endpoint onto it
                let (keep, drop) = if v < w { (v, w) } else { (w, v) };
                for e2 in out.edge_ids().collect::<Vec<_>>() {
                    if out.edge(e2).v0 == drop {
                        out.edge_mut(e2).v0 = keep;
                    }
                    if out.edge(e2).v1 == drop {
                        out.edge_mut(e2).v1 = keep;
                    }
                }
                out.vertex_mut(drop).alive = false;
                for p in placed.iter_mut() {
                    if p.v0 == drop {
                        p.v0 = keep;
                    }
                    if p.v1 == drop {
                        p.v1 = keep;
                    }
                }
                done.insert(v);
                done.insert(w);
                if dbg {
                    let kp = out.vertex(drop).p;
                    eprintln!(
                        "  cut-endpoint weld: ({:.3},{:.3},{:.3}) -> ({:.3},{:.3},{:.3}) d={d:.4}",
                        kp.x, kp.y, kp.z,
                        out.vertex(keep).p.x, out.vertex(keep).p.y, out.vertex(keep).p.z
                    );
                }
            }
        }
    }
    // After welding, some cut's two endpoints can collapse onto one vertex
    // (e.g. an interior seam foot where the whole cut lies on a shared corner):
    // a zero-length cut edge would be bridged into a rebuilt loop as a junk
    // degenerate coedge. Drop those Placed entries and kill their edges.
    {
        let mut dead: Vec<EdgeId> = vec![];
        placed.retain(|p| {
            if p.v0 == p.v1 {
                dead.push(p.e);
                false
            } else {
                true
            }
        });
        for e in dead {
            out.edge_mut(e).alive = false;
            if dbg {
                eprintln!("  drop degenerate zero-length cut edge {e:?}");
            }
        }
    }
    if placed.is_empty() {
        if std::env::var("TRIM_DEBUG").is_ok() {
            eprintln!("trim_curved: all {} cut endpoints failed to place", cuts.len());
        }
        return Err(TrimError::detail("trimNoIntersection", "the surfaces do not cut each other"));
    }
    if std::env::var("TRIM_DEBUG").is_ok() {
        eprintln!("trim_curved: placed {} cut edges", placed.len());
    }

    // rebuild each cut face's outer loop(s): keep the boundary runs on the
    // kept side, bridge the discarded runs with the cut edges
    let mut faces: Vec<FaceId> = tgt_faces.clone();
    faces.extend(tool_faces.iter().copied());
    faces.sort_unstable();
    faces.dedup();
    // The keep test asks "is this coedge covered by the opposing face?" — it
    // must see the opposing faces as they were BEFORE any loop is rebuilt, or
    // the answer depends on face order (a trimmed loft no longer covers the
    // barrel boundary run under its own arch).
    let snaps: HashMap<FaceId, FaceSnap> = faces
        .iter()
        .map(|&f| {
            let (lo, hi) = face_pts_box(&out, f);
            (f, FaceSnap { poly: face_poly(&out, f), lo, hi })
        })
        .collect();
    for f in faces {
        let fcuts: Vec<(EdgeId, VertexId, VertexId)> = placed
            .iter()
            .filter(|p| p.fa == f || p.fb == f)
            .map(|p| (p.e, out.edge(p.e).v0, out.edge(p.e).v1))
            .collect();
        if fcuts.is_empty() {
            continue;
        }
        let opposing: Vec<FaceId> = if tgt_faces.contains(&f) {
            tool_faces.clone()
        } else {
            tgt_faces.clone()
        };
        let mut any_kept = false;
        for l in out.face(f).loops.clone() {
            if !out.lp(l).alive {
                continue;
            }
            if rebuild_loop(&mut out, f, l, &fcuts, &opposing, &snaps, flip, mutual, tol) {
                any_kept = true;
            }
        }
        if !any_kept {
            ck_topo::edit::delete_face(&mut out, f);
        }
    }

    settle_vertices(&mut out, tol);
    snap_edge_ends(&mut out, tol);

    // non-mutual: the tool was copied only for cutting — remove it again
    if !mutual {
        for f in tool_faces {
            ck_topo::edit::delete_face(&mut out, f);
        }
    }
    ck_topo::edit::gc(&mut out);

    let any_piece = out.face_ids().next().is_some();
    if !any_piece {
        return Err(TrimError::detail("trimNothingLeft", "nothing left on that side"));
    }
    Ok(out)
}

/// Welding moves a shared vertex off the surfaces of the faces that meet at
/// it (it was snapped to one face's boundary within a loose radius). Settle
/// each such vertex onto all of its faces by alternating closest-point
/// projection (converges to the faces' common point/curve nearest the
/// vertex); `snap_edge_ends` then re-fits the edges to the settled vertices.
pub fn settle_vertices(out: &mut Body, tol: f64) {
    let eps = tol.max(1e-7);
    for v in out.vertex_ids().collect::<Vec<_>>() {
        if !out.vertex(v).alive {
            continue;
        }
        let mut faces: Vec<FaceId> = Vec::new();
        for e in out.edge_ids() {
            let ed = out.edge(e);
            if ed.alive && (ed.v0 == v || ed.v1 == v) {
                for c in out.edge_coedges(e) {
                    let f = out.coedge_face(c);
                    if out.face(f).alive && !faces.contains(&f) {
                        faces.push(f);
                    }
                }
            }
        }
        if faces.len() < 2 {
            continue;
        }
        let dev = |out: &Body, p: Vec3| faces.iter().map(|&f| out.face(f).surface.distance(p)).fold(0.0f64, f64::max);
        let p0 = out.vertex(v).p;
        let d0 = dev(out, p0);
        if d0 <= eps || d0 > 0.1 {
            continue;
        }
        let mut p = p0;
        for _ in 0..30 {
            for &f in &faces {
                p = out.face(f).surface.closest_point(p).2;
            }
            if dev(out, p) <= 1e-10 {
                break;
            }
        }
        // accept only a real improvement that stays near the original spot
        if dev(out, p) < d0 && p.dist(p0) <= 4.0 * d0 + 1e-6 {
            if std::env::var("TRIM_DEBUG").is_ok() {
                eprintln!("  settle vertex {v:?}: surface dev {d0:.3e} -> {:.3e}, moved {:.3e}", dev(out, p), p.dist(p0));
            }
            out.vertex_mut(v).p = p;
        }
    }
}

/// After the endpoint welds/snaps, an edge's end vertex can sit off its
/// curve (a fitted SSI curve stops short of the face boundary; a weld moved
/// a shared vertex). Re-fit such an edge so it passes through both vertices:
/// lines are rebuilt exactly, other curves become a cubic interpolant of the
/// old curve's samples with the end samples replaced by the vertices (and the
/// samples nearer than the gap to an end dropped, so no kink forms).
pub fn snap_edge_ends(out: &mut Body, tol: f64) {
    let eps = tol.max(1e-7);
    for e in out.edge_ids().collect::<Vec<_>>() {
        let ed = out.edge(e).clone();
        if !ed.alive || ed.v0 == ed.v1 {
            continue;
        }
        let (p0, p1) = (out.vertex(ed.v0).p, out.vertex(ed.v1).p);
        let (c0, c1) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
        let (d0, d1) = (c0.dist(p0), c1.dist(p1));
        if d0.max(d1) <= eps {
            continue;
        }
        let len = (p1 - p0).len();
        // gaps comparable to the edge itself are not a snap, they are a bug
        if d0.max(d1) > 0.5 * len.max(1e-9) {
            continue;
        }
        let new_curve = if let Curve3::Line { .. } = ed.curve {
            let d = (p1 - p0) * (1.0 / len.max(1e-300));
            Some((Curve3::Line { o: p0, d }, 0.0, len))
        } else {
            let n = 32usize;
            let mut pts = vec![p0];
            for i in 1..n {
                let q = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / n as f64);
                if q.dist(p0) > 1.5 * d0 && q.dist(p1) > 1.5 * d1 && pts.last().map_or(true, |l| l.dist(q) > 1e-9) {
                    pts.push(q);
                }
            }
            pts.push(p1);
            ck_geom::fit::interpolate(&pts, 3.min(pts.len() - 1), ck_geom::fit::Param::ChordLength)
                .ok()
                .map(|c| {
                    let (a, b) = (c.knots[0], c.knots[c.knots.len() - 1]);
                    (Curve3::BSpline(c), a, b)
                })
        };
        if let Some((curve, t0, t1)) = new_curve {
            if std::env::var("TRIM_DEBUG").is_ok() {
                eprintln!("  snap edge {e:?}: ends were {d0:.4}/{d1:.4} off");
            }
            let em = out.edge_mut(e);
            em.curve = curve;
            em.t0 = t0;
            em.t1 = t1;
            for c in out.edge(e).coedges.clone() {
                out.coedge_mut(c).pcurve = None;
            }
        }
    }
}

/// Rebuild loop `l` of face `f`: drop the coedges whose midpoints fall on
/// the wrong side, bridging each discarded run with the cut edges. Returns
/// whether any kept coedge survived (false = the whole face was dropped).
fn rebuild_loop(
    out: &mut Body,
    f: FaceId,
    l: ck_topo::body::LoopId,
    fcuts: &[(EdgeId, VertexId, VertexId)],
    opposing: &[FaceId],
    snaps: &HashMap<FaceId, FaceSnap>,
    flip: bool,
    mutual: bool,
    tol: f64,
) -> bool {
    let cos: Vec<CoedgeId> = out.lp(l).coedges.iter().copied().filter(|&c| out.coedge(c).alive).collect();
    if cos.is_empty() {
        return false;
    }
    let debug = std::env::var("TRIM_DEBUG").is_ok();
    let keep: Vec<bool> = cos.iter().map(|&c| {
        let k = !discarded(out, c, opposing, snaps, flip, mutual);
        if debug {
            let ed = out.edge(out.coedge(c).edge);
            let p = ed.curve.eval((ed.t0 + ed.t1) * 0.5);
            let mut info = String::new();
            for &of in opposing {
                let (u, v, cp) = out.face(of).surface.closest_point(p);
                info.push_str(&format!(
                    " {proj:?}:uv=({u:.3},{v:.3}) d={:.3} in={}",
                    cp.dist(p),
                    snap_contains(out, snaps, of, p),
                    proj = out.face(of).name.id()
                ));
            }
            eprintln!(
                "  face {:?} loop {l:?} coedge {:?} mid=({:.3},{:.3},{:.3}) keep={k}{info}",
                out.face(f).name.id(),
                c,
                p.x,
                p.y,
                p.z
            );
        }
        k
    }).collect();
    if keep.iter().all(|&k| !k) {
        for &c in &cos {
            out.coedge_mut(c).alive = false;
        }
        out.lp_mut(l).coedges.clear();
        return false;
    }
    let mut assigned = vec![false; cos.len()];
    let mut used_cut: HashSet<EdgeId> = Default::default();
    let mut chains: Vec<Vec<(EdgeId, bool)>> = Vec::new();
    for i in 0..cos.len() {
        if !keep[i] || assigned[i] {
            continue;
        }
        let start = out.coedge_start(cos[i]);
        let mut chain: Vec<(EdgeId, bool)> = Vec::new();
        let mut cur: Option<usize> = Some(i);
        let mut tail = start;
        loop {
            if let Some(ci) = cur {
                assigned[ci] = true;
                chain.push((out.coedge(cos[ci]).edge, out.coedge(cos[ci]).rev));
                tail = out.coedge_end(cos[ci]);
                if tail == start {
                    break;
                }
                cur = None;
            }
            // at `tail`: continue along a kept original, else bridge a cut
            if let Some(j) = (0..cos.len()).find(|&j| keep[j] && !assigned[j] && out.coedge_start(cos[j]) == tail) {
                cur = Some(j);
                continue;
            }
            if let Some(&(ce, _, _)) = fcuts.iter().find(|&&(ce, v0, v1)| !used_cut.contains(&ce) && (v0 == tail || v1 == tail)) {
                used_cut.insert(ce);
                let rev = out.edge(ce).v0 != tail;
                chain.push((ce, rev));
                tail = if rev { out.edge(ce).v0 } else { out.edge(ce).v1 };
                if tail == start {
                    break;
                }
                continue;
            }
            // Robustness net: a cut's clip range can stop short of a shared
            // seam vertex (the fit wanders off a patch at the seam), so the
            // chain reaches the vertex with no matching cut endpoint and
            // would dead-end, orphaning a coedge. Weld the open tail to the
            // nearest existing vertex when one is within a face-diag-scaled
            // radius — the seam vertices two faces share sit ~0.6 apart here.
            let (blo, bhi) = face_pts_box(out, f);
            let diag = (bhi - blo).len().max(1.0);
            let near_r = (0.02 * diag).max(tol * 10.0);
            let tp = out.vertex(tail).p;
            let mut best: Option<(f64, VertexId)> = None;
            for v in out.vertex_ids() {
                if !out.vertex(v).alive || v == tail {
                    continue;
                }
                let d = out.vertex(v).p.dist(tp);
                if d <= near_r && best.map(|(bd, _)| d < bd).unwrap_or(true) {
                    best = Some((d, v));
                }
            }
            if let Some((d, v)) = best {
                // repoint the chain's last edge onto the weld vertex
                if let Some(&(le, _)) = chain.last() {
                    if out.edge(le).v0 == tail {
                        out.edge_mut(le).v0 = v;
                    }
                    if out.edge(le).v1 == tail {
                        out.edge_mut(le).v1 = v;
                    }
                    out.vertex_mut(tail).alive = false;
                }
                if debug {
                    eprintln!(
                        "  face {:?} loop {l:?} weld tail=({:.3},{:.3},{:.3}) to ({:.3},{:.3},{:.3}) d={:.4}",
                        out.face(f).name.id(),
                        tp.x, tp.y, tp.z,
                        out.vertex(v).p.x, out.vertex(v).p.y, out.vertex(v).p.z, d
                    );
                }
                tail = v;
                if tail == start {
                    break;
                }
                continue;
            }
            if debug {
                eprintln!(
                    "  face {:?} loop {l:?} dead end at tail=({:.3},{:.3},{:.3}) after {} steps",
                    out.face(f).name.id(),
                    out.vertex(tail).p.x,
                    out.vertex(tail).p.y,
                    out.vertex(tail).p.z,
                    chain.len()
                );
            }
            break; // dead end (open boundary): keep what we have
        }
        if chain.is_empty() {
            continue;
        }
        if debug {
            eprintln!(
                "  face {:?} loop {l:?} chain: {:?}",
                out.face(f).name.id(),
                chain
                    .iter()
                    .map(|&(e, rev)| (e, rev))
                    .collect::<Vec<_>>()
            );
        }
        chains.push(chain);
    }
    if chains.is_empty() {
        for &c in &cos {
            out.coedge_mut(c).alive = false;
        }
        out.lp_mut(l).coedges.clear();
        return false;
    }
    // first chain takes the original loop; any further chain is a new inner loop
    for c in &cos {
        out.coedge_mut(*c).alive = false;
    }
    for (ci, chain) in chains.into_iter().enumerate() {
        let target_l = if ci == 0 {
            out.lp_mut(l).coedges.clear();
            l
        } else {
            out.add_loop(f)
        };
        for (e, rev) in chain {
            out.add_coedge(target_l, e, rev, None);
        }
    }
    true
}

/// Is this coedge on the discarded side? Mutual: its midpoint projects
/// inside an opposing face. Non-mutual: on the wrong side of the tool
/// surface's normal (GUESS for curved tools).
fn discarded(out: &Body, c: CoedgeId, opposing: &[FaceId], snaps: &HashMap<FaceId, FaceSnap>, flip: bool, mutual: bool) -> bool {
    let ed = out.edge(out.coedge(c).edge);
    let p = ed.curve.eval((ed.t0 + ed.t1) * 0.5);
    if mutual {
        // covered by an opposing face: its projection lands inside the face
        // region (boundary counts) AND the point is close enough that it can
        // belong to the overlap (far clamped projections do not). The cap is
        // a fraction of the smaller face-box diagonal.
        opposing.iter().any(|&of| {
            let s = &out.face(of).surface;
            let (_u, _v, cp) = s.closest_point(p);
            let d = cp.dist(p);
            let Some(sn) = snaps.get(&of) else { return false };
            let cap = 0.14 * (sn.hi - sn.lo).len().max(1.0);
            d <= cap && snap_contains(out, snaps, of, p)
        })
    } else {
        let mut best: Option<(f64, Vec3, Vec3)> = None;
        for &of in opposing {
            let s = &out.face(of).surface;
            let (u, v, cp) = s.closest_point(p);
            let d = cp.dist(p);
            if best.map(|(bd, _, _)| d < bd).unwrap_or(true) {
                best = Some((d, cp, s.normal(u, v)));
            }
        }
        let Some((_, cp, n)) = best else { return false };
        let s = (p - cp).dot(n);
        // flip=false keeps the tool-normal side (s >= 0)
        if flip {
            s > tol_sign()
        } else {
            s < -tol_sign()
        }
    }
}

fn tol_sign() -> f64 {
    1e-9
}

/// Pre-rebuild view of a face: outer-loop uv polygon + sampled box.
struct FaceSnap {
    poly: Option<Vec<Uv2>>,
    lo: Vec3,
    hi: Vec3,
}

fn snap_contains(b: &Body, snaps: &HashMap<FaceId, FaceSnap>, f: FaceId, p: Vec3) -> bool {
    match snaps.get(&f).and_then(|s| s.poly.as_ref()) {
        Some(poly) => face_contains_poly(b, f, poly, p),
        None => false,
    }
}

/// Does `p` project inside face `f`'s outer loop (uv containment of the
/// closest param, trying periodic shifts)?
fn face_contains(b: &Body, f: FaceId, p: Vec3) -> bool {
    match face_poly(b, f) {
        Some(poly) => face_contains_poly(b, f, &poly, p),
        None => false,
    }
}

/// Outer-loop uv polygon of `f` (None = no usable loop). Build once per face
/// when many containment queries follow.
fn face_poly(b: &Body, f: FaceId) -> Option<Vec<Uv2>> {
    let &l = b.face(f).loops.first()?;
    if !b.lp(l).alive {
        return None;
    }
    let poly = loop_uv_polygon(b, l);
    (poly.len() >= 3).then_some(poly)
}

fn face_contains_poly(b: &Body, f: FaceId, poly: &[Uv2], p: Vec3) -> bool {
    let face = b.face(f);
    let (u, v) = face.surface.closest_param(p);
    let (pu, pv) = face.surface.periods();
    for ku in -2i32..=2 {
        for kv in -2i32..=2 {
            let uu = u + ku as f64 * pu.unwrap_or(0.0);
            let vv = v + kv as f64 * pv.unwrap_or(0.0);
            if uv_in_poly(Uv2::new(uu, vv), poly) {
                return true;
            }
        }
    }
    false
}

/// Outer-loop uv polygon of a face (each live coedge sampled, unwrap
/// preferring the running range so a half-revolve seam at u=±π doesn't
/// teleport one coedge by a full period).
fn loop_uv_polygon(b: &Body, l: ck_topo::body::LoopId) -> Vec<Uv2> {
    let mut poly: Vec<Uv2> = Vec::new();
    let mut prev: Option<Uv2> = None;
    let (mut umin, mut umax, mut vmin, mut vmax) =
        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for &c in &b.lp(l).coedges {
        if !b.coedge(c).alive {
            continue;
        }
        let ed = b.edge(b.coedge(c).edge);
        let face = &b.face(b.coedge_face(c)).surface;
        let (pu, pv) = face.periods();
        for k in 0..24 {
            let t = ed.t0 + (ed.t1 - ed.t0) * k as f64 / 24.0;
            let (u0, v0) = face.closest_param(ed.curve.eval(t));
            let u = unwrap_in_range(u0, prev.map(|p| p.x), pu, umin, umax);
            let v = unwrap_in_range(v0, prev.map(|p| p.y), pv, vmin, vmax);
            let uv = Uv2::new(u, v);
            umin = umin.min(u);
            umax = umax.max(u);
            vmin = vmin.min(v);
            vmax = vmax.max(v);
            prev = Some(uv);
            poly.push(uv);
        }
    }
    poly
}

/// Unwrap a periodic parameter: prefer a shift landing inside the running
/// range, else the shift nearest `near`.
fn unwrap_in_range(x0: f64, near: Option<f64>, period: Option<f64>, lo: f64, hi: f64) -> f64 {
    let Some(per) = period.filter(|&p| p.abs() > 1e-12) else { return x0 };
    let mut best = x0;
    let mut best_score = f64::INFINITY;
    for k in -2i32..=2 {
        let x = x0 + k as f64 * per;
        let in_range = lo.is_finite() && x >= lo - 1e-9 && x <= hi + 1e-9;
        let score = if in_range {
            0.0
        } else {
            1.0 + near.map(|n| (x - n).abs()).unwrap_or_else(|| x.abs())
        };
        if score < best_score {
            best_score = score;
            best = x;
        }
    }
    best
}

/// Crossing-number point-in-polygon, boundary-inclusive (points within a
/// small margin of an edge count as inside). The overlap/distance cap in
/// `discarded` is what rejects far clamped projections; here the loft's
/// z=0-side rulings — which lie exactly ON the barrel's θ=180 boundary —
/// must count as covered.
fn uv_in_poly(p: Uv2, poly: &[Uv2]) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut scale = 0.0f64;
    for i in 0..n {
        scale = scale.max((poly[i].x - p.x).abs()).max((poly[i].y - p.y).abs());
    }
    let eps = (1e-6f64).max(scale * 1e-9);
    for i in 0..n {
        if uv_dist_to_seg(p, poly[i], poly[(i + 1) % n]) <= eps {
            return true;
        }
    }
    let mut inside = false;
    for i in 0..n {
        let a = poly[i];
        let c = poly[(i + 1) % n];
        if (a.y > p.y) != (c.y > p.y) {
            let x = a.x + (p.y - a.y) / (c.y - a.y) * (c.x - a.x);
            if x > p.x {
                inside = !inside;
            }
        }
    }
    inside
}

fn uv_dist_to_seg(p: Uv2, a: Uv2, b: Uv2) -> f64 {
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let len2 = abx * abx + aby * aby;
    if len2 <= 1e-30 {
        return ((p.x - a.x).powi(2) + (p.y - a.y).powi(2)).sqrt();
    }
    let t = (((p.x - a.x) * abx + (p.y - a.y) * aby) / len2).clamp(0.0, 1.0);
    let dx = p.x - (a.x + t * abx);
    let dy = p.y - (a.y + t * aby);
    (dx * dx + dy * dy).sqrt()
}

/// Sampled box of a face (for the SSI marching bounds).
fn face_pts_box(b: &Body, f: FaceId) -> (Vec3, Vec3) {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = -lo;
    for e in b.face_edges(f) {
        let ed = b.edge(e);
        for p in ed.curve.sample(ed.t0, ed.t1, 16) {
            lo = lo.min(p);
            hi = hi.max(p);
        }
    }
    if !lo.x.is_finite() {
        (Vec3::ZERO, Vec3::ZERO)
    } else {
        (lo, hi)
    }
}

/// Natural t-domain of a curve for marching-window purposes.
fn curve_domain(cv: &Curve3, diag: f64) -> (f64, f64) {
    match cv {
        Curve3::BSpline(b) => (b.knots[0], b.knots[b.knots.len() - 1]),
        Curve3::Circle { .. } | Curve3::Ellipse { .. } | Curve3::Conic { .. } => (0.0, std::f64::consts::TAU),
        _ => (-diag, diag),
    }
}

/// Is `p` on BOTH faces (inside their loops and close to their surfaces)?
fn on_pair(b: &Body, fa: FaceId, fb: FaceId, polys: &(Vec<Uv2>, Vec<Uv2>), p: Vec3, slack: f64) -> bool {
    b.face(fa).surface.distance(p) < slack
        && b.face(fb).surface.distance(p) < slack
        && face_contains_poly(b, fa, &polys.0, p)
        && face_contains_poly(b, fb, &polys.1, p)
}

/// Clip an SSI curve to the (fa, fb) face region: longest sampled run on
/// both faces, refined by bisection at each end. None = no real cut.
fn clip_to_face_pair(b: &Body, fa: FaceId, fb: FaceId, cv: &Curve3, tol: f64) -> Option<(f64, f64)> {
    let (blo, bhi) = face_pts_box(b, fa);
    let diag = (bhi - blo).len().max(1.0);
    let slack = (diag * 1e-4).max(1e-4);
    let (t0, t1) = curve_domain(cv, diag);
    let n = 400usize;
    let (Some(pa), Some(pb)) = (face_poly(b, fa), face_poly(b, fb)) else { return None };
    let polys = (pa, pb);
    let valid = |t: f64| on_pair(b, fa, fb, &polys, cv.eval(t), slack);
    let mut best: Option<(usize, usize)> = None;
    let mut cur: Option<usize> = None;
    for i in 0..=n {
        let t = t0 + (t1 - t0) * i as f64 / n as f64;
        if valid(t) {
            cur.get_or_insert(i);
        } else if let Some(s) = cur.take() {
            let run = (s, i - 1);
            if best.map(|(a, c)| run.1 - run.0 > c - a).unwrap_or(true) {
                best = Some(run);
            }
        }
    }
    if let Some(s) = cur {
        let run = (s, n);
        if best.map(|(a, c)| run.1 - run.0 > c - a).unwrap_or(true) {
            best = Some(run);
        }
    }
    let (a, c) = best?;
    let f = |i: usize| t0 + (t1 - t0) * i as f64 / n as f64;
    // bisect each end down from the sampled resolution
    let mut ta = f(a);
    let mut lo = f(a.saturating_sub(1));
    for _ in 0..40 {
        let m = 0.5 * (lo + ta);
        if valid(m) {
            ta = m;
        } else {
            lo = m;
        }
    }
    let mut tb = f(c);
    let mut hi = f((c + 1).min(n));
    for _ in 0..40 {
        let m = 0.5 * (tb + hi);
        if valid(m) {
            tb = m;
        } else {
            hi = m;
        }
    }
    // extend each end toward the face boundary when the fitted curve leaves
    // the valid run early (fits wander off the patches near a seam): march
    // in fine steps while both surface distances stay small, stopping once
    // the point is near the face's boundary
    let ext_slack = (0.02 * diag).max(10.0 * tol);
    let near = |p: Vec3, f: FaceId| -> f64 {
        let mut best = f64::INFINITY;
        for e in b.face_edges(f) {
            let ed = b.edge(e);
            for q in ed.curve.sample(ed.t0, ed.t1, 24) {
                best = best.min(q.dist(p));
            }
        }
        best
    };
    let on_face = |p: Vec3| {
        b.face(fa).surface.distance(p) < ext_slack && b.face(fb).surface.distance(p) < ext_slack
    };
    let step = (t1 - t0) / n as f64 / 4.0;
    let near_both = |p: Vec3| near(p, fa).min(near(p, fb));
    let bound = (0.01 * diag).max(1.0);
    let dbg = std::env::var("TRIM_DEBUG").is_ok();
    if dbg {
        let pa = cv.eval(ta);
        let pb = cv.eval(tb);
        eprintln!(
            "  clip({:?}x{:?}): ta={ta:.4} tb={tb:.4} pa=({:.4},{:.4},{:.4}) pb=({:.4},{:.4},{:.4}) near_a=({:.4},{:.4}) near_b=({:.4},{:.4}) bound={bound:.4}",
            b.face(fa).name.id(), b.face(fb).name.id(),
            pa.x, pa.y, pa.z, pb.x, pb.y, pb.z,
            near(pa, fa), near(pa, fb), near(pb, fa), near(pb, fb)
        );
    }
    for _ in 0..(4 * n) {
        let t = ta - step;
        if t < t0 {
            break;
        }
        let p = cv.eval(t);
        let n0 = near_both(p);
        if !on_face(p) && n0 > bound {
            break;
        }
        ta = t;
        if n0 <= bound {
            break;
        }
    }
    for _ in 0..(4 * n) {
        let t = tb + step;
        if t > t1 {
            break;
        }
        let p = cv.eval(t);
        let n0 = near_both(p);
        if !on_face(p) && n0 > bound {
            break;
        }
        tb = t;
        if n0 <= bound {
            break;
        }
    }
    if dbg {
        let pa = cv.eval(ta);
        let pb = cv.eval(tb);
        eprintln!(
            "    extended ta={ta:.4} tb={tb:.4} pa=({:.4},{:.4},{:.4}) pb=({:.4},{:.4},{:.4})",
            pa.x, pa.y, pa.z, pb.x, pb.y, pb.z
        );
    }
    if cv.eval(tb).dist(cv.eval(ta)) < tol.max(1e-9) {
        return None;
    }
    Some((ta, tb))
}

/// Nearest point of `face`'s boundary edges to `q`: (distance, edge, param).
/// 24-sample coarse search per edge, then ternary refine.
fn nearest_on_boundary(out: &Body, face: FaceId, q: Vec3) -> Option<(f64, EdgeId, f64)> {
    let mut best: Option<(f64, EdgeId, f64)> = None;
    for e in out.face_edges(face) {
        let ed = out.edge(e);
        let n = 24;
        let step = (ed.t1 - ed.t0) / n as f64;
        let mut ci = 0usize;
        let mut cd = f64::INFINITY;
        for i in 0..=n {
            let d = ed.curve.eval(ed.t0 + step * i as f64).dist(q);
            if d < cd {
                cd = d;
                ci = i;
            }
        }
        let mut lo = (ed.t0 + step * ci as f64 - step).max(ed.t0);
        let mut hi = (ed.t0 + step * ci as f64 + step).min(ed.t1);
        for _ in 0..50 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            if ed.curve.eval(m1).dist(q) < ed.curve.eval(m2).dist(q) {
                hi = m2;
            } else {
                lo = m1;
            }
        }
        let t = 0.5 * (lo + hi);
        let d = ed.curve.eval(t).dist(q);
        if best.map(|(bd, _, _)| d < bd).unwrap_or(true) {
            best = Some((d, e, t));
        }
    }
    best
}

/// Vertex at `p`: welds an existing vertex within `tol`, else splits the
/// face's boundary edge that passes through `p` (snapping to the nearest
/// boundary point within a scaled tolerance — the fitted SSI curve ends a
/// short distance off the exact boundary). None = `p` is interior.
fn endpoint_vertex(out: &mut Body, face: FaceId, p: Vec3, tol: f64) -> Option<VertexId> {
    let (blo, bhi) = face_pts_box(out, face);
    let diag = (bhi - blo).len().max(1.0);
    let snap = (0.05 * diag).max(tol * 10.0);
    // weld radius: the SSI endpoints from different pair-fits differ by up
    // to ~1e-2, so welding at tol alone leaves 4e-5-apart twins and breaks
    // the cut chains — but keep it well under the ~0.6 curve-vs-boundary
    // snap offset so an opposing-face call still splits its own boundary
    let weld = (1e-3 * diag).max(tol);
    let dbg = std::env::var("TRIM_DEBUG").is_ok();
    if let Some(v) = out.vertex_ids().find(|&v| out.vertex(v).alive && out.vertex(v).p.dist(p) <= weld) {
        if dbg {
            eprintln!("  ev({:?}, {:?}): weld to {:?} d={:.4}", out.face(face).name.id(), p, out.vertex(v).p, out.vertex(v).p.dist(p));
        }
        return Some(v);
    }
    let best = nearest_on_boundary(out, face, p);
    let (d, e, t) = best?;
    if d > snap {
        if std::env::var("TRIM_DEBUG").is_ok() {
            eprintln!("endpoint_vertex: no boundary within {snap:.4} of {p:?} (best {d:.4})");
        }
        return None;
    }
    let ed = out.edge(e);
    if (t - ed.t0).abs() <= 1e-12 * (1.0 + ed.t0.abs()) {
        if dbg {
            eprintln!("  ev({:?}, {:?}): at t0 of {e:?} -> {:?}", out.face(face).name.id(), p, out.vertex(ed.v0).p);
        }
        return Some(ed.v0);
    }
    if (ed.t1 - t).abs() <= 1e-12 * (1.0 + ed.t1.abs()) {
        if dbg {
            eprintln!("  ev({:?}, {:?}): at t1 of {e:?} -> {:?}", out.face(face).name.id(), p, out.vertex(ed.v1).p);
        }
        return Some(ed.v1);
    }
    let (v, _) = ck_topo::edit::split_edge(out, e, t);
    if dbg {
        eprintln!("  ev({:?}, {:?}): split {e:?} at t={t:.6} -> {:?} d={d:.4} (diag={diag:.4} snap={snap:.4})", out.face(face).name.id(), p, out.vertex(v).p);
    }
    Some(v)
}

/// Copy a face with shared vertex/edge maps (seams stay shared).
fn copy_face_mapped(
    out: &mut Body,
    shell: ck_topo::body::ShellId,
    src: &Body,
    f: FaceId,
    vmap: &mut HashMap<VertexId, VertexId>,
    emap: &mut HashMap<EdgeId, EdgeId>,
) -> FaceId {
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
            let ne = *emap.entry(co.edge).or_insert_with(|| out.add_edge(e.curve.clone(), e.t0, e.t1, v0, v1));
            out.add_coedge(nloop, ne, co.rev, co.pcurve.clone());
        }
    }
    nf
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
