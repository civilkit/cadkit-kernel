//! Repair Part: make a surface body that came in from an import (see `honest`) into a solid where its geometry allows,
//! and say exactly what was done. Modelled on SolidCraft's `body.repair` (PR #8676), over this kernel's healing:
//!
//! 1. **Faces the reader left out are rebuilt on their own surface**: a loop of free edges whose every point lies on
//!    the surface of a left-out face becomes that face again (the surface unchanged, new trims from the edges).
//! 2. **Gaps up to `gap` are sewn** (near vertices merged, free edges paired and joined, faces turned consistently,
//!    edges that deviate from their faces given a tolerance up to the cap).
//! 3. **Holes are patched** (`patch_holes`): a loop of free edges that lies in one plane gets a flat face. A curved hole
//!    is reported with its size and left open (no curved patch is built yet).
//!
//! The body is changed only through these steps; the caller keeps the original for undo.

use crate::honest::{reasons_of, Reason};
use crate::{heal_with, Fixed, HealOpts};
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3};
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, FaceName, VertexId};
use std::collections::HashMap;

/// Default and largest sewing gap (mm).
pub const DEFAULT_GAP: f64 = 0.2;
pub const MAX_GAP: f64 = 1.0;

#[derive(Debug, Clone, Copy)]
pub struct RepairOpts {
    /// Free edges / vertices this far apart (mm) are sewn. 0 < gap <= MAX_GAP.
    pub gap: f64,
    /// Fill flat holes with a face.
    pub patch_holes: bool,
    /// Largest edge / vertex tolerance (mm) the sewing may set.
    pub cap: f64,
}

impl Default for RepairOpts {
    fn default() -> RepairOpts {
        RepairOpts { gap: DEFAULT_GAP, patch_holes: true, cap: ck_check::DEFAULT_TOL_CAP }
    }
}

/// A face the reader left out, with the surface it lay on (from `SolidRead::refused_surfaces` /
/// `PartRead::refused_surfaces`).
#[derive(Debug, Clone)]
pub struct LeftOut {
    /// The face's id in the file (`#106`, `F5276`, ...).
    pub id: String,
    pub surface: Surface,
    pub sense: bool,
}

/// A loop of free edges found after sewing.
#[derive(Debug, Clone, PartialEq)]
pub struct Hole {
    pub edges: usize,
    /// Largest extent (mm).
    pub size: f64,
    pub planar: bool,
    pub patched: bool,
    /// Why it was left open (empty when patched).
    pub reason: String,
}

#[derive(Debug, Clone, Default)]
pub struct RepairReport {
    /// File ids of the left-out faces rebuilt on their own surface.
    pub rebuilt: Vec<String>,
    /// Left-out faces that could not be rebuilt, with the reason.
    pub not_rebuilt: Vec<Reason>,
    /// Every fix of the sewing pass.
    pub fixed: Vec<Fixed>,
    /// Edges joined by the sewing pass.
    pub sewn: usize,
    pub holes: Vec<Hole>,
    pub faces_before: usize,
    pub faces_after: usize,
    /// Largest edge / vertex tolerance before and after (mm).
    pub tol_before: f64,
    pub tol_after: f64,
    /// No free edge left.
    pub closed: bool,
    /// The validator finds nothing: the result is a solid.
    pub valid: bool,
    /// Volume of the result when it is valid (mm^3).
    pub volume: Option<f64>,
    /// What is still wrong, in plain words.
    pub remaining: Vec<Reason>,
}

/// Repair `b` in place. Refused (Err, body untouched) when the gap is out of range or there is nothing to repair.
pub fn repair(b: &mut Body, left_out: &[LeftOut], o: &RepairOpts) -> Result<RepairReport, Reason> {
    if !(o.gap > 0.0 && o.gap <= MAX_GAP) {
        return Err(Reason { code: "repairGap".into(), faces: vec![], message: format!("the gap must be more than 0 and at most {MAX_GAP} mm (got {})", o.gap) });
    }
    let before = ck_check::validate_cap(b, 1e-6, o.cap);
    if before.is_empty() {
        return Err(Reason { code: "repairNothing".into(), faces: vec![], message: "the body is already a valid solid: there is nothing to repair".into() });
    }
    let mut rep = RepairReport { faces_before: b.num_faces(), tol_before: ck_check::tolerance_used(b), ..Default::default() };
    let scale = scale_of(b);

    // 1. left-out faces on their own surface
    for lo in left_out {
        let holes = free_loops(b);
        let mut done = false;
        for h in &holes {
            if !loop_on(b, h, &lo.surface, 1e-5 * scale) {
                continue;
            }
            let n0 = issue_score(b, o.cap);
            let mut trial = b.clone();
            if add_face_on(&mut trial, h, lo.surface.clone(), FaceName::other("repair", &lo.id)).is_some() && issue_score(&trial, o.cap) < n0 {
                *b = trial;
                done = true;
                break;
            }
        }
        if done {
            rep.rebuilt.push(lo.id.clone());
        } else {
            rep.not_rebuilt.push(Reason {
                code: "repairNotRebuilt".into(),
                faces: vec![lo.id.clone()],
                message: format!("face {} could not be rebuilt on its {} surface: no open boundary lies on it", lo.id, lo.surface.kind()),
            });
        }
    }

    // 2. sew
    let mut ho = HealOpts::new(o.gap);
    ho.tolerant = true;
    ho.cap = o.cap;
    let h = heal_with(b, &ho);
    rep.sewn = h.fixed.iter().filter(|f| f.code.starts_with("SEW") || f.code.contains("SEW")).count();
    rep.fixed = h.fixed;
    ck_topo::region::rebuild_lumps(b);

    // 3. holes
    let mut patched = 0;
    for h in free_loops(b) {
        let pts = loop_points(b, &h);
        let size = extent(&pts);
        let plane = plane_of(&pts, 1e-6 * scale + 1e-9);
        let mut hole = Hole { edges: h.len(), size, planar: plane.is_some(), patched: false, reason: String::new() };
        match plane {
            None => hole.reason = format!("the hole ({} edges, {size:.3} mm across) is not flat; a curved patch is not built yet", h.len()),
            Some(_) if !o.patch_holes => hole.reason = "patching holes was not asked for".into(),
            Some((c, n)) => {
                patched += 1;
                if add_face_on(b, &h, Surface::plane(c, n), FaceName::other("repair", &format!("P{patched}"))).is_some() {
                    hole.patched = true;
                } else {
                    patched -= 1;
                    hole.reason = "the flat patch could not be trimmed by the hole's edges".into();
                }
            }
        }
        rep.holes.push(hole);
    }
    if patched > 0 {
        // the patches join faces that may face the other way: orient once more
        let mut ho = HealOpts::new(1e-6);
        ho.sew = false;
        ho.fix_small = false;
        ho.cap = o.cap;
        let h = heal_with(b, &ho);
        rep.fixed.extend(h.fixed);
        ck_topo::region::rebuild_lumps(b);
    }

    let after = ck_check::validate_cap(b, 1e-6, o.cap);
    rep.faces_after = b.num_faces();
    rep.tol_after = ck_check::tolerance_used(b);
    rep.closed = free_loops(b).is_empty() && !after.iter().any(|i| i.code == "SHELL_OPEN");
    rep.valid = after.is_empty();
    rep.volume = rep.valid.then(|| ck_topo::mass::mass_props(b).volume);
    rep.remaining = reasons_of(&after);
    Ok(rep)
}

/// Validator findings, open-shell ones weighted so that closing a hole always counts as progress.
fn issue_score(b: &Body, cap: f64) -> usize {
    ck_check::validate_cap(b, 1e-6, cap).iter().map(|i| if i.code == "SHELL_OPEN" { 1 } else { 4 }).sum()
}

fn scale_of(b: &Body) -> f64 {
    let (lo, hi) = b.bbox();
    1.0 + lo.dist(hi)
}

/// One hole edge in the direction the filling face must use it: (edge, reversed, from, to).
type HoleEdge = (EdgeId, bool, VertexId, VertexId);

/// Loops of free edges (used by one face only), each in the direction a face filling it uses its edges.
pub fn free_loops(b: &Body) -> Vec<Vec<HoleEdge>> {
    let mut from: HashMap<VertexId, Vec<HoleEdge>> = HashMap::new();
    let mut n = 0;
    for e in b.edge_ids() {
        let cs: Vec<CoedgeId> = b.edge_coedges(e);
        if cs.len() != 1 {
            continue;
        }
        let c = cs[0];
        let he = (e, !b.coedge(c).rev, b.coedge_end(c), b.coedge_start(c));
        from.entry(he.2).or_default().push(he);
        n += 1;
    }
    let mut out = vec![];
    let mut keys: Vec<VertexId> = from.keys().copied().collect();
    keys.sort_by_key(|v| v.idx());
    for start in keys {
        while let Some(first) = from.get_mut(&start).and_then(|v| v.pop()) {
            let mut lp = vec![first];
            let mut at = first.3;
            let mut ok = true;
            while at != start {
                match from.get_mut(&at).and_then(|v| v.pop()) {
                    Some(he) => {
                        at = he.3;
                        lp.push(he);
                    }
                    None => {
                        ok = false;
                        break;
                    }
                }
                if lp.len() > n {
                    ok = false;
                    break;
                }
            }
            if ok {
                out.push(lp);
            }
        }
    }
    out
}

/// Points along a hole (8 per edge, in loop order, without repeating the closing point).
fn loop_points(b: &Body, h: &[HoleEdge]) -> Vec<Vec3> {
    let mut pts = vec![];
    for &(e, rev, _, _) in h {
        let ed = b.edge(e);
        for k in 0..8 {
            let s = k as f64 / 8.0;
            let t = if rev { ed.t1 + (ed.t0 - ed.t1) * s } else { ed.t0 + (ed.t1 - ed.t0) * s };
            pts.push(ed.curve.eval(t));
        }
    }
    pts
}

fn extent(pts: &[Vec3]) -> f64 {
    let mut m: f64 = 0.0;
    for (i, p) in pts.iter().enumerate() {
        for q in &pts[i + 1..] {
            m = m.max(p.dist(*q));
        }
    }
    m
}

/// The plane of a closed polyline (point, unit normal about which it runs counter-clockwise) when every point is within
/// `tol` of it.
fn plane_of(pts: &[Vec3], tol: f64) -> Option<(Vec3, Vec3)> {
    if pts.len() < 3 {
        return None;
    }
    let mut n = Vec3::ZERO;
    let mut c = Vec3::ZERO;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        n = n + Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
        c = c + p;
    }
    let l = n.len();
    if l < 1e-12 {
        return None;
    }
    let n = n * (1.0 / l);
    let c = c * (1.0 / pts.len() as f64);
    pts.iter().all(|p| (*p - c).dot(n).abs() <= tol).then_some((c, n))
}

fn loop_on(b: &Body, h: &[HoleEdge], s: &Surface, tol: f64) -> bool {
    loop_points(b, h).iter().all(|p| s.distance(*p) <= tol)
}

/// Add a face on `surface` bounded by hole `h` (in the shell of the face next to its first edge). The sense is the one
/// that makes the loop run counter-clockwise about the face normal. None (body untouched) when a trim cannot be made.
fn add_face_on(b: &mut Body, h: &[HoleEdge], surface: Surface, name: FaceName) -> Option<FaceId> {
    let (pu, pv) = surface.periods();
    let mut pcs = vec![];
    let mut prev_end: Option<Vec2> = None;
    for &(e, rev, _, _) in h {
        let ed = b.edge(e);
        let mut pc = ck_geom::pcurve::pcurve_of(&surface, &ed.curve, ed.t0, ed.t1, 1e-6).ok()?;
        let (ts, te) = if rev { (ed.t1, ed.t0) } else { (ed.t0, ed.t1) };
        // periodic surfaces: each edge's trim continues where the previous one ended
        if let Some(pe) = prev_end {
            let s = pc.eval(ts);
            let du = pu.map_or(0.0, |p| ((pe.x - s.x) / p).round() * p);
            let dv = pv.map_or(0.0, |p| ((pe.y - s.y) / p).round() * p);
            if du != 0.0 || dv != 0.0 {
                pc = shift(&pc, du, dv)?;
            }
        }
        prev_end = Some(pc.eval(te));
        pcs.push((pc, ts, te));
    }
    // a loop that does not close in the parameter plane would need a seam: not rebuilt
    let start = pcs.first()?.0.eval(pcs[0].1);
    if prev_end?.dist(start) > 1e-6 {
        return None;
    }
    let mut area = 0.0;
    for (pc, ts, te) in &pcs {
        for k in 0..16 {
            let a = pc.eval(ts + (te - ts) * k as f64 / 16.0);
            let c = pc.eval(ts + (te - ts) * (k + 1) as f64 / 16.0);
            area += a.x * c.y - c.x * a.y;
        }
    }
    if area.abs() < 1e-14 {
        return None;
    }
    let first = b.edge_coedges(h[0].0).first().copied()?;
    let shell = b.face(b.coedge_face(first)).shell;
    let f = b.add_face(shell, surface, area > 0.0, name);
    let lp = b.add_loop(f);
    for (&(e, rev, _, _), (pc, _, _)) in h.iter().zip(pcs) {
        b.add_coedge(lp, e, rev, Some(pc));
    }
    Some(f)
}

fn shift(pc: &ck_geom::curve2::Curve2, du: f64, dv: f64) -> Option<ck_geom::curve2::Curve2> {
    use ck_geom::curve2::Curve2;
    let d = Vec2::new(du, dv);
    Some(match pc {
        Curve2::Line { o, d: dd } => Curve2::Line { o: *o + d, d: *dd },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
        Curve2::BSpline(b) => Curve2::BSpline(b.map(|p| p + d)),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;
    use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

    fn block() -> Body {
        let p = [Vec2::new(0.0, 0.0), Vec2::new(10.0, 0.0), Vec2::new(10.0, 20.0), Vec2::new(0.0, 20.0)];
        let tags = ["e1", "e2", "e3", "e4"];
        let outer = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
        extrude("F1", &Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }]), ExtrudeOpts::blind(30.0)).unwrap()
    }

    fn without_face(k: usize) -> (Body, Surface, bool) {
        let mut b = block();
        let f = b.face_ids().nth(k).unwrap();
        let (s, sense) = (b.face(f).surface.clone(), b.face(f).sense);
        ck_topo::edit::delete_face(&mut b, f);
        ck_topo::region::rebuild_lumps(&mut b);
        (b, s, sense)
    }

    #[test]
    fn a_left_out_face_is_rebuilt_on_its_own_surface() {
        let (mut b, s, sense) = without_face(2);
        let r = repair(&mut b, &[LeftOut { id: "#106".into(), surface: s, sense }], &RepairOpts::default()).unwrap();
        assert_eq!(r.rebuilt, vec!["#106".to_string()]);
        assert!(r.holes.is_empty(), "{:?}", r.holes);
        assert!(r.valid && r.closed, "{:?}", r.remaining);
        assert!((r.volume.unwrap() - 6000.0).abs() < 1e-6, "{:?}", r.volume);
        assert_eq!((r.faces_before, r.faces_after), (5, 6));
    }

    #[test]
    fn a_flat_hole_is_patched() {
        let (mut b, _, _) = without_face(0);
        let r = repair(&mut b, &[], &RepairOpts::default()).unwrap();
        assert_eq!(r.holes.len(), 1);
        assert!(r.holes[0].planar && r.holes[0].patched, "{:?}", r.holes[0]);
        assert!(r.valid, "{:?}", r.remaining);
        assert!((r.volume.unwrap() - 6000.0).abs() < 1e-6);
    }

    #[test]
    fn a_hole_is_offered_not_patched_when_not_asked() {
        let (mut b, _, _) = without_face(0);
        let r = repair(&mut b, &[], &RepairOpts { patch_holes: false, ..Default::default() }).unwrap();
        assert_eq!(r.holes.len(), 1);
        assert!(r.holes[0].planar && !r.holes[0].patched);
        assert!(!r.valid && !r.closed);
        assert!(r.remaining.iter().any(|x| x.code == "SHELL_OPEN"));
    }

    #[test]
    fn a_small_gap_is_sewn() {
        // one vertex of the box moved 0.05 mm: the edges at it no longer meet their neighbours'
        let mut b = block();
        let damaged = crate::damage::apply(&mut b, "detach", 0.05);
        assert!(damaged);
        assert!(!ck_check::validate(&b, 1e-6).is_empty());
        let r = repair(&mut b, &[], &RepairOpts::default()).unwrap();
        assert!(r.valid, "{:?} {:?}", r.remaining, r.fixed);
        assert!(r.holes.is_empty());
    }

    #[test]
    fn nothing_to_repair_and_bad_gaps_are_refused() {
        let mut b = block();
        assert_eq!(repair(&mut b, &[], &RepairOpts::default()).unwrap_err().code, "repairNothing");
        let (mut o, _, _) = without_face(0);
        for g in [0.0, -1.0, 1.5, f64::NAN] {
            assert_eq!(repair(&mut o, &[], &RepairOpts { gap: g, ..Default::default() }).unwrap_err().code, "repairGap");
        }
    }
}
