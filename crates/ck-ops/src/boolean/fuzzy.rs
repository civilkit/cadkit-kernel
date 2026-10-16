//! Fuzzy-tolerance booleans (OCCT's "fuzzy value"): geometry of the tool that lies closer than `fuzz` to the
//! host's geometry is treated as coincident.
//!
//! The arrangement itself works at the welding tolerance (1e-6, ladder 10x / 100x); inputs from imports or from
//! a sketch constraint that is 1e-5 off leave faces that nearly coincide, and the intersection curves of such
//! pairs cross back and forth along the whole outline. Instead of loosening the arrangement (which makes
//! every decision in it fuzzy), the tool is **snapped** first: every planar, cylindrical or spherical face of
//! the tool whose surface lies within `fuzz` of an equal-kind surface of the host (same plane normal up to
//! sign, same axis, same centre) is moved onto that surface exactly with the offset machinery of the move-face
//! feature (vertices and edges re-solved on the moved surfaces). The boolean then sees exactly coincident faces
//! and its coincident-face merging (identical-surface detection with orientation voting) applies.
//!
//! The result differs from the exact boolean of the unsnapped inputs by at most `fuzz` in position.
//! A snap that fails, makes the tool invalid, or changes its volume by more than `fuzz` times its area is
//! discarded and the plain boolean runs on the original tool.

use super::apply::{boolean, BoolOut};
use super::Op;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use ck_topo::body::{Body, FaceId};
use ck_topo::check::check;

thread_local! {
    static LAST_SNAPPED: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// Faces the last [`boolean_fuzzy`] call on this thread moved onto the host (0 = the tolerance changed nothing).
pub fn last_snapped() -> usize {
    LAST_SNAPPED.with(|c| c.get())
}

/// What snapping did: the snapped tool and how many faces moved (largest move reported too).
pub struct Snapped {
    pub tool: Body,
    pub faces: usize,
    pub max_move: f64,
}

fn parallel(a: Vec3, b: Vec3) -> bool {
    a.cross(b).len() < 1e-10
}

/// Signed move of tool face `f` (along its OUTWARD normal) that lands it on a host surface within `fuzz`,
/// or `None` when no host face is that close (or it already coincides exactly).
fn snap_move(host: &Body, tool: &Body, f: FaceId, fuzz: f64) -> Option<f64> {
    let face = tool.face(f);
    let mut best: Option<f64> = None; // natural-normal move
    for g in host.face_ids() {
        let nat = match (&face.surface, &host.face(g).surface) {
            (Surface::Plane { frame: ft }, Surface::Plane { frame: fh }) if parallel(ft.z, fh.z) => (fh.o - ft.o).dot(ft.z),
            (Surface::Cylinder { frame: ft, r: rt }, Surface::Cylinder { frame: fh, r: rh })
                if parallel(ft.z, fh.z) && (fh.o - ft.o).cross(ft.z).len() < 1e-9 =>
            {
                rh - rt
            }
            (Surface::Sphere { frame: ft, r: rt }, Surface::Sphere { frame: fh, r: rh }) if (fh.o - ft.o).len() < 1e-9 => rh - rt,
            _ => continue,
        };
        if nat.abs() > 1e-12 && nat.abs() <= fuzz * (1.0 + 1e-9) && best.is_none_or(|b| nat.abs() < b.abs()) {
            best = Some(nat);
        }
        if nat.abs() <= 1e-12 {
            // an exactly coincident host surface exists: the face is fine as it is
            return None;
        }
    }
    best.map(|n| if face.sense { n } else { -n })
}

/// Rebuild the pcurves the offset machinery drops (exact for the analytic pairs; the periodic branch follows the
/// old pcurve of `src`, whose coedge numbering `out` shares).
fn restore_pcurves(out: &mut Body, src: &Body) {
    use ck_geom::curve2::Curve2;
    use ck_geom::Vec2;
    fn shift(pc: &Curve2, d: Vec2) -> Curve2 {
        match pc {
            Curve2::Line { o, d: dir } => Curve2::Line { o: *o + d, d: *dir },
            Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
            Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
            other => other.clone(),
        }
    }
    for c in 0..out.coedges.len() {
        let cid = ck_topo::body::CoedgeId(c as u32);
        if !out.coedge(cid).alive {
            continue;
        }
        let f = out.coedge_face(cid);
        let ed = out.edge(out.coedge(cid).edge).clone();
        let surf = out.face(f).surface.clone();
        let new = ck_geom::pcurve::exact_pcurve(&surf, &ed.curve, 1e-5).or_else(|| ck_geom::pcurve::pcurve_of(&surf, &ed.curve, ed.t0, ed.t1, 1e-6).ok());
        let Some(mut pc) = new else { continue };
        if let Some(old) = &src.coedge(cid).pcurve {
            let (pu, pv) = surf.periods();
            let t = 0.5 * (ed.t0 + ed.t1);
            let (a, b) = (old.eval(t), pc.eval(t));
            let du = pu.map_or(0.0, |p| ((a.x - b.x) / p).round() * p);
            let dv = pv.map_or(0.0, |p| ((a.y - b.y) / p).round() * p);
            if du != 0.0 || dv != 0.0 {
                pc = shift(&pc, Vec2::new(du, dv));
            }
        }
        out.coedges[c].pcurve = Some(pc);
    }
}

/// Snap `tool` onto `host` (see the module doc). `None` when nothing is within `fuzz` or the snap is unusable.
pub fn snap_tool(host: &Body, tool: &Body, fuzz: f64) -> Option<Snapped> {
    if !(fuzz > 0.0) {
        return None;
    }
    let mut moves: Vec<f64> = vec![0.0; tool.faces.len()];
    let mut n = 0;
    let mut max_move = 0.0f64;
    for f in tool.face_ids() {
        if let Some(d) = snap_move(host, tool, f, fuzz) {
            moves[f.idx()] = d;
            max_move = max_move.max(d.abs());
            n += 1;
        }
    }
    if n == 0 {
        return None;
    }
    let before = check(tool, 1e-6).len();
    let mut out = crate::offset::move_faces_body(tool, &|f| crate::offset::FaceMove::Offset(moves[f.idx()])).ok()?;
    restore_pcurves(&mut out, tool);
    if check(&out, 1e-6).len() > before {
        return None;
    }
    let (m0, m1) = (ck_topo::mass::mass_props(tool), ck_topo::mass::mass_props(&out));
    let (v0, v1, area) = (m0.volume, m1.volume, m0.area);
    if !v1.is_finite() || (v1 - v0).abs() > 4.0 * fuzz * area.max(1e-12) {
        return None;
    }
    Some(Snapped { tool: out, faces: n, max_move })
}

/// `a op b` with `b` snapped onto `a` first when `fuzz > 0`. `fuzz == 0` is exactly [`boolean`].
pub fn boolean_fuzzy(a: &Body, b: &Body, op: Op, tol: f64, fuzz: f64) -> Result<BoolOut, String> {
    LAST_SNAPPED.with(|c| c.set(0));
    if fuzz > 0.0 {
        if let Some(s) = snap_tool(a, b, fuzz) {
            super::bool_dbg!("fuzzy {:e}: snapped {} faces (max move {:e})", fuzz, s.faces, s.max_move);
            let r = boolean(a, &s.tool, op, tol);
            if matches!(&r, Ok(o) if o.mismatch.is_none()) {
                LAST_SNAPPED.with(|c| c.set(s.faces));
                return r;
            }
            // the snapped tool did not help: the plain boolean decides (its error or result stands)
            if let Ok(plain) = boolean(a, b, op, tol) {
                return Ok(plain);
            }
            return r;
        }
    }
    boolean(a, b, op, tol)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::{box_body, cyl_body};

    fn vol(b: &Body) -> f64 {
        ck_topo::mass::mass_props(b).volume
    }

    #[test]
    fn near_coincident_wall_snaps_and_cut_opens_the_side() {
        let host = box_body("F2", 0.0, 0.0, 20.0, 20.0, 0.0, 10.0);
        // the tool's side wall lies 3e-5 inside the host's x = 20 wall
        let tool = box_body("F4", 5.0, 5.0, 20.0 - 3e-5, 15.0, -1.0, 11.0);
        let s = snap_tool(&host, &tool, 1e-4).expect("a face is within the fuzzy value");
        assert!(s.faces >= 1 && s.max_move <= 1e-4);
        let r = boolean_fuzzy(&host, &tool, Op::Cut, 1e-6, 1e-4).expect("fuzzy cut");
        assert!(r.mismatch.is_none());
        assert!((vol(&r.body) - 2500.0).abs() < 1e-3, "{}", vol(&r.body));
        assert!(ck_topo::check::check(&r.body, 1e-6).is_empty());
        assert_eq!(last_snapped(), s.faces);
    }

    #[test]
    fn plug_slightly_too_big_fuses_flush() {
        let plate = box_body("F2", -20.0, -20.0, 20.0, 20.0, 0.0, 10.0);
        let hole = cyl_body("F3", 0.0, 0.0, 5.0, -1.0, 11.0);
        let plate = boolean(&plate, &hole, Op::Cut, 1e-6).unwrap().body;
        let plug = cyl_body("F4", 0.0, 0.0, 5.0 + 2e-5, 0.0, 10.0);
        let r = boolean_fuzzy(&plate, &plug, Op::Union, 1e-6, 1e-4).expect("fuzzy fuse");
        assert!(r.mismatch.is_none());
        assert!((vol(&r.body) - 16000.0).abs() < 0.01, "{}", vol(&r.body));
        assert_eq!(ck_topo::region::components(&r.body).len(), 1);
    }

    #[test]
    fn no_fuzz_is_the_plain_boolean() {
        let a = box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
        let b = box_body("F4", 5.0, 5.0, 15.0, 15.0, 0.0, 10.0);
        let p = boolean(&a, &b, Op::Union, 1e-6).unwrap();
        let f = boolean_fuzzy(&a, &b, Op::Union, 1e-6, 0.0).unwrap();
        assert_eq!(p.body, f.body);
        assert_eq!(last_snapped(), 0);
    }

    #[test]
    fn boxes_sharing_only_an_edge_unite_into_two_lumps() {
        let a = box_body("F2", 0.0, 0.0, 10.0, 10.0, 0.0, 10.0);
        let b = box_body("F4", 10.0, 10.0, 20.0, 20.0, 0.0, 10.0);
        let r = boolean(&a, &b, Op::Union, 1e-6).expect("edge contact is not a failure");
        assert!((vol(&r.body) - 2000.0).abs() < 1e-6);
        assert_eq!(ck_topo::region::components(&r.body).len(), 2);
    }
}
