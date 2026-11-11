//! Control point edits of a rebuilt face (ck-ops/src/surfacing/rebuild/edits.rs).
//! Codes: `freeformEntryInvalid` (a point that is not `[i, j]` with i, j >= 0),
//! `freeformNotSymmetric` (the face is not mirror-symmetric about the entry's
//! plane), `freeformCrosses` (the edited face runs through other faces of the
//! part), `freeformEntryPlane` / `freeformOneFace` / `freeformMapped` are
//! decided by the feature layer.
//!
//! Oracle probes (docs/ck-ops-stubs.md, `ff*` in tools/stub_cases.mjs), box top
//! face rebuilt as a 5 x 5 net (degree 3, 2 spans; `net[i * 5 + j]`, i along u):
//! * an entry `{points:[[i,j]..], of:[nu,nv], frame, dx, dy, dz, proportional,
//!   symmetric, disabled}` moves the listed control points; the OUTER ring of
//!   the net never moves (it carries the edges: editing it changes nothing),
//!   a point outside the net is skipped, a negative index is
//!   `freeformEntryInvalid`; entries made for a net of another size (`of`)
//!   map their points by relative position (`[1,1]` of 3 x 3 -> `[2,2]`);
//! * `frame`: `world` (dx dy dz in X Y Z), `surface` (u, v, n at the control
//!   point's Greville parameters), `plane` (the plane's x, y, normal);
//! * `proportional: p`: a point at index distance `d < p + 1` from a listed
//!   one moves by `(1 - d^2 / (p + 1)^2)^2` of the offset (the listed point
//!   by all of it); checked on the net values of p = 1, 2, 3;
//! * `symmetric: <plane>`: the point mirrored in the plane moves by the
//!   mirrored offset; no mirror point in the net -> `freeformNotSymmetric`;
//! * edits apply in order; the volume change of a centre point `dy` on the
//!   40 x 20 box is dy * 800 / 16 (the integral of its basis functions).

use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, FaceId};

#[derive(Debug, Clone)]
pub enum EditFrame {
    World,
    /// (u, v, n) of the surface at the control point
    Surface,
    /// the plane's x, y, normal
    Plane(Frame),
}

#[derive(Debug, Clone)]
pub struct Edit {
    pub points: Vec<(usize, usize)>,
    /// Net size the entry was made for.
    pub of: Option<(usize, usize)>,
    pub frame: EditFrame,
    pub d: Vec3,
    pub proportional: f64,
    /// Plane of symmetry (origin, unit normal).
    pub symmetric: Option<(Vec3, Vec3)>,
    pub disabled: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct EditError {
    pub code: &'static str,
    /// 1-based entry number.
    pub entry: usize,
}

/// What happened to an entry.
#[derive(Debug, Clone)]
pub struct EntryInfo {
    pub points: Vec<(usize, usize)>,
    /// Mapped from a net of another size.
    pub mapped: bool,
    pub applied: bool,
}

pub struct Edited {
    pub body: Body,
    pub entries: Vec<EntryInfo>,
    pub net: (usize, usize),
}

fn reflect(v: Vec3, n: Vec3) -> Vec3 {
    v - n * (2.0 * v.dot(n))
}

/// Apply `edits` to the net of `face` (a B-spline face, e.g. a rebuilt one) and return the body with
/// the face's surface replaced.
pub fn apply_edits(body: &Body, face: FaceId, edits: &[Edit], tol: f64) -> Result<Edited, EditError> {
    let Surface::BSpline(bs) = &body.face(face).surface else {
        return Err(EditError { code: "freeformEntryInvalid", entry: 0 });
    };
    let (nu, nv) = (bs.nu, bs.nv);
    let mut ctrl = bs.ctrl.clone();
    let size = ctrl.iter().fold(1.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()));
    let gu = crate::surfacing::rebuild::net::greville(bs.deg_u, &bs.knots_u);
    let gv = crate::surfacing::rebuild::net::greville(bs.deg_v, &bs.knots_v);
    let movable = |i: usize, j: usize| i > 0 && j > 0 && i + 1 < nu && j + 1 < nv;
    let mut infos = vec![];
    for (ei, e) in edits.iter().enumerate() {
        let entry = ei + 1;
        let mut info = EntryInfo { points: e.points.clone(), mapped: false, applied: false };
        if e.disabled {
            infos.push(info);
            continue;
        }
        // map to this net
        let mut pts: Vec<(usize, usize)> = vec![];
        for &(i, j) in &e.points {
            let (mi, mj) = match e.of {
                Some((ou, ov)) if (ou, ov) != (nu, nv) && ou > 1 && ov > 1 => {
                    info.mapped = true;
                    (((i as f64) / (ou - 1) as f64 * (nu - 1) as f64).round() as usize, ((j as f64) / (ov - 1) as f64 * (nv - 1) as f64).round() as usize)
                }
                _ => (i, j),
            };
            if mi < nu && mj < nv {
                pts.push((mi, mj));
            }
        }
        // frame vectors at a control point
        let cur = BSplineSurface { ctrl: ctrl.clone(), ..(**bs).clone() };
        let offset = |i: usize, j: usize| -> Vec3 {
            match &e.frame {
                EditFrame::World => e.d,
                EditFrame::Plane(f) => f.x * e.d.x + f.y * e.d.y + f.z * e.d.z,
                EditFrame::Surface => {
                    let d = cur.derivs(gu[i], gv[j], 1);
                    let (su, sv) = (d[1][0], d[0][1]);
                    let n = su.cross(sv).normalize();
                    // outward normal of the face
                    let n = if body.face(face).sense { n } else { -n };
                    su.normalize() * e.d.x + sv.normalize() * e.d.y + n * e.d.z
                }
            }
        };
        let mut moves: Vec<(usize, usize, Vec3)> = vec![];
        for &(i, j) in &pts {
            let off = offset(i, j);
            moves.push((i, j, off));
            if let Some((po, pn)) = e.symmetric {
                let q = ctrl[i * nv + j];
                let mirror = q - pn * (2.0 * (q - po).dot(pn));
                let found = (0..nu * nv).find(|&k| ctrl[k].dist(mirror) <= 1e-6 * size + tol);
                let Some(k) = found else { return Err(EditError { code: "freeformNotSymmetric", entry }) };
                let (mi, mj) = (k / nv, k % nv);
                let moff = reflect(off, pn);
                if (mi, mj) == (i, j) {
                    // on the plane: only the symmetric part of the offset
                    moves.pop();
                    moves.push((i, j, (off + moff) * 0.5));
                } else {
                    moves.push((mi, mj, moff));
                }
            }
        }
        let p = e.proportional;
        let mut delta = vec![Vec3::ZERO; nu * nv];
        for &(i, j, off) in &moves {
            if p > 0.0 {
                let r = p + 1.0;
                for a in 0..nu {
                    for b in 0..nv {
                        let d2 = ((a as f64 - i as f64).powi(2) + (b as f64 - j as f64).powi(2)) / (r * r);
                        if d2 < 1.0 {
                            let w = (1.0 - d2) * (1.0 - d2);
                            delta[a * nv + b] = delta[a * nv + b] + off * w;
                        }
                    }
                }
            } else {
                delta[i * nv + j] = delta[i * nv + j] + off;
            }
        }
        for a in 0..nu {
            for b in 0..nv {
                if movable(a, b) && delta[a * nv + b].len2() > 0.0 {
                    ctrl[a * nv + b] = ctrl[a * nv + b] + delta[a * nv + b];
                    info.applied = true;
                }
            }
        }
        infos.push(info);
    }
    let mut out = body.clone();
    let new = BSplineSurface { ctrl, ..(**bs).clone() };
    crate::surfacing::rebuild::replace::replace_face(&mut out, face, Surface::BSpline(Box::new(new)));
    if crosses_other_faces(&out, face, tol) {
        return Err(EditError { code: "freeformCrosses", entry: edits.len() });
    }
    Ok(Edited { body: out, entries: infos, net: (nu, nv) })
}

/// Interior sample points of a face (grid in its uv window, kept when inside its loops).
fn face_samples(b: &Body, g: FaceId, n: usize) -> Vec<Vec3> {
    use crate::boolean::classify::{face_loops_uv, in_loops};
    let s = &b.face(g).surface;
    let loops = face_loops_uv(b, g);
    let (mut lo, mut hi) = (ck_geom::Vec2::new(f64::INFINITY, f64::INFINITY), ck_geom::Vec2::new(f64::NEG_INFINITY, f64::NEG_INFINITY));
    for l in &loops {
        for p in l {
            lo = ck_geom::Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = ck_geom::Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
        }
    }
    let mut out = vec![];
    if !(lo.x.is_finite() && hi.x.is_finite()) {
        return out;
    }
    for a in 0..n {
        for c in 0..n {
            let uv = ck_geom::Vec2::new(lo.x + (hi.x - lo.x) * (a as f64 + 0.5) / n as f64, lo.y + (hi.y - lo.y) * (c as f64 + 0.5) / n as f64);
            if in_loops(s, uv, &loops) {
                out.push(s.eval(uv.x, uv.y));
            }
        }
    }
    out
}

/// Does the (edited) face pass through another face of the part? A face crosses when its sample
/// points lie on both sides of the edited surface (points within a few `tol` of it, i.e. along
/// the shared edges, do not count).
fn crosses_other_faces(b: &Body, face: FaceId, tol: f64) -> bool {
    let Surface::BSpline(bs) = &b.face(face).surface else { return false };
    for g in b.face_ids().collect::<Vec<_>>() {
        if g == face {
            continue;
        }
        let (mut pos, mut neg) = (false, false);
        for p in face_samples(b, g, 10) {
            let (u, v) = bs.closest_param(p);
            let q = bs.eval(u, v);
            let d = (p - q).dot(bs.normal(u, v));
            if d.abs() <= 10.0 * tol.max(1e-6) {
                continue;
            }
            if d > 0.0 {
                pos = true;
            } else {
                neg = true;
            }
        }
        if pos && neg {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;

    /// 40 x 20 x 10 box (`F2`), top cap `F2.C1` rebuilt as a 5 x 5 net.
    fn rebuilt() -> (Body, FaceId) {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let f = b.find_face("F2.C1").unwrap();
        let out = crate::surfacing::rebuild::rebuild(&b, &[f], 3, 3, 2, 2, 1e-6).unwrap();
        (out, f)
    }

    fn e(points: &[(usize, usize)], d: Vec3) -> Edit {
        Edit { points: points.to_vec(), of: None, frame: EditFrame::World, d, proportional: 0.0, symmetric: None, disabled: false }
    }

    fn vol(b: &Body) -> f64 {
        ck_topo::mass::mass_props(b).volume
    }

    /// Oracle ffEdit / ffProp1 / ffTwoPts / ffMapped: volume change = dy * area * sum(w_ij I_i I_j),
    /// the basis integrals being [1/8, 1/4, 1/4, 1/4, 1/8] (closed forms below; the oracle's own
    /// fit is ~2e-4 off them: 8100.0194 / 8425 / 8199.96).
    #[test]
    fn centre_point_edits_have_closed_form_volumes() {
        let (b, f) = rebuilt();
        let up = Vec3::new(0.0, 2.0, 0.0);
        let r = apply_edits(&b, f, &[e(&[(2, 2)], up)], 1e-6).unwrap();
        assert!((vol(&r.body) - 8100.0).abs() < 1e-6, "{}", vol(&r.body));
        // boundary ring and out-of-net points do not move
        assert!(!apply_edits(&b, f, &[e(&[(0, 2)], up)], 1e-6).unwrap().entries[0].applied);
        assert!((vol(&apply_edits(&b, f, &[e(&[(9, 9)], up)], 1e-6).unwrap().body) - 8000.0).abs() < 1e-6);
        // proportional 1: weights (1 - d^2/4)^2 -> 8000 + 2 * 800 / 16 * (1 + 4 * .5625 + 4 * .25)
        let mut p1 = e(&[(2, 2)], up);
        p1.proportional = 1.0;
        let r = apply_edits(&b, f, &[p1], 1e-6).unwrap();
        assert!((vol(&r.body) - 8425.0).abs() < 1e-6, "{}", vol(&r.body));
        let r = apply_edits(&b, f, &[e(&[(1, 1), (3, 3)], up)], 1e-6).unwrap();
        assert!((vol(&r.body) - 8200.0).abs() < 1e-6, "{}", vol(&r.body));
        // a net of another size maps by relative position
        let mut m = e(&[(1, 1)], up);
        m.of = Some((3, 3));
        let r = apply_edits(&b, f, &[m], 1e-6).unwrap();
        assert!(r.entries[0].mapped && (vol(&r.body) - 8100.0).abs() < 1e-6);
        assert!(ck_topo::check::check(&r.body, 1e-6).is_empty());
    }

    #[test]
    fn symmetric_crossing_and_surface_frame() {
        let (b, f) = rebuilt();
        let up = Vec3::new(0.0, 2.0, 0.0);
        let mut s = e(&[(1, 2)], up);
        s.symmetric = Some((Vec3::new(20.0, 0.0, 0.0), Vec3::X));
        assert!((vol(&apply_edits(&b, f, &[s.clone()], 1e-6).unwrap().body) - 8200.0).abs() < 1e-6);
        s.symmetric = Some((Vec3::new(0.0, 0.0, 0.0), Vec3::X)); // x = 0 is not a symmetry plane of the face
        assert_eq!(apply_edits(&b, f, &[s], 1e-6).err().unwrap().code, "freeformNotSymmetric");
        // surface frame: dz is along the outward normal (+y on the top cap)
        let mut n = e(&[(2, 2)], Vec3::new(0.0, 0.0, 2.0));
        n.frame = EditFrame::Surface;
        assert!((vol(&apply_edits(&b, f, &[n], 1e-6).unwrap().body) - 8100.0).abs() < 1e-6);
        // dipping through the bottom face
        let r = apply_edits(&b, f, &[e(&[(2, 2)], Vec3::new(0.0, -50.0, 0.0))], 1e-6);
        assert_eq!(r.err().unwrap().code, "freeformCrosses");
        assert!(apply_edits(&b, f, &[e(&[(2, 2)], Vec3::new(0.0, -30.0, 0.0))], 1e-6).is_ok());
    }
}
