//! Surface rebuild: refit a face's surface onto a degree × spans net,
//! keeping the edges (their pcurves stay valid — the rebuild preserves the
//! uv parameterisation over the face's uv window).
//!
//! Pinned by oracle probe rb1 (box F2, rebuild the top cap F2.C1 with
//! degreeU/V=3, spansU/V=2): the face's kind becomes `spline`, its name and
//! its 12 edges are unchanged, and massProps stays at volume 1000 / area
//! 600 (the oracle's own numbers sit ~2e-10 off the exact values — its
//! control net is a least-squares fit rather than an exact interpolation).
//!
//! Errors: `rebuildPeriodicFace` (periodic faces must be split first — the
//! UI says so), `rebuildTooFine` (spans above the UI cap 50, GUESS),
//! `rebuildFailed` ("The face cannot be rebuilt" / singular fit),
//! `rebuildDirectionEdge` (the net-direction edge is not one of the
//! face's edges — payload validation, Gate B).
//!
//! HOW TO WIRE (ck-model, Gate B): feature `freeform` with `edits: []`
//! (the rebuildFace preset: `degreeU/V, spansU/V, useTolerance, tolerance,
//! extend, shrink`) → resolve the faces, then
//! `rebuild::rebuild(&body, &faces, deg_u, deg_v, spans_u, spans_v, tol)`;
//! per-face continuity requirements (edges[]) belong to the feature layer
//! (rebuildContinuityDegree/Rows/Inner).

pub mod deviation;
pub mod edits;
pub mod net;
pub mod replace;


use ck_topo::body::Body;

#[derive(Debug, Clone)]
pub struct RebuildError {
    pub code: &'static str,
    pub detail: String,
}

impl RebuildError {
    fn code(code: &'static str) -> RebuildError {
        RebuildError { code, detail: String::new() }
    }
}

impl std::fmt::Display for RebuildError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.code)
        } else {
            write!(f, "{}: {}", self.code, self.detail)
        }
    }
}

/// Refit each named face's surface onto the net. Returns the body with the
/// faces' surfaces replaced (edges untouched).
pub fn rebuild(
    body: &Body,
    faces: &[ck_topo::body::FaceId],
    deg_u: usize,
    deg_v: usize,
    spans_u: usize,
    spans_v: usize,
    tol: f64,
) -> Result<Body, RebuildError> {
    if faces.is_empty() {
        return Err(RebuildError::code("missingReference"));
    }
    if spans_u > 50 || spans_v > 50 {
        return Err(RebuildError::code("rebuildTooFine"));
    }
    let mut out = body.clone();
    for &f in faces {
        let face = out.face(f);
        let periodic = face.surface.periods().0.is_some() || face.surface.periods().1.is_some();
        if periodic {
            return Err(RebuildError::code("rebuildPeriodicFace"));
        }
        // the face's uv window (its loops' parametric bounds)
        let mut uw = (f64::INFINITY, f64::NEG_INFINITY);
        let mut vw = (f64::INFINITY, f64::NEG_INFINITY);
        for l in &face.loops {
            for &c in &out.lp(*l).coedges {
                if !out.coedge(c).alive {
                    continue;
                }
                let e = out.edge(out.coedge(c).edge);
                for k in 0..=4 {
                    let t = e.t0 + (e.t1 - e.t0) * (k as f64 / 4.0);
                    let uv = out.coedge_uv(c, t, None);
                    uw = (uw.0.min(uv.x), uw.1.max(uv.x));
                    vw = (vw.0.min(uv.y), vw.1.max(uv.y));
                }
            }
        }
        if !(uw.0.is_finite() && vw.0.is_finite()) {
            return Err(RebuildError::code("rebuildFailed"));
        }
        let surface = out.face(f).surface.clone();
        let bs = net::fit_surface(&surface, uw, vw, deg_u, deg_v, spans_u, spans_v)
            .map_err(|_| RebuildError::code("rebuildFailed"))?;
        let (max, _mean) = deviation::deviation(&surface, &bs, uw, vw, 6);
        if max > tol {
            return Err(RebuildError::code("rebuildFailed"));
        }
        replace::replace_face(&mut out, f, ck_geom::surface::Surface::BSpline(Box::new(bs)));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::curve::Curve;
    use ck_geom::surface::Surface;
    use ck_geom::Vec3;
    use ck_topo::body::{FaceName, Role};

    /// A unit-test box solid (10×10×10) like the probe: extrude F2 with
    /// caps C0 (z=0) / C1 (z=10) and side faces S.e0..e3, shared edges.
    fn box_body() -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let cap = |k: u32| FaceName { feature: "F2".into(), role: Role::Cap(k), region: None, suffix: String::new() };
        let side = |e: &str| FaceName { feature: "F2".into(), role: Role::Side(e.into()), region: None, suffix: String::new() };
        let p = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let bot: Vec<_> = p.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, 0.0))).collect();
        let top: Vec<_> = p.iter().map(|&(x, y)| b.add_vertex(Vec3::new(x, y, 10.0))).collect();
        let mut bot_e = Vec::new();
        let mut top_e = Vec::new();
        let mut rails = Vec::new();
        for i in 0..4 {
            let j = (i + 1) % 4;
            bot_e.push(b.add_edge(Curve::line_through(p[i].into3(0.0), p[j].into3(0.0)), 0.0, 10.0, bot[i], bot[j]));
            top_e.push(b.add_edge(Curve::line_through(p[i].into3(10.0), p[j].into3(10.0)), 0.0, 10.0, top[i], top[j]));
            rails.push(b.add_edge(Curve::line_through(p[i].into3(0.0), p[i].into3(10.0)), 0.0, 10.0, bot[i], top[i]));
        }
        let f0 = b.add_face(shell, Surface::plane(Vec3::new(0.0, 0.0, 0.0), -Vec3::Z), true, cap(0));
        {
            let lp = b.add_loop(f0);
            for i in 0..4 {
                b.add_coedge(lp, bot_e[i], true, None);
            }
        }
        let f1 = b.add_face(shell, Surface::plane(Vec3::new(0.0, 0.0, 10.0), Vec3::Z), true, cap(1));
        {
            let lp = b.add_loop(f1);
            for i in 0..4 {
                b.add_coedge(lp, top_e[i], false, None);
            }
        }
        let walls: [(&str, [f64; 3], [f64; 3]); 4] = [
            ("e0", [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
            ("e1", [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]),
            ("e2", [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]),
            ("e3", [0.0, 10.0, 0.0], [0.0, 0.0, 0.0]),
        ];
        for (k, (ent, a, c)) in walls.iter().enumerate() {
            let pa = Vec3::new(a[0], a[1], a[2]);
            let pb = Vec3::new(c[0], c[1], c[2]);
            let travel = (pb - pa).normalize();
            let nn = travel.cross(Vec3::Z);
            let f = b.add_face(shell, Surface::plane(pa, nn), true, side(ent));
            let lp = b.add_loop(f);
            b.add_coedge(lp, bot_e[k], false, None);
            b.add_coedge(lp, rails[(k + 1) % 4], false, None);
            b.add_coedge(lp, top_e[k], true, None);
            b.add_coedge(lp, rails[k], true, None);
        }
        b
    }

    trait Into3 {
        fn into3(self, z: f64) -> Vec3;
    }
    impl Into3 for (f64, f64) {
        fn into3(self, z: f64) -> Vec3 {
            Vec3::new(self.0, self.1, z)
        }
    }

    #[test]
    fn rebuild_box_cap_oracle() {
        // probe rb1: rebuild F2.C1 with degree 3, spans 2 both directions
        let body = box_body();
        let f = body.face_ids().find(|&f| body.face(f).name.id() == "F2.C1").unwrap();
        let b = rebuild(&body, &[f], 3, 3, 2, 2, 1e-6).unwrap();
        let rf = b.face_ids().find(|&f| b.face(f).name.id() == "F2.C1").unwrap();
        assert_eq!(b.face(rf).surface.kind(), "spline");
        assert_eq!(b.face_ids().count(), 6);
        assert_eq!(b.edge_ids().count(), 12);
        for (of, nf) in body.face_ids().zip(b.face_ids()) {
            assert_eq!(body.face(of).name.id(), b.face(nf).name.id());
        }
        let mp = ck_topo::mass::mass_props(&b);
        assert!((mp.volume - 1000.0).abs() < 1e-6, "volume {}", mp.volume);
        assert!((mp.area - 600.0).abs() < 1e-6, "area {}", mp.area);
    }

    #[test]
    fn rebuild_errors() {
        let body = box_body();
        let f = body.face_ids().next().unwrap();
        // periodic faces -> rebuildPeriodicFace
        let mut cyl = body.clone();
        let sh = cyl.shell_ids().next().unwrap();
        let cf = cyl.add_face(
            sh,
            Surface::Cylinder { frame: ck_geom::Frame { o: Vec3::new(5.0, 5.0, 0.0), x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r: 1.0 },
            true,
            FaceName { feature: "F2".into(), role: Role::Side("e9".into()), region: None, suffix: String::new() },
        );
        let e = rebuild(&cyl, &[cf], 3, 3, 2, 2, 1e-6).unwrap_err();
        assert_eq!(e.code, "rebuildPeriodicFace");
        // spans above the cap -> rebuildTooFine (GUESS)
        let e = rebuild(&body, &[f], 3, 3, 51, 2, 1e-6).unwrap_err();
        assert_eq!(e.code, "rebuildTooFine");
        // empty ref list -> missingReference
        let e = rebuild(&body, &[], 3, 3, 2, 2, 1e-6).unwrap_err();
        assert_eq!(e.code, "missingReference");
    }
}
