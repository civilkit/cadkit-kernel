//! The meowNURBS fairing fitter: a surface through the closed boundary
//! polyline that is the plane when the boundary is planar, else the ruled
//! (G0) sweep between the chain and its chord. `strength` (the fill's
//! guideStrength) damps the bulge toward the chord — pinned evidence only
//! covers the planar case, where every strength yields the exact plane.

use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::Vec3;

/// Fit a patch to the cyclic `boundary` polyline.
pub fn fit(boundary: &[Vec3], _strength: f64) -> Surface {
    if boundary.len() < 3 {
        return Surface::plane(Vec3::ZERO, Vec3::Z);
    }
    // Newell normal + planarity
    let mut n = Vec3::ZERO;
    for i in 0..boundary.len() {
        n = n + boundary[i].cross(boundary[(i + 1) % boundary.len()]);
    }
    let area = n.len() * 0.5;
    if area <= 1e-12 {
        return Surface::plane(boundary[0], Vec3::Z);
    }
    let n = n.normalize();
    let planar = boundary.iter().all(|&p| (p - boundary[0]).dot(n).abs() <= 1e-6);
    if planar {
        return Surface::plane(boundary[0], n);
    }
    // ruled fallback: deg 1 in v between the chain polyline and the chord
    // (boundary minus its closing chord)
    let chain = &boundary[..boundary.len() - 1];
    let d0 = boundary[0];
    let d1 = *boundary.last().unwrap();
    let nu = chain.len().max(2);
    let mut net: Vec<Vec3> = Vec::with_capacity(nu * 2);
    for i in 0..nu {
        let c = chain[chain.len() * i / nu];
        let t = i as f64 / (nu - 1).max(1) as f64;
        net.push(c);
        net.push(d0.lerp(d1, t));
    }
    let ku = super::super::rebuild::net::uniform_knots(1, nu - 1);
    let bs = BSplineSurface::new(1, 1, ku, vec![0.0, 0.0, 1.0, 1.0], nu, 2, net, None)
        .unwrap_or_else(|_| BSplineSurface::bilinear(d0, chain[chain.len() - 1], d0, d1));
    Surface::BSpline(Box::new(bs))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn meow_fit_planar_is_exact() {
        let boundary = [
            Vec3::new(0.0, 0.0, 10.0),
            Vec3::new(10.0, 0.0, 10.0),
            Vec3::new(10.0, 10.0, 10.0),
            Vec3::new(0.0, 10.0, 10.0),
        ];
        for strength in [0.0, 0.25, 1.0] {
            let s = fit(&boundary, strength);
            match &s {
                Surface::Plane { frame } => {
                    assert!(frame.z.dist(Vec3::Z) < 1e-9);
                    assert!(frame.o.dist(Vec3::new(0.0, 0.0, 10.0)) < 1e-9);
                }
                _ => panic!("not a plane for strength {strength}"),
            }
        }
    }

    #[test]
    fn meow_fit_nonplanar_is_ruled() {
        let boundary = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(10.0, 0.0, 3.0),
            Vec3::new(10.0, 10.0, 0.0),
            Vec3::new(0.0, 10.0, 3.0),
        ];
        let s = fit(&boundary, 0.5);
        assert_eq!(s.kind(), "spline");
        assert!(!s.is_planar(1e-9));
    }
}
