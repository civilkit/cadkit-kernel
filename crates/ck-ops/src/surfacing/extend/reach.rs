//! How far an extension reaches: plain distances, or up to a target
//! surface / point.
//!
//! Oracle evidence (probe ext3b: square sheet on z=0, edge at y=0 extended
//! `end:"surface"` to a target plane y=−7): the face grows to exactly
//! y ∈ [−7,10] — the extension stops when the moved edge reaches the
//! target. Error strings (kernel.strings): `extendNoTarget`, "the
//! extension does not reach the target within 10 times the face's size",
//! `extendPointBehind` ("the point lies behind the edge"), "the edge
//! touches the target".

use ck_geom::surface::Surface;
use ck_geom::Vec3;

#[derive(Debug, Clone)]
pub struct ReachError {
    pub code: &'static str,
    pub detail: String,
}

impl ReachError {
    fn code(code: &'static str) -> ReachError {
        ReachError { code, detail: String::new() }
    }
}

impl std::fmt::Display for ReachError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.code)
        } else {
            write!(f, "{}: {}", self.code, self.detail)
        }
    }
}

/// Distance from `p0` along `dir` (unit) until hitting the target surface.
/// `size` = the face's size (the 10× search bound). Plane targets are
/// solved in closed form.
pub fn reach_distance(p0: Vec3, dir: Vec3, target: &Surface, size: f64) -> Result<f64, ReachError> {
    match target {
        Surface::Plane { frame } => {
            let denom = dir.dot(frame.z);
            if denom.abs() < 1e-12 {
                return Err(ReachError::code("extendNoTarget"));
            }
            let s = (frame.o.dot(frame.z) - p0.dot(frame.z)) / denom;
            if s <= 1e-9 {
                // the target lies behind or at the edge
                return Err(ReachError::code("extendPointBehind"));
            }
            if s > 10.0 * size.max(1e-9) {
                return Err(ReachError {
                    code: "extendNoTarget",
                    detail: "the extension does not reach the target within 10 times the face's size".into(),
                });
            }
            Ok(s)
        }
        _ => Err(ReachError::code("extendNoTarget")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::Frame;

    #[test]
    fn plane_target_reach() {
        // probe ext3b: edge at y=0 extending -y, target plane y=-7
        let target = Surface::Plane { frame: Frame { o: Vec3::new(0.0, -7.0, 0.0), x: Vec3::X, y: Vec3::Z, z: Vec3::Y } };
        let d = reach_distance(Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0), &target, 10.0).unwrap();
        assert!((d - 7.0).abs() < 1e-9);
        // extending away from the plane -> extendNoTarget (parallel)
        let e = reach_distance(Vec3::new(5.0, 0.0, 0.0), Vec3::X, &target, 10.0).unwrap_err();
        assert_eq!(e.code, "extendNoTarget");
        // target behind -> extendPointBehind
        let e = reach_distance(Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), &target, 10.0).unwrap_err();
        assert_eq!(e.code, "extendPointBehind");
        // too far -> the 10x bound
        let far = Surface::Plane { frame: Frame { o: Vec3::new(0.0, -300.0, 0.0), x: Vec3::X, y: Vec3::Z, z: Vec3::Y } };
        let e = reach_distance(Vec3::new(5.0, 0.0, 0.0), Vec3::new(0.0, -1.0, 0.0), &far, 10.0).unwrap_err();
        assert_eq!(e.code, "extendNoTarget");
        assert!(e.detail.contains("10 times the face's size"), "{}", e.detail);
    }
}
