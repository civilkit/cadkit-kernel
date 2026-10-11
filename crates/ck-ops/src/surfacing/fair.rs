//! The fill's fairing: how the patch surface is fitted to its boundary.
//!
//! The UI offers `meowNURBS` (default), `plate`, `curvature`,
//! `variation` (fair/meow.rs is the meowNURBS fitter). Pinned evidence:
//! the oracle's fill probes use the default fairing and produce the PLANE
//! patch on a planar boundary; "the fairing sets how it bulges" implies
//! the modes differ only where the boundary allows bulging. On a planar
//! boundary every mode yields the same plane (the boundary allows no
//! bulge) — the modes' curved-boundary behaviour is not reconstructed in
//! this pass and falls back to the ruled G0 surface of `fill`.
//!
//! HOW TO WIRE (ck-model, Gate B): feature `fill {fairing, guideStrength}`
//! → parse the mode (absent = meowNURBS; plate on a planar chain omits the
//! field per the UI) and pass it to `fair::fair`; `guideStrength` clamps
//! to [0,1] (UI: `Math.min(1, Math.max(0, value))`).

use ck_geom::surface::Surface;
use ck_geom::Vec3;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fairing {
    MeowNurbs,
    Plate,
    Curvature,
    Variation,
}

impl Fairing {
    pub fn parse(s: &str) -> Option<Fairing> {
        match s {
            "meowNURBS" => Some(Fairing::MeowNurbs),
            "plate" => Some(Fairing::Plate),
            "curvature" => Some(Fairing::Curvature),
            "variation" => Some(Fairing::Variation),
            _ => None,
        }
    }
}

/// Fit a patch surface to the closed boundary polyline (chain + chord,
/// cyclic) with the given fairing. All modes coincide on a planar boundary
/// (the plane); the meowNURBS path runs through `meow::fit`.
pub fn fair(boundary: &[Vec3], mode: Fairing, strength: f64) -> Surface {
    let strength = strength.clamp(0.0, 1.0);
    match mode {
        Fairing::MeowNurbs => meow::fit(boundary, strength),
        Fairing::Plate | Fairing::Curvature | Fairing::Variation => meow::fit(boundary, 0.0),
    }
}

pub mod meow;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_fairing() {
        assert_eq!(Fairing::parse("meowNURBS"), Some(Fairing::MeowNurbs));
        assert_eq!(Fairing::parse("plate"), Some(Fairing::Plate));
        assert_eq!(Fairing::parse("curvature"), Some(Fairing::Curvature));
        assert_eq!(Fairing::parse("variation"), Some(Fairing::Variation));
        assert_eq!(Fairing::parse("bogus"), None);
    }

    #[test]
    fn planar_boundary_gives_the_plane_for_all_modes() {
        let boundary = [
            Vec3::new(0.0, 0.0, 10.0),
            Vec3::new(10.0, 0.0, 10.0),
            Vec3::new(10.0, 10.0, 10.0),
            Vec3::new(0.0, 10.0, 10.0),
        ];
        for mode in [Fairing::MeowNurbs, Fairing::Plate, Fairing::Curvature, Fairing::Variation] {
            for strength in [0.0, 0.5, 1.0] {
                let s = fair(&boundary, mode, strength);
                assert!(s.is_planar(1e-9), "{mode:?} strength {strength}");
                assert_eq!(s.kind(), "plane");
            }
        }
    }
}
