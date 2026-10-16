//! Loft centre line (ck-ops/src/loft/centre.rs).
//!
//! A centre line is an open chain that crosses every profile plane. A STRAIGHT
//! centre line (even slanted) leaves the loft unchanged; a gently curved one
//! shifts intermediate stations along the offset from the chord between the
//! first and last profile centroids (oracle `loftOffsetCentreCurved`). With
//! only two profiles the end offsets are zero, so the loft is unchanged — the
//! oracle still deforms the surface (unreconstructed). A corner between sketch
//! entities is `loftCentreLineKink`; one that bends tighter than the sections
//! allow is `loftCentreLineTight {detail}` (min radius of curvature, mm); one
//! that does not cross every profile plane is `loftCentreLineReach`.
//!
//! Reconstruction notes: the exact oracle deformation for a curved centre line
//! is not fully pinned (volumes of the probe: plain 48000, gentle mx=1/2/3
//! → 47889 / 47566 / 47061, tight mx≥5 refused). The station-offset model
//! here matches the sign and rough magnitude of the volume drop and the
//! mid-height lateral shift; it is not bit-exact.

use ck_geom::Vec3;

/// `loftCentreLineKink`: the chain has a corner (pieces not tangential).
pub const KINK: &str = "loftCentreLineKink";
/// `loftCentreLineTight`: bends too tightly; `detail` is the min radius (mm).
pub const TIGHT: &str = "loftCentreLineTight";

/// Sampled centre line in world space (dense polyline). `joints` are the
/// indices where one sketch entity ends and the next begins (kink checks).
#[derive(Debug, Clone)]
pub struct CentreIn {
    pub pts: Vec<Vec3>,
    /// Sample indices of entity joints (interior vertices of the chain).
    pub joints: Vec<usize>,
}

/// Outcome of analysing a centre line against the loft axis.
#[derive(Debug, Clone)]
pub struct CentrePlan {
    /// Per-station lateral offset to add (empty = straight, no effect).
    pub offsets: Vec<Vec3>,
    /// Min radius of curvature along the line (mm), `None` when undetermined.
    pub min_radius: Option<f64>,
    /// Corner angles (radians) at entity joints above the kink threshold.
    pub kinks: Vec<f64>,
}

/// Max joint angle still considered tangential (≈5°).
const KINK_ANGLE: f64 = 0.087;
/// Relative collinearity slack for the straightness test.
const STRAIGHT_EPS: f64 = 1e-7;

/// Analyse `cl` for a loft whose station centroids run `chord` (first → last
/// profile centroid). Returns the plan; the caller maps errors.
pub fn analyse(cl: &CentreIn, chord: (Vec3, Vec3), n_stations: usize) -> CentrePlan {
    let pts = &cl.pts;
    let mut plan = CentrePlan { offsets: vec![Vec3::ZERO; n_stations], min_radius: None, kinks: vec![] };
    if pts.len() < 2 || n_stations < 2 {
        return plan;
    }
    let (a, b) = chord;
    let chord_len = a.dist(b);
    if chord_len < 1e-12 {
        return plan;
    }
    let axis = (b - a) / chord_len;

    // kinks: only at entity joints (a corner between two sketch pieces)
    for &j in &cl.joints {
        if j == 0 || j + 1 >= pts.len() {
            continue;
        }
        let d0 = (pts[j] - pts[j - 1]).try_normalize();
        let d1 = (pts[j + 1] - pts[j]).try_normalize();
        if let (Some(d0), Some(d1)) = (d0, d1) {
            let ang = d0.dot(d1).clamp(-1.0, 1.0).acos();
            if ang > KINK_ANGLE {
                plan.kinks.push(ang);
            }
        }
    }

    // min radius of curvature: ds / dθ along the polyline chords
    let mut min_r = f64::INFINITY;
    for w in pts.windows(3) {
        let d0 = (w[1] - w[0]).try_normalize();
        let d1 = (w[2] - w[1]).try_normalize();
        if let (Some(d0), Some(d1)) = (d0, d1) {
            let ds = 0.5 * (w[0].dist(w[1]) + w[1].dist(w[2]));
            let th = d0.dot(d1).clamp(-1.0, 1.0).acos();
            if th > 1e-9 {
                min_r = min_r.min(ds / th);
            }
        }
    }
    plan.min_radius = min_r.is_finite().then_some(min_r);

    // straight? every sample within STRAIGHT_EPS * length of the chord axis
    let straight = pts.iter().all(|q| (*q - a).cross(axis).len() < STRAIGHT_EPS * (1.0 + chord_len));
    if straight {
        return plan; // offsets stay zero
    }

    // gentle curve: project each station onto the centre line by arc-length
    // fraction matching the station's chord fraction, then offset by the
    // lateral (chord-normal) displacement of the centre line at that fraction.
    let mut arc = vec![0.0f64; pts.len()];
    for i in 1..pts.len() {
        arc[i] = arc[i - 1] + pts[i - 1].dist(pts[i]);
    }
    let total = *arc.last().unwrap_or(&0.0);
    if total < 1e-12 {
        return plan;
    }
    for s in 0..n_stations {
        let t = s as f64 / (n_stations - 1) as f64;
        let target = t * total;
        // find the sample bracketing `target`
        let j = arc.partition_point(|&x| x < target).min(pts.len() - 1);
        let j0 = j.saturating_sub(1);
        let seg = arc[j] - arc[j0];
        let u = if seg > 1e-12 { ((target - arc[j0]) / seg).clamp(0.0, 1.0) } else { 0.0 };
        let cl_pt = pts[j0] + (pts[j] - pts[j0]) * u;
        let chord_pt = a + (b - a) * t;
        plan.offsets[s] = cl_pt - chord_pt;
    }
    plan
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(a: Vec3, b: Vec3) -> CentreIn {
        CentreIn { pts: (0..=32).map(|k| a + (b - a) * (k as f64 / 32.0)).collect(), joints: vec![] }
    }

    #[test]
    fn straight_is_noop() {
        let a = Vec3::new(0.0, 0.0, 0.0);
        let b = Vec3::new(0.0, 0.0, 30.0);
        let plan = analyse(&line(a, b), (a, b), 2);
        assert!(plan.kinks.is_empty());
        assert!(plan.offsets.iter().all(|o| o.len() < 1e-9));
    }

    #[test]
    fn kink_detected_at_joint() {
        // L-shape: sharp corner at mid, joint index at the corner sample
        let a = Vec3::new(0.0, 0.0, 0.0);
        let c = Vec3::new(0.0, 0.0, 15.0);
        let b = Vec3::new(20.0, 0.0, 15.0);
        let mut pts: Vec<Vec3> = (0..=16).map(|k| a + (c - a) * (k as f64 / 16.0)).collect();
        for k in 1..=16 {
            pts.push(c + (b - c) * (k as f64 / 16.0));
        }
        let plan = analyse(&CentreIn { pts, joints: vec![16] }, (a, b), 2);
        assert!(!plan.kinks.is_empty());
    }

    #[test]
    fn smooth_curve_has_no_kink() {
        // single spline-like bow, no joints
        let pts: Vec<Vec3> = (0..=32)
            .map(|k| {
                let t = k as f64 / 32.0;
                let z = 30.0 * t;
                let x = 2.0 * (t * std::f64::consts::PI).sin();
                Vec3::new(x, 0.0, z)
            })
            .collect();
        let a = Vec3::ZERO;
        let b = Vec3::new(0.0, 0.0, 30.0);
        let plan = analyse(&CentreIn { pts, joints: vec![] }, (a, b), 3);
        assert!(plan.kinks.is_empty());
    }

    #[test]
    fn gentle_curve_offsets_mid() {
        // smooth bow to x=2 at z=15
        let pts: Vec<Vec3> = (0..=32)
            .map(|k| {
                let t = k as f64 / 32.0;
                let z = 30.0 * t;
                let x = 2.0 * (t * std::f64::consts::PI).sin();
                Vec3::new(x, 0.0, z)
            })
            .collect();
        let a = Vec3::ZERO;
        let b = Vec3::new(0.0, 0.0, 30.0);
        let plan = analyse(&CentreIn { pts, joints: vec![] }, (a, b), 3);
        assert!(plan.kinks.is_empty());
        // ends on the chord, mid shifted toward the bow
        assert!(plan.offsets[0].len() < 1e-6);
        assert!(plan.offsets[2].len() < 1e-6);
        assert!(plan.offsets[1].x > 1.0);
    }

    #[test]
    fn tight_reports_radius() {
        // high-frequency wiggle → small radius
        let pts: Vec<Vec3> = (0..=64)
            .map(|k| {
                let t = k as f64 / 64.0;
                Vec3::new(3.0 * (t * 8.0 * std::f64::consts::PI).sin(), 0.0, 30.0 * t)
            })
            .collect();
        let plan = analyse(&CentreIn { pts, joints: vec![] }, (Vec3::ZERO, Vec3::new(0.0, 0.0, 30.0)), 2);
        assert!(plan.min_radius.unwrap() < 10.0);
    }
}
