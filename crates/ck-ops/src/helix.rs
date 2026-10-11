//! Helix curves for the `helix` feature (ck-ops/src/helix.rs).
//!
//! Wire shape pinned by oracle regens (`result.curves` = `[[featureId,
//! curve]]`), helix on a top-plane sketch circle c=(3,4) r=5:
//! `{pts, axis:false, t0:0, t1:angle, helix:{o, a, x, y, k, r, angle}}`
//! with p(θ) = o + (x cos θ + y sin θ)·r + a·k·θ, θ ∈ [0, angle]:
//! - `o` = circle centre, `a` = sketch normal (`reverse` → −normal),
//! - `x` = frame.x cos s + frame.y sin s (s = startAngle°, sketch frame
//!   axes, independent of hand / reverse),
//! - `y` = a × x (right hand) or −(a × x) (`leftHand`),
//! - `k` = pitch / 2π, `angle` = 2π · revolutions,
//! - definedBy: `pitchRevs` (pitch, revolutions), `heightRevs` (pitch =
//!   height / revolutions), `heightPitch` (revolutions = height / pitch);
//!   unknown values behave as `pitchRevs`. Non-positive inputs →
//!   `invalidValue` (pitch 0 / −2, revolutions 0 / −1, height −5).
//! - `pts`: n + 1 samples at θ_i = angle·i/n, n = max(2, ⌈32·revs⌉)
//!   (3 revs → 97, 2.5 → 81, 0.3 → 11, 0.01 → 3).
//! The sketch-level `helixNeedsCircle` check lives in ck-model.

use ck_geom::curve::Curve;
use ck_geom::fit::{interpolate, Param};
use ck_geom::{Frame, Vec3};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DefinedBy {
    PitchRevs,
    HeightRevs,
    HeightPitch,
}

impl DefinedBy {
    /// Wire value; unknown strings fall back to `pitchRevs` (oracle).
    pub fn parse(s: &str) -> DefinedBy {
        match s {
            "heightRevs" => DefinedBy::HeightRevs,
            "heightPitch" => DefinedBy::HeightPitch,
            _ => DefinedBy::PitchRevs,
        }
    }
}

/// Feature fields (frontend defaults: pitchRevs, pitch 1, revolutions 10,
/// height 10, startAngle 0).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HelixParams {
    pub defined_by: DefinedBy,
    pub pitch: f64,
    pub revolutions: f64,
    pub height: f64,
    /// degrees
    pub start_angle: f64,
    pub left_hand: bool,
    pub reverse: bool,
}

impl Default for HelixParams {
    fn default() -> Self {
        HelixParams {
            defined_by: DefinedBy::PitchRevs,
            pitch: 1.0,
            revolutions: 10.0,
            height: 10.0,
            start_angle: 0.0,
            left_hand: false,
            reverse: false,
        }
    }
}

/// Circular helix p(θ) = o + (x cos θ + y sin θ) r + a k θ.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Helix {
    pub o: Vec3,
    pub a: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub r: f64,
    pub k: f64,
    pub angle: f64,
}

/// Build the helix of a sketch circle (centre `c`, radius `r`) on sketch
/// frame `frame`. Error: `invalidValue`.
pub fn helix(frame: &Frame, c: Vec3, r: f64, p: &HelixParams) -> Result<Helix, String> {
    let bad = || Err("invalidValue".to_string());
    let (pitch, revs) = match p.defined_by {
        DefinedBy::PitchRevs => (p.pitch, p.revolutions),
        DefinedBy::HeightRevs => {
            if !(p.height > 0.0 && p.revolutions > 0.0) {
                return bad();
            }
            (p.height / p.revolutions, p.revolutions)
        }
        DefinedBy::HeightPitch => {
            if !(p.height > 0.0 && p.pitch > 0.0) {
                return bad();
            }
            (p.pitch, p.height / p.pitch)
        }
    };
    if !(pitch > 0.0 && revs > 0.0 && r > 0.0) || !pitch.is_finite() || !revs.is_finite() {
        return bad();
    }
    let a = if p.reverse { -frame.z } else { frame.z };
    let s = p.start_angle.to_radians();
    let x = frame.x * s.cos() + frame.y * s.sin();
    let y = if p.left_hand { -a.cross(x) } else { a.cross(x) };
    Ok(Helix { o: c, a, x, y, r, k: pitch / TAU, angle: TAU * revs })
}

impl Helix {
    pub fn revolutions(&self) -> f64 {
        self.angle / TAU
    }
    pub fn pitch(&self) -> f64 {
        self.k * TAU
    }
    pub fn height(&self) -> f64 {
        self.k * self.angle
    }
    pub fn eval(&self, t: f64) -> Vec3 {
        self.o + (self.x * t.cos() + self.y * t.sin()) * self.r + self.a * self.k * t
    }
    /// [p, p', p'', p''']
    pub fn derivs(&self, t: f64) -> [Vec3; 4] {
        let (s, c) = t.sin_cos();
        let rad = |cx: f64, sy: f64| (self.x * cx + self.y * sy) * self.r;
        [self.eval(t), rad(-s, c) + self.a * self.k, rad(-c, -s), rad(s, -c)]
    }
    /// Sample count of the wire `pts` (n segments).
    pub fn segments(&self) -> usize {
        ((32.0 * self.revolutions()).ceil() as usize).max(2)
    }
    /// Wire `pts`.
    pub fn pts(&self) -> Vec<Vec3> {
        let n = self.segments();
        (0..=n).map(|i| self.eval(self.angle * i as f64 / n as f64)).collect()
    }
    /// Cubic B-spline approximation (for sweeps / STEP): interpolates
    /// `per_rev` points per revolution. GUESS: the kernel's own sweep path
    /// representation is unknown.
    pub fn to_curve(&self, per_rev: usize) -> Result<Curve, String> {
        let n = ((per_rev as f64 * self.revolutions()).ceil() as usize).max(4);
        let pts: Vec<Vec3> = (0..=n).map(|i| self.eval(self.angle * i as f64 / n as f64)).collect();
        let mut b = interpolate(&pts, 3, Param::Uniform)?;
        b = b.reparam(0.0, self.angle);
        Ok(Curve::BSpline(b))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn top() -> Frame {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0., 0., -1.), z: Vec3::Y }
    }
    fn close(a: Vec3, b: [f64; 3]) -> bool {
        (a - Vec3::new(b[0], b[1], b[2])).len() < 1e-13
    }

    #[test]
    fn oracle_fields() {
        let c = Vec3::new(3., 0., -4.);
        let p = HelixParams { pitch: 2.0, revolutions: 3.0, ..Default::default() };
        let h = helix(&top(), c, 5.0, &p).unwrap();
        assert_eq!(h.a, Vec3::Y);
        assert_eq!(h.x, Vec3::X);
        assert_eq!(h.y, Vec3::new(0., 0., -1.));
        assert_eq!(h.k, 0.3183098861837907);
        assert_eq!(h.angle, 18.84955592153876);
        let pts = h.pts();
        assert_eq!(pts.len(), 97);
        assert!(close(pts[0], [8., 0., -4.]));
        assert!(close(pts[1], [7.903926402016152, 0.0625, -4.975451610080642]));
        assert!(close(pts[96], [8., 6., -3.9999999999999964]));
        // startAngle 30 + leftHand
        let p = HelixParams { defined_by: DefinedBy::HeightRevs, revolutions: 2.5, height: 6.0, start_angle: 30.0, left_hand: true, ..p };
        let h = helix(&top(), c, 5.0, &p).unwrap();
        assert_eq!(h.x, Vec3::new(0.8660254037844387, 0., -0.49999999999999994));
        assert_eq!(h.y, Vec3::new(0.49999999999999994, 0., 0.8660254037844387));
        assert_eq!(h.k, 0.3819718634205488);
        assert_eq!(h.angle, 15.707963267948966);
        assert_eq!(h.pts().len(), 81);
        // reverse + leftHand + 45
        let p = HelixParams { pitch: 2.0, revolutions: 3.0, start_angle: 45.0, left_hand: true, reverse: true, ..Default::default() };
        let h = helix(&top(), c, 5.0, &p).unwrap();
        assert_eq!(h.a, -Vec3::Y);
        assert!(close(h.x, [0.7071067811865476, 0., -0.7071067811865475]));
        assert!(close(h.y, [-0.7071067811865475, 0., -0.7071067811865476]));
        assert!(close(h.pts()[1], [5.777851165098012, -0.0625, -8.157348061512725]));
        // heightPitch
        let p = HelixParams { defined_by: DefinedBy::HeightPitch, pitch: 2.0, height: 5.0, ..Default::default() };
        let h = helix(&top(), c, 5.0, &p).unwrap();
        assert_eq!(h.angle, 15.707963267948966);
        assert_eq!(h.k, 0.3183098861837907);
        // sample counts
        for (revs, n) in [(0.3, 11), (10.0, 321), (1e-7, 3), (0.01, 3)] {
            let p = HelixParams { pitch: 2.0, revolutions: revs, ..Default::default() };
            assert_eq!(helix(&top(), c, 5.0, &p).unwrap().pts().len(), n);
        }
    }

    #[test]
    fn invalid_values() {
        let c = Vec3::ZERO;
        for p in [
            HelixParams { pitch: 0.0, revolutions: 3.0, ..Default::default() },
            HelixParams { pitch: -2.0, revolutions: 3.0, ..Default::default() },
            HelixParams { pitch: 2.0, revolutions: -1.0, ..Default::default() },
            HelixParams { defined_by: DefinedBy::HeightRevs, revolutions: 0.0, height: 6.0, ..Default::default() },
            HelixParams { defined_by: DefinedBy::HeightPitch, pitch: 2.0, height: -5.0, ..Default::default() },
        ] {
            assert_eq!(helix(&top(), c, 5.0, &p), Err("invalidValue".to_string()));
        }
        assert_eq!(DefinedBy::parse("bogus"), DefinedBy::PitchRevs);
    }

    #[test]
    fn derivs_and_spline() {
        let h = helix(&top(), Vec3::ZERO, 2.0, &HelixParams { pitch: 1.5, revolutions: 2.0, ..Default::default() }).unwrap();
        let e = 1e-6;
        for t in [0.3, 2.0, 7.5] {
            let d = h.derivs(t);
            let fd = (h.eval(t + e) - h.eval(t - e)) / (2.0 * e);
            assert!((d[1] - fd).len() < 1e-8);
        }
        let c = h.to_curve(16).unwrap();
        for i in 0..=40 {
            let t = h.angle * i as f64 / 40.0;
            assert!((c.eval(t) - h.eval(t)).len() < 1e-3);
        }
        assert!((h.height() - 3.0).abs() < 1e-15);
    }
}
