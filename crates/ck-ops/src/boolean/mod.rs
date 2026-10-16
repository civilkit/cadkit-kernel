//! Solid booleans: union / cut / intersect (ck-ops/src/boolean/*).
//!
//! Pipeline: [`imprint`] (face-pair SSI, trim, shared edge arrangement) →
//! [`assemble::fragments`] (loop walk per face) → [`classify`] (fragment
//! In / Out / On-same / On-opposite vs the other solid) →
//! [`assemble::select`] (keep / flip / merge) → [`assemble::build`]
//! (new Body, exact pcurves, lumps). Entry point: [`apply::boolean`].
//!
//! Oracle evidence (tools/q.mjs probes, top-plane rect (0,0)-(40,20)
//! extruded 10 = box `F2`):
//! - cut through by a circle extrude `F4` (throughBoth): faces
//!   `F2.S.e1..e4, F2.C0, F2.C1, F4.S.e1` (tool faces keep the TOOL
//!   feature's ids, cylinder sense flipped), edges `C0|F4.S.e1`, `C1|…`
//!   are circles with n = tool axis.
//! - blind pocket from C0: `F2.C1, F2.S.e1..e4, F2.C0, F4.C1, F4.S.e1`
//!   (pocket floor = tool's C1, normal flipped to -y); circle starts at
//!   c + r·x_sketch.
//! - slot cut splitting the box: split faces get `~k` (`F2.C0~1`,
//!   `F2.S.e1~2`, also on side faces; meta.entity stays `e1`), one lump
//!   per piece, lump 0 = the piece holding the first output face.
//! - union with an overlapping box: coplanar caps merge into ONE face
//!   named after body A's face, emitted after all other faces.
//! - a cut that misses: error `cutNoEffect` (feature warning), body
//!   unchanged (original edge order).
//! Error codes in kernel.strings: `booleanFailed`, `cutNoEffect`,
//! `nothingToCut`, `cancelled`.

/// Diagnostics for failed booleans: `CK_BOOL_DBG=1` prints the failing stage
/// to stderr (native builds only; the wasm build is silent).
macro_rules! bool_dbg {
    ($($a:tt)*) => {
        #[cfg(not(target_arch = "wasm32"))]
        if std::env::var("CK_BOOL_DBG").is_ok() {
            eprintln!("boolean: {}", format!($($a)*));
        }
    };
}
pub(crate) use bool_dbg;

thread_local! {
    static ROBUST: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Robust mode: set only for the last-resort attempts of a boolean whose ordinary attempts all failed to
/// assemble. It switches on three sliver-tolerant steps that would change (and slow) results that already work:
/// splitting two cut curves of one face pair where they cross each other (imprint), merging edge pieces that stay
/// within 100 tol of each other between the same two vertices (imprint), and classifying a fragment sampled
/// within a sliver's width of the other solid by a vote of all rays (classify).
pub(crate) fn robust() -> bool {
    ROBUST.with(|r| r.get())
}

pub(crate) struct RobustGuard(bool);
impl RobustGuard {
    pub(crate) fn new() -> RobustGuard {
        RobustGuard(ROBUST.with(|r| r.replace(true)))
    }
}
impl Drop for RobustGuard {
    fn drop(&mut self) {
        ROBUST.with(|r| r.set(self.0));
    }
}

pub mod apply;
pub mod assemble;
pub mod classify;
pub mod contact;
pub mod fuzzy;
pub mod imprint;
pub mod sanity;

/// Boolean operation (A op B).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Op {
    Union,
    /// A − B
    Cut,
    Intersect,
}

pub use apply::{boolean, BoolOut};

#[cfg(test)]
pub(crate) mod tests {
    use ck_geom::{Frame, Vec2, Vec3};
    use ck_topo::body::Body;
    use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

    /// Oracle std plane "top": x = X, y = -Z, normal +Y.
    pub fn top() -> Frame {
        Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
    }

    pub fn rect_tags(feature: &str, tags: [&str; 4], x0: f64, y0: f64, x1: f64, y1: f64, s0: f64, s1: f64) -> Body {
        let p = [Vec2::new(x0, y0), Vec2::new(x1, y0), Vec2::new(x1, y1), Vec2::new(x0, y1)];
        let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(tags[i], p[i], p[(i + 1) % 4])).collect());
        let prof = Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
        extrude(feature, &prof, ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
    }

    /// Top-plane rectangle extruded from height s0 to s1 (entities e1..e4).
    pub fn box_body(feature: &str, x0: f64, y0: f64, x1: f64, y1: f64, s0: f64, s1: f64) -> Body {
        rect_tags(feature, ["e1", "e2", "e3", "e4"], x0, y0, x1, y1, s0, s1)
    }

    /// Circle `e1` on `frame` extruded from s0 to s1.
    pub fn cyl_on(feature: &str, frame: Frame, cx: f64, cy: f64, r: f64, s0: f64, s1: f64) -> Body {
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(cx, cy), r)]);
        let prof = Profile::new(frame, vec![ProfileRegion { outer: lp, holes: vec![] }]);
        extrude(feature, &prof, ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
    }

    /// Top-plane circle `e1` extruded from s0 to s1.
    pub fn cyl_body(feature: &str, cx: f64, cy: f64, r: f64, s0: f64, s1: f64) -> Body {
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(cx, cy), r)]);
        let prof = Profile::new(top(), vec![ProfileRegion { outer: lp, holes: vec![] }]);
        extrude(feature, &prof, ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
    }
}

#[cfg(test)]
mod redrill;
