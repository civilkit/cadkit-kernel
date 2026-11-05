//! Shared blend model: the types the analytic builder (`build`, `analytic`, `rounds`) and the free-form builder
//! (`freeform`) both speak.
//!
//! - what a blend is asked to do: [`Spec`] (fillet / chamfer, [`FilletProfile`], [`ChamferLeg`]) and a [`Pick`];
//! - a chain: the selected edges in order ([`ChainPos`], [`Link`]) and what a tool does where a chain ends
//!   ([`EndConditions`]);
//! - a station along a chain ([`Station`]: contact points, ball centre, tangent) and the cross-section built from it
//!   ([`Sec`], `NV` arc samples + 3 leg points).

use ck_geom::Vec3;
use ck_topo::body::{EdgeId, FaceId};

/// What one blend cuts: a rolling-ball fillet of radius `r` or an
/// equal-distance chamfer of leg `d`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Spec {
    Fillet(f64, FilletProfile),
    Chamfer(f64, ChamferLeg),
}

/// Second leg of a chamfer: a distance on the second face, or the angle (radians) between the
/// chamfer face and the first face.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChamferLeg {
    Dist(f64),
    Angle(f64),
    /// [`ChamferLeg::Angle`] with the pick's faces listed the other way round: the angle is to
    /// the second face, the distance on it.
    AngleRev(f64),
}

/// Section of a fillet (`fillet.profile`): a circular arc, a conic (rational
/// quadratic through the circular fillet's contact points, `rho` in (0, 1)),
/// a circular arc of the given CHORD width, or the curvature-continuous
/// spiral - arc - spiral of `blend::g2` (`shape` in [0.1, 1]).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FilletProfile {
    Circular,
    Conic(f64),
    Chord,
    Curvature(f64),
}

impl Spec {
    pub(super) fn size(self) -> f64 {
        match self {
            Spec::Fillet(r, _) => r,
            Spec::Chamfer(a, ChamferLeg::Dist(b)) => a.max(b),
            Spec::Chamfer(a, ChamferLeg::Angle(_) | ChamferLeg::AngleRev(_)) => a,
        }
    }
}

/// Per-selection end information gathered on the original body.
#[derive(Default)]
pub(super) struct EndConditions {
    /// vertices where a concave tool runs on to (`true`: material behind the end face)
    pub(super) ext: Vec<(Vec3, bool)>,
    /// end vertices whose end face leans toward the material: a convex tool stops flush there
    pub(super) flush: Vec<Vec3>,
    /// `(vertex, direction away from it along this edge, along the partner edge)`: two selected convex
    /// edges that meet at a reflex corner of their common face. Both tools run on past the vertex and
    /// each keeps its own side of the bisector plane.
    pub(super) mitre: Vec<(Vec3, Vec3, Vec3)>,
    /// overhang-corner vertices (`super::overhang::detect`): no stripe tool may run on past them —
    /// the corner is rebuilt from the zone box afterwards, and a tool piercing through would cut
    /// material outside the zone that nothing repairs.
    pub(super) patch: Vec<Vec3>,
}

/// An edge pick of a blend feature: the two faces named by the feature and a point on the edge.
pub(super) type Pick = (String, String, Vec3);

/// Position of a selected edge in its chain: (chain number, signed step from the chain's first selected edge).
pub(super) type ChainPos = (usize, i64);

/// Arc samples per cross-section.
pub(super) const NV: usize = 9;
/// Section track count: arc samples + B2, Q, A2.
pub(super) const NT: usize = NV + 3;

/// One edge of a chain as the rolling ball traverses it.
#[derive(Clone)]
pub(super) struct Link {
    pub(super) e: EdgeId,
    /// Traversed v0 -> v1.
    pub(super) fwd: bool,
    pub(super) f1: FaceId,
    pub(super) f2: FaceId,
}

/// The rolling ball at one point of a chain.
#[derive(Clone)]
pub(super) struct Station {
    /// point on the edge
    pub(super) p: Vec3,
    /// contact point on face 1
    pub(super) a: Vec3,
    /// contact point on face 2
    pub(super) b: Vec3,
    /// ball centre
    pub(super) c: Vec3,
    /// Unit tangent along the chain.
    pub(super) t: Vec3,
}

/// One cross-section: arc samples, then B2, Q, A2.
#[derive(Clone)]
pub(super) struct Sec {
    pub(super) pts: [Vec3; NT],
    /// d/dv of the arc (v in [0, 1]) at its two ends: the surface stays
    /// tangent to the faces along the contact curves.
    pub(super) d0: Vec3,
    pub(super) d1: Vec3,
}
