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

use ck_geom::Vec2;
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
