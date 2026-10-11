//! Lofts between profiles (stations, guides, centre line, smoothing).
//!
//! Entry point [`loft`]: profiles (closed chains of sketch pieces, or open
//! chains for a surface loft) -> a solid (caps `C0` at the first profile,
//! `C1` at the last) or a sheet (`surface`). Reconstruction from oracle
//! probes (original: f2831 at ck-ops/src/loft.rs:298 and the stations /
//! smooth / guides / centre helpers):
//!
//! * face `S.<entity>` per piece of the FIRST profile (pieces cut by the
//!   station matching share the name and get `~k`), then `C0`, `C1`;
//! * edges: the rails first (one per piece start vertex, faces
//!   `piece j | piece j-1`; an open chain has one more at its far end), then
//!   all `C0` edges, then all `C1` edges, each in piece order; rails run
//!   `C0 -> C1`, cap edges follow the travel direction of the pieces;
//! * rails are straight for two profiles (side faces of straight pieces are
//!   planes when flat) and natural cubic splines through the matching points
//!   for more ([`smooth`]);
//! * profile pieces become rational B-splines (arcs as quarter-turn
//!   rational quadratics); the side faces interpolate the homogeneous
//!   control points of the stations, so the profile curves are exact
//!   iso-curves of the faces.
//!
//! Errors (codes): `loftProfiles`, `loftProfileOpen`, `loftProfileNotPlanar`,
//! `loftFailed`.

pub mod centre;
pub mod guides;
pub mod smooth;
pub mod stations;

use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, VertexId};
use ck_topo::make::plane_line_pc;
use stations::{geom_of, match_chains, Chain, Geom3, Piece, Stations};

/// One piece of an input profile: the sketch entity curve trimmed to
/// `[t0, t1]`, travelled backwards when `rev`.
#[derive(Debug, Clone)]
pub struct SegIn {
    pub tag: String,
    pub curve: Curve,
    pub t0: f64,
    pub t1: f64,
    pub rev: bool,
}

/// A profile: chain of pieces in travel order on the sketch `frame`.
#[derive(Debug, Clone)]
pub struct ProfileIn {
    pub frame: Frame,
    pub closed: bool,
    pub segs: Vec<SegIn>,
}

#[derive(Debug, Clone, Default)]
pub struct LoftOpts {
    pub surface: bool,
}

#[derive(Debug, Clone)]
pub struct LoftOut {
    pub body: Body,
    /// Start point of every matched profile (the `starts` marker).
    pub starts: Vec<Vec3>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct LoftError {
    pub code: String,
    pub detail: Option<String>,
}

impl LoftError {
    pub fn new(code: &str) -> LoftError {
        LoftError { code: code.into(), detail: None }
    }
}

pub const PROFILES: &str = "loftProfiles";
pub const PROFILE_OPEN: &str = "loftProfileOpen";
pub const PROFILE_NOT_PLANAR: &str = "loftProfileNotPlanar";
pub const FAILED: &str = "loftFailed";

const TOL: f64 = 1e-9;

fn chain_of(p: &ProfileIn) -> Result<Chain, LoftError> {
    let mut pieces = vec![];
    for s in &p.segs {
        let g = geom_of(&s.curve, s.t0, s.t1, s.rev).ok_or_else(|| LoftError::new(FAILED))?;
        pieces.push(Piece { tag: s.tag.clone(), g });
    }
    if pieces.is_empty() {
        return Err(LoftError::new(FAILED));
    }
    Ok(Chain { closed: p.closed, pieces })
}

/// Is the closed sample polygon of `c` flat (within `tol`)?
fn chain_planar(c: &Chain, tol: f64) -> bool {
    let n = c.normal();
    let Some(n) = n.try_normalize() else { return true };
    let o = c.pieces[0].g.start();
    c.pieces.iter().all(|p| (0..=4).all(|k| (p.g.at(k as f64 / 4.0) - o).dot(n).abs() <= tol))
}

/// Display samples of a profile (the `loops` marker): per piece 16 uniform
/// parameter steps on its rational B-spline (so a circle shows the
/// quarter-turn rational parameterisation: 21.6 degrees at 1/16), and `f`,
/// the arc-length fraction of the piece start plus `k/16` of its length.
pub fn marker_samples(p: &ProfileIn) -> Vec<(Vec3, f64)> {
    let pieces: Vec<Geom3> = p.segs.iter().filter_map(|s| geom_of(&s.curve, s.t0, s.t1, s.rev)).collect();
    let lens: Vec<f64> = pieces.iter().map(|g| g.length()).collect();
    let total: f64 = lens.iter().sum();
    let mut out = vec![];
    let mut before = 0.0;
    for (i, g) in pieces.iter().enumerate() {
        let c = g.to_nurbs();
        let (a, b) = c.domain();
        for k in 0..=16 {
            if i > 0 && k == 0 {
                continue;
            }
            let pt = if k == 16 { g.end() } else if k == 0 { g.start() } else { c.eval(a + (b - a) * k as f64 / 16.0) };
            let f = if total > 0.0 { (before + lens[i] * k as f64 / 16.0) / total } else { 0.0 };
            out.push((pt, f));
        }
        before += lens[i];
    }
    out
}

/// Loft through `profiles` (see the module header).
pub fn loft(feature: &str, profiles: &[ProfileIn], opts: &LoftOpts) -> Result<LoftOut, LoftError> {
    if profiles.len() < 2 {
        return Err(LoftError::new(PROFILES));
    }
    let chains: Vec<Chain> = profiles.iter().map(chain_of).collect::<Result<_, _>>()?;
    // different planes: the profile frames must not all coincide
    let f0 = &profiles[0].frame;
    let same_plane = profiles.iter().all(|p| {
        p.frame.z.cross(f0.z).len() < 1e-9 && (p.frame.o - f0.o).dot(f0.z).abs() < 1e-9
    });
    if same_plane {
        return Err(LoftError::new(PROFILES));
    }
    let closed = chains[0].closed;
    if chains.iter().any(|c| c.closed != closed) || (!opts.surface && !closed) {
        return Err(LoftError::new(PROFILE_OPEN));
    }
    if !opts.surface {
        let size = chains.iter().map(|c| c.pieces.iter().map(|p| p.g.length()).sum::<f64>()).fold(0.0, f64::max);
        if chains.iter().any(|c| !chain_planar(c, 1e-6 * size.max(1.0))) {
            return Err(LoftError::new(PROFILE_NOT_PLANAR));
        }
    }
    let st = match_chains(chains);
    let starts = st.chains.iter().map(|c| c.pieces[0].g.start()).collect();
    let body = build(feature, &st, profiles, opts.surface)?;
    Ok(LoftOut { body, starts })
}

// ------------------------------------------------------------------ faces

/// Compatible rational B-splines of one piece over all stations: same
/// degree and knot vector, weights present on all or none.
fn compat(mut cs: Vec<BSplineCurve>) -> Vec<BSplineCurve> {
    let p = cs.iter().map(|c| c.degree).max().unwrap();
    for c in cs.iter_mut() {
        if c.degree < p {
            *c = c.elevate(p - c.degree);
        }
    }
    // union of knots (max multiplicity)
    let mut all: Vec<(f64, usize)> = vec![];
    for c in &cs {
        for k in ck_geom::nurbs::basis::distinct_knots(&c.knots) {
            let m = ck_geom::nurbs::basis::multiplicity(&c.knots, k, 0.0);
            match all.iter_mut().find(|(x, _)| (*x - k).abs() < 1e-12) {
                Some(e) => e.1 = e.1.max(m),
                None => all.push((k, m)),
            }
        }
    }
    for c in cs.iter_mut() {
        for &(k, m) in &all {
            let have = ck_geom::nurbs::basis::multiplicity(&c.knots, k, 1e-12);
            if have < m {
                *c = c.insert_knot(k, m - have);
            }
        }
    }
    if cs.iter().any(|c| c.weights.is_some()) {
        for c in cs.iter_mut() {
            if c.weights.is_none() {
                c.weights = Some(vec![1.0; c.ctrl.len()]);
            }
        }
    }
    cs
}

struct Side {
    surface: Surface,
    /// Station curves (compatible NURBS), index = station.
    cs: Vec<BSplineCurve>,
    /// Plane frame when the face is a flat quad of straight pieces.
    plane: Option<Frame>,
}

fn side(st: &Stations, j: usize) -> Result<Side, LoftError> {
    let k = st.chains.len();
    let pieces: Vec<&Geom3> = st.chains.iter().map(|c| &c.pieces[j].g).collect();
    let cs = compat(pieces.iter().map(|g| g.to_nurbs()).collect());
    // flat quad of two straight pieces
    if k == 2 && pieces.iter().all(|g| g.is_line()) {
        let (a0, b0, a1, b1) = (pieces[0].start(), pieces[0].end(), pieces[1].start(), pieces[1].end());
        let x = (b0 - a0).try_normalize();
        let z = (b0 - a0).cross(a1 - a0).try_normalize();
        if let (Some(x), Some(z)) = (x, z) {
            let flat = (b1 - a0).dot(z).abs();
            let size = a0.dist(b0).max(a0.dist(a1)).max(1.0);
            if flat <= 1e-9 * size {
                let frame = Frame { o: a0, x, y: z.cross(x), z };
                return Ok(Side { surface: Surface::Plane { frame }, cs, plane: Some(frame) });
            }
        }
    }
    let nu = cs[0].ctrl.len();
    let mut net: Vec<Vec<[f64; 4]>> = vec![]; // [i][v ctrl]
    let mut vdeg = 1;
    let mut vknots = vec![];
    for i in 0..nu {
        let pts: Vec<[f64; 4]> = cs
            .iter()
            .map(|c| {
                let w = c.weight(i);
                let p = c.ctrl[i];
                [p.x * w, p.y * w, p.z * w, w]
            })
            .collect();
        let (d, kn, q) = smooth::through(&st.ts, &pts).map_err(|_| LoftError::new(FAILED))?;
        vdeg = d;
        vknots = kn;
        net.push(q);
    }
    let nv = net[0].len();
    let mut ctrl = Vec::with_capacity(nu * nv);
    let mut wts = Vec::with_capacity(nu * nv);
    for col in &net {
        for q in col {
            let w = q[3];
            ctrl.push(Vec3::new(q[0] / w, q[1] / w, q[2] / w));
            wts.push(w);
        }
    }
    let rational = cs.iter().any(|c| c.weights.is_some()) && wts.iter().any(|w| (w - 1.0).abs() > 1e-14);
    let surf = BSplineSurface::new(
        cs[0].degree,
        vdeg,
        cs[0].knots.clone(),
        vknots,
        nu,
        nv,
        ctrl,
        if rational { Some(wts) } else { None },
    )
    .map_err(|_| LoftError::new(FAILED))?;
    Ok(Side { surface: Surface::BSpline(Box::new(surf)), cs, plane: None })
}

/// Rail through the matching vertices `pts` of the stations.
fn rail(ts: &[f64], pts: &[Vec3]) -> Result<(Curve, f64, f64), LoftError> {
    if ts.len() == 2 {
        let d = pts[0].dist(pts[1]);
        return Ok((Curve::line_through(pts[0], pts[1]), 0.0, d));
    }
    let q: Vec<[f64; 3]> = pts.iter().map(|p| [p.x, p.y, p.z]).collect();
    let (deg, kn, c) = smooth::through(ts, &q).map_err(|_| LoftError::new(FAILED))?;
    let cur = BSplineCurve::new(deg, kn, c.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect())
        .map_err(|_| LoftError::new(FAILED))?;
    let (a, b) = cur.domain();
    Ok((Curve::BSpline(cur), a, b))
}

fn build(feature: &str, st: &Stations, profiles: &[ProfileIn], surface: bool) -> Result<Body, LoftError> {
    let k = st.chains.len();
    let n = st.tags.len();
    let closed = st.closed;
    let nv = if closed { n } else { n + 1 };
    let total = *st.ts.last().unwrap();
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    // sides
    let sides: Vec<Side> = (0..n).map(|j| side(st, j)).collect::<Result<_, _>>()?;
    let faces: Vec<FaceId> = (0..n)
        .map(|j| b.add_face(shell, sides[j].surface.clone(), true, FaceName::side(feature, &st.tags[j])))
        .collect();
    let caps = if surface {
        None
    } else {
        let fr0 = profiles[0].frame;
        let fr1 = profiles[profiles.len() - 1].frame;
        let f0 = b.add_face(shell, Surface::Plane { frame: fr0 }, true, FaceName::cap(feature, 0, None));
        let f1 = b.add_face(shell, Surface::Plane { frame: fr1 }, true, FaceName::cap(feature, 1, None));
        Some((f0, f1, fr0, fr1))
    };
    // vertices: [station][piece start (+ far end of an open chain)]
    let vp = |s: usize, i: usize| -> Vec3 {
        let c = &st.chains[s];
        if i < n {
            c.pieces[i].g.start()
        } else {
            c.pieces[n - 1].g.end()
        }
    };
    let v0: Vec<VertexId> = (0..nv).map(|i| b.add_vertex(vp(0, i))).collect();
    let v1: Vec<VertexId> = (0..nv).map(|i| b.add_vertex(vp(k - 1, i))).collect();
    // rails
    let mut rails: Vec<(EdgeId, bool)> = vec![];
    for i in 0..nv {
        let pts: Vec<Vec3> = (0..k).map(|s| vp(s, i)).collect();
        let (c, t0, t1) = rail(&st.ts, &pts)?;
        rails.push((b.add_edge(c.clone(), t0, t1, v0[i], v1[i]), matches!(c, Curve::Line { .. })));
    }
    // cap edges
    let cap_edge = |b: &mut Body, s: usize, j: usize, va: VertexId, vb: VertexId| -> (EdgeId, Curve, f64, f64) {
        let c = &sides[j].cs[s];
        let g = &st.chains[s].pieces[j].g;
        // a face with any curved station piece keeps ALL its station edges as
        // (elevated) B-splines, lines included (oracle: `pts 4`, no `line`)
        let all_lines = (0..k).all(|q| st.chains[q].pieces[j].g.is_line());
        let (curve, t0, t1) = if all_lines {
            let d = g.start().dist(g.end());
            (Curve::line_through(g.start(), g.end()), 0.0, d)
        } else {
            (Curve::BSpline(c.clone()), 0.0, 1.0)
        };
        (b.add_edge(curve.clone(), t0, t1, va, vb), curve, t0, t1)
    };
    let mut e0 = vec![];
    for j in 0..n {
        e0.push(cap_edge(&mut b, 0, j, v0[j], v0[(j + 1) % nv.max(1)]));
    }
    let mut e1 = vec![];
    for j in 0..n {
        e1.push(cap_edge(&mut b, k - 1, j, v1[j], v1[(j + 1) % nv.max(1)]));
    }
    // side loops
    for j in 0..n {
        let sd = &sides[j];
        let f = faces[j];
        let jn = if closed { (j + 1) % n } else { j + 1 };
        let lp = b.add_loop(f);
        let len0 = e0[j].3 - e0[j].2;
        let len1 = e1[j].3 - e1[j].2;
        let (pc_c0, pc_c1, pc_r0, pc_r1): (Curve2, Curve2, Curve2, Curve2);
        if let Some(fr) = sd.plane {
            let line_pc = |c: &Curve| match c {
                Curve::Line { o, d } => plane_line_pc(&fr, *o, *d),
                _ => unreachable!(),
            };
            pc_c0 = line_pc(&e0[j].1);
            pc_c1 = line_pc(&e1[j].1);
            let rc = |i: usize| -> Curve2 {
                let (e, _) = rails[i];
                match &b.edge(e).curve {
                    Curve::Line { o, d } => plane_line_pc(&fr, *o, *d),
                    _ => unreachable!(),
                }
            };
            pc_r0 = rc(j);
            pc_r1 = rc(jn);
        } else {
            let (vlo, vhi) = (0.0, total);
            let du0 = if matches!(e0[j].1, Curve::Line { .. }) { 1.0 / len0 } else { 1.0 };
            let du1 = if matches!(e1[j].1, Curve::Line { .. }) { 1.0 / len1 } else { 1.0 };
            pc_c0 = Curve2::Line { o: Vec2::new(0.0, vlo), d: Vec2::new(du0, 0.0) };
            pc_c1 = Curve2::Line { o: Vec2::new(0.0, vhi), d: Vec2::new(du1, 0.0) };
            let rpc = |u: f64, i: usize, b: &Body| -> Curve2 {
                let (e, is_line) = rails[i];
                let ed = b.edge(e);
                let dv = if is_line { total / (ed.t1 - ed.t0) } else { 1.0 };
                Curve2::Line { o: Vec2::new(u, vlo), d: Vec2::new(0.0, dv) }
            };
            pc_r0 = rpc(0.0, j, &b);
            pc_r1 = rpc(1.0, jn, &b);
        }
        b.add_coedge(lp, e0[j].0, false, Some(pc_c0));
        b.add_coedge(lp, rails[jn].0, false, Some(pc_r1));
        b.add_coedge(lp, e1[j].0, true, Some(pc_c1));
        b.add_coedge(lp, rails[j].0, true, Some(pc_r0));
    }
    // caps
    if let Some((f0, f1, fr0, fr1)) = caps {
        let pc = |fr: &Frame, c: &Curve| match c {
            // arcs are rational B-splines: the plane projection is affine, so the pcurve is exact
            Curve::BSpline(bs) => Some(Curve2::BSpline(bs.map(|p| Vec2::new((p - fr.o).dot(fr.x), (p - fr.o).dot(fr.y))))),
            _ => ck_topo::make::plane_pcurve(fr, c),
        };
        let l0 = b.add_loop(f0);
        for j in (0..n).rev() {
            b.add_coedge(l0, e0[j].0, true, pc(&fr0, &e0[j].1));
        }
        let l1 = b.add_loop(f1);
        for j in 0..n {
            b.add_coedge(l1, e1[j].0, false, pc(&fr1, &e1[j].1));
        }
        for (f, l) in [(f0, l0), (f1, l1)] {
            let a = ck_topo::orient::loop_uv_area(&b, l);
            b.faces[f.idx()].sense = a >= 0.0;
        }
    }
    b.number_split_faces();
    if !surface {
        ck_topo::orient::orient_body(&mut b);
    }
    let _ = TOL;
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    fn frame_at(y: f64) -> Frame {
        Frame { o: Vec3::new(0.0, y, 0.0), x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::new(0.0, 1.0, 0.0) }
    }
    fn circle_profile(y: f64, cx: f64, r: f64) -> ProfileIn {
        let mut fr = frame_at(y);
        fr.o = fr.o + fr.x * cx;
        ProfileIn {
            frame: fr,
            closed: true,
            segs: vec![SegIn { tag: "e1".into(), curve: Curve::Circle { frame: fr, r }, t0: 0.0, t1: TAU, rev: false }],
        }
    }

    /// Oracle (circle r 8 -> circle r 4 shifted by 2, h 30): oblique cone
    /// frustum, volume pi h/3 (R^2 + R r + r^2) = 3518.583772020568.
    #[test]
    fn oblique_frustum_volume() {
        let out = loft("F4", &[circle_profile(0.0, 0.0, 8.0), circle_profile(30.0, 2.0, 4.0)], &LoftOpts::default()).unwrap();
        let m = ck_topo::mass::mass_props(&out.body);
        let want = std::f64::consts::PI * 10.0 * 112.0;
        assert!((m.volume - want).abs() < 1e-6 * want, "{} vs {}", m.volume, want);
    }
}
