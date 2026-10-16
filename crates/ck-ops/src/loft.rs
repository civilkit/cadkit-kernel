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
use stations::{geom_of, Chain, Geom3, Piece, Stations};

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
    /// Guide curves (dense polylines): see [`guides`].
    pub guides: Vec<guides::GuideIn>,
    /// `startNormal` / `endNormal`: the loft leaves the first / last profile
    /// along its normal with this tension (unit speed times the tension).
    pub start_normal: Option<f64>,
    pub end_normal: Option<f64>,
    /// `connectors[i]`: arc-length fraction at which profile `i` starts.
    pub connectors: Vec<Option<f64>>,
}

/// End tangents of the rails (derivative with respect to the loft parameter).
#[derive(Debug, Clone, Copy, Default)]
pub struct Ends {
    pub d0: Option<Vec3>,
    pub d1: Option<Vec3>,
}

impl Ends {
    fn arr(d: Option<Vec3>) -> Option<[f64; 3]> {
        d.map(|v| [v.x, v.y, v.z])
    }
    fn any(&self) -> bool {
        self.d0.is_some() || self.d1.is_some()
    }
}

#[derive(Debug, Clone)]
pub struct LoftOut {
    pub body: Body,
    /// Start point of every matched profile (the `starts` marker).
    pub starts: Vec<Vec3>,
    /// The end tangents used (`d0`, `d1`) and the first / last station centroid.
    pub ends: Ends,
    pub end_centroids: [Vec3; 2],
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
/// A section whose polygon crosses itself and is small against the loft length.
pub const SECTION_TOO_SMALL: &str = "loftSectionTooSmall";
pub const PROFILE_SELF_INTERSECT: &str = "loftProfileSelfIntersect";
/// An end section whose size is below this fraction of the loft length becomes a point section.
pub const SNAP_REL: f64 = 1e-3;

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
    let nurbs: Vec<BSplineCurve> = pieces.iter().map(|g| g.to_nurbs()).collect();
    let lens: Vec<f64> = nurbs.iter().map(stations::nurbs_length).collect();
    let total: f64 = lens.iter().sum();
    let mut out = vec![];
    let mut before = 0.0;
    for (i, g) in pieces.iter().enumerate() {
        let c = &nurbs[i];
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

/// A profile without pieces is a point (apex) end: the point is `frame.o`.
pub fn is_point_profile(p: &ProfileIn) -> bool {
    p.segs.is_empty()
}

/// The station chains; a point profile becomes the chain of its neighbour
/// shrunk to the apex (same pieces, degree, knots and weights).
fn chains_of(profiles: &[ProfileIn], opts: &LoftOpts) -> Result<Vec<Chain>, LoftError> {
    let k = profiles.len();
    let mut apex: Vec<Option<Vec3>> = profiles.iter().map(|p| is_point_profile(p).then_some(p.frame.o)).collect();
    let fail = |d: &str| LoftError { code: FAILED.into(), detail: Some(d.into()) };
    let any_point = apex.iter().any(|a| a.is_some());
    if any_point {
        if apex.iter().enumerate().any(|(i, a)| a.is_some() && i != 0 && i != k - 1) {
            return Err(fail("only the first and the last section can be a point"));
        }
        if apex.iter().all(|a| a.is_some()) {
            return Err(LoftError::new(PROFILES));
        }
        if opts.surface {
            return Err(fail("a point section needs a solid loft"));
        }
        if !opts.guides.is_empty() {
            return Err(fail("guide curves are not supported with a point section"));
        }
    }
    let mut chains: Vec<Option<Chain>> = vec![];
    for (p, a) in profiles.iter().zip(&apex) {
        chains.push(if a.is_some() { None } else { Some(chain_of(p)?) });
    }
    // a tiny end section (below SNAP_REL of the loft length) is a point in
    // practice: its cap loop would be degenerate. Snap it to its centroid.
    if !opts.surface && opts.guides.is_empty() && k >= 2 {
        let cen = |i: usize| chains[i].as_ref().map(|c| c.centroid());
        let (c0, c1) = (cen(0).or(apex[0]), cen(k - 1).or(apex[k - 1]));
        if let (Some(c0), Some(c1)) = (c0, c1) {
            let length = c0.dist(c1);
            for i in [0, k - 1] {
                let Some(c) = &chains[i] else { continue };
                let pts: Vec<Vec3> = marker_samples_of(c);
                let (mut lo, mut hi) = (pts[0], pts[0]);
                for q in &pts {
                    lo = Vec3::new(lo.x.min(q.x), lo.y.min(q.y), lo.z.min(q.z));
                    hi = Vec3::new(hi.x.max(q.x), hi.y.max(q.y), hi.z.max(q.z));
                }
                if lo.dist(hi) < SNAP_REL * length {
                    let others = (0..k).filter(|&j| j != i).all(|j| apex[j].is_some());
                    if !others {
                        apex[i] = Some(c.centroid());
                    }
                }
            }
        }
    }
    for i in 0..k {
        if apex[i].is_some() {
            chains[i] = None;
        }
    }
    let n = chains.iter().flatten().map(|c| c.pieces.len()).max().unwrap_or(1).max(2);
    let mut out = vec![];
    for i in 0..k {
        out.push(match (&chains[i], apex[i]) {
            (Some(c), _) => c.clone(),
            (None, a) => {
                let nb = if i == 0 { 1 } else { k - 2 };
                let mut c = match &chains[nb] {
                    Some(c) => c.clone(),
                    None => chain_of(&profiles[nb])?,
                };
                c.split_to(n);
                c.collapsed(a.unwrap_or(profiles[i].frame.o))
            }
        });
    }
    if !opts.surface {
        let len = out[0].centroid().dist(out[k - 1].centroid());
        for c in out.iter().filter(|c| !c.is_point()) {
            if let Some(size) = self_intersection(c) {
                let code = if size < 1e-2 * len { SECTION_TOO_SMALL } else { PROFILE_SELF_INTERSECT };
                return Err(LoftError { code: code.into(), detail: Some("a section crosses itself; make it a simple closed curve (a section much smaller than the loft can be a point section: use a sketch with one point)".into()) });
            }
        }
    }
    Ok(out)
}

fn marker_samples_of(c: &Chain) -> Vec<Vec3> {
    let mut v = vec![];
    for p in &c.pieces {
        for k in 0..=8 {
            v.push(p.g.at(k as f64 / 8.0));
        }
    }
    v
}

/// Does the closed sample polygon of `c` cross itself? Returns the section size
/// (bounding-box diagonal) when it does.
fn self_intersection(c: &Chain) -> Option<f64> {
    let pts = marker_samples_of(c);
    let n = c.normal().try_normalize()?;
    let (ax, ay) = {
        let a = if n.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        let x = n.cross(a).try_normalize()?;
        (x, n.cross(x))
    };
    // drop the duplicated piece joins
    let mut p2: Vec<(f64, f64)> = vec![];
    for (i, q) in pts.iter().enumerate() {
        if i % 9 == 8 && i + 1 < pts.len() {
            continue;
        }
        p2.push((q.dot(ax), q.dot(ay)));
    }
    p2.pop();
    let m = p2.len();
    let (mut lo, mut hi) = ((f64::INFINITY, f64::INFINITY), (f64::NEG_INFINITY, f64::NEG_INFINITY));
    for q in &p2 {
        lo = (lo.0.min(q.0), lo.1.min(q.1));
        hi = (hi.0.max(q.0), hi.1.max(q.1));
    }
    let size = ((hi.0 - lo.0).powi(2) + (hi.1 - lo.1).powi(2)).sqrt();
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let eps = 1e-9 * size * size;
    for i in 0..m {
        let (a, b) = (p2[i], p2[(i + 1) % m]);
        for j in i + 2..m {
            if i == 0 && j == m - 1 {
                continue;
            }
            let (c0, d) = (p2[j], p2[(j + 1) % m]);
            let (d1, d2, d3, d4) = (cross(a, b, c0), cross(a, b, d), cross(c0, d, a), cross(c0, d, b));
            if ((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps)) {
                return Some(size);
            }
        }
    }
    None
}

/// Loft through `profiles` (see the module header). A profile with no
/// `segs` is a point: the loft converges to `frame.o` (a pole; no cap).
pub fn loft(feature: &str, profiles: &[ProfileIn], opts: &LoftOpts) -> Result<LoftOut, LoftError> {
    if profiles.len() < 2 {
        return Err(LoftError::new(PROFILES));
    }
    let chains = chains_of(profiles, opts)?;
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
    let (st, grails) = if opts.guides.is_empty() {
        (stations::match_chains_with(chains, &opts.connectors), vec![])
    } else {
        let g = guides::guided_stations(chains, &opts.guides)?;
        (g.st, g.rails)
    };
    // end constraints: the profile normal pointing into the loft
    let mut ends = Ends::default();
    let cen: Vec<Vec3> = st.chains.iter().map(|c| c.centroid()).collect();
    let k = cen.len();
    if let Some(t) = opts.start_normal {
        let mut n = profiles[0].frame.z;
        if n.dot(cen[1] - cen[0]) < 0.0 {
            n = -n;
        }
        ends.d0 = Some(n * t);
    }
    if let Some(t) = opts.end_normal {
        let mut n = profiles[profiles.len() - 1].frame.z;
        if n.dot(cen[k - 1] - cen[k - 2]) < 0.0 {
            n = -n;
        }
        ends.d1 = Some(n * t);
    }
    if st.chains[0].is_point() {
        ends.d0 = None;
    }
    if st.chains[k - 1].is_point() {
        ends.d1 = None;
    }
    let starts = st.chains.iter().map(|c| c.pieces[0].g.start()).collect();
    let body = build(feature, &st, profiles, opts.surface, &grails, ends)?;
    Ok(LoftOut { body, starts, ends, end_centroids: [cen[0], cen[k - 1]] })
}

// ------------------------------------------------------------------ faces

/// Compatible rational B-splines of one piece over all stations: same
/// degree and knot vector, weights present on all or none.
/// Err (never a panic downstream) when the curves still do not agree on the control point count.
fn compat(mut cs: Vec<BSplineCurve>) -> Result<Vec<BSplineCurve>, LoftError> {
    let mismatch = || LoftError { code: FAILED.into(), detail: Some("loft sections cannot be matched: their spline pieces have incompatible knots; redraw the sections with the same number of points, or simplify them".into()) };
    if cs.is_empty() || cs.iter().any(|c| c.ctrl.is_empty() || c.knots.is_empty()) {
        return Err(mismatch());
    }
    let p = cs.iter().map(|c| c.degree).max().unwrap();
    for c in cs.iter_mut() {
        if c.degree < p {
            *c = c.elevate(p - c.degree);
        }
    }
    // knots that agree to rounding (1e-12) are made bit-equal where no curve holds two distinct values
    // inside the group, so rounding noise does not become near-zero spans
    {
        let mut vals: Vec<f64> = cs.iter().flat_map(|c| c.knots.iter().copied()).collect();
        vals.sort_by(|a, b| a.total_cmp(b));
        let mut groups: Vec<(f64, f64)> = vec![];
        for v in vals {
            match groups.last_mut() {
                Some(g) if v - g.1 <= 1e-12 => g.1 = v,
                _ => groups.push((v, v)),
            }
        }
        for (lo, hi) in groups.into_iter().filter(|g| g.0 != g.1) {
            for c in cs.iter_mut() {
                let inside: Vec<f64> = ck_geom::nurbs::basis::distinct_knots(&c.knots).into_iter().filter(|&k| k >= lo && k <= hi).collect();
                if inside.len() == 1 {
                    for k in c.knots.iter_mut().filter(|k| **k >= lo && **k <= hi) {
                        *k = lo;
                    }
                }
            }
        }
    }
    // union of knots, by exact value: a tolerance would merge near-equal knots of one curve into a
    // multiplicity above the degree (knot insertion is capped there), so the curves could not agree
    let mut all: Vec<(f64, usize)> = vec![];
    for c in &cs {
        for k in ck_geom::nurbs::basis::distinct_knots(&c.knots) {
            let m = ck_geom::nurbs::basis::multiplicity(&c.knots, k, 0.0);
            match all.iter_mut().find(|(x, _)| *x == k) {
                Some(e) => e.1 = e.1.max(m),
                None => all.push((k, m)),
            }
        }
    }
    for c in cs.iter_mut() {
        for &(k, m) in &all {
            let have = ck_geom::nurbs::basis::multiplicity(&c.knots, k, 0.0);
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
    let n = cs[0].ctrl.len();
    if cs.iter().any(|c| c.ctrl.len() != n || c.degree != p || c.knots.len() != cs[0].knots.len() || c.weights.as_ref().is_some_and(|w| w.len() != n)) {
        return Err(mismatch());
    }
    Ok(cs)
}

struct Side {
    surface: Surface,
    /// Station curves (compatible NURBS), index = station.
    cs: Vec<BSplineCurve>,
    /// Plane frame when the face is a flat quad of straight pieces.
    plane: Option<Frame>,
}

fn side(st: &Stations, j: usize, ends: Ends) -> Result<Side, LoftError> {
    let k = st.chains.len();
    let pieces: Vec<&Geom3> = st.chains.iter().map(|c| &c.pieces[j].g).collect();
    let cs = compat(pieces.iter().map(|g| g.to_nurbs()).collect())?;
    // flat quad of straight pieces (every station's piece in one plane)
    if !ends.any() && pieces.iter().all(|g| g.is_line()) {
        let (a0, b0, a1, b1) = (pieces[0].start(), pieces[0].end(), pieces[k - 1].start(), pieces[k - 1].end());
        let x = (b0 - a0).try_normalize();
        let z = (b0 - a0).cross(a1 - a0).try_normalize();
        if let (Some(x), Some(z)) = (x, z) {
            let size = a0.dist(b0).max(a0.dist(a1)).max(1.0);
            let flat = pieces.iter().all(|g| (g.start() - a0).dot(z).abs() <= 1e-9 * size && (g.end() - a0).dot(z).abs() <= 1e-9 * size);
            if flat {
                let frame = Frame { o: a0, x, y: z.cross(x), z };
                return Ok(Side { surface: Surface::Plane { frame }, cs, plane: Some(frame) });
            }
        }
    }
    let same_weights = (1..k).all(|q| {
        (0..cs[0].ctrl.len()).all(|i| (cs[q].weight(i) - cs[0].weight(i)).abs() <= 1e-13)
    });
    if k > 2 && !same_weights {
        return Ok(Side { surface: blend_surface(&cs, &st.ts, ends)?, cs, plane: None });
    }
    let nu = cs[0].ctrl.len();
    // control points interpolated by the natural spline per control index
    // (weights agree, or there are only two stations: then the homogeneous
    // blend and this coincide)
    let mut cols: Vec<Vec<[f64; 4]>> = vec![];
    let mut vdeg = 1;
    let mut vknots = vec![];
    for i in 0..nu {
        let hp: Vec<[f64; 4]> = cs
            .iter()
            .map(|c| {
                let w = c.weight(i);
                let p = c.ctrl[i];
                [p.x * w, p.y * w, p.z * w, w]
            })
            .collect();
        let hd = |d: Option<Vec3>, c: &BSplineCurve| d.map(|v| [v.x * c.weight(i), v.y * c.weight(i), v.z * c.weight(i), 0.0]);
        let (d, kn, q) = smooth::through_ends(&st.ts, &hp, hd(ends.d0, &cs[0]), hd(ends.d1, &cs[k - 1])).map_err(|_| LoftError::new(FAILED))?;
        vdeg = d;
        vknots = kn;
        cols.push(q);
    }
    let nv = cols[0].len();
    let mut ctrl = Vec::with_capacity(nu * nv);
    let mut wts = Vec::with_capacity(nu * nv);
    for col in &cols {
        for q in col {
            ctrl.push(Vec3::new(q[0] / q[3], q[1] / q[3], q[2] / q[3]));
            wts.push(q[3]);
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

/// Side face of three or more stations whose profile pieces carry different
/// rational weights (arcs of different sweep, arcs with lines).
///
/// Oracle (random 3-station arc lofts, mesh vertices of the face): the
/// surface is the point blend `S(u, v) = sum_k b_k(v) C_k(u)` of the exact
/// station curves `C_k` (matched at the same NURBS parameter `u`) with the
/// natural-spline cardinal functions `b_k` of the rails, NOT a NURBS in the
/// homogeneous control points (mass off by ~1e-3) nor in separate point /
/// weight nets. The blend is not a rational spline, so it is represented
/// by a bicubic interpolant: exact (cubic) in `v`, interpolated at `U`
/// parameter samples in `u` (error ~1e-8 for arcs).
fn blend_surface(cs: &[BSplineCurve], ts: &[f64], ends: Ends) -> Result<Surface, LoftError> {
    let spans = (ck_geom::nurbs::basis::distinct_knots(&cs[0].knots).len() - 1).max(1);
    let n = 24 * spans + 1;
    let us: Vec<f64> = (0..n).map(|a| a as f64 / (n - 1) as f64).collect();
    // v-direction: control points of the natural spline through C_k(u_a)
    let mut vdeg = 3;
    let mut vknots = vec![];
    let mut q: Vec<Vec<Vec3>> = vec![]; // [a][j]
    for &u in &us {
        let pts: Vec<[f64; 3]> = cs.iter().map(|c| { let p = c.eval(u); [p.x, p.y, p.z] }).collect();
        let (d, kn, c) = smooth::through_ends(ts, &pts, Ends::arr(ends.d0), Ends::arr(ends.d1)).map_err(|_| LoftError::new(FAILED))?;
        vdeg = d;
        vknots = kn;
        q.push(c.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect());
    }
    let nv = q[0].len();
    // u-direction: cubic interpolation through the samples of every row j
    let ku = ck_geom::fit::averaged_knots(&us, 3);
    let mut ctrl = vec![Vec3::new(0.0, 0.0, 0.0); n * nv];
    for j in 0..nv {
        let col: Vec<Vec3> = (0..n).map(|a| q[a][j]).collect();
        let c = ck_geom::fit::interpolate_with(&col, 3, &us, ku.clone()).map_err(|_| LoftError::new(FAILED))?;
        for a in 0..n {
            ctrl[a * nv + j] = c.ctrl[a];
        }
    }
    let surf = BSplineSurface::new(3, vdeg, ku, vknots, n, nv, ctrl, None).map_err(|_| LoftError::new(FAILED))?;
    Ok(Surface::BSpline(Box::new(surf)))
}

/// Side face with a guide as (one of) its rails: the Gordon surface
/// `S = ruled(L, R) + blend(stations) - blend(ruled stations)` sampled and
/// interpolated by a bicubic B-spline (the same representation as
/// [`blend_surface`]); `lf` / `rf` give the left / right rail at the loft
/// parameter `v`.
fn gordon_side(st: &Stations, j: usize, lf: &dyn Fn(f64) -> Vec3, rf: &dyn Fn(f64) -> Vec3, ends: Ends) -> Result<Side, LoftError> {
    let k = st.chains.len();
    let pieces: Vec<&Geom3> = st.chains.iter().map(|c| &c.pieces[j].g).collect();
    let cs = compat(pieces.iter().map(|g| g.to_nurbs()).collect())?;
    let ts = &st.ts;
    let spans = (ck_geom::nurbs::basis::distinct_knots(&cs[0].knots).len() - 1).max(1);
    let n = 24 * spans + 1;
    let us: Vec<f64> = (0..n).map(|a| a as f64 / (n - 1) as f64).collect();
    let per = 8;
    let mut vs: Vec<f64> = vec![];
    for s in 0..k - 1 {
        for i in 0..per {
            vs.push(ts[s] + (ts[s + 1] - ts[s]) * (i as f64 / per as f64));
        }
    }
    vs.push(ts[k - 1]);
    let spline = |xs: &[f64], pts: &[Vec3]| -> Result<BSplineCurve, LoftError> {
        let q: Vec<[f64; 3]> = pts.iter().map(|p| [p.x, p.y, p.z]).collect();
        let (d, kn, c) = smooth::through_ends(xs, &q, Ends::arr(ends.d0), Ends::arr(ends.d1)).map_err(|_| LoftError::new(FAILED))?;
        BSplineCurve::new(d, kn, c.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect()).map_err(|_| LoftError::new(FAILED))
    };
    let mut vdeg = 3;
    let mut vknots = vec![];
    let mut q: Vec<Vec<Vec3>> = vec![]; // [a][i]
    for &u in &us {
        let cp: Vec<Vec3> = cs.iter().map(|c| c.eval(u)).collect();
        let tp: Vec<Vec3> = ts.iter().map(|&t| lf(t) * (1.0 - u) + rf(t) * u).collect();
        let (cc, tc) = (spline(ts, &cp)?, spline(ts, &tp)?);
        let row: Vec<Vec3> = vs.iter().map(|&v| lf(v) * (1.0 - u) + rf(v) * u + cc.eval(v) - tc.eval(v)).collect();
        let r3: Vec<[f64; 3]> = row.iter().map(|p| [p.x, p.y, p.z]).collect();
        let (d, kn, c) = smooth::through_ends(&vs, &r3, Ends::arr(ends.d0), Ends::arr(ends.d1)).map_err(|_| LoftError::new(FAILED))?;
        vdeg = d;
        vknots = kn;
        q.push(c.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect());
    }
    let nv = q[0].len();
    let ku = ck_geom::fit::averaged_knots(&us, 3);
    let mut ctrl = vec![Vec3::new(0.0, 0.0, 0.0); n * nv];
    for jj in 0..nv {
        let col: Vec<Vec3> = (0..n).map(|a| q[a][jj]).collect();
        let c = ck_geom::fit::interpolate_with(&col, 3, &us, ku.clone()).map_err(|_| LoftError::new(FAILED))?;
        for a in 0..n {
            ctrl[a * nv + jj] = c.ctrl[a];
        }
    }
    let surf = BSplineSurface::new(3, vdeg, ku, vknots, n, nv, ctrl, None).map_err(|_| LoftError::new(FAILED))?;
    Ok(Side { surface: Surface::BSpline(Box::new(surf)), cs, plane: None })
}

/// Rail through the matching vertices `pts` of the stations.
fn rail(ts: &[f64], pts: &[Vec3], ends: Ends) -> Result<(Curve, f64, f64), LoftError> {
    if ts.len() == 2 && !ends.any() {
        let d = pts[0].dist(pts[1]);
        return Ok((Curve::line_through(pts[0], pts[1]), 0.0, d));
    }
    if !ends.any() && ts.len() > 2 {
        let total = pts[0].dist(pts[pts.len() - 1]);
        let d = (pts[pts.len() - 1] - pts[0]).try_normalize();
        let span = ts[ts.len() - 1] - ts[0];
        if let (Some(d), true) = (d, total > 0.0 && span > 0.0) {
            let straight = pts.iter().zip(ts).all(|(p, t)| {
                let along = (*p - pts[0]).dot(d);
                (*p - pts[0] - d * along).len() <= 1e-9 * total && (along - total * (t - ts[0]) / span).abs() <= 1e-9 * total
            });
            if straight {
                return Ok((Curve::line_through(pts[0], pts[pts.len() - 1]), 0.0, total));
            }
        }
    }
    let q: Vec<[f64; 3]> = pts.iter().map(|p| [p.x, p.y, p.z]).collect();
    let (deg, kn, c) = smooth::through_ends(ts, &q, Ends::arr(ends.d0), Ends::arr(ends.d1)).map_err(|_| LoftError::new(FAILED))?;
    let cur = BSplineCurve::new(deg, kn, c.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect())
        .map_err(|_| LoftError::new(FAILED))?;
    let (a, b) = cur.domain();
    Ok((Curve::BSpline(cur), a, b))
}

fn build(feature: &str, st: &Stations, profiles: &[ProfileIn], surface: bool, grails: &[Option<guides::GuideRail>], ends: Ends) -> Result<Body, LoftError> {
    let k = st.chains.len();
    let n = st.tags.len();
    let closed = st.closed;
    let nv = if closed { n } else { n + 1 };
    let total = *st.ts.last().unwrap();
    let mut b = Body::new();
    let lump = b.add_lump();
    let shell = b.add_shell(lump);
    // vertices: [station][piece start (+ far end of an open chain)]
    let vp = |s: usize, i: usize| -> Vec3 {
        let c = &st.chains[s];
        if i < n {
            c.pieces[i].g.start()
        } else {
            c.pieces[n - 1].g.end()
        }
    };
    // rails: the natural spline through the stations, or the guide
    let mut rail_curves: Vec<(Curve, f64, f64)> = vec![];
    for i in 0..nv {
        match grails.get(i).and_then(|g| g.as_ref()) {
            Some(g) => {
                let (a, bb) = g.curve.domain();
                rail_curves.push((Curve::BSpline(g.curve.clone()), a, bb));
            }
            None => {
                let pts: Vec<Vec3> = (0..k).map(|s| vp(s, i)).collect();
                rail_curves.push(rail(&st.ts, &pts, ends)?);
            }
        }
    }
    let guided_rail = |i: usize| grails.get(i).map_or(false, |g| g.is_some());
    // position on rail `i` at the loft parameter `v`
    let rail_at = |i: usize, v: f64| -> Vec3 {
        let (c, t0, t1) = &rail_curves[i];
        match c {
            Curve::Line { .. } => c.eval(t0 + (t1 - t0) * (v / total)),
            _ => c.eval(v),
        }
    };
    // sides
    let sides: Vec<Side> = (0..n)
        .map(|j| {
            let jn = if closed { (j + 1) % n } else { j + 1 };
            if guided_rail(j) || guided_rail(jn) {
                gordon_side(st, j, &|v| rail_at(j, v), &|v| rail_at(jn, v), ends)
            } else {
                side(st, j, ends)
            }
        })
        .collect::<Result<_, _>>()?;
    let faces: Vec<FaceId> = (0..n)
        .map(|j| b.add_face(shell, sides[j].surface.clone(), true, FaceName::side(feature, &st.tags[j])))
        .collect();
    // a point end (apex) has no cap, no end edges, and one shared vertex
    let (pt0, pt1) = (st.chains[0].is_point(), st.chains[k - 1].is_point());
    let caps = if surface {
        None
    } else {
        let fr0 = profiles[0].frame;
        let fr1 = profiles[profiles.len() - 1].frame;
        let f0 = (!pt0).then(|| b.add_face(shell, Surface::Plane { frame: fr0 }, true, FaceName::cap(feature, 0, None)));
        let f1 = (!pt1).then(|| b.add_face(shell, Surface::Plane { frame: fr1 }, true, FaceName::cap(feature, 1, None)));
        Some((f0, f1, fr0, fr1))
    };
    let ends_of = |b: &mut Body, s: usize, pt: bool| -> Vec<VertexId> {
        if pt {
            let v = b.add_vertex(vp(s, 0));
            vec![v; nv]
        } else {
            (0..nv).map(|i| b.add_vertex(vp(s, i))).collect()
        }
    };
    let v0: Vec<VertexId> = ends_of(&mut b, 0, pt0);
    let v1: Vec<VertexId> = ends_of(&mut b, k - 1, pt1);
    // rails
    let mut rails: Vec<(EdgeId, bool)> = vec![];
    for i in 0..nv {
        let (c, t0, t1) = rail_curves[i].clone();
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
        e0.push((!pt0).then(|| cap_edge(&mut b, 0, j, v0[j], v0[(j + 1) % nv.max(1)])));
    }
    let mut e1 = vec![];
    for j in 0..n {
        e1.push((!pt1).then(|| cap_edge(&mut b, k - 1, j, v1[j], v1[(j + 1) % nv.max(1)])));
    }
    // side loops
    for j in 0..n {
        let sd = &sides[j];
        let f = faces[j];
        let jn = if closed { (j + 1) % n } else { j + 1 };
        let lp = b.add_loop(f);
        let len0 = e0[j].as_ref().map_or(1.0, |e| e.3 - e.2);
        let len1 = e1[j].as_ref().map_or(1.0, |e| e.3 - e.2);
        let (pc_c0, pc_c1, pc_r0, pc_r1): (Option<Curve2>, Option<Curve2>, Curve2, Curve2);
        if let Some(fr) = sd.plane {
            let line_pc = |c: &Curve| match c {
                Curve::Line { o, d } => plane_line_pc(&fr, *o, *d),
                _ => unreachable!(),
            };
            pc_c0 = e0[j].as_ref().map(|e| line_pc(&e.1));
            pc_c1 = e1[j].as_ref().map(|e| line_pc(&e.1));
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
            let du0 = if e0[j].as_ref().map_or(false, |e| matches!(e.1, Curve::Line { .. })) { 1.0 / len0 } else { 1.0 };
            let du1 = if e1[j].as_ref().map_or(false, |e| matches!(e.1, Curve::Line { .. })) { 1.0 / len1 } else { 1.0 };
            pc_c0 = Some(Curve2::Line { o: Vec2::new(0.0, vlo), d: Vec2::new(du0, 0.0) });
            pc_c1 = Some(Curve2::Line { o: Vec2::new(0.0, vhi), d: Vec2::new(du1, 0.0) });
            let rpc = |u: f64, i: usize, b: &Body| -> Curve2 {
                let (e, is_line) = rails[i];
                let ed = b.edge(e);
                let dv = if is_line { total / (ed.t1 - ed.t0) } else { 1.0 };
                Curve2::Line { o: Vec2::new(u, vlo), d: Vec2::new(0.0, dv) }
            };
            pc_r0 = rpc(0.0, j, &b);
            pc_r1 = rpc(1.0, jn, &b);
        }
        if let Some(e) = &e0[j] {
            b.add_coedge(lp, e.0, false, pc_c0);
        }
        b.add_coedge(lp, rails[jn].0, false, Some(pc_r1));
        if let Some(e) = &e1[j] {
            b.add_coedge(lp, e.0, true, pc_c1);
        }
        b.add_coedge(lp, rails[j].0, true, Some(pc_r0));
    }
    // caps
    if let Some((f0, f1, fr0, fr1)) = caps {
        let pc = |fr: &Frame, c: &Curve| match c {
            // arcs are rational B-splines: the plane projection is affine, so the pcurve is exact
            Curve::BSpline(bs) => Some(Curve2::BSpline(bs.map(|p| Vec2::new((p - fr.o).dot(fr.x), (p - fr.o).dot(fr.y))))),
            _ => ck_topo::make::plane_pcurve(fr, c),
        };
        if let Some(f0) = f0 {
            let l0 = b.add_loop(f0);
            for j in (0..n).rev() {
                let e = e0[j].as_ref().unwrap();
                b.add_coedge(l0, e.0, true, pc(&fr0, &e.1));
            }
            let a = ck_topo::orient::loop_uv_area(&b, l0);
            b.faces[f0.idx()].sense = a >= 0.0;
        }
        if let Some(f1) = f1 {
            let l1 = b.add_loop(f1);
            for j in 0..n {
                let e = e1[j].as_ref().unwrap();
                b.add_coedge(l1, e.0, false, pc(&fr1, &e.1));
            }
            let a = ck_topo::orient::loop_uv_area(&b, l1);
            b.faces[f1.idx()].sense = a >= 0.0;
        }
    }
    // sheets keep one name for all pieces of an entity (oracle: four
    // `F4.S.e1` faces, no `~k`); solids number them
    if !surface {
        b.number_split_faces();
        ck_topo::orient::orient_body(&mut b);
    }
    let _ = TOL;
    Ok(b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    fn cubic(knots: Vec<f64>) -> BSplineCurve {
        let n = knots.len() - 4;
        BSplineCurve::new(3, knots, (0..n).map(|i| Vec3::new(i as f64, (i * i % 5) as f64, 0.0)).collect()).unwrap()
    }

    /// Regression: sections drawn off-grid gave pieces with knots equal to rounding (or two knots 1e-13 apart
    /// in one piece); the curves ended with different control point counts and the rail loop indexed past
    /// the shorter one (kernel panic). `compat` now agrees on the counts or reports a structured error.
    #[test]
    fn compat_near_equal_knots_agree() {
        let a = cubic(vec![0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0]);
        let b = cubic(vec![0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5 + 5e-13, 0.5 + 5e-13, 0.5 + 5e-13, 1.0, 1.0, 1.0, 1.0]);
        let c = cubic(vec![0.0, 0.0, 0.0, 0.0, 0.5 + 3e-14, 0.5 + 3e-14, 0.5 + 3e-14, 1.0, 1.0, 1.0, 1.0]);
        for order in [[0, 1, 2], [2, 1, 0], [1, 0, 2]] {
            let all = [a.clone(), b.clone(), c.clone()];
            let cs = compat(order.iter().map(|&i| all[i].clone()).collect()).expect("compatible");
            assert!(cs.iter().all(|c| c.ctrl.len() == cs[0].ctrl.len() && c.knots.len() == cs[0].knots.len()));
        }
    }

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

    fn square_profile(y: f64, h: f64) -> ProfileIn {
        let fr = frame_at(y);
        let p = |x: f64, z: f64| Vec2::new(x, -z); // frame y = -z world
        let c = [p(-h, h), p(h, h), p(h, -h), p(-h, -h)];
        let segs = (0..4)
            .map(|i| {
                let (a, b) = (c[i], c[(i + 1) % 4]);
                let (wa, wb) = (fr.o + fr.x * a.x + fr.y * a.y, fr.o + fr.x * b.x + fr.y * b.y);
                SegIn { tag: format!("e{}", i + 1), curve: Curve::line_through(wa, wb), t0: 0.0, t1: wa.dist(wb), rev: false }
            })
            .collect();
        ProfileIn { frame: fr, closed: true, segs }
    }

    fn point_profile(y: f64) -> ProfileIn {
        let mut fr = frame_at(y);
        fr.o = Vec3::new(0.0, y, 0.0);
        ProfileIn { frame: fr, closed: true, segs: vec![] }
    }

    fn check_closed(b: &Body) {
        assert!(ck_topo::check::check(b, 1e-6).is_empty(), "{:?}", ck_topo::check::check(b, 1e-6));
    }

    /// Circle -> point is a cone: V = pi r^2 h / 3; two faces (lateral halves) + base cap.
    #[test]
    fn circle_to_point_is_a_cone() {
        for flip in [false, true] {
            let ps = if flip { [point_profile(0.0), circle_profile(30.0, 0.0, 8.0)] } else { [circle_profile(0.0, 0.0, 8.0), point_profile(30.0)] };
            let out = loft("F4", &ps, &LoftOpts::default()).unwrap();
            check_closed(&out.body);
            let m = ck_topo::mass::mass_props(&out.body);
            let want = std::f64::consts::PI * 64.0 * 30.0 / 3.0;
            assert!((m.volume - want).abs() < 1e-6 * want, "{} vs {}", m.volume, want);
            assert_eq!(out.body.num_faces(), 3);
        }
    }

    /// Square -> point is a pyramid: V = a^2 h / 3, 4 planar triangles + base.
    #[test]
    fn square_to_point_is_a_pyramid() {
        let out = loft("F4", &[square_profile(0.0, 5.0), point_profile(12.0)], &LoftOpts::default()).unwrap();
        check_closed(&out.body);
        let m = ck_topo::mass::mass_props(&out.body);
        let want = 100.0 * 12.0 / 3.0;
        assert!((m.volume - want).abs() < 1e-6 * want, "{} vs {}", m.volume, want);
        assert_eq!(out.body.num_faces(), 5);
        assert_eq!(out.body.edge_ids().count(), 8);
    }

    /// Point -> circle -> point (a spindle through three stations) stays closed.
    #[test]
    fn point_both_ends() {
        let ps = [point_profile(0.0), circle_profile(10.0, 0.0, 6.0), point_profile(20.0)];
        let out = loft("F4", &ps, &LoftOpts::default()).unwrap();
        check_closed(&out.body);
        let m = ck_topo::mass::mass_props(&out.body);
        assert!(m.volume > 0.0 && m.volume < std::f64::consts::PI * 36.0 * 20.0, "{}", m.volume);
        let e = loft("F4", &[point_profile(0.0), circle_profile(10.0, 0.0, 6.0), point_profile(20.0), circle_profile(30.0, 0.0, 6.0)], &LoftOpts::default());
        assert!(e.is_err());
    }

    /// Guided loft (oracle `loftGuideLine`: 7 faces / 15 edges, volume 7000 for a straight
    /// guide through two edge mid-points); a bulging guide is followed exactly and the
    /// result is a valid closed solid (oracle `loftGuideSpline`: 8029.2, ours is smaller:
    /// the oracle spreads the pull wider over the profile).
    #[test]
    fn guided_loft_follows_the_guide() {
        use crate::loft::guides::GuideIn;
        let profiles = [square_profile(0.0, 10.0), square_profile(30.0, 5.0)];
        let line = GuideIn { pts: (0..=8).map(|i| Vec3::new(10.0 - 5.0 * i as f64 / 8.0, 30.0 * i as f64 / 8.0, 0.0)).collect() };
        let out = loft("F5", &profiles, &LoftOpts { surface: false, guides: vec![line], ..Default::default() }).unwrap();
        assert_eq!(out.body.num_faces(), 7);
        assert_eq!(out.body.edge_ids().count(), 15);
        let m = ck_topo::mass::mass_props(&out.body);
        assert!((m.volume - 7000.0).abs() < 1e-6, "{}", m.volume);
        // bulging guide: x(y) = 10 - 5y/30 + 4 sin(pi y/30)
        let bulge = GuideIn {
            pts: (0..=60)
                .map(|i| {
                    let y = 30.0 * i as f64 / 60.0;
                    Vec3::new(10.0 - 5.0 * y / 30.0 + 4.0 * (std::f64::consts::PI * y / 30.0).sin(), y, 0.0)
                })
                .collect(),
        };
        let out = loft("F5", &profiles, &LoftOpts { surface: false, guides: vec![bulge.clone()], ..Default::default() }).unwrap();
        assert!(ck_topo::check::check(&out.body, 1e-6).is_empty(), "closed manifold");
        let m = ck_topo::mass::mass_props(&out.body);
        assert!(m.volume > 7000.0 + 300.0 && m.volume < 9500.0, "{}", m.volume);
        // the guide rail (the edge between the two halves of S.e2) lies on the guide
        let rail = out.body.edge_ids().find(|&e| {
            let f: Vec<String> = out.body.edge_faces(e).iter().map(|&f| out.body.face_id_str(f)).collect();
            f.iter().any(|n| n.starts_with("F5.S.e2~1")) && f.iter().any(|n| n.starts_with("F5.S.e2~2"))
        });
        let ed = out.body.edge(rail.expect("guide rail"));
        for k in 0..=10 {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / 10.0);
            let x = 10.0 - 5.0 * p.y / 30.0 + 4.0 * (std::f64::consts::PI * p.y / 30.0).sin();
            assert!((p.x - x).abs() < 2e-3 && p.z.abs() < 1e-9, "{p:?} vs x {x}");
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

    /// A cross-hole through a lofted (free-form) frustum: the boolean starts its welding ladder at 10x
    /// the base tolerance for free-form operands; the result must stay a valid solid whose volume is
    /// the frustum's minus the hole (grid-integrated here).
    #[test]
    fn cross_hole_through_lofted_frustum() {
        use crate::boolean::{boolean, Op};
        use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
        let frustum = loft("F4", &[circle_profile(0.0, 0.0, 8.0), circle_profile(30.0, 2.0, 4.0)], &LoftOpts::default()).unwrap().body;
        assert!(frustum.face_ids().any(|f| matches!(frustum.face(f).surface, ck_geom::surface::Surface::BSpline(_))), "a free-form lateral face");
        let fr = Frame { o: Vec3::new(-20.0, 15.0, 0.0), x: Vec3::new(0.0, 1.0, 0.0), y: Vec3::new(0.0, 0.0, 1.0), z: Vec3::new(1.0, 0.0, 0.0) };
        let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", Vec2::new(0.0, 0.0), 2.0)]);
        let tool = extrude("T", &Profile::new(fr, vec![ProfileRegion { outer: lp, holes: vec![] }]), ExtrudeOpts { s0: 0.0, s1: 40.0, dir: None, draft: 0.0 }).unwrap();
        let out = boolean(&frustum, &tool, Op::Cut, 1e-6).expect("cut");
        assert!(out.mismatch.is_none(), "{:?}", out.mismatch);
        assert!(ck_topo::check::check(&out.body, 1e-6).is_empty());
        // hole volume: midpoint grid over the tool cylinder (axis along x at y = 15, z = 0, radius 2)
        let (nx, nr, nt) = (400usize, 40usize, 64usize);
        let mut removed = 0.0;
        for i in 0..nx {
            let x = -20.0 + 40.0 * (i as f64 + 0.5) / nx as f64;
            for j in 0..nr {
                let rr = 2.0 * (j as f64 + 0.5) / nr as f64;
                for k in 0..nt {
                    let a = TAU * (k as f64 + 0.5) / nt as f64;
                    let (y, z) = (15.0 + rr * a.cos(), rr * a.sin());
                    let (cx, rad) = (2.0 * y / 30.0, 8.0 - 4.0 * y / 30.0);
                    if (x - cx) * (x - cx) + z * z <= rad * rad {
                        removed += (40.0 / nx as f64) * (2.0 / nr as f64) * rr * (TAU / nt as f64);
                    }
                }
            }
        }
        let full = std::f64::consts::PI * 10.0 * 112.0;
        let got = ck_topo::mass::mass_props(&out.body).volume;
        assert!((got - (full - removed)).abs() < 3e-3 * full, "{got} vs {}", full - removed);
    }
}
