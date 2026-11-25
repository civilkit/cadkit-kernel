//! The neutral B-rep ([`crate::brep::Body`], millimetres) built into a `ck-topo` body: the part
//! of the reader that is ours (upstream's counterpart feeds an OCCT builder).
//!
//! Surfaces: plane, cylinder, cone, sphere, torus, B-spline (a closed one on its one period), extrusion and
//! revolution of a line, circle, ellipse or B-spline, offset of a B-spline. Curves: line, circle, ellipse,
//! B-spline (a periodic one cut at its seam). A face on any other surface (ruled, arc sweep, blend, unknown,
//! a self-intersecting torus) or with an edge on any other curve (helix, interpolated, curve on surface,
//! unknown) is left out
//! and reported; so is a face whose geometry the builder cannot make consistent (its vertices off
//! its surface, loops that cannot be closed over a seam). A body that lost a face is not a closed
//! solid, and the caller says so.
//!
//! The neutral model follows OCCT's conventions and ours are the same: an edge runs along
//! increasing parameter from `vertices[0]` to `vertices[1]`, a coedge uses it forward or
//! reversed, a face's loops keep the face on their left against its material normal, and a
//! reversed face has its natural normal turned round. Periodic faces come without seam edges (as
//! ASM stores them): the seam is inserted here, as the SAT importer does (`ck-io`): a straight
//! line between the two loops that wrap a cylinder or cone, a meridian between the two that wrap
//! a sphere or torus, and for a cap (one wrapping loop) the seam to the pole or apex, which stays
//! implicit.

use crate::brep::{self, BSplineCurve as NCurve, BSplineSurface as NSurface, Body as NBody};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::pcurve::pcurve_of;
use ck_geom::surface::Surface;
use ck_geom::{Vec2, Vec3, frame_from};
use ck_topo::body::{Body, EdgeId, FaceName, LumpId, ShellId, VertexId};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

/// A face the builder left out, and why in plain words.
#[derive(Clone, Debug, PartialEq)]
pub struct SkippedFace {
    /// The face's index in the neutral body.
    pub face: usize,
    /// A stable code: `unsupportedSurface`, `unsupportedCurve`, `unknownGeometry`, `offSurface`,
    /// `seam`, `orientation`, `geometry`.
    pub code: &'static str,
    pub message: String,
    /// The kind of surface or curve concerned (`ruled`, `helix curve`, ...), when one is.
    pub what: String,
}

/// What building one neutral body gave.
#[derive(Clone, Debug, Default)]
pub struct Built {
    /// The lumps added to the output body, one per neutral lump.
    pub lumps: Vec<LumpId>,
    pub faces_built: usize,
    pub skipped: Vec<SkippedFace>,
    /// Edges and vertices given a tolerance because they deviate from their faces or curves by
    /// more than the strict tolerance (the file's own geometry is fitted to a looser one).
    pub tolerances_set: usize,
    /// Edges and vertices that deviate by more than the largest tolerance that may be set.
    pub over_tolerance: usize,
    /// Edges split so that a periodic face's two loops could be joined by a seam.
    pub splits: usize,
}

/// The import's geometric tolerance (mm): how far a vertex may lie from its edge's curve or from
/// its face's surface before the face is left out. It is the largest tolerance the validator
/// accepts (`ck_check::DEFAULT_TOL_CAP`): a file's own geometry is fitted to a looser tolerance
/// than the strict one (some of the corpus's edges miss their end vertices by 0.01 to 0.03 mm),
/// and what is within it is recorded as tolerances of the edges and vertices
/// (`ck_heal::tolerant_only`), as for any native import.
const IMPORT_TOL: f64 = ck_check::DEFAULT_TOL_CAP;

fn v3(p: brep::P3) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// The body's own transform (`p' = m p + t`): rotation and uniform scale only.
#[derive(Clone, Copy)]
struct Xf {
    m: [[f64; 3]; 3],
    t: Vec3,
    /// Length scale.
    s: f64,
}

impl Xf {
    fn of(t: Option<&brep::Transform>) -> Result<Xf, String> {
        let Some(t) = t else {
            return Ok(Xf::identity());
        };
        let m = t.m;
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if !(det > 0.0) || !det.is_finite() {
            return Err("the body is reflected by its transform".into());
        }
        let s = det.cbrt();
        for i in 0..3 {
            for j in 0..3 {
                let d: f64 = (0..3).map(|k| m[i][k] * m[j][k]).sum();
                if (d - if i == j { s * s } else { 0.0 }).abs() > 1e-9 * s * s {
                    return Err("the body's transform shears or scales unevenly".into());
                }
            }
        }
        Ok(Xf {
            m,
            t: v3(t.t),
            s,
        })
    }
    fn identity() -> Xf {
        Xf {
            m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            t: Vec3::ZERO,
            s: 1.0,
        }
    }
    /// A direction (turned and scaled; callers normalise where they need to).
    fn dir(&self, p: brep::P3) -> Vec3 {
        let m = &self.m;
        Vec3::new(
            m[0][0] * p[0] + m[0][1] * p[1] + m[0][2] * p[2],
            m[1][0] * p[0] + m[1][1] * p[1] + m[1][2] * p[2],
            m[2][0] * p[0] + m[2][1] * p[1] + m[2][2] * p[2],
        )
    }
    fn pt(&self, p: brep::P3) -> Vec3 {
        self.dir(p) + self.t
    }
    fn unit(&self, p: brep::P3) -> Vec3 {
        self.dir(p).normalize()
    }
}

/// One coedge of a loop being assembled.
#[derive(Clone)]
struct Ce {
    edge: EdgeId,
    rev: bool,
    /// Coedge leaving a pole or apex: its pcurve is aligned by its end to the loop's start
    /// instead of by its start to the previous end (the pole's `v = const` edge adds nothing).
    pole_jump: bool,
}

struct B<'a> {
    src: &'a NBody,
    xf: Xf,
    out: &'a mut Body,
    verts: HashMap<usize, VertexId>,
    edges: HashMap<usize, EdgeId>,
    /// An edge to split (neutral index and parameter) before the pass is run again.
    split: Option<(usize, f64)>,
    /// Edges made here whose pcurve is known exactly (a meridian to the pole of a closed B-spline surface).
    pc_override: HashMap<EdgeId, Curve2>,
}

/// Why one face was left out: code, message and the kind of surface or curve concerned.
type Why = (&'static str, String, String);

fn why(code: &'static str, message: impl Into<String>) -> Why {
    (code, message.into(), String::new())
}

/// Builds `src` into `out` (its lumps are added). Faces the builder cannot make are left out
/// and listed; the rest are built.
pub fn build_body(out: &mut Body, src: &NBody, tag: &str) -> Result<Built, String> {
    let xf = Xf::of(src.transform.as_ref())?;
    let mut skip: HashMap<usize, Why> = HashMap::new();
    for (i, f) in src.faces.iter().enumerate() {
        if let Some(w) = pre_check(src, f) {
            skip.insert(i, w);
        }
    }
    // The body as edited by edge splits (a copy, made when the first split is asked for). Edges that run
    // across the seam of a periodic B-spline curve are cut there first.
    let mut edited: Option<NBody> = cut_periodic_edges(src);
    let mut splits = 0usize;
    // Faces that fail to build are found on a scratch body and left out of the next pass (an
    // edge or loop made for a face that fails would stay behind, unused); an edge a periodic
    // face needs split is split and the pass run again.
    for _ in 0..(4 + MAX_SPLITS) {
        let work = edited.as_ref().unwrap_or(src);
        let mut scratch = Body::new();
        let (failed, split) = pass(&mut scratch, work, &xf, &skip, tag);
        if let Some((edge, t)) = split {
            if splits >= MAX_SPLITS {
                return Err("too many edges need splitting to join the loops of periodic faces".into());
            }
            let w = edited.get_or_insert_with(|| src.clone());
            split_edge(w, edge, t);
            splits += 1;
            continue;
        }
        if !failed.is_empty() {
            for (i, w) in failed {
                skip.insert(i, w);
            }
            continue;
        }
        let faces_built = scratch.num_faces();
        // what an import does to a solid read from a native file: edges and vertices that deviate
        // from their faces by more than the strict tolerance record how much
        let report = ck_heal::heal_with(&mut scratch, &ck_heal::tolerant_only());
        let tolerances_set = report
            .fixed
            .iter()
            .filter(|f| f.code == "EDGE_TOLERANCE" || f.code == "VERTEX_TOLERANCE")
            .count();
        let over_tolerance = report
            .remaining
            .iter()
            .filter(|i| i.code.ends_with("TOLERANCE_EXCEEDED") || i.code == "EDGE_OFF_SURFACE" || i.code == "VERTEX_OFF_CURVE")
            .count();
        let lumps = out.append(&scratch);
        let mut skipped: Vec<SkippedFace> = skip
            .into_iter()
            .map(|(face, (code, message, what))| SkippedFace {
                face,
                code,
                message,
                what,
            })
            .collect();
        skipped.sort_by_key(|s| s.face);
        return Ok(Built {
            lumps,
            faces_built,
            skipped,
            tolerances_set,
            over_tolerance,
            splits,
        });
    }
    Err("the faces of the body could not be built consistently".into())
}

/// The edges of `src` that run across the seam of a periodic B-spline curve, cut at the seam (each part inside
/// the curve's domain); other edges on such a curve are moved into the domain. `None` when nothing changes.
fn cut_periodic_edges(src: &NBody) -> Option<NBody> {
    let periodic = |e: &brep::Edge| match &src.curves[e.curve] {
        brep::Curve::BSpline(b) if b.periodic && b.is_valid() => Some((b.knots[0], *b.knots.last().unwrap())),
        _ => None,
    };
    let needs = src.edges.iter().any(|e| periodic(e).is_some_and(|(a, z)| e.t[0] < a - 1e-9 || e.t[1] > z + 1e-9));
    if !needs {
        return None;
    }
    let mut w = src.clone();
    for ei in 0..src.edges.len() {
        let e = src.edges[ei].clone();
        let Some((a, z)) = periodic(&e) else { continue };
        let period = z - a;
        if !(period > 0.0) || !(e.t[0] < a - 1e-9 || e.t[1] > z + 1e-9) {
            continue;
        }
        // into the domain: the start in [a, z)
        let k = ((e.t[0] - a) / period).floor();
        let (mut t0, mut t1) = (e.t[0] - k * period, e.t[1] - k * period);
        if t0 >= z - 1e-9 {
            t0 -= period;
            t1 -= period;
            t0 = t0.max(a);
        }
        if t1 <= z + 1e-9 {
            w.edges[ei].t = [t0, t1];
            continue;
        }
        if t1 - t0 > period + 1e-9 {
            continue; // more than a turn: left to the builder's refusal
        }
        // cut at the seam: this edge keeps [t0, z], a new one takes [a, t1 - period]
        let p = w.curves[e.curve].eval(a);
        let vtol = w.vertices[e.vertices[0]].tolerance.max(w.vertices[e.vertices[1]].tolerance);
        let vnew = w.vertices.len();
        w.vertices.push(brep::Vertex { point: p, tolerance: vtol });
        let ej = w.edges.len();
        w.edges.push(brep::Edge { curve: e.curve, t: [a, t1 - period], vertices: [vnew, e.vertices[1]], tolerance: e.tolerance });
        w.edges[ei].t = [t0, z];
        w.edges[ei].vertices[1] = vnew;
        for f in &mut w.faces {
            for lp in &mut f.loops {
                let mut out = Vec::with_capacity(lp.len() + 1);
                for c in lp.drain(..) {
                    if c.edge != ei {
                        out.push(c);
                    } else if c.forward {
                        out.push(brep::Coedge { edge: ei, forward: true });
                        out.push(brep::Coedge { edge: ej, forward: true });
                    } else {
                        out.push(brep::Coedge { edge: ej, forward: false });
                        out.push(brep::Coedge { edge: ei, forward: false });
                    }
                }
                *lp = out;
            }
        }
    }
    Some(w)
}

/// The most edges split in one body.
const MAX_SPLITS: usize = 64;

/// Splits edge `ei` of the neutral body at its parameter `t`: the edge keeps the part before,
/// a new edge takes the part after, and every loop that uses it runs through both.
fn split_edge(src: &mut NBody, ei: usize, t: f64) {
    let e = src.edges[ei].clone();
    let p = src.curves[e.curve].eval(t);
    let vtol = src.vertices[e.vertices[0]]
        .tolerance
        .max(src.vertices[e.vertices[1]].tolerance);
    let vnew = src.vertices.len();
    src.vertices.push(brep::Vertex {
        point: p,
        tolerance: vtol,
    });
    let ej = src.edges.len();
    src.edges.push(brep::Edge {
        curve: e.curve,
        t: [t, e.t[1]],
        vertices: [vnew, e.vertices[1]],
        tolerance: e.tolerance,
    });
    src.edges[ei].t[1] = t;
    src.edges[ei].vertices[1] = vnew;
    for f in &mut src.faces {
        for lp in &mut f.loops {
            let mut out = Vec::with_capacity(lp.len() + 1);
            for c in lp.drain(..) {
                if c.edge != ei {
                    out.push(c);
                } else if c.forward {
                    out.push(brep::Coedge { edge: ei, forward: true });
                    out.push(brep::Coedge { edge: ej, forward: true });
                } else {
                    out.push(brep::Coedge { edge: ej, forward: false });
                    out.push(brep::Coedge { edge: ei, forward: false });
                }
            }
            *lp = out;
        }
    }
}

/// The profile of an extrusion or revolution: a curve we make (line, circle, ellipse, B-spline).
fn profile_check(c: &brep::Curve) -> Option<Why> {
    match c {
        brep::Curve::Line { .. } | brep::Curve::Ellipse { .. } | brep::Curve::BSpline(_) => None,
        other => Some((
            "unsupportedSurface",
            format!("its surface is swept from a {} curve, which is not imported", other.kind()),
            format!("swept {} curve", other.kind()),
        )),
    }
}

/// A face the builder cannot make at all, judged before any building.
fn pre_check(src: &NBody, f: &brep::Face) -> Option<Why> {
    match &src.surfaces[f.surface] {
        brep::Surface::Unknown(m) => {
            return Some(why(
                "unknownGeometry",
                format!("its surface could not be read ({m})"),
            ));
        }
        brep::Surface::Plane { .. }
        | brep::Surface::Cone { .. }
        | brep::Surface::Sphere { .. }
        | brep::Surface::Torus { .. }
        | brep::Surface::BSpline(_) => {}
        brep::Surface::Extrusion { curve, .. } | brep::Surface::Revolution { curve, .. } => {
            if let Some(w) = profile_check(curve) {
                return Some(w);
            }
        }
        brep::Surface::Offset { basis, .. } if matches!(**basis, brep::Surface::BSpline(_)) => {}
        other => {
            return Some((
                "unsupportedSurface",
                format!(
                    "its surface is a {}, which is not imported (plane, cylinder, cone, sphere, torus and B-spline surfaces are)",
                    other.kind()
                ),
                other.kind().to_string(),
            ));
        }
    }
    for lp in &f.loops {
        for c in lp {
            let e = &src.edges[c.edge];
            match &src.curves[e.curve] {
                brep::Curve::Line { .. } | brep::Curve::Ellipse { .. } | brep::Curve::BSpline(_) => {}
                brep::Curve::Unknown(m) => {
                    return Some(why(
                        "unknownGeometry",
                        format!("an edge's curve could not be read ({m})"),
                    ));
                }
                other => {
                    return Some((
                        "unsupportedCurve",
                        format!(
                            "an edge is a {} curve, which is not imported (line, circle, ellipse and B-spline curves are)",
                            other.kind()
                        ),
                        format!("{} curve", other.kind()),
                    ));
                }
            }
        }
    }
    None
}

/// One building pass over the faces not in `skip`; the faces that failed.
fn pass(
    out: &mut Body,
    src: &NBody,
    xf: &Xf,
    skip: &HashMap<usize, Why>,
    tag: &str,
) -> (Vec<(usize, Why)>, Option<(usize, f64)>) {
    let mut b = B {
        src,
        xf: *xf,
        out,
        verts: HashMap::new(),
        edges: HashMap::new(),
        split: None,
        pc_override: HashMap::new(),
    };
    let mut failed = Vec::new();
    for lump in &src.lumps {
        let l = b.out.add_lump();
        for shell in &lump.shells {
            let sh = b.out.add_shell(l);
            for &fi in &shell.faces {
                if skip.contains_key(&fi) {
                    continue;
                }
                if let Err(w) = b.face(sh, fi, tag) {
                    if b.split.is_some() {
                        return (vec![], b.split);
                    }
                    failed.push((fi, w));
                }
            }
        }
    }
    (failed, None)
}

fn bspline_curve(c: &NCurve, xf: &Xf) -> Result<BSplineCurve, String> {
    if !c.is_valid() {
        return Err("a B-spline curve's knots and poles do not fit".into());
    }
    let mut ks = Vec::new();
    for (k, m) in c.knots.iter().zip(&c.mults) {
        ks.extend(std::iter::repeat_n(*k, *m));
    }
    let poles: Vec<Vec3> = c.poles.iter().map(|p| xf.pt(*p)).collect();
    match &c.weights {
        Some(w) => BSplineCurve::new_rational(c.degree, ks, poles, w.clone()),
        None => BSplineCurve::new(c.degree, ks, poles),
    }
}

fn bspline_surface(s: &NSurface, xf: &Xf) -> Result<BSplineSurface, String> {
    if !s.is_valid() {
        return Err("a B-spline surface's knots and poles do not fit".into());
    }
    let flat = |k: &[f64], m: &[usize]| {
        let mut ks = Vec::new();
        for (k, m) in k.iter().zip(m) {
            ks.extend(std::iter::repeat_n(*k, *m));
        }
        ks
    };
    BSplineSurface::new(
        s.u_degree,
        s.v_degree,
        flat(&s.u_knots, &s.u_mults),
        flat(&s.v_knots, &s.v_mults),
        s.nu,
        s.nv,
        s.poles.iter().map(|p| xf.pt(*p)).collect(),
        s.weights.clone(),
    )
}

impl B<'_> {
    fn vertex(&mut self, vi: usize) -> VertexId {
        if let Some(v) = self.verts.get(&vi) {
            return *v;
        }
        let p = self.xf.pt(self.src.vertices[vi].point);
        let id = self.out.add_vertex(p);
        self.verts.insert(vi, id);
        id
    }

    fn vpos(&self, v: VertexId) -> Vec3 {
        self.out.vertices[v.idx()].p
    }

    /// The curve of an edge in our terms, and the factor from the neutral parameter to ours.
    fn curve(&self, c: &brep::Curve) -> Result<(Curve, f64), Why> {
        let xf = &self.xf;
        match c {
            brep::Curve::Line { origin, dir } => {
                let d = xf.dir(*dir);
                let l = d.len();
                if l < 1e-12 {
                    return Err(why("geometry", "a line edge has no direction"));
                }
                // our lines are parameterised by arc length
                Ok((Curve::line(xf.pt(*origin), d), xf.s))
            }
            brep::Curve::Ellipse {
                center,
                normal,
                major_dir,
                major,
                minor,
            } => {
                let (n, m) = (xf.unit(*normal), xf.unit(*major_dir));
                let (a, b) = (major * xf.s, minor * xf.s);
                if !(a > 1e-12) || !(b > 1e-12) || n.len() < 0.5 {
                    return Err(why("geometry", "a circular or elliptic edge is degenerate"));
                }
                let c = xf.pt(*center);
                if (a - b).abs() <= 1e-12 * a {
                    Ok((Curve::circle_x(c, n, m, a), 1.0))
                } else {
                    Ok((Curve::ellipse(c, n, m, a, b), 1.0))
                }
            }
            brep::Curve::BSpline(b) => bspline_curve(b, xf)
                .map(|c| (Curve::BSpline(c), 1.0))
                .map_err(|e| why("geometry", e)),
            _ => Err(why("unsupportedCurve", "an edge's curve is not imported")),
        }
    }

    fn edge(&mut self, ei: usize) -> Result<EdgeId, Why> {
        if let Some(e) = self.edges.get(&ei) {
            return Ok(*e);
        }
        let ne = &self.src.edges[ei];
        let (curve, k) = self.curve(&self.src.curves[ne.curve])?;
        let (t0, t1) = (ne.t[0] * k, ne.t[1] * k);
        if !(t1 - t0 > 1e-12) {
            return Err(why("geometry", "an edge has no length"));
        }
        if let Curve::BSpline(b) = &curve {
            let (a, z) = b.domain();
            let slack = 1e-9 * (1.0 + (z - a).abs());
            if t0 < a - slack || t1 > z + slack {
                return Err(why(
                    "seam",
                    "an edge runs across the seam of a periodic B-spline curve, which is not imported",
                ));
            }
        }
        let (v0, v1) = (self.vertex(ne.vertices[0]), self.vertex(ne.vertices[1]));
        let (p0, p1) = (self.vpos(v0), self.vpos(v1));
        let d = curve.eval(t0).dist(p0).max(curve.eval(t1).dist(p1));
        if d > IMPORT_TOL {
            return Err(why(
                "geometry",
                format!("an edge misses its end vertices by {d:.2e} mm"),
            ));
        }
        let id = self.out.add_edge(curve, t0, t1, v0, v1);
        self.edges.insert(ei, id);
        Ok(id)
    }

    fn surface(&self, s: &brep::Surface) -> Result<Surface, Why> {
        let xf = &self.xf;
        let bad = |m: &str| why("geometry", m.to_string());
        match s {
            brep::Surface::Plane {
                origin,
                normal,
                u_dir,
            } => {
                let n = xf.unit(*normal);
                if n.len() < 0.5 {
                    return Err(bad("a plane has no normal"));
                }
                Ok(Surface::Plane {
                    frame: frame_from(xf.pt(*origin), n, xf.unit(*u_dir)),
                })
            }
            brep::Surface::Cone {
                origin,
                axis,
                ref_dir,
                radius,
                half_angle,
            } => {
                let r = radius * xf.s;
                let frame = frame_from(xf.pt(*origin), xf.unit(*axis), xf.unit(*ref_dir));
                if *half_angle == 0.0 {
                    if !(r > 1e-12) {
                        return Err(bad("a cylinder has no radius"));
                    }
                    Ok(Surface::Cylinder { frame, r })
                } else {
                    if r < 0.0 {
                        return Err(bad("a cone has a negative radius"));
                    }
                    Ok(Surface::Cone {
                        frame,
                        r,
                        semi_angle: *half_angle,
                    })
                }
            }
            brep::Surface::Sphere {
                center,
                axis,
                ref_dir,
                radius,
            } => {
                let r = radius * xf.s;
                if !(r > 1e-12) {
                    return Err(bad("a sphere has no radius"));
                }
                Ok(Surface::Sphere {
                    frame: frame_from(xf.pt(*center), xf.unit(*axis), xf.unit(*ref_dir)),
                    r,
                })
            }
            brep::Surface::Torus {
                center,
                axis,
                ref_dir,
                major,
                minor,
            } => {
                let (major, minor) = (major * xf.s, minor * xf.s);
                if !(major > 0.0) || !(minor > 1e-12) {
                    return Err(bad("a torus has no radius"));
                }
                if minor >= major {
                    return Err(why(
                        "unsupportedSurface",
                        "its surface is a self-intersecting torus (minor radius not below the major one), which is not imported",
                    ));
                }
                Ok(Surface::Torus {
                    frame: frame_from(xf.pt(*center), xf.unit(*axis), xf.unit(*ref_dir)),
                    major,
                    minor,
                })
            }
            brep::Surface::BSpline(b) => {
                // a closed free-form surface is built on its one period; a face that crosses or wraps
                // its seam is found in `face` (its loops do not close in the parameters) and left out
                bspline_surface(b, xf)
                    .map(|s| Surface::BSpline(Box::new(s)))
                    .map_err(|e| why("geometry", e))
            }
            // C(u) + v dir (OCCT's `Geom_SurfaceOfLinearExtrusion`, natural normal C' x dir)
            brep::Surface::Extrusion { curve, dir } => {
                let (profile, _) = self.curve(curve)?;
                let d = xf.unit(*dir);
                if d.len() < 0.5 {
                    return Err(bad("an extrusion has no direction"));
                }
                Ok(Surface::Extrusion {
                    profile: Box::new(profile),
                    dir: d,
                })
            }
            // the profile swept about the axis (u the angle, v the profile's parameter)
            brep::Surface::Revolution { curve, origin, axis } => {
                let (profile, _) = self.curve(curve)?;
                let a = xf.unit(*axis);
                if a.len() < 0.5 {
                    return Err(bad("a surface of revolution has no axis"));
                }
                Ok(Surface::Revolution {
                    profile: Box::new(profile),
                    axis_o: xf.pt(*origin),
                    axis: a,
                })
            }
            brep::Surface::Offset { basis, distance } => {
                let base = self.surface(basis)?;
                Ok(Surface::Offset {
                    base: Box::new(base),
                    dist: distance * xf.s,
                })
            }
            _ => Err(why("unsupportedSurface", "its surface is not imported")),
        }
    }

    fn face(&mut self, shell: ShellId, fi: usize, tag: &str) -> Result<(), Why> {
        let nf = &self.src.faces[fi];
        let surface = self.surface(&self.src.surfaces[nf.surface])?;
        let sense = !nf.reversed;

        let mut loops: Vec<Vec<Ce>> = vec![];
        let mut pts: Vec<Vec3> = vec![];
        for lp in &nf.loops {
            let mut ring = vec![];
            for c in lp {
                let id = self.edge(c.edge)?;
                ring.push(Ce {
                    edge: id,
                    rev: !c.forward,
                    pole_jump: false,
                });
                let e = &self.out.edges[id.idx()];
                pts.push(self.out.vertices[e.v0.idx()].p);
                pts.push(self.out.vertices[e.v1.idx()].p);
            }
            if ring.is_empty() {
                return Err(why("geometry", "a face has an empty boundary"));
            }
            loops.push(ring);
        }
        let point_loops: Vec<VertexId> = nf.point_loops.iter().map(|&v| self.vertex(v)).collect();
        for p in &pts {
            let d = surface.distance(*p);
            if d > IMPORT_TOL {
                return Err(why(
                    "offSurface",
                    format!(
                        "a vertex of its boundary lies {d:.2e} mm off its {} surface",
                        surface.kind()
                    ),
                ));
            }
        }
        if loops.is_empty() && !matches!(surface, Surface::Sphere { .. } | Surface::Torus { .. }) {
            return Err(why("geometry", "a face has no boundary on a surface that is not closed"));
        }
        if loops.is_empty() {
            // a whole sphere or torus: no edge in the file, its seams are made as a primitive's
            return self.whole_face(shell, &surface, sense, tag, fi);
        }

        let loops = match self.bspline_pole_cap(&surface, &self.src.surfaces[nf.surface], &point_loops, &loops) {
            Some(l) => l,
            None => self.seam_wrapping(&surface, sense, loops, &point_loops)?,
        };
        let loops = self.order_planar(&surface, sense, loops)?;

        let name = FaceName::other("f3d", &format!("{tag}f{fi}"));
        let f = self.out.add_face(shell, surface.clone(), sense, name);
        let (mut pu, mut pv) = surface.periods();
        let closed_bs = match &self.src.surfaces[nf.surface] {
            brep::Surface::BSpline(b) if b.u_periodic || b.v_periodic || b.u_closed || b.v_closed => Some((b.u_periodic || b.u_closed, b.v_periodic || b.v_closed)),
            _ => None,
        };
        let mut seam_gap: Option<f64> = None;
        let ((du0, du1), (dv0, dv1)) = surface.domain();
        let (ext_u, ext_v) = ((du1 - du0).abs(), (dv1 - dv0).abs());
        // a closed B-spline surface wraps in its closed direction as a periodic one does: the coedges of a
        // loop are placed one after the other (a seam of the file is a coedge at the start or the end of the domain)
        if let Some((cu, cv)) = closed_bs {
            if cu {
                pu = Some(ext_u);
            }
            if cv {
                pv = Some(ext_v);
            }
        }
        let gap = |a: Vec2, b: Vec2| ((a.x - b.x).abs() / ext_u.max(1e-12)).max((a.y - b.y).abs() / ext_v.max(1e-12));
        for ring in loops {
            let lp = self.out.add_loop(f);
            let mut prev_end: Option<Vec2> = None;
            let mut loop_start: Option<Vec2> = None;
            for ce in ring {
                let e = &self.out.edges[ce.edge.idx()];
                let (t0, t1, curve) = (e.t0, e.t1, e.curve.clone());
                let mut pc = match self.out_pc(ce.edge) {
                    Some(c) => Some(c),
                    None => pcurve_of(&surface, &curve, t0, t1, 1e-6).ok(),
                };
                let (ts, te) = if ce.rev { (t1, t0) } else { (t0, t1) };
                if let Some(c) = pc.as_mut() {
                    if ce.pole_jump {
                        if let Some(ls) = loop_start {
                            let s = c.eval(te);
                            let du = pu.map_or(0.0, |p| ((ls.x - s.x) / p).round() * p);
                            let dv = pv.map_or(0.0, |p| ((ls.y - s.y) / p).round() * p);
                            if du != 0.0 || dv != 0.0 {
                                *c = shift_pc(c, du, dv);
                            }
                        }
                    } else if let Some(pe) = prev_end {
                        let s = c.eval(ts);
                        let du = pu.map_or(0.0, |p| ((pe.x - s.x) / p).round() * p);
                        let dv = pv.map_or(0.0, |p| ((pe.y - s.y) / p).round() * p);
                        if du != 0.0 || dv != 0.0 {
                            *c = shift_pc(c, du, dv);
                        }
                    }
                    if closed_bs.is_some() {
                        // (a coedge leaving a pole starts anywhere along the pole: not a junction)
                        if let (Some(pe), false) = (prev_end, ce.pole_jump) {
                            let g = gap(pe, c.eval(ts));
                            seam_gap = Some(seam_gap.unwrap_or(0.0).max(g));
                        }
                        // the placed coedge must lie in the surface's domain, not beyond its seam
                        let slack = 1e-6;
                        for q in [c.eval(ts), c.eval(te)] {
                            if q.x < du0.min(du1) - slack * ext_u || q.x > du0.max(du1) + slack * ext_u || q.y < dv0.min(dv1) - slack * ext_v || q.y > dv0.max(dv1) + slack * ext_v {
                                seam_gap = Some(seam_gap.unwrap_or(0.0).max(1.0));
                            }
                        }
                    }
                    if loop_start.is_none() {
                        loop_start = Some(c.eval(ts));
                    }
                    prev_end = Some(c.eval(te));
                }
                self.out.add_coedge(lp, ce.edge, ce.rev, pc);
            }
            if closed_bs.is_some() {
                if let (Some(a), Some(b)) = (loop_start, prev_end) {
                    seam_gap = Some(seam_gap.unwrap_or(0.0).max(gap(a, b)));
                }
            }
        }
        if let (Some((u, v)), Some(g)) = (closed_bs, seam_gap) {
            if g > 1e-2 {
                return Err(why(
                    "seam",
                    format!(
                        "its surface is a B-spline closed in {} and the face crosses or wraps the seam, which is not imported",
                        match (u, v) {
                            (true, true) => "u and v",
                            (true, false) => "u",
                            _ => "v",
                        }
                    ),
                ));
            }
        }
        Ok(())
    }

    fn out_pc(&self, e: EdgeId) -> Option<Curve2> {
        self.pc_override.get(&e).cloned()
    }

    /// A face of a closed B-spline surface (a revolved one) cut off by one closed edge that wraps the closed
    /// direction, the other end of the surface being a pole (all its control points in one): the loop is closed
    /// with the meridian to the pole and back, the pole itself staying implicit, as for a cone's cap.
    fn bspline_pole_cap(&mut self, surface: &Surface, src_surface: &brep::Surface, point_loops: &[VertexId], loops: &[Vec<Ce>]) -> Option<Vec<Vec<Ce>>> {
        let brep::Surface::BSpline(nb) = src_surface else { return None };
        let (cu, cv) = (nb.u_periodic || nb.u_closed, nb.v_periodic || nb.v_closed);
        let Surface::BSpline(bs) = surface else { return None };
        if cu == cv || loops.len() != 1 || loops[0].len() != 1 || !point_loops.is_empty() {
            return None;
        }
        let ce = &loops[0][0];
        let e = &self.out.edges[ce.edge.idx()];
        if e.v0 != e.v1 {
            return None;
        }
        let va = e.v0;
        let pa = self.vpos(va);
        let (uc, vc, _) = surface.closest_point(pa);
        let same = |pts: Vec<Vec3>| pts.windows(2).all(|w| w[0].dist(w[1]) <= 1e-9 * (1.0 + w[0].len()));
        let (nu, nv) = (bs.nu, bs.nv);
        let ((u0, u1), (v0, v1)) = bs.domain();
        // `along`: the coordinate that runs from the pole to the loop; `seam`: the one the loop wraps
        let (curve, a_lo, a_hi, a_loop, seam, pole_at_lo, pole_at_hi) = if cv {
            let row = |i: usize| (0..nv).map(|j| bs.ctrl[i * nv + j]).collect::<Vec<_>>();
            (bs.iso_v(vc), u0, u1, uc, vc, same(row(0)), same(row(nu - 1)))
        } else {
            let col = |j: usize| (0..nu).map(|i| bs.ctrl[i * nv + j]).collect::<Vec<_>>();
            (bs.iso_u(uc), v0, v1, vc, uc, same(col(0)), same(col(nv - 1)))
        };
        if pole_at_lo == pole_at_hi {
            return None;
        }
        let a_pole = if pole_at_lo { a_lo } else { a_hi };
        if (a_pole - a_loop).abs() < 1e-9 {
            return None;
        }
        let ppt = curve.eval(a_pole);
        let pole = self.out.add_vertex(ppt);
        let pole_first = a_pole < a_loop;
        let (t0, t1) = if pole_first { (a_pole, a_loop) } else { (a_loop, a_pole) };
        let (ev0, ev1) = if pole_first { (pole, va) } else { (va, pole) };
        let seam_edge = self.out.add_edge(Curve::BSpline(curve), t0, t1, ev0, ev1);
        let pc = if cv {
            Curve2::Line { o: Vec2::new(0.0, seam), d: Vec2::new(1.0, 0.0) }
        } else {
            Curve2::Line { o: Vec2::new(seam, 0.0), d: Vec2::new(0.0, 1.0) }
        };
        self.pc_override.insert(seam_edge, pc);
        let mut joined = loops[0].clone();
        joined.push(Ce { edge: seam_edge, rev: pole_first, pole_jump: false });
        joined.push(Ce { edge: seam_edge, rev: !pole_first, pole_jump: true });
        Some(vec![joined])
    }

    /// A face that is a whole sphere or torus (the file has no loop for it): made as the revolve of its
    /// meridian, which gives the seams and the poles, and copied into the shell.
    fn whole_face(&mut self, shell: ShellId, surface: &Surface, sense: bool, tag: &str, fi: usize) -> Result<(), Why> {
        use ck_topo::make::{Profile, ProfileLoop, ProfileRegion, ProfileSeg};
        if !sense {
            return Err(why("orientation", "a whole sphere or torus turned inside out is not imported"));
        }
        let (frame, seg_loop) = match surface {
            Surface::Sphere { frame, r } => (
                *frame,
                ProfileLoop::new(vec![
                    ProfileSeg::arc("m", Vec2::ZERO, Vec2::new(0.0, -*r), Vec2::new(0.0, *r)),
                    ProfileSeg::line("a", Vec2::new(0.0, *r), Vec2::new(0.0, -*r)),
                ]),
            ),
            Surface::Torus { frame, major, minor } => (*frame, ProfileLoop::new(vec![ProfileSeg::circle("m", Vec2::new(*major, 0.0), *minor)])),
            _ => unreachable!(),
        };
        let pf = ck_geom::Frame { o: frame.o, x: frame.x, y: frame.z, z: frame.x.cross(frame.z) };
        let profile = Profile::new(pf, vec![ProfileRegion { outer: seg_loop, holes: vec![] }]);
        let made = ck_topo::make::revolve::revolve("w", &profile, frame.o, frame.z, 0.0, TAU)
            .map_err(|e| why("geometry", format!("a whole sphere or torus could not be made: {e}")))?;
        let mut vmap: HashMap<VertexId, VertexId> = HashMap::new();
        let mut emap: HashMap<EdgeId, EdgeId> = HashMap::new();
        for f in made.face_ids() {
            let fc = made.face(f);
            let name = FaceName::other("f3d", &format!("{tag}f{fi}"));
            let nf = self.out.add_face(shell, fc.surface.clone(), fc.sense, name);
            for &lp in &fc.loops {
                let nl = self.out.add_loop(nf);
                for &c in &made.lp(lp).coedges {
                    let co = made.coedge(c);
                    let ne = match emap.get(&co.edge) {
                        Some(&e) => e,
                        None => {
                            let e = made.edge(co.edge);
                            let mut mv = |x: VertexId, out: &mut Body| *vmap.entry(x).or_insert_with(|| out.add_vertex(made.vertex(x).p));
                            let (v0, v1) = (mv(e.v0, self.out), mv(e.v1, self.out));
                            let ne = self.out.add_edge(e.curve.clone(), e.t0, e.t1, v0, v1);
                            emap.insert(co.edge, ne);
                            ne
                        }
                    };
                    self.out.add_coedge(nl, ne, co.rev, co.pcurve.clone());
                }
            }
        }
        Ok(())
    }

    /// Polyline resolution for winding and area estimates: splines may spiral many turns
    /// (threads), so sample them densely.
    fn samples(&self, ce: &Ce) -> usize {
        match &self.out.edges[ce.edge.idx()].curve {
            Curve::BSpline(_) => 1024,
            _ => 16,
        }
    }

    fn ce_points(&self, ce: &Ce, n: usize) -> Vec<Vec3> {
        let e = &self.out.edges[ce.edge.idx()];
        let (a, b) = if ce.rev { (e.t1, e.t0) } else { (e.t0, e.t1) };
        (0..=n)
            .map(|i| e.curve.eval(a + (b - a) * i as f64 / n as f64))
            .collect()
    }

    fn ce_start(&self, ce: &Ce) -> VertexId {
        let e = &self.out.edges[ce.edge.idx()];
        if ce.rev { e.v1 } else { e.v0 }
    }

    /// Total u-winding (in turns) of each loop on a u-periodic surface.
    fn windings(&self, surface: &Surface, loops: &[Vec<Ce>]) -> Vec<i32> {
        let frame = match surface {
            Surface::Cylinder { frame, .. }
            | Surface::Cone { frame, .. }
            | Surface::Sphere { frame, .. }
            | Surface::Torus { frame, .. } => frame,
            _ => return vec![0; loops.len()],
        };
        loops
            .iter()
            .map(|ring| {
                let mut total = 0.0;
                let mut last: Option<f64> = None;
                for ce in ring {
                    for p in self.ce_points(ce, self.samples(ce)) {
                        let d = p - frame.o;
                        if d.dot(frame.x).hypot(d.dot(frame.y)) < 1e-9 * (1.0 + d.len()) {
                            continue; // on the pole axis: the azimuth is undefined
                        }
                        let u = d.dot(frame.y).atan2(d.dot(frame.x));
                        if let Some(l) = last {
                            let mut du = u - l;
                            while du > PI {
                                du -= TAU;
                            }
                            while du < -PI {
                                du += TAU;
                            }
                            total += du;
                        }
                        last = Some(u);
                    }
                }
                (total / TAU).round() as i32
            })
            .collect()
    }

    /// ASM periodic faces have no seam edge: the boundary loops that wrap the axis are joined
    /// with a seam, or closed with one to the pole or apex.
    fn seam_wrapping(
        &mut self,
        surface: &Surface,
        sense: bool,
        mut loops: Vec<Vec<Ce>>,
        point_loops: &[VertexId],
    ) -> Result<Vec<Vec<Ce>>, Why> {
        if surface.periods().0.is_none() {
            return Ok(loops);
        }
        let w = self.windings(surface, &loops);
        let wrap: Vec<usize> = (0..loops.len()).filter(|&i| w[i] != 0).collect();
        if wrap.is_empty() {
            return Ok(loops);
        }
        let kind = surface.kind();
        match surface {
            Surface::Sphere { .. } => return self.sphere_seam(surface, sense, loops, &wrap, &w),
            Surface::Torus { .. } => {}
            Surface::Cylinder { .. } | Surface::Cone { .. } => {}
            _ => {
                return Err(why(
                    "seam",
                    format!("a {kind} face with a loop wrapping its axis is not imported"),
                ));
            }
        }
        if wrap.len() == 1 && matches!(surface, Surface::Cone { .. }) && w[wrap[0]].abs() == 1 {
            return self.cone_cap(surface, loops, wrap[0], point_loops);
        }
        if wrap.len() != 2 || w[wrap[0]].abs() != 1 || w[wrap[1]].abs() != 1 {
            return Err(why(
                "seam",
                format!(
                    "a {kind} face has {} loop(s) wrapping its axis; a seam needs exactly two",
                    wrap.len()
                ),
            ));
        }
        if w[wrap[0]] == w[wrap[1]] {
            return Err(why(
                "orientation",
                "the two loops that wrap a face's axis run the same way round it",
            ));
        }
        let (frame, torus) = match surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => (*frame, None),
            Surface::Torus { frame, major, minor } => (*frame, Some((*major, *minor))),
            _ => unreachable!(),
        };
        let a = loops[wrap[0]].clone();
        let mut bl = loops[wrap[1]].clone();
        let va = self.ce_start(&a[0]);
        let pa = self.vpos(va);
        // the azimuth of a point about the axis
        let azimuth = |p: Vec3| {
            let d = p - frame.o;
            d.dot(frame.y).atan2(d.dot(frame.x))
        };
        let ua = azimuth(pa);
        let k = (0..bl.len()).find(|&k| {
            let p = self.vpos(self.ce_start(&bl[k]));
            let mut du = (azimuth(p) - ua).abs() % TAU;
            if du > PI {
                du = TAU - du;
            }
            du < 1e-6
        });
        let Some(k) = k else {
            return Err(self.need_vertex_at(&bl, &frame, ua, kind));
        };
        bl.rotate_left(k);
        let vb = self.ce_start(&bl[0]);
        let pb = self.vpos(vb);
        let (seam, fwd_rev) = if let Some((major, minor)) = torus {
            self.torus_seam(&frame, major, minor, sense, &a[0], (va, pa), (vb, pb))?
        } else {
            let len = pa.dist(pb);
            if !(len > 1e-9) {
                return Err(why("seam", "the seam of a face would have no length"));
            }
            (self.out.add_edge(Curve::line_through(pa, pb), 0.0, len, va, vb), false)
        };
        let mut joined = a;
        joined.push(Ce {
            edge: seam,
            rev: fwd_rev,
            pole_jump: false,
        });
        joined.extend(bl);
        joined.push(Ce {
            edge: seam,
            rev: !fwd_rev,
            pole_jump: false,
        });
        let mut out = vec![joined];
        for (i, l) in loops.drain(..).enumerate() {
            if i != wrap[0] && i != wrap[1] {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// The two loops of a periodic face have no vertices at a common angle about the axis: asks
    /// for the edge of `ring` that crosses the angle `target` to be split there (the pass then
    /// runs again on the split body), or says that it cannot be.
    fn need_vertex_at(&mut self, ring: &[Ce], frame: &ck_geom::Frame, target: f64, kind: &str) -> Why {
        let azimuth = |p: Vec3| {
            let d = p - frame.o;
            d.dot(frame.y).atan2(d.dot(frame.x))
        };
        let wrap = |a: f64| {
            let mut d = (a - target) % TAU;
            if d > PI {
                d -= TAU;
            } else if d < -PI {
                d += TAU;
            }
            d
        };
        let neutral_of = |id: EdgeId| self.edges.iter().find(|(_, v)| **v == id).map(|(k, _)| *k);
        for ce in ring {
            let e = &self.out.edges[ce.edge.idx()];
            if matches!(e.curve, Curve::Line { .. }) {
                continue;
            }
            const N: usize = 96;
            let at = |i: usize| e.t0 + (e.t1 - e.t0) * i as f64 / N as f64;
            let d: Vec<f64> = (0..=N).map(|i| wrap(azimuth(e.curve.eval(at(i))))).collect();
            for i in 0..N {
                if d[i] * d[i + 1] <= 0.0 && d[i].abs() + d[i + 1].abs() < PI {
                    let (mut lo, mut hi) = (at(i), at(i + 1));
                    let sign = d[i] <= 0.0;
                    for _ in 0..100 {
                        let mid = 0.5 * (lo + hi);
                        if (wrap(azimuth(e.curve.eval(mid))) <= 0.0) == sign {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    let t = 0.5 * (lo + hi);
                    if t > e.t0 + 1e-9 * (e.t1 - e.t0) && t < e.t1 - 1e-9 * (e.t1 - e.t0) {
                        if let Some(n) = neutral_of(ce.edge) {
                            self.split = Some((n, t));
                            return why("split", "an edge is split so that the loops of a periodic face can be joined");
                        }
                    }
                }
            }
        }
        why(
            "seam",
            format!("the two loops of a {kind} face have no vertices at a common angle, and no edge of one crosses the other's angle to split there"),
        )
    }

    /// The meridian between two loops that wrap a torus: the arc of the tube circle that the
    /// face lies on. Returns the edge and whether the coedge from `a`'s vertex to `b`'s runs
    /// against it.
    #[allow(clippy::too_many_arguments)]
    fn torus_seam(
        &mut self,
        frame: &ck_geom::Frame,
        major: f64,
        minor: f64,
        sense: bool,
        first: &Ce,
        (va, pa): (VertexId, Vec3),
        (vb, pb): (VertexId, Vec3),
    ) -> Result<(EdgeId, bool), Why> {
        let axis = frame.z;
        let radial = |p: Vec3| {
            let d = p - frame.o;
            d - axis * d.dot(axis)
        };
        let (ra, rb) = (radial(pa), radial(pb));
        if ra.len() < 1e-9 || rb.len() < 1e-9 {
            return Err(why("seam", "a torus loop vertex lies on the torus axis"));
        }
        let e = ra.normalize();
        let centre = frame.o + e * major;
        let ang = |p: Vec3| (p - centre).dot(axis).atan2((p - centre).dot(e));
        let (ta, mut tb) = (ang(pa), ang(pb));
        // which way round the tube the face lies from `a`: to the left of the first coedge
        // against the face's material normal
        let pts = self.ce_points(first, 8);
        let tangent = pts[1] - pts[0];
        let nrm = {
            let n = (pa - centre).normalize();
            if sense { n } else { -n }
        };
        let into_face = nrm.cross(tangent);
        let tube_tangent = e * -ta.sin() + axis * ta.cos();
        let increasing = into_face.dot(tube_tangent) > 0.0;
        let circle = Curve::circle_x(centre, e.cross(axis), e, minor);
        if increasing {
            while tb <= ta + 1e-12 {
                tb += TAU;
            }
            if tb - ta > TAU - 1e-9 {
                return Err(why("seam", "the seam of a torus face would run all the way round the tube"));
            }
            Ok((self.out.add_edge(circle, ta, tb, va, vb), false))
        } else {
            while tb >= ta - 1e-12 {
                tb -= TAU;
            }
            if ta - tb > TAU - 1e-9 {
                return Err(why("seam", "the seam of a torus face would run all the way round the tube"));
            }
            Ok((self.out.add_edge(circle, tb, ta, vb, va), true))
        }
    }

    /// A cone cut off by one loop and closed at its apex: a seam line from the loop to the
    /// apex and back, the apex itself staying implicit.
    fn cone_cap(
        &mut self,
        surface: &Surface,
        loops: Vec<Vec<Ce>>,
        wrap: usize,
        point_loops: &[VertexId],
    ) -> Result<Vec<Vec<Ce>>, Why> {
        let Surface::Cone { frame, r, semi_angle } = surface else {
            unreachable!()
        };
        let ring = loops[wrap].clone();
        let va = self.ce_start(&ring[0]);
        let pa = self.vpos(va);
        let tan = semi_angle.tan();
        if tan.abs() < 1e-12 {
            return Err(why("seam", "a cone face has no apex"));
        }
        let apex_pos = frame.o - frame.z * (r / tan);
        let apex = match point_loops.first() {
            Some(v) if self.vpos(*v).dist(apex_pos) < 1e-4 + 1e-6 * apex_pos.len() => *v,
            _ => self.out.add_vertex(apex_pos),
        };
        let len = pa.dist(apex_pos);
        if !(len > 1e-9) {
            return Err(why("seam", "the loop of a cone cap lies on its apex"));
        }
        let seam = self
            .out
            .add_edge(Curve::line_through(apex_pos, pa), 0.0, len, apex, va);
        let mut joined = ring;
        joined.push(Ce {
            edge: seam,
            rev: true,
            pole_jump: false,
        });
        joined.push(Ce {
            edge: seam,
            rev: false,
            pole_jump: true,
        });
        let mut out = vec![joined];
        for (i, l) in loops.into_iter().enumerate() {
            if i != wrap {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// Sphere faces whose boundary circles wrap the pole axis: a cap (one wrapping loop; a
    /// meridian seam runs from the loop to the pole and back, the pole itself stays implicit as
    /// in a revolved sphere) or a band (two wrapping loops joined by a meridian arc).
    fn sphere_seam(
        &mut self,
        surface: &Surface,
        sense: bool,
        loops: Vec<Vec<Ce>>,
        wrap: &[usize],
        w: &[i32],
    ) -> Result<Vec<Vec<Ce>>, Why> {
        let Surface::Sphere { frame, r } = surface else {
            unreachable!()
        };
        let (c, axis) = (frame.o, frame.z);
        if wrap.iter().any(|&i| w[i].abs() != 1) || wrap.len() > 2 {
            return Err(why(
                "seam",
                format!(
                    "a sphere face has {} loop(s) wrapping its axis (winding {:?}), which is not imported",
                    wrap.len(),
                    wrap.iter().map(|&i| w[i]).collect::<Vec<_>>()
                ),
            ));
        }
        let azimuth = |p: Vec3| -> Result<Vec3, Why> {
            let d = p - c;
            let h = d - axis * d.dot(axis);
            if h.len() < 1e-9 * r.max(1.0) {
                return Err(why("seam", "a sphere loop vertex lies on the pole axis"));
            }
            Ok(h.normalize())
        };
        let mut out: Vec<Vec<Ce>> = vec![];
        if wrap.len() == 1 {
            let ring = loops[wrap[0]].clone();
            let va = self.ce_start(&ring[0]);
            let pa = self.vpos(va);
            let pts = self.ce_points(&ring[0], 8);
            let tan = pts[1] - pts[0];
            let nrm = if sense {
                (pa - c).normalize()
            } else {
                -(pa - c).normalize()
            };
            let north = nrm.cross(tan).dot(axis) > 0.0;
            let xdir = if north { axis } else { -axis };
            let e = azimuth(pa)?;
            let t0 = (pa - c).dot(e).atan2((pa - c).dot(xdir));
            if !(t0 > 1e-9) {
                return Err(why("seam", "a sphere cap's loop lies on the pole"));
            }
            let pole = self.out.add_vertex(c + xdir * *r);
            let seam = self
                .out
                .add_edge(Curve::circle_x(c, xdir.cross(e), xdir, *r), 0.0, t0, pole, va);
            let mut joined = ring;
            joined.push(Ce {
                edge: seam,
                rev: true,
                pole_jump: false,
            });
            joined.push(Ce {
                edge: seam,
                rev: false,
                pole_jump: true,
            });
            out.push(joined);
        } else {
            if w[wrap[0]] == w[wrap[1]] {
                return Err(why(
                    "orientation",
                    "the two loops that wrap a face's axis run the same way round it",
                ));
            }
            let a = loops[wrap[0]].clone();
            let mut bl = loops[wrap[1]].clone();
            let va = self.ce_start(&a[0]);
            let pa = self.vpos(va);
            let ea = azimuth(pa)?;
            let mut k = None;
            for i in 0..bl.len() {
                let p = self.vpos(self.ce_start(&bl[i]));
                if azimuth(p)?.dist(ea) < 1e-6 {
                    k = Some(i);
                    break;
                }
            }
            let Some(k) = k else {
                let ua = (pa - c).dot(frame.y).atan2((pa - c).dot(frame.x));
                return Err(self.need_vertex_at(&bl, frame, ua, "sphere"));
            };
            bl.rotate_left(k);
            let vb = self.ce_start(&bl[0]);
            let pb = self.vpos(vb);
            let ang = |p: Vec3| (p - c).dot(ea).atan2((p - c).dot(axis));
            let (ta, tb) = (ang(pa), ang(pb));
            if (ta - tb).abs() < 1e-9 {
                return Err(why("seam", "the seam of a sphere band would have no length"));
            }
            let curve = Curve::circle_x(c, axis.cross(ea), axis, *r);
            let (lo, hi, vlo, vhi) = if ta < tb { (ta, tb, va, vb) } else { (tb, ta, vb, va) };
            let seam = self.out.add_edge(curve, lo, hi, vlo, vhi);
            let fwd_rev = ta > tb; // the coedge va -> vb runs against the edge
            let mut joined = a;
            joined.push(Ce {
                edge: seam,
                rev: fwd_rev,
                pole_jump: false,
            });
            joined.extend(bl);
            joined.push(Ce {
                edge: seam,
                rev: !fwd_rev,
                pole_jump: false,
            });
            out.push(joined);
        }
        for (i, l) in loops.into_iter().enumerate() {
            if !wrap.contains(&i) {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// Planar faces: the outer loop first (positive area about the outward normal), holes after;
    /// a negative outer area is an orientation error.
    fn order_planar(
        &self,
        surface: &Surface,
        sense: bool,
        loops: Vec<Vec<Ce>>,
    ) -> Result<Vec<Vec<Ce>>, Why> {
        let Surface::Plane { frame } = surface else {
            return Ok(loops);
        };
        if loops.len() < 2 {
            return Ok(loops);
        }
        let n = if sense { frame.z } else { -frame.z };
        let mut areas: Vec<(f64, Vec<Ce>)> = loops
            .into_iter()
            .map(|ring| {
                let mut poly: Vec<Vec3> = vec![];
                for ce in &ring {
                    let mut p = self.ce_points(ce, self.samples(ce));
                    p.pop();
                    poly.extend(p);
                }
                let mut acc = Vec3::ZERO;
                for i in 0..poly.len() {
                    acc = acc + poly[i].cross(poly[(i + 1) % poly.len()]);
                }
                (0.5 * acc.dot(n), ring)
            })
            .collect();
        areas.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
        if let Some((a, _)) = areas.first() {
            if *a < 0.0 && a.abs() > 1e-9 {
                return Err(why(
                    "orientation",
                    "a planar face's boundary runs clockwise about its outward normal",
                ));
            }
        }
        Ok(areas.into_iter().map(|(_, r)| r).collect())
    }
}

fn shift_pc(pc: &Curve2, du: f64, dv: f64) -> Curve2 {
    let d = Vec2::new(du, dv);
    match pc {
        Curve2::Line { o, d: dd } => Curve2::Line { o: *o + d, d: *dd },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle {
            c: *c + d,
            x: *x,
            r: *r,
            ccw: *ccw,
        },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse {
            c: *c + d,
            x: *x,
            a: *a,
            b: *b,
            ccw: *ccw,
        },
        Curve2::BSpline(b) => Curve2::BSpline(b.map(|p| p + d)),
    }
}
