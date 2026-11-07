//! Phase B (part 2): the intermediate representation extracted from an XT node graph.
//!
//! Plain data in metres (the file's unit); [`crate::build`] scales to millimetres and builds the kernel body.
//! Topology: bodies -> regions -> shells -> faces -> loops -> fins (half-edges) -> edges -> vertices.
//! Geometry is stored once per XT node and referenced by index.

use crate::error::{Limits, SldprtError, R};
use crate::xt::{Node, XtFile};
use std::collections::HashMap;

pub type V3 = [f64; 3];

#[derive(Debug, Clone)]
pub struct NurbsCurveIr {
    pub degree: usize,
    /// 2 (SP-curve parameter space) or 3.
    pub dim: usize,
    /// Euclidean control points (weights divided out), `dim` components each.
    pub ctrl: Vec<Vec<f64>>,
    pub weights: Option<Vec<f64>>,
    /// Full knot vector (multiplicities expanded).
    pub knots: Vec<f64>,
    pub periodic: bool,
    pub closed: bool,
}

#[derive(Debug, Clone)]
pub struct NurbsSurfIr {
    pub du: usize,
    pub dv: usize,
    pub nu: usize,
    pub nv: usize,
    /// Control points, index `i * nv + j`.
    pub ctrl: Vec<V3>,
    pub weights: Option<Vec<f64>>,
    pub uknots: Vec<f64>,
    pub vknots: Vec<f64>,
    pub u_periodic: bool,
    pub v_periodic: bool,
}

#[derive(Debug, Clone)]
pub enum IrSurface {
    Plane { o: V3, n: V3, x: V3 },
    Cylinder { o: V3, axis: V3, r: f64, x: V3 },
    Cone { o: V3, axis: V3, r: f64, sin: f64, cos: f64, x: V3 },
    Sphere { c: V3, r: f64, axis: V3, x: V3 },
    Torus { c: V3, axis: V3, major: f64, minor: f64, x: V3 },
    Nurbs(NurbsSurfIr),
    /// Linear sweep of a section curve along a direction.
    Swept { section: Option<usize>, dir: V3 },
    /// Profile spun about an axis.
    Spun { profile: Option<usize>, base: V3, axis: V3 },
    Offset { base: Option<usize>, dist: f64 },
    /// Rolling-ball blend: the ball of the radius `range` rolls on the two supports along the spine.
    Blend { supports: [Option<usize>; 2], support_plus: [bool; 2], spine: Option<usize>, spine_plus: bool, range: [f64; 2], limits: [Option<V3>; 2] },
    Unsupported(String),
}

#[derive(Debug, Clone)]
pub struct IrSurfaceNode {
    pub id: i64,
    pub geom: IrSurface,
    /// The surface's own sense (`+` = natural normal).
    pub sense_plus: bool,
}

#[derive(Debug, Clone)]
pub enum IrCurve {
    Line { o: V3, d: V3 },
    Circle { c: V3, n: V3, x: V3, r: f64 },
    Ellipse { c: V3, n: V3, x: V3, a: f64, b: f64 },
    BCurve(NurbsCurveIr),
    Trimmed { basis: Option<usize>, p1: V3, p2: V3, t1: f64, t2: f64 },
    /// Surface/surface intersection given by its chart points (terminators included).
    /// `bounds`: a support that is a blend's boundary (BLEND_BOUND): the blend surface and which boundary.
    /// `chordal`: the chart's chordal error (metres): how far its points may be from the curve (0 when not stored).
    Intersection { pts: Vec<V3>, surfaces: [Option<usize>; 2], bounds: [Option<(usize, i64)>; 2], chordal: f64 },
    /// Curve in the parameter space of a surface.
    Sp { surface: Option<usize>, uv: Option<NurbsCurveIr> },
    Unsupported(String),
}

#[derive(Debug, Clone)]
pub struct IrCurveNode {
    pub id: i64,
    pub geom: IrCurve,
    pub sense_plus: bool,
}

#[derive(Debug, Clone)]
pub struct IrVertex {
    pub id: i64,
    pub p: V3,
    /// Tolerance of a tolerant vertex (metres): how far the vertex may be from the curves and surfaces that meet there.
    /// `None` when the vertex is exact (the file's null-tolerance sentinel).
    pub tol: Option<f64>,
}

#[derive(Debug, Clone)]
pub struct IrEdge {
    pub id: i64,
    /// `None`: tolerant edge (geometry only on the fins) or missing.
    pub curve: Option<usize>,
    /// Vertex at the start / end of the edge in its own direction; both `None` for a ring edge on a closed curve.
    pub start: Option<usize>,
    pub end: Option<usize>,
    /// Curve of a fin, used by tolerant edges.
    pub fin_curve: Option<usize>,
    /// Tolerance of a tolerant edge (metres): how far its curve may be from the faces that meet there. `None` when the
    /// edge is exact (the file's null-tolerance sentinel).
    pub tol: Option<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct IrFin {
    pub edge: usize,
    /// The fin runs along the edge's own direction.
    pub forward: bool,
}

#[derive(Debug, Clone)]
pub struct IrLoop {
    pub id: i64,
    pub fins: Vec<IrFin>,
    /// Isolated vertex loop.
    pub vertex: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct IrFace {
    pub id: i64,
    pub surface: Option<usize>,
    /// face.sense == surface.sense: the face normal equals the surface's natural normal.
    pub same_sense: bool,
    pub loops: Vec<IrLoop>,
    /// The face was reached through the shell's front-face chain (seen from its other side).
    pub back: bool,
    /// Face colour (attribute `SDL/TYSA_COLOUR`), RGB in 0..1.
    pub colour: Option<[f64; 3]>,
    /// The SolidWorks feature that made the face (attribute `FACE_ID_2001`): the smallest feature id of [`IrFace::sw_ids`].
    pub sw_id: Option<SwFaceId>,
    /// Every `FACE_ID_2001` tag on the face: a face two features share (a cut wall flush with a side, a side a later
    /// boss extended) carries one per feature.
    pub sw_ids: Vec<SwFaceId>,
}

/// SolidWorks' face tag `FACE_ID_2001`, integers `[kind, feature id, stamp, 0, local index, ...]`. The feature id is the id of
/// the feature that built the face, as listed in `swXmlContents/KeyWords`; `kind` and `local` are inferred from sample files
/// (kind 40 = an end cap, 74 = a side, 72 = a fillet face), not documented.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SwFaceId {
    pub kind: i64,
    pub feature: i64,
    pub local: Option<i64>,
}

#[derive(Debug, Clone, Default)]
pub struct IrShell {
    pub id: i64,
    pub faces: Vec<IrFace>,
}

#[derive(Debug, Clone, Default)]
pub struct IrRegion {
    pub solid: bool,
    pub shells: Vec<IrShell>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyKind {
    Solid,
    Wire,
    Sheet,
    General,
    Other(i64),
}

#[derive(Debug, Clone)]
pub struct IrBody {
    pub id: i64,
    pub kind: BodyKind,
    pub regions: Vec<IrRegion>,
    /// Density as stored on the body (attribute `SDL/TYSA_DENSITY`). Its unit is not established: the sample files hold 1000
    /// (SolidWorks 2011) or 1.0 (later versions) whatever the material, so the material's density is the one to use.
    pub density: Option<f64>,
    /// Id of the last feature that modified the body (attribute `LAST_BODY_MODIFYING_FEATURE_ID`).
    pub last_feature: Option<i64>,
}

#[derive(Debug, Clone, Default)]
pub struct Ir {
    pub schema: String,
    pub bodies: Vec<IrBody>,
    pub surfaces: Vec<IrSurfaceNode>,
    pub curves: Vec<IrCurveNode>,
    pub vertices: Vec<IrVertex>,
    pub edges: Vec<IrEdge>,
    pub notes: Vec<String>,
    /// Build the faces of sheet bodies too (set by the caller for a file that holds no solid; see `import_sldprt`).
    pub import_sheets: bool,
}

impl Ir {
    /// Faces of sheet bodies.
    pub fn sheet_faces(&self) -> usize {
        self.bodies.iter().filter(|b| b.kind == BodyKind::Sheet).flat_map(|b| &b.regions).flat_map(|r| &r.shells).map(|s| s.faces.len()).sum()
    }
    /// Number of sheet bodies that have faces.
    pub fn sheet_bodies(&self) -> usize {
        self.bodies.iter().filter(|b| b.kind == BodyKind::Sheet && b.regions.iter().any(|r| r.shells.iter().any(|s| !s.faces.is_empty()))).count()
    }

    /// Faces in solid regions of solid bodies.
    pub fn solid_faces(&self) -> usize {
        self.bodies.iter().filter(|b| b.kind == BodyKind::Solid).flat_map(|b| &b.regions).filter(|r| r.solid).flat_map(|r| &r.shells).map(|s| s.faces.len()).sum()
    }
}

struct Ex<'a> {
    xt: &'a XtFile,
    /// Attributes by owner node index: (definition name, field nodes).
    attrs: HashMap<i64, Vec<(String, Vec<i64>)>>,
    ir: Ir,
    surf: HashMap<i64, usize>,
    curve: HashMap<i64, usize>,
    vert: HashMap<i64, usize>,
    edge: HashMap<i64, usize>,
}

fn expand_knots(distinct: &[f64], mult: &[i64]) -> Vec<f64> {
    let mut out = vec![];
    for (k, m) in distinct.iter().zip(mult) {
        for _ in 0..(*m).clamp(0, 1 << 20) {
            out.push(*k);
        }
    }
    out
}

impl<'a> Ex<'a> {
    fn nurbs_curve(&self, c: &Node) -> R<NurbsCurveIr> {
        let xt = self.xt;
        let nb = xt.deref(c, "nurbs").ok_or_else(|| SldprtError::new("xt.geometry", "B-curve without NURBS data"))?;
        let deg = nb.i("degree").max(0) as usize;
        let nv = nb.i("n_vertices").max(0) as usize;
        let vdim = nb.i("vertex_dim").max(0) as usize;
        let rational = nb.b("rational");
        let verts = xt.deref(nb, "bspline_vertices").map(|n| n.af("vertices")).unwrap_or(&[]);
        let mult = xt.deref(nb, "knot_mult").map(|n| n.ai("mult")).unwrap_or(&[]);
        let kn = xt.deref(nb, "knots").map(|n| n.af("knots")).unwrap_or(&[]);
        let knots = expand_knots(kn, mult);
        if vdim < 2 || verts.len() < nv * vdim || knots.len() != nv + deg + 1 {
            return Err(SldprtError::new("xt.geometry", format!("inconsistent B-curve (degree {deg}, {nv} vertices of dim {vdim}, {} knots)", knots.len())));
        }
        let dim = if rational { vdim - 1 } else { vdim };
        let mut ctrl = vec![];
        let mut weights = if rational { Some(vec![]) } else { None };
        for i in 0..nv {
            let row = &verts[i * vdim..(i + 1) * vdim];
            if rational {
                let w = row[vdim - 1];
                if !(w > 0.0) {
                    return Err(SldprtError::new("xt.geometry", "non-positive NURBS weight"));
                }
                ctrl.push(row[..dim].iter().map(|x| x / w).collect());
                weights.as_mut().unwrap().push(w);
            } else {
                ctrl.push(row.to_vec());
            }
        }
        Ok(NurbsCurveIr { degree: deg, dim, ctrl, weights, knots, periodic: nb.b("periodic"), closed: nb.b("closed") })
    }

    fn nurbs_surface(&self, s: &Node) -> R<NurbsSurfIr> {
        let xt = self.xt;
        let nb = xt.deref(s, "nurbs").ok_or_else(|| SldprtError::new("xt.geometry", "B-surface without NURBS data"))?;
        let (p, q) = (nb.i("u_degree").max(0) as usize, nb.i("v_degree").max(0) as usize);
        let (nu, nv) = (nb.i("n_u_vertices").max(0) as usize, nb.i("n_v_vertices").max(0) as usize);
        let vdim = nb.i("vertex_dim").max(0) as usize;
        let rational = nb.b("rational");
        let verts = xt.deref(nb, "bspline_vertices").map(|n| n.af("vertices")).unwrap_or(&[]);
        let get = |kn: &str, mn: &str| -> Vec<f64> {
            let m = xt.deref(nb, mn).map(|n| n.ai("mult")).unwrap_or(&[]);
            let k = xt.deref(nb, kn).map(|n| n.af("knots")).unwrap_or(&[]);
            expand_knots(k, m)
        };
        let uknots = get("u_knots", "u_knot_mult");
        let vknots = get("v_knots", "v_knot_mult");
        if nu.checked_mul(nv).map_or(true, |t| t > 10_000_000) || vdim < 3 || verts.len() < nu * nv * vdim || uknots.len() != nu + p + 1 || vknots.len() != nv + q + 1 {
            return Err(SldprtError::new("xt.geometry", format!("inconsistent B-surface ({nu}x{nv} vertices, degrees {p},{q}, knots {},{})", uknots.len(), vknots.len())));
        }
        let mut ctrl = Vec::with_capacity(nu * nv);
        let mut weights = if rational { Some(Vec::with_capacity(nu * nv)) } else { None };
        for k in 0..nu * nv {
            let v = &verts[k * vdim..(k + 1) * vdim];
            if rational {
                let w = v[3.min(vdim - 1)];
                if !(w > 0.0) {
                    return Err(SldprtError::new("xt.geometry", "non-positive NURBS weight"));
                }
                ctrl.push([v[0] / w, v[1] / w, v[2] / w]);
                weights.as_mut().unwrap().push(w);
            } else {
                ctrl.push([v[0], v[1], v[2]]);
            }
        }
        Ok(NurbsSurfIr { du: p, dv: q, nu, nv, ctrl, weights, uknots, vknots, u_periodic: nb.b("u_periodic"), v_periodic: nb.b("v_periodic") })
    }

    fn surface(&mut self, idx: i64, depth: usize) -> Option<usize> {
        if let Some(&i) = self.surf.get(&idx) {
            return Some(i);
        }
        let n = self.xt.get(idx)?;
        let sense_plus = n.plus("sense");
        let geom = if depth > 32 {
            IrSurface::Unsupported("surface reference nesting too deep".into())
        } else {
            match n.ty {
                50 => IrSurface::Plane { o: n.v3("pvec"), n: n.v3("normal"), x: n.v3("x_axis") },
                51 => IrSurface::Cylinder { o: n.v3("pvec"), axis: n.v3("axis"), r: n.f("radius"), x: n.v3("x_axis") },
                52 => IrSurface::Cone { o: n.v3("pvec"), axis: n.v3("axis"), r: n.f("radius"), sin: n.f("sin_half_angle"), cos: n.f("cos_half_angle"), x: n.v3("x_axis") },
                53 => IrSurface::Sphere { c: n.v3("centre"), r: n.f("radius"), axis: n.v3("axis"), x: n.v3("x_axis") },
                54 => IrSurface::Torus { c: n.v3("centre"), axis: n.v3("axis"), major: n.f("major_radius"), minor: n.f("minor_radius"), x: n.v3("x_axis") },
                124 => match self.nurbs_surface(n) {
                    Ok(s) => IrSurface::Nurbs(s),
                    Err(e) => IrSurface::Unsupported(e.message),
                },
                67 => {
                    let sec = n.i("section");
                    IrSurface::Swept { section: self.curve(sec, depth + 1), dir: n.v3("sweep") }
                }
                68 => {
                    let pr = n.i("profile");
                    IrSurface::Spun { profile: self.curve(pr, depth + 1), base: n.v3("base"), axis: n.v3("axis") }
                }
                60 => {
                    let b = n.i("surface");
                    let base = self.surface(b, depth + 1);
                    let mut dist = n.f("offset");
                    if let Some(bn) = self.xt.get(b) {
                        if !bn.plus("sense") {
                            dist = -dist;
                        }
                    }
                    IrSurface::Offset { base, dist }
                }
                56 => {
                    let sv = n.ai("surface").to_vec();
                    let (mut supports, mut support_plus) = ([None, None], [true, true]);
                    for k in 0..2 {
                        if let Some(sn) = sv.get(k).and_then(|&s| self.xt.get(s)) {
                            support_plus[k] = sn.plus("sense");
                            // blend boundaries (59) are not evaluable supports; a rolling-ball blend ('R', or 'E' where the
                            // ball runs along a sharp edge) is: a blend on a blend has the other blend's own surface for a
                            // support (a zero-radius one is the edge itself)
                            if sn.ty != 59 && (sn.ty != 56 || matches!(sn.c("blend_type"), b'R' | b'E')) {
                                supports[k] = self.surface(sn.index, depth + 1);
                            }
                        }
                    }
                    let sp = n.i("spine");
                    let spine_plus = self.xt.get(sp).map_or(true, |c| c.plus("sense"));
                    let spine = self.curve(sp, depth + 1);
                    let r = n.af("range");
                    let range = if r.len() >= 2 { [r[0], r[1]] } else { [0.0, 0.0] };
                    let lim = |k: &str| self.xt.deref(n, k).and_then(|l| l.av("hvec").first().copied());
                    let limits = [lim("start"), lim("end")];
                    IrSurface::Blend { supports, support_plus, spine, spine_plus, range, limits }
                }
                t => IrSurface::Unsupported(format!("surface node type {} ({})", t, n.name())),
            }
        };
        self.ir.surfaces.push(IrSurfaceNode { id: n.i("node_id"), geom, sense_plus });
        let i = self.ir.surfaces.len() - 1;
        self.surf.insert(idx, i);
        Some(i)
    }

    fn curve(&mut self, idx: i64, depth: usize) -> Option<usize> {
        if let Some(&i) = self.curve.get(&idx) {
            return Some(i);
        }
        let n = self.xt.get(idx)?;
        let sense_plus = n.plus("sense");
        let geom = if depth > 32 {
            IrCurve::Unsupported("curve reference nesting too deep".into())
        } else {
            match n.ty {
                30 => IrCurve::Line { o: n.v3("pvec"), d: n.v3("direction") },
                31 => IrCurve::Circle { c: n.v3("centre"), n: n.v3("normal"), x: n.v3("x_axis"), r: n.f("radius") },
                32 => IrCurve::Ellipse { c: n.v3("centre"), n: n.v3("normal"), x: n.v3("x_axis"), a: n.f("major_radius"), b: n.f("minor_radius") },
                134 => match self.nurbs_curve(n) {
                    Ok(c) => IrCurve::BCurve(c),
                    Err(e) => IrCurve::Unsupported(e.message),
                },
                133 => {
                    let b = n.i("basis_curve");
                    IrCurve::Trimmed { basis: self.curve(b, depth + 1), p1: n.v3("point_1"), p2: n.v3("point_2"), t1: n.f("parm_1"), t2: n.f("parm_2") }
                }
                38 => {
                    let mut pts: Vec<V3> = vec![];
                    let chordal = self.xt.deref(n, "chart").map(|c| c.f("chordal_error")).filter(|e| e.is_finite() && *e > 0.0 && *e < 1.0).unwrap_or(0.0);
                    if let Some(ch) = self.xt.deref(n, "chart") {
                        pts.extend(ch.av("hvec").iter().copied());
                        let ends = |k: &str| self.xt.deref(n, k).filter(|l| matches!(l.c("type"), b'T' | b'L')).and_then(|l| l.av("hvec").first().copied());
                        let dist = |a: &V3, b: &V3| ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
                        if let Some(q) = ends("start") {
                            if pts.first().map_or(true, |p| dist(&q, p) > 1e-9) {
                                pts.insert(0, q);
                            }
                        }
                        if let Some(q) = ends("end") {
                            if pts.last().map_or(true, |p| dist(&q, p) > 1e-9) {
                                pts.push(q);
                            }
                        }
                        pts.dedup_by(|a, b| dist(a, b) <= 1e-10);
                    }
                    let sv = n.ai("surface").to_vec();
                    // the two surfaces; a BLEND_BOUND (type 59) stands in for a blend support and is not evaluated
                    let mut surfaces = [None, None];
                    let mut bounds = [None, None];
                    for (k, s) in sv.iter().take(2).enumerate() {
                        match self.xt.get(*s).map(|sn| (sn.ty, sn.i("boundary"), sn.i("blend"))) {
                            Some((59, side, blend)) => bounds[k] = self.surface(blend, depth + 1).map(|b| (b, side)),
                            Some(_) => surfaces[k] = self.surface(*s, depth + 1),
                            None => {}
                        }
                    }
                    if pts.len() < 2 {
                        IrCurve::Unsupported("intersection curve without chart points".into())
                    } else {
                        IrCurve::Intersection { pts, surfaces, bounds, chordal }
                    }
                }
                137 => {
                    let s = n.i("surface");
                    let uv = self.xt.deref(n, "b_curve").and_then(|b| self.nurbs_curve(b).ok());
                    IrCurve::Sp { surface: self.surface(s, depth + 1), uv }
                }
                t => IrCurve::Unsupported(format!("curve node type {} ({})", t, n.name())),
            }
        };
        self.ir.curves.push(IrCurveNode { id: n.i("node_id"), geom, sense_plus });
        let i = self.ir.curves.len() - 1;
        self.curve.insert(idx, i);
        Some(i)
    }

    /// IR curve `ci` (or the basis of a trimmed curve) is an SP-curve on a rolling-ball blend.
    fn sp_on_blend(&self, ci: usize) -> bool {
        match &self.ir.curves[ci].geom {
            IrCurve::Trimmed { basis: Some(b), .. } => self.sp_on_blend(*b),
            IrCurve::Sp { surface: Some(s), .. } => matches!(self.ir.surfaces[*s].geom, IrSurface::Blend { .. }),
            _ => false,
        }
    }

    fn vertex(&mut self, idx: i64) -> Option<usize> {
        if let Some(&i) = self.vert.get(&idx) {
            return Some(i);
        }
        let v = self.xt.get(idx)?;
        let p = self.xt.deref(v, "point")?;
        let tol = Some(v.f("tolerance")).filter(|t| t.is_finite() && *t > 0.0 && *t < 1.0);
        self.ir.vertices.push(IrVertex { id: v.i("node_id"), p: p.v3("pvec"), tol });
        let i = self.ir.vertices.len() - 1;
        self.vert.insert(idx, i);
        Some(i)
    }

    fn edge(&mut self, idx: i64) -> Option<usize> {
        if let Some(&i) = self.edge.get(&idx) {
            return Some(i);
        }
        let e = self.xt.get(idx)?;
        let fin = self.xt.deref(e, "halfedge");
        let other = fin.and_then(|f| self.xt.deref(f, "other"));
        // `fin.vertex` is the vertex at the end of the fin's own travel; the edge's start/end follow the '+' fin.
        let (va, vb) = (fin.map(|f| f.i("vertex")), other.map(|f| f.i("vertex")));
        let fin_plus = fin.map_or(true, |f| f.plus("sense"));
        let (s_idx, e_idx) = if fin_plus { (vb, va) } else { (va, vb) };
        let start = s_idx.and_then(|v| self.vertex(v));
        let end = e_idx.and_then(|v| self.vertex(v));
        let c = e.i("curve");
        let curve = self.curve(c, 0);
        // a tolerant edge's own geometry is on its fins: prefer a fin curve that is not an SP-curve on a blend (whose
        // parameters are the file's blend parameters, which an approximated blend surface does not share)
        let fin_curves: Vec<usize> = [fin, other].iter().flatten().map(|f| f.i("curve")).filter(|&c| c > 0).filter_map(|c| self.curve(c, 0)).collect();
        let fin_curve = fin_curves.iter().copied().find(|&c| !self.sp_on_blend(c)).or(fin_curves.first().copied());
        let tol = Some(e.f("tolerance")).filter(|t| t.is_finite() && *t > 0.0 && *t < 1.0);
        self.ir.edges.push(IrEdge { id: e.i("node_id"), curve, start, end, fin_curve, tol });
        let i = self.ir.edges.len() - 1;
        self.edge.insert(idx, i);
        Some(i)
    }

    fn face(&mut self, f: &Node, back: bool) -> R<IrFace> {
        let xt = self.xt;
        let surf_node = xt.deref(f, "surface");
        let same_sense = match surf_node {
            Some(s) => f.c("sense") == s.c("sense"),
            None => true,
        };
        let surface = surf_node.and_then(|s| self.surface(s.index, 0));
        let mut loops = vec![];
        for lp in xt.chain(xt.deref(f, "loop"), "next") {
            let mut fins = vec![];
            let mut vertex = None;
            let head = xt.deref(lp, "halfedge");
            let mut cur = head;
            let mut seen = std::collections::HashSet::new();
            while let Some(fin) = cur {
                if !seen.insert(fin.index) || fins.len() > 1_000_000 {
                    break;
                }
                match xt.deref(fin, "edge") {
                    Some(e) => {
                        let ei = self.edge(e.index).ok_or_else(|| SldprtError::new("xt.topology", "edge could not be read"))?;
                        // the edge's direction is the '+' fin's direction
                        let forward = fin.plus("sense");
                        fins.push(IrFin { edge: ei, forward });
                    }
                    None => {
                        vertex = self.vertex(fin.i("vertex"));
                    }
                }
                cur = xt.deref(fin, "forward");
            }
            loops.push(IrLoop { id: lp.i("node_id"), fins, vertex });
        }
        let colour = self.attr_reals(f.index, "SDL/TYSA_COLOUR").filter(|v| v.len() == 3 && v.iter().all(|c| c.is_finite())).map(|v| [v[0], v[1], v[2]]);
        // a face modified by later features carries several tags (most recent first): the creating feature is the
        // one with the smallest id
        let sw_ids: Vec<SwFaceId> = self.attr_ints_all(f.index, "FACE_ID_2001").into_iter().filter(|v| v.len() >= 2).map(|v| SwFaceId { kind: v[0], feature: v[1], local: v.get(4).copied() }).collect();
        let sw_id = sw_ids.iter().min_by_key(|t| t.feature).cloned();
        Ok(IrFace { id: f.i("node_id"), surface, same_sense, loops, back, colour, sw_id, sw_ids })
    }

    /// Field nodes of the first attribute named `name` on node `owner`.
    fn attr_fields(&self, owner: i64, name: &str) -> impl Iterator<Item = &Node> {
        let fields = self.attrs.get(&owner).and_then(|v| v.iter().find(|a| a.0 == name)).map(|a| a.1.as_slice()).unwrap_or(&[]);
        fields.iter().filter_map(|&q| self.xt.get(q))
    }

    /// The first integer field (`INT_VALUES`) of attribute `name` on `owner`.
    fn attr_ints(&self, owner: i64, name: &str) -> Option<Vec<i64>> {
        self.attr_fields(owner, name).find(|n| n.ty == 82).map(|n| n.ai("values").to_vec())
    }

    /// The first integer field of every attribute named `name` on `owner`.
    fn attr_ints_all(&self, owner: i64, name: &str) -> Vec<Vec<i64>> {
        let attrs = self.attrs.get(&owner).map(|v| v.as_slice()).unwrap_or(&[]);
        attrs.iter().filter(|a| a.0 == name).filter_map(|a| a.1.iter().filter_map(|&q| self.xt.get(q)).find(|n| n.ty == 82).map(|n| n.ai("values").to_vec())).collect()
    }

    /// The first real field (`REAL_VALUES`) of attribute `name` on `owner`.
    fn attr_reals(&self, owner: i64, name: &str) -> Option<Vec<f64>> {
        self.attr_fields(owner, name).find(|n| n.ty == 83).map(|n| n.af("values").to_vec())
    }
}

/// Every attribute (`ATTRIBUTE` node) by the node index of its owner, with its definition's name.
fn attributes(xt: &XtFile) -> HashMap<i64, Vec<(String, Vec<i64>)>> {
    let mut names: HashMap<i64, String> = HashMap::new();
    for n in xt.nodes.values().filter(|n| n.ty == 80) {
        let id = xt.deref(n, "identifier").map(|i| match i.val("string") {
            Some(crate::xt::Val::AS(b)) => b.iter().take_while(|&&c| c != 0).map(|&c| c as char).collect(),
            _ => String::new(),
        });
        names.insert(n.index, id.unwrap_or_default());
    }
    let mut out: HashMap<i64, Vec<(String, Vec<i64>)>> = HashMap::new();
    let mut list: Vec<&Node> = xt.nodes.values().filter(|n| n.ty == 81).collect();
    list.sort_by_key(|n| n.index);
    for a in list {
        let Some(name) = names.get(&a.i("definition")) else { continue };
        out.entry(a.i("owner")).or_default().push((name.clone(), a.ai("fields").to_vec()));
    }
    out
}

/// Build the IR of every body in an XT file.
pub fn extract(xt: &XtFile, limits: &Limits) -> R<Ir> {
    let mut ex = Ex { xt, attrs: attributes(xt), ir: Ir { schema: xt.schema_id.clone(), ..Default::default() }, surf: HashMap::new(), curve: HashMap::new(), vert: HashMap::new(), edge: HashMap::new() };
    let mut bodies: Vec<&Node> = xt.nodes.values().filter(|n| n.ty == 12).collect();
    bodies.sort_by_key(|n| n.index);
    let mut nfaces = 0usize;
    for b in bodies {
        let kind = match b.i("body_type") {
            1 => BodyKind::Solid,
            2 => BodyKind::Wire,
            3 => BodyKind::Sheet,
            6 => BodyKind::General,
            o => BodyKind::Other(o),
        };
        let mut regions = vec![];
        for r in xt.chain(xt.deref(b, "region"), "next") {
            let mut reg = IrRegion { solid: r.c("type") == b'S', shells: vec![] };
            for sh in xt.chain(xt.deref(r, "shell"), "next") {
                let mut shell = IrShell { id: sh.i("node_id"), faces: vec![] };
                if reg.solid {
                    let mut seen = std::collections::HashSet::new();
                    for f in xt.chain(xt.deref(sh, "face"), "next") {
                        seen.insert(f.index);
                        shell.faces.push(ex.face(f, false)?);
                    }
                    for f in xt.chain(xt.deref(sh, "front_face"), "next_front") {
                        if seen.insert(f.index) {
                            shell.faces.push(ex.face(f, true)?);
                        }
                    }
                    nfaces += shell.faces.len();
                    if nfaces > limits.max_faces {
                        return Err(SldprtError::new("limit.faces", format!("more than {} faces", limits.max_faces)));
                    }
                } else if kind == BodyKind::Sheet {
                    // a sheet body: its faces hang off the shell of its one (void) region. A face that cannot be read is
                    // left out here (the file may hold solids that are read all the same), and so are faces past the limit.
                    for f in xt.chain(xt.deref(sh, "face"), "next") {
                        if nfaces >= limits.max_faces {
                            break;
                        }
                        if let Ok(face) = ex.face(f, false) {
                            shell.faces.push(face);
                            nfaces += 1;
                        }
                    }
                }
                reg.shells.push(shell);
            }
            regions.push(reg);
        }
        let density = ex.attr_reals(b.index, "SDL/TYSA_DENSITY").and_then(|v| v.first().copied()).filter(|d| d.is_finite());
        let last_feature = ex.attr_ints(b.index, "LAST_BODY_MODIFYING_FEATURE_ID").and_then(|v| v.first().copied());
        ex.ir.bodies.push(IrBody { id: b.index, kind, regions, density, last_feature });
    }
    if ex.ir.bodies.is_empty() {
        return Err(SldprtError::new("xt.empty", "no BODY node in the partition"));
    }
    Ok(ex.ir)
}
