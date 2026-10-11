//! STEP (AP203/AP214/AP242 B-rep) reader: entity graph -> ck-topo `Body`.
//!
//! Adapted from brepkit v2.129.15 (MIT OR Apache-2.0), crates/io/src/step/reader.rs
//! (licence texts: crates/ck-io/third_party/brepkit/). Kept from brepkit: the
//! entity walk MANIFOLD_SOLID_BREP -> CLOSED_SHELL -> ADVANCED_FACE ->
//! FACE_BOUND -> EDGE_LOOP -> ORIENTED_EDGE -> EDGE_CURVE -> VERTEX_POINT, the
//! surface / curve entity set, vertex and edge de-duplication by entity id and
//! the B-spline knot-multiplicity / rational-weight decoding (see parse.rs).
//! Rewritten onto ck-topo / ck-geom: edges are bounded by `[t0, t1]` on the
//! carrier curve, loops carry pcurves, and faces on periodic surfaces get the
//! explicit seam / pole structure ck-topo's mass / tessellation / booleans
//! expect (STEP bounds a full cylinder by two bare circles).
//!
//! Units: the first LENGTH_UNIT / PLANE_ANGLE_UNIT of the file scales all
//! lengths to millimetres and angles to radians (SI prefixes and
//! CONVERSION_BASED_UNIT such as INCH / DEGREE).
//!
//! Unsupported input never panics: it returns an [`ImportError`] naming the
//! STEP entity id.

use super::parse::{default_knots, expand_knots, parse, Ent, ImportError, StepFile, Val};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::pcurve::{exact_pcurve, pcurve_of};
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, LumpId, ShellId, VertexId};
use std::collections::{HashMap, HashSet};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

type R<T> = Result<T, ImportError>;

/// Result of an import.
#[derive(Debug)]
pub struct Imported {
    pub body: Body,
    /// MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS count (= lumps).
    pub solids: usize,
    /// Length scale applied (file unit -> mm).
    pub length_scale: f64,
    /// Non-fatal problems (skipped degenerate loops, ...).
    pub warnings: Vec<ImportError>,
}

/// Parse `text` and build the solids it contains.
pub fn read_step(text: &str) -> R<Imported> {
    let file = parse(text)?;
    Rd::new(&file).run()
}

// ------------------------------------------------------------------ specs

#[derive(Clone)]
struct Co {
    edge: EdgeId,
    rev: bool,
    pc: Option<Curve2>,
}

struct FaceSpec {
    id: u32,
    surface: Surface,
    sense: bool,
    loops: Vec<Vec<Co>>,
}

struct Rd<'a> {
    f: &'a StepFile,
    b: Body,
    len: f64,
    ang: f64,
    verts: HashMap<u32, VertexId>,
    vgrid: HashMap<(i64, i64, i64), Vec<VertexId>>,
    /// STEP EDGE_CURVE id -> (edge, our v0 is the STEP start vertex); None =
    /// degenerate (dropped)
    edges: HashMap<u32, Option<(EdgeId, bool)>>,
    warnings: Vec<ImportError>,
    nface: usize,
}

const VTOL: f64 = 1e-6;
const CELL: f64 = 1e-4;

fn bad(id: u32, msg: impl Into<String>) -> ImportError {
    ImportError::new("badEntity", Some(id), msg)
}
fn unsupported(id: u32, what: &str) -> ImportError {
    ImportError::new("unsupported", Some(id), format!("{what} is not supported"))
}

impl<'a> Rd<'a> {
    fn new(f: &'a StepFile) -> Self {
        Rd {
            f,
            b: Body::new(),
            len: 1.0,
            ang: 1.0,
            verts: HashMap::new(),
            vgrid: HashMap::new(),
            edges: HashMap::new(),
            warnings: vec![],
            nface: 0,
        }
    }

    fn ent(&self, id: u32) -> R<&'a Ent> {
        self.f.ents.get(&id).ok_or_else(|| bad(id, "referenced entity not found"))
    }

    /// Args of a simple entity of one of `kinds`.
    fn simple(&self, id: u32, kinds: &[&str]) -> R<&'a [Val]> {
        let e = self.ent(id)?;
        if e.parts.len() == 1 && kinds.contains(&e.parts[0].name.as_str()) {
            Ok(&e.parts[0].args)
        } else {
            Err(bad(id, format!("expected {}, found {}", kinds.join("/"), e.kind())))
        }
    }

    // ------------------------------------------------------------- units

    /// Scale of a unit entity to its SI base unit (metre / radian).
    fn unit_si(&self, id: u32, depth: u32) -> Option<f64> {
        if depth > 8 {
            return None;
        }
        let e = self.f.ents.get(&id)?;
        if let Some(si) = e.part("SI_UNIT") {
            let prefix = match si.args.first() {
                Some(Val::Enum(p)) => match p.as_str() {
                    "EXA" => 1e18,
                    "PETA" => 1e15,
                    "TERA" => 1e12,
                    "GIGA" => 1e9,
                    "MEGA" => 1e6,
                    "KILO" => 1e3,
                    "HECTO" => 1e2,
                    "DECA" => 1e1,
                    "DECI" => 1e-1,
                    "CENTI" => 1e-2,
                    "MILLI" => 1e-3,
                    "MICRO" => 1e-6,
                    "NANO" => 1e-9,
                    "PICO" => 1e-12,
                    _ => 1.0,
                },
                _ => 1.0,
            };
            return Some(prefix);
        }
        if let Some(cb) = e.part("CONVERSION_BASED_UNIT") {
            let m = self.f.ents.get(&cb.args.get(1)?.as_ref()?)?;
            let p = m.parts.first()?;
            let v = p.args.first()?.as_f64()?;
            let u = p.args.get(1)?.as_ref()?;
            return Some(v * self.unit_si(u, depth + 1)?);
        }
        None
    }

    fn detect_units(&mut self) {
        let mut ids: Vec<&u32> = self.f.ents.keys().collect();
        ids.sort();
        let (mut got_len, mut got_ang) = (false, false);
        for id in ids {
            let e = &self.f.ents[id];
            if !got_len && e.has("LENGTH_UNIT") {
                if let Some(s) = self.unit_si(*id, 0) {
                    self.len = s * 1000.0;
                    got_len = true;
                }
            } else if !got_ang && e.has("PLANE_ANGLE_UNIT") {
                if let Some(s) = self.unit_si(*id, 0) {
                    self.ang = s;
                    got_ang = true;
                }
            }
        }
    }

    // ---------------------------------------------------------- geometry

    fn point(&self, id: u32) -> R<Vec3> {
        let a = self.simple(id, &["CARTESIAN_POINT"])?;
        let c = a.get(1).and_then(Val::as_list).ok_or_else(|| bad(id, "CARTESIAN_POINT without coordinates"))?;
        let g = |i: usize| c.get(i).and_then(Val::as_f64);
        match (g(0), g(1)) {
            (Some(x), Some(y)) => Ok(Vec3::new(x, y, g(2).unwrap_or(0.0)) * self.len),
            _ => Err(bad(id, "bad coordinates")),
        }
    }

    fn dir(&self, id: u32) -> R<Vec3> {
        let a = self.simple(id, &["DIRECTION"])?;
        let c = a.get(1).and_then(Val::as_list).ok_or_else(|| bad(id, "DIRECTION without ratios"))?;
        let g = |i: usize| c.get(i).and_then(Val::as_f64);
        let v = match (g(0), g(1)) {
            (Some(x), Some(y)) => Vec3::new(x, y, g(2).unwrap_or(0.0)),
            _ => return Err(bad(id, "bad direction ratios")),
        };
        v.try_normalize().ok_or_else(|| bad(id, "zero direction"))
    }

    fn axis2(&self, id: u32) -> R<Frame> {
        let a = self.simple(id, &["AXIS2_PLACEMENT_3D"])?;
        let o = self.point(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "placement without location"))?)?;
        let z = match a.get(2).and_then(Val::as_ref) {
            Some(r) => self.dir(r)?,
            None => Vec3::Z,
        };
        let x = match a.get(3).and_then(Val::as_ref) {
            Some(r) => self.dir(r)?,
            None => ck_geom::any_perp(z),
        };
        Ok(frame_from(o, z, x))
    }

    /// AXIS1_PLACEMENT -> (origin, unit axis).
    fn axis1(&self, id: u32) -> R<(Vec3, Vec3)> {
        let a = self.simple(id, &["AXIS1_PLACEMENT"])?;
        let o = self.point(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "placement without location"))?)?;
        let z = match a.get(2).and_then(Val::as_ref) {
            Some(r) => self.dir(r)?,
            None => Vec3::Z,
        };
        Ok((o, z))
    }

    fn real(&self, id: u32, v: Option<&Val>, what: &str) -> R<f64> {
        v.and_then(Val::as_f64).ok_or_else(|| bad(id, format!("{what} is not a number")))
    }

    fn curve(&self, id: u32) -> R<Curve> {
        let e = self.ent(id)?;
        if e.has("B_SPLINE_CURVE") || e.has("B_SPLINE_CURVE_WITH_KNOTS") || e.has("BEZIER_CURVE") || e.has("QUASI_UNIFORM_CURVE") || e.has("UNIFORM_CURVE") {
            return self.bspline_curve(id, e);
        }
        if e.parts.len() != 1 {
            return Err(unsupported(id, &format!("complex curve {}", e.kind())));
        }
        let p = &e.parts[0];
        let a = &p.args;
        match p.name.as_str() {
            "LINE" => {
                let o = self.point(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "LINE without point"))?)?;
                let v = a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "LINE without vector"))?;
                let va = self.simple(v, &["VECTOR"])?;
                let d = self.dir(va.get(1).and_then(Val::as_ref).ok_or_else(|| bad(v, "VECTOR without direction"))?)?;
                Ok(Curve::line(o, d))
            }
            "CIRCLE" => {
                let f = self.axis2(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "CIRCLE without placement"))?)?;
                let r = self.real(id, a.get(2), "radius")? * self.len;
                if r <= 0.0 {
                    return Err(bad(id, "non-positive radius"));
                }
                Ok(Curve::Circle { frame: f, r })
            }
            "ELLIPSE" => {
                let f = self.axis2(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "ELLIPSE without placement"))?)?;
                let (sa, sb) = (self.real(id, a.get(2), "semi axis 1")? * self.len, self.real(id, a.get(3), "semi axis 2")? * self.len);
                if sa <= 0.0 || sb <= 0.0 {
                    return Err(bad(id, "non-positive semi axis"));
                }
                Ok(Curve::Ellipse { frame: f, a: sa, b: sb })
            }
            "TRIMMED_CURVE" => {
                let basis = self.curve(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "TRIMMED_CURVE without basis"))?)?;
                // the edge's vertices decide the bounds; sense_agreement picks the direction
                if a.get(4).and_then(Val::as_bool) == Some(false) {
                    Ok(basis.reversed())
                } else {
                    Ok(basis)
                }
            }
            "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" | "BOUNDED_SURFACE_CURVE" => {
                self.curve(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "surface curve without 3D curve"))?)
            }
            k => Err(unsupported(id, k)),
        }
    }

    fn refs_list(&self, id: u32, v: Option<&Val>, what: &str) -> R<Vec<u32>> {
        v.and_then(Val::as_list)
            .and_then(|l| l.iter().map(Val::as_ref).collect::<Option<Vec<u32>>>())
            .ok_or_else(|| bad(id, format!("{what} is not a reference list")))
    }

    fn bspline_curve(&self, id: u32, e: &Ent) -> R<Curve> {
        // complex entities carry (degree, ctrl, ..) in the B_SPLINE_CURVE part
        // without a name; simple ones prepend the name.
        let (kind, base): (&str, &[Val]) = if e.parts.len() == 1 {
            (e.parts[0].name.as_str(), &e.parts[0].args[..])
        } else {
            let p = e.part("B_SPLINE_CURVE").ok_or_else(|| bad(id, "complex B-spline curve without B_SPLINE_CURVE"))?;
            ("B_SPLINE_CURVE", &p.args[..])
        };
        let off = usize::from(e.parts.len() == 1);
        let deg = base.get(off).and_then(Val::as_usize).ok_or_else(|| bad(id, "bad degree"))?;
        let cps = self.refs_list(id, base.get(off + 1), "control points")?;
        let ctrl: Vec<Vec3> = cps.iter().map(|&r| self.point(r)).collect::<R<_>>()?;
        let n = ctrl.len();
        let knots = if let Some(k) = e.part("B_SPLINE_CURVE_WITH_KNOTS") {
            // simple: (name, deg, cps, form, closed, self, mults, knots, spec); complex: (mults, knots, spec)
            let (m, v) = if e.parts.len() == 1 { (k.args.get(6), k.args.get(7)) } else { (k.args.first(), k.args.get(1)) };
            let (m, v) = (
                m.and_then(Val::as_list).ok_or_else(|| bad(id, "bad knot multiplicities"))?,
                v.and_then(Val::as_list).ok_or_else(|| bad(id, "bad knot values"))?,
            );
            expand_knots(m, v).ok_or_else(|| bad(id, "bad knot vector"))?
        } else {
            default_knots(if kind == "UNIFORM_CURVE" { "UNIFORM_CURVE" } else { "QUASI_UNIFORM_CURVE" }, n, deg)
        };
        let weights = match e.part("RATIONAL_B_SPLINE_CURVE") {
            Some(p) => {
                let w: Vec<f64> = p
                    .args
                    .first()
                    .and_then(Val::as_list)
                    .and_then(|l| l.iter().map(Val::as_f64).collect::<Option<Vec<f64>>>())
                    .ok_or_else(|| bad(id, "bad weights"))?;
                Some(w)
            }
            None => None,
        };
        let r = match weights {
            Some(w) => BSplineCurve::new_rational(deg, knots, ctrl, w),
            None => BSplineCurve::new(deg, knots, ctrl),
        };
        r.map(Curve::BSpline).map_err(|m| bad(id, format!("invalid B-spline curve: {m}")))
    }

    fn surface(&self, id: u32) -> R<Surface> {
        let e = self.ent(id)?;
        if e.has("B_SPLINE_SURFACE") || e.has("B_SPLINE_SURFACE_WITH_KNOTS") || e.has("BEZIER_SURFACE") || e.has("QUASI_UNIFORM_SURFACE") || e.has("UNIFORM_SURFACE") {
            return self.bspline_surface(id, e);
        }
        if e.parts.len() != 1 {
            return Err(unsupported(id, &format!("complex surface {}", e.kind())));
        }
        let p = &e.parts[0];
        let a = &p.args;
        let ax = |i: usize| -> R<Frame> { self.axis2(a.get(i).and_then(Val::as_ref).ok_or_else(|| bad(id, "surface without placement"))?) };
        match p.name.as_str() {
            "PLANE" => Ok(Surface::Plane { frame: ax(1)? }),
            "CYLINDRICAL_SURFACE" => {
                let r = self.real(id, a.get(2), "radius")? * self.len;
                if r <= 0.0 {
                    return Err(bad(id, "non-positive radius"));
                }
                Ok(Surface::Cylinder { frame: ax(1)?, r })
            }
            "CONICAL_SURFACE" => {
                let r = self.real(id, a.get(2), "radius")? * self.len;
                let sa = self.real(id, a.get(3), "semi angle")? * self.ang;
                if r < 0.0 || sa.abs() <= 1e-12 || sa.abs() >= FRAC_PI_2 {
                    return Err(bad(id, "invalid cone radius / semi angle"));
                }
                Ok(Surface::Cone { frame: ax(1)?, r, semi_angle: sa })
            }
            "SPHERICAL_SURFACE" => {
                let r = self.real(id, a.get(2), "radius")? * self.len;
                if r <= 0.0 {
                    return Err(bad(id, "non-positive radius"));
                }
                Ok(Surface::Sphere { frame: ax(1)?, r })
            }
            "TOROIDAL_SURFACE" => {
                let (m, n) = (self.real(id, a.get(2), "major radius")? * self.len, self.real(id, a.get(3), "minor radius")? * self.len);
                if m <= 0.0 || n <= 0.0 {
                    return Err(bad(id, "non-positive torus radius"));
                }
                Ok(Surface::Torus { frame: ax(1)?, major: m, minor: n })
            }
            "SURFACE_OF_REVOLUTION" => {
                let c = self.curve(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "revolution without curve"))?)?;
                let (o, z) = self.axis1(a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "revolution without axis"))?)?;
                Ok(Surface::Revolution { profile: Box::new(c), axis_o: o, axis: z })
            }
            "SURFACE_OF_LINEAR_EXTRUSION" => {
                let c = self.curve(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "extrusion without curve"))?)?;
                let v = a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "extrusion without vector"))?;
                let va = self.simple(v, &["VECTOR"])?;
                let d = self.dir(va.get(1).and_then(Val::as_ref).ok_or_else(|| bad(v, "VECTOR without direction"))?)?;
                Ok(Surface::Extrusion { profile: Box::new(c), dir: d })
            }
            "OFFSET_SURFACE" => {
                let base = self.surface(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "offset without basis"))?)?;
                Ok(Surface::Offset { base: Box::new(base), dist: self.real(id, a.get(2), "distance")? * self.len })
            }
            "RECTANGULAR_TRIMMED_SURFACE" => self.surface(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "trimmed surface without basis"))?),
            k => Err(unsupported(id, k)),
        }
    }

    fn bspline_surface(&self, id: u32, e: &Ent) -> R<Surface> {
        let (kind, base): (&str, &[Val]) = if e.parts.len() == 1 {
            (e.parts[0].name.as_str(), &e.parts[0].args[..])
        } else {
            let p = e.part("B_SPLINE_SURFACE").ok_or_else(|| bad(id, "complex B-spline surface without B_SPLINE_SURFACE"))?;
            ("B_SPLINE_SURFACE", &p.args[..])
        };
        let off = usize::from(e.parts.len() == 1);
        let du = base.get(off).and_then(Val::as_usize).ok_or_else(|| bad(id, "bad u degree"))?;
        let dv = base.get(off + 1).and_then(Val::as_usize).ok_or_else(|| bad(id, "bad v degree"))?;
        let rows = base.get(off + 2).and_then(Val::as_list).ok_or_else(|| bad(id, "bad control net"))?;
        let nu = rows.len();
        let mut ctrl = Vec::new();
        let mut nv = 0;
        for r in rows {
            let refs = self.refs_list(id, Some(r), "control net row")?;
            if nv != 0 && refs.len() != nv {
                return Err(bad(id, "ragged control net"));
            }
            nv = refs.len();
            for rf in refs {
                ctrl.push(self.point(rf)?);
            }
        }
        if nu == 0 || nv == 0 {
            return Err(bad(id, "empty control net"));
        }
        let (ku, kv) = if let Some(k) = e.part("B_SPLINE_SURFACE_WITH_KNOTS") {
            // simple: (name, du, dv, net, form, uc, vc, self, mu, mv, ku, kv, spec); complex: (mu, mv, ku, kv, spec)
            let o = if e.parts.len() == 1 { 8 } else { 0 };
            let l = |i: usize| k.args.get(o + i).and_then(Val::as_list).ok_or_else(|| bad(id, "bad knot data"));
            (
                expand_knots(l(0)?, l(2)?).ok_or_else(|| bad(id, "bad u knot vector"))?,
                expand_knots(l(1)?, l(3)?).ok_or_else(|| bad(id, "bad v knot vector"))?,
            )
        } else {
            let k = if kind == "UNIFORM_SURFACE" { "UNIFORM_SURFACE" } else { "QUASI_UNIFORM_SURFACE" };
            (default_knots(k, nu, du), default_knots(k, nv, dv))
        };
        let weights = match e.part("RATIONAL_B_SPLINE_SURFACE") {
            Some(p) => {
                let g = p.args.first().and_then(Val::as_list).ok_or_else(|| bad(id, "bad weight grid"))?;
                let mut w = Vec::new();
                for row in g {
                    for x in row.as_list().ok_or_else(|| bad(id, "bad weight row"))? {
                        w.push(x.as_f64().ok_or_else(|| bad(id, "bad weight"))?);
                    }
                }
                Some(w)
            }
            None => None,
        };
        BSplineSurface::new(du, dv, ku, kv, nu, nv, ctrl, weights)
            .map(|s| Surface::BSpline(Box::new(s)))
            .map_err(|m| bad(id, format!("invalid B-spline surface: {m}")))
    }

    // ---------------------------------------------------------- topology

    fn vertex_at(&mut self, p: Vec3) -> VertexId {
        let key = |p: Vec3| ((p.x / CELL).floor() as i64, (p.y / CELL).floor() as i64, (p.z / CELL).floor() as i64);
        let (kx, ky, kz) = key(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(l) = self.vgrid.get(&(kx + dx, ky + dy, kz + dz)) {
                        for &v in l {
                            if self.b.vertex(v).p.dist(p) <= VTOL * (1.0 + p.len()) {
                                return v;
                            }
                        }
                    }
                }
            }
        }
        let v = self.b.add_vertex(p);
        self.vgrid.entry((kx, ky, kz)).or_default().push(v);
        v
    }

    fn vertex_point(&mut self, id: u32) -> R<(VertexId, Vec3)> {
        if let Some(&v) = self.verts.get(&id) {
            return Ok((v, self.b.vertex(v).p));
        }
        let a = self.simple(id, &["VERTEX_POINT"])?;
        let p = self.point(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "VERTEX_POINT without point"))?)?;
        let v = self.vertex_at(p);
        self.verts.insert(id, v);
        Ok((v, p))
    }

    /// Curve parameter of a point known to lie on the curve.
    fn param(c: &Curve, p: Vec3) -> f64 {
        if let Curve::BSpline(bs) = c {
            let (d0, d1) = bs.domain();
            let tol = 1e-6 * (1.0 + p.len());
            if c.eval(d0).dist(p) < tol {
                return d0;
            }
            if c.eval(d1).dist(p) < tol {
                return d1;
            }
        }
        c.closest_param(p)
    }

    fn edge_curve(&mut self, id: u32) -> R<Option<(EdgeId, bool)>> {
        if let Some(r) = self.edges.get(&id) {
            return Ok(*r);
        }
        let a = self.simple(id, &["EDGE_CURVE"])?;
        let r = |i: usize| a.get(i).and_then(Val::as_ref).ok_or_else(|| bad(id, "EDGE_CURVE is missing a reference"));
        let (va, pa) = self.vertex_point(r(1)?)?;
        let (vb, pb) = self.vertex_point(r(2)?)?;
        let mut curve = self.curve(r(3)?)?;
        let same = a.get(4).and_then(Val::as_bool).unwrap_or(true);
        let (mut lo, mut hi, mut plo, mut phi) = if same { (va, vb, pa, pb) } else { (vb, va, pb, pa) };
        let mut lo_is_start = same;
        // degenerate edge (pole / apex): same vertex, no extent
        if lo == hi {
            let (t0, t1) = (Self::param(&curve, plo), curve.period().map_or(f64::NAN, |p| p));
            let closed_curve = matches!(curve, Curve::Circle { .. } | Curve::Ellipse { .. })
                || matches!(&curve, Curve::BSpline(bs) if curve.eval(bs.domain().0).dist(curve.eval(bs.domain().1)) < VTOL * (1.0 + plo.len()) && curve.length_between(bs.domain().0, bs.domain().1) > 1e-6);
            let _ = (t0, t1);
            if !closed_curve {
                self.edges.insert(id, None);
                return Ok(None);
            }
        }
        let mut t0 = Self::param(&curve, plo);
        let mut t1 = Self::param(&curve, phi);
        if lo == hi {
            match (&curve, curve.period()) {
                (_, Some(p)) => t1 = t0 + p,
                (Curve::BSpline(bs), None) => {
                    let (d0, d1) = bs.domain();
                    t0 = d0;
                    t1 = d1;
                }
                _ => return Err(bad(id, "closed edge on an open curve")),
            }
        } else if let Some(p) = curve.period() {
            while t1 <= t0 + 1e-12 {
                t1 += p;
            }
            while t1 - t0 > p + 1e-9 {
                t1 -= p;
            }
        } else if t1 <= t0 {
            // runs against the carrier direction: flip the carrier
            curve = curve.reversed();
            std::mem::swap(&mut lo, &mut hi);
            std::mem::swap(&mut plo, &mut phi);
            lo_is_start = !lo_is_start;
            t0 = Self::param(&curve, plo);
            t1 = Self::param(&curve, phi);
            if t1 <= t0 {
                return Err(bad(id, "edge has zero extent on its curve"));
            }
        }
        let e = self.b.add_edge(curve, t0, t1, lo, hi);
        self.edges.insert(id, Some((e, lo_is_start)));
        Ok(Some((e, lo_is_start)))
    }

    fn edge_loop(&mut self, id: u32) -> R<Option<Vec<Co>>> {
        let e = self.ent(id)?;
        let kind = e.kind();
        if kind == "VERTEX_LOOP" {
            return Ok(None);
        }
        let a = self.simple(id, &["EDGE_LOOP"]).map_err(|_| unsupported(id, kind))?;
        let oes = self.refs_list(id, a.get(1), "edge list")?;
        let mut out = Vec::new();
        for oe in oes {
            let oa = self.simple(oe, &["ORIENTED_EDGE"])?;
            let ec = oa.get(3).and_then(Val::as_ref).ok_or_else(|| bad(oe, "ORIENTED_EDGE without edge"))?;
            let orient = oa.get(4).and_then(Val::as_bool).unwrap_or(true);
            // an ORIENTED_EDGE may wrap another ORIENTED_EDGE
            let (ec, orient) = match self.ent(ec)?.kind() {
                "ORIENTED_EDGE" => {
                    let ia = self.simple(ec, &["ORIENTED_EDGE"])?;
                    (
                        ia.get(3).and_then(Val::as_ref).ok_or_else(|| bad(ec, "ORIENTED_EDGE without edge"))?,
                        orient == ia.get(4).and_then(Val::as_bool).unwrap_or(true),
                    )
                }
                _ => (ec, orient),
            };
            if let Some((edge, lo_is_start)) = self.edge_curve(ec)? {
                out.push(Co { edge, rev: orient != lo_is_start, pc: None });
            }
        }
        Ok(Some(out))
    }

    fn face_spec(&mut self, id: u32) -> R<FaceSpec> {
        let a = self.simple(id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
        let bounds = self.refs_list(id, a.get(1), "bounds")?;
        let surface = self.surface(a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "face without surface"))?)?;
        let sense = a.get(3).and_then(Val::as_bool).unwrap_or(true);
        let mut loops = Vec::new();
        let mut outer = None;
        for (bi, bid) in bounds.iter().enumerate() {
            let ba = self.simple(*bid, &["FACE_BOUND", "FACE_OUTER_BOUND"])?;
            let lid = ba.get(1).and_then(Val::as_ref).ok_or_else(|| bad(*bid, "bound without loop"))?;
            let orient = ba.get(2).and_then(Val::as_bool).unwrap_or(true);
            let Some(mut l) = self.edge_loop(lid)? else {
                self.warnings.push(ImportError::new("skipped", Some(lid), "vertex loop ignored"));
                continue;
            };
            if l.is_empty() {
                self.warnings.push(ImportError::new("skipped", Some(lid), "loop with no non-degenerate edges ignored"));
                continue;
            }
            if !orient {
                l.reverse();
                for c in &mut l {
                    c.rev = !c.rev;
                }
            }
            if self.ent(*bid)?.kind() == "FACE_OUTER_BOUND" && outer.is_none() {
                outer = Some(bi.min(loops.len()));
            }
            loops.push(l);
        }
        if let Some(o) = outer {
            if o < loops.len() && o != 0 {
                loops.swap(0, o);
            }
        }
        Ok(FaceSpec { id, surface, sense, loops })
    }

    // ------------------------------------------------------------ driver

    fn run(mut self) -> R<Imported> {
        self.detect_units();
        let mut solids: Vec<u32> = self
            .f
            .ents
            .iter()
            .filter(|(_, e)| e.parts.len() == 1 && matches!(e.kind(), "MANIFOLD_SOLID_BREP" | "BREP_WITH_VOIDS"))
            .map(|(id, _)| *id)
            .collect();
        solids.sort();
        if solids.is_empty() {
            return Err(ImportError::new("noSolid", None, "no MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS in the file"));
        }
        for (k, sid) in solids.iter().enumerate() {
            self.solid(*sid, k)?;
        }
        let n = solids.len();
        Ok(Imported { body: self.b, solids: n, length_scale: self.len, warnings: self.warnings })
    }

    fn shell_faces(&self, id: u32) -> R<Vec<u32>> {
        let e = self.ent(id)?;
        match e.kind() {
            "CLOSED_SHELL" | "OPEN_SHELL" => {
                let a = self.simple(id, &["CLOSED_SHELL", "OPEN_SHELL"])?;
                self.refs_list(id, a.get(1), "face list")
            }
            k => Err(unsupported(id, k)),
        }
    }

    fn solid(&mut self, sid: u32, k: usize) -> R<()> {
        let e = self.ent(sid)?;
        let a = &e.parts[0].args;
        let outer = a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(sid, "solid without shell"))?;
        // (shell id, flipped)
        let mut shells = vec![(outer, false)];
        if e.kind() == "BREP_WITH_VOIDS" {
            for v in self.refs_list(sid, a.get(2), "void list")? {
                let ve = self.ent(v)?;
                if ve.kind() == "ORIENTED_CLOSED_SHELL" {
                    let va = &ve.parts[0].args;
                    let inner = va.get(2).and_then(Val::as_ref).ok_or_else(|| bad(v, "ORIENTED_CLOSED_SHELL without shell"))?;
                    let o = va.get(3).and_then(Val::as_bool).unwrap_or(true);
                    shells.push((inner, !o));
                } else {
                    shells.push((v, false));
                }
            }
        }
        let lump = self.b.add_lump();
        let feature = format!("I{}", k + 1);
        let mut shell_ids: Vec<(ShellId, bool)> = vec![];
        for (shid, flipped) in shells {
            let shell = self.b.add_shell(lump);
            let mut specs = Vec::new();
            for fid in self.shell_faces(shid)? {
                specs.push(self.face_spec(fid)?);
            }
            self.merge_full_periodic(&mut specs);
            for s in specs {
                self.nface += 1;
                let name = FaceName::other(&feature, &format!("F{}", self.nface));
                self.finalize_face(shell, s, name)?;
            }
            shell_ids.push((shell, flipped));
        }
        self.orient_shells(lump, &shell_ids);
        Ok(())
    }

    /// Outer shell outward (positive volume), void shells inward (negative).
    fn orient_shells(&mut self, lump: LumpId, shells: &[(ShellId, bool)]) {
        let _ = lump;
        for (i, &(s, flipped)) in shells.iter().enumerate() {
            let faces: Vec<FaceId> = self.b.shell(s).faces.clone();
            if flipped {
                for &f in &faces {
                    ck_topo::orient::flip_face(&mut self.b, f);
                }
            }
            let vol = ck_topo::mass::signed_volume(&self.b, &faces);
            let want_pos = i == 0;
            if vol.is_finite() && (vol > 0.0) != want_pos && vol.abs() > 1e-12 {
                for &f in &faces {
                    ck_topo::orient::flip_face(&mut self.b, f);
                }
            }
        }
    }
}

// ------------------------------------------------------- periodic faces

fn translate(pc: &Curve2, d: Vec2) -> Curve2 {
    match pc {
        Curve2::Line { o, d: dd } => Curve2::Line { o: *o + d, d: *dd },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
        Curve2::BSpline(b) => Curve2::BSpline(b.map(|p| p + d)),
    }
}

fn same_surface(a: &Surface, b: &Surface) -> bool {
    let fr = |f: &Frame, g: &Frame| f.o.dist(g.o) < 1e-7 && f.z.dot(g.z) > 1.0 - 1e-12 && f.x.dot(g.x) > 1.0 - 1e-9;
    match (a, b) {
        (Surface::Sphere { frame: f, r }, Surface::Sphere { frame: g, r: s }) => fr(f, g) && (r - s).abs() < 1e-7,
        (Surface::Torus { frame: f, major, minor }, Surface::Torus { frame: g, major: m2, minor: n2 }) => {
            fr(f, g) && (major - m2).abs() < 1e-7 && (minor - n2).abs() < 1e-7
        }
        _ => false,
    }
}

/// Seam isoline of a periodic surface: coordinate `d` (0 = u, 1 = v) fixed at
/// `s`; the other coordinate is `o_ref + k (t - t_ref)` along `curve`.
struct Iso {
    curve: Curve,
    k: f64,
}

fn radial(f: &Frame, u: f64) -> Vec3 {
    f.x * u.cos() + f.y * u.sin()
}

fn iso(surf: &Surface, d: usize, s: f64) -> Option<Iso> {
    match (surf, d) {
        (Surface::Cylinder { frame, .. }, 0) => {
            let p = surf.eval(s, 0.0);
            Some(Iso { curve: Curve::Line { o: p, d: frame.z }, k: 1.0 })
        }
        (Surface::Cone { .. }, 0) => {
            let (p0, p1) = (surf.eval(s, 0.0), surf.eval(s, 1.0));
            let l = p0.dist(p1);
            Some(Iso { curve: Curve::Line { o: p0, d: (p1 - p0) / l }, k: 1.0 / l })
        }
        (Surface::Sphere { frame, r }, 0) => {
            let x = radial(frame, s);
            Some(Iso { curve: Curve::Circle { frame: Frame { o: frame.o, x, y: frame.z, z: x.cross(frame.z) }, r: *r }, k: 1.0 })
        }
        (Surface::Torus { frame, major, minor }, 0) => {
            let x = radial(frame, s);
            let f = Frame { o: frame.o + x * *major, x, y: frame.z, z: x.cross(frame.z) };
            Some(Iso { curve: Curve::Circle { frame: f, r: *minor }, k: 1.0 })
        }
        (Surface::Torus { frame, major, minor }, 1) => {
            let r = major + minor * s.cos();
            if r <= 1e-9 {
                return None;
            }
            let f = Frame { o: frame.o + frame.z * (minor * s.sin()), x: frame.x, y: frame.y, z: frame.z };
            Some(Iso { curve: Curve::Circle { frame: f, r }, k: 1.0 })
        }
        (Surface::Revolution { profile, axis_o, axis }, 0) => {
            let a = axis.normalize();
            let (sn, cs) = s.sin_cos();
            let rot = move |v: Vec3| v * cs + a.cross(v) * sn + a * (a.dot(v) * (1.0 - cs));
            let o = *axis_o;
            let c = profile.transform(move |p| o + rot(p - o), move |v| rot(v));
            Some(Iso { curve: c, k: 1.0 })
        }
        _ => None,
    }
}

/// Line pcurve of an isoline over the edge parameter range.
fn iso_pc(d: usize, s: f64, k: f64, o_at_t0: f64, t0: f64) -> Curve2 {
    let o = o_at_t0 - k * t0;
    if d == 0 {
        Curve2::Line { o: Vec2::new(s, o), d: Vec2::new(0.0, k) }
    } else {
        Curve2::Line { o: Vec2::new(o, s), d: Vec2::new(k, 0.0) }
    }
}

fn comp(v: Vec2, d: usize) -> f64 {
    if d == 0 {
        v.x
    } else {
        v.y
    }
}
fn with(v: Vec2, d: usize, x: f64) -> Vec2 {
    if d == 0 {
        Vec2::new(x, v.y)
    } else {
        Vec2::new(v.x, x)
    }
}
fn unit2(d: usize, x: f64) -> Vec2 {
    if d == 0 {
        Vec2::new(x, 0.0)
    } else {
        Vec2::new(0.0, x)
    }
}

/// Exact pcurves beyond `ck_geom::pcurve::exact_pcurve`: meridian circles of
/// a torus and arcs of great circles through the poles of a sphere.
fn exact_ext(surf: &Surface, c: &Curve, t0: f64, t1: f64) -> Option<Curve2> {
    let Curve::Circle { frame: cf, r } = c else { return None };
    let tol = 1e-7;
    match surf {
        Surface::Torus { frame, major, minor } => {
            if (r - minor).abs() > tol * (1.0 + r) || cf.z.dot(frame.z).abs() > tol {
                return None;
            }
            let q = cf.o - frame.o;
            let (qx, qy, qz) = (q.dot(frame.x), q.dot(frame.y), q.dot(frame.z));
            let ring = (qx * qx + qy * qy).sqrt();
            if qz.abs() > tol * (1.0 + major) || (ring - major).abs() > tol * (1.0 + major) {
                return None;
            }
            let rh = (frame.x * qx + frame.y * qy) / ring;
            if cf.z.dot(rh).abs() > tol {
                return None;
            }
            let u0 = qy.atan2(qx);
            let x2 = Vec2::new(cf.x.dot(rh), cf.x.dot(frame.z));
            let y2 = Vec2::new(cf.y.dot(rh), cf.y.dot(frame.z));
            let sgn = if x2.cross(y2) >= 0.0 { 1.0 } else { -1.0 };
            let phi = x2.y.atan2(x2.x);
            Some(Curve2::Line { o: Vec2::new(u0, phi), d: Vec2::new(0.0, sgn) })
        }
        Surface::Sphere { frame, r: sr } => {
            if (r - sr).abs() > tol * (1.0 + r) || cf.o.dist(frame.o) > tol * (1.0 + r) || cf.z.dot(frame.z).abs() > tol {
                return None;
            }
            let h0 = frame.z.cross(cf.z).normalize();
            let pm = cf.o + (cf.x * (0.5 * (t0 + t1)).cos() + cf.y * (0.5 * (t0 + t1)).sin()) * *r;
            let h = if h0.dot(pm - frame.o) >= 0.0 { h0 } else { -h0 };
            let u0 = h.dot(frame.y).atan2(h.dot(frame.x));
            let x2 = Vec2::new(cf.x.dot(h), cf.x.dot(frame.z));
            let y2 = Vec2::new(cf.y.dot(h), cf.y.dot(frame.z));
            let sgn = if x2.cross(y2) >= 0.0 { 1.0 } else { -1.0 };
            let phi = x2.y.atan2(x2.x);
            // v must stay on one side of the poles over the range
            let (va, vb) = (phi + sgn * t0, phi + sgn * t1);
            let wrap = |v: f64| (v + PI).rem_euclid(TAU) - PI;
            let (wa, wb) = (wrap(va), wrap(vb));
            let ok = (wa.abs() <= FRAC_PI_2 + 1e-9) && (wb.abs() <= FRAC_PI_2 + 1e-9) && ((wb - wa) - (vb - va)).abs() < 1e-9;
            ok.then(|| Curve2::Line { o: Vec2::new(u0, phi + (wa - va)), d: Vec2::new(0.0, sgn) })
        }
        _ => None,
    }
}

fn edge_pcurve(b: &Body, e: EdgeId, surf: &Surface) -> Option<Curve2> {
    let ed = b.edge(e);
    if let Some(p) = exact_ext(surf, &ed.curve, ed.t0, ed.t1) {
        return Some(p);
    }
    if let Some(p) = exact_pcurve(surf, &ed.curve, 1e-7) {
        return Some(p);
    }
    pcurve_of(surf, &ed.curve, ed.t0, ed.t1, 1e-6).ok()
}

impl Rd<'_> {
    /// Groups of faces that together cover a whole sphere / torus (STEP
    /// writers split them in halves) -> one face with the seam structure
    /// ck-topo uses.
    fn merge_full_periodic(&mut self, specs: &mut Vec<FaceSpec>) {
        let n = specs.len();
        let mut used = vec![false; n];
        let mut out: Vec<FaceSpec> = vec![];
        for i in 0..n {
            if used[i] || !matches!(specs[i].surface, Surface::Sphere { .. } | Surface::Torus { .. }) {
                continue;
            }
            let group: Vec<usize> = (i..n).filter(|&j| !used[j] && same_surface(&specs[i].surface, &specs[j].surface)).collect();
            if group.len() < 2 {
                continue;
            }
            // every edge of the group is used exactly twice, by group faces only
            let mut cnt: HashMap<EdgeId, usize> = HashMap::new();
            for &j in &group {
                for l in &specs[j].loops {
                    for c in l {
                        *cnt.entry(c.edge).or_default() += 1;
                    }
                }
            }
            let in_group: HashSet<usize> = group.iter().copied().collect();
            let mut outside: HashSet<EdgeId> = HashSet::new();
            for (j, s) in specs.iter().enumerate() {
                if !in_group.contains(&j) {
                    for l in &s.loops {
                        outside.extend(l.iter().map(|c| c.edge));
                    }
                }
            }
            if cnt.is_empty() || cnt.iter().any(|(e, &c)| c != 2 || outside.contains(e)) {
                continue;
            }
            for &j in &group {
                used[j] = true;
            }
            for e in cnt.keys() {
                self.b.edges[e.idx()].alive = false;
            }
            let (surface, sense, id) = (specs[i].surface.clone(), specs[i].sense, specs[i].id);
            if let Some(loop_) = self.full_periodic_loop(&surface, sense) {
                out.push(FaceSpec { id, surface, sense, loops: vec![loop_] });
            }
        }
        let mut k = 0;
        specs.retain(|_| {
            k += 1;
            !used[k - 1]
        });
        specs.extend(out);
    }

    /// Loop of a full sphere (`[S, S^-1]`, pole-to-pole meridian; the pole
    /// jumps close it in uv) or torus (`[R, S, R^-1, S^-1]`, both seams), CCW
    /// in uv for `sense` (the "up" leg on the larger u).
    fn full_periodic_loop(&mut self, surf: &Surface, sense: bool) -> Option<Vec<Co>> {
        let line = |o: Vec2, d: Vec2| Curve2::Line { o, d };
        let mut l = match surf {
            Surface::Sphere { frame, r } => {
                let c = iso(surf, 0, 0.0)?.curve;
                let (s, n) = (frame.o - frame.z * *r, frame.o + frame.z * *r);
                let (vs, vn) = (self.vertex_at(s), self.vertex_at(n));
                let e = self.b.add_edge(c, -FRAC_PI_2, FRAC_PI_2, vs, vn);
                vec![
                    Co { edge: e, rev: false, pc: Some(line(Vec2::new(TAU, 0.0), Vec2::new(0.0, 1.0))) },
                    Co { edge: e, rev: true, pc: Some(line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0))) },
                ]
            }
            Surface::Torus { .. } => {
                let sc = iso(surf, 0, 0.0)?.curve;
                let rc = iso(surf, 1, 0.0)?.curve;
                let v = self.vertex_at(sc.eval(0.0));
                let es = self.b.add_edge(sc, 0.0, TAU, v, v);
                let er = self.b.add_edge(rc, 0.0, TAU, v, v);
                vec![
                    Co { edge: er, rev: false, pc: Some(line(Vec2::new(0.0, 0.0), Vec2::new(1.0, 0.0))) },
                    Co { edge: es, rev: false, pc: Some(line(Vec2::new(TAU, 0.0), Vec2::new(0.0, 1.0))) },
                    Co { edge: er, rev: true, pc: Some(line(Vec2::new(0.0, TAU), Vec2::new(1.0, 0.0))) },
                    Co { edge: es, rev: true, pc: Some(line(Vec2::new(0.0, 0.0), Vec2::new(0.0, 1.0))) },
                ]
            }
            _ => return None,
        };
        if !sense {
            l.reverse();
            for c in &mut l {
                c.rev = !c.rev;
            }
        }
        Some(l)
    }

    fn co_ts(&self, c: &Co) -> (f64, f64) {
        let e = self.b.edge(c.edge);
        if c.rev {
            (e.t1, e.t0)
        } else {
            (e.t0, e.t1)
        }
    }

    fn co_uv(&self, c: &Co, start: bool) -> Option<Vec2> {
        let (a, z) = self.co_ts(c);
        c.pc.as_ref().map(|p| p.eval(if start { a } else { z }))
    }

    /// Signed uv area of a loop (jumps included as straight segments).
    fn loop_area(&self, l: &[Co]) -> f64 {
        let mut pts: Vec<Vec2> = vec![];
        for c in l {
            let Some(pc) = &c.pc else { return 0.0 };
            let (a, z) = self.co_ts(c);
            for k in 0..=12 {
                pts.push(pc.eval(a + (z - a) * k as f64 / 12.0));
            }
        }
        let mut s = 0.0;
        for i in 0..pts.len() {
            s += pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        0.5 * s
    }

    fn finalize_face(&mut self, shell: ShellId, spec: FaceSpec, name: FaceName) -> R<()> {
        let FaceSpec { id, surface, sense, mut loops } = spec;
        let (pu, pv) = surface.periods();
        let periodic = pu.is_some() || pv.is_some();
        // pcurves
        for l in &mut loops {
            for c in l.iter_mut() {
                if c.pc.is_none() {
                    c.pc = edge_pcurve(&self.b, c.edge, &surface);
                }
            }
        }
        if periodic {
            if loops.iter().flatten().any(|c| c.pc.is_none()) {
                return Err(ImportError::new("topology", Some(id), "no pcurve for an edge of a periodic face"));
            }
            for l in &mut loops {
                self.unwrap_loop(l, pu, pv);
                self.fix_seam_pair(l, &surface, sense, pu, pv);
            }
            loops = self.complete_periodic(id, &surface, sense, loops, pu, pv)?;
        }
        let f = self.b.add_face(shell, surface.clone(), sense, name);
        for l in loops {
            let lp = self.b.add_loop(f);
            for c in l {
                self.b.add_coedge(lp, c.edge, c.rev, c.pc);
            }
        }
        if matches!(surface, Surface::Plane { .. }) {
            ck_topo::orient::fix_planar_loops(&mut self.b, f);
        }
        Ok(())
    }

    /// Shift pcurves by whole periods so consecutive coedges join up.
    fn unwrap_loop(&self, l: &mut [Co], pu: Option<f64>, pv: Option<f64>) {
        let mut prev: Option<Vec2> = None;
        for i in 0..l.len() {
            let (a, z) = self.co_ts(&l[i]);
            let pc = l[i].pc.clone().unwrap();
            if let Some(p) = prev {
                let s = pc.eval(a);
                let mut d = Vec2::new(0.0, 0.0);
                if let Some(per) = pu {
                    d.x = ((p.x - s.x) / per).round() * per;
                }
                if let Some(per) = pv {
                    d.y = ((p.y - s.y) / per).round() * per;
                }
                if d.x != 0.0 || d.y != 0.0 {
                    l[i].pc = Some(translate(&pc, d));
                }
            }
            prev = l[i].pc.as_ref().map(|p| p.eval(z));
        }
    }

    /// A seam used twice with identical uv (consecutive legs joined only by
    /// a pole jump): offset the second leg by one period so the loop
    /// encloses area with the sign the face sense asks for.
    fn fix_seam_pair(&self, l: &mut [Co], _surf: &Surface, sense: bool, pu: Option<f64>, pv: Option<f64>) {
        let n = l.len();
        for i in 0..n {
            for j in i + 1..n {
                if l[i].edge != l[j].edge || l[i].rev == l[j].rev {
                    continue;
                }
                let (a, b) = (self.co_uv(&l[i], true), self.co_uv(&l[j], false));
                let (Some(a), Some(b)) = (a, b) else { continue };
                if a.dist(b) > 1e-9 {
                    continue;
                }
                // axis the seam runs across
                let dir = match &l[i].pc {
                    Some(Curve2::Line { d, .. }) if d.x.abs() < 1e-12 => 0,
                    Some(Curve2::Line { d, .. }) if d.y.abs() < 1e-12 => 1,
                    _ => continue,
                };
                let Some(per) = (if dir == 0 { pu } else { pv }) else { continue };
                let want = if sense { 1.0 } else { -1.0 };
                for sh in [per, -per] {
                    let mut t = l.to_vec();
                    for c in t[j..].iter_mut() {
                        c.pc = c.pc.as_ref().map(|p| translate(p, unit2(dir, sh)));
                    }
                    if self.loop_area(&t) * want > 1e-9 {
                        l[j..].clone_from_slice(&t[j..]);
                        return;
                    }
                }
            }
        }
    }

    /// Windings of a loop along one periodic axis: Σ displacement / period.
    fn winding(&self, l: &[Co], d: usize, per: Option<f64>) -> Option<i64> {
        let per = per?;
        let mut s = 0.0;
        for c in l {
            let (a, z) = (self.co_uv(c, true)?, self.co_uv(c, false)?);
            s += comp(z, d) - comp(a, d);
        }
        let k = (s / per).round();
        ((s / per - k).abs() < 1e-6).then_some(k as i64)
    }

    fn complete_periodic(
        &mut self,
        id: u32,
        surf: &Surface,
        sense: bool,
        loops: Vec<Vec<Co>>,
        pu: Option<f64>,
        pv: Option<f64>,
    ) -> R<Vec<Vec<Co>>> {
        let pers = [pu, pv];
        // classify
        let mut wind: Vec<(usize, usize, i64)> = vec![]; // (loop idx, axis, k)
        for (li, l) in loops.iter().enumerate() {
            for d in 0..2 {
                match self.winding(l, d, pers[d]) {
                    Some(0) | None => {}
                    Some(k) => wind.push((li, d, k)),
                }
            }
        }
        if wind.is_empty() {
            return Ok(loops);
        }
        let d = wind[0].1;
        if wind.iter().any(|w| w.1 != d || w.2.abs() != 1) {
            return Err(ImportError::new("unsupported", Some(id), "periodic face whose boundary winds more than once or around both directions"));
        }
        let per = pers[d].unwrap();
        let mut loops = loops;
        match wind.len() {
            2 if wind[0].2 == -wind[1].2 => {
                let (ia, ib) = (wind[0].0, wind[1].0);
                let (la, lb) = (loops[ia].clone(), loops[ib].clone());
                let joined = self.join_two(id, surf, d, per, pers[1 - d], la, lb, wind[0].2)?;
                // replace the first by the joined loop, drop the second
                loops[ia] = joined;
                loops.remove(ib);
                Ok(loops)
            }
            1 => {
                let ia = wind[0].0;
                let la = loops[ia].clone();
                let joined = self.close_at_pole(id, surf, sense, d, per, la, wind[0].2)?;
                loops[ia] = joined;
                Ok(loops)
            }
            _ => Err(ImportError::new("unsupported", Some(id), "periodic face with an unsupported set of winding boundaries")),
        }
    }

    /// Index of the coedge of `l` whose start coordinate along `d` is `s`
    /// (mod period) -> rotated loop starting there.
    fn rotate_to(&self, l: &[Co], d: usize, s: f64, per: f64) -> Option<(Vec<Co>, f64)> {
        let mut best: Option<(usize, f64)> = None;
        for (i, c) in l.iter().enumerate() {
            let u = comp(self.co_uv(c, true)?, d);
            let diff = ((u - s) / per).round() * per - (u - s);
            let e = diff.abs();
            if best.map_or(true, |(_, be)| e < be) {
                best = Some((i, e));
            }
        }
        let (i, e) = best?;
        if e > 1e-6 * (1.0 + s.abs()) {
            return None;
        }
        let mut r = l[i..].to_vec();
        r.extend_from_slice(&l[..i]);
        Some((r, e))
    }

    #[allow(clippy::too_many_arguments)]
    fn join_two(&mut self, id: u32, surf: &Surface, d: usize, per: f64, per_o: Option<f64>, la: Vec<Co>, lb: Vec<Co>, wa: i64) -> R<Vec<Co>> {
        let o = 1 - d;
        let fail = || ImportError::new("topology", Some(id), "periodic face whose two boundaries have no vertices at a common seam position");
        // candidate seam positions: the start coordinate of every coedge of A
        let mut chosen = None;
        for i in 0..la.len() {
            let s = comp(self.co_uv(&la[i], true).ok_or_else(fail)?, d);
            if let Some((rb, _)) = self.rotate_to(&lb, d, s, per) {
                chosen = Some((i, rb));
                break;
            }
        }
        let (ia, mut rb) = chosen.ok_or_else(fail)?;
        let mut ra = la[ia..].to_vec();
        ra.extend_from_slice(&la[..ia]);
        let a0 = self.co_uv(&ra[0], true).ok_or_else(fail)?;
        let s = comp(a0, d);
        // shift B so its start meets the end of the seam leg
        let b0 = self.co_uv(&rb[0], true).ok_or_else(fail)?;
        let target_d = s + wa as f64 * per;
        let mut sh = unit2(d, ((target_d - comp(b0, d)) / per).round() * per);
        if let Some(po) = per_o {
            let k = ((comp(a0, o) - comp(b0, o)) / po).round();
            sh = with(sh, o, k * po);
        }
        for c in rb.iter_mut() {
            c.pc = c.pc.as_ref().map(|p| translate(p, sh));
        }
        let b0 = self.co_uv(&rb[0], true).ok_or_else(fail)?;
        // seam edge from vertex of A's start to vertex of B's start
        let va = self.co_start_vertex(&ra[0]);
        let vb = self.co_start_vertex(&rb[0]);
        let (o_a, o_b) = (comp(a0, o), comp(b0, o));
        let isoc = iso(surf, d, s).ok_or_else(|| ImportError::new("unsupported", Some(id), format!("seam on a {} surface", surf.kind())))?;
        if (o_b - o_a).abs() < 1e-12 {
            return Err(ImportError::new("topology", Some(id), "degenerate seam"));
        }
        // t of the seam curve at coordinate o: the iso curve has o(t) = k·t + c with c fixed by its start point
        let t_of = |ob: f64| -> f64 { self.iso_t(&isoc, surf, d, s, ob) };
        let (ta, tb) = (t_of(o_a), t_of(o_b));
        let (e, fwd_is_rev) = if ta < tb {
            (self.b.add_edge(isoc.curve.clone(), ta, tb, va, vb), false)
        } else {
            (self.b.add_edge(isoc.curve.clone(), tb, ta, vb, va), true)
        };
        let (t_lo, o_lo) = if ta < tb { (ta, o_a) } else { (tb, o_b) };
        let pc_at = |u: f64| iso_pc(d, u, isoc.k, o_lo, t_lo);
        let mut out = ra;
        out.push(Co { edge: e, rev: fwd_is_rev, pc: Some(pc_at(s + wa as f64 * per)) });
        out.extend(rb);
        out.push(Co { edge: e, rev: !fwd_is_rev, pc: Some(pc_at(s)) });
        Ok(out)
    }

    /// Parameter of the iso curve at the free coordinate `o_val`.
    fn iso_t(&self, isoc: &Iso, surf: &Surface, d: usize, s: f64, o_val: f64) -> f64 {
        let p = if d == 0 { surf.eval(s, o_val) } else { surf.eval(o_val, s) };
        let t = Self::param(&isoc.curve, p);
        // periodic carriers (sphere meridian circle, torus circles): choose the
        // representative whose parameter equals the free coordinate / k
        match isoc.curve.period() {
            Some(per) => {
                let want = o_val / isoc.k;
                t + ((want - t) / per).round() * per
            }
            None => t,
        }
    }

    fn co_start_vertex(&self, c: &Co) -> VertexId {
        let e = self.b.edge(c.edge);
        if c.rev {
            e.v1
        } else {
            e.v0
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn close_at_pole(&mut self, id: u32, surf: &Surface, sense: bool, d: usize, per: f64, la: Vec<Co>, wa: i64) -> R<Vec<Co>> {
        let fail = |m: &str| ImportError::new("topology", Some(id), m.to_string());
        if d != 0 {
            return Err(fail("single winding boundary around the v direction"));
        }
        // which way does the face lie from the boundary: left of the traversal
        let c0 = &la[0];
        let a0 = self.co_uv(c0, true).ok_or_else(|| fail("no pcurve"))?;
        let ed = self.b.edge(c0.edge);
        let tm = 0.5 * (ed.t0 + ed.t1);
        let mut tan = ed.curve.tangent(tm);
        if c0.rev {
            tan = -tan;
        }
        let uvm = c0.pc.as_ref().map(|p| p.eval(tm)).ok_or_else(|| fail("no pcurve"))?;
        let n = surf.normal(uvm.x, uvm.y) * if sense { 1.0 } else { -1.0 };
        let left = n.cross(tan);
        let sv = surf.partials(uvm.x, uvm.y).1;
        let up = left.dot(sv) > 0.0;
        let v_pole = match surf {
            Surface::Sphere { .. } => {
                if up {
                    FRAC_PI_2
                } else {
                    -FRAC_PI_2
                }
            }
            Surface::Cone { r, semi_angle, .. } => {
                let va = -r / semi_angle.tan();
                if (va > a0.y) != up {
                    return Err(fail("cone boundary faces away from the apex"));
                }
                va
            }
            _ => return Err(ImportError::new("unsupported", Some(id), format!("single boundary winding a {} surface", surf.kind()))),
        };
        let s = a0.x;
        let pole_p = surf.eval(s, v_pole);
        let vp = self.vertex_at(pole_p);
        let va = self.co_start_vertex(&la[0]);
        if va == vp {
            return Err(fail("boundary starts at the pole"));
        }
        let isoc = iso(surf, 0, s).ok_or_else(|| fail("no seam curve"))?;
        let (ta, tb) = (self.iso_t(&isoc, surf, 0, s, a0.y), self.iso_t(&isoc, surf, 0, s, v_pole));
        let (e, rev) = if ta < tb {
            (self.b.add_edge(isoc.curve.clone(), ta, tb, va, vp), false)
        } else {
            (self.b.add_edge(isoc.curve.clone(), tb, ta, vp, va), true)
        };
        let (t_lo, o_lo) = if ta < tb { (ta, a0.y) } else { (tb, v_pole) };
        let pc_at = |u: f64| iso_pc(0, u, isoc.k, o_lo, t_lo);
        let mut out = la;
        out.push(Co { edge: e, rev, pc: Some(pc_at(s + wa as f64 * per)) });
        out.push(Co { edge: e, rev: !rev, pc: Some(pc_at(s)) });
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BOX: &str = "ISO-10303-21;
HEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('b','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));ENDSEC;
DATA;
#1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#10=CARTESIAN_POINT('',(0.,0.,0.));
#11=CARTESIAN_POINT('',(10.,0.,0.));
#12=CARTESIAN_POINT('',(10.,10.,0.));
#13=CARTESIAN_POINT('',(0.,10.,0.));
#14=CARTESIAN_POINT('',(0.,0.,5.));
#15=CARTESIAN_POINT('',(10.,0.,5.));
#16=CARTESIAN_POINT('',(10.,10.,5.));
#17=CARTESIAN_POINT('',(0.,10.,5.));
#20=VERTEX_POINT('',#10);#21=VERTEX_POINT('',#11);#22=VERTEX_POINT('',#12);#23=VERTEX_POINT('',#13);
#24=VERTEX_POINT('',#14);#25=VERTEX_POINT('',#15);#26=VERTEX_POINT('',#16);#27=VERTEX_POINT('',#17);
#30=DIRECTION('',(1.,0.,0.));#31=DIRECTION('',(0.,1.,0.));#32=DIRECTION('',(0.,0.,1.));
#33=VECTOR('',#30,1.);#34=VECTOR('',#31,1.);#35=VECTOR('',#32,1.);
#40=LINE('',#10,#33);#41=LINE('',#11,#34);#42=LINE('',#13,#33);#43=LINE('',#10,#34);
#44=LINE('',#14,#33);#45=LINE('',#15,#34);#46=LINE('',#17,#33);#47=LINE('',#14,#34);
#48=LINE('',#10,#35);#49=LINE('',#11,#35);#50=LINE('',#12,#35);#51=LINE('',#13,#35);
#60=EDGE_CURVE('',#20,#21,#40,.T.);#61=EDGE_CURVE('',#21,#22,#41,.T.);#62=EDGE_CURVE('',#23,#22,#42,.T.);#63=EDGE_CURVE('',#20,#23,#43,.T.);
#64=EDGE_CURVE('',#24,#25,#44,.T.);#65=EDGE_CURVE('',#25,#26,#45,.T.);#66=EDGE_CURVE('',#27,#26,#46,.T.);#67=EDGE_CURVE('',#24,#27,#47,.T.);
#68=EDGE_CURVE('',#20,#24,#48,.T.);#69=EDGE_CURVE('',#21,#25,#49,.T.);#70=EDGE_CURVE('',#22,#26,#50,.T.);#71=EDGE_CURVE('',#23,#27,#51,.T.);
#80=AXIS2_PLACEMENT_3D('',#10,#32,#30);
#81=PLANE('',#80);
#82=AXIS2_PLACEMENT_3D('',#14,#32,#30);
#83=PLANE('',#82);
#84=AXIS2_PLACEMENT_3D('',#10,#31,#30);#85=PLANE('',#84);
#86=AXIS2_PLACEMENT_3D('',#13,#31,#30);#87=PLANE('',#86);
#88=AXIS2_PLACEMENT_3D('',#10,#30,#31);#89=PLANE('',#88);
#90=AXIS2_PLACEMENT_3D('',#11,#30,#31);#91=PLANE('',#90);
#100=ORIENTED_EDGE('',*,*,#60,.T.);#101=ORIENTED_EDGE('',*,*,#61,.T.);#102=ORIENTED_EDGE('',*,*,#62,.F.);#103=ORIENTED_EDGE('',*,*,#63,.F.);
#104=EDGE_LOOP('',(#100,#101,#102,#103));#105=FACE_OUTER_BOUND('',#104,.T.);
#106=ADVANCED_FACE('',(#105),#81,.F.);
#110=ORIENTED_EDGE('',*,*,#64,.T.);#111=ORIENTED_EDGE('',*,*,#65,.T.);#112=ORIENTED_EDGE('',*,*,#66,.F.);#113=ORIENTED_EDGE('',*,*,#67,.F.);
#114=EDGE_LOOP('',(#110,#111,#112,#113));#115=FACE_OUTER_BOUND('',#114,.T.);
#116=ADVANCED_FACE('',(#115),#83,.T.);
#120=ORIENTED_EDGE('',*,*,#60,.T.);#121=ORIENTED_EDGE('',*,*,#69,.T.);#122=ORIENTED_EDGE('',*,*,#64,.F.);#123=ORIENTED_EDGE('',*,*,#68,.F.);
#124=EDGE_LOOP('',(#120,#121,#122,#123));#125=FACE_OUTER_BOUND('',#124,.T.);
#126=ADVANCED_FACE('',(#125),#85,.F.);
#130=ORIENTED_EDGE('',*,*,#62,.T.);#131=ORIENTED_EDGE('',*,*,#70,.T.);#132=ORIENTED_EDGE('',*,*,#66,.F.);#133=ORIENTED_EDGE('',*,*,#71,.F.);
#134=EDGE_LOOP('',(#130,#131,#132,#133));#135=FACE_OUTER_BOUND('',#134,.T.);
#136=ADVANCED_FACE('',(#135),#87,.T.);
#140=ORIENTED_EDGE('',*,*,#63,.T.);#141=ORIENTED_EDGE('',*,*,#71,.T.);#142=ORIENTED_EDGE('',*,*,#67,.F.);#143=ORIENTED_EDGE('',*,*,#68,.F.);
#144=EDGE_LOOP('',(#140,#141,#142,#143));#145=FACE_OUTER_BOUND('',#144,.T.);
#146=ADVANCED_FACE('',(#145),#89,.F.);
#150=ORIENTED_EDGE('',*,*,#61,.T.);#151=ORIENTED_EDGE('',*,*,#70,.T.);#152=ORIENTED_EDGE('',*,*,#65,.F.);#153=ORIENTED_EDGE('',*,*,#69,.F.);
#154=EDGE_LOOP('',(#150,#151,#152,#153));#155=FACE_OUTER_BOUND('',#154,.T.);
#156=ADVANCED_FACE('',(#155),#91,.T.);
#200=CLOSED_SHELL('',(#106,#116,#126,#136,#146,#156));
#201=MANIFOLD_SOLID_BREP('box',#200);
ENDSEC;
END-ISO-10303-21;";

    #[test]
    fn imports_box() {
        let r = read_step(BOX).unwrap();
        assert_eq!(r.solids, 1);
        assert_eq!(r.body.num_faces(), 6);
        let m = ck_topo::mass::mass_props(&r.body);
        assert!((m.volume - 500.0).abs() < 1e-9, "{}", m.volume);
        assert!(ck_topo::check::check(&r.body, 1e-6).is_empty());
    }

    #[test]
    fn unsupported_is_structured() {
        let t = BOX.replace("#81=PLANE('',#80);", "#81=DEGENERATE_TOROIDAL_SURFACE('',#80,1.,1.);");
        let e = read_step(&t).unwrap_err();
        assert_eq!((e.code, e.entity), ("unsupported", Some(81)));
    }
}
