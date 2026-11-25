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

use super::recog;
use super::parse::{default_knots, expand_knots_max, parse, Ent, ImportError, StepFile, Val};
use ck_core::xform::Xform;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::pcurve::{exact_pcurve, pcurve_of, pcurve_of_dense};
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
    /// MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS count (`parts` also holds the shells of surface models).
    pub solids: usize,
    /// Length scale applied (file unit -> mm).
    pub length_scale: f64,
    /// Non-fatal problems (skipped degenerate loops, ...).
    pub warnings: Vec<ImportError>,
    /// What became of each solid of the file, in file order (one per MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS).
    pub parts: Vec<SolidRead>,
}

/// One solid of the file as read. A face that cannot be read is left out (and named in `refused`) instead of failing the
/// whole file; a solid whose shells cannot be walked at all carries `error` and no faces. Whether the faces that came in
/// make a valid solid is judged afterwards (`ck_heal::honest`).
#[derive(Debug, Clone)]
pub struct SolidRead {
    /// Entity id of the MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS.
    pub entity: u32,
    /// Its label in the file (occurrence name, MANIFOLD_SOLID_BREP name, or PRODUCT name; '' when unnamed).
    pub name: String,
    /// Optional RGB color `[r, g, b]` in `0.0..=1.0` from AP214/AP242 `STYLED_ITEM`.
    pub color: Option<[f64; 3]>,
    /// Optional per-face RGB colors `[r, g, b]` in `0.0..=1.0` from `STYLED_ITEM` on `ADVANCED_FACE`.
    pub face_colors: HashMap<FaceId, [f64; 3]>,
    /// Faces the file gives the solid.
    pub faces_total: usize,
    /// Faces built (ids in `Imported::body`).
    pub faces: Vec<FaceId>,
    /// Faces left out, one error each, `entity` = the face's id.
    pub refused: Vec<ImportError>,
    /// The surface of each refused face whose surface could be read (face id, surface, sense): what a repair rebuilds the
    /// face on.
    pub refused_surfaces: Vec<(u32, Surface, bool)>,
    /// The solid could not be read at all (its shells could not be walked).
    pub error: Option<ImportError>,
    /// A shell of a SHELL_BASED_SURFACE_MODEL: the file gives this part as surfaces, not as a solid.
    pub surface_model: bool,
}

/// Parse `text` and build the solids it contains.
///
/// B-spline carriers that are really a line, a circle or a plane are read as that (see `recog.rs`). The recognised read is
/// compared with the plain one and thrown away when it is worse (a face fewer, a refusal more, more validation issues, a
/// different volume), so recognition never makes an import worse than the splines would.
pub fn read_step(text: &str) -> R<Imported> {
    let file = parse(text)?;
    let mut rd = Rd::new(&file);
    rd.recog = true;
    let (rec, converted) = rd.run_counted();
    if converted == 0 {
        return rec;
    }
    let plain = Rd::new(&file).run();
    match (rec, plain) {
        (Ok(r), Ok(p)) => Ok(if recognised_is_no_worse(&r, &p) { r } else { p }),
        (Ok(r), Err(_)) => Ok(r),
        (Err(_), p) => p,
    }
}

/// [`read_step`] without analytic recognition: B-spline carriers stay B-splines (for comparison and for tests).
pub fn read_step_plain(text: &str) -> R<Imported> {
    let file = parse(text)?;
    Rd::new(&file).run()
}

/// Is the import with analytic recognition at least as good as the plain one? Faces, refusals, warnings, validation
/// issues and (when the same faces came in) the volume are compared.
fn recognised_is_no_worse(r: &Imported, p: &Imported) -> bool {
    if r.parts.len() != p.parts.len() {
        return false;
    }
    let mut same_faces = true;
    for (a, b) in r.parts.iter().zip(&p.parts) {
        if a.faces.len() < b.faces.len() || a.refused.len() > b.refused.len() || (a.error.is_some() && b.error.is_none()) {
            return false;
        }
        same_faces &= a.faces.len() == b.faces.len() && a.refused.len() == b.refused.len();
    }
    if r.warnings.len() > p.warnings.len() {
        return false;
    }
    if r.parts.iter().any(|x| x.surface_model || x.faces.is_empty()) {
        return true;
    }
    let (ir, ip) = (ck_check::validate(&r.body, 1e-6).len(), ck_check::validate(&p.body, 1e-6).len());
    if ir > ip {
        return false;
    }
    if same_faces {
        // faces both reads share (same surface and trimming) have the same share of the volume: when the faces that differ
        // change it by no more than rounding of the part's size, the volumes agree without measuring the whole body
        let (lo, hi) = r.body.bbox();
        let diag = (hi - lo).len();
        if ck_topo::mass::volume_difference_unshared(&r.body, &p.body).abs() <= 1e-10 * diag * diag * diag {
            return true;
        }
        let (vr, vp) = (ck_topo::mass::mass_props(&r.body).volume, ck_topo::mass::mass_props(&p.body).volume);
        // (not tighter: the spline carriers' own volume is only quadrature-accurate, ~5e-7 for a rational circle; a wrong
        // arc or sense changes the volume by far more)
        if (vr - vp).abs() > 1e-5 * vp.abs().max(vr.abs()).max(1e-12) {
            return false;
        }
    }
    true
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
    /// loop i came from a FACE_BOUND with orientation `.F.` (already applied)
    flagged: Vec<bool>,
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
    depth: std::cell::Cell<u32>,
    /// Read B-spline carriers that are really lines, circles or planes as those (`recog.rs`).
    recog: bool,
    /// How many carriers were so converted.
    converted: usize,
}

/// Highest B-spline degree read. Real files stay below 30; the cap keeps a
/// degree taken from the file from sizing a knot-vector allocation.
const MAX_DEGREE: usize = 1024;

/// Largest coordinate magnitude accepted, in mm (a thousand kilometres).
const MAX_COORD_MM: f64 = 1e12;

const VTOL: f64 = 1e-7;
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
            depth: std::cell::Cell::new(0),
            recog: false,
            converted: 0,
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
            let s = v * self.unit_si(u, depth + 1)?;
            // a zero / negative / infinite factor would collapse or poison the model
            return (s.is_finite() && s > 0.0).then_some(s);
        }
        None
    }

    fn detect_units(&mut self) {
        let mut ids: Vec<&u32> = self.f.ents.keys().collect();
        ids.sort();
        let (mut got_len, mut got_ang) = (false, false);
        // Prefer the units named by the geometric representation context
        // (GLOBAL_UNIT_ASSIGNED_CONTEXT): files also hold the SI base units
        // (radian) that a CONVERSION_BASED_UNIT (degree) is defined against, and
        // the lowest entity id is not necessarily the unit the geometry uses.
        for id in &ids {
            let e = &self.f.ents[*id];
            if !e.has("GEOMETRIC_REPRESENTATION_CONTEXT") {
                continue;
            }
            let Some(g) = e.part("GLOBAL_UNIT_ASSIGNED_CONTEXT") else { continue };
            let Some(units) = g.args.first().and_then(Val::as_list) else { continue };
            for u in units.iter().filter_map(Val::as_ref) {
                let Some(ue) = self.f.ents.get(&u) else { continue };
                if !got_len && ue.has("LENGTH_UNIT") {
                    if let Some(s) = self.unit_si(u, 0) {
                        let mm = s * 1000.0;
                        if mm.is_finite() && mm > 0.0 {
                            self.len = mm;
                            got_len = true;
                        }
                    }
                } else if !got_ang && ue.has("PLANE_ANGLE_UNIT") {
                    if let Some(s) = self.unit_si(u, 0) {
                        if s.is_finite() && s > 0.0 {
                            self.ang = s;
                            got_ang = true;
                        }
                    }
                }
            }
            if got_len || got_ang {
                break;
            }
        }
        for id in ids {
            let e = &self.f.ents[id];
            if !got_len && e.has("LENGTH_UNIT") {
                if let Some(s) = self.unit_si(*id, 0) {
                    let mm = s * 1000.0;
                    if mm.is_finite() && mm > 0.0 {
                        self.len = mm;
                        got_len = true;
                    }
                }
            } else if !got_ang && e.has("PLANE_ANGLE_UNIT") {
                if let Some(s) = self.unit_si(*id, 0) {
                    if s.is_finite() && s > 0.0 {
                        self.ang = s;
                        got_ang = true;
                    }
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
            (Some(x), Some(y)) => {
                let p = Vec3::new(x, y, g(2).unwrap_or(0.0)) * self.len;
                // finite and of a sane size: 1e300 survives the parser but overflows
                // every distance / area computed from it (NaN domains, clamp panics)
                if [p.x, p.y, p.z].iter().all(|c| c.abs() <= MAX_COORD_MM) {
                    Ok(p)
                } else {
                    Err(bad(id, "coordinate out of range"))
                }
            }
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

    /// Nesting guard: malformed files may reference an entity from itself.
    fn nested<T>(&self, id: u32, f: impl FnOnce() -> R<T>) -> R<T> {
        if self.depth.get() > 24 {
            return Err(bad(id, "entity references nest too deeply (cycle?)"));
        }
        self.depth.set(self.depth.get() + 1);
        let r = f();
        self.depth.set(self.depth.get() - 1);
        r
    }

    fn curve(&self, id: u32) -> R<Curve> {
        self.nested(id, || self.curve_inner(id))
    }

    fn surface(&self, id: u32) -> R<Surface> {
        self.nested(id, || self.surface_inner(id))
    }

    /// True when the surface entity's own normal is the reverse of the `Surface`
    /// built from it: a TOROIDAL_SURFACE with exactly one negative radius
    /// (P = (R + r cos v) radial + r sin v axis, so the sign of R*r flips the
    /// cross product dP/du x dP/dv relative to the outward normal).
    fn torus_flips(&self, id: u32) -> bool {
        let Some(e) = self.f.ents.get(&id) else { return false };
        if e.parts.len() != 1 || e.parts[0].name != "TOROIDAL_SURFACE" {
            return false;
        }
        let a = &e.parts[0].args;
        let g = |i: usize| a.get(i).and_then(Val::as_f64).unwrap_or(1.0);
        (g(2) < 0.0) != (g(3) < 0.0)
    }

    fn curve_inner(&self, id: u32) -> R<Curve> {
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
                if !(r.is_finite() && r > 0.0) {
                    return Err(bad(id, "non-positive radius"));
                }
                Ok(Curve::Circle { frame: f, r })
            }
            "ELLIPSE" => {
                let f = self.axis2(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "ELLIPSE without placement"))?)?;
                let (sa, sb) = (self.real(id, a.get(2), "semi axis 1")? * self.len, self.real(id, a.get(3), "semi axis 2")? * self.len);
                if !(sa.is_finite() && sb.is_finite() && sa > 0.0 && sb > 0.0) {
                    return Err(bad(id, "non-positive semi axis"));
                }
                Ok(Curve::Ellipse { frame: f, a: sa, b: sb })
            }
            // STEP conics are centred on the vertex / centre; ck-geom's Conic is
            // in focus form with the polar axis towards the vertex (the near
            // branch of a hyperbola, the only one STEP can express)
            "PARABOLA" => {
                let f = self.axis2(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "PARABOLA without placement"))?)?;
                let fd = self.real(id, a.get(2), "focal distance")? * self.len;
                if !(fd.is_finite() && fd > 0.0 && (2.0 * fd).is_finite()) {
                    return Err(bad(id, "non-positive focal distance"));
                }
                let o = f.o + f.x * fd;
                if !o.is_finite() {
                    return Err(bad(id, "non-finite parabola origin"));
                }
                Ok(Curve::Conic { frame: Frame { o, x: -f.x, y: f.z.cross(-f.x), z: f.z }, p: 2.0 * fd, e: 1.0 })
            }
            "HYPERBOLA" => {
                let f = self.axis2(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "HYPERBOLA without placement"))?)?;
                let (sa, sb) = (self.real(id, a.get(2), "semi axis")? * self.len, self.real(id, a.get(3), "semi imaginary axis")? * self.len);
                if !(sa.is_finite() && sb.is_finite() && sa > 0.0 && sb > 0.0) {
                    return Err(bad(id, "non-positive semi axis"));
                }
                let c = (sa * sa + sb * sb).sqrt();
                let p = sb * sb / sa;
                let e = c / sa;
                let o = f.o + f.x * c;
                if !(c.is_finite() && p.is_finite() && e.is_finite() && o.is_finite()) {
                    return Err(bad(id, "hyperbola parameter out of range"));
                }
                Ok(Curve::Conic { frame: Frame { o, x: -f.x, y: f.z.cross(-f.x), z: f.z }, p, e })
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
        let deg = base.get(off).and_then(Val::as_usize).filter(|&d| d <= MAX_DEGREE).ok_or_else(|| bad(id, "bad degree"))?;
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
            expand_knots_max(m, v, n + deg + 1).ok_or_else(|| bad(id, "bad knot vector"))?
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

    fn surface_inner(&self, id: u32) -> R<Surface> {
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
                if !(r.is_finite() && r > 0.0) {
                    return Err(bad(id, "non-positive radius"));
                }
                Ok(Surface::Cylinder { frame: ax(1)?, r })
            }
            "CONICAL_SURFACE" => {
                let r = self.real(id, a.get(2), "radius")? * self.len;
                let sa = self.real(id, a.get(3), "semi angle")? * self.ang;
                if !(r.is_finite() && r >= 0.0 && sa.is_finite() && sa.abs() > 1e-12 && sa.abs() < FRAC_PI_2) {
                    return Err(bad(id, "invalid cone radius / semi angle"));
                }
                Ok(Surface::Cone { frame: ax(1)?, r, semi_angle: sa })
            }
            "SPHERICAL_SURFACE" => {
                let r = self.real(id, a.get(2), "radius")? * self.len;
                if !(r.is_finite() && r > 0.0) {
                    return Err(bad(id, "non-positive radius"));
                }
                Ok(Surface::Sphere { frame: ax(1)?, r })
            }
            "TOROIDAL_SURFACE" => {
                let (m, n) = (self.real(id, a.get(2), "major radius")? * self.len, self.real(id, a.get(3), "minor radius")? * self.len);
                // A negative radius describes the same torus with the opposite
                // normal (see `torus_flips`); the face sense is flipped by the caller.
                let (m, n) = (m.abs(), n.abs());
                if !(m.is_finite() && n.is_finite() && m > 1e-12 && n > 1e-12) {
                    return Err(bad(id, "zero torus radius"));
                }
                Ok(Surface::Torus { frame: ax(1)?, major: m, minor: n })
            }
            "SURFACE_OF_REVOLUTION" => {
                let c = self.curve(a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "revolution without curve"))?)?;
                let (o, z) = self.axis1(a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "revolution without axis"))?)?;
                if let Some(sph) = revolved_circle_as_sphere(&c, o, z) {
                    return Ok(sph);
                }
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
                let dist = self.real(id, a.get(2), "distance")? * self.len;
                if !dist.is_finite() {
                    return Err(bad(id, "non-finite offset distance"));
                }
                Ok(Surface::Offset { base: Box::new(base), dist })
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
        let du = base.get(off).and_then(Val::as_usize).filter(|&d| d <= MAX_DEGREE).ok_or_else(|| bad(id, "bad u degree"))?;
        let dv = base.get(off + 1).and_then(Val::as_usize).filter(|&d| d <= MAX_DEGREE).ok_or_else(|| bad(id, "bad v degree"))?;
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
                expand_knots_max(l(0)?, l(2)?, nu + du + 1).ok_or_else(|| bad(id, "bad u knot vector"))?,
                expand_knots_max(l(1)?, l(3)?, nv + dv + 1).ok_or_else(|| bad(id, "bad v knot vector"))?,
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
                    if let Some(l) = self.vgrid.get(&(kx.saturating_add(dx), ky.saturating_add(dy), kz.saturating_add(dz))) {
                        for &v in l {
                            if self.b.vertex(v).p.dist(p) <= VTOL {
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

    /// Curve parameter of a point known to lie on the curve. A B-spline may
    /// pass through the point several times (closed curves, curves unrolled
    /// over more than one period): the first pass at or after `after` wins.
    fn param(c: &Curve, p: Vec3, after: Option<f64>) -> f64 {
        if let Curve::BSpline(bs) = c {
            let (d0, d1) = bs.domain();
            let tol = 1e-6 * (1.0 + p.len());
            let closed = c.eval(d0).dist(c.eval(d1)) < tol;
            if closed {
                if let Some(t) = first_pass(c, bs.domain(), bs.ctrl.len(), p, after) {
                    return t;
                }
            } else if c.eval(d0).dist(p) < tol {
                return d0;
            } else if c.eval(d1).dist(p) < tol {
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
        curve = self.recognise_curve(curve, plo, phi);
        let mut lo_is_start = same;
        // degenerate edge (pole / apex): same vertex, no extent
        if lo == hi {
            let closed_curve = matches!(curve, Curve::Circle { .. } | Curve::Ellipse { .. })
                || matches!(&curve, Curve::BSpline(bs) if curve.eval(bs.domain().0).dist(curve.eval(bs.domain().1)) < 1e-6 * (1.0 + plo.len()) && curve.length_between(bs.domain().0, bs.domain().1) > 1e-6);
            if !closed_curve {
                self.edges.insert(id, None);
                return Ok(None);
            }
        }
        let mut t0 = Self::param(&curve, plo, None);
        let mut t1 = Self::param(&curve, phi, Some(t0));
        if !(t0.is_finite() && t1.is_finite()) {
            return Err(bad(id, "non-finite edge parameter"));
        }
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
            if !(p.is_finite() && p > 0.0) {
                return Err(bad(id, "invalid curve period"));
            }
            if t1 <= t0 + 1e-12 {
                let k = ((t0 + 1e-12 - t1) / p).floor() + 1.0;
                if k.is_finite() && k <= 4.0 {
                    while t1 <= t0 + 1e-12 {
                        t1 += p;
                    }
                } else if k.is_finite() {
                    t1 += k * p;
                } else {
                    return Err(bad(id, "edge parameter overflow"));
                }
            }
            if t1 - t0 > p + 1e-9 {
                let k = ((t1 - t0 - (p + 1e-9)) / p).floor() + 1.0;
                if k.is_finite() && k <= 4.0 {
                    while t1 - t0 > p + 1e-9 {
                        t1 -= p;
                    }
                } else if k.is_finite() {
                    t1 -= k * p;
                } else {
                    return Err(bad(id, "edge parameter overflow"));
                }
            }
        } else if t1 <= t0 && lo != hi && matches!(&curve, Curve::BSpline(bs) if curve.eval(bs.domain().0).dist(curve.eval(bs.domain().1)) < 1e-6 * (1.0 + plo.len())) {
            // a closed spline carrier whose arc crosses the seam (first vertex later on the curve than the second):
            // move the seam to the first vertex so the arc is one parameter range
            if let Curve::BSpline(bs) = &curve {
                let (d0, d1) = bs.domain();
                if bs.degree > 0 && (d1 - d0).is_finite() && t0 > d0 && t0 < d1 {
                    curve = Curve::BSpline(bs.with_seam_at(t0));
                    t1 += d1 - d0;
                } else {
                    return Err(bad(id, "edge has zero extent on its curve"));
                }
            }
        } else if t1 <= t0 {
            // the carrier runs from the second vertex to the first (a wrong
            // same_sense flag): the edge starts at the other vertex
            if matches!(curve, Curve::BSpline(_)) {
                // (splines: flip the carrier as well, the closed / multi-pass
                // parameter search below depends on it)
                curve = curve.reversed();
            }
            std::mem::swap(&mut lo, &mut hi);
            std::mem::swap(&mut plo, &mut phi);
            lo_is_start = !lo_is_start;
            t0 = Self::param(&curve, plo, None);
            t1 = Self::param(&curve, phi, Some(t0));
            if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
                return Err(bad(id, "edge has zero extent on its curve"));
            }
        }
        if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
            return Err(bad(id, "edge has invalid parameter range"));
        }
        let e = self.b.add_edge(curve, t0, t1, lo, hi);
        self.edges.insert(id, Some((e, lo_is_start)));
        Ok(Some((e, lo_is_start)))
    }

    fn edge_loop(&mut self, id: u32) -> R<Option<Vec<Co>>> {
        let e = self.ent(id)?;
        let kind = e.kind();
        if kind == "VERTEX_LOOP" {
            // the loop carries nothing the body needs (a cone apex, a pole), but
            // the vertex it names must exist: a dangling reference is damage
            let a = self.simple(id, &["VERTEX_LOOP"])?;
            let v = a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(id, "VERTEX_LOOP without a vertex"))?;
            let va = self.simple(v, &["VERTEX_POINT"])?;
            self.point(va.get(1).and_then(Val::as_ref).ok_or_else(|| bad(v, "VERTEX_POINT without point"))?)?;
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
        // ORIENTED_FACE('', *, face_element, orientation): the wrapped face, reversed
        // when orientation is .F. The wrapper maps in place (one face, no copy of
        // the wrapped face left behind).
        if self.ent(id)?.kind() == "ORIENTED_FACE" {
            let a = self.simple(id, &["ORIENTED_FACE"])?;
            let inner = a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "ORIENTED_FACE without face"))?;
            let orient = a.get(3).and_then(Val::as_bool).unwrap_or(true);
            if self.depth.get() > 24 {
                return Err(bad(id, "entity references nest too deeply (cycle?)"));
            }
            self.depth.set(self.depth.get() + 1);
            let r = self.face_spec(inner);
            self.depth.set(self.depth.get() - 1);
            let mut spec = r?;
            if !orient {
                spec.sense = !spec.sense;
                for l in &mut spec.loops {
                    l.reverse();
                    for c in l.iter_mut() {
                        c.rev = !c.rev;
                    }
                }
            }
            return Ok(spec);
        }
        let a = self.simple(id, &["ADVANCED_FACE", "FACE_SURFACE"])?;
        let bounds = self.refs_list(id, a.get(1), "bounds")?;
        let surface = self.surface(a.get(2).and_then(Val::as_ref).ok_or_else(|| bad(id, "face without surface"))?)?;
        let (surface, flip) = self.recognise_surface(surface);
        let sid = a.get(2).and_then(Val::as_ref).unwrap_or(0);
        let sense = a.get(3).and_then(Val::as_bool).unwrap_or(true) != (self.torus_flips(sid) != flip);
        let mut loops = Vec::new();
        let mut flagged: Vec<bool> = Vec::new();
        let mut vertex_loop = false;
        let mut outer = None;
        for (bi, bid) in bounds.iter().enumerate() {
            let ba = self.simple(*bid, &["FACE_BOUND", "FACE_OUTER_BOUND"])?;
            let lid = ba.get(1).and_then(Val::as_ref).ok_or_else(|| bad(*bid, "bound without loop"))?;
            let orient = ba.get(2).and_then(Val::as_bool).unwrap_or(true);
            let Some(mut l) = self.edge_loop(lid)? else {
                self.warnings.push(ImportError::new("skipped", Some(lid), "vertex loop ignored"));
                vertex_loop = true;
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
            flagged.push(!orient);
        }
        if let Some(o) = outer {
            if o < loops.len() && o != 0 {
                loops.swap(0, o);
                flagged.swap(0, o);
            }
        }
        // a whole sphere / torus bounded only by a vertex (OCCT primitive
        // spheres): the seam structure ck-topo uses for a full surface
        if loops.is_empty() && vertex_loop && matches!(surface, Surface::Sphere { .. } | Surface::Torus { .. }) {
            if let Some(l) = self.full_periodic_loop(&surface, sense) {
                loops.push(l);
                flagged.push(false);
            }
        }
        Ok(FaceSpec { id, surface, sense, loops, flagged })
    }

    // ------------------------------------------------------------ driver

    fn run(self) -> R<Imported> {
        self.run_counted().0
    }

    /// Replace a B-spline carrier by the line or circle it is. A circle is only taken when the edge runs the way the
    /// spline does (an open spline whose second vertex comes first would be read as the complementary arc).
    fn recognise_curve(&mut self, curve: Curve, plo: Vec3, phi: Vec3) -> Curve {
        let Curve::BSpline(bs) = &curve else { return curve };
        if !self.recog {
            return curve;
        }
        let found = match recog::spline_as_line(bs) {
            Some(l) => Some(l),
            None => recog::spline_as_circle(bs).filter(|_| {
                let (d0, d1) = bs.domain();
                let closed = curve.eval(d0).dist(curve.eval(d1)) < 1e-6 * (1.0 + plo.len());
                closed || (plo.dist(phi) > 1e-9 && bs.closest_param(plo) < bs.closest_param(phi))
            }),
        };
        match found {
            Some(c) => {
                self.converted += 1;
                c
            }
            None => curve,
        }
    }

    /// Replace a B-spline surface by the plane, circular cylinder or torus it is; the flag says the face's sense flips (the
    /// spline's normal is the reverse of the cylinder's outward one).
    fn recognise_surface(&mut self, surface: Surface) -> (Surface, bool) {
        if self.recog {
            if let Surface::BSpline(bs) = &surface {
                if let Some(p) = recog::surface_as_plane(bs) {
                    self.converted += 1;
                    return (p, false);
                }
                if let Some((c, flip)) = recog::surface_as_cylinder(bs).or_else(|| recog::surface_as_torus(bs)) {
                    self.converted += 1;
                    return (c, flip);
                }
            }
        }
        (surface, false)
    }

    /// The refusal text of a file with nothing to build: what the file does contain (counts by entity type), so the user
    /// can tell an empty file from one whose solids are in a form this reader does not take.
    fn no_solid_message(&self) -> String {
        const KINDS: [&str; 13] = [
            "MANIFOLD_SOLID_BREP",
            "BREP_WITH_VOIDS",
            "SHELL_BASED_SURFACE_MODEL",
            "CLOSED_SHELL",
            "OPEN_SHELL",
            "ADVANCED_FACE",
            "FACETED_BREP",
            "MANIFOLD_SURFACE_SHAPE_REPRESENTATION",
            "TESSELLATED_SOLID",
            "TRIANGULATED_FACE",
            "GEOMETRIC_CURVE_SET",
            "POLYLINE",
            "CARTESIAN_POINT",
        ];
        let mut found = vec![];
        for k in KINDS {
            let n = self.f.ents.values().filter(|e| e.parts.iter().any(|p| p.name == k)).count();
            if n > 0 {
                found.push(format!("{n} {k}"));
            }
        }
        let has = if found.is_empty() { "no solid, shell, face, curve or point entities".to_string() } else { found.join(", ") };
        let faces = self.f.ents.values().any(|e| e.kind() == "ADVANCED_FACE");
        let hint = if faces {
            "faces are present but no MANIFOLD_SOLID_BREP, BREP_WITH_VOIDS or SHELL_BASED_SURFACE_MODEL refers to them, so nothing can be built from them"
        } else {
            "the file holds no B-rep solid or surface model (curves, points or tessellated shapes only, or an empty file); export the part as a B-rep (AP203/AP214/AP242 with MANIFOLD_SOLID_BREP)"
        };
        format!("no MANIFOLD_SOLID_BREP / BREP_WITH_VOIDS / SHELL_BASED_SURFACE_MODEL in the file (the file has: {has}). Hint: {hint}")
    }

    // ------------------------------------------------- AP214/AP242 colors & assembly

    fn parse_predefined_color(name: &str) -> Option<[f64; 3]> {
        match name.trim().to_ascii_lowercase().as_str() {
            "red" => Some([1.0, 0.0, 0.0]),
            "green" => Some([0.0, 1.0, 0.0]),
            "blue" => Some([0.0, 0.0, 1.0]),
            "yellow" => Some([1.0, 1.0, 0.0]),
            "magenta" => Some([1.0, 0.0, 1.0]),
            "cyan" => Some([0.0, 1.0, 1.0]),
            "white" => Some([1.0, 1.0, 1.0]),
            "black" => Some([0.0, 0.0, 0.0]),
            _ => None,
        }
    }

    fn resolve_color(&self, id: u32, depth: u32) -> Option<[f64; 3]> {
        if depth > 12 {
            return None;
        }
        let e = self.f.ents.get(&id)?;
        for p in &e.parts {
            match p.name.as_str() {
                "COLOUR_RGB" => {
                    let r = p.args.get(1).and_then(Val::as_f64)?;
                    let g = p.args.get(2).and_then(Val::as_f64)?;
                    let b = p.args.get(3).and_then(Val::as_f64)?;
                    if r.is_finite() && g.is_finite() && b.is_finite() {
                        return Some([r.clamp(0.0, 1.0), g.clamp(0.0, 1.0), b.clamp(0.0, 1.0)]);
                    }
                }
                "DRAUGHTING_PRE_DEFINED_COLOUR" => {
                    if let Some(s) = p.args.first().and_then(Val::as_str) {
                        if let Some(c) = Self::parse_predefined_color(s) {
                            return Some(c);
                        }
                    }
                }
                "FILL_AREA_STYLE_COLOUR" | "SURFACE_STYLE_RENDERING" | "SURFACE_STYLE_RENDERING_WITH_PROPERTIES" => {
                    if let Some(r) = p.args.get(1).and_then(Val::as_ref) {
                        if let Some(c) = self.resolve_color(r, depth + 1) {
                            return Some(c);
                        }
                    }
                }
                "FILL_AREA_STYLE" | "SURFACE_SIDE_STYLE" => {
                    if let Some(list) = p.args.get(1).and_then(Val::as_list) {
                        for v in list {
                            if let Some(r) = v.as_ref() {
                                if let Some(c) = self.resolve_color(r, depth + 1) {
                                    return Some(c);
                                }
                            }
                        }
                    }
                }
                "SURFACE_STYLE_FILL_AREA" => {
                    if let Some(r) = p.args.first().and_then(Val::as_ref) {
                        if let Some(c) = self.resolve_color(r, depth + 1) {
                            return Some(c);
                        }
                    }
                }
                "SURFACE_STYLE_USAGE" => {
                    if let Some(r) = p.args.get(1).and_then(Val::as_ref) {
                        if let Some(c) = self.resolve_color(r, depth + 1) {
                            return Some(c);
                        }
                    }
                }
                "PRESENTATION_STYLE_ASSIGNMENT" | "PRESENTATION_STYLE_BY_CONTEXT" => {
                    if let Some(list) = p.args.first().and_then(Val::as_list) {
                        for v in list {
                            if let Some(r) = v.as_ref() {
                                if let Some(c) = self.resolve_color(r, depth + 1) {
                                    return Some(c);
                                }
                            }
                        }
                    }
                }
                _ => {}
            }
        }
        None
    }

    fn collect_styled_colors(&self) -> HashMap<u32, [f64; 3]> {
        let mut map = HashMap::new();
        let mut ids: Vec<u32> = self.f.ents.keys().copied().collect();
        ids.sort_unstable();
        for pass in ["STYLED_ITEM", "OVER_RIDING_STYLED_ITEM"] {
            for &id in &ids {
                let Some(e) = self.f.ents.get(&id) else { continue };
                let Some(p) = e.part(pass) else { continue };
                let Some(styles) = p.args.get(1).and_then(Val::as_list) else { continue };
                let Some(item) = p.args.get(2).and_then(Val::as_ref) else { continue };
                for s in styles {
                    if let Some(sr) = s.as_ref() {
                        if let Some(c) = self.resolve_color(sr, 0) {
                            map.insert(item, c);
                            break;
                        }
                    }
                }
            }
        }
        map
    }

    fn extract_rr_reps_and_xform(&self, rr_id: u32) -> Option<(u32, u32, Option<(Frame, Frame)>)> {
        let e = self.f.ents.get(&rr_id)?;
        let mut rep1 = None;
        let mut rep2 = None;
        let mut idt_ref = None;
        for p in &e.parts {
            match p.name.as_str() {
                "REPRESENTATION_RELATIONSHIP" | "SHAPE_REPRESENTATION_RELATIONSHIP" => {
                    if p.args.len() >= 4 {
                        rep1 = p.args.get(2).and_then(Val::as_ref).or(rep1);
                        rep2 = p.args.get(3).and_then(Val::as_ref).or(rep2);
                    }
                }
                "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION" => {
                    if p.args.len() >= 5 {
                        rep1 = p.args.get(2).and_then(Val::as_ref).or(rep1);
                        rep2 = p.args.get(3).and_then(Val::as_ref).or(rep2);
                        idt_ref = p.args.get(4).and_then(Val::as_ref).or(idt_ref);
                    } else if let Some(r) = p.args.first().and_then(Val::as_ref) {
                        idt_ref = Some(r);
                    }
                }
                _ => {}
            }
        }
        let (r1, r2) = (rep1?, rep2?);
        let frames = idt_ref.and_then(|tid| {
            let te = self.f.ents.get(&tid)?;
            let tp = te.part("ITEM_DEFINED_TRANSFORMATION")?;
            let a1 = tp.args.get(2).and_then(Val::as_ref)?;
            let a2 = tp.args.get(3).and_then(Val::as_ref)?;
            Some((self.axis2(a1).ok()?, self.axis2(a2).ok()?))
        });
        Some((r1, r2, frames))
    }

    fn run_counted(mut self) -> (R<Imported>, usize) {
        let r = self.run_inner();
        (r, self.converted)
    }

    fn run_inner(&mut self) -> R<Imported> {
        self.detect_units();
        let item_colors = self.collect_styled_colors();
        let name_of = |id: u32| -> String {
            self.f
                .ents
                .get(&id)
                .and_then(|e| e.parts.first())
                .and_then(|p| p.args.first())
                .and_then(Val::as_str)
                .unwrap_or("")
                .trim()
                .to_string()
        };

        // --- Product & shape representation graph (for PRODUCT names & assembly placements) ---
        let mut all_ids: Vec<u32> = self.f.ents.keys().copied().collect();
        all_ids.sort_unstable();

        let mut prod_name: HashMap<u32, String> = HashMap::new();
        let mut pdf_to_prod: HashMap<u32, u32> = HashMap::new();
        let mut pd_to_pdf: HashMap<u32, u32> = HashMap::new();
        let mut pds_to_def: HashMap<u32, u32> = HashMap::new();
        let mut sdr_list: Vec<(u32, u32)> = Vec::new();
        let mut rep_items: HashMap<u32, Vec<u32>> = HashMap::new();
        let mut cdsr_list: Vec<(u32, u32)> = Vec::new();
        let mut nauo_list: Vec<(u32, String, u32, u32)> = Vec::new();

        for &id in &all_ids {
            let Some(e) = self.f.ents.get(&id) else { continue };
            for p in &e.parts {
                match p.name.as_str() {
                    "PRODUCT" => {
                        let nm = p.args.get(1).and_then(Val::as_str).unwrap_or("").trim();
                        let pid = p.args.first().and_then(Val::as_str).unwrap_or("").trim();
                        let chosen = if !nm.is_empty() { nm } else { pid };
                        if !chosen.is_empty() {
                            prod_name.insert(id, chosen.to_string());
                        }
                    }
                    "PRODUCT_DEFINITION_FORMATION" | "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE" => {
                        if let Some(pr) = p.args.get(2).and_then(Val::as_ref) {
                            pdf_to_prod.insert(id, pr);
                        }
                    }
                    "PRODUCT_DEFINITION" => {
                        if let Some(pdf) = p.args.get(2).and_then(Val::as_ref) {
                            pd_to_pdf.insert(id, pdf);
                        }
                    }
                    "PRODUCT_DEFINITION_SHAPE" => {
                        if let Some(def) = p.args.get(2).and_then(Val::as_ref) {
                            pds_to_def.insert(id, def);
                        }
                    }
                    "SHAPE_DEFINITION_REPRESENTATION" => {
                        if let (Some(pds), Some(rep)) = (p.args.first().and_then(Val::as_ref), p.args.get(1).and_then(Val::as_ref)) {
                            sdr_list.push((pds, rep));
                        }
                    }
                    "SHAPE_REPRESENTATION" | "ADVANCED_BREP_SHAPE_REPRESENTATION" | "MANIFOLD_SURFACE_SHAPE_REPRESENTATION" => {
                        if let Some(list) = p.args.get(1).and_then(Val::as_list) {
                            let items: Vec<u32> = list.iter().filter_map(Val::as_ref).collect();
                            rep_items.insert(id, items);
                        }
                    }
                    "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" => {
                        if let (Some(rr), Some(pds)) = (p.args.first().and_then(Val::as_ref), p.args.get(1).and_then(Val::as_ref)) {
                            cdsr_list.push((rr, pds));
                        }
                    }
                    "NEXT_ASSEMBLY_USAGE_OCCURRENCE" => {
                        if let (Some(parent_pd), Some(child_pd)) = (p.args.get(3).and_then(Val::as_ref), p.args.get(4).and_then(Val::as_ref)) {
                            let nm = p.args.get(1).and_then(Val::as_str).unwrap_or("").trim();
                            let oid = p.args.first().and_then(Val::as_str).unwrap_or("").trim();
                            let occ_name = if !nm.is_empty() { nm } else { oid };
                            nauo_list.push((id, occ_name.to_string(), parent_pd, child_pd));
                        }
                    }
                    _ => {}
                }
            }
        }

        let pd_name = |pd: u32| -> String {
            pd_to_pdf
                .get(&pd)
                .and_then(|pdf| pdf_to_prod.get(pdf))
                .and_then(|pr| prod_name.get(pr))
                .cloned()
                .unwrap_or_default()
        };

        let mut pd_to_reps: HashMap<u32, Vec<u32>> = HashMap::new();
        let mut rep_to_pd: HashMap<u32, u32> = HashMap::new();
        for (pds, rep) in sdr_list {
            if let Some(&pd) = pds_to_def.get(&pds) {
                pd_to_reps.entry(pd).or_default().push(rep);
                rep_to_pd.entry(rep).or_insert(pd);
            }
        }

        // Assembly occurrence transforms from CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
        let mut cdsr_rrs: HashSet<u32> = HashSet::new();
        let mut nauo_cdsr: HashMap<u32, (u32, u32, Option<(Frame, Frame)>)> = HashMap::new();
        for (rr, pds) in cdsr_list {
            cdsr_rrs.insert(rr);
            if let (Some(&nauo), Some(info)) = (pds_to_def.get(&pds), self.extract_rr_reps_and_xform(rr)) {
                nauo_cdsr.insert(nauo, info);
            }
        }

        // Non-assembly SHAPE_REPRESENTATION_RELATIONSHIP links (e.g. SHAPE_REPRESENTATION <-> ADVANCED_BREP_SHAPE_REPRESENTATION)
        let mut rep_links: HashMap<u32, Vec<u32>> = HashMap::new();
        for &id in &all_ids {
            if cdsr_rrs.contains(&id) {
                continue;
            }
            let Some(e) = self.f.ents.get(&id) else { continue };
            if e.has("SHAPE_REPRESENTATION_RELATIONSHIP") {
                if let Some((r1, r2, _)) = self.extract_rr_reps_and_xform(id) {
                    rep_links.entry(r1).or_default().push(r2);
                    rep_links.entry(r2).or_default().push(r1);
                }
            }
        }

        // Helper: collect all reps belonging to a PRODUCT_DEFINITION via BFS over non-assembly links
        let reps_for_pd = |pd: u32| -> Vec<u32> {
            let mut out = Vec::new();
            let mut seen = HashSet::new();
            let mut queue: Vec<u32> = pd_to_reps.get(&pd).cloned().unwrap_or_default();
            let mut qi = 0;
            while qi < queue.len() {
                let r = queue[qi];
                qi += 1;
                if !seen.insert(r) {
                    continue;
                }
                out.push(r);
                if let Some(adj) = rep_links.get(&r) {
                    for &nxt in adj {
                        if !seen.contains(&nxt) {
                            queue.push(nxt);
                        }
                    }
                }
            }
            out
        };

        // Map every solid/surface-model entity id to its owning (rep_id, product_name) for fallback naming & color
        let mut item_owner_rep: HashMap<u32, u32> = HashMap::new();
        let mut item_prod_name: HashMap<u32, String> = HashMap::new();
        for (&pd, _) in &pd_to_reps {
            let pnm = pd_name(pd);
            for r in reps_for_pd(pd) {
                if let Some(items) = rep_items.get(&r) {
                    for &it in items {
                        item_owner_rep.entry(it).or_insert(r);
                        if !pnm.is_empty() {
                            item_prod_name.entry(it).or_insert_with(|| pnm.clone());
                        }
                        if self.f.ents.get(&it).map(|e| e.kind()) == Some("SHELL_BASED_SURFACE_MODEL") {
                            if let Some(a) = self.f.ents.get(&it).and_then(|e| e.parts.first()).map(|p| &p.args) {
                                for sh in a.get(1).and_then(Val::as_list).unwrap_or(&[]).iter().filter_map(Val::as_ref) {
                                    item_owner_rep.entry(sh).or_insert(r);
                                    if !pnm.is_empty() {
                                        item_prod_name.entry(sh).or_insert_with(|| pnm.clone());
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        // Also record rep ownership for any rep even if not linked to a PD
        for (&r, items) in &rep_items {
            for &it in items {
                item_owner_rep.entry(it).or_insert(r);
            }
        }

        // Collect all solids and surface-model shells in a representation (supporting MAPPED_ITEM too)
        fn collect_rep_solids(
            rd: &Rd<'_>,
            rep_id: u32,
            rep_items: &HashMap<u32, Vec<u32>>,
            base_xform: Xform,
            depth: u32,
            out: &mut Vec<(u32, u32, String, bool, Xform)>,
        ) {
            if depth > 12 {
                return;
            }
            let Some(items) = rep_items.get(&rep_id) else { return };
            for &it in items {
                let Some(e) = rd.f.ents.get(&it) else { continue };
                if e.parts.len() == 1 && matches!(e.kind(), "MANIFOLD_SOLID_BREP" | "BREP_WITH_VOIDS") {
                    let nm = e.parts[0].args.first().and_then(Val::as_str).unwrap_or("").trim().to_string();
                    out.push((it, rep_id, nm, false, base_xform));
                } else if e.parts.len() == 1 && e.kind() == "SHELL_BASED_SURFACE_MODEL" {
                    let nm = e.parts[0].args.first().and_then(Val::as_str).unwrap_or("").trim().to_string();
                    for sh in e.parts[0].args.get(1).and_then(Val::as_list).unwrap_or(&[]).iter().filter_map(Val::as_ref) {
                        if matches!(rd.f.ents.get(&sh).map(|se| se.kind()), Some("OPEN_SHELL" | "CLOSED_SHELL")) {
                            out.push((sh, rep_id, nm.clone(), true, base_xform));
                        }
                    }
                } else if e.parts.len() == 1 && e.kind() == "MAPPED_ITEM" {
                    let Some(src_ref) = e.parts[0].args.get(1).and_then(Val::as_ref) else { continue };
                    let Some(tgt_ref) = e.parts[0].args.get(2).and_then(Val::as_ref) else { continue };
                    let Some(src_ent) = rd.f.ents.get(&src_ref).and_then(|se| se.part("REPRESENTATION_MAP")) else { continue };
                    let Some(orig_ax) = src_ent.args.first().and_then(Val::as_ref) else { continue };
                    let Some(mapped_rep) = src_ent.args.get(1).and_then(Val::as_ref) else { continue };
                    let (Ok(fa), Ok(fb)) = (rd.axis2(orig_ax), rd.axis2(tgt_ref)) else { continue };
                    let mx = base_xform.compose(&Xform::frame_to_frame(&fa, &fb));
                    collect_rep_solids(rd, mapped_rep, rep_items, mx, depth + 1, out);
                }
            }
        }

        struct SolidJob {
            sid: u32,
            rep_id: Option<u32>,
            name: String,
            xform: Xform,
            surface_model: bool,
            isolate_cache: bool,
        }

        let mut jobs: Vec<SolidJob> = Vec::new();
        let mut visited_sids: HashSet<u32> = HashSet::new();

        // Build assembly tree if NEXT_ASSEMBLY_USAGE_OCCURRENCE entities exist
        if !nauo_list.is_empty() {
            struct Occ {
                child_pd: u32,
                name: String,
                xform: Xform,
            }
            let mut children: HashMap<u32, Vec<Occ>> = HashMap::new();
            let mut is_child: HashSet<u32> = HashSet::new();
            let mut parent_pds: Vec<u32> = Vec::new();
            for (nauo_id, occ_name, parent_pd, child_pd) in nauo_list {
                let xform = if let Some(&(r1, r2, Some((f1, f2)))) = nauo_cdsr.get(&nauo_id) {
                    let r1_pd = rep_to_pd.get(&r1).copied();
                    let r2_pd = rep_to_pd.get(&r2).copied();
                    if parent_pd != child_pd && r1_pd == Some(parent_pd) && r2_pd == Some(child_pd) {
                        Xform::frame_to_frame(&f2, &f1)
                    } else {
                        Xform::frame_to_frame(&f1, &f2)
                    }
                } else {
                    Xform::IDENTITY
                };
                if !children.contains_key(&parent_pd) {
                    parent_pds.push(parent_pd);
                }
                children.entry(parent_pd).or_default().push(Occ { child_pd, name: occ_name, xform });
                is_child.insert(child_pd);
            }
            let roots: Vec<u32> = parent_pds.into_iter().filter(|pd| !is_child.contains(pd)).collect();

            struct WalkCtx<'a> {
                rd: &'a Rd<'a>,
                children: &'a HashMap<u32, Vec<Occ>>,
                rep_items: &'a HashMap<u32, Vec<u32>>,
                reps_for_pd: &'a dyn Fn(u32) -> Vec<u32>,
                pd_name: &'a dyn Fn(u32) -> String,
                jobs: &'a mut Vec<SolidJob>,
                visited: &'a mut HashSet<u32>,
            }
            fn walk_pd(ctx: &mut WalkCtx<'_>, pd: u32, xform: Xform, occ_name: &str, depth: u32) {
                if depth > 32 {
                    return;
                }
                let pnm = (ctx.pd_name)(pd);
                let mut direct = Vec::new();
                for r in (ctx.reps_for_pd)(pd) {
                    collect_rep_solids(ctx.rd, r, ctx.rep_items, Xform::IDENTITY, 0, &mut direct);
                }
                let multi = direct.len() > 1;
                for (sid, rep_id, solid_nm, surface_model, local_xform) in direct {
                    let name = if !occ_name.is_empty() {
                        if multi && !solid_nm.is_empty() && solid_nm != occ_name && solid_nm != pnm {
                            format!("{occ_name}/{solid_nm}")
                        } else {
                            occ_name.to_string()
                        }
                    } else if !solid_nm.is_empty() {
                        solid_nm
                    } else {
                        pnm.clone()
                    };
                    ctx.visited.insert(sid);
                    ctx.jobs.push(SolidJob {
                        sid,
                        rep_id: Some(rep_id),
                        name,
                        xform: xform.compose(&local_xform),
                        surface_model,
                        isolate_cache: true,
                    });
                }
                if let Some(occs) = ctx.children.get(&pd) {
                    for occ in occs {
                        let next_x = xform.compose(&occ.xform);
                        walk_pd(ctx, occ.child_pd, next_x, &occ.name, depth + 1);
                    }
                }
            }
            let mut wctx = WalkCtx {
                rd: self,
                children: &children,
                rep_items: &rep_items,
                reps_for_pd: &reps_for_pd,
                pd_name: &pd_name,
                jobs: &mut jobs,
                visited: &mut visited_sids,
            };
            for root in roots {
                walk_pd(&mut wctx, root, Xform::IDENTITY, "", 0);
            }
        }

        // Any solids or surface models not already emitted by the assembly tree (including all non-assembly files)
        let mut solids: Vec<u32> = self
            .f
            .ents
            .iter()
            .filter(|(_, e)| e.parts.len() == 1 && matches!(e.kind(), "MANIFOLD_SOLID_BREP" | "BREP_WITH_VOIDS"))
            .map(|(id, _)| *id)
            .collect();
        solids.sort();
        for sid in solids {
            if visited_sids.contains(&sid) {
                continue;
            }
            let mut nm = name_of(sid);
            if nm.is_empty() {
                if let Some(pnm) = item_prod_name.get(&sid) {
                    nm = pnm.clone();
                }
            }
            jobs.push(SolidJob {
                sid,
                rep_id: item_owner_rep.get(&sid).copied(),
                name: nm,
                xform: Xform::IDENTITY,
                surface_model: false,
                isolate_cache: false,
            });
        }
        // surface models (SHELL_BASED_SURFACE_MODEL): each shell is a part of its own, read the same way (a surface body)
        let mut models: Vec<u32> = self.f.ents.iter().filter(|(_, e)| e.parts.len() == 1 && e.kind() == "SHELL_BASED_SURFACE_MODEL").map(|(id, _)| *id).collect();
        models.sort();
        for m in models {
            let Some(a) = self.f.ents.get(&m).and_then(|e| e.parts.first()).map(|p| &p.args) else { continue };
            for sh in a.get(1).and_then(Val::as_list).unwrap_or(&[]).iter().filter_map(Val::as_ref) {
                if visited_sids.contains(&sh) {
                    continue;
                }
                if matches!(self.f.ents.get(&sh).map(|e| e.kind()), Some("OPEN_SHELL" | "CLOSED_SHELL")) {
                    let mut nm = name_of(m);
                    if nm.is_empty() {
                        if let Some(pnm) = item_prod_name.get(&sh).or_else(|| item_prod_name.get(&m)) {
                            nm = pnm.clone();
                        }
                    }
                    jobs.push(SolidJob {
                        sid: sh,
                        rep_id: item_owner_rep.get(&sh).or_else(|| item_owner_rep.get(&m)).copied(),
                        name: nm,
                        xform: Xform::IDENTITY,
                        surface_model: true,
                        isolate_cache: false,
                    });
                }
            }
        }
        if jobs.is_empty() {
            return Err(ImportError::new("noSolid", None, self.no_solid_message()));
        }

        let solid_count = jobs.iter().filter(|j| !j.surface_model).count();
        let mut parts = Vec::with_capacity(jobs.len());
        for (k, job) in jobs.into_iter().enumerate() {
            if job.isolate_cache {
                self.verts.clear();
                self.vgrid.clear();
                self.edges.clear();
            }
            let v0 = self.b.vertices.len();
            let e0 = self.b.edges.len();
            let f0 = self.b.faces.len();
            let mut part = SolidRead {
                entity: job.sid,
                name: job.name,
                color: None,
                face_colors: HashMap::new(),
                faces_total: 0,
                faces: vec![],
                refused: vec![],
                refused_surfaces: vec![],
                error: None,
                surface_model: job.surface_model,
            };
            if let Err(e) = self.solid(job.sid, k, job.rep_id, &item_colors, &mut part) {
                part.error = Some(e);
            } else if job.xform != Xform::IDENTITY {
                let x = &job.xform;
                for v in &mut self.b.vertices[v0..] {
                    v.p = x.apply_point(v.p);
                }
                for ed in &mut self.b.edges[e0..] {
                    ed.curve = ed.curve.transform(|p| x.apply_point(p), |d| x.apply_vec(d));
                }
                for fc in &mut self.b.faces[f0..] {
                    fc.surface = ck_topo::body::transform_surface(&fc.surface, x);
                }
                for (_, surf, _) in &mut part.refused_surfaces {
                    *surf = ck_topo::body::transform_surface(surf, x);
                }
            }
            parts.push(part);
        }
        // nothing at all came in: the file is refused with the first reason
        if parts.iter().all(|p| p.faces.is_empty()) {
            let first = parts.iter().find_map(|p| p.error.clone().or_else(|| p.refused.first().cloned()));
            return Err(first.unwrap_or_else(|| ImportError::new("noSolid", None, "the solids of the file have no faces")));
        }
        Ok(Imported { body: std::mem::replace(&mut self.b, Body::new()), solids: solid_count, length_scale: self.len, warnings: std::mem::take(&mut self.warnings), parts })
    }

    /// The surface and sense of face `id` (for a refused face), when they can be read.
    fn face_surface(&self, id: u32) -> Option<(Surface, bool)> {
        let a = self.simple(id, &["ADVANCED_FACE", "FACE_SURFACE"]).ok()?;
        let sid = a.get(2).and_then(Val::as_ref)?;
        let surface = self.surface(sid).ok()?;
        Some((surface, a.get(3).and_then(Val::as_bool).unwrap_or(true) != self.torus_flips(sid)))
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

    fn solid(&mut self, sid: u32, k: usize, rep_id: Option<u32>, item_colors: &HashMap<u32, [f64; 3]>, part: &mut SolidRead) -> R<()> {
        let e = self.ent(sid)?;
        let Some(p0) = e.parts.first() else {
            return Err(bad(sid, "empty entity"));
        };
        let a = &p0.args;
        // a shell of a surface model is read on its own; only a closed one is turned outward
        let lone = matches!(e.kind(), "OPEN_SHELL" | "CLOSED_SHELL");
        let outer = if lone { sid } else { a.get(1).and_then(Val::as_ref).ok_or_else(|| bad(sid, "solid without shell"))? };
        part.color = item_colors
            .get(&sid)
            .or_else(|| item_colors.get(&outer))
            .or_else(|| rep_id.and_then(|r| item_colors.get(&r)))
            .copied();
        // (shell id, flipped)
        let mut shells = vec![(outer, false)];
        if e.kind() == "BREP_WITH_VOIDS" {
            for v in self.refs_list(sid, a.get(2), "void list")? {
                let ve = self.ent(v)?;
                if ve.kind() == "ORIENTED_CLOSED_SHELL" {
                    let Some(vp0) = ve.parts.first() else {
                        return Err(bad(v, "empty entity"));
                    };
                    let va = &vp0.args;
                    let inner = va.get(2).and_then(Val::as_ref).ok_or_else(|| bad(v, "ORIENTED_CLOSED_SHELL without shell"))?;
                    let o = va.get(3).and_then(Val::as_bool).unwrap_or(true);
                    shells.push((inner, !o));
                } else {
                    shells.push((v, false));
                }
            }
        }
        let open = e.kind() == "OPEN_SHELL";
        let lump = self.b.add_lump();
        if !part.name.is_empty() {
            self.b.lumps[lump.idx()].name = Some(part.name.clone());
        }
        let feature = format!("I{}", k + 1);
        let mut shell_ids: Vec<(ShellId, bool)> = vec![];
        let mut queue: Vec<Option<(ShellId, FaceSpec)>> = vec![];
        for (shid, flipped) in shells {
            let shell = self.b.add_shell(lump);
            let mut specs = Vec::new();
            for fid in self.shell_faces(shid)? {
                part.faces_total += 1;
                match self.face_spec(fid) {
                    Ok(sp) => specs.push(sp),
                    Err(e) => self.refuse_face(part, fid, e),
                }
            }
            self.unflip_inconsistent_bounds(&mut specs);
            self.snap_edges_to_surfaces(&specs);
            self.merge_full_periodic(&mut specs);
            queue.extend(specs.into_iter().map(|s| Some((shell, s))));
            shell_ids.push((shell, flipped));
        }
        for i in 0..queue.len() {
            let Some((shell, s)) = queue[i].take() else { continue };
            self.nface += 1;
            let name = FaceName::other(&feature, &format!("F{}", self.nface));
            let fid = s.id;
            let n0 = self.b.faces.len();
            match self.finalize_face(shell, s, name, &mut queue[i + 1..]) {
                Ok(()) => {
                    let face_col = item_colors.get(&fid).copied();
                    for f in n0..self.b.faces.len() {
                        let face_id = FaceId(f as u32);
                        part.faces.push(face_id);
                        if let Some(c) = face_col {
                            part.face_colors.insert(face_id, c);
                        }
                    }
                }
                Err(e) => self.refuse_face(part, fid, e),
            }
        }
        if part.color.is_none() && !part.faces.is_empty() && part.face_colors.len() == part.faces.len() {
            if let Some(&first) = part.face_colors.get(&part.faces[0]) {
                if part.faces.iter().all(|f| part.face_colors.get(f) == Some(&first)) {
                    part.color = Some(first);
                }
            }
        }
        if !open {
            self.orient_shells(lump, &shell_ids);
        }
        Ok(())
    }

    /// Leave face `fid` out of the solid, with the reason (the error names the face; a deeper entity it came from is kept
    /// in the message).
    fn refuse_face(&mut self, part: &mut SolidRead, fid: u32, e: ImportError) {
        let message = match e.entity {
            Some(x) if x != fid => format!("{} (entity #{x})", e.message),
            _ => e.message,
        };
        if let Some((surf, sense)) = self.face_surface(fid) {
            part.refused_surfaces.push((fid, surf, sense));
        }
        part.refused.push(ImportError::new(e.code, Some(fid), message));
    }

    /// STEP exporters write intersection edges as fitted splines (and sometimes
    /// as conics) that follow the two faces only to the writer's tolerance, a few
    /// 1e-3 for a part of 100s of mm. This kernel's edges must lie on both faces,
    /// so an edge shared by two faces whose curve is off either surface by more
    /// than rounding but less than a file-tolerance sized amount is replaced by an
    /// interpolating spline through points refined onto the intersection of the
    /// two surfaces (end points stay at the vertices).
    fn snap_edges_to_surfaces(&mut self, specs: &[FaceSpec]) {
        let mut uses: HashMap<EdgeId, Vec<usize>> = HashMap::new();
        for (fi, s) in specs.iter().enumerate() {
            for l in &s.loops {
                for c in l {
                    uses.entry(c.edge).or_default().push(fi);
                }
            }
        }
        let mut todo: Vec<(EdgeId, usize, usize)> = uses.iter().filter(|(_, u)| u.len() == 2 && u[0] != u[1]).map(|(e, u)| (*e, u[0], u[1])).collect();
        todo.sort_by_key(|t| t.0.idx());
        let mut spline_budget = 250;
        for (e, fa, fb) in todo {
            let (sa, sb) = (&specs[fa].surface, &specs[fb].surface);
            if matches!(sa, Surface::Offset { .. } | Surface::Screw { .. }) || matches!(sb, Surface::Offset { .. } | Surface::Screw { .. }) {
                continue;
            }
            // free-form surfaces are costly to project onto: bound the work
            if matches!(sa, Surface::BSpline(_)) || matches!(sb, Surface::BSpline(_)) {
                if spline_budget == 0 {
                    continue;
                }
                spline_budget -= 1;
            }
            let ed = self.b.edge(e).clone();
            let n = if matches!(sa, Surface::BSpline(_)) || matches!(sb, Surface::BSpline(_)) { 8 } else { 16 };
            let scale = 1.0 + ed.curve.eval(ed.t0).len();
            let mut dev: f64 = 0.0;
            let mut feet: [Option<(f64, f64)>; 2] = [None, None];
            for k in 0..=n {
                let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64);
                for (i, sf) in [sa, sb].into_iter().enumerate() {
                    let (u, v, q) = foot(sf, p, feet[i]);
                    feet[i] = Some((u, v));
                    dev = dev.max(p.dist(q));
                }
            }
            if dev <= 2e-7 * scale || dev > 5e-3 * scale {
                continue;
            }
            let free_form = matches!(sa, Surface::BSpline(_)) || matches!(sb, Surface::BSpline(_));
            let m = if free_form { 96 } else { 160 };
            let mut pts: Vec<Vec3> = Vec::with_capacity(m + 1);
            let mut ok = true;
            let mut hint: Option<[f64; 4]> = None;
            let mut feet: [Option<(f64, f64)>; 2] = [None, None];
            for k in 0..=m {
                let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / m as f64);
                if k == 0 || k == m {
                    pts.push(p);
                    continue;
                }
                match snap_point_proj(sa, sb, p, 5e-3 * scale, &mut feet).or_else(|| snap_point(sa, sb, p, 5e-3 * scale, &mut hint)) {
                    Some(q) => pts.push(q),
                    None => {
                        ok = false;
                        break;
                    }
                }
            }
            if !ok {
                self.warnings.push(ImportError::new("skipped", None, format!("edge #{} lies {dev:.2e} off its faces and could not be moved onto their intersection", e.idx())));
                continue;
            }
            // a closed edge or a tangent-continuous start keeps the old ends
            if let Ok(bs) = ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength) {
                let edge = &mut self.b.edges[e.idx()];
                edge.curve = Curve::BSpline(bs);
                edge.t0 = 0.0;
                edge.t1 = 1.0;
            }
        }
    }

    /// Some writers (OpenCASCADE, on faces of reversed-axis tori) give a
    /// FACE_BOUND the orientation `.F.` although the loop as written already
    /// runs against its neighbours' edge uses. A flagged loop whose edges all
    /// run the same way as the faces next to them is put back the way it was
    /// written, which then makes every shared edge consistent.
    fn unflip_inconsistent_bounds(&self, specs: &mut [FaceSpec]) {
        // edge -> uses (face index, loop index, rev)
        let mut uses: HashMap<EdgeId, Vec<(usize, usize, bool)>> = HashMap::new();
        for (fi, s) in specs.iter().enumerate() {
            for (li, l) in s.loops.iter().enumerate() {
                for c in l {
                    uses.entry(c.edge).or_default().push((fi, li, c.rev));
                }
            }
        }
        for fi in 0..specs.len() {
            for li in 0..specs[fi].loops.len() {
                if !specs[fi].flagged[li] {
                    continue;
                }
                let (mut same, mut opp) = (0, 0);
                for c in &specs[fi].loops[li] {
                    let u = &uses[&c.edge];
                    // seam / doubly used edges are judged from other faces only
                    let others: Vec<&(usize, usize, bool)> = u.iter().filter(|x| x.0 != fi).collect();
                    if others.len() != 1 || u.iter().filter(|x| x.0 == fi).count() != 1 {
                        continue;
                    }
                    if others[0].2 == c.rev {
                        same += 1;
                    } else {
                        opp += 1;
                    }
                }
                if same > 0 && opp == 0 {
                    let l = &mut specs[fi].loops[li];
                    l.reverse();
                    for c in l.iter_mut() {
                        c.rev = !c.rev;
                    }
                    specs[fi].flagged[li] = false;
                }
            }
        }
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
            // (sign only: the quick, memoised quadrature; the validator reuses its per-face values)
            let vol = ck_topo::mass::signed_volume_quick(&self.b, &faces);
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

/// First parameter at or after `after` (domain start if None) where the
/// B-spline passes through `p`, by sampling + golden-section refinement.
fn first_pass(c: &Curve, dom: (f64, f64), ncp: usize, p: Vec3, after: Option<f64>) -> Option<f64> {
    let (d0, d1) = dom;
    let tol = 1e-6 * (1.0 + p.len());
    let skip = after.map(|t| t + 1e-9 * (d1 - d0).abs());
    let from = after.unwrap_or(d0);
    let n = (ncp * 8).clamp(128, 4096);
    let h = (d1 - from) / n as f64;
    if !(h > 0.0 && h.is_finite()) {
        return None;
    }
    let f = |t: f64| c.eval(t).dist(p);
    let mut ds: Vec<f64> = Vec::with_capacity(n + 2);
    ds.push(f(from));
    for i in 0..=n {
        let t = from + h * i as f64;
        // lazily one sample ahead
        if i < n {
            ds.push(f(from + h * (i + 1) as f64));
        }
        if skip.is_some_and(|s| t <= s) {
            continue;
        }
        let left = if i == 0 { f64::INFINITY } else { ds[i - 1] };
        let right = if i == n { f64::INFINITY } else { ds[i + 1] };
        if ds[i] <= left && ds[i] <= right && ds[i] < 0.5 * (1.0 + p.len()) {
            let (mut lo, mut hi) = ((t - h).max(from), (t + h).min(d1));
            for _ in 0..80 {
                let (a, b) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                if f(a) < f(b) {
                    hi = b;
                } else {
                    lo = a;
                }
            }
            let tm = 0.5 * (lo + hi);
            // exact domain ends stay exact
            let tm = if (tm - d0).abs() < 1e-9 * (d1 - d0) {
                d0
            } else if (tm - d1).abs() < 1e-9 * (d1 - d0) {
                d1
            } else {
                tm
            };
            if f(tm) < tol * 10.0 {
                return Some(tm);
            }
        }
    }
    None
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
            if !(l.is_finite() && l > 1e-15) {
                return None;
            }
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
        (Surface::Extrusion { profile, dir }, 0) => Some(Iso { curve: Curve::Line { o: profile.eval(s), d: *dir }, k: 1.0 }),
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
            let h0 = frame.z.cross(cf.z).try_normalize()?;
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

/// Edge curve = the extrusion profile moved along the sweep direction (same
/// parameter): the pcurve is the line v = const.
fn exact_extrusion(surf: &Surface, c: &Curve, t0: f64, t1: f64) -> Option<Curve2> {
    let Surface::Extrusion { profile, dir } = surf else { return None };
    let (Curve::BSpline(a), Curve::BSpline(b)) = (&**profile, c) else { return None };
    if a.degree != b.degree || a.knots != b.knots || a.ctrl.len() != b.ctrl.len() {
        return None;
    }
    let off = c.eval(t0) - profile.eval(t0);
    let v0 = off.dot(*dir);
    let scale = 1.0 + off.len();
    for k in 1..=8 {
        let t = t0 + (t1 - t0) * k as f64 / 8.0;
        let o = c.eval(t) - profile.eval(t);
        if (o - *dir * v0).len() > 1e-7 * scale {
            return None;
        }
    }
    Some(Curve2::Line { o: Vec2::new(0.0, v0), d: Vec2::new(1.0, 0.0) })
}

/// Smallest period of a closed B-spline (ck-model unrolls closed splines over
/// several laps so they can be cut anywhere), None if not closed.
fn spline_period(c: &Curve, dom: (f64, f64)) -> Option<f64> {
    let (d0, d1) = dom;
    let l = d1 - d0;
    let scale = 1.0 + c.eval(d0).len();
    if c.eval(d0).dist(c.eval(d1)) > 1e-6 * scale {
        return None;
    }
    for k in (2..=6).rev() {
        let p = l / k as f64;
        let ok = (0..16).all(|i| {
            let t = d0 + (l - p) * (i as f64 + 0.37) / 16.0;
            c.eval(t).dist(c.eval(t + p)) < 1e-6 * scale
        });
        if ok {
            return Some(p);
        }
    }
    Some(l)
}

/// `Surface::periods` plus extrusions of a closed B-spline profile (the
/// profile parameter wraps at the end of its period).
fn periods_of(surf: &Surface) -> (Option<f64>, Option<f64>) {
    let (pu, pv) = surf.periods();
    if let Surface::Extrusion { profile, .. } = surf {
        if let (None, Curve::BSpline(bs)) = (pu, &**profile) {
            if let Some(p) = spline_period(profile, bs.domain()) {
                return (Some(p), pv);
            }
        }
    }
    (pu, pv)
}

/// A B-spline lying in a plane: the pcurve is its control net in plane
/// coordinates (exact, same parameter).
fn exact_plane_spline(surf: &Surface, c: &Curve) -> Option<Curve2> {
    let (Surface::Plane { frame }, Curve::BSpline(bs)) = (surf, c) else { return None };
    let scale = 1.0 + bs.ctrl.iter().map(|p| p.len()).fold(0.0, f64::max);
    if bs.ctrl.iter().any(|p| (*p - frame.o).dot(frame.z).abs() > 1e-7 * scale) {
        return None;
    }
    Some(Curve2::BSpline(bs.map(|p| {
        let l = frame.to_local(p);
        Vec2::new(l.x, l.y)
    })))
}

fn edge_pcurve(b: &Body, e: EdgeId, surf: &Surface) -> Option<Curve2> {
    let ed = b.edge(e);
    if let Some(p) = exact_plane_spline(surf, &ed.curve) {
        return Some(p);
    }
    if let Some(p) = exact_extrusion(surf, &ed.curve, ed.t0, ed.t1) {
        return Some(p);
    }
    if let Some(p) = exact_ext(surf, &ed.curve, ed.t0, ed.t1) {
        return Some(p);
    }
    if let Some(p) = exact_pcurve(surf, &ed.curve, 1e-7) {
        return Some(p);
    }
    if matches!(surf, Surface::BSpline(_)) {
        return pcurve_of(surf, &ed.curve, ed.t0, ed.t1, 1e-6).ok();
    }
    pcurve_of_dense(surf, &ed.curve, ed.t0, ed.t1, 1e-6, 2e-6).ok()
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
                out.push(FaceSpec { id, surface, sense, loops: vec![loop_], flagged: vec![false] });
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
                if !(s.is_finite() && n.is_finite()) {
                    return None;
                }
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
                let p = sc.eval(0.0);
                if !p.is_finite() {
                    return None;
                }
                let v = self.vertex_at(p);
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

    fn finalize_face(&mut self, shell: ShellId, spec: FaceSpec, name: FaceName, rest: &mut [Option<(ShellId, FaceSpec)>]) -> R<()> {
        let FaceSpec { id, surface, mut sense, mut loops, .. } = spec;
        let (pu, pv) = periods_of(&surface);
        let periodic = pu.is_some() || pv.is_some();
        // pcurves
        for l in &mut loops {
            for c in l.iter_mut() {
                if c.pc.is_none() {
                    c.pc = edge_pcurve(&self.b, c.edge, &surface);
                }
            }
        }
        if matches!(surface, Surface::Sphere { .. }) {
            // a hemisphere bounded by one great circle through the poles
            for li in 0..loops.len() {
                if loops[li].len() == 1 && self.is_full_meridian(&loops[li][0]) {
                    let mut l = std::mem::take(&mut loops[li]);
                    for s in [FRAC_PI_2, -FRAC_PI_2] {
                        for _ in 0..3 {
                            if !self.split_loop_at(&mut l, 1, s, TAU, rest) {
                                break;
                            }
                        }
                    }
                    for c in &mut l {
                        self.fold_sphere_meridian(c);
                    }
                    // two meridian arcs, joined by jumps across the poles: put the
                    // second half a half turn to the side that makes the loop run
                    // the way the face sense asks for
                    if l.len() == 2 {
                        let want = if sense { 1.0 } else { -1.0 };
                        if let (Some(e0), Some(s1), Some(Curve2::Line { o, d })) = (self.co_uv(&l[0], false), self.co_uv(&l[1], true), l[1].pc.clone()) {
                            for sigma in [1.0, -1.0] {
                                let dx = e0.x + sigma * PI - s1.x;
                                let mut trial = l.clone();
                                trial[1].pc = Some(Curve2::Line { o: Vec2::new(o.x + dx, o.y), d });
                                if self.loop_area(&trial) * want > 0.0 {
                                    l = trial;
                                    break;
                                }
                            }
                        }
                    }
                    loops[li] = l;
                }
            }
        }
        if !periodic && matches!(surface, Surface::BSpline(_)) {
            for l in &mut loops {
                self.fix_closed_spline_loop(l, &surface, sense);
            }
            self.close_spline_pole_cap(&mut loops, &surface, sense);
            self.join_spline_rings(&mut loops, &surface, sense);
        }
        if periodic {
            if loops.iter().flatten().any(|c| c.pc.is_none()) {
                return Err(ImportError::new("topology", Some(id), "no pcurve for an edge of a periodic face"));
            }
            for l in &mut loops {
                self.unwrap_loop(l, pu, pv);
                self.fix_seam_pair(l, &surface, sense, pu, pv);
            }
            loops = self.complete_periodic(id, &surface, sense, loops, pu, pv, rest)?;
            // A cone face whose outer loop runs against the face sense in uv
            // (SolidWorks writes FACE_OUTER_BOUND .F. with sense .F. on the half
            // cones of a drill point): the edge directions agree with the
            // neighbouring faces, so the normal is the one the loop is CCW about.
            if matches!(surface, Surface::Cone { .. }) && !loops.is_empty() && loops[0].iter().all(|c| c.pc.is_some()) {
                let a = self.loop_area(&loops[0]);
                if a.abs() > 1e-9 && (a > 0.0) != sense {
                    sense = a > 0.0;
                }
            }
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

    /// A B-spline surface that closes on itself (a sweep of a circle stored as
    /// one patch: u closed) is bounded by a loop that runs up one side of the
    /// seam and back down the other. Projection cannot tell the two sides
    /// apart (both land on the same u), so the pcurves of the loop (already
    /// unwrapped within each edge) are shifted by whole periods: the seam
    /// twins to opposite sides, the two chains of edges between them to join
    /// them up, oriented to the face sense. Left alone if the result does not
    /// reproduce the 3D edges.
    fn fix_closed_spline_loop(&self, l: &mut [Co], surf: &Surface, sense: bool) {
        let Surface::BSpline(bs) = surf else { return };
        let n = l.len();
        if n < 3 || l.iter().any(|c| c.pc.is_none()) {
            return;
        }
        let (cu, cv) = ck_geom::pcurve::closed_periods(surf);
        let (d, per) = match (cu, cv) {
            (Some(p), None) => (0usize, p),
            (None, Some(p)) => (1usize, p),
            _ => return,
        };
        if !(per.is_finite() && per > 0.0) {
            return;
        }
        let pairs: Vec<(usize, usize)> = (0..n).flat_map(|i| (i + 1..n).map(move |j| (i, j))).filter(|&(i, j)| l[i].edge == l[j].edge && l[i].rev != l[j].rev).collect();
        let (du, dv) = bs.domain();
        let (lo, hi) = if d == 0 { du } else { dv };
        let tol = 1e-5 * per;
        let [(i, j)] = pairs[..] else {
            // No seam pair: a face that only touches the seam (a notch at the end of the range) has one coedge on the seam,
            // whose pcurve the file may put at the other end of the range from its neighbours, so the loop jumps a whole
            // period. Shift each pcurve to start where the one before it ends; keep that only if the loop then closes
            // inside the parameter range.
            // Several seam pairs (a face whose boundary meets the seam at two places, an island and its outer loop joined along
            // the seam) are chained the same way: each coedge continues where the one before it ends, the twins of a pair
            // landing a period apart; kept only when the loop closes, stays in range and runs the face's way round.
            if pairs.is_empty() || pairs.len() >= 2 {
                let mut t: Vec<Co> = l.to_vec();
                let mut ok = true;
                for k in 1..n {
                    let (Some(e), Some(s0)) = (self.co_uv(&t[k - 1], false), self.co_uv(&t[k], true)) else { return };
                    let sh = ((comp(e, d) - comp(s0, d)) / per).round() * per;
                    if sh != 0.0 {
                        let Some(pc) = t[k].pc.as_ref() else { return };
                        t[k].pc = Some(translate(pc, unit2(d, sh)));
                    }
                }
                let (Some(e), Some(s0)) = (self.co_uv(&t[n - 1], false), self.co_uv(&t[0], true)) else { return };
                ok &= (comp(e, d) - comp(s0, d)).abs() <= tol;
                // (the whole chain may sit a period off the range: bring it in by the shift of its mean)
                let (mut sum, mut cnt) = (0.0, 0usize);
                for c in &t {
                    let (a, z) = self.co_ts(c);
                    let Some(pc) = c.pc.as_ref() else { return };
                    for q in 0..=8 {
                        sum += comp(pc.eval(a + (z - a) * q as f64 / 8.0), d);
                        cnt += 1;
                    }
                }
                if !pairs.is_empty() && cnt > 0 {
                    let sh = -((sum / cnt as f64 - 0.5 * (lo + hi)) / per).round() * per;
                    if sh != 0.0 {
                        for c in t.iter_mut() {
                            let Some(pc) = c.pc.as_ref() else { return };
                            c.pc = Some(translate(pc, unit2(d, sh)));
                        }
                    }
                }
                for c in &t {
                    let (a, z) = self.co_ts(c);
                    let Some(pc) = c.pc.as_ref() else { return };
                    for q in 0..=8 {
                        let x = comp(pc.eval(a + (z - a) * q as f64 / 8.0), d);
                        ok &= x >= lo - tol && x <= hi + tol;
                    }
                }
                if !pairs.is_empty() {
                    ok &= self.loop_area(&t) * if sense { 1.0 } else { -1.0 } > 0.0;
                }
                if ok {
                    l.clone_from_slice(&t);
                }
            }
            return;
        };
        let shifted = |c: &Co, x: f64| -> Option<Co> {
            let pc = c.pc.as_ref()?;
            let mut o = c.clone();
            o.pc = Some(translate(pc, unit2(d, x)));
            Some(o)
        };
        for first in [lo, hi] {
            let other = if first == lo { hi } else { lo };
            let mut t: Vec<Co> = l.to_vec();
            let mut ok = true;
            // seam twins
            for (k, lvl) in [(i, first), (j, other)] {
                let Some(uv) = self.co_uv(&l[k], true) else { return };
                let x = comp(uv, d);
                let sh = ((lvl - x) / per).round() * per;
                if (x + sh - lvl).abs() > tol {
                    ok = false;
                }
                let Some(sc) = shifted(&l[k], sh) else { return };
                t[k] = sc;
            }
            // the two chains between the twins: from `lvl_from` to `lvl_to`
            for (start, count, lvl_from, lvl_to) in [(i + 1, j - i - 1, first, other), (j + 1, n - (j - i) - 1, other, first)] {
                let mut at = lvl_from;
                for q in 0..count {
                    let k = (start + q) % n;
                    let Some(uv) = self.co_uv(&l[k], true) else { return };
                    let x = comp(uv, d);
                    let sh = ((at - x) / per).round() * per;
                    let Some(sc) = shifted(&l[k], sh) else { return };
                    t[k] = sc;
                    let Some(end_uv) = self.co_uv(&t[k], false) else { return };
                    at = comp(end_uv, d);
                }
                if (at - lvl_to).abs() > tol {
                    ok = false;
                }
            }
            if !ok {
                continue;
            }
            // the pcurves must reproduce the 3D edges
            let scale = 1.0 + bs.eval(du.0, dv.0).len();
            // (a shift by whole periods leaves every point where it was: the edge may deviate from the face by the file's own
            // tolerance, so the test is that no pcurve got worse, not that the deviation is small)
            let dev = |c: &Co| -> Option<f64> {
                let ed = self.b.edge(c.edge);
                let tm = 0.5 * (ed.t0 + ed.t1);
                let uv = c.pc.as_ref()?.eval(tm);
                Some(bs.eval(uv.x, uv.y).dist(ed.curve.eval(tm)))
            };
            let mut good = true;
            for (c, c0) in t.iter().zip(l.iter()) {
                let (Some(dc), Some(dc0)) = (dev(c), dev(c0)) else { return };
                good &= dc < 1e-4 * scale || dc <= dc0 + 1e-6 * scale;
            }
            if good && self.loop_area(&t) * if sense { 1.0 } else { -1.0 } > 0.0 {
                l.clone_from_slice(&t);
                return;
            }
        }
    }

    /// A face of a B-spline surface closed in one direction (a revolved spline) that is cut off by one closed edge running once
    /// round the closed direction, the other end of the surface being a pole (a row of control points that are one point). Other
    /// systems write such a face with that single edge and no seam or pole edge (as does our export); the loop is closed here
    /// with the meridian to the pole and back, the pole staying implicit, as for a cone's cap. The rim must start on the
    /// surface's own seam; left alone otherwise, or when the loop would not run the face's way.
    fn close_spline_pole_cap(&mut self, loops: &mut Vec<Vec<Co>>, surf: &Surface, sense: bool) {
        let Surface::BSpline(bs) = surf else { return };
        if loops.len() != 1 || loops[0].len() != 1 {
            return;
        }
        let (cu, cv) = ck_geom::pcurve::closed_periods(surf);
        let (d, per) = match (cu, cv) {
            (Some(p), None) => (0usize, p),
            (None, Some(p)) => (1usize, p),
            _ => return,
        };
        let rim = loops[0][0].clone();
        let ed = self.b.edge(rim.edge);
        if ed.v0 != ed.v1 {
            return;
        }
        let va = ed.v0;
        let (Some(s_uv), Some(e_uv)) = (self.co_uv(&rim, true), self.co_uv(&rim, false)) else { return };
        let tol = 1e-5 * per;
        if ((comp(e_uv, d) - comp(s_uv, d)).abs() - per).abs() > tol || (comp(e_uv, 1 - d) - comp(s_uv, 1 - d)).abs() > tol {
            return;
        }
        let ((u0, u1), (v0, v1)) = bs.domain();
        let ((s_lo, s_hi), (a_lo, a_hi)) = if d == 0 { ((u0, u1), (v0, v1)) } else { ((v0, v1), (u0, u1)) };
        let (vs, ve, a_loop) = (comp(s_uv, d), comp(e_uv, d), comp(s_uv, 1 - d));
        let on_seam = |x: f64| (x - s_lo).abs() <= tol || (x - s_hi).abs() <= tol;
        if !on_seam(vs) || !on_seam(ve) {
            return;
        }
        let same = |pts: Vec<Vec3>| pts.windows(2).all(|w| w[0].dist(w[1]) <= 1e-9 * (1.0 + w[0].len()));
        let (nu, nv) = (bs.nu, bs.nv);
        // d == 1: closed in v, the poles are rows of u (`ctrl[i * nv + j]`); d == 0: columns
        let (pole_at_lo, pole_at_hi) = if d == 1 {
            let row = |i: usize| (0..nv).map(|j| bs.ctrl[i * nv + j]).collect::<Vec<_>>();
            (same(row(0)), same(row(nu - 1)))
        } else {
            let col = |j: usize| (0..nu).map(|i| bs.ctrl[i * nv + j]).collect::<Vec<_>>();
            (same(col(0)), same(col(nv - 1)))
        };
        if pole_at_lo == pole_at_hi {
            return;
        }
        let a_pole = if pole_at_lo { a_lo } else { a_hi };
        if (a_pole - a_loop).abs() < 1e-9 * (1.0 + a_pole.abs()) {
            return;
        }
        let curve = if d == 1 { bs.iso_v(vs) } else { bs.iso_u(vs) };
        let pole_p = curve.eval(a_pole);
        // the meridian must reach the rim's vertex
        if !pole_p.is_finite() || curve.eval(a_loop).dist(self.b.vertex(va).p) > 1e-4 * (1.0 + pole_p.len()) {
            return;
        }
        let pole_first = a_pole < a_loop;
        let (t0, t1) = if pole_first { (a_pole, a_loop) } else { (a_loop, a_pole) };
        let line = |at: f64| if d == 1 { Curve2::Line { o: Vec2::new(0.0, at), d: Vec2::new(1.0, 0.0) } } else { Curve2::Line { o: Vec2::new(at, 0.0), d: Vec2::new(0.0, 1.0) } };
        // the rim ends at `ve` and starts at `vs`: up the seam at `ve` to the pole, jump to `vs`, back down to the start
        let seam = self.b.add_edge(Curve::BSpline(curve), t0, t1, va, va);
        let l = vec![rim, Co { edge: seam, rev: pole_first, pc: Some(line(ve)) }, Co { edge: seam, rev: !pole_first, pc: Some(line(vs)) }];
        if self.loop_area(&l) * if sense { 1.0 } else { -1.0 } > 0.0 {
            let pole = self.vertex_at(pole_p);
            let e = &mut self.b.edges[seam.idx()];
            if pole_first {
                e.v0 = pole;
            } else {
                e.v1 = pole;
            }
            loops[0] = l;
        } else {
            self.b.edges[seam.idx()].alive = false;
        }
    }

    /// A ring of a B-spline surface closed in one direction (a revolved spline): two closed edges, each running once round the
    /// closed direction on the surface's seam, written as two loops with no seam edge (as other systems write it, and as our
    /// export does). They become one loop joined by a seam edge, run up the seam and back down, as for a cylinder's ring.
    /// Left alone when the two do not run opposite ways or the loop would not run the face's way.
    fn join_spline_rings(&mut self, loops: &mut Vec<Vec<Co>>, surf: &Surface, sense: bool) {
        let Surface::BSpline(bs) = surf else { return };
        if loops.len() != 2 || loops.iter().any(|l| l.len() != 1) {
            return;
        }
        let (cu, cv) = ck_geom::pcurve::closed_periods(surf);
        let (d, per) = match (cu, cv) {
            (Some(p), None) => (0usize, p),
            (None, Some(p)) => (1usize, p),
            _ => return,
        };
        let (a, b) = (loops[0][0].clone(), loops[1][0].clone());
        if a.edge == b.edge {
            return;
        }
        let (ea, eb) = (self.b.edge(a.edge), self.b.edge(b.edge));
        if ea.v0 != ea.v1 || eb.v0 != eb.v1 {
            return;
        }
        let (va, vb) = (ea.v0, eb.v0);
        let tol = 1e-5 * per;
        let ((u0, u1), (v0, v1)) = bs.domain();
        let (s_lo, s_hi) = if d == 0 { (u0, u1) } else { (v0, v1) };
        let on_seam = |x: f64| (x - s_lo).abs() <= tol || (x - s_hi).abs() <= tol;
        let (Some(sa), Some(ea_uv), Some(sb), Some(eb_uv)) = (self.co_uv(&a, true), self.co_uv(&a, false), self.co_uv(&b, true), self.co_uv(&b, false)) else { return };
        for (x, y) in [(sa, ea_uv), (sb, eb_uv)] {
            if ((comp(y, d) - comp(x, d)).abs() - per).abs() > tol || (comp(y, 1 - d) - comp(x, 1 - d)).abs() > tol || !on_seam(comp(x, d)) || !on_seam(comp(y, d)) {
                return;
            }
        }
        // the two run opposite ways: B starts where A ends (mod the period) and ends where A starts
        if (comp(sb, d) - comp(ea_uv, d)).abs() > tol && ((comp(sb, d) - comp(ea_uv, d)).abs() - per).abs() > tol {
            return;
        }
        let shift = comp(ea_uv, d) - comp(sb, d);
        let Some(pc_b) = b.pc.as_ref() else { return };
        let b_shifted = Co { edge: b.edge, rev: b.rev, pc: Some(translate(pc_b, unit2(d, (shift / per).round() * per))) };
        let (a_loop, b_loop) = (comp(sa, 1 - d), comp(sb, 1 - d));
        if (a_loop - b_loop).abs() < 1e-9 * (1.0 + a_loop.abs()) {
            return;
        }
        let (v_a_end, v_a_start) = (comp(ea_uv, d), comp(sa, d));
        let seam_at = v_a_start;
        let curve = if d == 1 { bs.iso_v(seam_at) } else { bs.iso_u(seam_at) };
        let (lo, hi, v_lo, v_hi) = if a_loop < b_loop { (a_loop, b_loop, va, vb) } else { (b_loop, a_loop, vb, va) };
        if curve.eval(a_loop).dist(self.b.vertex(va).p) > 1e-4 * (1.0 + curve.eval(a_loop).len()) || curve.eval(b_loop).dist(self.b.vertex(vb).p) > 1e-4 * (1.0 + curve.eval(b_loop).len()) {
            return;
        }
        let line = |at: f64| if d == 1 { Curve2::Line { o: Vec2::new(0.0, at), d: Vec2::new(1.0, 0.0) } } else { Curve2::Line { o: Vec2::new(at, 0.0), d: Vec2::new(0.0, 1.0) } };
        let seam = self.b.add_edge(Curve::BSpline(curve), lo, hi, v_lo, v_hi);
        let a_below = a_loop < b_loop;
        // up the seam from A's end to B's start, B round, and back down to A's start
        let l = vec![a, Co { edge: seam, rev: !a_below, pc: Some(line(v_a_end)) }, b_shifted, Co { edge: seam, rev: a_below, pc: Some(line(v_a_start)) }];
        if self.loop_area(&l) * if sense { 1.0 } else { -1.0 } > 0.0 {
            *loops = vec![l];
        } else {
            self.b.edges[seam.idx()].alive = false;
        }
    }

    /// Shift pcurves by whole periods so consecutive coedges join up.
    fn unwrap_loop(&self, l: &mut [Co], pu: Option<f64>, pv: Option<f64>) {
        let mut prev: Option<Vec2> = None;
        for i in 0..l.len() {
            let (a, z) = self.co_ts(&l[i]);
            let Some(pc) = l[i].pc.clone() else {
                prev = None;
                continue;
            };
            if let Some(p) = prev {
                let s = pc.eval(a);
                let mut d = Vec2::new(0.0, 0.0);
                if let Some(per) = pu {
                    // (a jump of exactly half a period stays: a pole jump)
                    let r = (p.x - s.x) / per;
                    d.x = if r.abs() <= 0.5 + 1e-9 { 0.0 } else { r.round() * per };
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
        if !(per.is_finite() && per > 0.0) {
            return None;
        }
        let mut s = 0.0;
        for c in l {
            let (a, z) = (self.co_uv(c, true)?, self.co_uv(c, false)?);
            s += comp(z, d) - comp(a, d);
        }
        let k = (s / per).round();
        (k.is_finite() && k.abs() < 1e15 && (s / per - k).abs() < 1e-6).then_some(k as i64)
    }

    fn complete_periodic(
        &mut self,
        id: u32,
        surf: &Surface,
        sense: bool,
        loops: Vec<Vec<Co>>,
        pu: Option<f64>,
        pv: Option<f64>,
        rest: &mut [Option<(ShellId, FaceSpec)>],
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
        if wind.iter().any(|w| w.1 != d || (w.2 != 1 && w.2 != -1)) {
            return Err(ImportError::new("unsupported", Some(id), "periodic face whose boundary winds more than once or around both directions"));
        }
        let Some(per) = pers[d] else {
            return Err(ImportError::new("topology", Some(id), "missing period for winding axis"));
        };
        let mut loops = loops;
        match wind.len() {
            2 if wind[0].2 == -wind[1].2 => {
                let (ia, ib) = (wind[0].0, wind[1].0);
                let (la, lb) = (loops[ia].clone(), loops[ib].clone());
                let joined = self.join_two(id, surf, sense, d, per, pers[1 - d], la, lb, wind[0].2, rest)?;
                // replace the first by the joined loop, drop the second
                loops[ia] = joined;
                loops.remove(ib);
                Ok(loops)
            }
            1 => {
                let ia = wind[0].0;
                let la = loops[ia].clone();
                // a loop that already runs into the pole and out again (the
                // seam spike OCC writes, with no degenerate edge) is closed
                // in 3D by the pole jump in uv: nothing to add
                if self.has_pole_jump(surf, &la, d, per) {
                    return Ok(loops);
                }
                let joined = self.close_at_pole(id, surf, sense, d, per, la, wind[0].2)?;
                loops[ia] = joined;
                Ok(loops)
            }
            _ => Err(ImportError::new("unsupported", Some(id), "periodic face with an unsupported set of winding boundaries")),
        }
    }

    /// A coedge whose pcurve is a v-line (constant u) running a full turn
    /// round a sphere: a great circle through both poles.
    fn is_full_meridian(&self, c: &Co) -> bool {
        let Some(Curve2::Line { d, .. }) = &c.pc else { return false };
        let ed = self.b.edge(c.edge);
        d.x.abs() < 1e-12 && d.y.abs() * (ed.t1 - ed.t0) >= TAU - 1e-6
    }

    /// Bring a v-line that runs past a sphere pole back to the v in
    /// [-pi/2, pi/2] band: the point (u, v) of the far side is (u + pi, pi - v).
    fn fold_sphere_meridian(&self, c: &mut Co) {
        let Some(Curve2::Line { o, d }) = c.pc.clone() else { return };
        if d.x.abs() > 1e-12 {
            return;
        }
        let ed = self.b.edge(c.edge);
        let tm = 0.5 * (ed.t0 + ed.t1);
        let vm = o.y + d.y * tm;
        let m = (vm / TAU).round();
        let (mut ox, mut oy, mut dy) = (o.x, o.y - m * TAU, d.y);
        let v0 = vm - m * TAU;
        if v0 > FRAC_PI_2 {
            ox += PI;
            oy = PI - oy;
            dy = -dy;
        } else if v0 < -FRAC_PI_2 {
            ox += PI;
            oy = -PI - oy;
            dy = -dy;
        }
        c.pc = Some(Curve2::Line { o: Vec2::new(ox, oy), d: Vec2::new(0.0, dy) });
    }

    /// Consecutive coedges whose uv ends differ by a whole period along `d`
    /// only: a jump across a pole.
    fn has_pole_jump(&self, surf: &Surface, l: &[Co], d: usize, per: f64) -> bool {
        let n = l.len();
        (0..n).any(|i| {
            let (Some(a), Some(b)) = (self.co_uv(&l[i], false), self.co_uv(&l[(i + 1) % n], true)) else { return false };
            let (dd, od) = (comp(b, d) - comp(a, d), comp(b, 1 - d) - comp(a, 1 - d));
            if !(od.abs() < 1e-6 && dd.abs() > 0.5 * per) {
                return false;
            }
            // the closing jump of a loop that winds once (one circle round a
            // cone / sphere) is a whole period too: it is a pole jump only
            // where the surface degenerates (the u-partial vanishes)
            if i + 1 < n {
                return true;
            }
            let (du, dv) = surf.partials(a.x, a.y);
            let g = if d == 0 { du } else { dv };
            g.len() < 1e-9 * (1.0 + surf.eval(a.x, a.y).len())
        })
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

    /// Split the edge of one coedge of `l` where its pcurve crosses
    /// coordinate `s` (mod `per`) along axis `d`; patches `l`, the faces
    /// already built and the pending specs. True if a split happened.
    fn split_loop_at(&mut self, l: &mut Vec<Co>, d: usize, s: f64, per: f64, rest: &mut [Option<(ShellId, FaceSpec)>]) -> bool {
        for i in 0..l.len() {
            let Some(pc) = l[i].pc.clone() else { continue };
            let ed = self.b.edge(l[i].edge);
            let (t0, t1, e) = (ed.t0, ed.t1, l[i].edge);
            let n = 64;
            let h = |t: f64| (comp(pc.eval(t), d) - s) / per;
            let mut found = None;
            for k in 0..n {
                let (a, b) = (t0 + (t1 - t0) * k as f64 / n as f64, t0 + (t1 - t0) * (k + 1) as f64 / n as f64);
                let (ha, hb) = (h(a), h(b));
                if ha.floor() != hb.floor() {
                    let m = if hb > ha { hb.floor() } else { ha.floor() };
                    let (mut lo, mut hi) = (a, b);
                    for _ in 0..80 {
                        let mid = 0.5 * (lo + hi);
                        if (h(mid) - m) * (ha - m) > 0.0 {
                            lo = mid;
                        } else {
                            hi = mid;
                        }
                    }
                    found = Some(0.5 * (lo + hi));
                    break;
                }
            }
            let Some(t) = found else { continue };
            let eps = 1e-9 * (1.0 + (t1 - t0).abs());
            if t - t0 < eps || t1 - t < eps {
                continue;
            }
            let (_, ne) = ck_topo::edit::split_edge(&mut self.b, e, t);
            let rev = l[i].rev;
            let (a, b) = (Co { edge: e, rev, pc: l[i].pc.clone() }, Co { edge: ne, rev, pc: l[i].pc.clone() });
            if rev {
                l.splice(i..=i, [b, a]);
            } else {
                l.splice(i..=i, [a, b]);
            }
            for sp in rest.iter_mut().flatten() {
                for lp in &mut sp.1.loops {
                    let mut out = Vec::with_capacity(lp.len() + 1);
                    for c in lp.drain(..) {
                        if c.edge == e {
                            let (a, b) = (Co { edge: e, rev: c.rev, pc: None }, Co { edge: ne, rev: c.rev, pc: None });
                            if c.rev {
                                out.extend([b, a]);
                            } else {
                                out.extend([a, b]);
                            }
                        } else {
                            out.push(c);
                        }
                    }
                    *lp = out;
                }
            }
            return true;
        }
        false
    }

    #[allow(clippy::too_many_arguments)]
    fn join_two(
        &mut self,
        id: u32,
        surf: &Surface,
        sense: bool,
        d: usize,
        per: f64,
        per_o: Option<f64>,
        la: Vec<Co>,
        mut lb: Vec<Co>,
        wa: i64,
        rest: &mut [Option<(ShellId, FaceSpec)>],
    ) -> R<Vec<Co>> {
        let o = 1 - d;
        let fail = || ImportError::new("topology", Some(id), "periodic face whose two boundaries have no vertices at a common seam position");
        if la.is_empty() || lb.is_empty() {
            return Err(fail());
        }
        // candidate seam positions: the start coordinate of every coedge of A
        let mut chosen = None;
        for i in 0..la.len() {
            let s = comp(self.co_uv(&la[i], true).ok_or_else(fail)?, d);
            if let Some((rb, _)) = self.rotate_to(&lb, d, s, per) {
                chosen = Some((i, rb));
                break;
            }
        }
        if chosen.is_none() {
            // no common seam position: cut B's boundary where it crosses A's first vertex
            let first_a = la.first().ok_or_else(fail)?;
            let s = comp(self.co_uv(first_a, true).ok_or_else(fail)?, d);
            if self.split_loop_at(&mut lb, d, s, per, rest) {
                if let Some((rb, _)) = self.rotate_to(&lb, d, s, per) {
                    chosen = Some((0, rb));
                }
            }
        }
        let (ia, mut rb) = chosen.ok_or_else(fail)?;
        let mut ra = la[ia..].to_vec();
        ra.extend_from_slice(&la[..ia]);
        // The loops were unwrapped from their first coedge, so the +-period
        // closure jump sits where that coedge was; rotating moved it into
        // the middle. Unwrap again from the new start so each loop is
        // continuous and gains its whole winding at the end.
        let (pu, pv) = if d == 0 { (Some(per), per_o) } else { (per_o, Some(per)) };
        self.unwrap_loop(&mut ra, pu, pv);
        self.unwrap_loop(&mut rb, pu, pv);
        let a0 = self.co_uv(ra.first().ok_or_else(fail)?, true).ok_or_else(fail)?;
        let s = comp(a0, d);
        // shift B so its start meets the end of the seam leg
        let b0 = self.co_uv(rb.first().ok_or_else(fail)?, true).ok_or_else(fail)?;
        let target_d = s + wa as f64 * per;
        let mut sh = unit2(d, ((target_d - comp(b0, d)) / per).round() * per);
        if let Some(po) = per_o.filter(|&po| po.is_finite() && po > 0.0) {
            // B's copy along the other period: the face lies to the left of A's
            // travel (seen against the face normal, so to the right for a
            // reversed face). When the loops are not half a period apart the
            // nearest copy is the same thing; at exactly half (a half torus)
            // this orientation is all there is to decide by.
            let plus_o = (if d == 0 { wa > 0 } else { wa < 0 }) == sense;
            let diff = comp(b0, o) - comp(a0, o);
            let mut k = ((comp(a0, o) - comp(b0, o)) / po).round();
            let lo_hi = |k: f64| diff + k * po;
            // wanted: o_b - o_a in (0, po) for plus_o, (-po, 0) otherwise
            let want_k = if plus_o { (-diff / po).ceil() } else { (-diff / po).floor() };
            let near_half = ((lo_hi(k)).abs() - 0.5 * po).abs() < 1e-6 * po;
            if near_half {
                k = want_k;
            }
            sh = with(sh, o, k * po);
        }
        for c in rb.iter_mut() {
            c.pc = c.pc.as_ref().map(|p| translate(p, sh));
        }
        let b0 = self.co_uv(rb.first().ok_or_else(fail)?, true).ok_or_else(fail)?;
        // seam edge from vertex of A's start to vertex of B's start
        let va = self.co_start_vertex(ra.first().ok_or_else(fail)?);
        let vb = self.co_start_vertex(rb.first().ok_or_else(fail)?);
        let (o_a, o_b) = (comp(a0, o), comp(b0, o));
        let isoc = iso(surf, d, s).ok_or_else(|| ImportError::new("unsupported", Some(id), format!("seam on a {} surface", surf.kind())))?;
        if (o_b - o_a).abs() < 1e-12 {
            return Err(ImportError::new("topology", Some(id), "degenerate seam"));
        }
        // t of the seam curve at coordinate o: the iso curve has o(t) = k·t + c with c fixed by its start point
        let t_of = |ob: f64| -> f64 { self.iso_t(&isoc, surf, d, s, ob) };
        let (ta, tb) = (t_of(o_a), t_of(o_b));
        if !(ta.is_finite() && tb.is_finite() && ta != tb) {
            return Err(ImportError::new("topology", Some(id), "degenerate seam"));
        }
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
        let t = Self::param(&isoc.curve, p, None);
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
        let c0 = la.first().ok_or_else(|| fail("empty loop"))?;
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
        if !pole_p.is_finite() {
            return Err(fail("non-finite pole point"));
        }
        let vp = self.vertex_at(pole_p);
        let va = self.co_start_vertex(c0);
        if va == vp {
            return Err(fail("boundary starts at the pole"));
        }
        let isoc = iso(surf, 0, s).ok_or_else(|| fail("no seam curve"))?;
        let (ta, tb) = (self.iso_t(&isoc, surf, 0, s, a0.y), self.iso_t(&isoc, surf, 0, s, v_pole));
        if !(ta.is_finite() && tb.is_finite() && ta != tb) {
            return Err(fail("degenerate pole seam"));
        }
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

/// `x.clamp(lo, hi)` that leaves `x` alone when the bounds are NaN or reversed
/// (a surface fitted through absurd coordinates has no usable domain); the plain
/// `clamp` panics on such bounds.
fn clamp_to(x: f64, lo: f64, hi: f64) -> f64 {
    if lo <= hi {
        x.clamp(lo, hi)
    } else {
        x
    }
}

/// Refine `p` onto the intersection of two surfaces: Gauss-Newton on
/// S_a(u1, v1) - S_b(u2, v2) = 0 (three equations, four unknowns; the minimum-norm
/// step keeps the point near where it started), from the feet of `p`. `None`
/// when it does not converge or moves more than `cap`.
fn snap_point(sa: &Surface, sb: &Surface, p0: Vec3, cap: f64, hint: &mut Option<[f64; 4]>) -> Option<Vec3> {
    let scale = 1.0 + p0.len();
    // (the feet of the previous sample start the search: far cheaper than a global
    // closest-point search on a free-form surface)
    let ((mut u1, mut v1, _), (mut u2, mut v2, _)) = match *hint {
        Some(h) => (sa.closest_point_near(p0, (h[0], h[1])), sb.closest_point_near(p0, (h[2], h[3]))),
        None => (sa.closest_point(p0), sb.closest_point(p0)),
    };
    for _ in 0..30 {
        let (a, b) = (sa.eval(u1, v1), sb.eval(u2, v2));
        let r = a - b;
        if r.len() < 1e-10 * scale {
            let p = (a + b) * 0.5;
            *hint = Some([u1, v1, u2, v2]);
            return (p.dist(p0) <= cap).then_some(p);
        }
        let (a_u, a_v) = sa.partials(u1, v1);
        let (b_u, b_v) = sb.partials(u2, v2);
        let cols = [a_u, a_v, -b_u, -b_v];
        // (J J^T + lambda I) y = r, then step = -J^T y
        let mut m = [[0.0f64; 3]; 3];
        for c in &cols {
            let c = [c.x, c.y, c.z];
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] += c[i] * c[j];
                }
            }
        }
        let lam = 1e-12 * (m[0][0] + m[1][1] + m[2][2]).max(1e-300);
        for i in 0..3 {
            m[i][i] += lam;
        }
        let y = solve3(m, [r.x, r.y, r.z])?;
        let y = Vec3::new(y[0], y[1], y[2]);
        u1 -= cols[0].dot(y);
        v1 -= cols[1].dot(y);
        u2 -= cols[2].dot(y);
        v2 -= cols[3].dot(y);
        // a curve lying on the boundary of a free-form patch: stay on the patch
        if matches!(sa, Surface::BSpline(_)) {
            let d = sa.domain();
            u1 = clamp_to(u1, d.0 .0, d.0 .1);
            v1 = clamp_to(v1, d.1 .0, d.1 .1);
        }
        if matches!(sb, Surface::BSpline(_)) {
            let d = sb.domain();
            u2 = clamp_to(u2, d.0 .0, d.0 .1);
            v2 = clamp_to(v2, d.1 .0, d.1 .1);
        }
        if !(u1.is_finite() && v1.is_finite() && u2.is_finite() && v2.is_finite()) {
            return None;
        }
        if sa.eval(u1, v1).dist(p0) > 4.0 * cap {
            return None;
        }
    }
    None
}

/// Alternating normal projections (Gauss-Newton on the two signed distances):
/// robust for analytic faces, tangent faces fall back to the mean of the two feet.
fn snap_point_proj(sa: &Surface, sb: &Surface, p0: Vec3, cap: f64, feet: &mut [Option<(f64, f64)>; 2]) -> Option<Vec3> {
    let mut p = p0;
    let scale = 1.0 + p0.len();
    for _ in 0..16 {
        let (u1, v1, q1) = foot(sa, p, feet[0]);
        let (u2, v2, q2) = foot(sb, p, feet[1]);
        feet[0] = Some((u1, v1));
        feet[1] = Some((u2, v2));
        let (n1, n2) = (sa.normal(u1, v1), sb.normal(u2, v2));
        let (d1, d2) = ((p - q1).dot(n1), (p - q2).dot(n2));
        if d1.abs().max(d2.abs()) < 1e-10 * scale && (p - q1).len().max((p - q2).len()) < 1e-9 * scale {
            return Some(p);
        }
        let c = n1.dot(n2);
        let det = 1.0 - c * c;
        if det < 1e-6 {
            p = (q1 + q2) * 0.5;
        } else {
            let (a, b) = ((c * d2 - d1) / det, (c * d1 - d2) / det);
            p = p + n1 * a + n2 * b;
        }
        if p.dist(p0) > cap {
            return None;
        }
    }
    let res = p.dist(foot(sa, p, feet[0]).2).max(p.dist(foot(sb, p, feet[1]).2));
    (res < 1e-6 * scale).then_some(p)
}

/// Closest point on `s` to `p`: Newton from the previous station's foot when
/// there is one (a march along an edge), the global search otherwise.
fn foot(s: &Surface, p: Vec3, hint: Option<(f64, f64)>) -> (f64, f64, Vec3) {
    match hint {
        Some(h) => s.closest_point_near(p, h),
        None => s.closest_point(p),
    }
}

fn solve3(m: [[f64; 3]; 3], r: [f64; 3]) -> Option<[f64; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if !(det.abs() > 1e-300) {
        return None;
    }
    let cof = |i: usize, j: usize| {
        let (r0, r1) = ((i + 1) % 3, (i + 2) % 3);
        let (c0, c1) = ((j + 1) % 3, (j + 2) % 3);
        m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0]
    };
    let mut out = [0.0; 3];
    for j in 0..3 {
        // x_j = sum_i adj[j][i] r_i / det, adj[j][i] = cofactor(i, j)
        out[j] = (0..3).map(|i| cof(i, j) * r[i]).sum::<f64>() / det;
    }
    Some(out)
}

/// A circle revolved about an axis through (almost) its centre, lying in a plane
/// containing the axis, with its parameter origin on the equator, is a sphere
/// whose (u, v) equal the revolution's (angle, circle parameter). CAD exporters
/// that build a dome by revolving an arc emit this form (centre off the axis by
/// ~1e-4); the pole then needs the sphere treatment on import.
fn revolved_circle_as_sphere(c: &Curve, axis_o: Vec3, axis: Vec3) -> Option<Surface> {
    let Curve::Circle { frame, r } = c else { return None };
    let a = axis.normalize();
    let off = frame.o - axis_o;
    let along = off.dot(a);
    let perp = (off - a * along).len();
    if perp > 5e-4 * *r || frame.z.dot(a).abs() > 1e-4 || frame.x.dot(a).abs() > 1e-4 || frame.y.dot(a) < 1.0 - 1e-6 {
        return None;
    }
    Some(Surface::Sphere { frame: frame_from(axis_o + a * along, a, frame.x), r: *r })
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

    /// Stage timings of an import (`STEP_PROFILE=<file> cargo test --profile fast -p ck-io profile_import -- --ignored --nocapture`).
    #[test]
    #[ignore = "diagnostic: needs STEP_PROFILE env input, prints stage timings; run by hand"]
    fn profile_import() {
        let Ok(path) = std::env::var("STEP_PROFILE") else { return };
        let text = std::fs::read_to_string(path).unwrap();
        let t = std::time::Instant::now();
        let file = parse(&text).unwrap();
        eprintln!("parse        {:?}", t.elapsed());
        let t = std::time::Instant::now();
        let mut rd = Rd::new(&file);
        rd.recog = true;
        let (rec, conv) = rd.run_counted();
        eprintln!("read recog   {:?} (converted {conv})", t.elapsed());
        let t = std::time::Instant::now();
        let plain = Rd::new(&file).run();
        eprintln!("read plain   {:?}", t.elapsed());
        if let (Ok(r), Ok(p)) = (rec, plain) {
            let t = std::time::Instant::now();
            let n = ck_check::validate(&r.body, 1e-6).len();
            eprintln!("validate r   {:?} ({n})", t.elapsed());
            let t = std::time::Instant::now();
            let n = ck_check::validate(&p.body, 1e-6).len();
            eprintln!("validate p   {:?} ({n})", t.elapsed());
            let t = std::time::Instant::now();
            let d = ck_topo::mass::volume_difference_unshared(&r.body, &p.body);
            eprintln!("unshared dV  {:?} ({d:e})", t.elapsed());
            let t = std::time::Instant::now();
            let v = ck_topo::mass::mass_props(&r.body).volume;
            eprintln!("volume r     {:?} ({v})", t.elapsed());
            let t = std::time::Instant::now();
            let v = ck_topo::mass::mass_props(&p.body).volume;
            eprintln!("volume p     {:?} ({v})", t.elapsed());
        }
    }

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
        // nothing of the file can be read: the file is refused with the first face's reason
        let t = BOX.replace("=PLANE('',", "=DEGENERATE_TOROIDAL_SURFACE('',");
        let e = read_step(&t).unwrap_err();
        assert_eq!(e.code, "unsupported");
        assert!(e.entity.is_some());
    }

    #[test]
    fn a_face_that_cannot_be_read_is_left_out_and_named() {
        let t = BOX.replace("#81=PLANE('',#80);", "#81=DEGENERATE_TOROIDAL_SURFACE('',#80,1.,1.);");
        let r = read_step(&t).unwrap();
        assert_eq!(r.parts.len(), 1);
        let p = &r.parts[0];
        assert_eq!((p.entity, p.name.as_str(), p.faces_total, p.faces.len()), (201, "box", 6, 5));
        assert_eq!(p.refused.len(), 1);
        assert_eq!((p.refused[0].code, p.refused[0].entity), ("unsupported", Some(106)));
        assert!(p.refused[0].message.contains("#81"), "{}", p.refused[0].message);
        assert!(p.error.is_none());
        assert_eq!(r.body.num_faces(), 5);
    }

    #[test]
    fn a_complete_solid_has_no_refusals() {
        let r = read_step(BOX).unwrap();
        assert_eq!(r.parts.len(), 1);
        assert_eq!((r.parts[0].faces_total, r.parts[0].faces.len(), r.parts[0].refused.len()), (6, 6, 0));
    }

    #[test]
    fn a_refused_face_keeps_its_surface_for_repair() {
        // the loop of face #106 references a missing edge: the face is left out but its plane is kept
        let t = BOX.replace("#106=ADVANCED_FACE('',(#105),#81,.F.);", "#106=ADVANCED_FACE('',(#999),#81,.F.);");
        let r = read_step(&t).unwrap();
        let p = &r.parts[0];
        assert_eq!(p.refused.len(), 1);
        assert_eq!(p.refused_surfaces.len(), 1);
        assert_eq!(p.refused_surfaces[0].0, 106);
        assert!(matches!(p.refused_surfaces[0].1, Surface::Plane { .. }));
    }

    #[test]
    fn a_file_without_a_solid_is_refused_with_what_it_contains() {
        // a header and some geometry, no solid, no surface model
        let t = "ISO-10303-21;\nHEADER;FILE_DESCRIPTION((''),'2;1');FILE_NAME('e','',(''),(''),'','','');FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));ENDSEC;\nDATA;\n#1=CARTESIAN_POINT('',(0.,0.,0.));#2=CARTESIAN_POINT('',(1.,0.,0.));\nENDSEC;\nEND-ISO-10303-21;";
        let e = read_step(t).err().expect("refused");
        assert_eq!(e.code, "noSolid");
        assert!(e.message.contains("2 CARTESIAN_POINT") && e.message.contains("Hint:"), "{}", e.message);
        // an empty DATA section
        let e = read_step("ISO-10303-21;\nHEADER;ENDSEC;\nDATA;\nENDSEC;\nEND-ISO-10303-21;").err().expect("refused");
        assert!(e.message.contains("no solid, shell, face, curve or point entities"), "{}", e.message);
        // faces with no solid over them: said so
        let e = read_step(&BOX.replace("#201=MANIFOLD_SOLID_BREP('box',#200);", "")).err().expect("refused");
        assert_eq!(e.code, "noSolid");
        assert!(e.message.contains("6 ADVANCED_FACE") && e.message.contains("refers to them"), "{}", e.message);
    }

    #[test]
    fn unshared_volume_difference_counts_only_faces_that_differ() {
        let a = read_step(BOX).unwrap();
        let b = read_step_plain(BOX).unwrap();
        assert_eq!(ck_topo::mass::volume_difference_unshared(&a.body, &b.body), 0.0);
        // a taller box: the top and the four walls differ, the bottom is shared; the difference is the volume's
        let tall = BOX.replace("(0.,0.,5.)", "(0.,0.,8.)").replace("(10.,0.,5.)", "(10.,0.,8.)").replace("(10.,10.,5.)", "(10.,10.,8.)").replace("(0.,10.,5.)", "(0.,10.,8.)");
        let c = read_step(&tall).unwrap();
        let d = ck_topo::mass::volume_difference_unshared(&c.body, &a.body);
        assert!((d - 300.0).abs() < 1e-6, "{d}");
        // the recognised-versus-plain comparison still accepts equal reads and refuses a different volume
        assert!(recognised_is_no_worse(&a, &b));
        assert!(!recognised_is_no_worse(&c, &a));
    }

    #[test]
    fn extreme_winding_does_not_overflow_abs() {
        let f = StepFile::default();
        let mut rd = Rd::new(&f);
        let p0 = Vec3::new(1.0, 0.0, 0.0);
        let p1 = Vec3::new(0.0, 1.0, 0.0);
        let v0 = rd.b.add_vertex(p0);
        let v1 = rd.b.add_vertex(p1);
        let e = rd.b.add_edge(Curve::line(p0, Vec3::new(-1.0, 1.0, 0.0).normalize()), 0.0, 1.0, v0, v1);
        // Huge negative u-displacement produces k = -1e25 -> i64::MIN in `k as i64`, which panicked on `w.2.abs()`
        let c = Co { edge: e, rev: false, pc: Some(Curve2::Line { o: Vec2::new(0.0, 0.0), d: Vec2::new(-1.0e25 * TAU, 0.0) }) };
        let surf = Surface::Cylinder { frame: Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r: 1.0 };
        assert!(rd.winding(std::slice::from_ref(&c), 0, Some(TAU)).is_none());
        let _ = rd.complete_periodic(1, &surf, true, vec![vec![c]], Some(TAU), None, &mut []);
    }

    #[test]
    fn missing_pcurves_in_loop_helpers_do_not_panic() {
        let f = StepFile::default();
        let mut rd = Rd::new(&f);
        let v0 = rd.b.add_vertex(Vec3::ZERO);
        let v1 = rd.b.add_vertex(Vec3::X);
        let e = rd.b.add_edge(Curve::line(Vec3::ZERO, Vec3::X), 0.0, 1.0, v0, v1);
        let mut l = vec![Co { edge: e, rev: false, pc: None }; 3];
        rd.unwrap_loop(&mut l, Some(TAU), None);
        let bs = BSplineSurface::bilinear(Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::new(1.0, 1.0, 0.0));
        rd.fix_closed_spline_loop(&mut l, &Surface::BSpline(Box::new(bs)), true);
    }
}
