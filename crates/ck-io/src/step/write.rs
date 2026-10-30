//! STEP AP214 (ISO 10303-21) writer — `step` request (ck-io/src/step/write.rs).
//!
//! The output format is pinned by recorded reference output (not published):
//! - header / product / context boilerplate and every entity format string
//!   (`FILE_NAME(` .. `'CadKit','CadKit','');`, `(BOUNDED_CURVE()B_SPLINE_CURVE(`, ...).
//! - FILE_NAME time stamp = host `ck_unix_ms()` rendered as UTC
//!   `YYYY-MM-DDTHH:MM:SS`; ms -> secs via `(ms/1000) as u64` (NaN and
//!   negatives saturate to 1970-01-01T00:00:00; 1e20 ms gives year
//!   3168875820).
//! - strings: `'` -> `''`, `\` -> `\\`, ASCII control chars dropped,
//!   every non-ASCII char -> `\X2\<UTF-16 units, %04X>\X0\` (one block per
//!   char, e.g. 😀 -> `\X2\D83DDE00\X0\`).
//! - reals: `0.` for zero, otherwise shortest round-trip; scientific
//!   (`1.E-7`, `6.123233995736766E-17`, `1.E15`) when |x| < 1e-6 or
//!   |x| >= 1e15, else plain decimal (`0.000001`, `999999999999999.`).
//!   A `.` is always present in the mantissa.
//! - numbering: #1..#14 context, #15..#18 world placement, then per lump /
//!   shell / face: surface, then per loop: per coedge the edge on first use
//!   (start vertex, end vertex, curve, EDGE_CURVE) followed by its
//!   ORIENTED_EDGE, then EDGE_LOOP + bound; ADVANCED_FACE; CLOSED_SHELL
//!   after each shell; all MANIFOLD_SOLID_BREPs after all shells.
//! - DIRECTIONs are shared through a cache keyed by the exact f64 bit
//!   patterns (so (-0,1,0) and (0,1,0) are distinct entities — that is the
//!   only rule that reproduces the oracle's reuse pattern; both print
//!   `0.`). CARTESIAN_POINTs and VECTORs are never shared.
//! - seams (both coedges on one face) are never written; a loop is cut into
//!   one EDGE_LOOP per run of non-seam coedges. Faces left with no boundary
//!   (full sphere / full torus) are split into two halves on one shared
//!   surface entity (equator resp. meridians u=0 and u=π).
//! - only planar faces get a FACE_OUTER_BOUND (the loop with the largest
//!   positive area about the outward normal); every other bound is
//!   FACE_BOUND.
//! - void shells: CLOSED_SHELL written outward (face sense and bound flags
//!   inverted), wrapped in ORIENTED_CLOSED_SHELL('',*,#,.F.) and the lump
//!   becomes BREP_WITH_VOIDS.

use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, CoedgeId, EdgeId, FaceId, ShellId, VertexId};
use std::collections::HashMap;
use std::fmt::Write as _;

/// Options of one export.
#[derive(Debug, Clone, Copy, Default)]
pub struct StepOpts {
    /// Host clock (`ck_unix_ms` import) used for the FILE_NAME time stamp.
    pub unix_ms: f64,
}

/// Format one STEP REAL.
pub fn real(x: f64) -> String {
    // GUESS: -0.0 prints `0.` (the oracle never printed `-0.` although
    // its geometry certainly carries negative zeros).
    if x == 0.0 {
        return "0.".to_string();
    }
    let a = x.abs();
    if a < 1e-6 || a >= 1e15 {
        let s = format!("{:E}", x);
        match s.find('E') {
            Some(i) if !s[..i].contains('.') => format!("{}.{}", &s[..i], &s[i..]),
            _ => s,
        }
    } else {
        let s = format!("{}", x);
        if s.contains('.') || s.contains("inf") || s.contains("NaN") {
            s
        } else {
            s + "."
        }
    }
}

/// Encode text as the inside of a STEP string literal (no quotes).
pub fn string(s: &str) -> String {
    let mut o = String::new();
    for c in s.chars() {
        match c {
            '\'' => o.push_str("''"),
            '\\' => o.push_str("\\\\"),
            c if c.is_ascii_control() => {}
            c if (c as u32) < 0x80 => o.push(c),
            c => {
                o.push_str("\\X2\\");
                let mut buf = [0u16; 2];
                for u in c.encode_utf16(&mut buf) {
                    let _ = write!(o, "{:04X}", u);
                }
                o.push_str("\\X0\\");
            }
        }
    }
    o
}

/// `YYYY-MM-DDTHH:MM:SS` (UTC) of a unix time in milliseconds.
pub fn timestamp(unix_ms: f64) -> String {
    let secs = (unix_ms / 1000.0) as u64;
    let days = (secs / 86400) as i64;
    let rem = secs % 86400;
    // Howard Hinnant's civil_from_days.
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z - era * 146097;
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = yoe + era * 400 + if m <= 2 { 1 } else { 0 };
    format!("{:04}-{:02}-{:02}T{:02}:{:02}:{:02}", y, m, d, rem / 3600, rem / 60 % 60, rem % 60)
}

fn bits(v: Vec3) -> [u64; 3] {
    [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()]
}

fn triple(v: Vec3) -> String {
    format!("({},{},{})", real(v.x), real(v.y), real(v.z))
}

fn flag(b: bool) -> &'static str {
    if b {
        ".T."
    } else {
        ".F."
    }
}

fn refs(ids: &[usize]) -> String {
    let v: Vec<String> = ids.iter().map(|i| format!("#{i}")).collect();
    v.join(",")
}

fn reals(xs: &[f64]) -> String {
    let v: Vec<String> = xs.iter().map(|x| real(*x)).collect();
    v.join(",")
}

/// An unclamped spline as its domain only: clamped ends, parameter counted
/// from the start of the domain.
fn unclamped_to_clamped(bs: &ck_geom::nurbs::bspline::BSplineCurve) -> ck_geom::nurbs::bspline::BSplineCurve {
    let a = bs.domain().0;
    let mut c = bs.clamp_ends();
    for k in c.knots.iter_mut() {
        *k -= a;
    }
    c
}

/// Distinct knot values + multiplicities (exact comparison).
fn knot_mults(knots: &[f64]) -> (Vec<usize>, Vec<f64>) {
    let mut m: Vec<usize> = Vec::new();
    let mut k: Vec<f64> = Vec::new();
    for &t in knots {
        if k.last() == Some(&t) {
            *m.last_mut().unwrap() += 1;
        } else {
            k.push(t);
            m.push(1);
        }
    }
    (m, k)
}

fn ints(xs: &[usize]) -> String {
    let v: Vec<String> = xs.iter().map(|x| x.to_string()).collect();
    v.join(",")
}

/// Entity emitter with the direction cache and topology maps.
pub struct StepWriter {
    out: String,
    n: usize,
    dirs: HashMap<[u64; 3], usize>,
    verts: HashMap<VertexId, usize>,
    edges: HashMap<EdgeId, usize>,
    /// print -0 DIRECTION components as `-0.` (test support)
    signed_zero: bool,
    /// member edge -> (head, tail) of its seam-split chain (len > 1 only)
    chain: HashMap<EdgeId, (EdgeId, EdgeId)>,
}

impl Default for StepWriter {
    fn default() -> Self {
        Self::new()
    }
}

impl StepWriter {
    pub fn new() -> StepWriter {
        StepWriter {
            out: String::new(),
            n: 0,
            dirs: HashMap::new(),
            verts: HashMap::new(),
            edges: HashMap::new(),
            signed_zero: false,
            chain: HashMap::new(),
        }
    }

    /// Writer whose next entity is `#next` (entity-level tests).
    pub fn starting_at(next: usize) -> StepWriter {
        let mut w = StepWriter::new();
        w.n = next - 1;
        w
    }

    /// Register an already-written DIRECTION (entity-level tests).
    pub fn seed_dir(&mut self, d: Vec3, id: usize) {
        self.dirs.insert(bits(d), id);
    }

    /// The DATA section text written so far.
    pub fn text(&self) -> &str {
        &self.out
    }

    pub fn ent(&mut self, s: &str) -> usize {
        self.n += 1;
        let _ = writeln!(self.out, "#{}={};", self.n, s);
        self.n
    }

    pub fn point(&mut self, p: Vec3) -> usize {
        self.ent(&format!("CARTESIAN_POINT('',{})", triple(p)))
    }

    /// DIRECTION, normalised first exactly like the oracle's writer
    /// (f414: `len = sqrt(x²+y²+z²)`; `len > 1e-300` and finite →
    /// components × (1/len), otherwise (0,0,0)), then shared by bits.
    pub fn dir(&mut self, d: Vec3) -> usize {
        let len = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
        let d = if len > 1e-300 && len.is_finite() {
            let k = 1.0 / len;
            Vec3::new(d.x * k, d.y * k, d.z * k)
        } else {
            Vec3::ZERO
        };
        if let Some(&i) = self.dirs.get(&bits(d)) {
            return i;
        }
        let t = if self.signed_zero {
            let r = |x: f64| if x == 0.0 && x.is_sign_negative() { "-0.".to_string() } else { real(x) };
            format!("({},{},{})", r(d.x), r(d.y), r(d.z))
        } else {
            triple(d)
        };
        let i = self.ent(&format!("DIRECTION('',{t})"));
        self.dirs.insert(bits(d), i);
        i
    }

    pub fn vector(&mut self, d: Vec3, mag: f64) -> usize {
        let di = self.dir(d);
        self.ent(&format!("VECTOR('',#{di},{})", real(mag)))
    }

    /// AXIS2_PLACEMENT_3D(location, axis, ref_direction).
    /// AXIS2_PLACEMENT_3D as the oracle's f416 builds it: axis = unit z,
    /// ref = unit (x − z(x·z)) (fallback: unit z × e, e = the world axis of
    /// z's smallest |component|, f3907); both then go through `dir`
    /// (normalised once more). Summation orders are the binary's.
    pub fn axis2(&mut self, o: Vec3, z: Vec3, x: Vec3) -> usize {
        let unit = |v: Vec3, l2: f64| {
            let l = l2.sqrt();
            if l > 1e-300 && l.is_finite() {
                let k = 1.0 / l;
                Some(Vec3::new(v.x * k, v.y * k, v.z * k))
            } else {
                None
            }
        };
        let zn = unit(z, z.x * z.x + z.y * z.y + z.z * z.z).unwrap_or(Vec3::ZERO);
        let d = x.x * zn.x + x.y * zn.y + zn.z * x.z;
        let xp = Vec3::new(x.x - zn.x * d, x.y - zn.y * d, x.z - zn.z * d);
        let xr = unit(xp, xp.z * xp.z + (xp.x * xp.x + xp.y * xp.y)).unwrap_or_else(|| {
            let (ax, ay, az) = (zn.x.abs(), zn.y.abs(), zn.z.abs());
            let ex = ax <= ay && ax <= az;
            let ey = !ex && ay <= az;
            let e = Vec3::new(ex as u8 as f64, ey as u8 as f64, (!ex && !ey) as u8 as f64);
            let c = Vec3::new(zn.y * e.z - zn.z * e.y, zn.z * e.x - zn.x * e.z, zn.x * e.y - zn.y * e.x);
            unit(c, c.z * c.z + (c.y * c.y + c.x * c.x)).unwrap_or(Vec3::ZERO)
        });
        let p = self.point(o);
        let zi = self.dir(zn);
        let xi = self.dir(xr);
        self.ent(&format!("AXIS2_PLACEMENT_3D('',#{p},#{zi},#{xi})"))
    }

    fn frame(&mut self, f: &Frame) -> usize {
        self.axis2(f.o, f.z, f.x)
    }

    /// Placement of a PLANE / CIRCLE: the oracle stores those as
    /// (o, x, y) and writes axis = x × y (f417 / f418), whatever the
    /// frame's own z holds (decides the sign of zero components).
    fn frame_xy(&mut self, f: &Frame) -> usize {
        self.axis2(f.o, f.x.cross(f.y), f.x)
    }

    pub fn curve(&mut self, c: &Curve) -> usize {
        match c {
            Curve::Line { o, d } => {
                let p = self.point(*o);
                // f417: the LINE vector is `VECTOR('',#dir,1.)` verbatim.
                let di = self.dir(*d);
                let v = self.ent(&format!("VECTOR('',#{di},1.)"));
                self.ent(&format!("LINE('',#{p},#{v})"))
            }
            Curve::Circle { frame, r } => {
                let a = self.frame_xy(frame);
                self.ent(&format!("CIRCLE('',#{a},{})", real(*r)))
            }
            Curve::Ellipse { frame, a, b } => {
                let ax = self.frame(frame);
                self.ent(&format!("ELLIPSE('',#{ax},{},{})", real(*a), real(*b)))
            }
            Curve::BSpline(bs) => {
                let norm;
                let bs = if bs.knots[..=bs.degree].iter().any(|&k| k != bs.knots[0]) {
                    // unclamped (periodic-style) knots: the domain as a clamped curve
                    norm = unclamped_to_clamped(bs);
                    &norm
                } else {
                    bs
                };
                let pts: Vec<usize> = bs.ctrl.iter().map(|p| self.point(*p)).collect();
                // GUESS: closed_curve = first and last control point equal
                // (the oracle writes .T. for the periodic extrusion profile
                // of a closed spline and .F. for its open halves).
                let closed = bs.ctrl.len() > 1
                    && bs.ctrl.first().zip(bs.ctrl.last()).map_or(false, |(a, b)| a.dist(*b) <= 1e-9 * (1.0 + a.len()));
                let (m, k) = knot_mults(&bs.knots);
                // all-ones weights are a non-rational curve
                let weights = bs.weights.as_ref().filter(|w| w.iter().any(|&x| x != 1.0));
                match weights {
                    None => self.ent(&format!(
                        "B_SPLINE_CURVE_WITH_KNOTS('',{},({}),.UNSPECIFIED.,{},.F.,({}),({}),.UNSPECIFIED.)",
                        bs.degree,
                        refs(&pts),
                        flag(closed),
                        ints(&m),
                        reals(&k)
                    )),
                    Some(w) => self.ent(&format!(
                        "(BOUNDED_CURVE()B_SPLINE_CURVE({},({}),.UNSPECIFIED.,{},.F.)B_SPLINE_CURVE_WITH_KNOTS(({}),({}),.UNSPECIFIED.)CURVE()GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_CURVE(({}))REPRESENTATION_ITEM(''))",
                        bs.degree,
                        refs(&pts),
                        flag(closed),
                        ints(&m),
                        reals(&k),
                        reals(w)
                    )),
                }
            }
            // GUESS: no oracle evidence of STEP conic entities. Conics are
            // written as the analytic ELLIPSE / PARABOLA / HYPERBOLA (the
            // inverse of the importer's focus-form mapping) so they survive a
            // round trip exactly; sinusoids as B-spline approximations.
            Curve::Conic { frame, p, e } if *e >= 0.0 && *p > 0.0 && e.is_finite() => {
                if *e == 0.0 {
                    let a = self.frame(frame);
                    return self.ent(&format!("CIRCLE('',#{a},{})", real(*p)));
                } else if *e < 1.0 {
                    let a = p / (1.0 - e * e);
                    let b = p / (1.0 - e * e).sqrt();
                    let ax = self.axis2(frame.o - frame.x * (a * e), frame.z, frame.x);
                    self.ent(&format!("ELLIPSE('',#{ax},{},{})", real(a), real(b)))
                } else if *e == 1.0 {
                    let f = 0.5 * p;
                    let ax = self.axis2(frame.o + frame.x * f, frame.z, -frame.x);
                    self.ent(&format!("PARABOLA('',#{ax},{})", real(f)))
                } else {
                    let a = p / (e * e - 1.0);
                    let b = (a * p).sqrt();
                    let ax = self.axis2(frame.o + frame.x * (a * e), frame.z, -frame.x);
                    self.ent(&format!("HYPERBOLA('',#{ax},{},{})", real(a), real(b)))
                }
            }
            Curve::Conic { .. } | Curve::Sinusoid { .. } => {
                let bs = ck_geom::convert::curve_to_nurbs(c, c.range().0, c.range().1);
                self.curve(&Curve::BSpline(bs))
            }
        }
    }

    pub fn surface(&mut self, s: &Surface) -> usize {
        match s {
            Surface::Plane { frame } => {
                let a = self.frame_xy(frame);
                self.ent(&format!("PLANE('',#{a})"))
            }
            Surface::Cylinder { frame, r } => {
                let a = self.frame(frame);
                self.ent(&format!("CYLINDRICAL_SURFACE('',#{a},{})", real(*r)))
            }
            Surface::Cone { frame, r, semi_angle } if *semi_angle < 0.0 => {
                // STEP needs semi_angle > 0: same point set and normal with
                // the axis reversed (oracle writes axis (0,-1,0), +π/4 where
                // ck-topo holds (0,1,0), -π/4). GUESS: ref direction kept.
                let a = self.axis2(frame.o, -frame.z, frame.x);
                self.ent(&format!("CONICAL_SURFACE('',#{a},{},{})", real(*r), real(-*semi_angle)))
            }
            Surface::Cone { frame, r, semi_angle } => {
                let a = self.frame(frame);
                self.ent(&format!("CONICAL_SURFACE('',#{a},{},{})", real(*r), real(*semi_angle)))
            }
            Surface::Sphere { frame, r } => {
                let a = self.frame(frame);
                self.ent(&format!("SPHERICAL_SURFACE('',#{a},{})", real(*r)))
            }
            Surface::Torus { frame, major, minor } => {
                let a = self.frame(frame);
                self.ent(&format!("TOROIDAL_SURFACE('',#{a},{},{})", real(*major), real(*minor)))
            }
            Surface::Revolution { profile, axis_o, axis } => {
                let c = self.curve(profile);
                let p = self.point(*axis_o);
                let d = self.dir(*axis);
                let a = self.ent(&format!("AXIS1_PLACEMENT('',#{p},#{d})"));
                self.ent(&format!("SURFACE_OF_REVOLUTION('',#{c},#{a})"))
            }
            Surface::Extrusion { profile, dir } => {
                let c = self.curve(profile);
                // Oracle: VECTOR('',#unit,1.) even for a 5 mm extrusion.
                // GUESS: direction normalised, magnitude always 1.
                let v = self.vector(dir.normalize(), 1.0);
                self.ent(&format!("SURFACE_OF_LINEAR_EXTRUSION('',#{c},#{v})"))
            }
            Surface::BSpline(bs) => {
                let mut rows = Vec::new();
                for i in 0..bs.nu {
                    let r: Vec<usize> = (0..bs.nv).map(|j| self.point(bs.ctrl[i * bs.nv + j])).collect();
                    rows.push(format!("({})", refs(&r)));
                }
                let (mu, ku) = knot_mults(&bs.knots_u);
                let (mv, mut kv) = knot_mults(&bs.knots_v);
                // a ruled direction (one linear span) is written on [0, 1] (the
                // oracle's lofts); the length-parametrised domain stays internal
                if bs.deg_v == 1 && kv.len() == 2 && kv[0] == 0.0 && kv[1] > 0.0 {
                    kv[1] = 1.0;
                }
                match &bs.weights {
                    None => self.ent(&format!(
                        "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
                        bs.deg_u,
                        bs.deg_v,
                        rows.join(","),
                        ints(&mu),
                        ints(&mv),
                        reals(&ku),
                        reals(&kv)
                    )),
                    Some(w) => {
                        let wr: Vec<String> =
                            (0..bs.nu).map(|i| format!("({})", reals(&w[i * bs.nv..(i + 1) * bs.nv]))).collect();
                        self.ent(&format!(
                            "(BOUNDED_SURFACE()B_SPLINE_SURFACE({},{},({}),.UNSPECIFIED.,.F.,.F.,.F.)B_SPLINE_SURFACE_WITH_KNOTS(({}),({}),({}),({}),.UNSPECIFIED.)GEOMETRIC_REPRESENTATION_ITEM()RATIONAL_B_SPLINE_SURFACE(({}))REPRESENTATION_ITEM('')SURFACE())",
                            bs.deg_u,
                            bs.deg_v,
                            rows.join(","),
                            ints(&mu),
                            ints(&mv),
                            reals(&ku),
                            reals(&kv),
                            wr.join(",")
                        ))
                    }
                }
            }
            // no STEP entity for a helicoidal patch: write the interpolated B-spline surface
            Surface::Screw { urange, vrange, .. } => match ck_geom::convert::surface_to_nurbs(s, *urange, *vrange) {
                Ok(bs) => self.surface(&Surface::BSpline(Box::new(bs))),
                Err(_) => self.surface(&Surface::plane(Vec3::ZERO, Vec3::Z)),
            },
            Surface::Offset { base, dist } => {
                let b = self.surface(base);
                // GUESS: self_intersect flag .F. (string `OFFSET_SURFACE('',#`
                // only; never seen in an oracle reply).
                self.ent(&format!("OFFSET_SURFACE('',#{b},{},.F.)", real(*dist)))
            }
        }
    }

    pub fn vertex_point(&mut self, p: Vec3) -> usize {
        let pi = self.point(p);
        self.ent(&format!("VERTEX_POINT('',#{pi})"))
    }

    fn vertex(&mut self, body: &Body, v: VertexId) -> usize {
        if let Some(&i) = self.verts.get(&v) {
            return i;
        }
        let i = self.vertex_point(body.vertex(v).p);
        self.verts.insert(v, i);
        i
    }

    fn edge(&mut self, body: &Body, e: EdgeId) -> usize {
        // a seam-split arc (ck-topo `wire_chains`) is ONE oracle edge: the
        // head's curve from the head's start to the tail's end vertex
        let (e, tail) = self.chain.get(&e).copied().unwrap_or((e, e));
        if let Some(&i) = self.edges.get(&e) {
            return i;
        }
        let ed = body.edge(e);
        let a = self.vertex(body, ed.v0);
        let b = self.vertex(body, body.edge(tail).v1);
        // Oracle LINE location = the edge start point (draft rails).
        // GUESS: lines are re-anchored at t0, other curves written as is.
        let c = match &ed.curve {
            Curve::Line { o, d } if ed.t0 != 0.0 => self.curve(&Curve::Line { o: *o + *d * ed.t0, d: *d }),
            c => self.curve(c),
        };
        let i = self.ent(&format!("EDGE_CURVE('',#{a},#{b},#{c},.T.)"));
        self.edges.insert(e, i);
        i
    }

    fn oriented(&mut self, e: usize, fwd: bool) -> usize {
        self.ent(&format!("ORIENTED_EDGE('',*,*,#{e},{})", flag(fwd)))
    }

    fn bound(&mut self, oes: &[usize], outer: bool, ori: bool) -> usize {
        let l = self.ent(&format!("EDGE_LOOP('',({}))", refs(oes)));
        let kind = if outer { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
        self.ent(&format!("{kind}('',#{l},{})", flag(ori)))
    }

    fn adv_face(&mut self, bounds: &[usize], surf: usize, sense: bool) -> usize {
        self.ent(&format!("ADVANCED_FACE('',({}),#{surf},{})", refs(bounds), flag(sense)))
    }

    /// One B-rep face (possibly two when a closed sphere/torus is split).
    fn face(&mut self, body: &Body, f: FaceId, void: bool, faces_out: &mut Vec<usize>) {
        let face = body.face(f);
        let pieces = face_pieces(body, f);
        let bflag = !void;
        let fsense = face.sense != void;
        if pieces.is_empty() {
            if let Some(()) = self.split_face(body, f, void, faces_out) {
                return;
            }
        }
        let surf = self.surface(&face.surface);
        let outer = if matches!(face.surface, Surface::Plane { .. }) { outer_piece(body, f, &pieces) } else { None };
        let mut bounds = Vec::new();
        for (k, piece) in pieces.iter().enumerate() {
            // collapse runs of coedges on one chain (also across the loop
            // start) into one ORIENTED_EDGE
            let head = |c: CoedgeId| {
                let e = body.coedge(c).edge;
                self.chain.get(&e).map_or(e, |h| h.0)
            };
            let mut cs: Vec<CoedgeId> = Vec::new();
            for &c in piece {
                if cs.last().map_or(true, |&p| head(p) != head(c)) {
                    cs.push(c);
                }
            }
            if cs.len() > 1 && head(cs[0]) == head(*cs.last().unwrap()) {
                cs.pop();
            }
            let mut oes = Vec::new();
            for c in cs {
                let ce = body.coedge(c);
                let e = self.edge(body, ce.edge);
                oes.push(self.oriented(e, !ce.rev));
            }
            bounds.push(self.bound(&oes, outer == Some(k), bflag));
        }
        faces_out.push(self.adv_face(&bounds, surf, fsense));
    }

    /// Closed sphere / torus without boundary: two halves on one surface.
    fn split_face(&mut self, body: &Body, f: FaceId, void: bool, faces_out: &mut Vec<usize>) -> Option<()> {
        let face = body.face(f);
        let s = face.sense;
        let bflag = !void;
        // GUESS: split halves always written with same_sense = sense
        // (inverted for void shells, as every void face).
        let fsense = s != void;
        match &face.surface {
            Surface::Sphere { frame, r } => {
                let surf = self.surface(&face.surface);
                let v = self.vertex_point(frame.o + frame.x * *r);
                let c = self.curve(&Curve::Circle {
                    frame: Frame { o: frame.o, x: frame.x, y: frame.y, z: frame.x.cross(frame.y) },
                    r: *r,
                });
                let e = self.ent(&format!("EDGE_CURVE('',#{v},#{v},#{c},.T.)"));
                for half in [true, false] {
                    let oe = self.oriented(e, half == s);
                    let b = self.bound(&[oe], false, bflag);
                    faces_out.push(self.adv_face(&[b], surf, fsense));
                }
                Some(())
            }
            Surface::Torus { frame, major, minor } => {
                let surf = self.surface(&face.surface);
                let n0 = frame.x.cross(frame.z);
                let mut es = Vec::new();
                for (x, n) in [(frame.x, n0), (-frame.x, -n0)] {
                    let c = frame.o + x * *major;
                    let v = self.vertex_point(frame.o + x * (*major + *minor));
                    let cu = self.curve(&Curve::Circle { frame: Frame { o: c, x, y: n.cross(x), z: n }, r: *minor });
                    es.push(self.ent(&format!("EDGE_CURVE('',#{v},#{v},#{cu},.T.)")));
                }
                for half in [false, true] {
                    let a = self.oriented(es[0], half == s);
                    let b0 = self.bound(&[a], false, bflag);
                    let b = self.oriented(es[1], half != s);
                    let b1 = self.bound(&[b], false, bflag);
                    faces_out.push(self.adv_face(&[b0, b1], surf, fsense));
                }
                Some(())
            }
            _ => None,
        }
    }

    fn shell(&mut self, body: &Body, s: ShellId, void: bool) -> (usize, bool) {
        let mut faces = Vec::new();
        for &f in &body.shell(s).faces {
            if body.face(f).alive {
                self.face(body, f, void, &mut faces);
            }
        }
        let closed = shell_closed(body, s);
        let kind = if closed { "CLOSED_SHELL" } else { "OPEN_SHELL" };
        (self.ent(&format!("{kind}('',({}))", refs(&faces))), closed)
    }
}

/// A chain (several wire edges written as one oracle edge) may only be
/// written as one EDGE_CURVE if every face walks it as one run. A planar face
/// with a slit (or any vertex where a face's loop turns onto another edge)
/// uses part of the chain on its own: e.g. the two halves of a hole circle
/// cut by a slit line, which `wire_chains` fuses into one closed circle
/// through the slit's endpoints. Split such chains between the members that
/// some face does not walk back to back; every piece is still one edge for
/// all the faces that use it.
fn split_chains(body: &Body, chains: Vec<Vec<EdgeId>>) -> Vec<Vec<EdgeId>> {
    if chains.iter().all(|c| c.len() < 2) {
        return chains;
    }
    // member -> position in its chain
    let mut pos: HashMap<EdgeId, (usize, usize)> = HashMap::new();
    for (ci, ch) in chains.iter().enumerate() {
        if ch.len() > 1 {
            for (k, &e) in ch.iter().enumerate() {
                pos.insert(e, (ci, k));
            }
        }
    }
    // joined[ci][k]: members k and k+1 are walked back to back by every piece using either
    let mut joined: Vec<Vec<bool>> = chains.iter().map(|c| vec![true; c.len().saturating_sub(1)]).collect();
    for f in body.face_ids() {
        for piece in face_pieces(body, f) {
            let n = piece.len();
            let cyc = n > 0 && body.coedge_start(piece[0]) == body.coedge_end(piece[n - 1]);
            let edge_at = |i: usize| body.coedge(piece[i]).edge;
            // neighbours of position i inside the piece
            let adj = |i: usize| -> [Option<usize>; 2] {
                let prev = if i > 0 { Some(i - 1) } else if cyc && n > 1 { Some(n - 1) } else { None };
                let next = if i + 1 < n { Some(i + 1) } else if cyc && n > 1 { Some(0) } else { None };
                [prev, next]
            };
            for i in 0..n {
                let Some(&(ci, k)) = pos.get(&edge_at(i)) else { continue };
                let ch = &chains[ci];
                // the link to the next member (k, k+1) and to the previous one (k-1, k)
                for (link, other) in [(k, ch.get(k + 1)), (k.wrapping_sub(1), k.checked_sub(1).map(|j| &ch[j]))] {
                    let Some(&other) = other else { continue };
                    let present = adj(i).iter().flatten().any(|&j| edge_at(j) == other);
                    if !present {
                        joined[ci][link] = false;
                    }
                }
                // a closed chain (a whole circle) is one EDGE_CURVE from and
                // back to its closing vertex: a run that enters it anywhere
                // else (a slit line ending on the circle) cannot use it
                if body.edge(*ch.last().unwrap()).v1 == body.edge(ch[0]).v0 {
                    let [prev, _] = adj(i);
                    let starts_run = prev.map_or(true, |j| pos.get(&edge_at(j)).map(|p| p.0) != Some(ci));
                    let x = body.coedge_start(piece[i]);
                    if starts_run && x != body.edge(ch[0]).v0 {
                        if let Some(j) = ch.iter().position(|&m| body.edge(m).v1 == x) {
                            if j + 1 < ch.len() {
                                joined[ci][j] = false;
                            }
                        }
                    }
                }
            }
        }
    }
    let mut out = vec![];
    for (ci, ch) in chains.into_iter().enumerate() {
        let mut cur: Vec<EdgeId> = vec![];
        for (k, e) in ch.iter().enumerate() {
            cur.push(*e);
            if k + 1 < ch.len() && !joined[ci][k] {
                out.push(std::mem::take(&mut cur));
            }
        }
        if !cur.is_empty() {
            out.push(cur);
        }
    }
    out
}

/// Runs of non-seam coedges per loop (seam = edge with both coedges on
/// this face). A planar face has no seam, but the oracle-style revolve cuts a
/// disc / annulus with a slit edge that is dropped like one; an edge used
/// twice in a planar loop is only dropped when the runs it leaves are closed
/// (a hole bridged to the outer boundary would otherwise be cut into open
/// chains no reader can close again, so those loops stay whole).
fn face_pieces(body: &Body, f: FaceId) -> Vec<Vec<CoedgeId>> {
    let out = pieces_of(body, f, true);
    let planar = matches!(body.face(f).surface, Surface::Plane { .. });
    let closed = |run: &Vec<CoedgeId>| body.coedge_start(run[0]) == body.coedge_end(*run.last().unwrap());
    if planar && !out.iter().all(closed) {
        return pieces_of(body, f, false);
    }
    out
}

fn pieces_of(body: &Body, f: FaceId, drop_seams: bool) -> Vec<Vec<CoedgeId>> {
    let seam = |c: CoedgeId| {
        if !drop_seams {
            return false;
        }
        let e = body.edge(body.coedge(c).edge);
        let n = e.coedges.iter().filter(|&&k| body.coedge(k).alive && body.lp(body.coedge(k).lp).face == f).count();
        n >= 2
    };
    let mut out = Vec::new();
    for &l in &body.face(f).loops {
        let lp = body.lp(l);
        if !lp.alive {
            continue;
        }
        let cs: Vec<CoedgeId> = lp.coedges.iter().copied().filter(|&c| body.coedge(c).alive).collect();
        let mut runs: Vec<Vec<CoedgeId>> = Vec::new();
        let mut cur = Vec::new();
        let mut any_seam = false;
        for &c in &cs {
            if seam(c) {
                any_seam = true;
                if !cur.is_empty() {
                    runs.push(std::mem::take(&mut cur));
                }
            } else {
                cur.push(c);
            }
        }
        if !cur.is_empty() {
            // GUESS: a run wrapping past the loop start joins the first run.
            if any_seam && !runs.is_empty() && !seam(cs[0]) {
                cur.extend(runs[0].iter().copied());
                runs[0] = cur;
            } else {
                runs.push(cur);
            }
        }
        out.extend(join_open_runs(body, runs));
    }
    out
}

/// A hole that straddles the seam of a periodic face (a cross hole through a
/// cylinder at its reference angle) is cut by the dropped seam into runs that
/// are open chains each; they close up with one another. Chain every open run
/// to the open run that starts where it ends so the loop is written whole.
fn join_open_runs(body: &Body, runs: Vec<Vec<CoedgeId>>) -> Vec<Vec<CoedgeId>> {
    let closed = |r: &Vec<CoedgeId>| body.coedge_start(r[0]) == body.coedge_end(*r.last().unwrap());
    if runs.iter().all(closed) {
        return runs;
    }
    let mut used = vec![false; runs.len()];
    let mut out = Vec::new();
    for i in 0..runs.len() {
        if used[i] {
            continue;
        }
        used[i] = true;
        let mut cur = runs[i].clone();
        while !closed(&cur) {
            let end = body.coedge_end(*cur.last().unwrap());
            let Some(j) = (0..runs.len()).find(|&j| !used[j] && body.coedge_start(runs[j][0]) == end) else { break };
            used[j] = true;
            cur.extend(runs[j].iter().copied());
        }
        out.push(cur);
    }
    out
}

/// Planar faces: index of the piece with the largest positive area about
/// the outward normal.
fn outer_piece(body: &Body, f: FaceId, pieces: &[Vec<CoedgeId>]) -> Option<usize> {
    let face = body.face(f);
    let Surface::Plane { frame } = &face.surface else { return None };
    let n = if face.sense { frame.z } else { -frame.z };
    let mut best: Option<(usize, f64)> = None;
    for (k, piece) in pieces.iter().enumerate() {
        let mut pts = Vec::new();
        for &c in piece {
            let ce = body.coedge(c);
            let e = body.edge(ce.edge);
            let steps = if matches!(e.curve, Curve::Line { .. }) { 1 } else { 32 };
            for i in 0..steps {
                let s = i as f64 / steps as f64;
                let t = if ce.rev { e.t1 + (e.t0 - e.t1) * s } else { e.t0 + (e.t1 - e.t0) * s };
                pts.push(e.curve.eval(t));
            }
        }
        let mut a = Vec3::ZERO;
        for i in 0..pts.len() {
            a = a + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        let area = a.dot(n);
        if area > 0.0 && best.map_or(true, |(_, b)| area > b) {
            best = Some((k, area));
        }
    }
    best.map(|b| b.0)
}

/// Every edge used by the shell's faces has two coedges in the shell.
fn shell_closed(body: &Body, s: ShellId) -> bool {
    let mut count: HashMap<EdgeId, usize> = HashMap::new();
    for &f in &body.shell(s).faces {
        let face = body.face(f);
        if !face.alive {
            continue;
        }
        for &l in &face.loops {
            for &c in &body.lp(l).coedges {
                let ce = body.coedge(c);
                if ce.alive {
                    *count.entry(ce.edge).or_default() += 1;
                }
            }
        }
    }
    count.values().all(|&n| n == 2)
}

/// The full `step` reply text for one body.
pub fn write_step(body: &Body, name: &str, opts: &StepOpts) -> String {
    write_step_impl(body, name, opts, false)
}

/// Whole-word `NaN` / `inf` outside string literals: what [`real`] prints for a
/// non-finite value, which is not a valid Part 21 REAL.
pub fn has_non_finite_token(text: &str) -> bool {
    let mut in_str = false;
    let mut word = String::new();
    for c in text.chars().chain(std::iter::once(' ')) {
        if c == '\'' {
            in_str = !in_str;
            word.clear();
        } else if in_str {
            continue;
        } else if c.is_ascii_alphanumeric() || c == '_' {
            word.push(c);
        } else {
            if word == "NaN" || word == "inf" {
                return true;
            }
            word.clear();
        }
    }
    false
}

/// [`write_step`] that refuses a body holding a NaN or infinite number (the
/// file would otherwise contain `NaN` / `inf`, which no STEP reader accepts).
pub fn try_write_step(body: &Body, name: &str, opts: &StepOpts) -> Result<String, String> {
    let t = write_step(body, name, opts);
    if has_non_finite_token(&t) {
        Err("the body holds a non-finite number (NaN or infinity); STEP export refused".to_string())
    } else {
        Ok(t)
    }
}

/// Test support: [`write_step`] with negative-zero DIRECTION components
/// printed as `-0.` (the oracle prints `0.`), so goldens can pin the sign of
/// zero of every direction, not only the resulting sharing.
#[doc(hidden)]
pub fn write_step_signed_zeros(body: &Body, name: &str, opts: &StepOpts) -> String {
    write_step_impl(body, name, opts, true)
}

fn write_step_impl(body: &Body, name: &str, opts: &StepOpts, signed_zero: bool) -> String {
    let nm = string(name);
    let mut w = StepWriter::new();
    w.signed_zero = signed_zero;
    for ch in split_chains(body, body.wire_chains()).into_iter().filter(|c| c.len() > 1) {
        for &e in &ch {
            w.chain.insert(e, (ch[0], *ch.last().unwrap()));
        }
    }
    w.ent("APPLICATION_CONTEXT('core data for automotive mechanical design processes')");
    w.ent("APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#1)");
    w.ent("PRODUCT_CONTEXT('',#1,'mechanical')");
    w.ent(&format!("PRODUCT('{nm}','{nm}','',(#3))"));
    w.ent("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#4))");
    w.ent("PRODUCT_DEFINITION_FORMATION('','',#4)");
    w.ent("PRODUCT_DEFINITION_CONTEXT('part definition',#1,'design')");
    w.ent("PRODUCT_DEFINITION('design','',#6,#7)");
    let pds = w.ent("PRODUCT_DEFINITION_SHAPE('','',#8)");
    w.ent("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))");
    w.ent("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))");
    w.ent("(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT())");
    w.ent("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-06),#10,'distance_accuracy_value','confusion accuracy')");
    let ctx = w.ent("(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#13))GLOBAL_UNIT_ASSIGNED_CONTEXT((#10,#11,#12))REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY'))");
    let world = w.axis2(Vec3::ZERO, Vec3::Z, Vec3::X);

    // Shells per lump: first alive shell = outer, the rest are voids.
    let mut solids: Vec<String> = Vec::new();
    let mut open: Vec<usize> = Vec::new();
    for l in body.lump_ids() {
        let shells: Vec<ShellId> = body.lump(l).shells.iter().copied().filter(|&s| body.shell(s).alive).collect();
        let mut outer = None;
        let mut voids = Vec::new();
        for (k, &s) in shells.iter().enumerate() {
            let (id, closed) = w.shell(body, s, k > 0);
            if !closed {
                open.push(id);
            } else if k == 0 {
                outer = Some(id);
            } else {
                voids.push(w.ent(&format!("ORIENTED_CLOSED_SHELL('',*,#{id},.F.)")));
            }
        }
        if let Some(o) = outer {
            solids.push(if voids.is_empty() {
                format!("MANIFOLD_SOLID_BREP('{nm}',#{o})")
            } else {
                format!("BREP_WITH_VOIDS('{nm}',#{o},({}))", refs(&voids))
            });
        }
    }
    let mut items: Vec<usize> = solids.iter().map(|s| w.ent(s)).collect();
    // GUESS: open shells (never produced by an oracle regen body so far):
    // SHELL_BASED_SURFACE_MODEL + MANIFOLD_SURFACE_SHAPE_REPRESENTATION,
    // linked to the solid representation by SHAPE_REPRESENTATION_RELATIONSHIP
    // when both exist (all three strings are in kernel.strings).
    let surf_rep = if open.is_empty() {
        None
    } else {
        let m = w.ent(&format!("SHELL_BASED_SURFACE_MODEL('{nm}',({}))", refs(&open)));
        Some(m)
    };
    let rep = if !items.is_empty() || surf_rep.is_none() {
        items.push(world);
        let r = w.ent(&format!("ADVANCED_BREP_SHAPE_REPRESENTATION('{nm}',({}),#{ctx})", refs(&items)));
        Some(r)
    } else {
        None
    };
    let mrep = surf_rep.map(|m| w.ent(&format!("MANIFOLD_SURFACE_SHAPE_REPRESENTATION('{nm}',(#{m},#{world}),#{ctx})")));
    match (rep, mrep) {
        (Some(r), Some(m)) => {
            w.ent(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{r})"));
            w.ent(&format!("SHAPE_REPRESENTATION_RELATIONSHIP('','',#{r},#{m})"));
        }
        (Some(r), None) | (None, Some(r)) => {
            w.ent(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{r})"));
        }
        (None, None) => {}
    }

    let mut s = String::new();
    s.push_str("ISO-10303-21;\nHEADER;\nFILE_DESCRIPTION(('CadKit model'),'2;1');\n");
    let _ = writeln!(
        s,
        "FILE_NAME('{nm}.step','{}',(''),(''),'CadKit','CadKit','');",
        timestamp(opts.unix_ms)
    );
    s.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
    s.push_str(w.text());
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reals_match_oracle() {
        for (x, s) in [
            (0.0, "0."),
            (1.0, "1."),
            (-1.0, "-1."),
            (0.5, "0.5"),
            (1e-5, "0.00001"),
            (1e-6, "0.000001"),
            (1.5e-6, "0.0000015"),
            (9.99e-7, "9.99E-7"),
            (1e-7, "1.E-7"),
            (1.234e-7, "1.234E-7"),
            (2.5e-10, "2.5E-10"),
            (1e-100, "1.E-100"),
            (5e-324, "5.E-324"),
            (6.123233995736766e-17, "6.123233995736766E-17"),
            (123456.0, "123456."),
            (999999999999999.0, "999999999999999."),
            (999999999999999.9, "999999999999999.9"),
            (1e15, "1.E15"),
            (-1e15, "-1.E15"),
            (1.5e17, "1.5E17"),
            (123456789012345680.0, "1.2345678901234568E17"),
            (-2.5e-8, "-2.5E-8"),
            (-2.499999999999999, "-2.499999999999999"),
        ] {
            assert_eq!(real(x), s, "{x}");
        }
    }

    #[test]
    fn timestamps_match_oracle() {
        for (ms, s) in [
            (0.0, "1970-01-01T00:00:00"),
            (1760000000123.0, "2025-10-09T08:53:20"),
            (1767225599999.0, "2025-12-31T23:59:59"),
            (1767225600000.0, "2026-01-01T00:00:00"),
            (1800000000000.0, "2027-01-15T08:00:00"),
            (4102444800000.0, "2100-01-01T00:00:00"),
            (951782400000.0, "2000-02-29T00:00:00"),
            (-1000.0, "1970-01-01T00:00:00"),
            (f64::NAN, "1970-01-01T00:00:00"),
            (1e20, "3168875820-09-06T09:46:40"),
        ] {
            assert_eq!(timestamp(ms), s, "{ms}");
        }
    }

    #[test]
    fn strings_match_oracle() {
        for (a, b) in [
            ("a'b", "a''b"),
            ("a\\b", "a\\\\b"),
            ("é", "\\X2\\00E9\\X0\\"),
            ("€😀", "\\X2\\20AC\\X0\\\\X2\\D83DDE00\\X0\\"),
            ("a\nb\tc", "abc"),
            ("A/B:C*?", "A/B:C*?"),
            ("naïve ü", "na\\X2\\00EF\\X0\\ve \\X2\\00FC\\X0\\"),
            ("\u{7f}\u{80}", "\\X2\\0080\\X0\\"),
        ] {
            assert_eq!(string(a), b);
        }
    }
}
