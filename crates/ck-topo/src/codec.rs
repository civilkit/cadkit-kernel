//! Lossless binary form of a [`Body`], for caches that outlive the process (see `docs/regen-cache.md`, "Saved
//! bodies").
//!
//! The arenas are written as they are, dead entries included, so every id stays what it was and a restored body
//! compares equal (`==`) to the one that was written; every float is its 8 bytes (little endian), so the geometry is
//! bit-for-bit the same. The layout is flat and regular (counts, then records) so that a general compressor does well on
//! it. Nothing here is trusted on the way back: [`decode`] checks the magic and version, every count against the bytes
//! left, every index against its arena and every tag, and returns `Err` for anything else, so damaged input is an error
//! and never a panic or a body with dangling ids.
//!
//! Layout (all integers little endian, `u32` unless said): `"CKBD"`, version, then the arenas in the order vertices,
//! edges, coedges, loops, faces, shells, lumps. Strings are a `u32` length and UTF-8 bytes.

use crate::body::*;
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};

/// Bumped when the layout changes; an older blob then fails to decode (the caller rebuilds).
pub const VERSION: u32 = 1;
const MAGIC: &[u8; 4] = b"CKBD";

// ------------------------------------------------------------------ writing

pub struct W {
    pub buf: Vec<u8>,
}

impl W {
    pub fn new() -> W {
        W { buf: Vec::new() }
    }
    pub fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    pub fn bool(&mut self, v: bool) {
        self.buf.push(v as u8);
    }
    pub fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn len(&mut self, n: usize) {
        self.u32(u32::try_from(n).expect("count fits u32"));
    }
    pub fn f64(&mut self, v: f64) {
        self.buf.extend_from_slice(&v.to_bits().to_le_bytes());
    }
    pub fn str(&mut self, s: &str) {
        self.len(s.len());
        self.buf.extend_from_slice(s.as_bytes());
    }
    fn v3(&mut self, p: Vec3) {
        self.f64(p.x);
        self.f64(p.y);
        self.f64(p.z);
    }
    fn v2(&mut self, p: Vec2) {
        self.f64(p.x);
        self.f64(p.y);
    }
    fn frame(&mut self, f: &Frame) {
        for p in [f.o, f.x, f.y, f.z] {
            self.v3(p);
        }
    }
    fn f64s(&mut self, v: &[f64]) {
        self.len(v.len());
        for &x in v {
            self.f64(x);
        }
    }
    fn weights(&mut self, w: &Option<Vec<f64>>) {
        match w {
            None => self.u8(0),
            Some(w) => {
                self.u8(1);
                self.f64s(w);
            }
        }
    }
    fn ids<T>(&mut self, v: &[T], f: impl Fn(&T) -> u32) {
        self.len(v.len());
        for x in v {
            self.u32(f(x));
        }
    }
    fn opt_str(&mut self, s: &Option<String>) {
        match s {
            None => self.u8(0),
            Some(s) => {
                self.u8(1);
                self.str(s);
            }
        }
    }
}

impl Default for W {
    fn default() -> Self {
        W::new()
    }
}

fn w_curve(w: &mut W, c: &Curve) {
    match c {
        Curve::Line { o, d } => {
            w.u8(0);
            w.v3(*o);
            w.v3(*d);
        }
        Curve::Circle { frame, r } => {
            w.u8(1);
            w.frame(frame);
            w.f64(*r);
        }
        Curve::Ellipse { frame, a, b } => {
            w.u8(2);
            w.frame(frame);
            w.f64(*a);
            w.f64(*b);
        }
        Curve::Conic { frame, p, e } => {
            w.u8(3);
            w.frame(frame);
            w.f64(*p);
            w.f64(*e);
        }
        Curve::Sinusoid { frame, amp, freq, phase } => {
            w.u8(4);
            w.frame(frame);
            w.f64(*amp);
            w.f64(*freq);
            w.f64(*phase);
        }
        Curve::BSpline(b) => {
            w.u8(5);
            w.len(b.degree);
            w.f64s(&b.knots);
            w.len(b.ctrl.len());
            for &p in &b.ctrl {
                w.v3(p);
            }
            w.weights(&b.weights);
        }
    }
}

fn w_curve2(w: &mut W, c: &Curve2) {
    match c {
        Curve2::Line { o, d } => {
            w.u8(0);
            w.v2(*o);
            w.v2(*d);
        }
        Curve2::Circle { c, x, r, ccw } => {
            w.u8(1);
            w.v2(*c);
            w.v2(*x);
            w.f64(*r);
            w.bool(*ccw);
        }
        Curve2::Ellipse { c, x, a, b, ccw } => {
            w.u8(2);
            w.v2(*c);
            w.v2(*x);
            w.f64(*a);
            w.f64(*b);
            w.bool(*ccw);
        }
        Curve2::BSpline(b) => {
            w.u8(3);
            w.len(b.degree);
            w.f64s(&b.knots);
            w.len(b.ctrl.len());
            for &p in &b.ctrl {
                w.v2(p);
            }
            w.weights(&b.weights);
        }
    }
}

fn w_surface(w: &mut W, s: &Surface) {
    match s {
        Surface::Plane { frame } => {
            w.u8(0);
            w.frame(frame);
        }
        Surface::Cylinder { frame, r } => {
            w.u8(1);
            w.frame(frame);
            w.f64(*r);
        }
        Surface::Cone { frame, r, semi_angle } => {
            w.u8(2);
            w.frame(frame);
            w.f64(*r);
            w.f64(*semi_angle);
        }
        Surface::Sphere { frame, r } => {
            w.u8(3);
            w.frame(frame);
            w.f64(*r);
        }
        Surface::Torus { frame, major, minor } => {
            w.u8(4);
            w.frame(frame);
            w.f64(*major);
            w.f64(*minor);
        }
        Surface::Revolution { profile, axis_o, axis } => {
            w.u8(5);
            w_curve(w, profile);
            w.v3(*axis_o);
            w.v3(*axis);
        }
        Surface::Extrusion { profile, dir } => {
            w.u8(6);
            w_curve(w, profile);
            w.v3(*dir);
        }
        Surface::BSpline(b) => {
            w.u8(7);
            w.len(b.deg_u);
            w.len(b.deg_v);
            w.f64s(&b.knots_u);
            w.f64s(&b.knots_v);
            w.len(b.nu);
            w.len(b.nv);
            w.len(b.ctrl.len());
            for &p in &b.ctrl {
                w.v3(p);
            }
            w.weights(&b.weights);
        }
        Surface::Screw { profile, axis_o, axis, rise, urange, vrange } => {
            w.u8(8);
            w_curve(w, profile);
            w.v3(*axis_o);
            w.v3(*axis);
            w.f64(*rise);
            w.f64(urange.0);
            w.f64(urange.1);
            w.f64(vrange.0);
            w.f64(vrange.1);
        }
        Surface::Offset { base, dist } => {
            w.u8(9);
            w_surface(w, base);
            w.f64(*dist);
        }
    }
}

fn w_role(w: &mut W, r: &Role) {
    match r {
        Role::Side(s) => {
            w.u8(0);
            w.str(s);
        }
        Role::Cap(k) => {
            w.u8(1);
            w.u32(*k);
        }
        Role::Blend(k) => {
            w.u8(2);
            w.u32(*k);
        }
        Role::Other(s) => {
            w.u8(3);
            w.str(s);
        }
    }
}

/// The binary form of `b`.
pub fn encode(b: &Body) -> Vec<u8> {
    let mut w = W::new();
    w.buf.extend_from_slice(MAGIC);
    w.u32(VERSION);
    w.len(b.vertices.len());
    for v in &b.vertices {
        w.v3(v.p);
        w.f64(v.tol);
        w.bool(v.alive);
    }
    w.len(b.edges.len());
    for e in &b.edges {
        w_curve(&mut w, &e.curve);
        w.f64(e.t0);
        w.f64(e.t1);
        w.u32(e.v0.0);
        w.u32(e.v1.0);
        w.ids(&e.coedges, |c| c.0);
        w.f64(e.tol);
        w.bool(e.alive);
    }
    w.len(b.coedges.len());
    for c in &b.coedges {
        w.u32(c.edge.0);
        w.bool(c.rev);
        w.u32(c.lp.0);
        match &c.pcurve {
            None => w.u8(0),
            Some(p) => {
                w.u8(1);
                w_curve2(&mut w, p);
            }
        }
        w.bool(c.alive);
    }
    w.len(b.loops.len());
    for l in &b.loops {
        w.u32(l.face.0);
        w.ids(&l.coedges, |c| c.0);
        w.bool(l.alive);
    }
    w.len(b.faces.len());
    for f in &b.faces {
        w_surface(&mut w, &f.surface);
        w.bool(f.sense);
        w.ids(&f.loops, |l| l.0);
        w.u32(f.shell.0);
        w.str(&f.name.feature);
        w_role(&mut w, &f.name.role);
        match f.name.region {
            None => w.u8(0),
            Some(r) => {
                w.u8(1);
                w.u32(r);
            }
        }
        w.str(&f.name.suffix);
        w.len(f.absorbed.len());
        for a in &f.absorbed {
            w.str(a);
        }
        w.bool(f.alive);
    }
    w.len(b.shells.len());
    for s in &b.shells {
        w.ids(&s.faces, |f| f.0);
        w.u32(s.lump.0);
        w.bool(s.alive);
    }
    w.len(b.lumps.len());
    for l in &b.lumps {
        w.ids(&l.shells, |s| s.0);
        w.bool(l.alive);
        w.opt_str(&l.name);
    }
    w.buf
}

// ------------------------------------------------------------------ reading

pub struct R<'a> {
    b: &'a [u8],
    pos: usize,
}

type Res<T> = Result<T, String>;

impl<'a> R<'a> {
    pub fn new(b: &'a [u8]) -> R<'a> {
        R { b, pos: 0 }
    }
    fn take(&mut self, n: usize) -> Res<&'a [u8]> {
        let end = self.pos.checked_add(n).filter(|&e| e <= self.b.len()).ok_or("truncated")?;
        let s = &self.b[self.pos..end];
        self.pos = end;
        Ok(s)
    }
    pub fn u8(&mut self) -> Res<u8> {
        Ok(self.take(1)?[0])
    }
    pub fn bool(&mut self) -> Res<bool> {
        match self.u8()? {
            0 => Ok(false),
            1 => Ok(true),
            t => Err(format!("bad bool {t}")),
        }
    }
    pub fn u32(&mut self) -> Res<u32> {
        Ok(u32::from_le_bytes(self.take(4)?.try_into().unwrap()))
    }
    pub fn f64(&mut self) -> Res<f64> {
        Ok(f64::from_bits(u64::from_le_bytes(self.take(8)?.try_into().unwrap())))
    }
    /// A count of records of at least `min` bytes each, checked against the bytes left.
    pub fn count(&mut self, min: usize) -> Res<usize> {
        let n = self.u32()? as usize;
        if n.checked_mul(min.max(1)).map_or(true, |t| t > self.b.len() - self.pos) {
            return Err("count exceeds data".into());
        }
        Ok(n)
    }
    pub fn str(&mut self) -> Res<String> {
        let n = self.count(1)?;
        String::from_utf8(self.take(n)?.to_vec()).map_err(|_| "bad utf-8".to_string())
    }
    /// The next `n` bytes.
    pub fn bytes(&mut self, n: usize) -> Res<&'a [u8]> {
        self.take(n)
    }
    pub fn at_end(&self) -> bool {
        self.pos == self.b.len()
    }
    fn v3(&mut self) -> Res<Vec3> {
        Ok(Vec3::new(self.f64()?, self.f64()?, self.f64()?))
    }
    fn v2(&mut self) -> Res<Vec2> {
        Ok(Vec2 { x: self.f64()?, y: self.f64()? })
    }
    fn frame(&mut self) -> Res<Frame> {
        Ok(Frame { o: self.v3()?, x: self.v3()?, y: self.v3()?, z: self.v3()? })
    }
    fn f64s(&mut self) -> Res<Vec<f64>> {
        let n = self.count(8)?;
        (0..n).map(|_| self.f64()).collect()
    }
    fn weights(&mut self) -> Res<Option<Vec<f64>>> {
        match self.u8()? {
            0 => Ok(None),
            1 => Ok(Some(self.f64s()?)),
            t => Err(format!("bad weights tag {t}")),
        }
    }
    fn ids(&mut self, limit: usize, what: &str) -> Res<Vec<u32>> {
        let n = self.count(4)?;
        (0..n)
            .map(|_| {
                let i = self.u32()?;
                if (i as usize) < limit {
                    Ok(i)
                } else {
                    Err(format!("{what} index {i} out of range"))
                }
            })
            .collect()
    }
    fn opt_str(&mut self) -> Res<Option<String>> {
        match self.u8()? {
            0 => Ok(None),
            1 => Ok(Some(self.str()?)),
            t => Err(format!("bad option tag {t}")),
        }
    }
}

fn r_curve(r: &mut R) -> Res<Curve> {
    Ok(match r.u8()? {
        0 => Curve::Line { o: r.v3()?, d: r.v3()? },
        1 => Curve::Circle { frame: r.frame()?, r: r.f64()? },
        2 => Curve::Ellipse { frame: r.frame()?, a: r.f64()?, b: r.f64()? },
        3 => Curve::Conic { frame: r.frame()?, p: r.f64()?, e: r.f64()? },
        4 => Curve::Sinusoid { frame: r.frame()?, amp: r.f64()?, freq: r.f64()?, phase: r.f64()? },
        5 => {
            let degree = r.u32()? as usize;
            let knots = r.f64s()?;
            let n = r.count(24)?;
            let ctrl = (0..n).map(|_| r.v3()).collect::<Res<Vec<_>>>()?;
            let weights = r.weights()?;
            check_spline(degree, knots.len(), ctrl.len(), &weights)?;
            Curve::BSpline(ck_geom::nurbs::bspline::BSplineCurve { degree, knots, ctrl, weights })
        }
        t => return Err(format!("bad curve tag {t}")),
    })
}

fn check_spline(degree: usize, knots: usize, ctrl: usize, w: &Option<Vec<f64>>) -> Res<()> {
    if degree == 0 || degree > 64 || ctrl == 0 || knots != ctrl + degree + 1 || w.as_ref().map_or(false, |w| w.len() != ctrl) {
        return Err("inconsistent spline".into());
    }
    Ok(())
}

fn r_curve2(r: &mut R) -> Res<Curve2> {
    Ok(match r.u8()? {
        0 => Curve2::Line { o: r.v2()?, d: r.v2()? },
        1 => Curve2::Circle { c: r.v2()?, x: r.v2()?, r: r.f64()?, ccw: r.bool()? },
        2 => Curve2::Ellipse { c: r.v2()?, x: r.v2()?, a: r.f64()?, b: r.f64()?, ccw: r.bool()? },
        3 => {
            let degree = r.u32()? as usize;
            let knots = r.f64s()?;
            let n = r.count(16)?;
            let ctrl = (0..n).map(|_| r.v2()).collect::<Res<Vec<_>>>()?;
            let weights = r.weights()?;
            check_spline(degree, knots.len(), ctrl.len(), &weights)?;
            Curve2::BSpline(ck_geom::nurbs::bspline::BSplineCurve { degree, knots, ctrl, weights })
        }
        t => return Err(format!("bad pcurve tag {t}")),
    })
}

fn r_surface(r: &mut R, depth: usize) -> Res<Surface> {
    if depth > 8 {
        return Err("offset chain too deep".into());
    }
    Ok(match r.u8()? {
        0 => Surface::Plane { frame: r.frame()? },
        1 => Surface::Cylinder { frame: r.frame()?, r: r.f64()? },
        2 => Surface::Cone { frame: r.frame()?, r: r.f64()?, semi_angle: r.f64()? },
        3 => Surface::Sphere { frame: r.frame()?, r: r.f64()? },
        4 => Surface::Torus { frame: r.frame()?, major: r.f64()?, minor: r.f64()? },
        5 => Surface::Revolution { profile: Box::new(r_curve(r)?), axis_o: r.v3()?, axis: r.v3()? },
        6 => Surface::Extrusion { profile: Box::new(r_curve(r)?), dir: r.v3()? },
        7 => {
            let (deg_u, deg_v) = (r.u32()? as usize, r.u32()? as usize);
            let knots_u = r.f64s()?;
            let knots_v = r.f64s()?;
            let (nu, nv) = (r.u32()? as usize, r.u32()? as usize);
            let n = r.count(24)?;
            let ctrl = (0..n).map(|_| r.v3()).collect::<Res<Vec<_>>>()?;
            let weights = r.weights()?;
            check_spline(deg_u, knots_u.len(), nu, &None)?;
            check_spline(deg_v, knots_v.len(), nv, &None)?;
            if nu.checked_mul(nv) != Some(ctrl.len()) || weights.as_ref().map_or(false, |w| w.len() != ctrl.len()) {
                return Err("inconsistent spline surface".into());
            }
            Surface::BSpline(Box::new(ck_geom::nurbs::bsurf::BSplineSurface { deg_u, deg_v, knots_u, knots_v, nu, nv, ctrl, weights }))
        }
        8 => Surface::Screw {
            profile: Box::new(r_curve(r)?),
            axis_o: r.v3()?,
            axis: r.v3()?,
            rise: r.f64()?,
            urange: (r.f64()?, r.f64()?),
            vrange: (r.f64()?, r.f64()?),
        },
        9 => Surface::Offset { base: Box::new(r_surface(r, depth + 1)?), dist: r.f64()? },
        t => return Err(format!("bad surface tag {t}")),
    })
}

fn r_role(r: &mut R) -> Res<Role> {
    Ok(match r.u8()? {
        0 => Role::Side(r.str()?),
        1 => Role::Cap(r.u32()?),
        2 => Role::Blend(r.u32()?),
        3 => Role::Other(r.str()?),
        t => return Err(format!("bad role tag {t}")),
    })
}

/// The body `bytes` hold, or why they are not one.
pub fn decode(bytes: &[u8]) -> Result<Body, String> {
    let mut r = R::new(bytes);
    if r.take(4)? != MAGIC {
        return Err("not a body blob".into());
    }
    let v = r.u32()?;
    if v != VERSION {
        return Err(format!("body blob version {v}, expected {VERSION}"));
    }
    let b = decode_inner(&mut r)?;
    if !r.at_end() {
        return Err("trailing bytes".into());
    }
    Ok(b)
}

/// Decode the body at the reader's position (after the header [`encode`] writes has been consumed by [`decode`]); used by
/// [`decode`] only. Counts of every arena are read up front so indices can be range-checked as they are met.
fn decode_inner(r: &mut R) -> Result<Body, String> {
    // Pre-scan is not possible with a single pass over variable-size records, so records are read first and their
    // indices checked against the final arena lengths afterwards.
    let mut b = Body::new();
    let nv = r.count(33)?;
    for _ in 0..nv {
        b.vertices.push(Vertex { p: r.v3()?, tol: r.f64()?, alive: r.bool()? });
    }
    let ne = r.count(30)?;
    for _ in 0..ne {
        let curve = r_curve(r)?;
        let (t0, t1) = (r.f64()?, r.f64()?);
        let (v0, v1) = (r.u32()?, r.u32()?);
        let coedges = r.ids(usize::MAX, "coedge")?.into_iter().map(CoedgeId).collect();
        let tol = r.f64()?;
        let alive = r.bool()?;
        b.edges.push(Edge { curve, t0, t1, v0: VertexId(v0), v1: VertexId(v1), coedges, tol, alive });
    }
    let nc = r.count(10)?;
    for _ in 0..nc {
        let edge = r.u32()?;
        let rev = r.bool()?;
        let lp = r.u32()?;
        let pcurve = match r.u8()? {
            0 => None,
            1 => Some(r_curve2(r)?),
            t => return Err(format!("bad pcurve option {t}")),
        };
        let alive = r.bool()?;
        b.coedges.push(Coedge { edge: EdgeId(edge), rev, lp: LoopId(lp), pcurve, alive });
    }
    let nl = r.count(9)?;
    for _ in 0..nl {
        let face = r.u32()?;
        let coedges = r.ids(usize::MAX, "coedge")?.into_iter().map(CoedgeId).collect();
        b.loops.push(Loop { face: FaceId(face), coedges, alive: r.bool()? });
    }
    let nf = r.count(20)?;
    for _ in 0..nf {
        let surface = r_surface(r, 0)?;
        let sense = r.bool()?;
        let loops = r.ids(usize::MAX, "loop")?.into_iter().map(LoopId).collect();
        let shell = r.u32()?;
        let feature = r.str()?;
        let role = r_role(r)?;
        let region = match r.u8()? {
            0 => None,
            1 => Some(r.u32()?),
            t => return Err(format!("bad region option {t}")),
        };
        let suffix = r.str()?;
        let na = r.count(4)?;
        let absorbed = (0..na).map(|_| r.str()).collect::<Res<Vec<_>>>()?;
        let alive = r.bool()?;
        b.faces.push(Face { surface, sense, loops, shell: ShellId(shell), name: FaceName { feature, role, region, suffix }, absorbed, alive });
    }
    let ns = r.count(9)?;
    for _ in 0..ns {
        let faces = r.ids(usize::MAX, "face")?.into_iter().map(FaceId).collect();
        let lump = r.u32()?;
        b.shells.push(Shell { faces, lump: LumpId(lump), alive: r.bool()? });
    }
    let nl2 = r.count(6)?;
    for _ in 0..nl2 {
        let shells = r.ids(usize::MAX, "shell")?.into_iter().map(ShellId).collect();
        let alive = r.bool()?;
        let name = r.opt_str()?;
        b.lumps.push(Lump { shells, alive, name });
    }
    check_indices(&b)?;
    Ok(b)
}

/// Every id in `b` names an entry of its arena.
fn check_indices(b: &Body) -> Result<(), String> {
    let (nv, ne, nc, nl, nf, ns, nu) = (b.vertices.len(), b.edges.len(), b.coedges.len(), b.loops.len(), b.faces.len(), b.shells.len(), b.lumps.len());
    let bad = |what: &str| Err(format!("{what} index out of range"));
    for e in &b.edges {
        if e.v0.idx() >= nv || e.v1.idx() >= nv {
            return bad("vertex");
        }
        if e.coedges.iter().any(|c| c.idx() >= nc) {
            return bad("coedge");
        }
    }
    for c in &b.coedges {
        if c.edge.idx() >= ne {
            return bad("edge");
        }
        if c.lp.idx() >= nl {
            return bad("loop");
        }
    }
    for l in &b.loops {
        if l.face.idx() >= nf {
            return bad("face");
        }
        if l.coedges.iter().any(|c| c.idx() >= nc) {
            return bad("coedge");
        }
    }
    for f in &b.faces {
        if f.shell.idx() >= ns {
            return bad("shell");
        }
        if f.loops.iter().any(|l| l.idx() >= nl) {
            return bad("loop");
        }
    }
    for s in &b.shells {
        if s.lump.idx() >= nu {
            return bad("lump");
        }
        if s.faces.iter().any(|f| f.idx() >= nf) {
            return bad("face");
        }
    }
    for l in &b.lumps {
        if l.shells.iter().any(|s| s.idx() >= ns) {
            return bad("shell");
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use ck_geom::nurbs::{bspline::BSplineCurve, bsurf::BSplineSurface};

    /// One of every geometry kind, a dead entry of every arena, names of every role: not a valid solid, a codec fixture.
    fn sample() -> Body {
        let mut b = Body::new();
        let l = b.add_lump();
        b.lumps[l.idx()].name = Some("Body".into());
        let s = b.add_shell(l);
        let fr = Frame { o: Vec3::new(1.0, 2.0, 3.0), x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z: Vec3::new(0.0, 0.0, 1.0) };
        let v0 = b.add_vertex(Vec3::new(0.1, 0.2, f64::MIN_POSITIVE));
        let v1 = b.add_vertex(Vec3::new(-1e300, 1e-300, -0.0));
        b.vertices[v1.idx()].tol = 3e-4;
        b.vertices.push(Vertex { p: Vec3::new(9.0, 9.0, 9.0), tol: 0.0, alive: false });
        let spline = BSplineCurve { degree: 2, knots: vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0], ctrl: (0..4).map(|i| Vec3::new(i as f64, (i * i) as f64, 0.1)).collect(), weights: Some(vec![1.0, 0.5, 2.0, 1.0]) };
        let curves = vec![
            Curve::Line { o: Vec3::new(0.0, 0.0, 0.0), d: Vec3::new(1.0, 0.0, 0.0) },
            Curve::Circle { frame: fr, r: 2.5 },
            Curve::Ellipse { frame: fr, a: 3.0, b: 1.0 },
            Curve::Conic { frame: fr, p: 1.5, e: 0.3 },
            Curve::Sinusoid { frame: fr, amp: 1.0, freq: 2.0, phase: 0.25 },
            Curve::BSpline(spline.clone()),
        ];
        let pcs = vec![
            Curve2::Line { o: Vec2::new(0.0, 1.0), d: Vec2::new(1.0, 0.0) },
            Curve2::Circle { c: Vec2::new(0.0, 0.0), x: Vec2::new(1.0, 0.0), r: 1.0, ccw: false },
            Curve2::Ellipse { c: Vec2::new(1.0, 0.0), x: Vec2::new(0.0, 1.0), a: 2.0, b: 1.0, ccw: true },
            Curve2::BSpline(BSplineCurve { degree: 1, knots: vec![0.0, 0.0, 1.0, 1.0], ctrl: vec![Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)], weights: None }),
        ];
        let surf = BSplineSurface { deg_u: 1, deg_v: 1, knots_u: vec![0.0, 0.0, 1.0, 1.0], knots_v: vec![0.0, 0.0, 1.0, 1.0], nu: 2, nv: 2, ctrl: vec![Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.1), Vec3::new(1.0, 0.0, 0.2), Vec3::new(1.0, 1.0, 0.3)], weights: Some(vec![1.0, 2.0, 3.0, 4.0]) };
        let surfaces = vec![
            Surface::Plane { frame: fr },
            Surface::Cylinder { frame: fr, r: 1.0 },
            Surface::Cone { frame: fr, r: 1.0, semi_angle: 0.3 },
            Surface::Sphere { frame: fr, r: 2.0 },
            Surface::Torus { frame: fr, major: 5.0, minor: 1.0 },
            Surface::Revolution { profile: Box::new(curves[5].clone()), axis_o: Vec3::new(0.0, 0.0, 0.0), axis: Vec3::new(0.0, 0.0, 1.0) },
            Surface::Extrusion { profile: Box::new(curves[1].clone()), dir: Vec3::new(0.0, 0.0, 7.0) },
            Surface::BSpline(Box::new(surf)),
            Surface::Screw { profile: Box::new(curves[0].clone()), axis_o: Vec3::new(0.0, 0.0, 0.0), axis: Vec3::new(0.0, 0.0, 1.0), rise: 0.5, urange: (0.0, 6.0), vrange: (-1.0, 1.0) },
            Surface::Offset { base: Box::new(Surface::Sphere { frame: fr, r: 2.0 }), dist: 0.25 },
        ];
        let names = vec![
            FaceName::side("F1", "e3"),
            FaceName::cap("F2", 1, Some(2)),
            FaceName::blend("F3", 4),
            FaceName::other("F4", "I1F7"),
        ];
        for (i, sf) in surfaces.into_iter().enumerate() {
            let mut n = names[i % names.len()].clone();
            if i == 3 {
                n.push_instance("F9.1");
            }
            let f = b.add_face(s, sf, i % 2 == 0, n);
            if i == 2 {
                b.faces[f.idx()].absorbed = vec!["F5.C0".into(), "F6.S.e1".into()];
            }
            let lp = b.add_loop(f);
            let e = b.add_edge(curves[i % curves.len()].clone(), 0.0, 1.0, v0, v1);
            b.edges[e.idx()].tol = 1e-3 * i as f64;
            b.add_coedge(lp, e, i % 3 == 0, if i % 5 == 4 { None } else { Some(pcs[i % pcs.len()].clone()) });
        }
        b.faces.push(Face { surface: Surface::Plane { frame: fr }, sense: true, loops: vec![], shell: s, name: FaceName::cap("F0", 0, None), absorbed: vec![], alive: false });
        b.shells.push(Shell { faces: vec![], lump: l, alive: false });
        b.lumps.push(Lump { shells: vec![], alive: false, name: None });
        b
    }

    #[test]
    fn round_trip_is_exact() {
        let b = sample();
        let bytes = encode(&b);
        let c = decode(&bytes).unwrap();
        assert_eq!(b, c);
        // same bits, not just `==` (-0.0, NaN payloads)
        assert_eq!(encode(&c), bytes);
        assert_eq!(c.vertices[1].p.z.to_bits(), (-0.0f64).to_bits());
    }

    #[test]
    fn empty_body_round_trips() {
        assert_eq!(decode(&encode(&Body::new())).unwrap(), Body::new());
    }

    #[test]
    fn every_truncation_is_an_error() {
        let bytes = encode(&sample());
        for n in 0..bytes.len() {
            assert!(decode(&bytes[..n]).is_err(), "prefix of {n} bytes decoded");
        }
    }

    #[test]
    fn damage_never_panics_or_leaves_dangling_ids() {
        let b = sample();
        let bytes = encode(&b);
        let mut seed = 0x9e3779b97f4a7c15u64;
        for _ in 0..4000 {
            let mut d = bytes.clone();
            for _ in 0..1 + seed % 3 {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
                let i = (seed >> 33) as usize % d.len();
                d[i] ^= 1 << ((seed >> 20) % 8);
            }
            if let Ok(c) = decode(&d) {
                // a flip that lands in a float is still a body; every id of it must be in range
                assert!(check_indices(&c).is_ok());
            }
        }
        assert!(decode(b"CKBD\x02\0\0\0").is_err());
        assert!(decode(b"nope").is_err());
        let mut extra = bytes.clone();
        extra.push(0);
        assert!(decode(&extra).is_err());
    }
}
