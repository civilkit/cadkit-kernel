//! SAT record table -> ck-topo [`Body`].

use super::parse::{Rec, Table};
use super::{SatError, SatImport};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::pcurve::pcurve_of;
use ck_geom::surface::Surface;
use ck_geom::{frame_from, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceName, ShellId, VertexId};
use std::collections::{HashMap, HashSet};
use std::f64::consts::{PI, TAU};

type R<T> = Result<T, SatError>;

/// Distance tolerance (mm) between a vertex and its curve / face surface.
fn geo_tol(p: Vec3) -> f64 {
    1e-4 + 1e-6 * p.len()
}

struct EInfo {
    id: EdgeId,
    /// Our edge direction (increasing parameter) is opposite to ACIS's
    /// start-vertex -> end-vertex direction.
    flip: bool,
}

/// One coedge of a loop being assembled.
#[derive(Clone)]
struct Ce {
    edge: EdgeId,
    rev: bool,
}

struct CurveG {
    curve: Curve,
    /// our parameter = file parameter * scale + offset
    scale: f64,
    offset: f64,
}

struct Cur<'a> {
    r: &'a Rec,
    k: usize,
}

impl<'a> Cur<'a> {
    fn tok(&mut self) -> R<&'a str> {
        let t = self.r.t.get(self.k).ok_or_else(|| SatError::at("sat.parse", self.r, format!("missing token {}", self.k)))?;
        self.k += 1;
        Ok(t.as_str())
    }
    fn f(&mut self) -> R<f64> {
        let t = self.tok()?;
        t.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| SatError::at("sat.parse", self.r, format!("cannot parse '{t}' as a number")))
    }
    fn u(&mut self) -> R<usize> {
        let t = self.tok()?;
        t.parse::<usize>().map_err(|_| SatError::at("sat.parse", self.r, format!("cannot parse '{t}' as a count")))
    }
}

struct B<'a> {
    t: &'a Table,
    body: Body,
    verts: HashMap<usize, (VertexId, Vec3)>,
    edges: HashMap<usize, EInfo>,
    units: f64,
}

pub fn build(t: &Table) -> R<SatImport> {
    let mut b = B { t, body: Body::new(), verts: HashMap::new(), edges: HashMap::new(), units: t.units };
    let bodies: Vec<&Rec> = t.recs.iter().filter(|r| r.ty() == "body").collect();
    if bodies.is_empty() {
        return Err(SatError { code: "sat.topology", entity: None, kind: String::new(), line: 0, detail: "no body record".into() });
    }
    // phase 1: lumps / shells / faces, and the single/double-sided census
    let mut plan: Vec<Vec<Vec<&Rec>>> = vec![];
    let (mut single, mut double) = (None::<&Rec>, None::<&Rec>);
    for br in &bodies {
        b.check_transform(br)?;
        let mut lumps = vec![];
        for lump in b.chain(b.ptr(br, 4)?, 4, "lump")? {
            let mut shells = vec![];
            for sh in b.chain(b.ptr(lump, 5)?, 4, "shell")? {
                let faces = b.chain(b.ptr(sh, 6)?, 4, "face")?;
                for f in &faces {
                    match b.tok(f, 10)? {
                        "single" => single = single.or(Some(f)),
                        "double" => double = double.or(Some(f)),
                        o => return Err(SatError::at("sat.parse", f, format!("bad face sidedness '{o}'"))),
                    }
                }
                shells.push(faces);
            }
            lumps.push(shells);
        }
        plan.extend(lumps);
    }
    if let (Some(s), Some(_)) = (single, double) {
        return Err(SatError::at("sat.unsupported", s, "mixed single- and double-sided faces (solid with internal sheets)"));
    }
    let sheet = double.is_some();
    // phase 2: build
    for shells in &plan {
        let lump = b.body.add_lump();
        for faces in shells {
            let sh = b.body.add_shell(lump);
            for f in faces {
                b.face(sh, f, sheet)?;
            }
        }
    }
    Ok(SatImport { body: b.body, sheet, units: t.units })
}

impl<'a> B<'a> {
    // ------------------------------------------------------------ records
    fn rec(&self, from: &Rec, i: usize) -> R<&'a Rec> {
        self.t.recs.get(i).ok_or_else(|| SatError::at("sat.topology", from, format!("pointer ${i} out of range")))
    }
    fn tok(&self, r: &Rec, k: usize) -> R<&'a str> {
        // lifetime: tokens live in the table
        let r2: &'a Rec = &self.t.recs[r.idx];
        r2.t.get(k).map(|s| s.as_str()).ok_or_else(|| SatError::at("sat.parse", r, format!("missing token {k}")))
    }
    fn ptr(&self, r: &Rec, k: usize) -> R<Option<usize>> {
        let t = self.tok(r, k)?;
        if t == "$-1" {
            return Ok(None);
        }
        t.strip_prefix('$')
            .and_then(|n| n.parse::<usize>().ok())
            .map(Some)
            .ok_or_else(|| SatError::at("sat.parse", r, format!("expected pointer at token {k}, got '{t}'")))
    }
    fn ptr_req(&self, r: &Rec, k: usize) -> R<usize> {
        self.ptr(r, k)?.ok_or_else(|| SatError::at("sat.topology", r, format!("null pointer at token {k}")))
    }
    fn num(&self, r: &Rec, k: usize) -> R<f64> {
        let t = self.tok(r, k)?;
        t.parse::<f64>().ok().filter(|v| v.is_finite()).ok_or_else(|| SatError::at("sat.parse", r, format!("cannot parse '{t}' as a number at token {k}")))
    }
    /// Raw 3-vector (no unit scaling).
    fn vec(&self, r: &Rec, k: usize) -> R<Vec3> {
        Ok(Vec3::new(self.num(r, k)?, self.num(r, k + 1)?, self.num(r, k + 2)?))
    }
    /// Point / length-valued vector scaled to mm.
    fn pt(&self, r: &Rec, k: usize) -> R<Vec3> {
        Ok(self.vec(r, k)? * self.units)
    }
    fn typed(&self, from: &Rec, i: usize, tys: &[&str]) -> R<&'a Rec> {
        let r = self.rec(from, i)?;
        if tys.contains(&r.ty()) {
            Ok(r)
        } else {
            Err(SatError::at("sat.topology", from, format!("${i} is a {} record, expected {}", r.ty(), tys.join("/"))))
        }
    }
    /// Follow a `next` pointer chain starting at `start`.
    fn chain(&self, start: Option<usize>, next_k: usize, ty: &str) -> R<Vec<&'a Rec>> {
        let mut out = vec![];
        let mut seen = HashSet::new();
        let mut cur = start;
        while let Some(i) = cur {
            let r = self.t.recs.get(i).ok_or_else(|| SatError::parse(0, format!("pointer ${i} out of range")))?;
            if r.ty() != ty {
                return Err(SatError::at("sat.topology", r, format!("expected a {ty} record in the chain")));
            }
            if !seen.insert(i) {
                return Err(SatError::at("sat.topology", r, format!("{ty} chain loops back on itself")));
            }
            out.push(r);
            cur = self.ptr(r, next_k)?;
        }
        Ok(out)
    }

    fn check_transform(&self, br: &Rec) -> R<()> {
        let Some(ti) = self.ptr(br, 6).ok().flatten() else { return Ok(()) };
        let tr = self.typed(br, ti, &["transform"])?;
        let mut v = vec![];
        for k in 4..17 {
            v.push(self.num(tr, k)?);
        }
        let id = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        if v.iter().zip(id.iter()).any(|(a, b)| (a - b).abs() > 1e-12) {
            return Err(SatError::at("sat.unsupported", tr, "non-identity body transform"));
        }
        Ok(())
    }

    // ------------------------------------------------------------ vertices / edges
    fn vertex(&mut self, from: &Rec, vi: usize) -> R<(VertexId, Vec3)> {
        if let Some(v) = self.verts.get(&vi) {
            return Ok(*v);
        }
        let vr = self.typed(from, vi, &["vertex"])?;
        let pr = self.typed(vr, self.ptr_req(vr, 5)?, &["point"])?;
        let p = self.pt(pr, 4)?;
        let id = self.body.add_vertex(p);
        self.verts.insert(vi, (id, p));
        Ok((id, p))
    }

    fn curve_g(&self, r: &Rec) -> R<CurveG> {
        match r.ty() {
            "straight-curve" => {
                let (o, d) = (self.pt(r, 4)?, self.vec(r, 7)?);
                let l = d.len();
                if l < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "zero direction"));
                }
                Ok(CurveG { curve: Curve::line(o, d), scale: l * self.units, offset: 0.0 })
            }
            "ellipse-curve" => {
                let (c, n, m) = (self.pt(r, 4)?, self.vec(r, 7)?, self.vec(r, 10)?);
                let ratio = self.num(r, 13)?;
                let a = m.len() * self.units;
                if n.len() < 1e-12 || a < 1e-12 || ratio <= 0.0 {
                    return Err(SatError::at("sat.geometry", r, "degenerate ellipse (axis, major radius or ratio)"));
                }
                let curve = if (ratio - 1.0).abs() < 1e-12 { Curve::circle_x(c, n, m, a) } else { Curve::ellipse(c, n, m, a, a * ratio) };
                Ok(CurveG { curve, scale: 1.0, offset: 0.0 })
            }
            "intcurve-curve" => {
                let reversed = match self.tok(r, 4)? {
                    "forward" => false,
                    "reversed" => true,
                    o => return Err(SatError::at("sat.parse", r, format!("bad intcurve sense '{o}'"))),
                };
                let c = self.spline_curve(r)?;
                let (a, b) = c.domain();
                if reversed {
                    // ACIS C'(s) = C(-s); ours R(u) = C(a+b-u)  =>  u = a+b+s
                    Ok(CurveG { curve: Curve::BSpline(c.reversed()), scale: 1.0, offset: a + b })
                } else {
                    Ok(CurveG { curve: Curve::BSpline(c), scale: 1.0, offset: 0.0 })
                }
            }
            other => Err(SatError::at("sat.unsupported", r, format!("curve type '{other}'"))),
        }
    }

    fn edge(&mut self, from: &Rec, ei: usize) -> R<(EdgeId, bool)> {
        if let Some(e) = self.edges.get(&ei) {
            return Ok((e.id, e.flip));
        }
        let er = self.typed(from, ei, &["edge"])?;
        let (vs, ps) = self.vertex(er, self.ptr_req(er, 4)?)?;
        let (ve, pe) = self.vertex(er, self.ptr_req(er, 6)?)?;
        let cr = self.rec(er, self.ptr_req(er, 9)?)?;
        let cg = self.curve_g(cr)?;
        let ts = self.num(er, 5)? * cg.scale + cg.offset;
        let te = self.num(er, 7)? * cg.scale + cg.offset;
        let reversed_sense = match self.tok(er, 10)? {
            "forward" => false,
            "reversed" => true,
            o => return Err(SatError::at("sat.parse", er, format!("bad edge sense '{o}'"))),
        };
        let (lo, hi) = (ts.min(te), ts.max(te));
        if !(hi - lo > 1e-12) {
            return Err(SatError::at("sat.unsupported", er, "zero-length / degenerate edge"));
        }
        let (p_ts, p_te) = (cg.curve.eval(ts), cg.curve.eval(te));
        let d_a = p_ts.dist(ps) + p_te.dist(pe);
        let d_b = p_ts.dist(pe) + p_te.dist(ps);
        let tie = vs == ve || ps.dist(pe) < 1e-9;
        let use_b = !tie && d_b < d_a;
        let best = if use_b { d_b } else { d_a };
        if best > 2.0 * (geo_tol(ps).max(geo_tol(pe))) {
            return Err(SatError::at("sat.geometry", er, format!("edge curve misses its vertices by {best:.3e}")));
        }
        let fwd_params = ts < te;
        let flip = if tie { (!fwd_params) ^ reversed_sense } else { (!fwd_params) ^ use_b };
        // vertex at the low / high parameter
        let (lo_v, hi_v) = if tie {
            (vs, ve)
        } else {
            let (v_ts, v_te) = if use_b { (ve, vs) } else { (vs, ve) };
            if fwd_params {
                (v_ts, v_te)
            } else {
                (v_te, v_ts)
            }
        };
        let id = self.body.add_edge(cg.curve, lo, hi, lo_v, hi_v);
        self.edges.insert(ei, EInfo { id, flip });
        Ok((id, flip))
    }

    // ------------------------------------------------------------ spline blocks
    /// Cursor positioned at the first token after `{ exactcur|exactsur [full]`.
    fn spline_cursor(&self, r: &'a Rec) -> R<Cur<'a>> {
        let open = r.t.iter().position(|s| s == "{").ok_or_else(|| SatError::at("sat.unsupported", r, "spline record without an inline definition block"))?;
        let mut k = open + 1;
        while let Some(t) = r.t.get(k) {
            match t.as_str() {
                "exactcur" | "exactsur" | "full" => k += 1,
                "nubs" | "nurbs" => break,
                "ref" => return Err(SatError::at("sat.unsupported", r, "shared spline definition (ref)")),
                other => return Err(SatError::at("sat.unsupported", r, format!("spline definition '{other}' (only exact nubs/nurbs)"))),
            }
        }
        Ok(Cur { r, k })
    }

    /// Distinct knots + multiplicities -> full clamped knot vector and the
    /// control-point count. ACIS stores end multiplicity = degree (one knot
    /// less than the textbook form at each end); degree+1 is accepted too.
    fn knots(&self, c: &mut Cur, deg: usize, n: usize) -> R<(Vec<f64>, usize)> {
        if n == 0 || n > 1_000_000 {
            return Err(SatError::at("sat.parse", c.r, "bad knot count"));
        }
        let mut vals = vec![];
        let mut mult = vec![];
        for _ in 0..n {
            vals.push(c.f()?);
            mult.push(c.u()?);
        }
        let total: usize = mult.iter().sum();
        let (f, l) = (mult[0], mult[n - 1]);
        let mut ks: Vec<f64> = vec![];
        for (v, m) in vals.iter().zip(&mult) {
            ks.extend(std::iter::repeat(*v).take(*m));
        }
        let ncp;
        if f == deg && l == deg {
            ks.insert(0, vals[0]);
            ks.push(vals[n - 1]);
            ncp = (total + 2).checked_sub(deg + 1).unwrap_or(0);
        } else if f == deg + 1 && l == deg + 1 {
            ncp = total.checked_sub(deg + 1).unwrap_or(0);
        } else {
            return Err(SatError::at("sat.unsupported", c.r, "non-clamped (periodic-style) knot vector"));
        }
        if ncp < deg + 1 {
            return Err(SatError::at("sat.parse", c.r, "too few control points for the degree"));
        }
        Ok((ks, ncp))
    }

    fn form(&self, c: &mut Cur) -> R<()> {
        match c.tok()? {
            "open" | "closed" => Ok(()),
            "periodic" => Err(SatError::at("sat.unsupported", c.r, "periodic spline")),
            o => Err(SatError::at("sat.parse", c.r, format!("bad spline form '{o}'"))),
        }
    }

    fn spline_curve(&self, r: &'a Rec) -> R<BSplineCurve> {
        let mut c = self.spline_cursor(r)?;
        let rational = c.tok()? == "nurbs";
        let deg = c.u()?;
        self.form(&mut c)?;
        let n = c.u()?;
        let (ks, ncp) = self.knots(&mut c, deg, n)?;
        if ncp > 10_000_000 {
            return Err(SatError::at("sat.parse", r, "absurd control point count"));
        }
        let mut ctrl = Vec::with_capacity(ncp);
        let mut w = Vec::with_capacity(ncp);
        for _ in 0..ncp {
            ctrl.push(Vec3::new(c.f()?, c.f()?, c.f()?) * self.units);
            if rational {
                w.push(c.f()?);
            }
        }
        let res = if rational { BSplineCurve::new_rational(deg, ks, ctrl, w) } else { BSplineCurve::new(deg, ks, ctrl) };
        res.map_err(|e| SatError::at("sat.geometry", r, format!("bad spline curve: {e}")))
    }

    fn spline_surface(&self, r: &'a Rec) -> R<BSplineSurface> {
        let mut c = self.spline_cursor(r)?;
        let rational = c.tok()? == "nurbs";
        let (du, dv) = (c.u()?, c.u()?);
        self.form(&mut c)?;
        self.form(&mut c)?;
        // singularity flags (u, v)
        for _ in 0..2 {
            match c.tok()? {
                "none" | "low" | "high" | "both" => {}
                o => return Err(SatError::at("sat.parse", r, format!("bad singularity flag '{o}'"))),
            }
        }
        let (nku, nkv) = (c.u()?, c.u()?);
        let (ku, ncu) = self.knots(&mut c, du, nku)?;
        let (kv, ncv) = self.knots(&mut c, dv, nkv)?;
        if ncu.saturating_mul(ncv) > 10_000_000 {
            return Err(SatError::at("sat.parse", r, "absurd control net size"));
        }
        let mut ctrl = Vec::with_capacity(ncu * ncv);
        let mut w = Vec::with_capacity(ncu * ncv);
        for _ in 0..ncu * ncv {
            ctrl.push(Vec3::new(c.f()?, c.f()?, c.f()?) * self.units);
            if rational {
                w.push(c.f()?);
            }
        }
        BSplineSurface::new(du, dv, ku, kv, ncu, ncv, ctrl, if rational { Some(w) } else { None })
            .map_err(|e| SatError::at("sat.geometry", r, format!("bad spline surface: {e}")))
    }

    // ------------------------------------------------------------ surfaces
    /// (surface, flip): `flip` = the ACIS natural normal is opposite to ours.
    fn surface(&self, r: &'a Rec, pts: &[Vec3]) -> R<(Surface, bool)> {
        match r.ty() {
            "plane-surface" => {
                let (o, n, u) = (self.pt(r, 4)?, self.vec(r, 7)?, self.vec(r, 10)?);
                if n.len() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "zero plane normal"));
                }
                Ok((Surface::Plane { frame: frame_from(o, n, u) }, false))
            }
            "cone-surface" => {
                let (o, ax, m) = (self.pt(r, 4)?, self.vec(r, 7)?, self.vec(r, 10)?);
                let ratio = self.num(r, 13)?;
                if (ratio - 1.0).abs() > 1e-9 {
                    return Err(SatError::at("sat.unsupported", r, "elliptical cone/cylinder (ratio != 1)"));
                }
                let r0 = m.len() * self.units;
                if ax.len() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "zero cone axis"));
                }
                // tokens: 14 sense, then the u-range (two bounds), then sin, cos
                let mut c = Cur { r, k: 15 };
                for _ in 0..2 {
                    match c.tok()? {
                        "I" => {}
                        "F" | "T" => {
                            c.f()?;
                        }
                        o => return Err(SatError::at("sat.parse", r, format!("bad range bound '{o}'"))),
                    }
                }
                let (s, co) = (c.f()?, c.f()?);
                if (s * s + co * co - 1.0).abs() > 1e-6 || co <= 0.0 {
                    return Err(SatError::at("sat.parse", r, "cone sine/cosine are not a half-angle"));
                }
                let frame = frame_from(o, ax, m);
                if r0 < 1e-12 && s.abs() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "zero-radius cylinder"));
                }
                if s.abs() < 1e-12 {
                    return Ok((Surface::Cylinder { frame, r: r0 }, false));
                }
                // sign of the taper: pick the one the face's vertices satisfy
                let t = (s / co).abs();
                let err = |tan: f64| -> f64 {
                    pts.iter()
                        .map(|p| {
                            let d = *p - frame.o;
                            let h = d.dot(frame.z);
                            let rho = (d - frame.z * h).len();
                            (rho - (r0 + h * tan)).abs()
                        })
                        .fold(0.0, f64::max)
                };
                let tan = if err(t) + 1e-9 < err(-t) { t } else { -t };
                Ok((Surface::Cone { frame, r: r0, semi_angle: tan.atan() }, false))
            }
            "sphere-surface" => {
                let (c, rad, u, pole) = (self.pt(r, 4)?, self.num(r, 7)? * self.units, self.vec(r, 8)?, self.vec(r, 11)?);
                if rad.abs() < 1e-12 || pole.len() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "degenerate sphere"));
                }
                Ok((Surface::Sphere { frame: frame_from(c, pole, u), r: rad.abs() }, rad < 0.0))
            }
            "torus-surface" => {
                let (c, n) = (self.pt(r, 4)?, self.vec(r, 7)?);
                let (major, minor) = (self.num(r, 10)? * self.units, self.num(r, 11)? * self.units);
                let u = self.vec(r, 12)?;
                if n.len() < 1e-12 || !(major > 0.0) || minor.abs() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "degenerate torus"));
                }
                if minor.abs() >= major {
                    return Err(SatError::at("sat.unsupported", r, "self-intersecting (apple/lemon) torus"));
                }
                Ok((Surface::Torus { frame: frame_from(c, n, u), major, minor: minor.abs() }, minor < 0.0))
            }
            "spline-surface" => {
                let rev = match self.tok(r, 4)? {
                    "forward" => false,
                    "reversed" => true,
                    o => return Err(SatError::at("sat.parse", r, format!("bad spline-surface sense '{o}'"))),
                };
                Ok((Surface::BSpline(Box::new(self.spline_surface(r)?)), rev))
            }
            other => Err(SatError::at("sat.unsupported", r, format!("surface type '{other}'"))),
        }
    }

    // ------------------------------------------------------------ faces
    fn ce_points(&self, ce: &Ce, n: usize) -> Vec<Vec3> {
        let e = &self.body.edges[ce.edge.idx()];
        let (a, b) = if ce.rev { (e.t1, e.t0) } else { (e.t0, e.t1) };
        (0..=n).map(|i| e.curve.eval(a + (b - a) * i as f64 / n as f64)).collect()
    }

    fn ce_start(&self, ce: &Ce) -> VertexId {
        let e = &self.body.edges[ce.edge.idx()];
        if ce.rev {
            e.v1
        } else {
            e.v0
        }
    }

    fn face(&mut self, shell: ShellId, fr: &'a Rec, sheet: bool) -> R<()> {
        let sr = self.rec(fr, self.ptr_req(fr, 8)?)?;
        let sense_rev = match self.tok(fr, 9)? {
            "forward" => false,
            "reversed" => true,
            o => return Err(SatError::at("sat.parse", fr, format!("bad face sense '{o}'"))),
        };
        // single-sided with containment 'out': the back of the face is outside
        // the material, i.e. the stored normal points into the material.
        let cont_flip = !sheet && fr.t.get(11).map(|s| s == "out").unwrap_or(false);

        // loops
        let mut loops: Vec<Vec<Ce>> = vec![];
        let mut pts: Vec<Vec3> = vec![];
        for lr in self.chain(self.ptr(fr, 5)?, 4, "loop")? {
            let mut ring = vec![];
            let mut seen = HashSet::new();
            let start = self.ptr_req(lr, 5)?;
            let mut cur = start;
            loop {
                let cr = self.typed(lr, cur, &["coedge"])?;
                if !seen.insert(cur) {
                    break;
                }
                let sense = match self.tok(cr, 8)? {
                    "forward" => false,
                    "reversed" => true,
                    o => return Err(SatError::at("sat.parse", cr, format!("bad coedge sense '{o}'"))),
                };
                let (id, flip) = self.edge(cr, self.ptr_req(cr, 7)?)?;
                ring.push(Ce { edge: id, rev: sense ^ flip });
                let e = &self.body.edges[id.idx()];
                pts.push(self.body.vertices[e.v0.idx()].p);
                pts.push(self.body.vertices[e.v1.idx()].p);
                cur = self.ptr_req(cr, 4)?;
                if cur == start {
                    break;
                }
            }
            if ring.is_empty() {
                return Err(SatError::at("sat.topology", lr, "empty loop"));
            }
            loops.push(ring);
        }

        let (surface, sflip) = self.surface(sr, &pts)?;
        let sense = !(sense_rev ^ sflip ^ cont_flip);

        // every vertex must lie on the surface
        for p in &pts {
            let d = surface.distance(*p);
            if d > 10.0 * geo_tol(*p) {
                return Err(SatError::at("sat.geometry", fr, format!("vertex ({:.6}, {:.6}, {:.6}) is {d:.3e} off its {} surface ${}", p.x, p.y, p.z, surface.kind(), sr.idx)));
            }
        }
        if loops.is_empty() && !matches!(surface, Surface::Sphere { .. } | Surface::Torus { .. }) {
            return Err(SatError::at("sat.topology", fr, "face without a loop on a non-closed surface"));
        }

        let loops = self.seam_wrapping(fr, &surface, loops)?;
        let loops = self.order_planar(fr, &surface, sense, loops)?;

        let name = FaceName::other("sat", &format!("F{}", fr.idx));
        let f = self.body.add_face(shell, surface.clone(), sense, name);
        let (pu, pv) = surface.periods();
        for ring in loops {
            let lp = self.body.add_loop(f);
            // pcurves (continuity-aligned across periodic seams)
            let mut prev_end: Option<Vec2> = None;
            for ce in ring {
                let e = &self.body.edges[ce.edge.idx()];
                let (t0, t1, curve) = (e.t0, e.t1, e.curve.clone());
                let mut pc = pcurve_of(&surface, &curve, t0, t1, 1e-6).ok();
                let (ts, te) = if ce.rev { (t1, t0) } else { (t0, t1) };
                if let Some(c) = pc.as_mut() {
                    if let Some(pe) = prev_end {
                        let s = c.eval(ts);
                        let du = pu.map_or(0.0, |p| ((pe.x - s.x) / p).round() * p);
                        let dv = pv.map_or(0.0, |p| ((pe.y - s.y) / p).round() * p);
                        if du != 0.0 || dv != 0.0 {
                            *c = shift_pc(c, du, dv);
                        }
                    }
                    prev_end = Some(c.eval(te));
                }
                self.body.add_coedge(lp, ce.edge, ce.rev, pc);
            }
        }
        Ok(())
    }

    /// Total u-winding (in turns) of each loop on a u-periodic surface.
    fn windings(&self, surface: &Surface, loops: &[Vec<Ce>]) -> Vec<i32> {
        let frame = match surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } | Surface::Sphere { frame, .. } | Surface::Torus { frame, .. } => frame,
            _ => return vec![0; loops.len()],
        };
        loops
            .iter()
            .map(|ring| {
                let mut total = 0.0;
                let mut last: Option<f64> = None;
                for ce in ring {
                    for p in self.ce_points(ce, 16) {
                        let d = p - frame.o;
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

    /// ACIS periodic faces have no seam edge: the two boundary loops of a
    /// cylinder / cone each wrap the axis. Join them with a seam line.
    fn seam_wrapping(&mut self, fr: &Rec, surface: &Surface, mut loops: Vec<Vec<Ce>>) -> R<Vec<Vec<Ce>>> {
        if surface.periods().0.is_none() {
            return Ok(loops);
        }
        let w = self.windings(surface, &loops);
        let wrap: Vec<usize> = (0..loops.len()).filter(|&i| w[i] != 0).collect();
        if wrap.is_empty() {
            return Ok(loops);
        }
        let frame = match surface {
            Surface::Cylinder { frame, .. } | Surface::Cone { frame, .. } => frame.clone(),
            _ => return Err(SatError::at("sat.unsupported", fr, format!("{} face with a loop wrapping its axis (needs poles/seams)", surface.kind()))),
        };
        if wrap.len() != 2 || w[wrap[0]].abs() != 1 || w[wrap[1]].abs() != 1 {
            return Err(SatError::at("sat.unsupported", fr, format!("{} face with {} wrapping loops (a seam needs exactly two)", surface.kind(), wrap.len())));
        }
        if w[wrap[0]] == w[wrap[1]] {
            return Err(SatError::at("sat.orientation", fr, "both wrapping loops run the same way round the axis"));
        }
        let ang = |p: Vec3| {
            let d = p - frame.o;
            d.dot(frame.y).atan2(d.dot(frame.x))
        };
        let a = loops[wrap[0]].clone();
        let mut bl = loops[wrap[1]].clone();
        let va = self.ce_start(&a[0]);
        let pa = self.body.vertices[va.idx()].p;
        let ua = ang(pa);
        let k = (0..bl.len())
            .find(|&k| {
                let p = self.body.vertices[self.ce_start(&bl[k]).idx()].p;
                let mut du = (ang(p) - ua).abs() % TAU;
                if du > PI {
                    du = TAU - du;
                }
                du < 1e-6
            })
            .ok_or_else(|| SatError::at("sat.unsupported", fr, "periodic face whose two loops have no vertices at a common angle (seam needs an edge split)"))?;
        bl.rotate_left(k);
        let vb = self.ce_start(&bl[0]);
        let pb = self.body.vertices[vb.idx()].p;
        let curve = Curve::line_through(pa, pb);
        let len = pa.dist(pb);
        if !(len > 1e-9) {
            return Err(SatError::at("sat.geometry", fr, "zero-length seam"));
        }
        let seam = self.body.add_edge(curve, 0.0, len, va, vb);
        let mut joined = a;
        joined.push(Ce { edge: seam, rev: false });
        joined.extend(bl);
        joined.push(Ce { edge: seam, rev: true });
        let mut out = vec![joined];
        for (i, l) in loops.drain(..).enumerate() {
            if i != wrap[0] && i != wrap[1] {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// Planar faces: outer loop first (positive area about the outward
    /// normal), holes after; a negative outer area is an orientation error.
    fn order_planar(&self, fr: &Rec, surface: &Surface, sense: bool, loops: Vec<Vec<Ce>>) -> R<Vec<Vec<Ce>>> {
        let Surface::Plane { frame } = surface else { return Ok(loops) };
        let n = if sense { frame.z } else { -frame.z };
        let mut areas: Vec<(f64, Vec<Ce>)> = loops
            .into_iter()
            .map(|ring| {
                let mut poly: Vec<Vec3> = vec![];
                for ce in &ring {
                    let mut p = self.ce_points(ce, 16);
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
                return Err(SatError::at("sat.orientation", fr, "planar face loop runs clockwise about the face's outward normal"));
            }
        }
        Ok(areas.into_iter().map(|(_, r)| r).collect())
    }
}

fn shift_pc(pc: &Curve2, du: f64, dv: f64) -> Curve2 {
    let d = Vec2::new(du, dv);
    match pc {
        Curve2::Line { o, d: dd } => Curve2::Line { o: *o + d, d: *dd },
        Curve2::Circle { c, x, r, ccw } => Curve2::Circle { c: *c + d, x: *x, r: *r, ccw: *ccw },
        Curve2::Ellipse { c, x, a, b, ccw } => Curve2::Ellipse { c: *c + d, x: *x, a: *a, b: *b, ccw: *ccw },
        Curve2::BSpline(b) => Curve2::BSpline(b.map(|p| p + d)),
    }
}
