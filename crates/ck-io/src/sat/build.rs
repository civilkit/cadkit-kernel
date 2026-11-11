//! SAT record table -> ck-topo [`Body`].

use super::parse::{Rec, Table};
use super::{SatError, SatImport, SatSummary};
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
    /// Coedge leaving an (implicit) pole: its pcurve is aligned by its END to
    /// the loop's start instead of by its start to the previous end, so the
    /// loop's u-winding is kept (the pole's v = const edge adds nothing).
    pole_jump: bool,
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

/// Body transform: `p' = (p . M) * s + t` (ACIS row-vector convention).
#[derive(Clone, Copy)]
struct Xf {
    m: [[f64; 3]; 3],
    s: f64,
    t: Vec3,
}

impl Xf {
    fn identity() -> Xf {
        Xf { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], s: 1.0, t: Vec3::ZERO }
    }
    /// Row-vector times the linear part (rotation only).
    fn dir(&self, v: Vec3) -> Vec3 {
        let m = &self.m;
        Vec3::new(
            v.x * m[0][0] + v.y * m[1][0] + v.z * m[2][0],
            v.x * m[0][1] + v.y * m[1][1] + v.z * m[2][1],
            v.x * m[0][2] + v.y * m[1][2] + v.z * m[2][2],
        )
    }
}

struct B<'a> {
    t: &'a Table,
    body: Body,
    verts: HashMap<usize, (VertexId, Vec3)>,
    edges: HashMap<usize, EInfo>,
    units: f64,
    xf: Xf,
}

/// Record types the importer reads (everything else is skipped and counted).
const READ_TYPES: &[&str] = &[
    "body", "lump", "shell", "face", "loop", "coedge", "edge", "vertex", "point", "transform", "plane-surface", "cone-surface", "sphere-surface", "torus-surface", "spline-surface",
    "straight-curve", "ellipse-curve", "intcurve-curve",
];

pub fn build(t: &Table) -> R<SatImport> {
    let mut b = B { t, body: Body::new(), verts: HashMap::new(), edges: HashMap::new(), units: t.units, xf: Xf::identity() };
    let bodies: Vec<&Rec> = t.recs.iter().filter(|r| r.ty() == "body").collect();
    if bodies.is_empty() {
        return Err(SatError { code: "sat.topology", entity: None, kind: String::new(), line: 0, detail: "no body record".into() });
    }
    // phase 1: lumps / shells / faces, and the single/double-sided census
    let mut plan: Vec<(Xf, Vec<Vec<&Rec>>)> = vec![];
    let (mut single, mut double) = (None::<&Rec>, None::<&Rec>);
    for br in &bodies {
        let xf = b.body_transform(br)?;
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
        plan.extend(lumps.into_iter().map(|l| (xf, l)));
    }
    if let (Some(s), Some(_)) = (single, double) {
        return Err(SatError::at("sat.unsupported", s, "mixed single- and double-sided faces (solid with internal sheets)"));
    }
    let sheet = double.is_some();
    // phase 2: build
    for (xf, shells) in &plan {
        b.xf = *xf;
        let lump = b.body.add_lump();
        for faces in shells {
            let sh = b.body.add_shell(lump);
            for f in faces {
                b.face(sh, f, sheet)?;
            }
        }
    }
    let nfaces = b.body.num_faces();
    if nfaces == 0 {
        return Err(SatError {
            code: "sat.empty",
            entity: None,
            kind: String::new(),
            line: 0,
            detail: format!("file has {} body record(s) but no faces (wire/point-only or empty bodies)", bodies.len()),
        });
    }
    let mut skipped: std::collections::BTreeMap<String, usize> = Default::default();
    for r in &t.recs {
        if !READ_TYPES.contains(&r.ty()) {
            *skipped.entry(r.ty().to_string()).or_default() += 1;
        }
    }
    let summary = SatSummary {
        version: t.version,
        records: t.recs.len(),
        bodies: bodies.len(),
        lumps: plan.len(),
        shells: plan.iter().map(|p| p.1.len()).sum(),
        faces: nfaces,
        skipped: skipped.into_iter().collect(),
    };
    Ok(SatImport { body: b.body, sheet, units: t.units, summary })
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
    /// Raw 3-vector (file frame, no transform, no unit scaling).
    fn raw(&self, r: &Rec, k: usize) -> R<Vec3> {
        Ok(Vec3::new(self.num(r, k)?, self.num(r, k + 1)?, self.num(r, k + 2)?))
    }
    /// Direction vector: body-transform rotation applied, no unit scaling.
    fn vec(&self, r: &Rec, k: usize) -> R<Vec3> {
        Ok(self.xf.dir(self.raw(r, k)?))
    }
    /// Point in mm: body transform (rotate, scale, translate), then units.
    fn pt(&self, r: &Rec, k: usize) -> R<Vec3> {
        Ok(self.xp(self.raw(r, k)?))
    }
    /// File-frame point -> mm.
    fn xp(&self, p: Vec3) -> Vec3 {
        (self.xf.dir(p) * self.xf.s + self.xf.t) * self.units
    }
    /// File length -> mm (units x body-transform scale).
    fn ls(&self) -> f64 {
        self.units * self.xf.s
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

    /// Body transform: rotation (3 rows), translation, uniform scale, then
    /// `rotate|no_rotate reflect|no_reflect shear|no_shear`. Reflections and
    /// shears / non-uniform scale are rejected.
    fn body_transform(&self, br: &Rec) -> R<Xf> {
        let Some(ti) = self.ptr(br, 6).ok().flatten() else { return Ok(Xf::identity()) };
        let tr = self.typed(br, ti, &["transform"])?;
        let flag = tr
            .t
            .iter()
            .position(|s| s == "rotate" || s == "no_rotate")
            .filter(|&k| k >= 14)
            .ok_or_else(|| SatError::at("sat.parse", tr, "transform without rotate flag"))?;
        let mut v = [0.0f64; 13];
        for (i, x) in v.iter_mut().enumerate() {
            *x = self.num(tr, flag - 13 + i)?;
        }
        let has = |w: &str| tr.t.iter().any(|s| s == w);
        if has("reflect") {
            return Err(SatError::at("sat.unsupported", tr, "reflecting body transform"));
        }
        let m = [[v[0], v[1], v[2]], [v[3], v[4], v[5]], [v[6], v[7], v[8]]];
        for i in 0..3 {
            for j in 0..3 {
                let d: f64 = (0..3).map(|k| m[i][k] * m[j][k]).sum();
                if (d - if i == j { 1.0 } else { 0.0 }).abs() > 1e-6 {
                    return Err(SatError::at("sat.unsupported", tr, "body transform with shear / non-uniform scale in its matrix"));
                }
            }
        }
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
        if det < 0.0 {
            return Err(SatError::at("sat.unsupported", tr, "reflecting body transform"));
        }
        let s = v[12];
        if !(s > 0.0) || !s.is_finite() {
            return Err(SatError::at("sat.parse", tr, "non-positive transform scale"));
        }
        Ok(Xf { m, s, t: Vec3::new(v[9], v[10], v[11]) })
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
                Ok(CurveG { curve: Curve::line(o, d), scale: l * self.ls(), offset: 0.0 })
            }
            "ellipse-curve" => {
                let (c, n, m) = (self.pt(r, 4)?, self.vec(r, 7)?, self.vec(r, 10)?);
                let ratio = self.num(r, 13)?;
                let a = m.len() * self.ls();
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
        // `{ ref N }`: shared definition, the N-th `{` block of the file
        let (dr, mut k) = if r.t.get(open + 1).map(|s| s.as_str()) == Some("ref") {
            let n: usize = r.t.get(open + 2).and_then(|s| s.parse().ok()).ok_or_else(|| SatError::at("sat.parse", r, "bad spline ref index"))?;
            let &(ri, ki) = self.t.defs.get(n).ok_or_else(|| SatError::at("sat.topology", r, format!("spline ref {n} has no definition")))?;
            let d: &'a Rec = &self.t.recs[ri];
            if d.t.get(ki + 1).map(|s| s.as_str()) == Some("spline") {
                return Err(SatError::at("sat.unsupported", r, format!("spline ref {n} resolves to a non-exact definition")));
            }
            (d, ki + 1)
        } else {
            (r, open + 1)
        };
        while let Some(t) = dr.t.get(k) {
            match t.as_str() {
                "exactcur" | "exactsur" | "full" => k += 1,
                "nubs" | "nurbs" => break,
                other => return Err(SatError::at("sat.unsupported", r, format!("spline definition '{other}' (only exact nubs/nurbs)"))),
            }
        }
        Ok(Cur { r: dr, k })
    }

    /// Distinct knots + multiplicities -> full clamped knot vector and the
    /// control-point count. ACIS stores end multiplicity = degree (one knot
    /// less than the textbook form at each end); degree+1 is accepted too.
    fn knots(&self, c: &mut Cur, deg: usize, n: usize) -> R<(Vec<f64>, usize)> {
        if n == 0 || n > 1_000_000 {
            return Err(SatError::at("sat.parse", c.r, "bad knot count"));
        }
        // same limit as the STEP reader; keeps `deg + 1` and the knot vector from overflowing
        if deg > 1024 {
            return Err(SatError::at("sat.parse", c.r, "bad spline degree"));
        }
        let mut vals = vec![];
        let mut mult = vec![];
        let mut total = 0usize;
        for _ in 0..n {
            vals.push(c.f()?);
            let m = c.u()?;
            if m > deg + 1 {
                return Err(SatError::at("sat.parse", c.r, "knot multiplicity above degree + 1"));
            }
            total += m;
            if total > 10_000_000 {
                return Err(SatError::at("sat.parse", c.r, "absurd knot vector length"));
            }
            mult.push(m);
        }
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
            ctrl.push(self.xp(Vec3::new(c.f()?, c.f()?, c.f()?)));
            if rational {
                w.push(c.f()?);
            }
        }
        let res = if rational { BSplineCurve::new_rational(deg, ks, ctrl, w) } else { BSplineCurve::new(deg, ks, ctrl) };
        res.map_err(|e| SatError::at("sat.geometry", r, format!("bad spline curve: {e}")))
    }

    fn spline_surface(&self, r: &'a Rec) -> R<(BSplineSurface, bool)> {
        let mut c = self.spline_cursor(r)?;
        let rational = c.tok()? == "nurbs";
        let (du, dv) = (c.u()?, c.u()?);
        // forms (open|closed|periodic) and singularity flags (none|low|high|both);
        // newer ACIS writes one extra leading flag, so read the whole run.
        let mut forms = 0;
        while let Some(t) = c.r.t.get(c.k) {
            match t.as_str() {
                "open" | "closed" | "periodic" => {
                    self.form(&mut c)?;
                    forms += 1;
                }
                "none" | "low" | "high" | "both" => c.k += 1,
                _ => break,
            }
        }
        if forms != 2 {
            return Err(SatError::at("sat.parse", r, format!("expected two spline forms, found {forms}")));
        }
        let (nku, nkv) = (c.u()?, c.u()?);
        let (ku, ncu) = self.knots(&mut c, du, nku)?;
        let (kv, ncv) = self.knots(&mut c, dv, nkv)?;
        if ncu.saturating_mul(ncv) > 10_000_000 {
            return Err(SatError::at("sat.parse", r, "absurd control net size"));
        }
        // File order: u varies fastest (P[v][u]); ours is ctrl[i * nv + j].
        let mut ctrl = vec![Vec3::ZERO; ncu * ncv];
        let mut w = vec![1.0; ncu * ncv];
        for j in 0..ncv {
            for i in 0..ncu {
                ctrl[i * ncv + j] = self.xp(Vec3::new(c.f()?, c.f()?, c.f()?));
                if rational {
                    w[i * ncv + j] = c.f()?;
                }
            }
        }
        // Long-in-v nets (thread / screw flanks): swap u and v so the long
        // direction is u, the layout ck-topo's mass integrator and the
        // sweep surfaces are optimised for (22 s -> ms per face). The swap
        // reverses the natural normal; the caller flips the face sense.
        let wopt = |w: Vec<f64>| if rational { Some(w) } else { None };
        if nkv >= 32 && nku < 32 {
            let mut t_ctrl = vec![Vec3::ZERO; ncu * ncv];
            let mut t_w = vec![1.0; ncu * ncv];
            for i in 0..ncu {
                for j in 0..ncv {
                    t_ctrl[j * ncu + i] = ctrl[i * ncv + j];
                    t_w[j * ncu + i] = w[i * ncv + j];
                }
            }
            return BSplineSurface::new(dv, du, kv, ku, ncv, ncu, t_ctrl, wopt(t_w))
                .map(|s| (s, true))
                .map_err(|e| SatError::at("sat.geometry", r, format!("bad spline surface: {e}")));
        }
        BSplineSurface::new(du, dv, ku, kv, ncu, ncv, ctrl, wopt(w))
            .map(|s| (s, false))
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
                let r0 = m.len() * self.ls();
                if ax.len() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "zero cone axis"));
                }
                // Real ACIS: `ratio <u-range: 2 bounds> sin cos scale sense ...`.
                // (Hand-written fixtures put the sense before the range and
                // have no scale; both are accepted.)
                let sense_first = matches!(self.tok(r, 14)?, "forward" | "reversed");
                let mut c = Cur { r, k: if sense_first { 15 } else { 14 } };
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
                let cone_rev = if sense_first { self.tok(r, 14)? == "reversed" } else { r.t.iter().skip(c.k).any(|t| t == "reversed") };
                if (s * s + co * co - 1.0).abs() > 1e-6 || co <= 0.0 {
                    return Err(SatError::at("sat.parse", r, "cone sine/cosine are not a half-angle"));
                }
                let frame = frame_from(o, ax, m);
                if r0 < 1e-12 && s.abs() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "zero-radius cylinder"));
                }
                if s.abs() < 1e-12 {
                    return Ok((Surface::Cylinder { frame, r: r0 }, cone_rev));
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
                Ok((Surface::Cone { frame, r: r0, semi_angle: tan.atan() }, cone_rev))
            }
            "sphere-surface" => {
                let (c, rad, u, pole) = (self.pt(r, 4)?, self.num(r, 7)? * self.ls(), self.vec(r, 8)?, self.vec(r, 11)?);
                if rad.abs() < 1e-12 || pole.len() < 1e-12 {
                    return Err(SatError::at("sat.geometry", r, "degenerate sphere"));
                }
                Ok((Surface::Sphere { frame: frame_from(c, pole, u), r: rad.abs() }, rad < 0.0))
            }
            "torus-surface" => {
                let (c, n) = (self.pt(r, 4)?, self.vec(r, 7)?);
                let (major, minor) = (self.num(r, 10)? * self.ls(), self.num(r, 11)? * self.ls());
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
                let (s, transposed) = self.spline_surface(r)?;
                Ok((Surface::BSpline(Box::new(s)), rev ^ transposed))
            }
            other => Err(SatError::at("sat.unsupported", r, format!("surface type '{other}'"))),
        }
    }

    // ------------------------------------------------------------ faces
    /// Polyline resolution for winding / area estimates: splines may spiral
    /// many turns (threads), so sample them densely.
    fn samples(&self, ce: &Ce) -> usize {
        match &self.body.edges[ce.edge.idx()].curve {
            Curve::BSpline(_) => 1024,
            _ => 16,
        }
    }

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
        let cont_flip = !sheet && fr.t.get(11..).map(|s| s.iter().any(|t| t == "out")).unwrap_or(false);

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
                ring.push(Ce { edge: id, rev: sense ^ flip, pole_jump: false });
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

        let loops = self.seam_wrapping(fr, &surface, sense, loops)?;
        let loops = self.order_planar(fr, &surface, sense, loops)?;

        let name = FaceName::other("sat", &format!("F{}", fr.idx));
        let f = self.body.add_face(shell, surface.clone(), sense, name);
        let (pu, pv) = surface.periods();
        for ring in loops {
            let lp = self.body.add_loop(f);
            // pcurves (continuity-aligned across periodic seams)
            let mut prev_end: Option<Vec2> = None;
            let mut loop_start: Option<Vec2> = None;
            for ce in ring {
                let e = &self.body.edges[ce.edge.idx()];
                let (t0, t1, curve) = (e.t0, e.t1, e.curve.clone());
                let mut pc = pcurve_of(&surface, &curve, t0, t1, 1e-6).ok();
                let (ts, te) = if ce.rev { (t1, t0) } else { (t0, t1) };
                if let Some(c) = pc.as_mut() {
                    if ce.pole_jump {
                        if let Some(ls) = loop_start {
                            let s = c.eval(te);
                            let du = pu.map_or(0.0, |p| ((ls.x - s.x) / p).round() * p);
                            if du != 0.0 {
                                *c = shift_pc(c, du, 0.0);
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
                    if loop_start.is_none() {
                        loop_start = Some(c.eval(ts));
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
                    for p in self.ce_points(ce, self.samples(ce)) {
                        let d = p - frame.o;
                        if d.dot(frame.x).hypot(d.dot(frame.y)) < 1e-9 * (1.0 + d.len()) {
                            continue; // on the pole axis: azimuth undefined
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

    /// ACIS periodic faces have no seam edge: the two boundary loops of a
    /// cylinder / cone each wrap the axis. Join them with a seam line.
    fn seam_wrapping(&mut self, fr: &Rec, surface: &Surface, sense: bool, mut loops: Vec<Vec<Ce>>) -> R<Vec<Vec<Ce>>> {
        if surface.periods().0.is_none() {
            return Ok(loops);
        }
        let w = self.windings(surface, &loops);
        let wrap: Vec<usize> = (0..loops.len()).filter(|&i| w[i] != 0).collect();
        if wrap.is_empty() {
            return Ok(loops);
        }
        if let Surface::Sphere { frame, r } = surface {
            return self.sphere_seam(fr, surface, sense, loops, &wrap, &w);
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
        joined.push(Ce { edge: seam, rev: false, pole_jump: false });
        joined.extend(bl);
        joined.push(Ce { edge: seam, rev: true, pole_jump: false });
        let mut out = vec![joined];
        for (i, l) in loops.drain(..).enumerate() {
            if i != wrap[0] && i != wrap[1] {
                out.push(l);
            }
        }
        Ok(out)
    }

    /// Sphere faces whose boundary circles wrap the pole axis: a cap (one
    /// wrapping loop; a meridian seam runs from the loop to the pole and
    /// back, the pole itself stays implicit as in a revolved sphere) or a
    /// band (two wrapping loops joined by a meridian arc).
    fn sphere_seam(&mut self, fr: &Rec, surface: &Surface, sense: bool, loops: Vec<Vec<Ce>>, wrap: &[usize], w: &[i32]) -> R<Vec<Vec<Ce>>> {
        let Surface::Sphere { frame, r } = surface else { unreachable!() };
        let (c, axis) = (frame.o, frame.z);
        if wrap.iter().any(|&i| w[i].abs() != 1) || wrap.len() > 2 {
            return Err(SatError::at("sat.unsupported", fr, format!("sphere face with {} wrapping loop(s) of winding {:?}", wrap.len(), wrap.iter().map(|&i| w[i]).collect::<Vec<_>>())));
        }
        let azimuth = |p: Vec3| -> R<Vec3> {
            let d = p - c;
            let h = d - axis * d.dot(axis);
            if h.len() < 1e-9 * r.max(1.0) {
                return Err(SatError::at("sat.geometry", fr, "sphere loop vertex sits on the pole axis"));
            }
            Ok(h.normalize())
        };
        let mut out: Vec<Vec<Ce>> = vec![];
        if wrap.len() == 1 {
            let ring = loops[wrap[0]].clone();
            let va = self.ce_start(&ring[0]);
            let pa = self.body.vertices[va.idx()].p;
            let pts = self.ce_points(&ring[0], 8);
            let tan = pts[1] - pts[0];
            let nrm = if sense { (pa - c).normalize() } else { -(pa - c).normalize() };
            let north = nrm.cross(tan).dot(axis) > 0.0;
            let xdir = if north { axis } else { -axis };
            let e = azimuth(pa)?;
            let t0 = (pa - c).dot(e).atan2((pa - c).dot(xdir));
            if !(t0 > 1e-9) {
                return Err(SatError::at("sat.geometry", fr, "sphere cap loop lies on the pole"));
            }
            let pole = self.body.add_vertex(c + xdir * *r);
            let seam = self.body.add_edge(Curve::circle_x(c, xdir.cross(e), xdir, *r), 0.0, t0, pole, va);
            let mut joined = ring;
            joined.push(Ce { edge: seam, rev: true, pole_jump: false });
            joined.push(Ce { edge: seam, rev: false, pole_jump: true });
            out.push(joined);
        } else {
            if w[wrap[0]] == w[wrap[1]] {
                return Err(SatError::at("sat.orientation", fr, "both wrapping loops run the same way round the axis"));
            }
            let a = loops[wrap[0]].clone();
            let mut bl = loops[wrap[1]].clone();
            let va = self.ce_start(&a[0]);
            let pa = self.body.vertices[va.idx()].p;
            let ea = azimuth(pa)?;
            let mut k = None;
            for i in 0..bl.len() {
                let p = self.body.vertices[self.ce_start(&bl[i]).idx()].p;
                if azimuth(p)?.dist(ea) < 1e-6 {
                    k = Some(i);
                    break;
                }
            }
            let k = k.ok_or_else(|| SatError::at("sat.unsupported", fr, "sphere band whose two loops have no vertices at a common azimuth (seam needs an edge split)"))?;
            bl.rotate_left(k);
            let vb = self.ce_start(&bl[0]);
            let pb = self.body.vertices[vb.idx()].p;
            let ang = |p: Vec3| (p - c).dot(ea).atan2((p - c).dot(axis));
            let (ta, tb) = (ang(pa), ang(pb));
            if (ta - tb).abs() < 1e-9 {
                return Err(SatError::at("sat.geometry", fr, "zero-length seam"));
            }
            let curve = Curve::circle_x(c, axis.cross(ea), axis, *r);
            let (lo, hi, vlo, vhi) = if ta < tb { (ta, tb, va, vb) } else { (tb, ta, vb, va) };
            let seam = self.body.add_edge(curve, lo, hi, vlo, vhi);
            let fwd_rev = ta > tb; // coedge va -> vb runs against the edge
            let mut joined = a;
            joined.push(Ce { edge: seam, rev: fwd_rev, pole_jump: false });
            joined.extend(bl);
            joined.push(Ce { edge: seam, rev: !fwd_rev, pole_jump: false });
            out.push(joined);
        }
        for (i, l) in loops.into_iter().enumerate() {
            if !wrap.contains(&i) {
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
