//! Rolling-ball fillets along chains of free-form edges (B-spline,
//! revolution ... faces; spline intersection edges).
//!
//! The ball centre is solved per station along the edge (Newton on the two
//! signed face distances and the plane through the edge point normal to the
//! edge tangent), giving contact curves `A` (on face 1) and `B` (face 2) and
//! the circular cross-section between them. The blend is built as a
//! boolean TOOL like the analytic fillets in `build.rs`: a closed shell
//! swept along the chain with the section polygon
//!
//! ```text
//!   A --arc--> B --> B2 --> Q --> A2 --> A
//! ```
//!
//! (`B2`/`A2` leave the faces along their normals at the contact curves, `Q`
//! is the polygon corner beyond the edge), concave edges UNION the tool
//! (legs run into the material), convex ones CUT it (legs run out of the
//! body). The blend surface and contact legs are tensor-product cubic
//! B-spline fits through the stations that share one parameterisation, so
//! the tool's shared edges coincide exactly. The tool's blend patch is
//! tangent to the two faces along `A`/`B`, which surface/surface
//! intersection cannot resolve, so the imprint is told those curves
//! directly ([`crate::boolean::imprint::PairHook`]).
//!
//! Chain ends: a convex chain's tool is extended past the end by a margin
//! (it pierces the neighbouring face); a concave chain ends flush on a planar
//! neighbour (the end section is projected onto that plane). One G1 chain is
//! one blend face `B<k>` (k = first selected edge of the chain).
//! Closed rings and branching selections are `blendUnsupported`.

use super::build::{apply_tool_hooked, find_edge};
use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::fit::{averaged_knots, interpolate_with};
use ck_geom::nurbs::bspline::BSplineCurve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::Vec3;
use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName};

/// Arc samples per cross-section.
const NV: usize = 9;
/// Section track count: arc samples + B2, Q, A2.
const NT: usize = NV + 3;
/// Welding tolerance of the imprint: the trimmed faces' boundary edges are
/// only accurate to about this much against their surfaces.
const FF_TOL: f64 = 1e-3;
/// Contact-curve fit tolerance between stations.
const FIT_TOL: f64 = 1.2e-5;

fn dbg(m: &str) {
    if std::env::var_os("CK_BLEND_DEBUG").is_some() {
        #[cfg(not(target_arch = "wasm32"))]
        {
            thread_local!(static T0: std::time::Instant = std::time::Instant::now());
            eprintln!("freeform [{:.1}s]: {m}", T0.with(|t| t.elapsed().as_secs_f64()));
        }
        #[cfg(target_arch = "wasm32")]
        eprintln!("freeform: {m}");
    }
}

/// A free-form blend refusal: the message is the `detail` of the structured
/// error (`blendUnsupported` / `blendFailed` carry `params.detail`), so a
/// rejected edit says which chain / face pair / end condition failed.
fn unsupported(m: &str) -> BlendError {
    dbg(m);
    BlendError::detail("blendUnsupported", m)
}

fn failed(m: &str) -> BlendError {
    dbg(m);
    BlendError::detail("blendFailed", m)
}

/// Does the (face, face, edge) combination fall outside the analytic fillet
/// builder (so the free-form path must handle it)?
pub fn needs_freeform(b: &Body, e: EdgeId) -> bool {
    let ed = b.edge(e);
    if !matches!(ed.curve, Curve::Line { .. } | Curve::Circle { .. }) {
        return true;
    }
    // a straight edge needs a plane on both sides for the analytic extruded tool (a plane meeting a cylinder or cone along
    // a generator has no analytic section in the line builder)
    if matches!(ed.curve, Curve::Line { .. }) && b.edge_faces(e).iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
        return true;
    }
    b.edge_faces(e).iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. }))
}

#[derive(Clone)]
struct Link {
    e: EdgeId,
    /// Traversed v0 -> v1.
    fwd: bool,
    f1: FaceId,
    f2: FaceId,
}

#[derive(Clone)]
struct Station {
    p: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    /// Unit tangent along the chain.
    t: Vec3,
}

#[derive(Clone)]
struct Node {
    li: usize,
    s: f64,
    st: Station,
}

struct Ctx<'a> {
    b: &'a Body,
    links: Vec<Link>,
    /// Distinct faces on each side of the chain (the foot of the ball centre
    /// may lie on a neighbouring face of the same G1 skin).
    set1: Vec<FaceId>,
    set2: Vec<FaceId>,
    r: f64,
    /// +1 concave (ball outside the material), -1 convex.
    sg: f64,
}

fn foot(b: &Body, f: FaceId, q: Vec3) -> Option<(Vec3, Vec3)> {
    let sf = &b.face(f).surface;
    let (u, v, pt) = sf.closest_point(q);
    let n = b.outward_normal(f, u, v).try_normalize()?;
    Some((pt, n))
}

thread_local! {
    /// Last foot parameters per face (warm starts for the next station) and
    /// each face's bounding box (candidates farther than the ball radius
    /// from a face's box are skipped).
    static HINTS: std::cell::RefCell<std::collections::HashMap<u32, (f64, f64, Vec3)>> = Default::default();
    static BOXES: std::cell::RefCell<std::collections::HashMap<u32, crate::bvh::Aabb>> = Default::default();
    static REACH: std::cell::Cell<f64> = const { std::cell::Cell::new(1.0) };
}

fn foot_hint(b: &Body, f: FaceId, q: Vec3) -> Option<(Vec3, Vec3)> {
    let reach = REACH.with(|r| r.get());
    let bx = BOXES.with(|m| *m.borrow_mut().entry(f.idx() as u32).or_insert_with(|| crate::bvh::face_box(b, f)));
    if !bx.padded(2.5 * reach + 0.05 * bx.diagonal()).contains(q) {
        return None;
    }
    let sf = &b.face(f).surface;
    let h = HINTS.with(|m| m.borrow().get(&(f.idx() as u32)).copied());
    let (u, v, pt) = match h {
        Some((hu, hv, hq)) if hq.dist(q) < 6.0 * reach => sf.closest_point_near(q, (hu, hv)),
        _ => sf.closest_point(q),
    };
    HINTS.with(|m| m.borrow_mut().insert(f.idx() as u32, (u, v, pt)));
    let n = b.outward_normal(f, u, v).try_normalize()?;
    Some((pt, n))
}

fn foot_set(b: &Body, fs: &[FaceId], q: Vec3) -> Option<(Vec3, Vec3)> {
    let mut best: Option<(f64, Vec3, Vec3)> = None;
    for &f in fs {
        if let Some((pt, n)) = foot_hint(b, f, q) {
            let d = pt.dist(q);
            if best.as_ref().map_or(true, |x| d < x.0) {
                best = Some((d, pt, n));
            }
        }
    }
    best.map(|x| (x.1, x.2))
}

impl<'a> Ctx<'a> {
    /// Edge point and unit chain tangent at link parameter `s` in [0, 1].
    fn edge_at(&self, li: usize, s: f64) -> (Vec3, Vec3, Vec3) {
        let l = &self.links[li];
        let ed = self.b.edge(l.e);
        let (ta, tb) = if l.fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        let t = ta + (tb - ta) * s;
        let p = ed.curve.eval(t);
        let sign = if tb >= ta { 1.0 } else { -1.0 };
        let tan = ed.curve.tangent(t).try_normalize().unwrap_or(Vec3::X) * sign;
        // edge forward (v0 -> v1) direction
        let fw = if l.fwd { tan } else { -tan };
        (p, tan, fw)
    }

    /// Newton for the ball centre at `s` of link `li`; `guess` warm-starts.
    fn solve(&self, li: usize, s: f64, guess: Option<Vec3>) -> Result<Station, BlendError> {
        self.solve_with(li, s, guess, None)
    }

    /// As [`Self::solve`], with the centre constrained to the plane `(o, n)`
    /// instead of the plane normal to the edge (flush chain ends: the
    /// section then lies in the end face's plane).
    fn solve_with(&self, li: usize, s: f64, guess: Option<Vec3>, plane: Option<(Vec3, Vec3)>) -> Result<Station, BlendError> {
        let l = &self.links[li];
        let _ = l;
        let (p, t, _) = self.edge_at(li, s);
        let (row3, off3) = match plane {
            Some((o, n)) => (n, o),
            None => (t, p),
        };
        let r = self.r;
        let mut c = match guess {
            Some(c) => c,
            None => {
                let (_, n1) = foot_set(self.b, &self.set1, p).ok_or_else(|| failed("normal 1"))?;
                let (_, n2) = foot_set(self.b, &self.set2, p).ok_or_else(|| failed("normal 2"))?;
                let c12 = n1.dot(n2);
                p + (n1 + n2) * (self.sg * r / (1.0 + c12))
            }
        };
        for _ in 0..80 {
            let (q1, n1) = foot_set(self.b, &self.set1, c).ok_or_else(|| failed("foot 1"))?;
            let (q2, n2) = foot_set(self.b, &self.set2, c).ok_or_else(|| failed("foot 2"))?;
            let g = [(c - q1).dot(n1) - self.sg * r, (c - q2).dot(n2) - self.sg * r, (c - off3).dot(row3)];
            let err = g.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            if err < 1e-11 * (1.0 + r) {
                if std::env::var_os("CK_BLEND_DEBUG3").is_some() {
                    eprintln!("   foot residual: perp1 {:.2e} perp2 {:.2e}", (c - q1).cross(n1).len(), (c - q2).cross(n2).len());
                }
                return Ok(Station { p, a: q1, b: q2, c, t });
            }
            let dc = crate::intersect::solve3([n1, n2, row3], [-g[0], -g[1], -g[2]]).ok_or_else(|| unsupported("singular centre system (tangent faces)"))?;
            c = c + dc;
            if !c.is_finite() || dc.len() > 50.0 * r {
                return Err(failed("centre diverged"));
            }
        }
        Err(failed("centre did not converge"))
    }

    /// +1 when concave at `s` of link `li` (ball outside), -1 convex.
    fn sign_at(&self, li: usize, s: f64) -> Result<f64, BlendError> {
        let l = &self.links[li];
        let (p, _, fw) = self.edge_at(li, s);
        let (_, n1) = foot(self.b, l.f1, p).ok_or_else(|| failed("normal 1"))?;
        let (_, n2) = foot(self.b, l.f2, p).ok_or_else(|| failed("normal 2"))?;
        if (1.0 - n1.dot(n2).abs()) < 1e-9 {
            return Err(unsupported("tangent / folded faces"));
        }
        let co = self.b.edge_coedges(l.e).into_iter().find(|&c| self.b.coedge_face(c) == l.f1).ok_or_else(|| BlendError::code("missingEdge"))?;
        let tdir = if self.b.coedge(co).rev { -fw } else { fw };
        Ok(if n1.cross(tdir).dot(n2) < 0.0 { -1.0 } else { 1.0 })
    }
}

/// One cross-section: arc samples, then B2, Q, A2.
#[derive(Clone)]
struct Sec {
    pts: [Vec3; NT],
    /// d/dv of the arc (v in [0, 1]) at its two ends: the surface stays
    /// tangent to the faces along the contact curves.
    d0: Vec3,
    d1: Vec3,
}

fn section(st: &Station, sg: f64, r: f64, k: f64) -> Result<Sec, BlendError> {
    let a = st.a - st.c;
    let bb = st.b - st.c;
    let cth = (a.dot(bb) / (a.len() * bb.len())).clamp(-1.0, 1.0);
    let th = cth.acos();
    let sin = th.sin();
    if sin < 1e-3 {
        return Err(unsupported("blend arc degenerate (faces tangent or folded)"));
    }
    let mut pts = [Vec3::ZERO; NT];
    for j in 0..NV {
        let v = j as f64 / (NV - 1) as f64;
        pts[j] = st.c + (a * ((1.0 - v) * th).sin() + bb * (v * th).sin()) / sin;
    }
    pts[0] = st.a;
    pts[NV - 1] = st.b;
    let n1 = (st.c - st.a) * (sg / r);
    let n2 = (st.c - st.b) * (sg / r);
    let e2d = st.c + (a + bb) / (1.0 + cth);
    pts[NV] = st.b - n2 * (sg * k);
    pts[NV + 1] = e2d - (n1 + n2) * (sg * k);
    pts[NV + 2] = st.a - n1 * (sg * k);
    let d0 = (bb - a * cth) * (th / sin);
    let d1 = (bb * cth - a) * (th / sin);
    Ok(Sec { pts, d0, d1 })
}

/// Track order of the section polygon: A, B, B2, Q, A2 (column indices).
const TRACKS: [usize; 5] = [0, NV - 1, NV, NV + 1, NV + 2];

struct Fit {
    ku: Vec<f64>,
    u: Vec<f64>,
    cols: Vec<BSplineCurve>,
    /// Arc end-derivative fields (control points over the same knots).
    t0: BSplineCurve,
    t1: BSplineCurve,
}

fn fit_cols(secs: &[Sec], ps: &[Vec3]) -> Result<Fit, BlendError> {
    let n = secs.len();
    let mut u = vec![0.0; n];
    for i in 1..n {
        u[i] = u[i - 1] + ps[i].dist(ps[i - 1]).max(1e-9);
    }
    let tot = u[n - 1];
    for x in &mut u {
        *x /= tot;
    }
    u[n - 1] = 1.0;
    let ku = averaged_knots(&u, 3);
    let mut cols = vec![];
    for j in 0..NT {
        let pts: Vec<Vec3> = secs.iter().map(|s| s.pts[j]).collect();
        cols.push(interpolate_with(&pts, 3, &u, ku.clone()).map_err(|e| failed(&format!("fit: {e}")))?);
    }
    let t0 = interpolate_with(&secs.iter().map(|s| s.d0).collect::<Vec<_>>(), 3, &u, ku.clone()).map_err(|e| failed(&format!("fit: {e}")))?;
    let t1 = interpolate_with(&secs.iter().map(|s| s.d1).collect::<Vec<_>>(), 3, &u, ku.clone()).map_err(|e| failed(&format!("fit: {e}")))?;
    Ok(Fit { ku, u, cols, t0, t1 })
}

fn ruled(fit: &Fit, c0: usize, c1: usize) -> Result<BSplineSurface, BlendError> {
    let nu = fit.u.len();
    let mut ctrl = Vec::with_capacity(nu * 2);
    for i in 0..nu {
        ctrl.push(fit.cols[c0].ctrl[i]);
        ctrl.push(fit.cols[c1].ctrl[i]);
    }
    BSplineSurface::new(3, 1, fit.ku.clone(), vec![0.0, 0.0, 1.0, 1.0], nu, 2, ctrl, None).map_err(|e| failed(&format!("ruled: {e}")))
}

fn blend_patch(fit: &Fit) -> Result<BSplineSurface, BlendError> {
    let nu = fit.u.len();
    let mut ctrl = vec![];
    let mut kv = vec![];
    let mut nv = 0;
    for i in 0..nu {
        let row: Vec<Vec3> = (0..NV).map(|j| fit.cols[j].ctrl[i]).collect();
        // end derivatives pin the arc's tangent at both contacts (G1 with the faces)
        let c = ck_geom::fit::interpolate_cubic_tangents(&row, fit.t0.ctrl[i], fit.t1.ctrl[i], ck_geom::fit::Param::Uniform).map_err(|e| failed(&format!("blend fit: {e}")))?;
        if i == 0 {
            kv = c.knots.clone();
            nv = c.ctrl.len();
        }
        ctrl.extend(c.ctrl);
    }
    BSplineSurface::new(3, 3, fit.ku.clone(), kv, nu, nv, ctrl, None).map_err(|e| failed(&format!("blend: {e}")))
}

/// Chains of the selected edges (G1 by construction of the selection):
/// ordered links; rings and branches are unsupported.
fn build_chain(b: &Body, edges: &[EdgeId], start: usize) -> Result<Vec<(usize, bool)>, BlendError> {
    let ends = |i: usize| (b.edge(edges[i]).v0, b.edge(edges[i]).v1);
    let deg = |v: ck_topo::body::VertexId| edges.iter().enumerate().filter(|(i, _)| ends(*i).0 == v || ends(*i).1 == v).count();
    let mut chain = vec![(start, true)];
    let mut used = vec![false; edges.len()];
    used[start] = true;
    // forward
    let mut v = ends(start).1;
    loop {
        if deg(v) > 2 {
            return Err(unsupported("branching blend selection"));
        }
        let Some(nx) = (0..edges.len()).find(|&i| !used[i] && (ends(i).0 == v || ends(i).1 == v)) else { break };
        used[nx] = true;
        let fwd = ends(nx).0 == v;
        chain.push((nx, fwd));
        v = if fwd { ends(nx).1 } else { ends(nx).0 };
    }
    if v == ends(start).0 {
        return Err(unsupported("closed ring of free-form edges"));
    }
    // backward
    let mut v = ends(start).0;
    if deg(v) > 2 {
        return Err(unsupported("branching blend selection"));
    }
    loop {
        let Some(pv) = (0..edges.len()).find(|&i| !used[i] && (ends(i).0 == v || ends(i).1 == v)) else { break };
        used[pv] = true;
        // walking backwards: the link ends at v
        let fwd = ends(pv).1 == v;
        chain.insert(0, (pv, fwd));
        v = if fwd { ends(pv).0 } else { ends(pv).1 };
    }
    Ok(chain)
}

/// Fillet `edges` (free-form capable); see the module docs.
pub fn fillet_freeform(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    HINTS.with(|m| m.borrow_mut().clear());
    BOXES.with(|m| m.borrow_mut().clear());
    REACH.with(|c| c.set(r));
    let mut cur = b.clone();
    let mut remaining: Vec<usize> = (0..edges.len()).collect();
    while let Some(&first) = remaining.first() {
        let eids: Vec<EdgeId> = remaining
            .iter()
            .map(|&k| find_edge(&cur, &edges[k].0, &edges[k].1, edges[k].2).ok_or_else(|| BlendError::code("missingEdge")))
            .collect::<Result<_, _>>()?;
        let chain = build_chain(&cur, &eids, 0)?;
        let picked: Vec<usize> = chain.iter().map(|&(i, _)| remaining[i]).collect();
        let links = make_links(&cur, &eids, &chain)?;
        let k = *picked.iter().min().unwrap_or(&first) as u32;
        cur = blend_chain(&cur, links, r, feature, k, tol)?;
        remaining.retain(|x| !picked.contains(x));
    }
    Ok(cur)
}

fn make_links(b: &Body, eids: &[EdgeId], chain: &[(usize, bool)]) -> Result<Vec<Link>, BlendError> {
    let mut links: Vec<Link> = vec![];
    for &(i, fwd) in chain {
        let e = eids[i];
        let fs = b.edge_faces(e);
        if fs.len() != 2 {
            return Err(BlendError::code("blendNonManifold"));
        }
        let (mut f1, mut f2) = (fs[0], fs[1]);
        if let Some(prev) = links.last() {
            let pf: [FaceId; 2] = [prev.f1, prev.f2];
            let has = |f: FaceId| fs.contains(&f);
            if has(pf[0]) && has(pf[1]) {
                (f1, f2) = (pf[0], pf[1]);
            } else if has(pf[1]) {
                f2 = pf[1];
                f1 = if fs[0] == f2 { fs[1] } else { fs[0] };
            } else if has(pf[0]) {
                f1 = pf[0];
                f2 = if fs[0] == f1 { fs[1] } else { fs[0] };
            } else {
                return Err(unsupported("chain faces do not continue"));
            }
        }
        links.push(Link { e, fwd, f1, f2 });
    }
    Ok(links)
}

fn blend_chain(b: &Body, links: Vec<Link>, r: f64, feature: &str, k: u32, tol: f64) -> Result<Body, BlendError> {
    let mut set1: Vec<FaceId> = vec![];
    let mut set2: Vec<FaceId> = vec![];
    for l in &links {
        if !set1.contains(&l.f1) {
            set1.push(l.f1);
        }
        if !set2.contains(&l.f2) {
            set2.push(l.f2);
        }
    }
    let mut ctx = Ctx { b, links, set1, set2, r, sg: 1.0 };
    let sg = ctx.sign_at(0, 0.5)?;
    ctx.sg = sg;
    for li in 0..ctx.links.len() {
        for s in [0.0, 0.5, 1.0] {
            if ctx.sign_at(li, s)? != sg {
                return Err(unsupported("convex and concave stretches in one chain"));
            }
        }
    }
    let convex = sg < 0.0;
    let kk = 0.5 * r;
    // ---- stations (adaptive)
    let mut nodes: Vec<Node> = vec![];
    let mut prev: Option<Station> = None;
    for li in 0..ctx.links.len() {
        let len = b.edge_length(ctx.links[li].e);
        let n = ((len / (0.5 * r)).ceil() as usize).clamp(4, 400);
        for i in 0..=n {
            if li > 0 && i == 0 {
                continue;
            }
            let s = i as f64 / n as f64;
            let (p, _, _) = ctx.edge_at(li, s);
            let guess = prev.as_ref().map(|q| q.c + (p - q.p));
            let st = ctx.solve(li, s, guess)?;
            prev = Some(st.clone());
            nodes.push(Node { li, s, st });
        }
    }
    if std::env::var_os("CK_BLEND_DEBUG2").is_some() {
        for n in &nodes {
            let st = &n.st;
            eprintln!("  node l{} s{:.3} p({:.3},{:.3},{:.3}) a({:.3},{:.3},{:.3}) b({:.3},{:.3},{:.3}) c({:.3},{:.3},{:.3})", n.li, n.s, st.p.x, st.p.y, st.p.z, st.a.x, st.a.y, st.a.z, st.b.x, st.b.y, st.b.z, st.c.x, st.c.y, st.c.z);
        }
    }
    if std::env::var("CK_FF_STOP").as_deref() == Ok("1") {
        return Err(failed("stopped"));
    }
    let secs_of = |nodes: &[Node]| -> Result<Vec<Sec>, BlendError> { nodes.iter().map(|n| section(&n.st, sg, r, kk)).collect() };
    for round in 0..16 {
        let secs = secs_of(&nodes)?;
        let ps: Vec<Vec3> = nodes.iter().map(|n| n.st.p).collect();
        let fit = fit_cols(&secs, &ps)?;
        let mut insert: Vec<(usize, usize, f64)> = vec![]; // (node index after which, link, s)
        let mut worst = 0.0f64;
        for i in 0..nodes.len() - 1 {
            let (n0, n1) = (&nodes[i], &nodes[i + 1]);
            let (li, s0) = if n0.li == n1.li { (n1.li, n0.s) } else { (n1.li, 0.0) };
            let sm = 0.5 * (s0 + n1.s);
            // the fitted contact curves must stay on their faces between stations
            let mut dev = 0.0f64;
            for f in [0.25, 0.5, 0.75] {
                let u = fit.u[i] + (fit.u[i + 1] - fit.u[i]) * f;
                for (col, set) in [(0usize, &ctx.set1), (NV - 1, &ctx.set2)] {
                    let q = fit.cols[col].eval(u);
                    if let Some((pt, _)) = foot_set(b, set, q) {
                        dev = dev.max(pt.dist(q));
                    }
                }
            }
            worst = worst.max(dev);
            if dev > FIT_TOL {
                insert.push((i, li, sm));
            }
        }
        dbg(&format!("fit round {round}: {} nodes, worst on-face deviation {worst:.2e}", nodes.len()));
        if insert.is_empty() {
            break;
        }
        for &(i, li, sm) in insert.iter().rev() {
            let guess = Some((nodes[i].st.c + nodes[i + 1].st.c) * 0.5);
            let st = ctx.solve(li, sm, guess)?;
            nodes.insert(i + 1, Node { li, s: sm, st });
        }
        if nodes.len() > 1500 {
            break;
        }
    }
    if std::env::var("CK_FF_STOP").as_deref() == Ok("2") {
        return Err(failed("stopped after fit"));
    }
    // ---- end handling
    let mut secs = secs_of(&nodes)?;
    let mut ps: Vec<Vec3> = nodes.iter().map(|n| n.st.p).collect();
    let end_info = |end: usize| -> (ck_topo::body::VertexId, Vec<FaceId>) {
        let l = &ctx.links[if end == 0 { 0 } else { ctx.links.len() - 1 }];
        let ed = b.edge(l.e);
        let v = if (end == 0) == l.fwd { ed.v0 } else { ed.v1 };
        let fs: Vec<FaceId> = b
            .face_ids()
            .filter(|&f| f != l.f1 && f != l.f2 && b.face_edges(f).iter().any(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v))
            .collect();
        (v, fs)
    };
    // planes that trim the finished tool (a convex chain that ends in a concave corner), stations put before the chain
    let mut trims: Vec<(Vec3, Vec3)> = vec![];
    let mut pre = 0usize;
    // ends whose cap lies in the end face's plane
    let mut plane_caps = [false; 2];
    if convex {
        let m = 0.5 * r;
        let shift = |s: &Sec, d: Vec3| Sec { pts: s.pts.map(|p| p + d), d0: s.d0, d1: s.d1 };
        for end in [0usize, 1] {
            let last = nodes.len() - 1;
            let nd = nodes[if end == 0 { 0 } else { last }].clone();
            let t_out = if end == 0 { -nd.st.t } else { nd.st.t };
            // A neighbouring plane whose material lies behind the chain end (the edge runs down into a floor, a concave
            // corner): a prism past the end would cut into that material. The tool then continues along the faces past
            // the plane and is trimmed to it, its cap flush with the plane.
            let (_, fs) = end_info(end);
            let floor = fs
                .iter()
                .filter_map(|&f| match &b.face(f).surface {
                    Surface::Plane { frame } => Some((b.planar_normal(f).dot(t_out), frame.o, b.planar_normal(f))),
                    _ => None,
                })
                .filter(|x| x.0 < -0.1)
                .min_by(|a, c| a.0.total_cmp(&c.0));
            let idx = if end == 0 { 0 } else { secs.len() - 1 };
            if let Some((_, o, n)) = floor {
                let out = extend_beyond(&ctx, &nd, t_out, o, n, -1.0, r, sg, kk)?;
                for (sc, p) in out {
                    if end == 0 {
                        secs.insert(0, sc);
                        ps.insert(0, p);
                        pre += 1;
                    } else {
                        secs.push(sc);
                        ps.push(p);
                    }
                }
                trims.push((o, n));
            } else {
                let d = t_out * m;
                let sh = shift(&secs[idx], d);
                if end == 0 {
                    secs.insert(0, sh);
                    ps.insert(0, nd.st.p + d);
                    pre += 1;
                } else {
                    secs.push(sh);
                    ps.push(nd.st.p + d);
                }
            }
        }
    } else {
        for end in [0usize, 1] {
            let (_, fs) = end_info(end);
            let plane = fs.iter().find_map(|&f| match &b.face(f).surface {
                Surface::Plane { frame } => Some((frame.o, b.planar_normal(f))),
                _ => None,
            });
            let Some((o, n)) = plane else { return Err(unsupported("concave chain end is not on a planar face")) };
            let idx = if end == 0 { 0 } else { secs.len() - 1 };
            let before = (secs[idx].clone(), ps[idx]);
            let ni = if end == 0 { 0 } else { nodes.len() - 1 };
            // the end station with its centre IN the end plane (its section
            // then lies in that plane, so the cap coincides with the face)
            {
                let nd = &nodes[ni];
                let st = ctx.solve_with(nd.li, nd.s, Some(nd.st.c), Some((o, n)))?;
                secs[idx] = section(&st, sg, r, kk)?;
                ps[idx] = st.p;
            }
            if std::env::var_os("CK_BLEND_DEBUG").is_some() {
                for (nm, pt, set) in [("A", secs[idx].pts[0], &ctx.set1), ("B", secs[idx].pts[NV - 1], &ctx.set2)] {
                    for &f in set.iter() {
                        for e in b.face_edges(f) {
                            let ed = b.edge(e);
                            let d = crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0, ed.t1), pt).1;
                            if d < 1e-2 {
                                dbg(&format!("end {end}: {nm} end is {d:.2e} from edge {:?} of {}", e, b.face_id_str(f)));
                            }
                        }
                    }
                }
            }
            let mut moved = 0.0f64;
            let mut flat = secs[idx].clone();
            for p in flat.pts.iter_mut() {
                let d = (*p - o).dot(n);
                moved = moved.max(d.abs());
                *p = *p - n * d;
            }
            if moved <= 0.02 * r {
                secs[idx] = flat;
                ps[idx] = ps[idx] - n * (ps[idx] - o).dot(n);
                plane_caps[end] = true;
                continue;
            }
            // The faces meet the end plane at an angle (a tapered wall running into a flat end): the section cannot lie
            // in the plane. The tool runs on past the plane along the faces and is trimmed to it.
            secs[idx] = before.0;
            ps[idx] = before.1;
            let nd = nodes[ni].clone();
            let t_out = if end == 0 { -nd.st.t } else { nd.st.t };
            let out = extend_beyond(&ctx, &nd, t_out, o, n, 1.0, r, sg, kk)?;
            for (sc, p) in out {
                if end == 0 {
                    secs.insert(0, sc);
                    ps.insert(0, p);
                    pre += 1;
                } else {
                    secs.push(sc);
                    ps.push(p);
                }
            }
            trims.push((o, -n));
        }
    }
    let fit = fit_cols(&secs, &ps)?;
    // chain parameter of every link junction (a blend face per link, like
    // one fillet per edge): the last station of link L-1
    let junction_u: Vec<f64> = (1..ctx.links.len())
        .filter_map(|l| nodes.iter().rposition(|n| n.li == l - 1).map(|i| fit.u[i + pre]))
        .collect();
    let mut tool = build_tool(&fit, &secs, feature, k, plane_caps, b, &ctx.links, &end_info, tol)?;
    for &(o, n) in &trims {
        tool = trim_tool(&tool, o, n, &FaceName::blend(feature, k))?;
    }
    if std::env::var("CK_FF_STOP").as_deref() == Ok("3") {
        return Err(failed("stopped after tool"));
    }
    // ---- apply
    let blend_name = FaceName::blend(feature, k);
    let mut hint: Vec<String> = vec![];
    for l in &ctx.links {
        for f in [l.f1, l.f2] {
            let id = b.face_id_str(f);
            if !hint.contains(&id) {
                hint.push(id);
            }
        }
    }
    let f1set: Vec<FaceId> = ctx.links.iter().map(|l| l.f1).collect();
    let f2set: Vec<FaceId> = ctx.links.iter().map(|l| l.f2).collect();
    let blend_f = tool.face_ids().find(|&f| tool.face(f).name == blend_name).ok_or_else(|| failed("tool lost its blend face"))?;
    let leg_a = tool.face_ids().find(|&f| tool.face(f).name == FaceName::other(feature, "TA4")).ok_or_else(|| failed("tool leg A"))?;
    let leg_b = tool.face_ids().find(|&f| tool.face(f).name == FaceName::other(feature, "TA1")).ok_or_else(|| failed("tool leg B"))?;
    let curve_a = Curve::BSpline(fit.cols[0].clone());
    let curve_b = Curve::BSpline(fit.cols[NV - 1].clone());
    let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> {
        if g == blend_f && (f1set.contains(&f) || f2set.contains(&f)) {
            return Some(vec![]);
        }
        if g == leg_a && f1set.contains(&f) {
            return Some(vec![curve_a.clone()]);
        }
        if g == leg_b && f2set.contains(&f) {
            return Some(vec![curve_b.clone()]);
        }
        None
    };
    let op = if convex { Op::Cut } else { Op::Union };
    let (mut out, _) = apply_tool_hooked(b, &tool, op, &blend_name, hint, tol.max(FF_TOL), Some(&hook))?;
    // the imprint welded at FF_TOL: put the vertices back on their faces and
    // re-fit edges to them, so the result validates at the model tolerance
    split_blend_at_links(&mut out, &blend_name, &tool, blend_f, &junction_u, tol.max(FF_TOL));
    // (vertices that coincide with a tool vertex take the tool's exact spot:
    // it lies on all the faces it joins)
    if std::env::var_os("CK_BLEND_DEBUG3").is_some() {
        for v in out.vertex_ids() {
            eprintln!("PRE-SNAP VERT {:?} {:?}", v, out.vertex(v).p);
        }
        for v in tool.vertex_ids() {
            eprintln!("TOOL VERT {:?} {:?}", v, tool.vertex(v).p);
        }
    }
    for v in out.vertex_ids().collect::<Vec<_>>() {
        let p = out.vertex(v).p;
        let near = tool.vertex_ids().map(|w| tool.vertex(w).p).filter(|q| q.dist(p) <= 2.0 * FF_TOL).min_by(|a, b| a.dist(p).total_cmp(&b.dist(p)));
        if let Some(q) = near {
            out.vertex_mut(v).p = q;
        }
    }
    collapse_short_edges(&mut out, 6.0 * FF_TOL);
    crate::surfacing::trim::settle_vertices(&mut out, tol);
    refit_edges_on_faces(&mut out);
    crate::surfacing::trim::snap_edge_ends(&mut out, tol);
    Ok(out)
}

/// Stations of the chain end `nd` carried on along the faces (the ball centre solved on planes normal to the edge,
/// `t_out` away from the chain) until the whole section lies on the far side of the plane `(o, n)`: `beyond` = -1.0
/// behind it (the material side of a face with outward normal `n`), +1.0 in front of it.
#[allow(clippy::too_many_arguments)]
fn extend_beyond(ctx: &Ctx, nd: &Node, t_out: Vec3, o: Vec3, n: Vec3, beyond: f64, r: f64, sg: f64, kk: f64) -> Result<Vec<(Sec, Vec3)>, BlendError> {
    let mut mult = 2.0;
    while mult <= 32.0 {
        let mut out: Vec<(Sec, Vec3)> = vec![];
        let mut prev_c = nd.st.c;
        let mut prev_d = 0.0;
        let mut ok = true;
        for frac in [0.5, 1.0] {
            let d = mult * r * frac;
            let shifted = nd.st.p + t_out * d;
            let guess = prev_c + t_out * (d - prev_d);
            match ctx.solve_with(nd.li, nd.s, Some(guess), Some((shifted, t_out))).and_then(|st| section(&st, sg, r, kk).map(|sc| (st, sc))) {
                Ok((st, sc)) => {
                    prev_c = st.c;
                    prev_d = d;
                    out.push((sc, shifted));
                }
                Err(e) => {
                    dbg(&format!("extension at {d:.3} fails: {}", e.code));
                    ok = false;
                    break;
                }
            }
        }
        if ok && out.last().is_some_and(|(sc, _)| sc.pts.iter().all(|p| beyond * (*p - o).dot(n) > 1e-3)) {
            return Ok(out);
        }
        mult *= 2.0;
    }
    Err(unsupported("chain end: the section does not clear the end plane"))
}

/// `tool` cut back to the side of the plane `(o, n)` that `n` points to (the air side of a face with outward normal
/// `n`: the tool ran on into the material behind it); the tool's faces keep their names, the new cap is the plane.
/// The curves where the plane meets the tool's strips are handed to the imprint directly (the generic marcher misses
/// them on strips this thin).
fn trim_tool(tool: &Body, o: Vec3, n: Vec3, blend: &FaceName) -> Result<Body, BlendError> {
    use ck_core::linalg::Frame;
    use ck_geom::Vec2;
    use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    let z = n;
    let axis = if z.x.abs() < 0.6 { Vec3::X } else { Vec3::Y };
    let x = axis.cross(z).normalize();
    let y = z.cross(x);
    let size = 1e3;
    let p = [Vec2::new(-size, -size), Vec2::new(size, -size), Vec2::new(size, size), Vec2::new(-size, size)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    let prof = Profile::new(Frame { o, x, y, z }, vec![ProfileRegion { outer: lp, holes: vec![] }]);
    let half = extrude("Fh", &prof, ExtrudeOpts { s0: 0.0, s1: size, dir: None, draft: 0.0 }).map_err(|e| failed(&format!("half space: {e}")))?;
    let cap = half.face_ids().find(|&f| half.face_id_str(f) == "Fh.C0").ok_or_else(|| failed("half space cap"))?;
    // plane x strip: for samples across v, the u where the strip crosses the plane, joined into a curve
    let cut_of = |f: FaceId| -> Vec<Curve> {
        let Surface::BSpline(bs) = &tool.face(f).surface else { return vec![] };
        let ((u0, u1), (v0, v1)) = bs.domain();
        let nvs = 24;
        let mut pts: Vec<Vec3> = vec![];
        for j in 0..=nvs {
            let v = v0 + (v1 - v0) * j as f64 / nvs as f64;
            let g = |u: f64| (bs.eval(u, v) - o).dot(n);
            let nus = 96;
            let mut found = None;
            let mut prev = (u0, g(u0));
            for i in 1..=nus {
                let u = u0 + (u1 - u0) * i as f64 / nus as f64;
                let gu = g(u);
                if prev.1 == 0.0 || prev.1 * gu < 0.0 {
                    let (mut lo, mut hi) = (prev.0, u);
                    let mut glo = prev.1;
                    for _ in 0..80 {
                        let mid = 0.5 * (lo + hi);
                        let gm = g(mid);
                        if glo * gm <= 0.0 {
                            hi = mid;
                        } else {
                            lo = mid;
                            glo = gm;
                        }
                    }
                    found = Some(0.5 * (lo + hi));
                    break;
                }
                prev = (u, gu);
            }
            match found {
                Some(u) => pts.push(bs.eval(u, v)),
                None => return vec![],
            }
        }
        let mut t = vec![0.0; pts.len()];
        for i in 1..pts.len() {
            t[i] = t[i - 1] + pts[i].dist(pts[i - 1]).max(1e-9);
        }
        let tot = t[pts.len() - 1];
        for x in &mut t {
            *x /= tot;
        }
        let last = t.len() - 1;
        t[last] = 1.0;
        let ku = averaged_knots(&t, 3);
        match interpolate_with(&pts, 3, &t, ku) {
            Ok(c) => vec![Curve::BSpline(c)],
            Err(_) => vec![],
        }
    };
    let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> {
        if g == cap {
            Some(cut_of(f))
        } else {
            Some(vec![])
        }
    };
    let (out, _) = apply_tool_hooked(tool, &half, Op::Intersect, blend, vec![], FF_TOL, Some(&hook))?;
    Ok(out)
}

#[allow(clippy::too_many_arguments)]
fn build_tool(
    fit: &Fit,
    secs: &[Sec],
    feature: &str,
    k: u32,
    plane_caps: [bool; 2],
    b: &Body,
    links: &[Link],
    end_info: &dyn Fn(usize) -> (ck_topo::body::VertexId, Vec<FaceId>),
    _tol: f64,
) -> Result<Body, BlendError> {
    use ck_topo::orient::orient_body;
    let nu = secs.len();
    let mut tool = Body::new();
    let lump = tool.add_lump();
    let shell = tool.add_shell(lump);
    let (u0, u1) = (fit.ku[0], fit.ku[fit.ku.len() - 1]);
    // vertices per end per track
    let vs: Vec<[ck_topo::body::VertexId; 5]> = [0usize, nu - 1]
        .iter()
        .map(|&i| {
            let mut a = [ck_topo::body::VertexId(0); 5];
            for (t, &c) in TRACKS.iter().enumerate() {
                a[t] = tool.add_vertex(secs[i].pts[c]);
            }
            a
        })
        .collect();
    // longitudinal edges
    let long: Vec<EdgeId> = (0..5).map(|t| tool.add_edge(Curve::BSpline(fit.cols[TRACKS[t]].clone()), u0, u1, vs[0][t], vs[1][t])).collect();
    // surfaces per strip
    let blend = blend_patch(fit)?;
    let strips: Vec<Surface> = vec![
        Surface::BSpline(Box::new(blend.clone())),
        Surface::BSpline(Box::new(ruled(fit, TRACKS[1], TRACKS[2])?)),
        Surface::BSpline(Box::new(ruled(fit, TRACKS[2], TRACKS[3])?)),
        Surface::BSpline(Box::new(ruled(fit, TRACKS[3], TRACKS[4])?)),
        Surface::BSpline(Box::new(ruled(fit, TRACKS[4], TRACKS[0])?)),
    ];
    // cross edges per end per strip
    let mut cross: Vec<Vec<EdgeId>> = vec![vec![], vec![]];
    for (end, &u) in [u0, u1].iter().enumerate() {
        for s in 0..5 {
            let (va, vb) = (vs[end][s], vs[end][(s + 1) % 5]);
            let (pa, pb) = (tool.vertex(va).p, tool.vertex(vb).p);
            let e = if s == 0 {
                let arc = blend.iso_u(u);
                let (v0, v1) = (arc.domain().0, arc.domain().1);
                tool.add_edge(Curve::BSpline(arc), v0, v1, va, vb)
            } else {
                let d = pb - pa;
                let len = d.len();
                tool.add_edge(Curve::Line { o: pa, d: d / len.max(1e-300) }, 0.0, len, va, vb)
            };
            cross[end].push(e);
        }
    }
    let name_of = |s: usize| if s == 0 { FaceName::blend(feature, k) } else { FaceName::other(feature, &format!("TA{s}")) };
    // strip faces (loop: v=0 side forward, end cross forward, v=1 side back, start cross back)
    for s in 0..5 {
        let f = tool.add_face(shell, strips[s].clone(), true, if s == 4 { FaceName::other(feature, "TA4") } else { name_of(s) });
        let lp = tool.add_loop(f);
        tool.add_coedge(lp, long[s], false, None);
        tool.add_coedge(lp, cross[1][s], false, None);
        tool.add_coedge(lp, long[(s + 1) % 5], true, None);
        tool.add_coedge(lp, cross[0][s], true, None);
    }
    // caps
    let mut cap_frames: Vec<(Vec3, Vec3)> = vec![];
    for (end, i) in [0usize, nu - 1].iter().enumerate() {
        let pts: Vec<Vec3> = TRACKS.iter().map(|&c| secs[*i].pts[c]).collect();
        let mut area = Vec3::ZERO;
        for q in 0..5 {
            area = area + pts[q].cross(pts[(q + 1) % 5]);
        }
        let n = area.try_normalize().ok_or_else(|| failed("degenerate cap"))?;
        let mut frame_n = n;
        let mut frame_o = pts[0];
        if plane_caps[end] {
            // flush with the end face: use its exact plane
            let (_, fs) = end_info(end);
            if let Some((o, nn)) = fs.iter().find_map(|&f| match &b.face(f).surface {
                Surface::Plane { frame } => Some((frame.o, frame.z)),
                _ => None,
            }) {
                frame_o = o;
                frame_n = if nn.dot(n) >= 0.0 { nn } else { -nn };
            }
        }
        cap_frames.push((frame_o, frame_n));
        let f = tool.add_face(shell, Surface::plane(frame_o, frame_n), true, FaceName::other(feature, &format!("TC{end}")));
        let lp = tool.add_loop(f);
        for s in 0..5 {
            tool.add_coedge(lp, cross[end][s], false, None);
        }
    }
    let _ = links;
    orient_body(&mut tool);
    // a sign test only (the shell was just oriented outward): the coarse tier is enough
    let tool_faces: Vec<_> = tool.face_ids().collect();
    let vol = ck_topo::mass::signed_volume_quick(&tool, &tool_faces);
    if !(vol > 0.0) {
        return Err(failed(&format!("tool volume {vol}")));
    }
    if std::env::var_os("CK_BLEND_DEBUG").is_some() {
        let vol = ck_topo::mass::mass_props(&tool).volume;
        dbg(&format!("tool built: {nu} stations, volume {vol:.3}, issues {:?}", ck_topo::check::check(&tool, 1e-5).len()));
    }
    Ok(tool)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;
    use ck_topo::mass::mass_props;

    /// Replace the plane carrying face `id` by an equal bilinear B-spline
    /// patch (same orientation), so the analytic fillet path steps aside.
    fn splinify(b: &mut Body, id: &str) {
        let f = b.find_face(id).unwrap();
        let Surface::Plane { frame } = b.face(f).surface.clone() else { panic!("not a plane") };
        let c = |u: f64, v: f64| frame.o + frame.x * u + frame.y * v;
        let s = 200.0;
        b.faces[f.idx()].surface = Surface::BSpline(Box::new(BSplineSurface::bilinear(c(-s, -s), c(s, -s), c(-s, s), c(s, s))));
        for l in b.faces[f.idx()].loops.clone() {
            for co in b.loops[l.idx()].coedges.clone() {
                b.coedges[co.idx()].pcurve = None;
            }
        }
    }

    #[test]
    fn convex_edge_of_spline_faces_matches_the_analytic_fillet() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let want = mass_props(&super::super::build::fillet(&b, &[("F2.C1".into(), "F2.S.e1".into(), Vec3::new(20., 10., 0.))], 2.0, "F3", 1e-6).unwrap()).volume;
        let mut s = b.clone();
        splinify(&mut s, "F2.C1");
        splinify(&mut s, "F2.S.e1");
        let r = fillet_freeform(&s, &[("F2.C1".into(), "F2.S.e1".into(), Vec3::new(20., 10., 0.))], 2.0, "F3", 1e-6).unwrap();
        let (v, area) = (mass_props(&r).volume, mass_props(&r).area);
        eprintln!("convex: volume {v} (analytic {want}), area {area} (analytic 2763.946891450771)");
        assert!(ck_topo::check::check(&r, 1e-4).is_empty(), "{:?}", ck_topo::check::check(&r, 1e-4));
        let bf = r.find_face("F3.B0").unwrap();
        let Surface::BSpline(bs) = &r.face(bf).surface else { panic!("blend is not a spline") };
        // the rolling ball (r = 2) runs on the axis y = 8, z = -2 (x along the edge)
        let ((u0, u1), (v0, v1)) = bs.domain();
        for i in 0..=10 {
            for j in 0..=10 {
                let p = bs.eval(u0 + (u1 - u0) * i as f64 / 10.0, v0 + (v1 - v0) * j as f64 / 10.0);
                let d = ((p.y - 8.0).powi(2) + (p.z + 2.0).powi(2)).sqrt();
                assert!((d - 2.0).abs() < 1e-5, "blend point {p:?} is {d} from the ball axis");
            }
        }
        // (the spline blend patch is integrated numerically: ~1e-5 relative)
        assert!((v - want).abs() < 2e-5 * want, "{v} vs analytic {want}");
    }

    #[test]
    fn concave_edge_of_spline_faces_matches_the_analytic_fillet() {
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let tool = box_body("F4", 0.0, 0.0, 20.0, 20.0, 5.0, 11.0);
        let l = crate::boolean::boolean(&a, &tool, crate::boolean::Op::Cut, 1e-6).unwrap().body;
        let floor = l.face_ids().find(|&f| l.face_id_str(f).starts_with("F4.C0")).unwrap();
        let (fa, fb) = (l.face_id_str(floor), "F4.S.e2".to_string());
        let at = Vec3::new(20., 5., -10.);
        let want = mass_props(&super::super::build::fillet(&l, &[(fa.clone(), fb.clone(), at)], 1.0, "F5", 1e-6).unwrap()).volume;
        let mut s = l.clone();
        splinify(&mut s, &fa);
        splinify(&mut s, &fb);
        let r = fillet_freeform(&s, &[(fa, fb, at)], 1.0, "F5", 1e-6).unwrap();
        let v = mass_props(&r).volume;
        assert!((v - want).abs() < 2e-4 * want.abs().sqrt(), "{v} vs analytic {want}");
        assert!(ck_topo::check::check(&r, 1e-4).is_empty(), "{:?}", ck_topo::check::check(&r, 1e-4));
    }
}

/// Re-fit spline edges that drifted off the two faces they join (trimmed
/// intersection curves are only as accurate as their fit): every sample is
/// pulled onto both surfaces by alternating projection and the curve is
/// re-interpolated through them, with the vertices as end points. Edges
/// between grazing (nearly tangent) faces keep their curve (alternating
/// projection does not converge there).
pub fn refit_edges_on_faces(out: &mut Body) {
    for e in out.edge_ids().collect::<Vec<_>>() {
        let ed = out.edge(e).clone();
        if !matches!(ed.curve, Curve::BSpline(_)) || ed.v0 == ed.v1 {
            continue;
        }
        let fs = out.edge_faces(e);
        if fs.len() != 2 {
            continue;
        }
        let (s1, s2) = (out.face(fs[0]).surface.clone(), out.face(fs[1]).surface.clone());
        if s1 == s2 {
            continue; // a split rail inside one surface
        }
        let mid = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        let nrm = |s: &Surface| -> Option<Vec3> {
            let (u, v, _) = s.closest_point(mid);
            s.normal(u, v).try_normalize()
        };
        let (Some(n1), Some(n2)) = (nrm(&s1), nrm(&s2)) else { continue };
        if n1.cross(n2).len() < 0.3 {
            continue;
        }
        let len = out.edge_length(e);
        let n = ((len / 0.5).ceil() as usize).clamp(24, 600);
        let (mut h1, mut h2): (Option<(f64, f64)>, Option<(f64, f64)>) = (None, None);
        let mut proj = |s: &Surface, p: Vec3, h: &mut Option<(f64, f64)>| -> Vec3 {
            let (u, v, q) = match *h {
                Some(hh) => s.closest_point_near(p, hh),
                None => s.closest_point(p),
            };
            *h = Some((u, v));
            q
        };
        let mut pts = Vec::with_capacity(n + 1);
        let mut dev = 0.0f64;
        for k in 0..=n {
            let p0 = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * k as f64 / n as f64);
            let mut p = p0;
            for _ in 0..40 {
                let a = proj(&s1, p, &mut h1);
                let b = proj(&s2, a, &mut h2);
                let moved = b.dist(p);
                p = b;
                if moved < 1e-12 {
                    break;
                }
            }
            dev = dev.max(p.dist(p0));
            pts.push(p);
        }
        if dev <= 2e-7 || dev > 5e-2 {
            continue;
        }
        pts[0] = out.vertex(ed.v0).p;
        pts[n] = out.vertex(ed.v1).p;
        pts.dedup_by(|a, b| a.dist(*b) < 1e-10);
        if pts.len() < 4 {
            continue;
        }
        if let Ok(c) = ck_geom::fit::interpolate(&pts, 3, ck_geom::fit::Param::ChordLength) {
            let (a, b) = c.domain();
            let em = out.edge_mut(e);
            em.curve = Curve::BSpline(c);
            em.t0 = a;
            em.t1 = b;
            for co in out.edge(e).coedges.clone() {
                out.coedge_mut(co).pcurve = None;
            }
        }
    }
}

/// Remove closed edges (both ends the same vertex) of length below `max_len`: the wrap-around
/// remnant of a conic edge cut at a seam parameter (a 3e-6 rad ellipse sliver) is an open,
/// degenerate loop member that bounds nothing.
pub fn drop_zero_edges(out: &mut Body, max_len: f64) {
    let cand: Vec<_> = out.edge_ids().filter(|&e| out.edge(e).v0 == out.edge(e).v1 && out.edge_length(e) < max_len).collect();
    for e in cand {
        for c in out.edge(e).coedges.clone() {
            let l = out.coedge(c).lp;
            out.loops[l.idx()].coedges.retain(|&x| x != c);
            out.coedges[c.idx()].alive = false;
        }
        out.edges[e.idx()].alive = false;
        out.edges[e.idx()].coedges.clear();
    }
}

/// Collapse edges shorter than `max_len` whose end vertices coincide to
/// within it (slivers the welding at a loose tolerance leaves where an
/// inexact trimmed edge crosses a tool edge): the edge and its coedges
/// vanish and its second vertex is merged into the first.
pub fn collapse_short_edges(out: &mut Body, max_len: f64) {
    drop_zero_edges(out, max_len);
    loop {
        let cand = out.edge_ids().find(|&e| {
            let ed = out.edge(e);
            ed.v0 != ed.v1 && out.vertex(ed.v0).p.dist(out.vertex(ed.v1).p) < max_len && out.edge_length(e) < 3.0 * max_len
        });
        let Some(e) = cand else { break };
        let (keep, gone) = (out.edge(e).v0, out.edge(e).v1);
        // never collapse an edge that is a whole loop on its own face
        for c in out.edge(e).coedges.clone() {
            let l = out.coedge(c).lp;
            out.loops[l.idx()].coedges.retain(|&x| x != c);
            out.coedges[c.idx()].alive = false;
        }
        out.edges[e.idx()].alive = false;
        out.edges[e.idx()].coedges.clear();
        for ed in out.edges.iter_mut().filter(|d| d.alive) {
            if ed.v0 == gone {
                ed.v0 = keep;
            }
            if ed.v1 == gone {
                ed.v1 = keep;
            }
        }
        out.vertices[gone.idx()].alive = false;
    }
}

/// Cut the blend face of a chain into one face per link along the section
/// arcs at the link junctions (the oracle blends edge by edge: one blend
/// face per edge, all named alike; `~k` numbering follows later).
fn split_blend_at_links(out: &mut Body, name: &FaceName, tool: &Body, tool_blend: FaceId, junction_u: &[f64], tol: f64) {
    let Surface::BSpline(bs) = tool.face(tool_blend).surface.clone() else { return };
    for &uj in junction_u {
        let arc = bs.iso_u(uj);
        let (a_pt, b_pt, mid) = (arc.eval(arc.domain().0), arc.eval(arc.domain().1), arc.eval(0.5 * (arc.domain().0 + arc.domain().1)));
        let faces: Vec<FaceId> = out.face_ids().filter(|&f| out.face(f).name == *name).collect();
        let Some(f) = faces.into_iter().find(|&f| crate::boolean::classify::point_in_face(out, f, mid, tol) == crate::boolean::classify::Where::In) else {
            dbg("junction arc is not inside a blend face");
            continue;
        };
        let mut ends = vec![];
        for pt in [a_pt, b_pt] {
            // the vertex of f's boundary at the arc end, splitting the edge there when it has none
            let mut found = None;
            for &l in &out.face(f).loops.clone() {
                for &c in &out.lp(l).coedges.clone() {
                    let v = out.coedge_start(c);
                    if out.vertex(v).p.dist(pt) <= 1e-4 {
                        found = Some(v);
                    }
                }
            }
            if found.is_none() {
                let best = out
                    .face_edges(f)
                    .into_iter()
                    .map(|e| {
                        let ed = out.edge(e);
                        let (t, d) = crate::boolean::imprint::dist_in_range(&ed.curve, (ed.t0.min(ed.t1), ed.t0.max(ed.t1)), pt);
                        (d, e, t)
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0));
                if let Some((d, e, t)) = best {
                    let ed = out.edge(e);
                    if d <= tol && t > ed.t0.min(ed.t1) + 1e-9 && t < ed.t0.max(ed.t1) - 1e-9 {
                        let (v, ne) = ck_topo::edit::split_edge(out, e, t);
                        out.vertex_mut(v).p = pt;
                        // the copied pcurves no longer fit the halves
                        for x in [e, ne] {
                            for c in out.edge(x).coedges.clone() {
                                out.coedge_mut(c).pcurve = None;
                            }
                        }
                        found = Some(v);
                    }
                }
            }
            ends.push(found);
        }
        let (Some(va), Some(vb)) = (ends[0], ends[1]) else {
            dbg("junction arc ends are not on the blend boundary");
            continue;
        };
        let (t0, t1) = arc.domain();
        if let Err(e) = ck_topo::edit::split_face(out, f, va, vb, Curve::BSpline(arc), t0, t1, None, name.clone()) {
            dbg(&format!("junction split failed: {e}"));
        }
    }
}
