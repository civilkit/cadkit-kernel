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
use ck_geom::curve2::Curve2;
use ck_geom::{Vec2, Vec3};
use crate::surfacing::knit::BodyExt;
use ck_topo::body::{Body, EdgeId, FaceId, FaceName};
use super::diag::{dbg, failed, failed_hint, unsupported, unsupported_hint};
use super::model::{Link, Sec, Station, NT, NV};

/// Welding tolerance of the imprint: the trimmed faces' boundary edges are
/// only accurate to about this much against their surfaces.
const FF_TOL: f64 = 1e-3;
/// Contact-curve fit tolerance between stations.
const FIT_TOL: f64 = 1.2e-5;

/// User-facing hint when the rolling ball cannot be placed on a blade-root chain: the dihedral is too
/// shallow (a sharp trailing edge, a large radius) for the ball to touch both faces.
const BALL_HINT: &str = "try a smaller fillet radius; near a sharp or nearly tangent trailing edge the ball cannot touch both faces, so give the edge a small blunt width or use a smaller radius";

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
    // a circle between a ball centred on its axis and a coaxial plane, cylinder or ball (a rod standing in a ball) is analytic:
    // the rolling ball's section is solved in the meridian plane (`analytic::sphere_section`)
    let fs = b.edge_faces(e);
    if let (Curve::Circle { .. }, Some((c, axis, rad))) = (&ed.curve, ed.curve.circle_info()) {
        let axis = axis.normalize();
        let coaxial_ball = |f: FaceId| matches!(&b.face(f).surface, Surface::Sphere { frame, .. } if (frame.o - c).cross(axis).len() < 1e-7 * (1.0 + rad));
        let coaxial_other = |f: FaceId| match &b.face(f).surface {
            Surface::Plane { frame } => frame.z.cross(axis).len() < 1e-9,
            Surface::Cylinder { frame, .. } => frame.z.cross(axis).len() < 1e-9 && (frame.o - c).cross(axis).len() < 1e-7 * (1.0 + rad),
            Surface::Sphere { .. } => coaxial_ball(f),
            _ => false,
        };
        if fs.len() == 2 && fs.iter().any(|&f| coaxial_ball(f)) && fs.iter().all(|&f| coaxial_other(f)) && concave_circle(b, e, [fs[0], fs[1]]) {
            return false;
        }
    }
    fs.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. }))
}

/// Is the circle edge `e` between `faces` a concave one (the material wraps past a half turn there)? The test of the analytic
/// builder: the direction into the first face, against the second face's outward normal.
fn concave_circle(b: &Body, e: EdgeId, faces: [FaceId; 2]) -> bool {
    let ed = b.edge(e);
    let t = 0.5 * (ed.t0 + ed.t1);
    let p = ed.curve.eval(t);
    let Some(co) = b.edge_coedges(e).into_iter().find(|&c| b.coedge_face(c) == faces[0]) else { return false };
    let d = ed.curve.tangent(t).normalize();
    let tdir = if b.coedge(co).rev { -d } else { d };
    let n = |f: FaceId| {
        let (u, v) = b.face(f).surface.closest_param(p);
        b.outward_normal(f, u, v).normalize()
    };
    let (n1, n2) = (n(faces[0]), n(faces[1]));
    n1.cross(tdir).dot(n2) >= 0.0
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

/// A screw (twisted extrude) face's surface with its profile range padded by `pad` each side: the ball of a chain end
/// that runs on past the face's boundary (the tool's end extension) is still carried by the same surface there, where
/// the foot of the finite patch would clamp to its boundary and the contact would leave the surface.
fn padded_screw(sf: &Surface, pad: f64) -> Option<Surface> {
    match sf {
        Surface::Screw { profile, axis_o, axis, rise, urange: (u0, u1), vrange: (v0, v1) } => {
            // (along the axis too: a ball near the end of a vertical edge has its foot a little past the face's last turn)
            let pu = if rise.abs() > 1e-12 { pad / rise.abs() } else { 0.0 };
            Some(Surface::Screw { profile: profile.clone(), axis_o: *axis_o, axis: *axis, rise: *rise, urange: (u0 - pu, u1 + pu), vrange: (v0 - pad, v1 + pad) })
        }
        _ => None,
    }
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
    let padded = padded_screw(&b.face(f).surface, 3.0 * reach);
    let sf = padded.as_ref().unwrap_or(&b.face(f).surface);
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

/// Diagnostic for a `foot_set` miss (only called behind `CK_BLEND_DEBUG`): for
/// every face of the set, the closest point distance, whether the ball centre
/// lies in the padded face box and the outward normal there.
fn explain_foot(b: &Body, fs: &[FaceId], q: Vec3) -> String {
    let reach = REACH.with(|r| r.get());
    let mut s = format!("c=({:.4},{:.4},{:.4})", q.x, q.y, q.z);
    for &f in fs {
        let bx = crate::bvh::face_box(b, f);
        let inside = bx.padded(2.5 * reach + 0.05 * bx.diagonal()).contains(q);
        let sf = &b.face(f).surface;
        let (u, v, pt) = sf.closest_point(q);
        let n = b.outward_normal(f, u, v).try_normalize();
        s += &format!(
            " | {} d={:.3e} box={} n={}",
            b.face_id_str(f),
            pt.dist(q),
            if inside { "in" } else { "OUT" },
            match n {
                Some(x) => format!("({:.2},{:.2},{:.2})", x.x, x.y, x.z),
                None => "NULL".into(),
            }
        );
    }
    s
}

/// Diagnostic for `CK_BLEND_DEBUG`: which face of the set the foot came from and
/// at which `(u, v)`, so a foot that jumps faces or sits on a face boundary is visible.
fn foot_verbose(b: &Body, fs: &[FaceId], q: Vec3) -> String {
    let mut best = f64::INFINITY;
    let mut parts: Vec<(FaceId, f64, f64, f64, Option<Vec3>)> = vec![];
    for &f in fs {
        let sf = &b.face(f).surface;
        let (u, v, pt) = sf.closest_point(q);
        let n = b.outward_normal(f, u, v).try_normalize();
        let d = pt.dist(q);
        if d < best {
            best = d;
        }
        parts.push((f, u, v, d, n));
    }
    parts
        .into_iter()
        .map(|(f, u, v, d, n)| {
            format!(
                "{} d={:.4} uv=({:.3},{:.3}){} n={}",
                b.face_id_str(f),
                d,
                u,
                v,
                if d <= best + 1e-9 { "*" } else { " " },
                n.map(|x| format!("({:.2},{:.2},{:.2})", x.x, x.y, x.z)).unwrap_or_else(|| "NULL".into()),
            )
        })
        .collect::<Vec<_>>()
        .join(" | ")
}

/// Signed distance to the nearest face of the set (the `foot_set` pick), as
/// `(dist, face, closest point, normal)`. Positive means `q` is outside the solid there.
fn signed_dist_set(b: &Body, fs: &[FaceId], q: Vec3) -> Option<(f64, FaceId, Vec3, Vec3)> {
    let mut best: Option<(f64, FaceId, Vec3, Vec3)> = None;
    for &f in fs {
        if let Some((pt, n)) = foot(b, f, q) {
            let d = pt.dist(q);
            if best.as_ref().map_or(true, |x| d < x.0) {
                best = Some((d, f, pt, n));
            }
        }
    }
    best
}

/// Decisive ground truth (behind `CK_BLEND_DEBUG`): sample the hub-offset cylinder
/// (the locus of ball centres exactly `r` from the hub) around `p` and find where
/// the blade is also `r` away, with NO plane constraint. If such a point exists but
/// not in the normal plane, the plane constraint is the culprit (cause 4); if none
/// exists, `r` is too large for the local geometry (cause 3).
fn probe_centre_cyl(b: &Body, set1: &[FaceId], set2: &[FaceId], r: f64, p: Vec3) -> String {
    let Some((o, ax, cr)) = set1.iter().find_map(|&f| match &b.face(f).surface {
        Surface::Cylinder { frame, r } => Some((frame.o, frame.z, *r)),
        _ => None,
    }) else {
        return "no cylinder in set1".into();
    };
    let rad = (p - o) - ax * (p - o).dot(ax);
    let Some(e2) = rad.try_normalize() else { return "p on the axis".into() };
    let e3 = ax.cross(e2);
    let z0 = (p - o).dot(ax);
    let (nr, nz) = (100usize, 50usize);
    let (arange, zrange) = (0.6f64, 4.0 * r);
    let mut best = (f64::INFINITY, 0.0, 0.0, 0.0);
    for i in 0..=nr {
        let dphi = -arange + 2.0 * arange * (i as f64) / (nr as f64);
        for j in 0..=nz {
            let dz = -zrange + 2.0 * zrange * (j as f64) / (nz as f64);
            let c = o + e2 * ((cr + r) * dphi.cos()) + e3 * ((cr + r) * dphi.sin()) + ax * (z0 + dz);
            if let Some((d2, _, _, _)) = signed_dist_set(b, set2, c) {
                let g = (d2 - r).abs();
                if g < best.0 {
                    best = (g, dphi, dz, c.dist(p));
                }
            }
        }
    }
    format!(
        "hub-offset cylinder probe: min|d2-r|={:.3e} at dphi={:.3}rad dz={:.3} |c-p|={:.4}",
        best.0, best.1, best.2, best.3
    )
}

/// Ground truth for a failed station (behind `CK_BLEND_DEBUG`): sample the normal
/// plane `(p, t)` on a grid and find where both face sets are `sg*r` away, so it is
/// visible whether the ball fits there at all. Returns the best residual found.
fn probe_centre(b: &Body, set1: &[FaceId], set2: &[FaceId], sg: f64, r: f64, p: Vec3, t: Vec3) -> String {
    let tn = t.try_normalize().unwrap_or(Vec3::Z);
    let a1 = if tn.x.abs() < 0.9 { tn.cross(Vec3::X) } else { tn.cross(Vec3::Y) };
    let a1 = a1.try_normalize().unwrap_or(Vec3::X);
    let a2 = tn.cross(a1).try_normalize().unwrap_or(Vec3::Y);
    let (n, span) = (100usize, 6.0 * r);
    let mut best = (f64::INFINITY, 0.0, 0.0, 0.0);
    let mut hits = 0usize;
    let mut min1 = f64::INFINITY;
    let mut min2 = f64::INFINITY;
    for i in 0..=n {
        for j in 0..=n {
            let a = -span + 2.0 * span * (i as f64) / (n as f64);
            let bb = -span + 2.0 * span * (j as f64) / (n as f64);
            let c = p + a1 * a + a2 * bb;
            let (Some((d1, _, _, _)), Some((d2, _, _, _))) = (signed_dist_set(b, set1, c), signed_dist_set(b, set2, c)) else { continue };
            let g1 = d1 - sg * r;
            let g2 = d2 - sg * r;
            if g1.abs() < min1 {
                min1 = g1.abs();
            }
            if g2.abs() < min2 {
                min2 = g2.abs();
            }
            let e = g1.abs().max(g2.abs());
            if e < 1e-3 * r {
                hits += 1;
            }
            if e < best.0 {
                best = (e, a, bb, c.dist(p));
            }
        }
    }
    format!(
        "normal-plane probe (span {span:.0}r): min|g1|={min1:.3e} min|g2|={min2:.3e}; best max|g|={:.3e} at a=({:.4},{:.4}), |c-p|={:.4}; cells |g|<1e-3r: {}",
        best.0, best.1, best.2, best.3, hits
    )
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
                // the two planes formula p + (n1+n2) r/(1+n1·n2) blows up where the faces are
                // near-opposite (a shallow dihedral, common at a blade root): fall back to the
                // bisector at radius r
                let denom = 1.0 + c12;
                if denom > 0.2 {
                    p + (n1 + n2) * (self.sg * r / denom)
                } else {
                    let dir = (n1 + n2).try_normalize().unwrap_or(n1);
                    p + dir * (self.sg * r)
                }
            }
        };
        for it in 0..80 {
            let (q1, n1) = match foot_set(self.b, &self.set1, c) {
                Some(x) => x,
                None => {
                    dbg(&format!("foot 1 miss: li={li} s={s:.4} {}", explain_foot(self.b, &self.set1, c)));
                    return Err(failed_hint("foot 1", BALL_HINT));
                }
            };
            let (q2, n2) = match foot_set(self.b, &self.set2, c) {
                Some(x) => x,
                None => {
                    dbg(&format!("foot 2 miss: li={li} s={s:.4} {}", explain_foot(self.b, &self.set2, c)));
                    return Err(failed_hint("foot 2", BALL_HINT));
                }
            };
            let g = [(c - q1).dot(n1) - self.sg * r, (c - q2).dot(n2) - self.sg * r, (c - off3).dot(row3)];
            let err = g.iter().fold(0.0f64, |m, x| m.max(x.abs()));
            if super::diag::on() {
                eprintln!(
                    "   it={it} c=({:.4},{:.4},{:.4}) g=[{:.3e},{:.3e},{:.3e}] p1=|c-q1|={:.4} p2=|c-q2|={:.4}",
                    c.x, c.y, c.z, g[0], g[1], g[2], c.dist(q1), c.dist(q2)
                );
                eprintln!("      set1: {}", foot_verbose(self.b, &self.set1, c));
                eprintln!("      set2: {}", foot_verbose(self.b, &self.set2, c));
            }
            if err < 1e-11 * (1.0 + r) {
                if super::diag::on_level(3) {
                    eprintln!("   foot residual: perp1 {:.2e} perp2 {:.2e}", (c - q1).cross(n1).len(), (c - q2).cross(n2).len());
                }
                return Ok(Station { p, a: q1, b: q2, c, t });
            }
            let mut dc = crate::intersect::solve3([n1, n2, row3], [-g[0], -g[1], -g[2]]).ok_or_else(|| unsupported("singular centre system (tangent faces)"))?;
            // damp the step: at a shallow dihedral (a blade root) the three-plane
            // system is near-singular and a full Newton step runs away
            let dl = dc.len();
            if super::diag::on() {
                eprintln!("      dc=({:.4},{:.4},{:.4}) |dc|={:.4}{}", dc.x, dc.y, dc.z, dl, if dl > 2.0 * r { " (clamped to 2r)" } else { "" });
            }
            if dl > 2.0 * r {
                dc = dc * (2.0 * r / dl);
            }
            c = c + dc;
            if !c.is_finite() || c.dist(p) > 20.0 * r {
                return Err(failed_hint("centre diverged", BALL_HINT));
            }
        }
        if super::diag::on() {
            eprintln!(
                "   li={li} s={s:.4} did not converge: p=({:.4},{:.4},{:.4}) t=({:.4},{:.4},{:.4}) sg={} r={}",
                p.x, p.y, p.z, t.x, t.y, t.z, self.sg, r
            );
            eprintln!("   at p: set1 {}", foot_verbose(self.b, &self.set1, p));
            eprintln!("   at p: set2 {}", foot_verbose(self.b, &self.set2, p));
            eprintln!("   {}", probe_centre_cyl(self.b, &self.set1, &self.set2, r, p));
            eprintln!("   {}", probe_centre(self.b, &self.set1, &self.set2, self.sg, r, p, t));
        }
        Err(failed_hint("centre did not converge", BALL_HINT))
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
/// ordered links. A chain that returns to its start is a closed ring
/// (`closed = true`); branches are unsupported.
/// Edges `x` and `y` leave their shared vertex `v` along one line (a smooth continuation, not a sharp corner).
fn continues_at(b: &Body, x: EdgeId, y: EdgeId, v: ck_topo::body::VertexId) -> bool {
    let dir = |e: EdgeId| {
        let ed = b.edge(e);
        ed.curve.tangent(if ed.v0 == v { ed.t0 } else { ed.t1 }).try_normalize()
    };
    match (dir(x), dir(y)) {
        (Some(a), Some(c)) => a.cross(c).len() < 1e-3,
        _ => false,
    }
}

/// Walk the selected `edges` from `start` into one chain. With `split_sharp` the chain also stops at a vertex where
/// the next selected edge leaves at an angle (a sharp corner): each smooth run is its own chain.
fn build_chain(b: &Body, edges: &[EdgeId], start: usize, split_sharp: bool) -> Result<(Vec<(usize, bool)>, bool), BlendError> {
    let ends = |i: usize| (b.edge(edges[i]).v0, b.edge(edges[i]).v1);
    let deg = |v: ck_topo::body::VertexId| edges.iter().enumerate().filter(|(i, _)| ends(*i).0 == v || ends(*i).1 == v).count();
    let mut chain = vec![(start, true)];
    let mut used = vec![false; edges.len()];
    used[start] = true;
    // forward
    let mut v = ends(start).1;
    let mut last = start;
    loop {
        if deg(v) > 2 && !split_sharp {
            return Err(unsupported("branching blend selection"));
        }
        let Some(nx) = (0..edges.len()).find(|&i| !used[i] && (ends(i).0 == v || ends(i).1 == v) && (!split_sharp || continues_at(b, edges[last], edges[i], v))) else { break };
        used[nx] = true;
        let fwd = ends(nx).0 == v;
        chain.push((nx, fwd));
        v = if fwd { ends(nx).1 } else { ends(nx).0 };
        last = nx;
    }
    // the walk returned to where it started: a closed ring of edges (no ends)
    if v == ends(start).0 && (!split_sharp || continues_at(b, edges[last], edges[start], v)) {
        return Ok((chain, true));
    }
    // backward
    let mut v = ends(start).0;
    if deg(v) > 2 && !split_sharp {
        return Err(unsupported("branching blend selection"));
    }
    let mut first = start;
    loop {
        let Some(pv) = (0..edges.len()).find(|&i| !used[i] && (ends(i).0 == v || ends(i).1 == v) && (!split_sharp || continues_at(b, edges[first], edges[i], v))) else { break };
        used[pv] = true;
        // walking backwards: the link ends at v
        let fwd = ends(pv).1 == v;
        chain.insert(0, (pv, fwd));
        v = if fwd { ends(pv).0 } else { ends(pv).1 };
        first = pv;
    }
    Ok((chain, false))
}

/// Tolerance of the B-spline a procedural face is converted to before it is rounded (mm).
const PROC_TOL: f64 = 1e-4;

/// A copy of `b` whose procedural faces (a sweep or a rolling-ball blend surface) on the picked edges are their certified
/// B-spline (within `PROC_TOL`), or `None` when there is none. Rounded as a procedural face the boolean classifies points
/// against its definition by closest-point search (tens of seconds for one rim) and no seam path can see its closed profile
/// (no declared period); as a B-spline it is a spline face like any other (a seam in v is read through `closed_v_span` in the
/// classifier and `closed_v_width` in the mass integral). The whole blend, its validity gate and its guard see the copy.
pub(super) fn procedural_to_bspline(b: &Body, edges: &[(String, String, Vec3)]) -> Option<Body> {
    let mut conv: Option<Body> = None;
    for (fa, fb, at) in edges {
        let Some(e) = find_edge(b, fa, fb, *at) else { continue };
        for f in b.edge_faces(e) {
            let sf = &b.face(f).surface;
            if !sf.is_procedural() {
                continue;
            }
            let (ur, vr) = sf.domain();
            let Ok((bs, _)) = sf.to_nurbs_within(ur, vr, PROC_TOL) else { continue };
            conv.get_or_insert_with(|| b.clone()).faces[f.idx()].surface = Surface::BSpline(Box::new(bs));
        }
    }
    conv
}

/// Fillet `edges` (free-form capable); see the module docs.
pub fn fillet_freeform(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64) -> Result<Body, BlendError> {
    // The wall of a thickened closed spline is two faces meeting at the seam vertices. A round along its rim crosses the
    // seams (the faces change there), and the blend's tools do not close there (with the wall's booleans fixed, every ring and
    // half rim still came out open or with a grown volume, after 25 s to 15 min): refuse at once, with the reason.
    if let Some(at) = split_skin_rim(b, edges) {
        return Err(failed_hint(
            &format!("the round runs along the rim of a closed-spline wall (a wall split in two faces at a seam, here at ({:.2}, {:.2}, {:.2}))", at.x, at.y, at.z),
            "rounding the rim of a thickened closed spline is not supported yet (the round's tool does not close across the seam between the wall's two halves). Round the edges of the sketch profile before the extrude and thicken (a sketch fillet at the corners), or build the rounded rim with a swept profile",
        ));
    }
    // A pick whose two faces are tangent all along it (the edge where a flat face runs into an earlier round) has no corner
    // to round at any size: the marcher refuses it at its first station. Refused up front, so the answer does not depend
    // on the pick order or the size (picked after an edge whose round consumes the face between them, it was reported as
    // that edge's overflow, or as missing).
    for (fa, fb, at) in edges {
        let Some(e) = find_edge(b, fa, fb, *at) else { continue };
        let fs = b.edge_faces(e);
        if fs.len() != 2 {
            continue;
        }
        let ed = b.edge(e);
        let smooth = [b.vertex(ed.v0).p, b.edge_mid(e), b.vertex(ed.v1).p].iter().all(|&p| match (foot(b, fs[0], p), foot(b, fs[1], p)) {
            (Some((_, n1)), Some((_, n2))) => 1.0 - n1.dot(n2).abs() < 1e-9,
            _ => false,
        });
        if smooth {
            return Err(super::diag::unsupported_hint(
                &format!("the faces {fa} and {fb} are tangent all along the picked edge between them: it is smooth, there is no corner to round"),
                "leave this edge out of the selection: a fillet rounds edges where the faces meet at an angle",
            ));
        }
    }
    // A face that is a procedural surface still here (the conversion of `procedural_to_bspline` failed or was not run) is read
    // through its definition, which the boolean classifies points against by closest-point search: tens of seconds for one
    // rim, and the result then fails its volume check. Refused up front, with the way round it that builds.
    for (fa, fb, at) in edges {
        let Some(e) = find_edge(b, fa, fb, *at) else { continue };
        if b.edge_faces(e).iter().any(|&f| b.face(f).surface.is_procedural()) {
            return Err(super::diag::unsupported_hint(
                &format!("the rounding of the edge between {fa} and {fb} does not depend on the radius: a fillet on a face that is a procedural sweep or blend surface could not be converted to a B-spline"),
                "use a plain sweep feature for the tube (its side face is a torus or a cylinder, which rounds) when the path is a line or an arc, or leave this rim sharp",
            ));
        }
    }
    let whole = fillet_freeform_chains(b, edges, r, feature, tol, false);
    // A selection that runs round a sharp corner is one chain above, which the solver cannot follow (the ball's two
    // faces change at the corner). Each smooth run on its own is a convex tool that runs on past its end, so the
    // runs meet in a mitre like two plain edge fillets do; retried only when the whole selection failed, and only when
    // two picked edges meet at a vertex (separate rings or runs are the same chains the first pass built: the retry
    // would repeat the failed build, seconds long on a fine free-form tool, and fail the same way).
    let eids_sel: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    let meets = eids_sel.iter().enumerate().any(|(i, &x)| {
        let (a0, a1) = (b.edge(x).v0, b.edge(x).v1);
        eids_sel[i + 1..].iter().any(|&y| [b.edge(y).v0, b.edge(y).v1].iter().any(|&v| v == a0 || v == a1))
    });
    if whole.is_err() && edges.len() > 1 && meets {
        if let Ok(done) = fillet_freeform_chains(b, edges, r, feature, tol, true) {
            return Ok(done);
        }
    }
    // A free-form tool that was built (the ball rolled along the whole edge) but whose boolean with the body could not be
    // assembled, on a rim between a plane and a curved side face that is not a cylinder or a cone (a swept tube is a torus or
    // a procedural sweep surface): the failure comes from the join of the tool with that face, not from the radius, so a
    // smaller radius fails the same way and the caller must not search sizes for it.
    if let Err(e) = &whole {
        let curved_rim = edges.iter().any(|(fa, fb, at)| {
            find_edge(b, fa, fb, *at).is_some_and(|ed| {
                matches!(b.edge(ed).curve, Curve::Circle { .. })
                    && b.edge_faces(ed).iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. }))
            })
        });
        if curved_rim && e.code == "blendFailed" && e.detail.as_deref().is_some_and(|d| d.starts_with("select:")) {
            return Err(failed_hint(
                "the rounding of a rim on a curved side face (a swept tube: a torus or sweep surface) could not be joined to that face; this does not depend on the radius",
                "a smaller radius will not help: leave this rim sharp, or build the rounded end into the shape (for example a revolved profile that includes the rounded end)",
            ));
        }
    }
    // Faces of a twisted extrude are screw surfaces. A failure there is not the generic "smaller radius" one (a ring of
    // end edges at a twist of 60 degrees and over leaves the last chains' tools open, issue 67).
    if let Err(e) = &whole {
        let on_screw = edges.iter().any(|(fa, fb, at)| find_edge(b, fa, fb, *at).is_some_and(|ed| b.edge_faces(ed).iter().any(|&f| matches!(b.face(f).surface, Surface::Screw { .. }))));
        if on_screw && e.code == "blendFailed" {
            return Err(failed_hint(
                "a fillet on a twisted extrude face: the blend could not be joined to the twisted (screw) side faces",
                "a ring of end edges on a strongly twisted extrude (about 60 degrees and over) may not close: fillet fewer edges at a time, fillet before the twist, or build the rounded edge with other features",
            ));
        }
    }
    whole
}

/// A seam vertex of a wall split in two faces (the next edge of the selection, or the rim's other half, carries on tangentially
/// along faces that share no face with this edge's but continue them), if a selected edge meets one.
fn split_skin_rim(b: &Body, edges: &[(String, String, Vec3)]) -> Option<Vec3> {
    let sel: Vec<EdgeId> = edges.iter().filter_map(|(fa, fb, at)| find_edge(b, fa, fb, *at)).collect();
    for &e in &sel {
        let ed = b.edge(e);
        let fe = b.edge_faces(e);
        if fe.len() != 2 {
            continue;
        }
        for v in [ed.v0, ed.v1] {
            for x in b.edge_ids().filter(|&x| x != e && (b.edge(x).v0 == v || b.edge(x).v1 == v)) {
                let fx = b.edge_faces(x);
                if fx.len() == 2 && !fx.iter().any(|f| fe.contains(f)) && continues_at(b, e, x, v) && continues_smoothly(b, [fe[0], fe[1]], [fx[0], fx[1]]).is_some() {
                    return Some(b.vertex(v).p);
                }
            }
        }
    }
    None
}

fn fillet_freeform_chains(b: &Body, edges: &[(String, String, Vec3)], r: f64, feature: &str, tol: f64, split_sharp: bool) -> Result<Body, BlendError> {
    HINTS.with(|m| m.borrow_mut().clear());
    BOXES.with(|m| m.borrow_mut().clear());
    REACH.with(|c| c.set(r));
    let mut cur = b.clone();
    let mut remaining: Vec<usize> = (0..edges.len()).collect();
    while let Some(&first) = remaining.first() {
        // The per-face caches below are keyed by FaceId. Each chain's blend is a boolean that rebuilds the body, so
        // face ids change meaning between chains; a stale box or warm start would prune the face that now owns the
        // id or aim closest_point_near at another edge's surface. Clear them for each chain (the same chain never
        // changes the body, so its own warm starts stay valid).
        HINTS.with(|m| m.borrow_mut().clear());
        BOXES.with(|m| m.borrow_mut().clear());
        let eids: Vec<EdgeId> = remaining
            .iter()
            .map(|&k| {
                find_edge(&cur, &edges[k].0, &edges[k].1, edges[k].2).ok_or_else(|| {
                    // gone after an earlier chain of this blend although the input has it: that chain's blend consumed
                    // the face beside it (a round wider than the face between two picked edges)
                    if remaining.len() < edges.len() && find_edge(b, &edges[k].0, &edges[k].1, edges[k].2).is_some() {
                        super::diag::consumed_by_earlier(feature, k as u32, &edges[k].0, &edges[k].1)
                    } else {
                        BlendError::code("missingEdge")
                    }
                })
            })
            .collect::<Result<_, _>>()?;
        let (chain, closed) = build_chain(&cur, &eids, 0, split_sharp)?;
        let picked: Vec<usize> = chain.iter().map(|&(i, _)| remaining[i]).collect();
        let links = make_links(&cur, &eids, &chain)?;
        let k = *picked.iter().min().unwrap_or(&first) as u32;
        // (split runs: a later run starts where an earlier blend ended, up to about r short of the corner both meet at;
        // its tool must run back past that corner, or the mitre is left with a ledge)
        cur = blend_chain(&cur, links, r, feature, k, tol, closed, if split_sharp { 2.5 } else { 0.5 })?;
        remaining.retain(|x| !picked.contains(x));
    }
    // The boolean fits a pcurve on the blend surface for each new edge. On the contact edges the two faces are
    // tangent by construction, so `refit_edges_on_faces` leaves the fitted pcurve alone (grazing faces keep their
    // curve); on the daisy petals its deviation (~2e-4) is just over the validator's limit. The contact curves lie
    // on the blend surface exactly, so drop the pcurves on the blend faces and let the validator project instead.
    // A later chain's boolean rebuilds the body and can re-fit pcurves on an earlier chain's blend face, so this
    // runs once at the end over every blend face of this feature, not once per chain. The boolean may rename a
    // split face (`Role::Blend` vs `Role::Other("B0")` both print `F90.B0`), so match the wire id prefix, not the role.
    let prefix = format!("{feature}.B");
    let mut dropped = 0usize;
    for e in cur.edge_ids().collect::<Vec<_>>() {
        let mut touched = false;
        for c in cur.edge(e).coedges.clone() {
            if cur.face(cur.coedge_face(c)).name.id().starts_with(&prefix) {
                cur.coedge_mut(c).pcurve = None;
                dropped += 1;
                touched = true;
            }
        }
        if touched {
            // The contact edge's 3D curve is the boolean's fit of the intersection curve; its disagreement with the
            // B-spline blend surface is ~2e-4 on the daisy petals. Dropping the pcurve leaves the projection with that
            // deviation, which is just over the validator's `tol * scale * 10` limit. The edge tolerance is the
            // designed record of a fitted curve's fit error (see `Edge::tol`), so carry the measured deviation there.
            let dev = ck_check::face_deviation(&cur, e, 8);
            let want = dev * 1.05;
            if want > cur.edge(e).tol && want <= 0.04 {
                cur.edge_mut(e).tol = want;
            }
        }
    }
    dbg(&format!("drop pcurves: prefix {prefix}, dropped {dropped}"));
    Ok(cur)
}

/// Do the faces `next` carry on the faces `prev` of a chain across edges where each pair is tangent-continuous (the
/// same skin split in two)? Returns `next` ordered like `prev`.
pub fn continues_smoothly(b: &Body, prev: [FaceId; 2], next: [FaceId; 2]) -> Option<(FaceId, FaceId)> {
    let smooth = |f: FaceId, g: FaceId| -> bool {
        b.face_edges(f).into_iter().any(|e| {
            let fs = b.edge_faces(e);
            if !(fs.contains(&f) && fs.contains(&g)) {
                return false;
            }
            let ed = b.edge(e);
            let p = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
            match (foot(b, f, p), foot(b, g, p)) {
                (Some((_, n1)), Some((_, n2))) => n1.dot(n2) > 0.995,
                _ => false,
            }
        })
    };
    if smooth(prev[0], next[0]) && smooth(prev[1], next[1]) {
        Some((next[0], next[1]))
    } else if smooth(prev[0], next[1]) && smooth(prev[1], next[0]) {
        Some((next[1], next[0]))
    } else {
        None
    }
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
            } else if let Some((a, c)) = continues_smoothly(b, [prev.f1, prev.f2], [fs[0], fs[1]]) {
                // a wall split into several faces of one smooth skin (a thickened closed spline: each wall and each cap is
                // two faces meeting at the seam vertices): the next link's faces carry on the previous ones
                (f1, f2) = (a, c);
            } else {
                return Err(unsupported("chain faces do not continue"));
            }
        }
        links.push(Link { e, fwd, f1, f2 });
    }
    Ok(links)
}

#[allow(clippy::too_many_arguments)]
fn blend_chain(b: &Body, links: Vec<Link>, r: f64, feature: &str, k: u32, tol: f64, closed: bool, end_ext: f64) -> Result<Body, BlendError> {
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
    dbg(&format!("chain: {} links, closed={closed}, {} + {} faces", links.len(), set1.len(), set2.len()));
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
    let mut kk = 0.5 * r;
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
            let st = ctx.solve(li, s, guess).map_err(|e| {
                dbg(&format!(
                    "station solve failed li={li} s={s:.4} p=({:.4},{:.4},{:.4}): {}",
                    p.x, p.y, p.z, e.detail.as_deref().unwrap_or(&e.code)
                ));
                e
            })?;
            prev = Some(st.clone());
            nodes.push(Node { li, s, st });
        }
    }
    // A concave blend is built as a union with the tool: the section polygon reaches `kk` past the two contact points into the
    // material (and diagonally past the corner). Where the wall is thinner than that (a rim 5 mm thick, a 10 mm round) the
    // tool would come out of the far side and stay there as a bulge, valid but wrong: shorten the reach until the polygon's
    // outer corners lie in the material.
    if !convex {
        let step = (nodes.len() / 4).max(1);
        let in_material = |k: f64| {
            nodes.iter().step_by(step).chain(nodes.last()).all(|n| match section(&n.st, sg, r, k) {
                Ok(sc) => [NV, NV + 1, NV + 2].iter().all(|&j| ck_topo::region::classify_near(b, sc.pts[j], 1e-3) != ck_topo::region::Containment::Outside),
                Err(_) => true,
            })
        };
        for f in [1.0, 0.6, 0.35, 0.2, 0.1] {
            kk = 0.5 * r * f;
            if in_material(kk) {
                break;
            }
        }
    }
    if super::diag::on_level(2) {
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
        let mut worst_at = 0usize;
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
            if dev > worst {
                worst_at = i;
            }
            worst = worst.max(dev);
            if dev > FIT_TOL {
                insert.push((i, li, sm));
            }
        }
        dbg(&format!(
            "fit round {round}: {} nodes, worst on-face deviation {worst:.2e} at node {worst_at} (link {} s {:.4}, p=({:.3},{:.3},{:.3}))",
            nodes.len(), nodes[worst_at].li, nodes[worst_at].s, nodes[worst_at].st.p.x, nodes[worst_at].st.p.y, nodes[worst_at].st.p.z
        ));
        if insert.is_empty() {
            break;
        }
        for &(i, li, sm) in insert.iter().rev() {
            let guess = Some((nodes[i].st.c + nodes[i + 1].st.c) * 0.5);
            // (the warm starts are those of the last node solved, which is far from this one: stale on a closed spline face)
            HINTS.with(|m| m.borrow_mut().clear());
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
    // A ring starts and closes at a vertex of the body, where the faces of a split wall meet. A tool whose seam sits on
    // that vertex has its seam lines run along the body's own edge, and the imprint cannot tell the two apart. Start
    // the ring in the middle of its first link instead.
    let mut ring_start = false;
    // the section where the ring tool's second half starts (the first half starts at section 0): the middle of the
    // opposite link, again away from the body's own vertices
    let mut ring_split = nodes.len() / 2;
    if closed && ctx.links.len() >= 2 {
        let first: Vec<usize> = (0..nodes.len()).filter(|&i| nodes[i].li == 0).collect();
        if first.len() >= 3 {
            let m = first[first.len() / 2];
            nodes.pop();
            nodes.rotate_left(m);
            let head = nodes[0].clone();
            nodes.push(head);
            ring_start = true;
            let opposite: Vec<usize> = (0..nodes.len()).filter(|&i| nodes[i].li == ctx.links.len() / 2).collect();
            if opposite.len() >= 3 {
                ring_split = opposite[opposite.len() / 2];
            }
        }
    }
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
    if closed {
        // a ring has no ends: the sweep closes on itself. Force the last
        // section onto the first so the fitted strip surfaces (and their rails)
        // are exactly closed in u.
        let last = nodes.len() - 1;
        nodes[last].st = nodes[0].st.clone();
        secs[last] = secs[0].clone();
        ps[last] = ps[0];
    } else if convex {
        let m = end_ext * r;
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
                // a chain end on a twisted face: the ball is carried on along the faces (a straight shift of the
                // section leaves the twisted face, and its contact curve then misses the face's boundary edge)
                let on_screw = [ctx.links[nd.li].f1, ctx.links[nd.li].f2].iter().any(|&f| matches!(b.face(f).surface, Surface::Screw { .. }));
                let carried = on_screw
                    .then(|| ctx.solve_with(nd.li, nd.s, Some(nd.st.c + d), Some((nd.st.p + d, t_out))).and_then(|st| section(&st, sg, r, kk)))
                    .and_then(|x| x.ok());
                let sh = carried.unwrap_or_else(|| shift(&secs[idx], d));
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
            let (ev, fs) = end_info(end);
            let plane = fs.iter().find_map(|&f| match &b.face(f).surface {
                Surface::Plane { frame } => Some((frame.o, b.planar_normal(f))),
                _ => None,
            });
            let Some((o, n)) = plane else {
                // (name the faces the end runs into: at a blade root it is the hub, at the sharp tip of a drafted
                // pocket the floor or a cone, and the remedy differs)
                let kinds: Vec<&str> = fs.iter().map(|&f| b.face(f).surface.kind()).collect();
                if fs.iter().any(|&f| matches!(b.face(f).surface, Surface::Torus { .. })) {
                    // the end runs into the torus of an earlier round (a round of the top edge of a plate that this edge
                    // meets): the ball would have to leave the face onto that blend, and no corner patch joins the two
                    let p = b.vertex(ev).p;
                    let hint = format!(
                        "this round ends at ({:.3}, {:.3}, {:.3}) on the torus of an earlier round, and a concave round is only built up to a flat face, not onto another round (at any radius). Leave the edges that run into the earlier round out of this selection (the others, whose ends lie on flat faces, still round), or leave the earlier round off this corner",
                        p.x, p.y, p.z
                    );
                    return Err(unsupported_hint("concave chain end is not on a planar face", &hint));
                }
                let hint = format!(
                    "a concave free-form blend ends only on a plane, and this end meets {} ({}). Select the edges the blend runs on into past this end so the chain is closed (at a blade root: every blade-face / hub edge of the loop), or round the corner first so the edge ends on a flat face (a sharp tip between two walls: give it a small radius in the sketch)",
                    if kinds.len() == 1 { "a curved face" } else { "curved faces" },
                    kinds.join(", ")
                );
                return Err(unsupported_hint("concave chain end is not on a planar face", &hint));
            };
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
            if super::diag::on() {
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
    let mut junction_u: Vec<f64> = (1..ctx.links.len())
        .filter_map(|l| nodes.iter().rposition(|n| n.li == l - 1).map(|i| fit.u[i + pre]))
        .collect();
    if ring_start {
        // (the ring was rotated: a junction is where a link's first station is, and the first link's start is mid-ring)
        junction_u = (1..ctx.links.len()).filter_map(|l| nodes.iter().position(|n| n.li == l).map(|i| fit.u[i + pre])).collect();
        if let Some(i) = nodes.iter().position(|n| n.li == 0 && n.s == 0.0) {
            junction_u.push(fit.u[i + pre]);
        }
        junction_u.sort_by(|a, c| a.total_cmp(c));
    } else if closed {
        // the junction where the ring closes (last link meets the first)
        junction_u.push(1.0);
    }
    // A ring is fitted and built as two sweeps (sections 0 to `m`, `m` back to 0): the strips are B-spline surfaces, which have
    // no period, and a closed one has its first and last rows on one curve, so a point there projects to either end.
    let ring_m = ring_split.clamp(1, secs.len().saturating_sub(2).max(1));
    let halves: Option<[Fit; 2]> = if closed { Some([fit_cols(&secs[..=ring_m], &ps[..=ring_m])?, fit_cols(&secs[ring_m..], &ps[ring_m..])?]) } else { None };
    let mut tool = build_tool(&fit, &secs, feature, k, plane_caps, b, &ctx.links, &end_info, tol, closed, ring_m, halves.as_ref())?;
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
    // (a ring tool has two faces of each name, one per half)
    let named = |n: FaceName| -> Vec<FaceId> { tool.face_ids().filter(|&f| tool.face(f).name == n).collect() };
    let blend_fs = named(blend_name.clone());
    let leg_as = named(FaceName::other(feature, "TA4"));
    let leg_bs = named(FaceName::other(feature, "TA1"));
    let Some(&blend_f) = blend_fs.first() else { return Err(failed("tool lost its blend face")) };
    if leg_as.is_empty() {
        return Err(failed("tool leg A"));
    }
    if leg_bs.is_empty() {
        return Err(failed("tool leg B"));
    }
    // the contact curves, one per tool half (a chain's tool is one piece, a ring's two)
    let (curves_a, curves_b): (Vec<Curve>, Vec<Curve>) = match &halves {
        Some(hs) => (hs.iter().map(|h| Curve::BSpline(h.cols[0].clone())).collect(), hs.iter().map(|h| Curve::BSpline(h.cols[NV - 1].clone())).collect()),
        None => (vec![Curve::BSpline(fit.cols[0].clone())], vec![Curve::BSpline(fit.cols[NV - 1].clone())]),
    };
    let hook = |f: FaceId, g: FaceId| -> Option<Vec<Curve>> {
        if blend_fs.contains(&g) && (f1set.contains(&f) || f2set.contains(&f)) {
            return Some(vec![]);
        }
        if let Some(i) = leg_as.iter().position(|&x| x == g) {
            if f1set.contains(&f) {
                return Some(vec![curves_a[i.min(curves_a.len() - 1)].clone()]);
            }
        }
        if let Some(i) = leg_bs.iter().position(|&x| x == g) {
            if f2set.contains(&f) {
                return Some(vec![curves_b[i.min(curves_b.len() - 1)].clone()]);
            }
        }
        None
    };
    let op = if convex { Op::Cut } else { Op::Union };
    let (mut out, _) = apply_tool_hooked(b, &tool, op, &blend_name, hint, tol.max(FF_TOL), Some(&hook))?;
    // the imprint welded at FF_TOL: put the vertices back on their faces and
    // re-fit edges to them, so the result validates at the model tolerance
    // (the blend surface a junction arc is cut from, and its parameter on it: a ring has one surface per half)
    let (surfs, junctions): (Vec<BSplineSurface>, Vec<(usize, f64)>) = match &halves {
        Some(hs) => {
            let um = fit.u[ring_m];
            let js = junction_u.iter().map(|&u| if u <= um + 1e-12 { (0, u / um) } else { (1, (u - um) / (1.0 - um)) }).collect();
            (hs.iter().filter_map(|h| blend_patch(h).ok()).collect(), js)
        }
        None => (
            match &tool.face(blend_f).surface {
                Surface::BSpline(bs) => vec![(**bs).clone()],
                _ => vec![],
            },
            junction_u.iter().map(|&u| (0, u)).collect(),
        ),
    };
    split_blend_at_links(&mut out, &blend_name, &surfs, &junctions, tol.max(FF_TOL));
    // (vertices that coincide with a tool vertex take the tool's exact spot:
    // it lies on all the faces it joins)
    if super::diag::on_level(3) {
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
    // (only slivers the imprint made: a short edge of the input between two of its own vertices, far from the blend
    // or not, is model geometry; collapsing one moved its neighbours' ends by its length and left their edges off
    // their faces, e.g. a 7.5e-4 mm edge of the fan shroud's grille ring)
    let input_verts: Vec<Vec3> = b.vertex_ids().map(|v| b.vertex(v).p).collect();
    collapse_short_edges_keeping(&mut out, 6.0 * FF_TOL, &input_verts);
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
    closed: bool,
    split: usize,
    halves: Option<&[Fit; 2]>,
) -> Result<Body, BlendError> {
    let nu = secs.len();
    let mut tool = Body::new();
    let lump = tool.add_lump();
    let shell = tool.add_shell(lump);
    let (u0, u1) = (fit.ku[0], fit.ku[fit.ku.len() - 1]);
    let name_of = |s: usize| if s == 0 { FaceName::blend(feature, k) } else { FaceName::other(feature, &format!("TA{s}")) };
    if let (true, Some(halves)) = (closed, halves) {
        // A ring: the sweep closes on itself (`secs` first == last), built as two sweeps (sections 0 to `split`, `split` back
        // to 0), each with its own strips, like the two halves of the walls of a closed spline sketch: ten faces, the cross
        // edges of the two sections shared by the halves, no caps. (A band that closed on itself would be one face with a
        // seam, which a B-spline surface cannot place.)
        let m = split.clamp(1, nu - 2);
        let blends: Vec<BSplineSurface> = halves.iter().map(blend_patch).collect::<Result<_, _>>()?;
        let mut strips: Vec<Vec<Surface>> = vec![];
        for (h, ft) in halves.iter().enumerate() {
            strips.push(vec![
                Surface::BSpline(Box::new(blends[h].clone())),
                Surface::BSpline(Box::new(ruled(ft, TRACKS[1], TRACKS[2])?)),
                Surface::BSpline(Box::new(ruled(ft, TRACKS[2], TRACKS[3])?)),
                Surface::BSpline(Box::new(ruled(ft, TRACKS[3], TRACKS[4])?)),
                Surface::BSpline(Box::new(ruled(ft, TRACKS[4], TRACKS[0])?)),
            ]);
        }
        // the vertices of the two sections, and the rails between them (half 0: section 0 to `split`, half 1: back)
        let sec_v: Vec<[ck_topo::body::VertexId; 5]> = [0usize, m]
            .iter()
            .map(|&i| {
                let mut a = [ck_topo::body::VertexId(0); 5];
                for (t, &c) in TRACKS.iter().enumerate() {
                    a[t] = tool.add_vertex(secs[i].pts[c]);
                }
                a
            })
            .collect();
        let rails: Vec<Vec<EdgeId>> = (0..2)
            .map(|h| {
                (0..5)
                    .map(|t| {
                        let (ta, tb) = (halves[h].ku[0], halves[h].ku[halves[h].ku.len() - 1]);
                        let (va, vb) = if h == 0 { (sec_v[0][t], sec_v[1][t]) } else { (sec_v[1][t], sec_v[0][t]) };
                        tool.add_edge(Curve::BSpline(halves[h].cols[TRACKS[t]].clone()), ta, tb, va, vb)
                    })
                    .collect()
            })
            .collect();
        // the cross edges of a section: the first side is the blend patch's arc at that u, the others straight
        let arcs = [blends[0].iso_u(halves[0].ku[0]), blends[0].iso_u(halves[0].ku[halves[0].ku.len() - 1])];
        let cross: Vec<Vec<EdgeId>> = [0usize, 1]
            .iter()
            .map(|&si| {
                (0..5)
                    .map(|t| {
                        let (va, vb) = (sec_v[si][t], sec_v[si][(t + 1) % 5]);
                        let (pa, pb) = (tool.vertex(va).p, tool.vertex(vb).p);
                        if t == 0 {
                            let arc = arcs[si].clone();
                            let (t0, t1) = (arc.domain().0, arc.domain().1);
                            tool.add_edge(Curve::BSpline(arc), t0, t1, va, vb)
                        } else {
                            let d = pb - pa;
                            let len = d.len();
                            tool.add_edge(Curve::Line { o: pa, d: d / len.max(1e-300) }, 0.0, len, va, vb)
                        }
                    })
                    .collect()
            })
            .collect();
        // pcurves: the rails are iso-v lines of a strip (u is the edge parameter), the cross edges iso-u lines (the
        // first side's arc has v as its parameter, the others run over their length)
        let (va0, va1) = (arcs[0].domain().0, arcs[0].domain().1);
        for h in 0..2 {
            let (ua, ub) = (halves[h].ku[0], halves[h].ku[halves[h].ku.len() - 1]);
            // the section at the start of the half and at its end
            let (sa, sb) = if h == 0 { (0usize, 1usize) } else { (1, 0) };
            for t in 0..5 {
                let name = if t == 4 { FaceName::other(feature, "TA4") } else { name_of(t) };
                let f = tool.add_face(shell, strips[h][t].clone(), true, name);
                let (vlo, vhi) = if t == 0 { (va0, va1) } else { (0.0, 1.0) };
                let rail = |v: f64| Curve2::Line { o: Vec2::new(0.0, v), d: Vec2::new(1.0, 0.0) };
                let clen = |si: usize| if t == 0 { 1.0 } else { (tool.vertex(sec_v[si][(t + 1) % 5]).p - tool.vertex(sec_v[si][t]).p).len().max(1e-300) };
                let across = |u: f64, len: f64| Curve2::Line { o: Vec2::new(u, 0.0), d: Vec2::new(0.0, 1.0 / len) };
                let (la, lb) = (clen(sa), clen(sb));
                let lp = tool.add_loop(f);
                tool.add_coedge(lp, rails[h][t], false, Some(rail(vlo)));
                tool.add_coedge(lp, cross[sb][t], false, Some(across(ub, lb)));
                tool.add_coedge(lp, rails[h][(t + 1) % 5], true, Some(rail(vhi)));
                tool.add_coedge(lp, cross[sa][t], true, Some(across(ua, la)));
            }
        }
        let (_, vol) = ck_topo::orient::orient_body_volume_quick(&mut tool);
        // The faces are bands without a seam edge (their two loops are the closed rails), which the volume integral cannot
        // read: it comes out as rounding noise of either sign, so it cannot say which way the shell faces. The tool lies between
        // the blend arc and the section's far corner: the blend face's outward normal points away from that corner.
        let blend_f = tool.face_ids().find(|&f| tool.face(f).name == FaceName::blend(feature, k)).ok_or_else(|| failed("ring tool lost its blend face"))?;
        // (at the start of the blend face's own domain, where the first section lies: a ring built as two sweeps has a blend
        // face per half, and the middle of the whole fit lies on the other one)
        let (um, vm) = (tool.face(blend_f).surface.domain().0 .0, 0.5);
        let m = tool.face(blend_f).surface.eval(um, vm);
        let n = tool.outward_normal(blend_f, um, vm);
        let corner = secs[0].pts[TRACKS[3]];
        if !vol.is_finite() || n.dot(corner - m) == 0.0 {
            return Err(failed(&format!("ring tool volume {vol}")));
        }
        if n.dot(corner - m) > 0.0 {
            for f in tool.face_ids().collect::<Vec<_>>() {
                ck_topo::orient::flip_face(&mut tool, f);
            }
        }
        if super::diag::on() {
            dbg(&format!("ring tool built: {nu} stations, volume {vol:e} (mass {:e}), issues {:?}", ck_topo::mass::mass_props(&tool).volume, ck_topo::check::check(&tool, 1e-5).len()));
        }
        return Ok(tool);
    }
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
    // (the outward test of the orienting already integrated the volume: no second pass of moments over the same faces)
    let (_, vol) = ck_topo::orient::orient_body_volume_quick(&mut tool);
    if !(vol > 0.0) {
        return Err(failed(&format!("tool volume {vol}")));
    }
    if super::diag::on() {
        // (the validation pass only runs when its line is printed)
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
    collapse_short_edges_keeping(out, max_len, &[]);
}

/// [`collapse_short_edges`], leaving alone the edges whose two end vertices both sit (within 1e-7) on points of `keep`
/// (the vertices of the body before the operation: such an edge is the input's own geometry, not a welding sliver).
pub fn collapse_short_edges_keeping(out: &mut Body, max_len: f64, keep: &[Vec3]) {
    drop_zero_edges(out, max_len);
    let kept = |p: Vec3| keep.iter().any(|q| q.dist(p) <= 1e-7);
    loop {
        let cand = out.edge_ids().find(|&e| {
            let ed = out.edge(e);
            ed.v0 != ed.v1
                && out.vertex(ed.v0).p.dist(out.vertex(ed.v1).p) < max_len
                && out.edge_length(e) < 3.0 * max_len
                && !(kept(out.vertex(ed.v0).p) && kept(out.vertex(ed.v1).p))
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
fn split_blend_at_links(out: &mut Body, name: &FaceName, surfs: &[BSplineSurface], junctions: &[(usize, f64)], tol: f64) {
    for &(h, uj) in junctions {
        let Some(bs) = surfs.get(h) else { continue };
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
