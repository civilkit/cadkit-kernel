//! The undo of a fillet or chamfer on the file's solid, for a part that starts with imported bodies (`undo.rs`).
//!
//! A round or chamfer of an edge between two support faces takes away (on a convex edge) or fills in (on a concave
//! one) the material between its strip face and the sharp edge the supports meet at. In a section across the strip
//! (a plane normal to the edge for a straight edge, a half plane through the axis for a circular one) the supports are
//! two lines meeting at the corner `P`, the strip is an arc tangent to both at `TA` and `TB` (a fillet) or the chord
//! from `TA` to `TB` (a chamfer), and that material lies inside the triangle `P TA TB`. The rest of the triangle is
//! on the other side of the strip: material of the body where the blend removed the corner, empty where it filled
//! one. So the triangle swept along the strip (extruded along a straight edge, revolved about the axis of a circular
//! one), added to the body for a convex blend and cut from it for a concave one, puts the sharp edge back, and the
//! blend run again on that body gives the file's faces back.
//!
//! Handled: cylinder and plane strips along a straight edge between two planes parallel to it; torus and cone strips
//! about an axis between two planes, cylinders or cones about the same axis. Any other face of the blend (a corner
//! patch, a sphere, a spline strip) leaves the step not undone. The tools are not checked here: the whole history,
//! with the blend run again on the undone body, is verified against the file's solid as every undo is.

use ck_geom::Vec3;
use ck_json::{Json, Obj};
use ck_recog::emit::{arr, n, vec3};
use ck_recog::model::{Kind, Model};
use ck_topo::body::FaceId;
use std::collections::HashSet;

type P = (f64, f64);

/// A line `n . X = k` of the section (`n` a unit vector).
#[derive(Clone, Copy, Debug)]
struct Line {
    n: P,
    k: f64,
}

impl Line {
    fn new(n: P, k: f64) -> Option<Line> {
        let l = n.0.hypot(n.1);
        (l > 1e-9).then(|| Line { n: (n.0 / l, n.1 / l), k: k / l })
    }
    fn dist(&self, p: P) -> f64 {
        self.n.0 * p.0 + self.n.1 * p.1 - self.k
    }
    fn foot(&self, p: P) -> P {
        let d = self.dist(p);
        (p.0 - self.n.0 * d, p.1 - self.n.1 * d)
    }
    fn same(&self, o: &Line, tol: f64) -> bool {
        ((self.n.0 - o.n.0).abs() < 1e-7 && (self.n.1 - o.n.1).abs() < 1e-7 && (self.k - o.k).abs() < tol) || ((self.n.0 + o.n.0).abs() < 1e-7 && (self.n.1 + o.n.1).abs() < 1e-7 && (self.k + o.k).abs() < tol)
    }
}

fn meet(a: Line, b: Line) -> Option<P> {
    let det = a.n.0 * b.n.1 - a.n.1 * b.n.0;
    if det.abs() < 1e-6 {
        return None;
    }
    Some(((a.k * b.n.1 - b.k * a.n.1) / det, (a.n.0 * b.k - b.n.0 * a.k) / det))
}

/// The section the triangle lies in.
#[derive(Clone, Copy)]
enum Sec {
    /// A plane normal to `d` (2D axes `e1`, `e2 = d x e1` about `o`): the triangle is extruded along `d`.
    Prism { o: Vec3, d: Vec3, e1: Vec3, e2: Vec3 },
    /// The half planes through the axis (`c`, `a`): `(radius, height)`; the triangle is revolved about the axis.
    Revolve { c: Vec3, a: Vec3 },
}

/// Some vector normal to the unit `a`.
fn normal_to(a: Vec3) -> Vec3 {
    let t = if a.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    (t - a * t.dot(a)).normalize()
}

impl Sec {
    /// The point in section coordinates (a revolve section: the half plane through the point).
    fn xy(&self, p: Vec3) -> P {
        match *self {
            Sec::Prism { o, e1, e2, .. } => ((p - o).dot(e1), (p - o).dot(e2)),
            Sec::Revolve { c, a } => {
                let w = p - c;
                let h = w.dot(a);
                ((w - a * h).len(), h)
            }
        }
    }

    /// The trace of a face's surface on the section, when it is a line there.
    fn line(&self, k: &Kind) -> Option<Line> {
        match (*self, k) {
            (Sec::Prism { o, d, e1, e2 }, Kind::Plane { n: pn, d: pd }) if pn.dot(d).abs() < 1e-9 => Line::new((pn.dot(e1), pn.dot(e2)), pd - pn.dot(o)),
            (Sec::Revolve { c, a }, Kind::Plane { n: pn, d: pd }) if pn.cross(a).len() < 1e-9 => {
                let h = (*pn * *pd - c).dot(a);
                Line::new((0.0, 1.0), h)
            }
            (Sec::Revolve { c, a }, Kind::Cyl { o, a: ca, r, .. }) if coaxial(c, a, *o, *ca) => Line::new((1.0, 0.0), *r),
            (Sec::Revolve { c, a }, Kind::Cone { o, a: ca, r0, half, .. }) if coaxial(c, a, *o, *ca) => {
                // radius r0 + h' tan(half) at the height h' along the cone's own axis from its origin
                let sg = ca.dot(a).signum();
                let t = half.tan();
                let zo = (*o - c).dot(a);
                Line::new((1.0, -sg * t), r0 - sg * t * zo)
            }
            _ => None,
        }
    }
}

fn coaxial(c: Vec3, a: Vec3, o: Vec3, b: Vec3) -> bool {
    a.cross(b).len() < 1e-9 && (o - c).cross(a).len() < 1e-6 * (1.0 + (o - c).len())
}

/// Sample points of the boundary of a face: every edge at its ends and along it.
fn boundary(m: &Model, f: FaceId) -> Vec<Vec3> {
    let mut out = vec![];
    for e in m.b.face_edges(f) {
        let ed = m.b.edge(e);
        for i in 0..=32 {
            out.push(ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / 32.0));
        }
    }
    out
}

/// The tool of one strip: the sketch plane's frame (origin, x, y), the triangle, and how it is swept.
struct Tool {
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    tri: [P; 3],
    /// Extrude depth along the frame normal, or the revolve angle (degrees) about the frame's y axis.
    sweep: Sweep,
    add: bool,
}

enum Sweep {
    Extrude(f64),
    Revolve(f64),
}

/// The tool that undoes the blend at strip face `f` (`own`: the blend's faces), or `None`.
fn strip_tool(m: &Model, f: FaceId, own: &HashSet<FaceId>, chamfer: bool) -> Option<Tool> {
    let k = m.kind(f).clone();
    // the section, and the strip's trace on it (a circle for a fillet, a line for a chamfer)
    let (sec, circle, sline): (Sec, Option<(P, f64)>, Option<Line>) = match k {
        Kind::Cyl { o, a, r, .. } if !chamfer => {
            let e1 = normal_to(a);
            let sec = Sec::Prism { o, d: a, e1, e2: a.cross(e1) };
            (sec, Some(((0.0, 0.0), r)), None)
        }
        Kind::Torus { c, a, major, minor, .. } if !chamfer => (Sec::Revolve { c, a }, Some(((major, 0.0), minor)), None),
        Kind::Plane { n: pn, .. } if chamfer => {
            // the strip runs along the edges it shares with plane neighbours that are parallel to each other and lie
            // in the strip's plane (its rails)
            let mut dir = None;
            for e in m.edges(f) {
                let (p0, p1) = (m.p0(e), m.p1(e));
                if matches!(m.curve(e), ck_geom::curve::Curve::Line { .. }) && p0.dist(p1) > 1e-6 && m.across(e, f).is_some_and(|g| !own.contains(&g) && matches!(m.kind(g), Kind::Plane { .. })) {
                    let d = (p1 - p0).normalize();
                    if m.edges(f).into_iter().filter(|&x| x != e).any(|x| {
                        let (q0, q1) = (m.p0(x), m.p1(x));
                        q0.dist(q1) > 1e-6 && matches!(m.curve(x), ck_geom::curve::Curve::Line { .. }) && (q1 - q0).normalize().cross(d).len() < 1e-7 && (q0 - p0).cross(d).len() > 1e-6
                    }) {
                        dir = Some(d);
                        break;
                    }
                }
            }
            let d = dir?;
            if d.dot(pn).abs() > 1e-9 {
                return None;
            }
            let o = m.p0(m.edges(f)[0]);
            let e1 = normal_to(d);
            let sec = Sec::Prism { o, d, e1, e2: d.cross(e1) };
            (sec, None, Some(sec.line(&k)?))
        }
        Kind::Cone { o, a, .. } if chamfer => {
            let sec = Sec::Revolve { c: o, a };
            (sec, None, Some(sec.line(&k)?))
        }
        _ => return None,
    };
    let scale = 1.0 + circle.map_or(0.0, |c| c.1);
    let tol = 1e-6 * scale.max(1.0);
    // the supports: neighbours whose surface is a line of the section, tangent to the arc (a fillet) or meeting the
    // chord (a chamfer) where they share an edge with the strip
    let mut sup: Vec<Line> = vec![];
    for e in m.edges(f) {
        let Some(g) = m.across(e, f) else { continue };
        if own.contains(&g) {
            continue;
        }
        let Some(l) = sec.line(m.kind(g)) else { continue };
        let q = sec.xy(m.mid(e));
        let ok = match (circle, sline) {
            (Some((c, r)), _) => (l.dist(c).abs() - r).abs() < tol && l.dist(q).abs() < 1e-5 * scale,
            (None, Some(sl)) => l.dist(q).abs() < 1e-5 * scale && sl.dist(q).abs() < 1e-5 * scale && meet(l, sl).is_some(),
            _ => false,
        };
        if ok && !sup.iter().any(|x| x.same(&l, tol)) {
            sup.push(l);
        }
    }
    let [la, lb] = sup[..] else { return None };
    let p = meet(la, lb)?;
    let (ta, tb) = match (circle, sline) {
        (Some((c, _)), _) => (la.foot(c), lb.foot(c)),
        (None, Some(sl)) => (meet(la, sl)?, meet(lb, sl)?),
        _ => return None,
    };
    let area = ((ta.0 - p.0) * (tb.1 - p.1) - (ta.1 - p.1) * (tb.0 - p.0)).abs();
    let size = ta.0.hypot(ta.1).max(tb.0.hypot(tb.1)).max(1.0);
    if area < 1e-9 * size * size {
        return None;
    }
    // the strip's point nearest the corner, and on which side of the strip the corner lies
    let mpt = match circle {
        Some((c, r)) => {
            let v = (p.0 - c.0, p.1 - c.1);
            let l = v.0.hypot(v.1);
            if l <= r {
                return None;
            }
            (c.0 + v.0 / l * r, c.1 + v.1 / l * r)
        }
        None => ((ta.0 + tb.0) / 2.0, (ta.1 + tb.1) / 2.0),
    };
    let pts = boundary(m, f);
    match sec {
        Sec::Prism { o, d, e1, e2 } => {
            let ss: Vec<f64> = pts.iter().map(|q| (*q - o).dot(d)).collect();
            let (s0, s1) = ss.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |a, &x| (a.0.min(x), a.1.max(x)));
            if !(s1 - s0 > 1e-6) {
                return None;
            }
            let at = |q: P, s_: f64| o + e1 * q.0 + e2 * q.1 + d * s_;
            let sm = 0.5 * (s0 + s1);
            let add = side(m, f, at(mpt, sm), at(p, sm))?;
            Some(Tool { origin: o + d * s0, x: e1, y: e2, tri: [p, ta, tb], sweep: Sweep::Extrude(s1 - s0), add })
        }
        Sec::Revolve { c, a } => {
            if p.0 < -1e-9 || ta.0 < -1e-9 || tb.0 < -1e-9 {
                return None;
            }
            // the strip's span about the axis: the samples' angles, less the largest gap between them
            let u = normal_to(a);
            let w = a.cross(u);
            let mut th: Vec<f64> = pts.iter().map(|q| (*q - c).dot(w).atan2((*q - c).dot(u))).collect();
            th.sort_by(|x, y| x.partial_cmp(y).unwrap());
            let tau = std::f64::consts::TAU;
            let (mut gap, mut after) = (th[0] + tau - th[th.len() - 1], th[0]);
            for i in 1..th.len() {
                if th[i] - th[i - 1] > gap {
                    gap = th[i] - th[i - 1];
                    after = th[i];
                }
            }
            let full = gap < 30f64.to_radians() || m.b.face_edges(f).iter().any(|&e| m.b.is_seam(e));
            let (t0, span) = if full { (0.0, 360.0) } else { (after, (tau - gap).to_degrees()) };
            let r0 = u * t0.cos() + w * t0.sin();
            let rm = u * (t0 + 0.5 * span.to_radians()).cos() + w * (t0 + 0.5 * span.to_radians()).sin();
            let add = side(m, f, c + rm * mpt.0 + a * mpt.1, c + rm * p.0 + a * p.1)?;
            Some(Tool { origin: c, x: r0, y: a, tri: [p, ta, tb], sweep: Sweep::Revolve(span), add })
        }
    }
}

/// Whether the corner `pc` lies outside the strip (the blend removed it: the undo adds), from the strip's outward
/// normal at its point `pm`.
fn side(m: &Model, f: FaceId, pm: Vec3, pc: Vec3) -> Option<bool> {
    let surf = &m.b.face(f).surface;
    let (u, v) = surf.closest_param(pm);
    let q = surf.eval(u, v);
    if q.dist(pm) > 1e-5 * (1.0 + pm.len()) {
        return None;
    }
    let s_ = m.b.outward_normal(f, u, v).dot(pc - pm);
    (s_.abs() > 1e-9).then_some(s_ > 0.0)
}

/// The features that undo a blend step whose faces on the file's solid are `faces`: for every strip a frame plane, a
/// sketch of the triangle and its extrude or revolve (`add` / `cut`), with ids from `next`. `None` when a face of the
/// step is not a strip this handles.
pub fn tools(m: &Model, faces: &[FaceId], chamfer: bool, next: &mut dyn FnMut() -> String) -> Option<Vec<Json>> {
    if faces.is_empty() {
        return None;
    }
    let own: HashSet<FaceId> = faces.iter().copied().collect();
    let mut out = vec![];
    for &f in faces {
        let t = strip_tool(m, f, &own, chamfer)?;
        let (pid, sid, tid) = (next(), next(), next());
        out.push(Obj::new().put("type", "plane").put("id", pid.as_str()).put("mode", "frame").put("origin", vec3(t.origin)).put("xDir", vec3(t.x)).put("yDir", vec3(t.y)).build());
        let pt = |id: &str, p: P| Obj::new().put("id", id).put("x", n(p.0)).put("y", n(p.1)).build();
        let ln = |id: &str, a: &str, b: &str, c: bool| {
            let o = Obj::new().put("type", "line").put("id", id).put("p1", a).put("p2", b);
            if c { o.put("construction", true).build() } else { o.build() }
        };
        let mut points = vec![pt("p0", t.tri[0]), pt("p1", t.tri[1]), pt("p2", t.tri[2])];
        let mut ents = vec![ln("t1", "p0", "p1", false), ln("t2", "p1", "p2", false), ln("t3", "p2", "p0", false)];
        if matches!(t.sweep, Sweep::Revolve(_)) {
            points.push(pt("a0", (0.0, 0.0)));
            points.push(pt("a1", (0.0, 1.0)));
            ents.push(ln("ax", "a0", "a1", true));
        }
        out.push(
            Obj::new()
                .put("type", "sketch")
                .put("id", sid.as_str())
                .put("plane", Obj::new().put("kind", "feature").put("id", pid.as_str()).build())
                .put("points", arr(points))
                .put("entities", arr(ents))
                .put("constraints", arr(vec![]))
                .build(),
        );
        let op = if t.add { "add" } else { "cut" };
        out.push(match t.sweep {
            // (a cut runs against the sketch normal unless flipped: both run along it)
            Sweep::Extrude(depth) => ck_recog::emit::extrude(&tid, &sid, op, depth, !t.add, false),
            Sweep::Revolve(angle) => Obj::new().put("type", "revolve").put("id", tid.as_str()).put("op", op).put("sketch", sid.as_str()).put("axis", "ax").put("angle", n(angle)).build(),
        });
    }
    Some(out)
}
