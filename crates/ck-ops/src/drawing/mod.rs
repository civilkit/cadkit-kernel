//! 2D drawing extraction (`{type:"drawing", tag, views, hidden}` -> `{views:[...]}`).
//!
//! Reconstructed from oracle probes (kernel.wasm). Pinned: reply schema and
//! key order, the standard frames, the straight-line ordering key
//! `(round(angle·1e6), round(offset/step))` with `step = model bbox diagonal·1e-6`
//! (cross-checked against the instrumented oracle's own SipHash keys), section
//! cuts with `SEC.*` faces + 45° hatching, per-view `error:"cutNoEffect"`,
//! `{}` for a body-less state, empty/missing views -> `{"views":[]}`.
//! GUESS (marked inline): curved-piece emission order, the `round` flag rule,
//! exact split-point numerics, hidden-line details of curved occluders.
use crate::boolean::{self, Op};
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId};

mod hatch;
mod silhouette;

/// Standard view frames (pinned: box probes in all seven names).
pub fn std_frame(name: &str) -> Option<(Vec3, Vec3, Vec3)> {
    let v = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| (Vec3::new(a[0], a[1], a[2]), Vec3::new(b[0], b[1], b[2]), Vec3::new(c[0], c[1], c[2]));
    match name {
        "front" => Some(v([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0])),
        "back" => Some(v([-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0])),
        "right" => Some(v([0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0])),
        "left" => Some(v([0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0])),
        "top" => Some(v([1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0])),
        "bottom" => Some(v([1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0])),
        "iso" => {
            // the oracle's f64 constants (0.7071067811865476, 0.5773502691896258)
            // with y = z × x (last-ulp pinned)
            let x = Vec3::new(0.7071067811865476, 0.0, -0.7071067811865476);
            let z = Vec3::new(0.5773502691896258, 0.5773502691896258, 0.5773502691896258);
            let y = z.cross(x);
            Some(v([x.x, x.y, x.z], [y.x, y.y, y.z], [z.x, z.y, z.z]))
        }
        _ => None,
    }
}

/// One parsed view request.
pub struct ViewReq {
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
    pub section: Option<(Vec3, f64)>,
}

/// Lenient re-parse of the `views` array (ck-server owns the strict errors).
pub fn parse_views(msg: &Json) -> Vec<ViewReq> {
    let arr = msg.get_arr("views").map(|a| a).unwrap_or(&[]);
    fn vec3(j: Option<&Json>) -> Option<Vec3> {
        let a = match j? { Json::Arr(a) => a, _ => return None };
        Some(Vec3::new(
            a.first().and_then(Json::as_f64).unwrap_or(0.0),
            a.get(1).and_then(Json::as_f64).unwrap_or(0.0),
            a.get(2).and_then(Json::as_f64).unwrap_or(0.0),
        ))
    }
    let mut out = vec![];
    for v in arr {
        let (x, y, z) = match v.get_str("name").and_then(std_frame) {
            Some(f) => f,
            None => {
                let (Some(x), Some(y)) = (vec3(v.get("x")), vec3(v.get("y"))) else { continue };
                let z = x.cross(y);
                let n = z.len();
                (x, y, if n > 1e-12 { z / n } else { z })
            }
        };
        let section = v.get("section").and_then(|s| {
            let p = vec3(s.get("point"))?;
            Some((p, s.get_f64("hatch").unwrap_or(3.0)))
        });
        out.push(ViewReq { x, y, z, section });
    }
    out
}

/// `{type:"drawing"}` reply for a regenerated body.
pub fn reply(body: &Body, msg: &Json) -> Json {
    let hidden = msg.get("hidden").map_or(true, |h| h.as_bool().unwrap_or(true));
    let views = parse_views(msg);
    let views: Vec<Json> = views.iter().map(|v| view_json(body, v, hidden)).collect();
    Obj::new().put("views", Json::Arr(views)).build()
}

/// A 2D point with view depth.
#[derive(Clone, Copy, Debug)]
struct P2 {
    u: f64,
    v: f64,
    w: f64,
}

impl P2 {
    fn lerp(&self, o: &P2, t: f64) -> P2 {
        P2 { u: self.u + (o.u - self.u) * t, v: self.v + (o.v - self.v) * t, w: self.w + (o.w - self.w) * t }
    }
}

/// One drawn piece: a polyline in one visibility class.
struct Piece {
    k: u8, // b'e' edge, b'o' outline, b't' tangent
    v: bool,
    p: Vec<P2>,
    e: usize, // index in this view's edges array (usize::MAX = none)
}

fn point_json(p: [f64; 2]) -> Json {
    Json::Arr(vec![Json::Num(p[0]), Json::Num(p[1])])
}

fn piece_json(pc: &Piece) -> Json {
    let mut p: Vec<Json> = vec![];
    for pt in &pc.p {
        p.push(Json::Num(pt.u));
        p.push(Json::Num(pt.v));
    }
    let mut o = Obj::new().put("k", Json::Str((pc.k as char).to_string())).put("v", Json::Bool(pc.v)).put("p", Json::Arr(p));
    if pc.e != usize::MAX {
        o = o.put("e", Json::Num(pc.e as f64));
    }
    o.build()
}

enum CurveKind {
    Line,
    Circle { full: bool },
    Other,
}

fn classify_curve(c: &Curve, t0: f64, t1: f64) -> CurveKind {
    if c.kind() == "line" {
        return CurveKind::Line;
    }
    if c.circle_info().is_some() {
        let period = c.period().unwrap_or(f64::INFINITY);
        return CurveKind::Circle { full: t1 - t0 >= period - 1e-9 };
    }
    CurveKind::Other
}

/// The radial direction of a circle's plane perpendicular to the view.
fn view_radial(normal: Vec3, z: Vec3) -> Vec3 {
    let d = normal.cross(z);
    let n = d.len();
    if n > 1e-12 {
        d / n
    } else {
        z.cross(normal).normalize()
    }
}

trait NormFall {
    fn normalized_or_x(self) -> Vec3;
}
impl NormFall for Vec3 {
    fn normalized_or_x(self) -> Vec3 {
        let n = self.len();
        if n > 1e-12 {
            self / n
        } else {
            Vec3::X
        }
    }
}

/// Round half toward zero, then truncate (the oracle's quantization rounding,
/// pinned against its own logged keys).
fn rq(x: f64) -> i64 {
    (x + if x >= 0.0 { 0.49999999999999994 } else { -0.49999999999999994 }) as i64
}

/// The (angle, offset) sort key of a piece (pinned semantics).
fn piece_key(p: &[P2], step: f64) -> (f64, f64) {
    let (a, b) = (p[0], p[p.len() - 1]);
    let (mut dx, mut dy) = (b.u - a.u, b.v - a.v);
    let len = dx.hypot(dy);
    if !(len > 0.0) {
        return (0.0, 0.0);
    }
    dx /= len;
    dy /= len;
    if dy < 0.0 || (dy == 0.0 && dx < 0.0) {
        dx = -dx;
        dy = -dy;
    }
    let ang = rq(dy.atan2(dx) * 1e6) as f64;
    let off = rq((a.u * dy - a.v * dx) / step) as f64;
    (ang, off)
}

fn piece_len(p: &[P2]) -> f64 {
    let mut l = 0.0;
    for w in p.windows(2) {
        l += (w[1].u - w[0].u).hypot(w[1].v - w[0].v);
    }
    l
}

/// Project one view of `body` and build the reply JSON.
fn view_json(body: &Body, view: &ViewReq, hidden: bool) -> Json {
    // Section: cut with the plane; the kept half is the one the viewer is on
    // (pinned: box z=-10 section viewed from +z keeps z in [-10,0]).
    let cut;
    let hatch;
    let sec_faces: Vec<FaceId>;
    let body: &Body = match view.section {
        Some((point, h)) => {
            match section_cut(body, view.z, point) {
                Some(out) => {
                    cut = out.body;
                    sec_faces = cut.face_ids().filter(|&f| cut.face(f).name.id().starts_with("SEC.")).collect();
                    hatch = h;
                    &cut
                }
                _ => return Obj::new().put("error", "cutNoEffect").build(),
            }
        }
        None => {
            hatch = 0.0;
            sec_faces = vec![];
            body
        }
    };
    let (x, y, z) = (view.x, view.y, view.z);
    let proj = |p: Vec3| P2 { u: p.dot(x), v: p.dot(y), w: p.dot(z) };

    // Quantization step: model bbox diagonal * 1e-6 (pinned: box front keys).
    let Some((blo, bhi)) = body_bbox(body) else { return Obj::new().put("error", "cutNoEffect").build() };
    let step = (bhi.x - blo.x).hypot(bhi.y - blo.y).hypot(bhi.z - blo.z) * 1e-6;
    // View tolerance: max(0.001, diag(u,v,w)·0.0004) over the projected box.
    let tol = {
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for v in body.vertex_ids() {
            let p = body.vertex(v).p;
            let q = [p.dot(x), p.dot(y), p.dot(z)];
            for i in 0..3 {
                lo[i] = lo[i].min(q[i]);
                hi[i] = hi[i].max(q[i]);
            }
        }
        ((hi[0] - lo[0]).hypot(hi[1] - lo[1]).hypot(hi[2] - lo[2]) * 0.0004).max(0.001)
    };

    let tess = ck_tess::mesh::tessellate(body);
    let occ = Occluders::build(&tess, x, y, z, tol);

    let mut edges_json: Vec<Json> = vec![];
    // per edge entry: (retained 2D extent, is a full circle seen edge-on)
    let mut round_info: Vec<Option<([f64; 4], bool)>> = vec![];
    let mut pieces: Vec<Piece> = vec![];
    let mut axes: Vec<Json> = vec![];

    // body edges (the view's edges array follows body edge order); the `e`
    // index of a piece = the edge's ARENA index (pinned: the two-part probe,
    // where dead slots leave gaps), outlines continue after the arena.
    let arena_base = body.edge_ids().map(|e| e.idx()).max().map_or(0, |m| m + 1);
    for e in body.edge_ids() {
        let ed = body.edge(e);
        if !ed.alive || body.is_seam(e) {
            continue; // the oracle's bodies carry no seam edges
        }
        // the piece index = the key's `#N` suffix = the oracle's arena index
        // (our own arena numbering is compressed; the regen keys pin the
        // oracle's) — the two-part probe: our idx 4 carries the key "#8".
        let seq = body.wire_numbered().0.iter().find(|(_, c)| c[0] == e).map_or(e.idx(), |(n, _)| *n);
        let eidx = seq;
        let fs: Vec<FaceId> = body.edge_faces(e);
        let mut ids: Vec<String> = fs.iter().map(|&f| body.face_id_str(f)).collect();
        ids.sort();
        let faces: Vec<Json> = ids.into_iter().map(Json::Str).collect();
        // tangent ("t") when the two face normals are parallel at the edge
        // (G1: fillet boundaries; chamfer terminations stay "e") — pinned on
        // the bracket (F11.B0 edges "t", F12.B0-to-bottom edge "e").
        let tangent = match fs.as_slice() {
            [fa, fb] => {
                let m = body.edge_mid(e);
                let na = face_normal_at(body, *fa, m);
                let nb = face_normal_at(body, *fb, m);
                na.dot(nb).abs() > 1.0 - 1e-7
            }
            _ => false,
        };
        let mid3 = body.edge_mid(e);
        let mid2 = proj(mid3);
        let mut ev = Obj::new()
            .put("faces", Json::Arr(faces))
            .put("at", Json::Arr(vec![Json::Num(mid3.x), Json::Num(mid3.y), Json::Num(mid3.z)]));
        let k = if tangent { b't' } else { b'e' };
        match classify_curve(&ed.curve, ed.t0, ed.t1) {
            CurveKind::Line => {
                let a = proj(ed.curve.eval(ed.t0));
                let b = proj(ed.curve.eval(ed.t1));
                if (a.u - b.u).abs() > 1e-12 || (a.v - b.v).abs() > 1e-12 {
                    ev = ev
                        .put("line", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]))
                        .put("mid", point_json([mid2.u, mid2.v]))
                        .put("ends", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]));
                    push_pieces(&occ, &mut pieces, &[a, b], eidx, k, tol);
                } else {
                    ev = ev.put("mid", point_json([mid2.u, mid2.v]))
                        .put("ends", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]));
                }
                round_info.push(None);
            }
            CurveKind::Circle { full } => {
                let (c3, n3, r) = ed.curve.circle_info().unwrap();
                let c = proj(c3);
                let dot = n3.dot(z);
                if dot.abs() > 1.0 - 1e-9 {
                    // face-on: projects to a true circle
                    ev = ev
                        .put("circle", Obj::new().put("c", point_json([c.u, c.v])).put("r", Json::Num(r)).build())
                        .put("mid", point_json([mid2.u, mid2.v]));
                    let pts: Vec<P2> = tess.edge_polyline(body, e).iter().map(|&p| proj(p)).collect();
                    push_pieces_curve(&occ, &mut pieces, &pts, eidx, k, tol, full);
                    round_info.push(None);
                } else if dot.abs() < 1e-9 {
                    // edge-on: the projected diameter line
                    let d = view_radial(n3, z);
                    let (pa, pb) = (proj(c3 - d * r), proj(c3 + d * r));
                    ev = ev.put("line", Json::Arr(vec![point_json([pa.u, pa.v]), point_json([pb.u, pb.v])]))
                        .put("mid", point_json([mid2.u, mid2.v]));
                    let ext = [pa.u.min(pb.u), pa.u.max(pb.u), pa.v.min(pb.v), pa.v.max(pb.v)];
                    if !full {
                        let ea = proj(ed.curve.eval(ed.t0));
                        let eb = proj(ed.curve.eval(ed.t1));
                        ev = ev.put("ends", Json::Arr(vec![point_json([ea.u, ea.v]), point_json([eb.u, eb.v])]));
                    }
                    push_pieces_arc(&occ, &mut pieces, &[pa, pb], eidx, k, tol, r);
                    round_info.push(Some((ext, full)));
                } else {
                    // tilted circle: polyline of the projected samples (GUESS)
                    let pts: Vec<P2> = tess.edge_polyline(body, e).iter().map(|&p| proj(p)).collect();
                    let (Some(a), Some(b)) = (pts.first().copied(), pts.last().copied()) else { continue };
                    ev = ev
                        .put("line", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]))
                        .put("mid", point_json([mid2.u, mid2.v]))
                        .put("ends", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]));
                    push_pieces_curve(&occ, &mut pieces, &pts, eidx, k, tol, full);
                    round_info.push(None);
                }
            }
            CurveKind::Other => {
                let pts: Vec<P2> = tess.edge_polyline(body, e).iter().map(|&p| proj(p)).collect();
                let (Some(a), Some(b)) = (pts.first().copied(), pts.last().copied()) else { continue };
                ev = ev
                    .put("line", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]))
                    .put("mid", point_json([mid2.u, mid2.v]))
                    .put("ends", Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]));
                push_pieces_curve(&occ, &mut pieces, &pts, eidx, k, tol, false);
                round_info.push(None);
            }
        }
        edges_json.push(ev.build());
    }

    // silhouettes + centre axes. Pinned rules:
    // - an outline line = the cylinder silhouette generator (surface normal
    //   perpendicular to the view) that lies INSIDE the face's angular range
    //   (the bracket front: F6.S.e6/e16 only, at = the generator's axis mid);
    // - the straddling display-mesh silhouette also contributes (the
    //   counterbore extremes in the top view, o91..o96);
    // - centre-mark axes only for FULL cylinder faces whose axis is
    //   perpendicular to the view (top view of the counterbores yes; iso no).
    for f in body.face_ids() {
        let face = body.face(f);
        if !face.alive {
            continue;
        }
        let (frame, r) = match &face.surface {
            Surface::Cylinder { frame, r } => (frame, *r),
            _ => continue,
        };
        let (o, axis) = (frame.o, frame.z);
        let (mut tmin, mut tmax) = (f64::INFINITY, f64::NEG_INFINITY);
        let mut thetas: Vec<f64> = vec![];
        for &lp in &face.loops {
            for &c in body.lp(lp).coedges.iter() {
                let co = body.coedge(c);
                if !co.alive {
                    continue;
                }
                let e = body.edge(co.edge);
                let n = if e.curve.kind() == "line" { 2 } else { 16 };
                for i in 0..n {
                    let t = e.t0 + (e.t1 - e.t0) * i as f64 / n as f64;
                    let p = e.curve.eval(t);
                    tmin = tmin.min((p - o).dot(axis));
                    tmax = tmax.max((p - o).dot(axis));
                    thetas.push(face.surface.closest_param(p).0);
                }
            }
        }
        if !(tmax > tmin) || thetas.is_empty() {
            continue;
        }
        thetas.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let tau = std::f64::consts::TAU;
        let mut max_gap = thetas[0] + tau - thetas[thetas.len() - 1];
        let mut max_gap_start = thetas[thetas.len() - 1];
        for w in thetas.windows(2) {
            if w[1] - w[0] > max_gap {
                max_gap = w[1] - w[0];
                max_gap_start = w[0];
            }
        }
        let full = max_gap < std::f64::consts::FRAC_PI_2;
        let wz = axis.dot(z);
        // centre axis
        if full && wz.abs() < 1e-9 {
            let a = proj(o + axis * tmin);
            let b = proj(o + axis * tmax);
            if (a.u - b.u).abs() > 1e-9 || (a.v - b.v).abs() > 1e-9 {
                axes.push(Json::Arr(vec![point_json([a.u, a.v]), point_json([b.u, b.v])]));
            }
        }
        if wz.abs() >= 1.0 - 1e-9 {
            continue; // axis along the view: silhouette handled by the cap edges
        }
        // silhouette generators: the radials d with normal·view = 0
        let d = axis.cross(z).normalized_or_x();
        let in_range = |th: f64| {
            if full {
                return true;
            }
            // angular range = the complement of the largest gap
            let (a0, a1) = ((max_gap_start + max_gap) % tau, max_gap_start % tau);
            let t = (th % tau + tau) % tau;
            if a0 <= a1 {
                t >= a0 - 1e-9 && t <= a1 + 1e-9
            } else {
                t >= a0 - 1e-9 || t <= a1 + 1e-9
            }
        };
        for s in [-1.0, 1.0] {
            let n = (frame.x * d.x + frame.y * d.y) * s; // generator normal
            let th = n.dot(frame.y).atan2(n.dot(frame.x));
            if !in_range(th) {
                continue;
            }
            let pa3 = o + axis * tmin + d * (r * s);
            let pb3 = o + axis * tmax + d * (r * s);
            let (pa, pb) = (proj(pa3), proj(pb3));
            let eidx = edges_json.len();
            edges_json.push(
                Obj::new()
                    .put("faces", Json::Arr(vec![Json::Str(body.face_id_str(f))]))
                    .put(
                        "at",
                        Json::Arr(vec![
                            Json::Num((pa3.x + pb3.x) * 0.5),
                            Json::Num((pa3.y + pb3.y) * 0.5),
                            Json::Num((pa3.z + pb3.z) * 0.5),
                        ]),
                    )
                    .put("line", Json::Arr(vec![point_json([pa.u, pa.v]), point_json([pb.u, pb.v])]))
                    .put("mid", point_json([(pa.u + pb.u) * 0.5, (pa.v + pb.v) * 0.5]))
                    .put("outline", Json::Bool(true))
                    .put("ends", Json::Arr(vec![point_json([pa.u, pa.v]), point_json([pb.u, pb.v])]))
                    .build(),
            );
            round_info.push(None);
            push_pieces(&occ, &mut pieces, &[pa, pb], eidx, b'o', tol);
        }
    }

    // silhouettes of the non-planar, non-cylindrical smooth faces (cone,
    // sphere, torus, revolution/extrusion, B-spline, screw): marching
    // triangles on n·z over the display mesh (trimmed like the face). Straight
    // generators (cone) become a two-point outline with an `edges` entry, the
    // curved contours are plain `o` polylines without `e` (like the oracle's
    // sphere/torus outlines).
    // scale for the silhouette passes: the DISPLAY MESH extent (the vertex
    // bbox above is degenerate for vertex-less bodies such as a torus)
    let mesh_diag = tess.mesh.bbox().map_or(step * 1e6, |(lo, hi)| (hi - lo).len()).max(step * 1e6);
    let sil_off = (mesh_diag * 1.5e-3).max(tol);
    for (wi, f) in body.face_ids().enumerate() {
        let face = body.face(f);
        if !face.alive || matches!(face.surface, Surface::Plane { .. } | Surface::Cylinder { .. }) {
            continue;
        }
        for chain in silhouette::face_contours(&tess.mesh, wi as i32, z, mesh_diag, (mesh_diag * 2e-4).max(tol * 0.25)) {
            let pts: Vec<P2> = chain.iter().map(|&(p, _)| proj(p)).collect();
            // visibility is tested a little OUTSIDE the surface (along the
            // projected outward normal): a silhouette point sits where the
            // surface is edge-on, so its own face right next to it would
            // otherwise read as an occluder
            let test: Vec<P2> = chain
                .iter()
                .zip(&pts)
                .map(|(&(_, n), p)| {
                    let (a, b) = (n.dot(x), n.dot(y));
                    let l = a.hypot(b);
                    if l > 1e-9 { P2 { u: p.u + a / l * sil_off, v: p.v + b / l * sil_off, w: p.w } } else { *p }
                })
                .collect();
            if pts.len() == 2 {
                let (pa, pb) = (pts[0], pts[1]);
                let (pa3, pb3) = (chain[0].0, chain[1].0);
                let eidx = edges_json.len();
                edges_json.push(
                    Obj::new()
                        .put("faces", Json::Arr(vec![Json::Str(body.face_id_str(f))]))
                        .put(
                            "at",
                            Json::Arr(vec![
                                Json::Num((pa3.x + pb3.x) * 0.5),
                                Json::Num((pa3.y + pb3.y) * 0.5),
                                Json::Num((pa3.z + pb3.z) * 0.5),
                            ]),
                        )
                        .put("line", Json::Arr(vec![point_json([pa.u, pa.v]), point_json([pb.u, pb.v])]))
                        .put("mid", point_json([(pa.u + pb.u) * 0.5, (pa.v + pb.v) * 0.5]))
                        .put("outline", Json::Bool(true))
                        .put("ends", Json::Arr(vec![point_json([pa.u, pa.v]), point_json([pb.u, pb.v])]))
                        .build(),
                );
                round_info.push(None);
                push_pieces_impl(&occ, &mut pieces, &pts, Some(&test), None, eidx, b'o', tol, false);
            } else {
                push_pieces_impl(&occ, &mut pieces, &pts, Some(&test), None, usize::MAX, b'o', tol, true);
            }
        }
    }

    // ---- line assembly ---------------------------------------------------
    let mut straight: Vec<Piece> = vec![];
    let mut curved: Vec<Piece> = vec![];
    for p in pieces {
        if p.p.len() == 2 {
            straight.push(p);
        } else {
            curved.push(p);
        }
    }
    let mut resolved: Vec<(f64, f64, f64, f64, usize, Piece)> = vec![];
    for (seq, pc) in straight.into_iter().enumerate() {
        let (ang, off) = piece_key(&pc.p, step);
        let len = piece_len(&pc.p);
        // span start along the piece direction (pinned: bracket front family
        // order e59, e42 = left span before right span)
        let (a, b) = (pc.p[0], pc.p[pc.p.len() - 1]);
        let (mut dx, mut dy) = (b.u - a.u, b.v - a.v);
        let l = dx.hypot(dy);
        let start = if l > 0.0 { a.u * (dx / l) + a.v * (dy / l) } else { 0.0 };
        resolved.push((ang, off, start, len, seq, pc));
    }
    resolved.sort_by(|a, b| {
        (a.0, a.1)
            .partial_cmp(&(b.0, b.1))
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
            .then(a.3.total_cmp(&b.3))
            .then(a.5.k.cmp(&b.5.k))
            .then(a.5.e.cmp(&b.5.e))
            .then(a.4.cmp(&b.4))
    });
    drop_covered(&mut resolved, tol);

    let mut lines: Vec<Json> = vec![];
    for (_, _, _, _, _, pc) in &resolved {
        if !pc.v && !hidden {
            continue;
        }
        lines.push(piece_json(pc));
    }
    // GUESS: curved pieces after the straight ones, in edge/scan order (the
    // oracle iterates an internal map we could not pin).
    for pc in &curved {
        if !pc.v && !hidden {
            continue;
        }
        lines.push(piece_json(pc));
    }
    if view.section.is_some() {
        for l in hatch::hatch_lines(body, &sec_faces, x, y, z, hatch) {
            lines.push(l);
        }
    }

    // bounds over the emitted polyline points (pinned: polyline vertices only)
    let (mut umin, mut vmin) = (f64::INFINITY, f64::INFINITY);
    let (mut umax, mut vmax) = (f64::NEG_INFINITY, f64::NEG_INFINITY);
    for l in &lines {
        let Json::Obj(kv) = l else { continue };
        let Some((_, Json::Arr(p))) = kv.iter().find(|(k, _)| k == "p") else { continue };
        for i in 0..p.len() / 2 {
            let (u, v) = (p[2 * i].as_f64().unwrap_or(0.0), p[2 * i + 1].as_f64().unwrap_or(0.0));
            umin = umin.min(u);
            vmin = vmin.min(v);
            umax = umax.max(u);
            vmax = vmax.max(v);
        }
    }
    // GUESS `round`: a full circle seen edge-on whose line spans the view's
    // full width in its direction (cyl cap yes; box3 rim no; section cap no).
    for (i, info) in round_info.iter().enumerate() {
        let Some((ext, full)) = info else { continue };
        if !*full {
            continue;
        }
        let horiz = ext[1] - ext[0] >= ext[3] - ext[2];
        let covers = if horiz {
            ext[0] <= umin + tol * 0.5 && ext[1] >= umax - tol * 0.5
        } else {
            ext[2] <= vmin + tol * 0.5 && ext[3] >= vmax - tol * 0.5
        };
        if covers {
            if let Json::Obj(kv) = &mut edges_json[i] {
                kv.push(("round".into(), Json::Bool(true)));
            }
        }
    }

    Obj::new()
        .put("min", point_json([umin, vmin]))
        .put("max", point_json([umax, vmax]))
        .put("lines", Json::Arr(lines))
        .put("edges", Json::Arr(edges_json))
        .put("axes", Json::Arr(axes))
        .build()
}

fn body_bbox(body: &Body) -> Option<(Vec3, Vec3)> {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for v in body.vertex_ids() {
        let p = body.vertex(v).p;
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    (lo.x <= hi.x).then_some((lo, hi))
}

/// The section cut of `body` by the plane through `point` (the kept half is the viewer's).
///
/// First the plain attempt (the tool sized from the vertex box, centred on `point`): every pinned reply comes
/// out of it unchanged. Two things make it fail on real models, so a failed or no-effect cut is retried:
/// - the vertex box of a body of revolution is flat (a circle has one vertex), and `point` can lie far from the
///   body along the plane (the kit puts it on the axis through the origin), so the tool square misses part of
///   the body; the retry sizes the tool from sampled edge points and centres it on the body;
/// - a plane through a hole's axis or along a seam can fail the boolean; the retry then moves the plane a hair
///   to either side (1e-5 and 1e-4 of the body's diagonal, far under the drawing's tolerance).
fn section_cut(body: &Body, n: Vec3, point: Vec3) -> Option<boolean::BoolOut> {
    let try_cut = |tool: &Body| match boolean::boolean(body, tool, Op::Cut, 1e-6) {
        Ok(out) if !out.no_effect => Some(out),
        _ => None,
    };
    if let Some(out) = try_cut(&plane_tool(body, n, point)) {
        return Some(out);
    }
    let (lo, hi) = body_extent(body)?;
    let diag = (hi - lo).len();
    for off in [0.0, 1e-5, -1e-5, 1e-4, -1e-4] {
        let p = point + n * (off * diag);
        let c = (lo + hi) * 0.5;
        let origin = c - n * (c - p).dot(n);
        if let Some(out) = try_cut(&plane_tool_at(n, origin, diag * 2.0 + 1.0)) {
            return Some(out);
        }
    }
    None
}

/// Bounding box over the vertices and over points sampled along every edge (a revolved body has one vertex per
/// circle, so its vertex box can be flat).
fn body_extent(body: &Body) -> Option<(Vec3, Vec3)> {
    let (mut lo, mut hi) = body_bbox(body).unwrap_or((Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY)));
    for e in body.edge_ids() {
        let ed = body.edge(e);
        if !ed.alive {
            continue;
        }
        let n = if ed.curve.kind() == "line" { 1 } else { 32 };
        for i in 0..=n {
            let p = ed.curve.eval(ed.t0 + (ed.t1 - ed.t0) * i as f64 / n as f64);
            lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        }
    }
    (lo.x <= hi.x).then_some((lo, hi))
}

/// A half-space tool box for the section cut: base square on the plane
/// (feature "SEC", so the cutting cap is named `SEC.C0`, pinned), extruded
/// away from the viewer.
fn plane_tool(body: &Body, n: Vec3, point: Vec3) -> Body {
    let (lo, hi) = body_bbox(body).unwrap_or((Vec3::ZERO, Vec3::new(1.0, 1.0, 1.0)));
    plane_tool_at(n, point, (hi.x - lo.x).hypot(hi.y - lo.y).hypot(hi.z - lo.z) * 2.0 + 1.0)
}

/// The half-space tool: a square of side `d` centred on `point` in the plane, extruded away from the viewer.
fn plane_tool_at(n: Vec3, point: Vec3, d: f64) -> Body {
    use ck_geom::{Frame, Vec2};
    use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
    let xx = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
    let xx = (xx - n * xx.dot(n)).normalize();
    let yy = n.cross(xx);
    let frame = Frame { o: point, x: xx, y: yy, z: n };
    let h = d * 0.5;
    let c = [Vec2::new(-h, -h), Vec2::new(h, -h), Vec2::new(h, h), Vec2::new(-h, h)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line("s", c[i], c[(i + 1) % 4])).collect());
    let prof = Profile::new(frame, vec![ProfileRegion { outer: lp, holes: vec![] }]);
    extrude("SEC", &prof, ExtrudeOpts { s0: 0.0, s1: -d, dir: None, draft: 0.0 }).unwrap_or_default()
}

/// Occluders: the display mesh triangles projected to 2D with depth.
struct Occluders {
    tris: Vec<([P2; 3], bool)>, // (projected triangle, degenerate)
}

impl Occluders {
    fn build(tess: &ck_tess::mesh::Tess, x: Vec3, y: Vec3, z: Vec3, tol: f64) -> Occluders {
        let m = &tess.mesh;
        let mut tris = vec![];
        let pos = |i: usize| {
            let b = i * 3;
            Vec3::new(m.positions[b] as f64, m.positions[b + 1] as f64, m.positions[b + 2] as f64)
        };
        let pv = |p: Vec3| P2 { u: p.dot(x), v: p.dot(y), w: p.dot(z) };
        for (t, &f) in m.tri_face.iter().enumerate() {
            if f < 0 {
                continue;
            }
            let (a, b, c) = (pos(t * 3), pos(t * 3 + 1), pos(t * 3 + 2));
            let (pa, pb, pc) = (pv(a), pv(b), pv(c));
            let area = (pb.u - pa.u) * (pc.v - pa.v) - (pb.v - pa.v) * (pc.u - pa.u);
            let max_edge = (pb.u - pa.u).hypot(pb.v - pa.v)
                .max((pc.u - pb.u).hypot(pc.v - pb.v))
                .max((pa.u - pc.u).hypot(pa.v - pc.v));
            // a face seen edge-on (projected sliver) occludes nothing
            let deg = area.abs() * 2.0 < max_edge * tol;
            tris.push(([pa, pb, pc], deg));
        }
        Occluders { tris }
    }

    /// Is the 2D point covered by any face in front of `w` (+ tol)?
    fn covered(&self, p: &P2, tol: f64) -> bool {
        self.covered_dbg(p, tol).is_some()
    }

    fn covered_dbg(&self, p: &P2, tol: f64) -> Option<(f64, f64, f64)> {
        for (t, deg) in &self.tris {
            if *deg {
                continue;
            }
            let Some(s) = bary(t, p, tol) else { continue };
            let w = t[0].w * s[0] + t[1].w * s[1] + t[2].w * s[2];
            if w > p.w + tol {
                return Some((w, t[0].u, t[0].v));
            }
        }
        None
    }

    /// Is the point covered by any of the candidate triangles (see `runs_line`)?
    fn covered_in(cand: &[&[P2; 3]], p: &P2, tol: f64) -> bool {
        cand.iter().any(|t| {
            bary(t, p, tol).map_or(false, |s| t[0].w * s[0] + t[1].w * s[1] + t[2].w * s[2] > p.w + tol)
        })
    }

    /// `covered_in`, plus the mesh-seam case: a point exactly on an interior mesh edge of the occluder is strictly inside
    /// neither triangle, so it reads "not covered" although the surface is in front of it on both sides. When the point
    /// is not covered, it counts as covered if the points `d` to BOTH sides of the line (unit normal `nrm`) are covered
    /// (an occluder boundary has the surface on one side only, so a grazing edge there stays visible).
    fn covered_seam(cand: &[&[P2; 3]], p: &P2, nrm: (f64, f64), tol: f64) -> bool {
        if Self::covered_in(cand, p, tol) {
            return true;
        }
        let d = tol * 1.5;
        let side = |s: f64| P2 { u: p.u + nrm.0 * d * s, v: p.v + nrm.1 * d * s, ..*p };
        Self::covered_in(cand, &side(1.0), tol) && Self::covered_in(cand, &side(-1.0), tol)
    }

    /// Visibility runs along ONE straight segment a-b. The segment is cut at every place where it crosses the
    /// outline of a triangle in front of it, and each stretch is classified by its own midpoint, so a long line
    /// that runs behind one body and out the other side (a shaft through a bearing, a can and a plate) comes out
    /// as a hidden piece and visible pieces instead of one class for its whole length.
    ///
    /// `arc` is the radius of a circle seen edge-on that this segment is the diameter of: the circle's points
    /// project on the line with depth `a.w`-centred +-sqrt(r^2 - s^2) (its front and back half), and the line is
    /// visible where either half is. A circle is not at the depth of its centre.
    fn runs_line(&self, a: P2, b: P2, arc: Option<f64>, tol: f64) -> Vec<(f64, f64, bool)> {
        let len = (b.u - a.u).hypot(b.v - a.v);
        if len <= tol * 0.5 {
            return vec![];
        }
        // triangles that can cover some point of the segment: they overlap its box and reach in front of it
        let w_lo = a.w.min(b.w) - arc.unwrap_or(0.0);
        let (u0, u1) = (a.u.min(b.u) - tol, a.u.max(b.u) + tol);
        let (v0, v1) = (a.v.min(b.v) - tol, a.v.max(b.v) + tol);
        let cand: Vec<&[P2; 3]> = self
            .tris
            .iter()
            .filter(|(t, deg)| {
                !*deg
                    && t.iter().map(|q| q.w).fold(f64::NEG_INFINITY, f64::max) > w_lo + tol
                    && t.iter().map(|q| q.u).fold(f64::NEG_INFINITY, f64::max) >= u0
                    && t.iter().map(|q| q.u).fold(f64::INFINITY, f64::min) <= u1
                    && t.iter().map(|q| q.v).fold(f64::NEG_INFINITY, f64::max) >= v0
                    && t.iter().map(|q| q.v).fold(f64::INFINITY, f64::min) <= v1
            })
            .map(|(t, _)| t)
            .collect();
        // the crossings strictly inside the segment, then the two true ends (never moved by the merging)
        let mut cuts: Vec<f64> = vec![];
        for t in &cand {
            for k in 0..3 {
                if let Some(s) = seg_isect(a, b, t[k], t[(k + 1) % 3]) {
                    if s * len > tol * 0.25 && (1.0 - s) * len > tol * 0.25 {
                        cuts.push(s);
                    }
                }
            }
        }
        cuts.sort_by(|x, y| x.total_cmp(y));
        cuts.dedup_by(|x, y| (*x - *y).abs() * len < tol * 0.25);
        let mut ts = vec![0.0];
        ts.extend(cuts);
        ts.push(1.0);
        let vis_at = |t: f64| -> bool {
            let m = a.lerp(&b, t);
            match arc {
                None => !Self::covered_in(&cand, &m, tol),
                Some(r) => {
                    let sd = 2.0 * t - 1.0;
                    let dw = r * (1.0 - sd * sd).max(0.0).sqrt();
                    let (f, k) = (P2 { w: m.w + dw, ..m }, P2 { w: m.w - dw, ..m });
                    let nrm = ((a.v - b.v) / len, (b.u - a.u) / len);
                    !Self::covered_seam(&cand, &f, nrm, tol) || !Self::covered_seam(&cand, &k, nrm, tol)
                }
            }
        };
        // one class per stretch; a stretch shorter than twice the tolerance (a mesh seam, not a gap) takes the class of its neighbour
        let mut iv: Vec<(f64, f64, bool, bool)> = vec![]; // (t0, t1, visible, long enough)
        for w in ts.windows(2) {
            let long = (w[1] - w[0]) * len >= tol * 2.0;
            iv.push((w[0], w[1], vis_at(0.5 * (w[0] + w[1])), long));
        }
        for i in 0..iv.len() {
            if !iv[i].3 {
                let from = if i > 0 { iv[i - 1].2 } else { iv.iter().find(|x| x.3).map_or(iv[i].2, |x| x.2) };
                iv[i].2 = from;
            }
        }
        // an edge-on circle: a visible stretch of a few tolerances between two hidden ones is a sliver of the occluder's
        // mesh (triangles too thin to count as covering), not a gap the circle shows through
        if arc.is_some() {
            for i in 1..iv.len().saturating_sub(1) {
                if iv[i].2 && !iv[i - 1].2 && !iv[i + 1].2 && (iv[i].1 - iv[i].0) * len < tol * 8.0 {
                    iv[i].2 = false;
                }
            }
        }
        let mut runs: Vec<(f64, f64, bool)> = vec![];
        for (t0, t1, vis, _) in iv {
            match runs.last_mut() {
                Some(last) if last.2 == vis => last.1 = t1,
                _ => runs.push((t0, t1, vis)),
            }
        }
        runs
    }

    /// Visibility runs along a polyline: (t0, t1, visible) params.
    fn runs(&self, pts: &[P2], tol: f64) -> Vec<(f64, f64, bool)> {
        let n = pts.len().saturating_sub(1);
        if n == 0 {
            return vec![];
        }
        let mut runs: Vec<(f64, f64, bool)> = vec![];
        for i in 0..n {
            let mid = pts[i].lerp(&pts[i + 1], 0.5);
            let len = (pts[i + 1].u - pts[i].u).hypot(pts[i + 1].v - pts[i].v);
            if len <= tol * 0.5 {
                continue; // degenerate segment: skip (no line drawn)
            }
            let vis = !self.covered(&mid, tol);
            let (t0, t1) = (i as f64, (i + 1) as f64);
            match runs.last_mut() {
                Some(last) if last.2 == vis && (last.1 - t0).abs() < 1e-12 => last.1 = t1,
                _ => runs.push((t0, t1, vis)),
            }
        }
        runs
    }

    /// Refine a transition param: the occluder boundary intersection nearest
    /// the transition point (GUESS numerics).
    fn split_param(&self, a: P2, b: P2, t_hint: f64, tol: f64) -> f64 {
        let mut best: Option<(f64, f64)> = None; // (dist to hint, param)
        for (t, deg) in &self.tris {
            if *deg {
                continue;
            }
            for k in 0..3 {
                let (p, q) = (t[k], t[(k + 1) % 3]);
                if let Some(s) = seg_isect(a, b, p, q) {
                    let x = a.u + (b.u - a.u) * s;
                    let y = a.v + (b.v - a.v) * s;
                    let h = a.u + (b.u - a.u) * t_hint;
                    let hy = a.v + (b.v - a.v) * t_hint;
                    let d = (x - h).hypot(y - hy);
                    if d <= tol * 8.0 && best.map_or(true, |(bd, _)| d < bd) {
                        best = Some((d, s));
                    }
                }
            }
        }
        best.map_or(t_hint, |(_, s)| s)
    }
}

fn bary(t: &[P2; 3], p: &P2, tol: f64) -> Option<[f64; 3]> {
    let (a, b, c) = (t[0], t[1], t[2]);
    let d = (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
    if d.abs() <= 1e-15 {
        return None;
    }
    let l1 = ((p.u - a.u) * (c.v - a.v) - (p.v - a.v) * (c.u - a.u)) / d;
    let l2 = ((b.u - a.u) * (p.v - a.v) - (b.v - a.v) * (p.u - a.u)) / d;
    let l0 = 1.0 - l1 - l2;
    // strictly inside (pinned: the oracle does not let an occluder cover a
    // grazing ray on its own boundary — the bracket's e72 stays visible) with
    // a half-tolerance shrink against noise
    // heights from each vertex to the opposite edge (2*area / edge length)
    let h0 = d.abs() / (t[1].u - t[2].u).hypot(t[1].v - t[2].v).max(1e-12);
    let h1 = d.abs() / (t[0].u - t[2].u).hypot(t[0].v - t[2].v).max(1e-12);
    let h2 = d.abs() / (t[0].u - t[1].u).hypot(t[0].v - t[1].v).max(1e-12);
    let m0 = tol * 0.5 / h0.max(1e-12);
    let m1 = tol * 0.5 / h1.max(1e-12);
    let m2 = tol * 0.5 / h2.max(1e-12);
    if l0 < m0 || l1 < m1 || l2 < m2 || l0 > 1.0 - m0 || l1 > 1.0 - m1 || l2 > 1.0 - m2 {
        return None;
    }
    Some([l0, l1, l2])
}

/// Segment AB × CD in 2D, returning the param on AB.
fn seg_isect(a: P2, b: P2, c: P2, d: P2) -> Option<f64> {
    let (r, s) = ((b.u - a.u, b.v - a.v), (d.u - c.u, d.v - c.v));
    let den = r.0 * s.1 - r.1 * s.0;
    if den.abs() < 1e-15 {
        return None;
    }
    let t = ((c.u - a.u) * s.1 - (c.v - a.v) * s.0) / den;
    let q = ((c.u - a.u) * r.1 - (c.v - a.v) * r.0) / den;
    (t >= -1e-12 && t <= 1.0 + 1e-12 && q >= -1e-12 && q <= 1.0 + 1e-12).then_some(t.clamp(0.0, 1.0))
}

/// Canonical direction for a piece's points (pinned: the oracle's pieces run
/// in the canonical direction of their line).
fn canonicalize(p: &mut [P2]) {
    if p.len() < 2 {
        return;
    }
    let (a, b) = (p[0], p[p.len() - 1]);
    let (mut dx, mut dy) = (b.u - a.u, b.v - a.v);
    if dx == 0.0 && dy == 0.0 {
        return;
    }
    if dy < 0.0 || (dy == 0.0 && dx < 0.0) {
        p.reverse();
    }
}

/// Split a polyline into visibility pieces and push them.
fn push_pieces(occ: &Occluders, out: &mut Vec<Piece>, pts: &[P2], e: usize, k: u8, tol: f64) {
    push_pieces_impl(occ, out, pts, None, None, e, k, tol, false)
}

/// Like `push_pieces`, for the diameter line of a circle of radius `r` seen edge-on.
fn push_pieces_arc(occ: &Occluders, out: &mut Vec<Piece>, pts: &[P2], e: usize, k: u8, tol: f64, r: f64) {
    push_pieces_impl(occ, out, pts, None, Some(r), e, k, tol, false);
}


fn push_pieces_curve(occ: &Occluders, out: &mut Vec<Piece>, pts: &[P2], e: usize, k: u8, tol: f64, _closed: bool) {
    push_pieces_impl(occ, out, pts, None, None, e, k, tol, true)
}

static DBGLOG: std::sync::Mutex<Vec<String>> = std::sync::Mutex::new(Vec::new());
pub(crate) fn dbg_dump() -> Vec<String> {
    std::mem::take(&mut *DBGLOG.lock().unwrap())
}

fn push_pieces_impl(occ: &Occluders, out: &mut Vec<Piece>, pts: &[P2], test: Option<&[P2]>, arc: Option<f64>, e: usize, k: u8, tol: f64, _closed: bool) {
    let tp = test.unwrap_or(pts);
    let runs = if tp.len() == 2 { occ.runs_line(tp[0], tp[1], arc, tol) } else { occ.runs(tp, tol) };
    if std::env::var("FWDBG").is_ok() {
        eprintln!("push e={e} k={} pts={:?} runs={:?}", k as char, pts.iter().map(|p| (p.u, p.v, p.w)).collect::<Vec<_>>(), runs);
    }
    if runs.is_empty() {
        return;
    }
    for (t0, t1, v) in runs {
        let mut p = sub_polyline(pts, t0, t1, occ, tol);
        canonicalize(&mut p);
        if p.len() >= 2 {
            out.push(Piece { k, v, p, e });
        }
    }
}

/// Extract the polyline param range [t0, t1] (segment units), refining the
/// endpoints at visibility transitions.
fn sub_polyline(pts: &[P2], t0: f64, t1: f64, occ: &Occluders, tol: f64) -> Vec<P2> {
    let n = pts.len().saturating_sub(1);
    if n == 0 {
        return vec![];
    }
    let i0 = (t0.floor() as usize).min(n - 1);
    let i1 = (t1.ceil() as usize).min(n);
    let mut p = vec![];
    // start
    if t0.fract() < 1e-9 {
        p.push(pts[i0]);
    } else {
        let s = occ.split_param(pts[i0], pts[i0 + 1], t0.fract(), tol);
        p.push(pts[i0].lerp(&pts[i0 + 1], s));
    }
    for i in i0 + 1..i1 {
        p.push(pts[i]);
    }
    // end
    let last_seg = i1.min(n) - 1;
    if t1.fract() < 1e-9 || t1 >= n as f64 - 1e-9 {
        p.push(pts[(t1.round() as usize).min(n)]);
    } else {
        let s = occ.split_param(pts[last_seg], pts[last_seg + 1], t1.fract(), tol);
        p.push(pts[last_seg].lerp(&pts[last_seg + 1], s));
    }
    p
}

/// Drop spans fully covered by a nearer span of the same line family.
fn drop_covered(resolved: &mut Vec<(f64, f64, f64, f64, usize, Piece)>, tol: f64) {
    loop {
        let mut dropped = false;
        for i in 0..resolved.len() {
            let (ki, oi, _, _, si, xi) = &resolved[i];
            let (k, o) = (*ki, *oi);
            let xa = xi.p[0];
            let xb = xi.p[xi.p.len() - 1];
            let (mut dx, mut dy) = (xb.u - xa.u, xb.v - xa.v);
            let len = dx.hypot(dy);
            if len <= 0.0 {
                continue;
            }
            dx /= len;
            dy /= len;
            let span = |p: &P2| p.u * dx + p.v * dy;
            let (s0, s1) = (span(&xa).min(span(&xb)) + tol * 0.5, span(&xa).max(span(&xb)) - tol * 0.5);
            let mut cover = false;
            for (kj, oj, _, _, sj, yj) in resolved.iter() {
                if (*kj, *oj) != (k, o) || yj.p.len() < 2 {
                    continue;
                }
                let ya = yj.p[0];
                let yb = yj.p[yj.p.len() - 1];
                let (ss0, ss1) = (span(&ya).min(span(&yb)), span(&ya).max(span(&yb)));
                if !(ss0 <= s0 && ss1 >= s1) {
                    continue;
                }
                // nearer wins over the covered range; equal-depth ties break to
                // the later-inserted piece (pinned: box e5 over e9, the two's
                // e8, box3's silhouette over the coincident rim)
                let mut ok = true;
                let (p0, p1) = (span(&xa).min(span(&xb)), span(&xa).max(span(&xb)));
                for t in [0.15, 0.5, 0.85] {
                    let wx = xa.w + (xb.w - xa.w) * t;
                    let sy = span(&ya);
                    let ey = span(&yb);
                    let tt = if (ey - sy).abs() > 1e-12 { ((p0 + (p1 - p0) * t) - sy) / (ey - sy) } else { 0.5 };
                    let wy = ya.w + (yb.w - ya.w) * tt.clamp(0.0, 1.0);
                    let tie = (wy - wx).abs() <= tol;
                    if !(wy > wx + tol || (tie && sj > si)) {
                        ok = false;
                        break;
                    }
                }
                if ok {
                    cover = true;
                    break;
                }
            }
            if cover {
                resolved.remove(i);
                dropped = true;
                break;
            }
        }
        if !dropped {
            break;
        }
    }
}

/// Face outward normal at the surface point nearest `q`.
fn face_normal_at(body: &Body, f: FaceId, q: Vec3) -> Vec3 {
    let face = body.face(f);
    let (u, v) = face.surface.closest_param(q);
    body.outward_normal(f, u, v)
}
