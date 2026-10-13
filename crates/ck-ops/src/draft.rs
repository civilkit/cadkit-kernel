//! Draft faces about a neutral plane (ck-ops/src/draft.rs).
//!
//! Codes: draftParallelFace, draftAxisNotParallel, draftUnsupported,
//! draftTangentFaces, draftTooLarge, draftFailed; invalidValue (angle).
//! (Extrude-time draft lives in ck-topo make.rs; dome's `domeUnsupported`
//! belongs to the dome feature.)
//!
//! Wasm provenance (kernel.wasm): f2867 `draft` (the operation), f2868
//! cylinder -> cone, f2866 face interior point + normal, f1204 the feature
//! wrapper (ck-model `features/draft.rs`). The original hands the per-face
//! replacement surfaces to the generic offset engine (f2871, offset.rs:257)
//! and, when that fails, retries with `~slab` / `~wedge` boolean tools
//! (f2874). Neither is reconstructed: [`rebuild`] below is a small stand-in
//! for the engine (same contract: face surfaces replaced, ids / arena order
//! kept, edges and vertices re-derived by intersection) and the slab/wedge
//! retry is not ported, so those cases report `draftFailed`.
//!
//! Mapping of the offset engine's error classes to draft codes (table at
//! 0x817d98): too large -> draftTooLarge, unsupported -> draftUnsupported,
//! otherwise draftFailed.
//!
//! GUESS / known mismatch: the oracle's vertices carry ~1e-10 solver noise
//! (e.g. a box vertex at y = 9.999999999992346 for a 5 degree draft); here
//! they are exact to rounding.

use crate::boolean::classify::{face_loops_uv, point_in_face, Where};
use ck_geom::curve::Curve;
use ck_geom::curve2::Curve2;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec2, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, VertexId};
use ck_topo::edit::reverse_line_edge;
use ck_topo::make::plane_pcurve;

/// Error class: `code` is the wire error code, `detail` the optional
/// `params.detail` string.
#[derive(Debug, Clone, PartialEq)]
pub struct DraftError {
    pub code: &'static str,
    pub detail: Option<String>,
}

fn err(code: &'static str, detail: impl Into<String>) -> DraftError {
    DraftError { code, detail: Some(detail.into()) }
}

/// Smallest angle (rad) that still counts as a draft; the largest is just
/// below a right angle (1.5697963267948967 = pi/2 - 1e-3).
const ANGLE_MIN: f64 = 1e-9;
const ANGLE_MAX: f64 = 1.5697963267948967;

/// `1/l` for a usable length (the original's normalise guard), else `None`
/// (the vector becomes zero).
fn inv_len(l: f64) -> Option<f64> {
    if l > 1e-300 && l.is_finite() {
        Some(1.0 / l)
    } else {
        None
    }
}

/// Outward unit normal of `f` at the surface point nearest `p`.
fn outward_at(b: &Body, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = b.face(f).surface.closest_param(p);
    b.outward_normal(f, u, v)
}

/// f2866: a point strictly inside the face and the face's outward normal
/// there. Candidates are the centroid of the boundary polygon in uv, then a
/// grid over its bounding box.
pub fn interior_point(b: &Body, f: FaceId, tol: f64) -> Option<(Vec3, Vec3)> {
    let face = b.face(f);
    let loops = face_loops_uv(b, f);
    let pts: Vec<Vec2> = loops.iter().flatten().copied().filter(|p| p.x.is_finite() && p.y.is_finite()).collect();
    let mut cands: Vec<Vec2> = vec![];
    if pts.is_empty() {
        let ((u0, u1), (v0, v1)) = face.surface.domain();
        if u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite() {
            cands.push(Vec2::new(0.5 * (u0 + u1), 0.5 * (v0 + v1)));
        }
    } else {
        let n = pts.len() as f64;
        let c = pts.iter().fold(Vec2::ZERO, |a, p| Vec2::new(a.x + p.x, a.y + p.y));
        cands.push(Vec2::new(c.x / n, c.y / n));
        let (mut lo, mut hi) = (pts[0], pts[0]);
        for p in &pts {
            lo = Vec2::new(lo.x.min(p.x), lo.y.min(p.y));
            hi = Vec2::new(hi.x.max(p.x), hi.y.max(p.y));
        }
        for n in [3usize, 5, 9, 17] {
            for i in 0..n {
                for j in 0..n {
                    let (s, t) = ((i as f64 + 0.5) / n as f64, (j as f64 + 0.5) / n as f64);
                    cands.push(Vec2::new(lo.x + (hi.x - lo.x) * s, lo.y + (hi.y - lo.y) * t));
                }
            }
        }
    }
    for uv in cands {
        let p = face.surface.eval(uv.x, uv.y);
        if p.is_finite() && point_in_face(b, f, p, tol) == Where::In {
            return Some((p, b.outward_normal(f, uv.x, uv.y)));
        }
    }
    None
}

/// Plane branch of f2867: rotate the face plane about its intersection with
/// the neutral plane (`q`, pull direction `d`) by `ang`. `Err(())` when the
/// face normal is parallel to the pull direction.
fn draft_plane(fr: &Frame, sense: bool, d: Vec3, q: Vec3, ang: f64) -> Result<Surface, ()> {
    let (sn, cs) = (ck_topo::trig::sin(ang), ck_topo::trig::cos(ang));
    let s = if sense { 1.0 } else { -1.0 };
    let nx = (fr.x.y * fr.y.z - fr.x.z * fr.y.y) * s;
    let ny = s * (fr.x.z * fr.y.x - fr.y.z * fr.x.x);
    let nz = s * (fr.y.y * fr.x.x - fr.x.y * fr.y.x);
    // hinge direction h = N x d
    let hz_raw = d.y * nx - d.x * ny;
    let hx_raw = d.z * ny - d.y * nz;
    let hy_raw = d.x * nz - d.z * nx;
    let l = (hz_raw * hz_raw + hx_raw * hx_raw + hy_raw * hy_raw).sqrt();
    if !(l >= 1e-9) {
        return Err(());
    }
    let (mut hx, mut hy, mut hz) = (0.0, 0.0, 0.0);
    if let Some(inv) = inv_len(l) {
        hz = hz_raw * inv;
        hy = hy_raw * inv;
        hx = hx_raw * inv;
    }
    // slope direction w = h x N
    let wz = ny * hx - nx * hy;
    let wx = nz * hy - ny * hz;
    let wy = nx * hz - nz * hx;
    // rotated normal
    let z1 = cs * nz + sn * wz;
    let x1 = cs * nx + sn * wx;
    let y1 = cs * ny + sn * wy;
    let l2 = (z1 * z1 + x1 * x1 + y1 * y1).sqrt();
    let (mut n2x, mut n2y, mut n2z) = (0.0, 0.0, 0.0);
    if let Some(inv) = inv_len(l2) {
        n2z = z1 * inv;
        n2y = y1 * inv;
        n2x = x1 * inv;
    }
    // slide the origin along w onto the neutral plane
    let t = (d.z * (q.z - fr.o.z) + d.x * (q.x - fr.o.x) + d.y * (q.y - fr.o.y)) / (d.z * wz + d.x * wx + d.y * wy);
    let o = Vec3::new(fr.o.x + wx * t, fr.o.y + wy * t, fr.o.z + wz * t);
    // the oracle's normals come out normalised a second time (probe: 4, 6,
    // 7, 10 ... degrees match only with two passes), as if the frame were
    // rebuilt from (o, z, x) afterwards
    let z = Vec3::new(n2x, n2y, n2z).normalize();
    let x = Vec3::new(hx, hy, hz);
    let y = z.cross(x);
    Ok(Surface::Plane { frame: Frame { o, x, y, z } })
}

/// f2868: the cone that replaces a cylinder (or a degenerate cone) whose
/// axis is parallel to the pull direction: apex-less cone through the
/// cylinder's circle at the neutral plane, half angle `+-ang` (negative when
/// the face normal points away from the axis).
#[allow(clippy::too_many_arguments)]
fn cone_from_cylinder(
    r: f64,
    fr: &Frame,
    d: Vec3,
    q: Vec3,
    ang: f64,
    inside: (Vec3, Vec3),
) -> Surface {
    let (o, ax, xr) = (fr.o, fr.z, fr.x);
    // reference direction: the frame x made perpendicular to the pull dir
    let s0 = xr.z * d.z + xr.x * d.x + xr.y * d.y;
    let (zz, zx, zy) = (xr.z - d.z * s0, xr.x - d.x * s0, xr.y - d.y * s0);
    let l = (zz * zz + zx * zx + zy * zy).sqrt();
    let (mut ux, mut uy, mut uz) = (0.0, 0.0, 0.0);
    if let Some(inv) = inv_len(l) {
        uz = zz * inv;
        uy = zy * inv;
        ux = zx * inv;
    }
    // axis point at the neutral level
    let t = (q.y - o.y) * ax.y + (q.x - o.x) * ax.x + (q.z - o.z) * ax.z;
    let a = Vec3::new(o.x + ax.x * t, o.y + ax.y * t, o.z + ax.z * t);
    let (p, n) = inside;
    let (vz, vx, vy) = (p.z - a.z, p.x - a.x, p.y - a.y);
    let s1 = d.z * vz + d.x * vx + d.y * vy;
    let (vz, vx, vy) = (vz - d.z * s1, vx - d.x * s1, vy - d.y * s1);
    let l = (vz * vz + vx * vx + vy * vy).sqrt();
    let (mut rx, mut ry, mut rz) = (0.0, 0.0, 0.0);
    if let Some(inv) = inv_len(l) {
        rz = vz * inv;
        ry = vy * inv;
        rx = vx * inv;
    }
    let y = Vec3::new(d.y * uz - d.z * uy, d.z * ux - d.x * uz, d.x * uy - d.y * ux);
    let semi = if n.z * rz + n.y * ry + n.x * rx <= 0.0 { ang } else { -ang };
    Surface::Cone { frame: Frame { o: a, x: Vec3::new(ux, uy, uz), y, z: d }, r, semi_angle: semi }
}

/// Wire kind names used in `draftUnsupported` details.
fn kind_name(s: &Surface) -> &'static str {
    match s {
        Surface::Plane { .. } => "plane",
        Surface::Cylinder { .. } => "cylinder",
        Surface::Cone { .. } => "cone",
        Surface::Sphere { .. } => "sphere",
        Surface::Torus { .. } => "torus",
        Surface::BSpline(_) | Surface::Screw { .. } => "spline",
        Surface::Offset { .. } => "offset",
        Surface::Extrusion { .. } => "extrusion",
        Surface::Revolution { .. } => "revolution",
    }
}

/// One replaced face: new surface and sense.
#[derive(Debug, Clone)]
struct Edit {
    face: FaceId,
    surface: Surface,
    sense: bool,
}

/// Draft `faces` of `body` by `angle` (radians, signed) about the neutral
/// plane through `neutral` with pull direction `dir` (f2867).
///
/// The face normals tilt towards the pull direction for a positive angle;
/// points on the neutral plane stay put. Errors: `invalidValue` (angle out
/// of range), `draftTangentFaces` ("<a> and <b>": a drafted face is tangent
/// to a neighbour that is not drafted), `draftParallelFace` (face normal
/// parallel to the pull direction), `draftAxisNotParallel` (cylinder axis
/// not parallel to it), `draftUnsupported` ("<face>: <kind>"),
/// `draftTooLarge` (an edge collapses or turns over), `draftFailed`.
pub fn draft(body: &Body, neutral: Vec3, dir: Vec3, faces: &[FaceId], angle: f64, tol: f64) -> Result<Body, DraftError> {
    let a = angle.abs();
    if !(a > ANGLE_MIN && a < ANGLE_MAX) {
        return Err(DraftError { code: "invalidValue", detail: None });
    }
    let d = dir.normalize();

    // a drafted face may not be tangent to a neighbour that stays put
    for &f in faces {
        for e in body.face_edges(f) {
            for g in body.edge_faces(e) {
                if g == f || faces.contains(&g) {
                    continue;
                }
                let m = body.edge_mid(e);
                let (nf, ng) = (outward_at(body, f, m), outward_at(body, g, m));
                if nf.x * ng.x + nf.y * ng.y + nf.z * ng.z > 0.999999 {
                    return Err(err("draftTangentFaces", format!("{} and {}", body.face_id_str(f), body.face_id_str(g))));
                }
            }
        }
    }

    let mut edits: Vec<Edit> = vec![];
    for &f in faces {
        let face = body.face(f);
        let name = body.face_id_str(f);
        let surface = match &face.surface {
            Surface::Plane { frame } => match draft_plane(frame, face.sense, d, neutral, angle) {
                Ok(s) => s,
                Err(()) => return Err(err("draftParallelFace", name)),
            },
            Surface::Cylinder { frame, r } => cyl_to_cone(body, f, frame, *r, d, neutral, angle, tol, &name)?,
            Surface::Cone { frame, r, semi_angle } if semi_angle.abs() < 1e-12 => {
                cyl_to_cone(body, f, frame, *r, d, neutral, angle, tol, &name)?
            }
            other => return Err(err("draftUnsupported", format!("{}: {}", name, kind_name(other)))),
        };
        let Some((p, n_old)) = interior_point(body, f, tol) else {
            return Err(err("draftFailed", format!("{name}: no interior point")));
        };
        let (u, v) = surface.closest_param(p);
        let nn = surface.normal(u, v);
        let sense = n_old.z * nn.z + n_old.x * nn.x + n_old.y * nn.y > 0.0;
        let edit = Edit { face: f, surface, sense };
        match edits.iter_mut().find(|e| e.face == f) {
            Some(slot) => *slot = edit,
            None => edits.push(edit),
        }
    }
    rebuild(body, &edits)
}

#[allow(clippy::too_many_arguments)]
fn cyl_to_cone(
    body: &Body,
    f: FaceId,
    fr: &Frame,
    r: f64,
    d: Vec3,
    neutral: Vec3,
    angle: f64,
    tol: f64,
    name: &str,
) -> Result<Surface, DraftError> {
    let (ax, ay, az) = (fr.z.x, fr.z.y, fr.z.z);
    let (ca, cb, cc) = (d.y * ax - d.x * ay, d.z * ay - d.y * az, d.x * az - d.z * ax);
    let n = (ca * ca + cb * cb + cc * cc).sqrt();
    if n > 1e-9 {
        return Err(err("draftAxisNotParallel", name));
    }
    let Some(inside) = interior_point(body, f, tol) else {
        return Err(DraftError { code: "draftFailed", detail: Some("no interior point".into()) });
    };
    Ok(cone_from_cylinder(r, fr, d, neutral, angle, inside))
}

// ------------------------------------------------------------------ rebuild

/// Least-change point on the intersection of `surfs`, starting at `p`
/// (Gauss-Newton with minimum-norm steps).
fn solve_vertex(mut p: Vec3, surfs: &[&Surface]) -> Option<Vec3> {
    for _ in 0..40 {
        let mut rows: Vec<(Vec3, f64)> = vec![];
        for s in surfs {
            let (u, v) = s.closest_param(p);
            let q = s.eval(u, v);
            let n = s.normal(u, v).normalize();
            rows.push((n, (p - q).dot(n)));
        }
        // already on every surface (a vertex on the neutral plane): leave it
        // bit-exact instead of letting rounding noise move it
        if rows.iter().all(|r| r.1.abs() < 1e-13) {
            return Some(p);
        }
        let step = match rows.len() {
            0 => return Some(p),
            1 => rows[0].0 * rows[0].1,
            2 => {
                let (a, b) = (rows[0], rows[1]);
                let (aa, ab, bb) = (a.0.dot(a.0), a.0.dot(b.0), b.0.dot(b.0));
                let det = aa * bb - ab * ab;
                if det.abs() < 1e-18 {
                    return None;
                }
                let l0 = (bb * a.1 - ab * b.1) / det;
                let l1 = (aa * b.1 - ab * a.1) / det;
                a.0 * l0 + b.0 * l1
            }
            3 => {
                let m = ck_core::linalg::Mat3::from_rows(rows[0].0, rows[1].0, rows[2].0);
                let inv = m.inverse()?;
                inv.mul_vec(Vec3::new(rows[0].1, rows[1].1, rows[2].1))
            }
            _ => {
                // least squares: (J^T J) x = J^T r
                let mut jtj = ck_core::linalg::Mat3::from_rows(Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
                let mut jtr = Vec3::ZERO;
                for (n, r) in &rows {
                    jtj = ck_core::linalg::Mat3::from_rows(
                        jtj.row(0) + *n * n.x,
                        jtj.row(1) + *n * n.y,
                        jtj.row(2) + *n * n.z,
                    );
                    jtr = jtr + *n * *r;
                }
                jtj.inverse()?.mul_vec(jtr)
            }
        };
        p = p - step;
        if !p.is_finite() {
            return None;
        }
        if step.len() < 1e-14 {
            break;
        }
    }
    Some(p)
}

/// Curve of the intersection of two (new) face surfaces near `near`.
fn new_edge_curve(sa: &Surface, sb: &Surface, old: &Curve, near: Vec3) -> Option<Curve> {
    match (sa, sb) {
        (Surface::Plane { frame: fa }, Surface::Plane { frame: fb }) => {
            let (na, nb) = (fa.z, fb.z);
            let dir = na.cross(nb);
            let l2 = dir.len2();
            if l2 < 1e-24 {
                return None;
            }
            // point of the line nearest `near`
            let (ha, hb) = (na.dot(fa.o), nb.dot(fb.o));
            let hc = dir.dot(near);
            let det = l2;
            let p = (nb.cross(dir) * ha + dir.cross(na) * hb + na.cross(nb) * hc) * (1.0 / det);
            // the old line still lies on both planes (the edge on the neutral
            // plane): keep it bit-exact
            if let Curve::Line { o: oo, d: od } = old {
                let nd = dir.normalize();
                if od.cross(nd).len() < 1e-12 && (na.dot(*oo) - ha).abs() < 1e-12 && (nb.dot(*oo) - hb).abs() < 1e-12 {
                    return Some(Curve::Line { o: *oo, d: *od });
                }
            }
            Some(Curve::Line { o: p, d: dir.normalize() })
        }
        (Surface::Plane { frame: pf }, s @ (Surface::Cone { frame: cf, .. } | Surface::Cylinder { frame: cf, .. }))
        | (s @ (Surface::Cone { frame: cf, .. } | Surface::Cylinder { frame: cf, .. }), Surface::Plane { frame: pf })
            if pf.z.cross(cf.z).len() < 1e-9 =>
        {
            // plane perpendicular to the axis: circle about the axis
            let s_ax = pf.z.dot(pf.o - cf.o) / pf.z.dot(cf.z);
            let c = cf.o + cf.z * s_ax;
            let rad = match s {
                Surface::Cone { r, semi_angle, .. } => r + s_ax * semi_angle.tan(),
                Surface::Cylinder { r, .. } => *r,
                _ => unreachable!(),
            };
            let Curve::Circle { frame: of, .. } = old else { return None };
            Some(Curve::Circle { frame: Frame { o: c, ..*of }, r: rad })
        }
        _ => {
            let r = crate::intersect::ss::intersect(sa, sb, None, 1e-9);
            let mut best: Option<(f64, Curve)> = None;
            for c in r.curves() {
                let cv = c.curve();
                let (_, q) = cv.closest_point(near);
                let dd = q.dist(near);
                if best.as_ref().map_or(true, |b| dd < b.0) {
                    best = Some((dd, cv.clone()));
                }
            }
            best.map(|b| b.1)
        }
    }
}

/// Sort key of a face name: feature, role (Side < Cap < Blend < Other),
/// role payload, region, instance suffix.
fn name_key(b: &Body, f: FaceId) -> (String, u8, String, Option<u32>, String) {
    use ck_topo::body::Role;
    let n = &b.face(f).name;
    let (rank, payload) = match &n.role {
        Role::Side(e) => (0, e.clone()),
        Role::Cap(k) => (1, format!("{k:010}")),
        Role::Blend(k) => (2, format!("{k:010}")),
        Role::Other(o) => (3, o.clone()),
    };
    (n.feature.clone(), rank, payload, n.region, n.suffix.clone())
}

fn pair_name(b: &Body, fs: &[FaceId]) -> String {
    match fs {
        [a, c] => format!("between {} and {}", b.face_id_str(*a), b.face_id_str(*c)),
        [a] => b.face_id_str(*a),
        _ => String::new(),
    }
}

/// Stand-in for the offset engine (f2871): replace the face surfaces and
/// re-derive every touched edge and vertex. Ids and arena order are kept.
fn rebuild(b: &Body, edits: &[Edit]) -> Result<Body, DraftError> {
    let mut nb = b.clone();
    let changed: Vec<FaceId> = edits.iter().map(|e| e.face).collect();
    for e in edits {
        let f = &mut nb.faces[e.face.idx()];
        f.surface = e.surface.clone();
        f.sense = e.sense;
    }
    let touched: Vec<EdgeId> =
        b.edge_ids().filter(|&e| b.edge_faces(e).iter().any(|f| changed.contains(f))).collect();
    let failed = |m: &str| err("draftFailed", m);

    // vertices of the touched edges, solved against all their faces
    let mut verts: Vec<VertexId> = vec![];
    for &e in &touched {
        let ed = b.edge(e);
        for v in [ed.v0, ed.v1] {
            if !verts.contains(&v) {
                verts.push(v);
            }
        }
    }
    let mut newp: Vec<(VertexId, Vec3)> = vec![];
    for &v in &verts {
        let mut fs: Vec<FaceId> = vec![];
        for e in b.edge_ids() {
            let ed = b.edge(e);
            if ed.v0 == v || ed.v1 == v {
                for f in b.edge_faces(e) {
                    if !fs.contains(&f) {
                        fs.push(f);
                    }
                }
            }
        }
        let surfs: Vec<&Surface> = fs.iter().map(|&f| &nb.faces[f.idx()].surface).collect();
        let p = solve_vertex(b.vertex(v).p, &surfs)
            .ok_or_else(|| failed(&format!("no vertex position near {:?}", b.vertex(v).p)))?;
        newp.push((v, p));
    }
    let pos = |v: VertexId| newp.iter().find(|x| x.0 == v).map(|x| x.1).unwrap_or_else(|| b.vertex(v).p);

    // edge curves
    for &e in &touched {
        let ed = b.edge(e).clone();
        let fs = b.edge_faces(e);
        let (p0, p1) = (pos(ed.v0), pos(ed.v1));
        let near = {
            let m = b.edge_mid(e);
            // mid of the old edge moved by the average vertex displacement
            m + ((p0 - b.vertex(ed.v0).p) + (p1 - b.vertex(ed.v1).p)) * 0.5
        };
        let (curve, t0, t1);
        if b.is_seam(e) {
            if p0.dist(p1) < 1e-9 {
                return Err(err("draftTooLarge", format!("edge {} shrinks to nothing", pair_name(b, &fs))));
            }
            curve = Curve::line_through(p0, p1);
            t0 = 0.0;
            t1 = p0.dist(p1);
        } else {
            if fs.len() != 2 {
                return Err(failed(&format!("edge {} with {} faces", e.0, fs.len())));
            }
            let (fa, fb) = if fs[0].0 <= fs[1].0 { (fs[0], fs[1]) } else { (fs[1], fs[0]) };
            let c = new_edge_curve(&nb.faces[fa.idx()].surface, &nb.faces[fb.idx()].surface, &ed.curve, near)
                .ok_or_else(|| failed("edges do not intersect"))?;
            match c {
                Curve::Line { o, d } => {
                    // keep the old direction; the draft rule re-orients below
                    let old_d = match &ed.curve {
                        Curve::Line { d, .. } => *d,
                        _ => d,
                    };
                    let d = if d.dot(old_d) < 0.0 { -d } else { d };
                    let ln = Curve::Line { o, d };
                    let (a, bb) = (ln.closest_param(p0), ln.closest_param(p1));
                    let span = (b.vertex(ed.v1).p - b.vertex(ed.v0).p).len().max(1e-9);
                    if bb - a < -1e-9 * span.max(1.0) {
                        // GUESS: the oracle names the first edge (lowest id)
                        // at the collapsing corner (draft 89 degrees probe)
                        let first = b.edge_ids().find(|&x| b.edge(x).v0 == ed.v0 || b.edge(x).v1 == ed.v0).unwrap_or(e);
                        return Err(err("draftTooLarge", format!("edge {} turns over", pair_name(b, &b.edge_faces(first)))));
                    }
                    if bb - a <= 1e-9 {
                        return Err(err("draftTooLarge", format!("edge {} shrinks to nothing", pair_name(b, &[fa, fb]))));
                    }
                    curve = ln;
                    t0 = a;
                    t1 = bb;
                }
                Curve::Circle { frame, r } => {
                    if !(r > 1e-9) {
                        return Err(err("draftTooLarge", format!("edge {} shrinks to nothing", pair_name(b, &[fa, fb]))));
                    }
                    let ln = Curve::Circle { frame, r };
                    if ed.v0 == ed.v1 {
                        t0 = ed.t0;
                        t1 = ed.t1;
                    } else {
                        let a = ln.closest_param(p0);
                        let mut bb = ln.closest_param(p1);
                        while bb <= a {
                            bb += std::f64::consts::TAU;
                        }
                        t0 = a;
                        t1 = bb;
                    }
                    curve = ln;
                }
                other => {
                    let (a, bb) = (other.closest_param(p0), other.closest_param(p1));
                    t0 = a;
                    t1 = bb;
                    curve = other;
                }
            }
        }
        let slot = &mut nb.edges[e.idx()];
        slot.curve = curve;
        slot.t0 = t0;
        slot.t1 = t1;
    }
    for (v, p) in &newp {
        nb.vertices[v.idx()].p = *p;
    }
    // edges that keep their curve but end at a moved vertex slide along it
    for e in b.edge_ids() {
        if touched.contains(&e) {
            continue;
        }
        let ed = b.edge(e);
        if !(newp.iter().any(|x| x.0 == ed.v0) || newp.iter().any(|x| x.0 == ed.v1)) || ed.v0 == ed.v1 {
            continue;
        }
        let (a, bb) = (ed.curve.closest_param(pos(ed.v0)), ed.curve.closest_param(pos(ed.v1)));
        if !(bb > a) {
            return Err(err("draftTooLarge", format!("edge {} turns over", pair_name(b, &b.edge_faces(e)))));
        }
        let slot = &mut nb.edges[e.idx()];
        slot.t0 = a;
        slot.t1 = bb;
    }

    // plane/plane edges are re-derived as n_a x n_b (natural normals):
    // reverse the ones that run the other way (oracle: the drafted extrude
    // probes pinned in ck-topo make.rs `orient_draft_edges`, golden brep.json)
    for &e in &touched {
        let fs = b.edge_faces(e);
        if fs.len() != 2 {
            continue;
        }
        // faces in name order (side faces sort before caps: cap edges run
        // side x cap in natural normals; rails by entity id)
        let (fa, fb) = if name_key(&nb, fs[0]) <= name_key(&nb, fs[1]) { (fs[0], fs[1]) } else { (fs[1], fs[0]) };
        let (Surface::Plane { frame: ra }, Surface::Plane { frame: rb }) = (&nb.faces[fa.idx()].surface, &nb.faces[fb.idx()].surface)
        else {
            continue;
        };
        let want = ra.z.cross(rb.z);
        let Curve::Line { d, .. } = nb.edges[e.idx()].curve else { continue };
        if d.dot(want) < 0.0 {
            reverse_line_edge(&mut nb, e);
            // negating a unit axis leaves -0 components; DIRECTION sharing in
            // the STEP writer is keyed by exact bits, so normalise them
            if let Curve::Line { o, d } = nb.edges[e.idx()].curve {
                nb.edges[e.idx()].curve = Curve::Line { o, d: Vec3::new(d.x + 0.0, d.y + 0.0, d.z + 0.0) };
            }
        }
    }

    // pcurves of every coedge on a touched edge
    for &e in &touched {
        for c in nb.edge_coedges(e) {
            let f = nb.coedge_face(c);
            let curve = nb.edges[e.idx()].curve.clone();
            let old = nb.coedges[c.idx()].pcurve.clone();
            let pc = match &nb.faces[f.idx()].surface {
                Surface::Plane { frame } => plane_pcurve(frame, &curve),
                Surface::Cone { frame, .. } | Surface::Cylinder { frame, .. } => match (old, &curve) {
                    (Some(Curve2::Line { o, d }), Curve::Circle { frame: cf, .. }) if d.y == 0.0 => {
                        Some(Curve2::Line { o: Vec2::new(o.x, (cf.o - frame.o).dot(frame.z)), d })
                    }
                    (Some(Curve2::Line { o, d }), Curve::Line { o: lo, d: ld }) if d.x == 0.0 => {
                        Some(Curve2::Line { o: Vec2::new(o.x, (*lo - frame.o).dot(frame.z)), d: Vec2::new(0.0, ld.dot(frame.z)) })
                    }
                    _ => None,
                },
                _ => None,
            };
            nb.coedges[c.idx()].pcurve = pc;
        }
    }
    Ok(nb)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean::tests::box_body;

    fn face(b: &Body, name: &str) -> FaceId {
        b.find_face(name).unwrap_or_else(|| panic!("no face {name}"))
    }

    #[test]
    fn box_sides_about_bottom() {
        // box_body: feature F2 on the top-style plane, z in [s0, s1]
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let sides: Vec<FaceId> = b.face_ids().filter(|&f| b.face_id_str(f).contains(".S.")).collect();
        let out = draft(&b, Vec3::ZERO, Vec3::Y, &sides, 5f64.to_radians(), 1e-6).unwrap();
        assert!(ck_topo::check::check(&out, 1e-9).is_empty(), "{:?}", ck_topo::check::check(&out, 1e-9));
        let (lo, hi) = out.bbox();
        let t = 10.0 * 5f64.to_radians().tan();
        assert!((hi.x - 40.0).abs() < 1e-9 && lo.x.abs() < 1e-9, "{lo:?} {hi:?}");
        // top cap shrinks by tan(5 deg) * 10 on every side (inward)
        let top = out.edge_ids().map(|e| out.edge_mid(e)).filter(|m| (m.y - 10.0).abs() < 1e-9).count();
        assert_eq!(top, 4);
        let _ = t;
    }

    #[test]
    fn parallel_face_is_rejected() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let cap = b.face_ids().find(|&f| b.face_id_str(f).contains(".C1")).unwrap();
        let e = draft(&b, Vec3::ZERO, Vec3::Y, &[cap], 0.1, 1e-6).unwrap_err();
        assert_eq!(e.code, "draftParallelFace");
        assert_eq!(e.detail.as_deref(), Some("F2.C1"));
        let _ = face(&b, "F2.C0");
    }

    #[test]
    fn cylinder_becomes_cone() {
        use crate::boolean::tests::cyl_body;
        let b = cyl_body("F2", 5.0, 3.0, 10.0, 0.0, 10.0);
        let side = b.face_ids().find(|&f| b.face_id_str(f).contains(".S.")).unwrap();
        let out = draft(&b, Vec3::ZERO, Vec3::Y, &[side], 3f64.to_radians(), 1e-6).unwrap();
        assert!(ck_topo::check::check(&out, 1e-9).is_empty(), "{:?}", ck_topo::check::check(&out, 1e-9));
        let Surface::Cone { r, semi_angle, .. } = &out.face(side).surface else { panic!("not a cone") };
        assert!((r - 10.0).abs() < 1e-12 && (semi_angle.abs() - 3f64.to_radians()).abs() < 1e-12);
        // top circle shrinks by 10 * tan(3 deg)
        let (lo, hi) = out.bbox();
        assert!(((hi.x - lo.x) - 2.0 * (10.0 - 10.0 * 3f64.to_radians().tan()).max(10.0)).abs() < 1e-6, "{lo:?} {hi:?}");
    }

    #[test]
    fn angle_range() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let f = face(&b, "F2.S.e1");
        for a in [0.0, 1e-10, 1.5707963, 3.0] {
            assert_eq!(draft(&b, Vec3::ZERO, Vec3::Y, &[f], a, 1e-6).unwrap_err().code, "invalidValue");
        }
    }
}
