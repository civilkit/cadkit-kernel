//! Thicken: turn a sheet (open shell) into a solid wall.
//!
//! Semantics pinned by oracle probes (square-tube `surfaceExtrude` + thicken
//! t=2 front/back/both, single planar sheet front/back/both, cylinder tube
//! front — all with massProps):
//! - Material is added on the natural-normal side (`front`), the opposite
//!   side (`back`), or both (`both`); for `both` the surface itself is not
//!   part of the result.
//! - Face order: the layer whose natural normal points INTO the material
//!   (the smaller-d layer) first — its faces end up sense-flipped relative
//!   to natural — then the other layer, then the rim walls `F3.W0…` in
//!   (sheet-face order × open-rim order).
//! - Layer suffixes `~k`: layers sorted by `d·s` ascending (d = signed
//!   offset along the natural normal, s = +1 when the normal points along a
//!   positive world axis / radially outward). Probe: tube e1 (normal −y):
//!   front copy y=−2 → `~1`, original → `~2`; e2 (normal +x): original
//!   `~1`, copy x=12 → `~2`.
//! - The offset copy of a closed planar shell is EXTENDED at convex corners
//!   to the miter lines (tube: offset rails at (−2,−2), rim edges extended
//!   x −2..12). Walls run between the first layer's rim edge and the second
//!   layer's extended rim edge, closed by straight connectors created while
//!   walking each wall CCW about its outward normal `u × nf` (this pins the
//!   connector edge order: probe W0 → end connector first, W1 → start
//!   first). Closed rim edges (cylinder) give annulus walls (two loops).
//! - Errors: non-positive thickness → `invalidValue`; offset past the
//!   centre of curvature → `offsetTooLarge` ("the offset passes the
//!   surface's centre of curvature"); miter past a face → `thickenReach`;
//!   anything not closing → `thickenFailed`.
//!
//! HOW TO WIRE (ck-model, Gate B): feature `thicken {surface, thickness,
//! side: front|back|both}` → resolve the surface body (`missingReference`),
//! then `thicken::thicken(&sheet, &feat_id, thickness, side, 1e-6)`; map
//! `ThickenError {code, detail}` onto the FeatureError (strings are
//! kernel-verbatim). The solid is registered under the feature id.

use crate::surfacing::knit::BodyExt;
use ck_geom::curve::Curve;
use ck_geom::nurbs::bsurf::BSplineSurface;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, LoopId, Role, ShellId, VertexId};
use std::ops::Add;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Side {
    Front,
    Back,
    Both,
}

impl Side {
    pub fn parse(s: &str) -> Option<Side> {
        match s {
            "front" => Some(Side::Front),
            "back" => Some(Side::Back),
            "both" => Some(Side::Both),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ThickenError {
    pub code: &'static str,
    pub detail: String,
    /// Why the offset is impossible, in numbers (`maxThickness` is the thickness that would work).
    pub limit: Option<crate::offset::OffsetLimit>,
}

impl ThickenError {
    fn code(code: &'static str) -> ThickenError {
        ThickenError { code, detail: String::new(), limit: None }
    }
    fn detail(code: &'static str, detail: impl Into<String>) -> ThickenError {
        ThickenError { code, detail: detail.into(), limit: None }
    }
}

impl std::fmt::Display for ThickenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.detail.is_empty() {
            write!(f, "{}", self.code)
        } else {
            write!(f, "{}: {}", self.code, self.detail)
        }
    }
}

/// Natural (sense-independent) surface normal. For cylinders this is the
/// radial direction at the domain middle, oriented positively (the layer
/// suffix and wall-side rules treat a radial normal as +1, probe-pinned).
pub(crate) fn natural_normal(s: &Surface) -> Vec3 {
    let n = match s {
        Surface::Plane { frame } => frame.z,
        Surface::Cylinder { frame, .. } => {
            let ((u0, u1), _) = s.domain();
            let u = (u0 + u1) * 0.5;
            let p = s.eval(u, 0.0);
            let n = (p - closest_axis_point(frame, p)).normalize();
            // a radial normal counts as positive regardless of direction
            if positive_axis_sign(n) < 0.0 {
                n.scale(-1.0)
            } else {
                n
            }
        }
        _ => {
            let ((u0, u1), (v0, v1)) = s.domain();
            // an infinite v-domain (cone) has no midpoint: sample the reference v = 0
            let v = if v0.is_finite() && v1.is_finite() { (v0 + v1) * 0.5 } else { 0.0 };
            s.normal((u0 + u1) * 0.5, v).normalize()
        }
    };
    n
}

/// Unit radial direction of `p` about the axis of `frame`.
fn radial_of(frame: &Frame, p: Vec3) -> Vec3 {
    let q = p - frame.o;
    (q - frame.z.scale(q.dot(frame.z))).try_normalize().unwrap_or(frame.x)
}

/// Unit normal of a cone along the generator through the radial direction `rad`: constant along the generator.
fn cone_normal_dir(frame: &Frame, semi_angle: f64, rad: Vec3) -> Vec3 {
    rad.scale(semi_angle.cos()) - frame.z.scale(semi_angle.sin())
}

/// +1 when the face's oriented normal is the cone's outward normal (sense true), else -1. A cone patch built by a
/// loft whose section runs clockwise about the loft direction keeps the orientation the free-form surface had.
fn cone_sign(sheet: &Body, f: FaceId) -> f64 {
    if sheet.face(f).sense {
        1.0
    } else {
        -1.0
    }
}

/// Mid angle (about the cone axis) of the points a face's boundary covers; a full ring has no mid.
fn face_mid_radial(sheet: &Body, f: FaceId, frame: &Frame) -> Vec3 {
    let mut acc = Vec3::ZERO;
    for &l in &sheet.face(f).loops {
        if !sheet.lp(l).alive {
            continue;
        }
        for &c in &sheet.lp(l).coedges {
            let co = sheet.coedge(c);
            if !co.alive {
                continue;
            }
            let e = sheet.edge(co.edge);
            for k in 0..=8 {
                let t = e.t0 + (e.t1 - e.t0) * k as f64 / 8.0;
                acc = acc + radial_of(frame, e.curve.eval(t));
            }
        }
    }
    acc.try_normalize().unwrap_or(frame.x)
}

/// The face's normal as thicken uses it: the surface's natural normal; for a cone, the oriented normal at the
/// middle of the patch (the cone's normal turns with the angle, one fixed direction is wrong for a patch that
/// spans a large angle).
fn face_normal(sheet: &Body, f: FaceId) -> Vec3 {
    match &sheet.face(f).surface {
        Surface::Cone { frame, semi_angle, .. } => cone_normal_dir(frame, *semi_angle, face_mid_radial(sheet, f, frame)).scale(cone_sign(sheet, f)),
        s => natural_normal(s),
    }
}

/// The face's normal at the point `p` of its boundary (`fallback` for a surface whose normal is constant).
fn normal_at(sheet: &Body, f: FaceId, p: Vec3, fallback: Vec3) -> Vec3 {
    match &sheet.face(f).surface {
        Surface::Cone { frame, semi_angle, .. } => cone_normal_dir(frame, *semi_angle, radial_of(frame, p)).scale(cone_sign(sheet, f)),
        _ => fallback,
    }
}

/// Signed offset along the surface's own normal (`d` along the face's oriented normal).
fn surface_offset(sheet: &Body, f: FaceId, d: f64) -> f64 {
    match &sheet.face(f).surface {
        Surface::Cone { .. } => d * cone_sign(sheet, f),
        _ => d,
    }
}

fn same_cone(a: &Surface, b: &Surface) -> bool {
    match (a, b) {
        (Surface::Cone { frame: fa, r: ra, semi_angle: sa }, Surface::Cone { frame: fb, r: rb, semi_angle: sb }) => {
            let scale = 1.0 + ra.abs().max(rb.abs());
            fa.z.cross(fb.z).len() < 1e-9
                && fa.z.dot(fb.z) > 0.0
                && (fb.o - fa.o).cross(fa.z).len() < 1e-9 * scale
                && (sa - sb).abs() < 1e-9
                && ((fb.o - fa.o).dot(fa.z) * sa.tan() + rb - ra).abs() < 1e-9 * scale
        }
        _ => false,
    }
}

/// Closest point on the cylinder axis through `frame.o` along `frame.z`.
fn closest_axis_point(frame: &Frame, p: Vec3) -> Vec3 {
    frame.o + frame.z.scale((p - frame.o).dot(frame.z))
}

/// Offset `s` by `d` along its natural normal (None when the offset
/// collapses the surface or the type is unsupported here).
fn offset_surface(s: &Surface, d: f64) -> Option<Surface> {
    match s {
        Surface::Plane { frame } => Some(Surface::Plane {
            frame: Frame { o: frame.o + frame.z.scale(d), ..frame.clone() },
        }),
        Surface::Cylinder { frame, r } => {
            let r2 = r + d;
            (r2 > 1e-9).then(|| Surface::Cylinder { frame: frame.clone(), r: r2 })
        }
        Surface::Cone { frame, r, semi_angle } => {
            let r2 = r + d / semi_angle.cos();
            (r2 > 1e-9).then(|| Surface::Cone { frame: frame.clone(), r: r2, semi_angle: *semi_angle })
        }
        Surface::Sphere { frame, r } => {
            let r2 = r + d;
            (r2 > 1e-9).then(|| Surface::Sphere { frame: frame.clone(), r: r2 })
        }
        Surface::Torus { frame, major, minor } => {
            let m2 = major + d;
            (m2 > *minor + 1e-9).then(|| Surface::Torus { frame: frame.clone(), major: m2, minor: *minor })
        }
        _ => None,
    }
}

/// +1 when the unit normal points along a positive canonical axis (world
/// axis or radial outward), else −1. Pins the `~1/~2` layer suffix order.
fn positive_axis_sign(n: Vec3) -> f64 {
    let (ax, ay, az) = (n.x.abs(), n.y.abs(), n.z.abs());
    let m = ax.max(ay).max(az);
    let dom = if m == ax {
        n.x
    } else if m == ay {
        n.y
    } else {
        n.z
    };
    if dom < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Layer suffix `~k` of layer `li`: rank by `d·s` ascending (probe-pinned).
fn layer_suffix(layers: &[(f64, bool)], li: usize, n: Vec3) -> u32 {
    let s = positive_axis_sign(n);
    let mut keyed: Vec<(f64, usize)> = layers.iter().enumerate().map(|(i, &(d, _))| (d * s, i)).collect();
    keyed.sort_by(|a, b| a.0.total_cmp(&b.0));
    keyed.iter().position(|&(_, i)| i == li).map_or(1, |p| p as u32 + 1)
}

struct Layer {
    /// signed offset along the face's natural normal
    d: f64,
    original: bool,
    edges: std::collections::HashMap<EdgeId, EdgeId>,
}

/// Translate a curve by `disp` (exact for line/circle, straight fallback).
fn translate_curve(c: &Curve, t0: f64, t1: f64, disp: Vec3) -> (Curve, f64, f64) {
    match c {
        Curve::Line { o, d } => (Curve::Line { o: *o + disp, d: *d }, t0, t1),
        Curve::Circle { frame, r } => {
            let frame = Frame { o: frame.o + disp, ..frame.clone() };
            (Curve::Circle { frame, r: *r }, t0, t1)
        }
        _ => {
            let a = c.eval(t0) + disp;
            let b = c.eval(t1) + disp;
            (Curve::line_through(a, b), 0.0, (b - a).len())
        }
    }
}

/// Rim edge of a cylinder patch moved onto the cylinder offset by `d` (radius r + d): a circle coaxial with the
/// cylinder keeps its frame and gets radius + d; a line parallel to the axis slides radially outwards by d.
fn offset_cylinder_rim(c: &Curve, t0: f64, t1: f64, cf: &Frame, _r: f64, d: f64, tol: f64) -> Option<(Curve, f64, f64)> {
    match c {
        Curve::Circle { frame, r } => {
            let coaxial = frame.z.cross(cf.z).len() < 1e-9 && closest_axis_point(cf, frame.o).dist(frame.o) < tol.max(1e-9) * 10.0;
            coaxial.then(|| (Curve::Circle { frame: frame.clone(), r: r + d }, t0, t1))
        }
        Curve::Line { o, d: dir } => {
            if dir.cross(cf.z).len() > 1e-9 {
                return None;
            }
            let radial = *o - closest_axis_point(cf, *o);
            let l = radial.len();
            if l < 1e-12 {
                return None;
            }
            Some((Curve::Line { o: *o + radial.scale(d / l), d: *dir }, t0, t1))
        }
        _ => None,
    }
}

fn closest_t(c: &Curve, q: Vec3) -> (f64, Vec3) {
    match c {
        Curve::Line { o, d } => {
            let t = (q - *o).dot(*d);
            (t, *o + d.scale(t))
        }
        Curve::Circle { frame, r } => {
            let u = q - frame.o;
            let t = u.y.atan2(u.x);
            let p = frame.o + frame.x.scale(r * t.cos()) + frame.y.scale(r * t.sin());
            (t, p)
        }
        _ => {
            let t = c.closest_param(q);
            (t, c.eval(t))
        }
    }
}

/// Plane constant `n·p = c` of a face (planes exactly, others at their
/// domain middle).
fn plane_const(s: &Surface, n: Vec3) -> f64 {
    match s {
        Surface::Plane { frame } => n.dot(frame.o),
        _ => {
            let ((u0, u1), (v0, v1)) = s.domain();
            // an infinite v-domain (cone) has no midpoint: sample the reference v = 0
            let v = if v0.is_finite() && v1.is_finite() { (v0 + v1) * 0.5 } else { 0.0 };
            n.dot(s.eval((u0 + u1) * 0.5, v))
        }
    }
}

fn miter_disp(nf: Vec3, ng: Vec3, d: f64) -> Option<Vec3> {
    let denom = 1.0 + nf.dot(ng);
    (denom.abs() > 1e-9).then(|| nf.add(ng).scale(d / denom))
}

/// Open rim edges of the sheet: (edge, face) in face order, loop order.
fn sheet_rims(b: &Body, faces: &[FaceId]) -> Vec<(EdgeId, FaceId)> {
    let mut rims = Vec::new();
    for &f in faces {
        for l in &b.face(f).loops {
            if !b.lp(*l).alive {
                continue;
            }
            for &c in &b.lp(*l).coedges {
                let co = b.coedge(c);
                if !co.alive {
                    continue;
                }
                let live = b.edge_coedges(co.edge).iter().filter(|&&cc| b.coedge(cc).alive).count();
                if live == 1 {
                    rims.push((co.edge, f));
                }
            }
        }
    }
    rims
}

/// Thicken an open sheet into a solid.
pub fn thicken(sheet: &Body, feature: &str, thickness: f64, side: Side, tol: f64) -> Result<Body, ThickenError> {
    if !(thickness > 0.0) || !thickness.is_finite() {
        return Err(ThickenError::code("invalidValue"));
    }
    let faces: Vec<FaceId> = sheet.face_ids().collect();
    if faces.is_empty() {
        return Err(ThickenError::code("missingReference"));
    }
    if faces.iter().any(|&f| !matches!(sheet.face(f).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. })) {
        return freeform::thicken_freeform(sheet, feature, thickness, side, tol);
    }
    let normals: Vec<Vec3> = faces.iter().map(|&f| face_normal(sheet, f)).collect();
    let rims = sheet_rims(sheet, &faces);
    if rims.is_empty() {
        return Err(ThickenError::detail("thickenFailed", "the thickened body is not closed"));
    }

    let layers: Vec<(f64, bool)> = match side {
        Side::Front => vec![(0.0, true), (thickness, false)],
        Side::Back => vec![(0.0, true), (-thickness, false)],
        // both: half the thickness each way (probe: t=2 → layers at ±1)
        Side::Both => vec![(-thickness * 0.5, false), (thickness * 0.5, false)],
    };

    let mut out = Body::new();
    let lump = out.add_lump();
    let shell = out.add_shell(lump);

    // build the smaller-d (material-facing, flipped) layer first
    let order: Vec<usize> = {
        let mut o: Vec<usize> = (0..layers.len()).collect();
        o.sort_by(|&a, &b| layers[a].0.total_cmp(&layers[b].0));
        o
    };
    let mut ls: Vec<Layer> = layers
        .iter()
        .map(|&(d, original)| Layer { d, original, edges: Default::default() })
        .collect();
    let flipped_of = |li: usize| layers[li].0 < layers[1 - li].0 || layers.len() == 1;
    for &li in &order {
        build_layer(&mut out, shell, sheet, &faces, &normals, &rims, &layers, &mut ls, li, flipped_of(li), tol)?;
    }

    // walls: one per open rim edge, in (face order, rim order)
    let mut w = 0u32;
    for &(e, f) in &rims {
        build_wall(&mut out, shell, sheet, &faces, &normals, &ls, e, f, &mut w, feature)?;
    }

    weld_vertices(&mut out, tol);
    for fi in out.face_ids().collect::<Vec<_>>() {
        if matches!(out.face(fi).surface, Surface::Plane { .. }) {
            ck_topo::orient::fix_planar_loops(&mut out, fi);
        }
    }
    // a cone's loops are built from the sheet's and the rim walks, with no promise about their turn about the cone's own
    // normal: the face keeps the sense that makes its outer loop run CCW about the outward normal (planes were
    // repaired above by reordering their loops)
    for fi in out.face_ids().collect::<Vec<_>>() {
        if matches!(out.face(fi).surface, Surface::Cone { .. }) && !ck_topo::orient::loops_consistent(&out, fi) {
            let sense = out.face(fi).sense;
            out.faces[fi.idx()].sense = !sense;
        }
    }
    ck_topo::orient::orient_body(&mut out);
    for e in out.edge_ids() {
        if out.edge_coedges(e).len() != 2 {
            if std::env::var("CK_THICKEN_DEBUG").is_ok() {
                eprintln!("DBG open edge {} coedges {:?}", e.idx(), out.edge_coedges(e).iter().map(|&c| (c.idx(), out.coedge_face(c).idx())).collect::<Vec<_>>());
            }
            return Err(ThickenError::detail("thickenFailed", "the thickened body is not closed"));
        }
    }
    let vol = ck_topo::mass::signed_volume(&out, &out.face_ids().collect::<Vec<_>>());
    if std::env::var("CK_THICKEN_DEBUG").is_ok() {
        eprintln!("DBG vol={vol} faces={}", out.num_faces());
    }
    if (!vol.is_finite() || vol <= 0.0) && std::env::var("CK_THICKEN_DEBUG").is_err() {
        return Err(ThickenError::detail("thickenFailed", "the thickened body is not closed"));
    }
    Ok(out)
}

/// Build one layer: edges in sheet edge order, then faces with loops.
#[allow(clippy::too_many_arguments)]
fn build_layer(
    out: &mut Body,
    shell: ShellId,
    sheet: &Body,
    faces: &[FaceId],
    normals: &[Vec3],
    rims: &[(EdgeId, FaceId)],
    layers: &[(f64, bool)],
    ls: &mut [Layer],
    li: usize,
    flipped: bool,
    tol: f64,
) -> Result<(), ThickenError> {
    let d = ls[li].d;

    // edge images (sheet edge -> (curve, t0, t1))
    let mut images: std::collections::HashMap<EdgeId, (Curve, f64, f64)> = Default::default();
    if ls[li].original {
        for e in sheet.edge_ids().collect::<Vec<_>>() {
            if !sheet.edge(e).alive {
                continue;
            }
            let se = sheet.edge(e);
            images.insert(e, (se.curve.clone(), se.t0, se.t1));
        }
    } else {
        // shared edges: miter segments between the two offset planes
        let mut shared_miter: std::collections::HashMap<EdgeId, (Vec3, Vec3)> = Default::default();
        for e in sheet.edge_ids().collect::<Vec<_>>() {
            if !sheet.edge(e).alive {
                continue;
            }
            let fs = sheet.edge_faces(e);
            if fs.len() != 2 {
                continue;
            }
            let (Some(fi), Some(gi)) = (faces.iter().position(|&x| x == fs[0]), faces.iter().position(|&x| x == fs[1])) else {
                return Err(ThickenError::code("thickenFailed"));
            };
            let se = sheet.edge(e);
            let pm = se.curve.eval(0.5 * (se.t0 + se.t1));
            let (nf, ng) = (normal_at(sheet, fs[0], pm, normals[fi]), normal_at(sheet, fs[1], pm, normals[gi]));
            let Some(disp) = miter_disp(nf, ng, d) else {
                return Err(ThickenError::code("thickenReach"));
            };
            let a = se.curve.eval(se.t0) + disp;
            let b = se.curve.eval(se.t1) + disp;
            shared_miter.insert(e, (a, b));
            images.insert(e, (Curve::line_through(a, b), 0.0, (b - a).len()));
        }
        // open edges: translated, extended to the miter lines at corners
        // that touch a shared edge. Closed rims of quadric faces offset to
        // a new closed curve (cylinder: radius r+d) instead of translating.
        for &(e, f) in rims {
            if images.contains_key(&e) {
                continue;
            }
            let se = sheet.edge(e);
            let closed_edge = se.curve.eval(se.t0).dist(se.curve.eval(se.t1)) < 1e-9;
            if closed_edge {
                match (&se.curve, &sheet.face(f).surface) {
                    (Curve::Circle { frame, r }, Surface::Cylinder { .. }) => {
                        images.insert(e, (Curve::Circle { frame: frame.clone(), r: r + d }, se.t0, se.t1));
                        continue;
                    }
                    _ => {}
                }
            }
            extend_open_edge(sheet, f, e, normals[faces.iter().position(|&x| x == f).unwrap()], d, faces, normals, &shared_miter, &mut images, tol)?;
        }
    }

    // edges in sheet edge order
    for e in sheet.edge_ids().collect::<Vec<_>>() {
        if !sheet.edge(e).alive {
            continue;
        }
        let Some(&(ref curve, t0, t1)) = images.get(&e) else { continue };
        let v0 = out.add_vertex(curve.eval(t0));
        let v1 = out.add_vertex(curve.eval(t1));
        let ne = out.add_edge(curve.clone(), t0, t1, v0, v1);
        ls[li].edges.insert(e, ne);
    }
    // merge duplicate corner vertices before chaining loops
    weld_vertices(out, tol);

    // faces
    for (fi, &f) in faces.iter().enumerate() {
        let surface = if ls[li].original {
            sheet.face(f).surface.clone()
        } else {
            match offset_surface(&sheet.face(f).surface, surface_offset(sheet, f, d)) {
                Some(s) => s,
                None if !matches!(sheet.face(f).surface, Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. }) => {
                    // not a curvature failure: thicken only offsets analytic sheets, a free-form
                    // (B-spline) sheet has no offset here at any thickness
                    return Err(ThickenError::detail(
                        "offsetUnsupported",
                        format!(
                            "{}: thicken does not offset free-form (B-spline) sheets yet, at any thickness; close the surface into a solid (cap the ends, or loft closed sections) and use shell",
                            sheet.face_id_str(f)
                        ),
                    ));
                }
                None => {
                    return Err(ThickenError::detail(
                        "offsetTooLarge",
                        "the offset passes the surface's centre of curvature",
                    ))
                }
            }
        };
        // periodic faces with two closed-ring boundaries get a seam rail
        // (like ck-topo revolves): one loop [ring0, seam, ring1, seam] with
        // uv pcurves, so mass/orientation integrate the closed uv strip.
        let loops: Vec<ck_topo::body::LoopId> =
            sheet.face(f).loops.iter().copied().filter(|&l| sheet.lp(l).alive).collect();
        let seamable = matches!(surface, Surface::Cylinder { .. })
            && loops.len() == 2
            && loops.iter().all(|&l| {
                let cs: Vec<_> = sheet.lp(l).coedges.iter().filter(|&&c| sheet.coedge(c).alive).collect();
                cs.len() == 1 && {
                    let e = &out.edges[ls[li].edges[&sheet.coedge(*cs[0]).edge].idx()];
                    e.curve.eval(e.t0).dist(e.curve.eval(e.t1)) < 1e-9
                }
            });
        let mut nf: Option<FaceId> = None;
        if seamable {
            let mut name = sheet.face(f).name.clone();
            name.region = Some(layer_suffix(layers, li, normals[fi]));
            let nfi = out.add_face(shell, surface.clone(), sheet.face(f).sense != flipped, name);
            let Surface::Cylinder { frame, .. } = &surface else { unreachable!() };
            let ring_edges: Vec<EdgeId> = loops
                .iter()
                .map(|&l| {
                    let c = sheet.lp(l).coedges.iter().find(|&&c| sheet.coedge(c).alive).unwrap();
                    ls[li].edges[&sheet.coedge(*c).edge]
                })
                .collect();
            let tau = std::f64::consts::TAU;
            let vs: Vec<f64> = ring_edges
                .iter()
                .map(|&ne| (out.edges[ne.idx()].curve.eval(out.edges[ne.idx()].t0) - frame.o).dot(frame.z))
                .collect();
            let rim_pc = |v: f64| ck_geom::curve2::Curve2::Line { o: ck_geom::Vec2::new(0.0, v), d: ck_geom::Vec2::new(1.0, 0.0) };
            // seam rail between the rings' start vertices
            let p0 = out.edges[ring_edges[0].idx()].curve.eval(out.edges[ring_edges[0].idx()].t0);
            let p1 = out.edges[ring_edges[1].idx()].curve.eval(out.edges[ring_edges[1].idx()].t0);
            let (Some(v0), Some(v1)) = (
                out.vertex_ids().find(|&v| out.vertex(v).p.dist(p0) <= 1e-9),
                out.vertex_ids().find(|&v| out.vertex(v).p.dist(p1) <= 1e-9),
            ) else {
                return Err(ThickenError::code("thickenFailed"));
            };
            let seam = out.add_edge(Curve::line_through(p0, p1), 0.0, (p1 - p0).len(), v0, v1);
            let up_pc = ck_geom::curve2::Curve2::Line { o: ck_geom::Vec2::new(tau, vs[0]), d: ck_geom::Vec2::new(0.0, 1.0) };
            let down_pc = ck_geom::curve2::Curve2::Line { o: ck_geom::Vec2::new(0.0, vs[0]), d: ck_geom::Vec2::new(0.0, 1.0) };
            let nloop = out.add_loop(nfi);
            let r = flipped;
            out.add_coedge(nloop, ring_edges[0], r, Some(rim_pc(vs[0])));
            out.add_coedge(nloop, seam, r, Some(up_pc));
            out.add_coedge(nloop, ring_edges[1], !r, Some(rim_pc(vs[1])));
            out.add_coedge(nloop, seam, !r, Some(down_pc));
            continue;
        }
        for &lid in &loops {
            if nf.is_none() {
                let mut name = sheet.face(f).name.clone();
                name.region = Some(layer_suffix(layers, li, normals[fi]));
                nf = Some(out.add_face(shell, surface.clone(), sheet.face(f).sense != flipped, name));
            }
            let nloop = out.add_loop(nf.unwrap());
            if ls[li].original {
                // a flipped layer runs the loop backwards: reverse the coedge
                // order as well as each direction so the loop still chains
                let cl = &sheet.lp(lid).coedges;
                let ordered: Vec<_> = if flipped { cl.iter().rev().copied().collect() } else { cl.to_vec() };
                for cid in ordered {
                    let co = sheet.coedge(cid);
                    if !co.alive {
                        continue;
                    }
                    let ne = ls[li].edges[&co.edge];
                    out.add_coedge(nloop, ne, co.rev != flipped, co.pcurve.clone());
                }
            } else {
                // extensions/trims change the edges' extents: re-chain the
                // loop head-to-tail from the image edges. Loops of closed
                // rings (periodic faces) chain trivially.
                let all_closed = sheet
                    .lp(lid)
                    .coedges
                    .iter()
                    .filter(|&&c| sheet.coedge(c).alive)
                    .all(|&c| {
                        let e = &out.edges[ls[li].edges[&sheet.coedge(c).edge].idx()];
                        e.curve.eval(e.t0).dist(e.curve.eval(e.t1)) < 1e-9
                    });
                if all_closed {
                    let loop_i = out.face(nf.unwrap()).loops.len() - 1;
                    for &c in &sheet.lp(lid).coedges {
                        if sheet.coedge(c).alive {
                            out.add_coedge(nloop, ls[li].edges[&sheet.coedge(c).edge], (loop_i % 2 == 1) != flipped, None);
                        }
                    }
                    continue;
                }
                let mut used: Vec<(EdgeId, bool)> = Vec::new();
                let mut remaining: Vec<EdgeId> = sheet
                    .lp(lid)
                    .coedges
                    .iter()
                    .filter(|&&c| sheet.coedge(c).alive)
                    .map(|&c| ls[li].edges[&sheet.coedge(c).edge])
                    .collect();
                let first = remaining.swap_remove(0);
                used.push((first, false));
                // the first edge may need to run reversed for the chain
                if !remaining.iter().any(|&x| out.edge(x).v0 == out.edge(used[0].0).v1 || out.edge(x).v1 == out.edge(used[0].0).v1) {
                    used[0].1 = true;
                }
                while !remaining.is_empty() {
                    let tail = edge_end(out, *used.last().unwrap());
                    let Some(nxt) = remaining.iter().position(|&x| out.edge(x).v0 == tail || out.edge(x).v1 == tail) else {
                        return Err(ThickenError::code("thickenFailed"));
                    };
                    let ne = remaining.swap_remove(nxt);
                    let fwd = out.edge(ne).v0 == tail;
                    used.push((ne, !fwd));
                }
                for (ne, rev) in used {
                    out.add_coedge(nloop, ne, rev, None);
                }
            }
        }
    }

    Ok(())
}

/// Walk-end vertex of an edge traversed `rev` (false = v0→v1).
fn edge_end(b: &Body, (e, rev): (EdgeId, bool)) -> VertexId {
    if rev {
        b.edge(e).v0
    } else {
        b.edge(e).v1
    }
}

/// Image of one open rim edge: translated by `d·nf`, extended at each end
/// that touches a shared edge, to that corner's miter line.
#[allow(clippy::too_many_arguments)]
fn extend_open_edge(
    sheet: &Body,
    f: FaceId,
    e: EdgeId,
    nf: Vec3,
    d: f64,
    faces: &[FaceId],
    normals: &[Vec3],
    shared_miter: &std::collections::HashMap<EdgeId, (Vec3, Vec3)>,
    images: &mut std::collections::HashMap<EdgeId, (Curve, f64, f64)>,
    tol: f64,
) -> Result<(), ThickenError> {
    let se = sheet.edge(e);
    // A stand-alone cylinder patch (no neighbour sheet face): its rim offsets radially, so the offset
    // face's boundary lies on the offset cylinder. A plain translation by the mid-patch normal leaves
    // arcs and end lines off the surface (the display mesh then fills with sliver triangles).
    if let Surface::Cylinder { frame, r } = &sheet.face(f).surface {
        let stand_alone = sheet.face(f).loops.iter().filter(|&&l| sheet.lp(l).alive).all(|&l| {
            sheet.lp(l).coedges.iter().all(|&c| !sheet.coedge(c).alive || sheet.edge_faces(sheet.coedge(c).edge).len() == 1)
        });
        if stand_alone {
            if let Some(img) = offset_cylinder_rim(&se.curve, se.t0, se.t1, frame, *r, d, tol) {
                images.insert(e, img);
                return Ok(());
            }
        }
    }
    // A rim circle of a cone patch moves onto the offset cone: the centre slides along the axis and the radius
    // changes by the normal's components (the patch's other boundary edges are generators of the same cone, which
    // end exactly on the offset circles, so there is no corner to extend).
    if let (Surface::Cone { frame, .. }, Curve::Circle { frame: cf, r: rc }) = (&sheet.face(f).surface, &se.curve) {
        let coaxial = cf.z.cross(frame.z).len() < 1e-9 && (cf.o - frame.o).cross(frame.z).len() < 1e-7 * (1.0 + rc);
        let neighbours_same = [se.t0, se.t1].iter().all(|&t| {
            let p = se.curve.eval(t);
            sheet.face(f).loops.iter().filter(|&&l| sheet.lp(l).alive).all(|&l| {
                sheet.lp(l).coedges.iter().all(|&c| {
                    let co = sheet.coedge(c);
                    if !co.alive || co.edge == e {
                        return true;
                    }
                    let e2 = sheet.edge(co.edge);
                    let near = e2.curve.eval(e2.t0).dist(p) <= tol || e2.curve.eval(e2.t1).dist(p) <= tol;
                    !near || sheet.edge_faces(co.edge).iter().all(|&g| g == f || same_cone(&sheet.face(f).surface, &sheet.face(g).surface))
                })
            })
        });
        if coaxial && neighbours_same {
            let p = se.curve.eval(se.t0);
            let n = normal_at(sheet, f, p, nf);
            let rad = radial_of(frame, p);
            let r2 = rc + d * n.dot(rad);
            if !(r2 > 1e-9) {
                return Err(ThickenError::detail("offsetTooLarge", "the offset passes the surface's centre of curvature"));
            }
            let nf_ = Frame { o: cf.o + frame.z.scale(d * n.dot(frame.z)), ..cf.clone() };
            images.insert(e, (Curve::Circle { frame: nf_, r: r2 }, se.t0, se.t1));
            return Ok(());
        }
    }
    let (curve, mut t0, mut t1) = translate_curve(&se.curve, se.t0, se.t1, nf.scale(d));
    for end in [0usize, 1] {
        let te = if end == 0 { se.t0 } else { se.t1 };
        let p = se.curve.eval(te);
        let u = {
            let q = se.curve.eval(if end == 0 { se.t1 } else { se.t0 });
            let u = q - p;
            let l = u.len();
            (l > 1e-12).then(|| u.scale(1.0 / l))
        };
        let Some(u) = u else { continue };
        // the face's shared edge at this corner (the neighbour)
        let mut other_shared: Option<EdgeId> = None;
        'outer: for &lid in &sheet.face(f).loops {
            if !sheet.lp(lid).alive {
                continue;
            }
            for &cid in &sheet.lp(lid).coedges {
                let co = sheet.coedge(cid);
                if !co.alive || co.edge == e {
                    continue;
                }
                let e2 = sheet.edge(co.edge);
                let near = e2.curve.eval(e2.t0).dist(p) <= tol || e2.curve.eval(e2.t1).dist(p) <= tol;
                if near && sheet.edge_faces(co.edge).len() == 2 {
                    other_shared = Some(co.edge);
                    break 'outer;
                }
            }
        }
        let Some(se2) = other_shared else { continue };
        let g = *sheet.edge_faces(se2).iter().find(|&&g| g != f).unwrap();
        let gi = faces.iter().position(|&x| x == g).unwrap();
        let ng = normals[gi];
        let denom = u.dot(ng);
        if denom.abs() < 1e-9 {
            return Err(ThickenError::code("thickenReach"));
        }
        let s = (plane_const(&sheet.face(g).surface, ng) + d - ng.dot(p)) / denom;
        let q = p + nf.scale(d) + u.scale(s);
        // q must lie within the neighbour pair's miter segment
        let (ma, mb) = shared_miter[&se2];
        let along = (mb - ma).normalize();
        let sq = (q - ma).dot(along);
        let len = (mb - ma).len();
        if sq < -tol || sq > len + tol {
            return Err(ThickenError::detail("thickenReach", "the offset reaches past a face"));
        }
        let (tq, _) = closest_t(&curve, q);
        if end == 0 {
            t0 = tq;
        } else {
            t1 = tq;
        }
    }
    images.insert(e, (curve, t0, t1));
    Ok(())
}

/// Merge coincident vertices (miter corners) and kill leftovers.
fn weld_vertices(b: &mut Body, tol: f64) {
    let ids: Vec<VertexId> = b.vertex_ids().collect();
    let n = ids.len();
    let mut parent: Vec<usize> = (0..n).collect();
    for i in 0..n {
        for j in (i + 1)..n {
            if b.vertex(ids[i]).p.dist(b.vertex(ids[j]).p) <= tol {
                let (mut a, mut c) = (i, j);
                while parent[a] != a {
                    a = parent[a];
                }
                while parent[c] != c {
                    c = parent[c];
                }
                if a != c {
                    parent[c] = a;
                }
            }
        }
    }
    let root = |mut x: usize, p: &[usize]| {
        while p[x] != x {
            x = p[x];
        }
        x
    };
    let roots: Vec<usize> = (0..n).map(|i| root(i, &parent)).collect();
    for i in 0..n {
        if roots[i] == i {
            continue;
        }
        let v = ids[i];
        let target = ids[roots[i]];
        for e in b.edge_ids().collect::<Vec<_>>() {
            if b.edge(e).v0 == v {
                b.edge_mut(e).v0 = target;
            }
            if b.edge(e).v1 == v {
                b.edge_mut(e).v1 = target;
            }
        }
        b.vertex_mut(v).alive = false;
    }
}

/// One wall: strip/annulus between the two layers' images of the rim edge.
/// The loop walks the first layer's edge (CCW about the outward normal
/// `u × nf`), then the connector, the second layer's edge reversed, the
/// connector back — this pins the connector creation order to the oracle.
#[allow(clippy::too_many_arguments)]
fn build_wall(
    out: &mut Body,
    shell: ShellId,
    sheet: &Body,
    faces: &[FaceId],
    normals: &[Vec3],
    ls: &[Layer],
    e: EdgeId,
    f: FaceId,
    w: &mut u32,
    feature: &str,
) -> Result<(), ThickenError> {
    let img: Vec<EdgeId> = ls.iter().map(|l| *l.edges.get(&e).unwrap()).collect();
    let (ca, cb, ta0, ta1, tb0, tb1) = {
        let (ea, eb) = (&out.edges[img[0].idx()], &out.edges[img[1].idx()]);
        (ea.curve.clone(), eb.curve.clone(), ea.t0, ea.t1, eb.t0, eb.t1)
    };
    let p00 = ca.eval(ta0);
    let p01 = ca.eval(ta1);
    let p10 = cb.eval(tb0);
    let p11 = cb.eval(tb1);
    let closed = p00.dist(p01) < 1e-9 && p10.dist(p11) < 1e-9;

    let name =
        FaceName { feature: feature.into(), role: Role::Other(format!("W{}", w)), region: None, suffix: String::new() };
    *w += 1;

    if closed {
        // annulus wall: plane through both circles, frame aligned with the
        // circles', exact uv circle pcurves (the projection fallback's
        // 48-gon loses ~0.3%)
        let (fa, ra, rb) = match (&ca, &cb) {
            (Curve::Circle { frame: fa, r: ra }, Curve::Circle { r: rb, .. }) => (fa, *ra, *rb),
            _ => return Err(ThickenError::code("thickenFailed")),
        };
        let frame = Frame { o: fa.o, x: fa.x, y: fa.y, z: fa.z };
        let surface = Surface::Plane { frame };
        let wf = out.add_face(shell, surface, true, name);
        let (outer, inner, ro, ri_) = if ra >= rb { (img[0], img[1], ra, rb) } else { (img[1], img[0], rb, ra) };
        let pc = |r: f64| {
            ck_geom::curve2::Curve2::Circle {
                c: ck_geom::Vec2::ZERO,
                x: ck_geom::Vec2::X,
                r,
                ccw: true,
            }
        };
        let l0 = out.add_loop(wf);
        out.add_coedge(l0, outer, false, Some(pc(ro)));
        let l1 = out.add_loop(wf);
        out.add_coedge(l1, inner, true, Some(pc(ri_)));
        return Ok(());
    }

    let n1 = (p01 - p00).cross(p10 - p00);
    let n2 = (p01 - p00).cross(p11 - p00);
    // two coplanar circle arcs (the rims of a cylinder patch): the wall is the planar ring sector they bound.
    // Their end points can be collinear (half circles), so the plane comes from the circles, not the corners.
    let arc_plane = match (&ca, &cb) {
        (Curve::Circle { frame: fa, .. }, Curve::Circle { frame: fb, .. })
            if fa.z.cross(fb.z).len() < 1e-9 && (fb.o - fa.o).dot(fa.z).abs() < 1e-9 =>
        {
            Some(Surface::Plane { frame: Frame { o: fa.o, x: fa.x, y: fa.y, z: fa.z } })
        }
        _ => None,
    };
    // two coaxial arcs in different planes (the rims of a cone patch): the wall is the cone through both
    let arc_cone = match (&ca, &cb) {
        (Curve::Circle { frame: fa, r: ra }, Curve::Circle { frame: fb, r: rb }) if arc_plane.is_none() && fa.z.cross(fb.z).len() < 1e-9 && (fb.o - fa.o).cross(fa.z).len() < 1e-7 * (1.0 + ra) => {
            let vb = (fb.o - fa.o).dot(fa.z);
            // the cone opens along +z: a ring that grows against the circle's axis uses the flipped frame
            (vb.abs() > 1e-9).then(|| {
                let semi = ((rb - ra) / vb).atan();
                if semi >= 0.0 {
                    Surface::Cone { frame: Frame { o: fa.o, x: fa.x, y: fa.y, z: fa.z }, r: *ra, semi_angle: semi }
                } else {
                    Surface::Cone { frame: Frame { o: fa.o, x: fa.x, y: fa.y.scale(-1.0), z: fa.z.scale(-1.0) }, r: *ra, semi_angle: -semi }
                }
            })
        }
        _ => None,
    };
    let surface = if let Some(pl) = arc_plane {
        pl
    } else if let Some(c) = arc_cone {
        c
    } else if n1.len2() > 1e-18 && n1.normalize().dist(n2.normalize()) < 1e-6 {
        Surface::plane(p00, n1.normalize())
    } else {
        Surface::BSpline(Box::new(BSplineSurface::bilinear(p00, p01, p10, p11)))
    };
    let wf = out.add_face(shell, surface, true, name);

    let fi = faces.iter().position(|&x| x == f).unwrap();
    let nf = normals[fi];
    let se = sheet.edge(e);
    let u = (se.curve.eval(se.t1) - se.curve.eval(se.t0)).normalize();
    let outward = u.cross(nf);
    // B endpoint nearest the A walk-end
    let near = |p: Vec3, q0: Vec3, q1: Vec3| if (q1 - p).len2() < (q0 - p).len2() { q1 } else { q0 };
    let a_fwd = {
        let b1 = near(p01, p10, p11);
        (p01 - p00).cross(b1 - p00).dot(outward) >= 0.0
    };
    let (rev0, rev1) = if a_fwd { (false, true) } else { (true, false) };
    let wl = out.add_loop(wf);
    out.add_coedge(wl, img[0], rev0, None);
    // connector at the A walk end (curve stored A->B, walked A->B)
    let (a_end, b_near) = if a_fwd { (p01, near(p01, p10, p11)) } else { (p00, near(p00, p10, p11)) };
    connector(out, wl, a_end, b_near, false);
    // second layer's edge, opposite direction
    out.add_coedge(wl, img[1], rev1, None);
    // connector back at the A walk start (curve stored A->B, walked B->A)
    let (a_start, b_start) = if a_fwd { (p00, near(p00, p10, p11)) } else { (p01, near(p01, p10, p11)) };
    connector(out, wl, a_start, b_start, true);
    Ok(())
}

/// Connector edge between the layers at a rim corner. The curve always
/// runs a -> b (the A-layer vertex to the B-layer vertex, oracle-pinned);
/// `rev` = the wall's loop traverses it backwards.
fn connector(out: &mut Body, l: LoopId, a: Vec3, b: Vec3, rev: bool) {
    if a.dist(b) <= 1e-12 {
        return;
    }
    let find = |out: &Body, p: Vec3| out.vertex_ids().find(|&v| out.vertex(v).p.dist(p) <= 1e-9);
    let (va, vb) = match (find(out, a), find(out, b)) {
        (Some(x), Some(y)) => (x, y),
        _ => return,
    };
    // reuse the connector edge when an adjacent wall already created it
    let existing: Option<EdgeId> = {
        let ids: Vec<EdgeId> = out.edge_ids().collect();
        ids.into_iter().find(|&x| {
            let ed = out.edge(x);
            (ed.v0 == va && ed.v1 == vb) || (ed.v0 == vb && ed.v1 == va)
        })
    };
    let ce = match existing {
        Some(ce) => ce,
        None => out.add_edge(Curve::line_through(a, b), 0.0, (b - a).len(), va, vb),
    };
    out.add_coedge(l, ce, rev, None);
}

mod freeform;

#[cfg(test)]
mod tests {
    use super::*;
    use ck_topo::body::Role;

    fn name(feat: &str, role: &str) -> FaceName {
        FaceName { feature: feat.into(), role: Role::Other(role.into()), region: None, suffix: String::new() }
    }

    /// One planar sheet face: rectangle o..o+dx+dy at natural normal
    /// dx×dy, loops CCW about the normal, all 4 edges open rims.
    fn sheet_square(feat: &str, o: Vec3, dx: Vec3, dy: Vec3) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let n = dx.cross(dy).normalize();
        let f = b.add_face(shell, Surface::plane(o, n), true, name(feat, "P"));
        let p = |i: usize| o + dx.scale([0.0, 1.0, 1.0, 0.0][i]) + dy.scale([0.0, 0.0, 1.0, 1.0][i]);
        let lp = b.add_loop(f);
        let vs: Vec<_> = (0..4).map(|i| b.add_vertex(p(i))).collect();
        for i in 0..4 {
            let a = p(i);
            let c = p((i + 1) % 4);
            let e = b.add_edge(Curve::line_through(a, c), 0.0, (c - a).len(), vs[i], vs[(i + 1) % 4]);
            b.add_coedge(lp, e, false, None);
        }
        b
    }

    /// Closed square tube: 4 planar sides (entity order like the sketch:
    /// e1 (0,0)→(10,0) at y=0 normal −y, e2 (10,0)→(10,10) at x=10 normal
    /// +x, e3 (10,10)→(0,10) at y=10 normal +y, e4 (0,10)→(0,0) at x=0
    /// normal −x), open at z=0 and z=10.
    pub(crate) fn tube_square(feat: &str, z0: f64, z1: f64) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let corners = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let mut rails = Vec::new();
        let mut vbottom = Vec::new();
        let mut vtop = Vec::new();
        for &(x, y) in &corners {
            vbottom.push(b.add_vertex(Vec3::new(x, y, z0)));
            vtop.push(b.add_vertex(Vec3::new(x, y, z1)));
        }
        for i in 0..4 {
            let e = b.add_edge(
                Curve::line_through(Vec3::new(corners[i].0, corners[i].1, z0), Vec3::new(corners[i].0, corners[i].1, z1)),
                0.0,
                z1 - z0,
                vbottom[i],
                vtop[i],
            );
            rails.push(e);
        }
        // sides: entity k runs corner k -> k+1 (e1: (5,0)->(5,10)? no:
        // sketch e1 travels +x at y=0). Side k loops: rail[k], rim z0
        // (corner k -> k+1), rail[k+1], rim z1 (k+1 -> k).
        for k in 0..4 {
            let (x0, y0) = corners[k];
            let (x1, y1) = corners[(k + 1) % 4];
            // natural normal: travel × +z
            let travel = Vec3::new(x1 - x0, y1 - y0, 0.0).normalize();
            let nn = travel.cross(Vec3::new(0.0, 0.0, 1.0));
            let f = b.add_face(shell, Surface::plane(Vec3::new(x0, y0, z0), nn), true, FaceName::side(feat, &format!("e{}", k + 1)));
            let lp = b.add_loop(f);
            let r0 = b.add_vertex(Vec3::new(x0, y0, z0));
            let r1 = b.add_vertex(Vec3::new(x1, y1, z0));
            let r2 = b.add_vertex(Vec3::new(x1, y1, z1));
            let r3 = b.add_vertex(Vec3::new(x0, y0, z1));
            let rim0 = b.add_edge(Curve::line_through(Vec3::new(x0, y0, z0), Vec3::new(x1, y1, z0)), 0.0, (Vec3::new(x1 - x0, y1 - y0, 0.0)).len(), r0, r1);
            let rim1 = b.add_edge(Curve::line_through(Vec3::new(x1, y1, z1), Vec3::new(x0, y0, z1)), 0.0, (Vec3::new(x0 - x1, y1 - y0, 0.0)).len(), r2, r3);
            b.add_coedge(lp, rim0, false, None);
            b.add_coedge(lp, rails[(k + 1) % 4], false, None);
            b.add_coedge(lp, rim1, false, None);
            b.add_coedge(lp, rails[k], true, None);
        }
        b
    }

    /// Closed cylinder tube r=5, z 0..10, natural normal radially outward,
    /// two closed rim circles (no seam).
    pub(crate) fn tube_cyl(feat: &str, r: f64, z0: f64, z1: f64) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let c = Vec3::new(0.0, 0.0, z0);
        let f = b.add_face(
            shell,
            Surface::Cylinder { frame: Frame { o: c, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r },
            true,
            FaceName::side(feat, "e1"),
        );
        let v0 = b.add_vertex(c + Vec3::new(r, 0.0, 0.0));
        let v1 = b.add_vertex(Vec3::new(0.0, 0.0, z1) + Vec3::new(r, 0.0, 0.0));
        let e0 = b.add_edge(Curve::circle(c, Vec3::Z, r), 0.0, std::f64::consts::TAU, v0, v0);
        let e1 = b.add_edge(Curve::circle(Vec3::new(0.0, 0.0, z1), Vec3::Z, r), 0.0, std::f64::consts::TAU, v1, v1);
        let l0 = b.add_loop(f);
        b.add_coedge(l0, e0, false, None);
        let l1 = b.add_loop(f);
        b.add_coedge(l1, e1, true, None);
        b
    }

    fn keys(b: &Body) -> Vec<String> {
        b.edge_ids().map(|e| b.face_id_str_pair(e)).collect()
    }

    trait KeyPair {
        fn face_id_str_pair(&self, e: ck_topo::body::EdgeId) -> String;
    }
    impl KeyPair for Body {
        fn face_id_str_pair(&self, e: ck_topo::body::EdgeId) -> String {
            let fs = self.edge_faces(e);
            let mut ids: Vec<String> = fs.iter().map(|&f| self.face(f).name.id()).collect();
            ids.sort();
            ids.join("|")
        }
    }

    fn endpoints(b: &Body, e: ck_topo::body::EdgeId) -> (Vec3, Vec3) {
        let ed = b.edge(e);
        (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1))
    }

    #[test]
    fn thicken_sheet_front_oracle() {
        // oracle probe: surfacePlanar square 10×10 z=0 + thicken t=2 front
        let sheet = sheet_square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let b = thicken(&sheet, "F3", 2.0, Side::Front, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(
            fids,
            ["F2.P~1", "F2.P~2", "F3.W0", "F3.W1", "F3.W2", "F3.W3"]
                .iter()
                .map(|s| s.to_string())
                .collect::<Vec<_>>()
        );
        // edges in order
        let eids: Vec<EdgeId> = b.edge_ids().collect();
        assert_eq!(eids.len(), 12);
        let want: &[(&str, &str, [f64; 3], [f64; 3])] = &[
            ("F2.P~1|F3.W0", "", [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
            ("F2.P~1|F3.W1", "", [10.0, 0.0, 0.0], [10.0, 10.0, 0.0]),
            ("F2.P~1|F3.W2", "", [10.0, 10.0, 0.0], [0.0, 10.0, 0.0]),
            ("F2.P~1|F3.W3", "", [0.0, 10.0, 0.0], [0.0, 0.0, 0.0]),
            ("F2.P~2|F3.W0", "", [0.0, 0.0, 2.0], [10.0, 0.0, 2.0]),
            ("F2.P~2|F3.W1", "", [10.0, 0.0, 2.0], [10.0, 10.0, 2.0]),
            ("F2.P~2|F3.W2", "", [10.0, 10.0, 2.0], [0.0, 10.0, 2.0]),
            ("F2.P~2|F3.W3", "", [0.0, 10.0, 2.0], [0.0, 0.0, 2.0]),
            ("F3.W0|F3.W1", "", [10.0, 0.0, 0.0], [10.0, 0.0, 2.0]),
            ("F3.W0|F3.W3", "", [0.0, 0.0, 0.0], [0.0, 0.0, 2.0]),
            ("F3.W1|F3.W2", "", [10.0, 10.0, 0.0], [10.0, 10.0, 2.0]),
            ("F3.W2|F3.W3", "", [0.0, 10.0, 0.0], [0.0, 10.0, 2.0]),
        ];
        for (i, (key, _, a, c)) in want.iter().enumerate() {
            assert_eq!(&keys(&b)[i], key, "edge {i} got {:?}", keys(&b));
            let (p, q) = endpoints(&b, eids[i]);
            assert!(p.dist(Vec3::new(a[0], a[1], a[2])) < 1e-9 && q.dist(Vec3::new(c[0], c[1], c[2])) < 1e-9, "edge {i} got {p:?} {q:?}");
        }
        let mp = ck_topo::mass::mass_props(&b);
        assert!((mp.volume - 200.0).abs() < 1e-7, "volume {}", mp.volume);
        assert!((mp.area - 280.0).abs() < 1e-7, "area {}", mp.area);
    }

    #[test]
    fn thicken_sheet_back_oracle() {
        let sheet = sheet_square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let b = thicken(&sheet, "F3", 2.0, Side::Back, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids[0], "F2.P~1");
        assert_eq!(fids[1], "F2.P~2");
        // ~1 = offset copy at z=-2 (lower), ~2 = original
        let layer_at = |id: &str| -> f64 {
            let f = b.face_ids().find(|&f| b.face(f).name.id() == id).unwrap();
            let e = b.face_edges(f)[0];
            endpoints(&b, e).0.z
        };
        assert!((layer_at("F2.P~1") + 2.0).abs() < 1e-9);
        assert!(layer_at("F2.P~2").abs() < 1e-9);
        let mp = ck_topo::mass::mass_props(&b);
        assert!((mp.volume - 200.0).abs() < 1e-7, "{}", mp.volume);
    }

    #[test]
    fn thicken_sheet_both_oracle() {
        let sheet = sheet_square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        let b = thicken(&sheet, "F3", 2.0, Side::Both, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids.len(), 6);
        assert_eq!(fids[0], "F2.P~1");
        assert_eq!(fids[1], "F2.P~2");
        let layer_at = |id: &str| -> f64 {
            let f = b.face_ids().find(|&f| b.face(f).name.id() == id).unwrap();
            let e = b.face_edges(f)[0];
            endpoints(&b, e).0.z
        };
        assert!((layer_at("F2.P~1") + 1.0).abs() < 1e-9, "~1 at -1");
        assert!((layer_at("F2.P~2") - 1.0).abs() < 1e-9, "~2 at +1");
        let mp = ck_topo::mass::mass_props(&b);
        assert!((mp.volume - 200.0).abs() < 1e-7, "{}", mp.volume);
    }

    #[test]
    fn thicken_tube_front_oracle() {
        // oracle probe: surfaceExtrude square tube (sides 10×10, z 0..10) +
        // thicken t=2 side=front: 16 faces, 32 edges, volume 960
        let sheet = tube_square("F2", 0.0, 10.0);
        let b = thicken(&sheet, "F3", 2.0, Side::Front, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(
            fids,
            [
                "F2.S.e1~2", "F2.S.e2~1", "F2.S.e3~1", "F2.S.e4~2", "F2.S.e1~1", "F2.S.e2~2", "F2.S.e3~2", "F2.S.e4~1",
                "F3.W0", "F3.W1", "F3.W2", "F3.W3", "F3.W4", "F3.W5", "F3.W6", "F3.W7"
            ]
            .iter()
            .map(|s| s.to_string())
            .collect::<Vec<_>>()
        );
        let eids: Vec<EdgeId> = b.edge_ids().collect();
        assert_eq!(eids.len(), 32);
        let want_keys = [
            "F2.S.e1~2|F2.S.e4~2", "F2.S.e1~2|F2.S.e2~1", "F2.S.e2~1|F2.S.e3~1", "F2.S.e3~1|F2.S.e4~2",
            "F2.S.e1~2|F3.W0", "F2.S.e1~2|F3.W1", "F2.S.e2~1|F3.W2", "F2.S.e2~1|F3.W3", "F2.S.e3~1|F3.W4",
            "F2.S.e3~1|F3.W5", "F2.S.e4~2|F3.W6", "F2.S.e4~2|F3.W7", "F2.S.e1~1|F2.S.e4~1", "F2.S.e1~1|F2.S.e2~2",
            "F2.S.e2~2|F2.S.e3~2", "F2.S.e3~2|F2.S.e4~1", "F2.S.e1~1|F3.W0", "F2.S.e1~1|F3.W1", "F2.S.e2~2|F3.W2",
            "F2.S.e2~2|F3.W3", "F2.S.e3~2|F3.W4", "F2.S.e3~2|F3.W5", "F2.S.e4~1|F3.W6", "F2.S.e4~1|F3.W7",
            "F3.W0|F3.W2", "F3.W0|F3.W6", "F3.W1|F3.W7", "F3.W1|F3.W3", "F3.W2|F3.W4", "F3.W3|F3.W5", "F3.W4|F3.W6",
            "F3.W5|F3.W7",
        ];
        for (i, k) in want_keys.iter().enumerate() {
            assert_eq!(&keys(&b)[i], k, "edge {i}");
        }
        // spot-check geometry: offset rails at the miters, extended rims
        let (p, q) = endpoints(&b, eids[12]);
        assert!(p.dist(Vec3::new(-2.0, -2.0, 0.0)) < 1e-9 && q.dist(Vec3::new(-2.0, -2.0, 10.0)) < 1e-9);
        let (p, q) = endpoints(&b, eids[16]);
        assert!(p.dist(Vec3::new(-2.0, -2.0, 0.0)) < 1e-9 && q.dist(Vec3::new(12.0, -2.0, 0.0)) < 1e-9);
        let (p, q) = endpoints(&b, eids[24]);
        assert!(p.dist(Vec3::new(10.0, 0.0, 0.0)) < 1e-9 && q.dist(Vec3::new(12.0, -2.0, 0.0)) < 1e-9);
        let mp = ck_topo::mass::mass_props(&b);
        assert!((mp.volume - 960.0).abs() < 1e-6, "volume {}", mp.volume);
        assert!((mp.area - 1152.0).abs() < 1e-6, "area {}", mp.area);
    }

    #[test]
    fn thicken_cyl_front_oracle() {
        // oracle probe: circle tube r=5 + thicken t=1.5 front: volume
        // pi*(6.5^2-5^2)*10 = 541.92…, area 830.95…
        let sheet = tube_cyl("F2", 5.0, 0.0, 10.0);
        let b = thicken(&sheet, "F3", 1.5, Side::Front, 1e-6).unwrap();
        let fids: Vec<String> = b.face_ids().map(|f| b.face(f).name.id()).collect();
        assert_eq!(fids, ["F2.S.e1~1", "F2.S.e1~2", "F3.W0", "F3.W1"].iter().map(|s| s.to_string()).collect::<Vec<_>>());
        let eids: Vec<EdgeId> = b.edge_ids().filter(|&e| !b.is_seam(e)).collect();
        assert_eq!(eids.len(), 4);
        for (i, r) in [5.0, 5.0, 6.5, 6.5].iter().enumerate() {
            match &b.edge(eids[i]).curve {
                Curve::Circle { frame, r: cr } => {
                    assert!((cr - r).abs() < 1e-9, "edge {i} r={cr}");
                    assert!(frame.z.dist(Vec3::Z) < 1e-9);
                }
                _ => panic!("edge {i} not a circle"),
            }
        }
        let mp = ck_topo::mass::mass_props(&b);
        let want = std::f64::consts::PI * (6.5 * 6.5 - 25.0) * 10.0;
        assert!((mp.volume - want).abs() < 1e-6, "volume {} want {want}", mp.volume);
        let want_area = 2.0 * std::f64::consts::PI * (6.5 + 5.0) * 10.0 + 2.0 * std::f64::consts::PI * 17.25;
        assert!((mp.area - want_area).abs() < 1e-6, "area {}", mp.area);
    }

    /// Half-cylinder patch (an open C-shaped sheet), axis Z, angles 0..pi.
    fn half_cyl(feat: &str, r: f64, z0: f64, z1: f64) -> Body {
        let mut b = Body::new();
        let lump = b.add_lump();
        let shell = b.add_shell(lump);
        let f = b.add_face(
            shell,
            Surface::Cylinder { frame: Frame { o: Vec3::ZERO, x: Vec3::X, y: Vec3::Y, z: Vec3::Z }, r },
            true,
            FaceName::side(feat, "e1"),
        );
        let pi = std::f64::consts::PI;
        let (a0, a1) = (Vec3::new(r, 0.0, z0), Vec3::new(-r, 0.0, z0));
        let (b0, b1) = (Vec3::new(r, 0.0, z1), Vec3::new(-r, 0.0, z1));
        let (va0, va1, vb0, vb1) = (b.add_vertex(a0), b.add_vertex(a1), b.add_vertex(b0), b.add_vertex(b1));
        let ea = b.add_edge(Curve::circle_x(Vec3::new(0.0, 0.0, z0), Vec3::Z, Vec3::X, r), 0.0, pi, va0, va1);
        let eb = b.add_edge(Curve::circle_x(Vec3::new(0.0, 0.0, z1), Vec3::Z, Vec3::X, r), 0.0, pi, vb0, vb1);
        let e1 = b.add_edge(Curve::line_through(a1, b1), 0.0, (b1 - a1).len(), va1, vb1);
        let e0 = b.add_edge(Curve::line_through(a0, b0), 0.0, (b0 - a0).len(), va0, vb0);
        let l = b.add_loop(f);
        b.add_coedge(l, ea, false, None);
        b.add_coedge(l, e1, false, None);
        b.add_coedge(l, eb, true, None);
        b.add_coedge(l, e0, true, None);
        b
    }

    #[test]
    fn thicken_half_cylinder_stays_on_the_offset_surface() {
        // an open C-shaped sheet thickened outwards: every rim edge of the offset layer must lie on the
        // offset cylinder (radius r + t), and the display mesh must cover the exact areas.
        let (r, t, h) = (30.0, 3.0, 20.0);
        let sheet = half_cyl("F2", r, 0.0, h);
        let b = thicken(&sheet, "F3", t, Side::Front, 1e-6).unwrap();
        let pi = std::f64::consts::PI;
        let want = pi * ((r + t) * (r + t) - r * r) / 2.0 * h;
        let mp = ck_topo::mass::mass_props(&b);
        assert!((mp.volume - want).abs() < 1e-3 * want, "volume {} want {want}", mp.volume);
        for e in b.edge_ids() {
            let ed = b.edge(e);
            if matches!(&ed.curve, Curve::Line { d, .. } if d.cross(Vec3::Z).len() > 1e-9) {
                continue; // the radial connectors between the layers
            }
            for p in ed.curve.sample(ed.t0, ed.t1, 9) {
                let rad = (p.x * p.x + p.y * p.y).sqrt();
                assert!((rad - r).abs() < 1e-9 || (rad - r - t).abs() < 1e-9, "edge point off both cylinders: radius {rad}");
            }
        }
        let tess = ck_tess::mesh::tessellate(&b);
        assert!(tess.mesh.failed.is_empty());
        let mut area = vec![0.0f64; b.num_faces()];
        let mut count = vec![0usize; b.num_faces()];
        for (ti, &fi) in tess.mesh.tri_face.iter().enumerate() {
            let p = &tess.mesh.positions[ti * 9..ti * 9 + 9];
            let v = |k: usize| Vec3::new(p[k * 3] as f64, p[k * 3 + 1] as f64, p[k * 3 + 2] as f64);
            area[fi as usize] += (v(1) - v(0)).cross(v(2) - v(0)).len() * 0.5;
            count[fi as usize] += 1;
        }
        let want_area = [pi * r * h, pi * (r + t) * h, pi * ((r + t) * (r + t) - r * r) / 2.0, t * h, pi * ((r + t) * (r + t) - r * r) / 2.0, t * h];
        let total: f64 = area.iter().sum();
        let want_total: f64 = want_area.iter().sum();
        assert!((total - want_total).abs() < 0.01 * want_total, "mesh area {total} want {want_total}: {area:?}");
        assert!(count.iter().all(|&c| c < 2000), "sliver blow-up: {count:?}");
    }

    #[test]
    fn thicken_errors() {
        let sheet = sheet_square("F2", Vec3::ZERO, Vec3::new(10.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0));
        // oracle: thickness 0 / negative -> invalidValue
        assert_eq!(thicken(&sheet, "F3", 0.0, Side::Front, 1e-6).unwrap_err().code, "invalidValue");
        assert_eq!(thicken(&sheet, "F3", -1.0, Side::Front, 1e-6).unwrap_err().code, "invalidValue");
        // oracle: cylinder r=5 back t=6 -> offsetTooLarge ("…centre of
        // curvature")
        let cyl = tube_cyl("F2", 5.0, 0.0, 10.0);
        let e = thicken(&cyl, "F3", 6.0, Side::Back, 1e-6).unwrap_err();
        assert_eq!(e.code, "offsetTooLarge");
        assert!(e.detail.contains("centre of curvature"), "{}", e.detail);
    }
}
