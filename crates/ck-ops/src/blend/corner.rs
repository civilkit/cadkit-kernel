//! Vertex blend of three equal fillets at a convex corner of planar faces
//! (the oracle's `F3.V0`, a sphere patch). RECONSTRUCTED from box probes
//! (`filletCorner3`), not a port of corner.rs (f2419 / f2420 ...).
//!
//! Geometry: the rolling ball touching the three planes sits at `c` (offset
//! planes meet there); the rounded corner is the part of the body within
//! `r` of the inner corner, i.e. the three edge cylinders and, where all
//! three offsets are positive, the ball. The mitre left by three sequential
//! edge fillets keeps too much material there; cutting the "corner block
//! minus ball" tool removes exactly the difference:
//!
//! * block `H` = the corner region bounded by the three face planes pushed
//!   OUT by a margin and the three planes through `c` perpendicular to the
//!   edges (the block's inner side), a hexahedron;
//! * tool `T = H - ball(c, r)`.
//!
//! `T`'s inner planes pass through the ball's centre (great circles), and
//! the ball is tangent to the edge cylinders along the circles those planes
//! cut; the block is pushed out so nothing but the ball touches the body's
//! planes.

use super::BlendError;
use crate::boolean::Op;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_geom::{Frame, Vec3};
use ck_topo::body::{Body, EdgeId, FaceId, FaceName, VertexId};

/// The three selected edges meeting at a convex 3-valent vertex of planar
/// faces: the vertex, its three edges (ascending ids) and the face opposite
/// to each edge (the one not containing it).
pub struct Corner {
    pub vertex: VertexId,
    pub edges: [EdgeId; 3],
    pub faces: [FaceId; 3],
    /// All three edges are concave (the void side of the vertex is a trihedral
    /// octant): the corner is filled with material (union), not cut.
    pub concave: bool,
    /// Two planes and a convex cylinder, one of the edges an arc round the cylinder's axis (the rim of a stepped disc at its step
    /// wall): `edges` are `[arc, line between the planes, line along the cylinder]` and `faces` the face opposite to each
    /// (`[wall plane, cylinder, top plane]`); see [`cyl_corner_tool`].
    pub curved: bool,
}

/// Corner at the vertex shared by exactly the three `selected` edges, or
/// `None` (not 3-valent / not planar / not straight).
pub fn find_corner(b: &Body, selected: &[EdgeId]) -> Vec<Corner> {
    let mut out = vec![];
    for v in b.vertex_ids() {
        let inc: Vec<EdgeId> = b.edge_ids().filter(|&e| b.edge(e).v0 == v || b.edge(e).v1 == v).collect();
        if inc.len() != 3 || !inc.iter().all(|e| selected.contains(e)) {
            continue;
        }
        if inc.iter().any(|&e| b.edge(e).v0 == b.edge(e).v1 || !matches!(b.edge(e).curve, ck_geom::curve::Curve::Line { .. })) {
            if let Some(c) = find_cyl_corner(b, v, &inc) {
                out.push(c);
            }
            continue;
        }
        // the three faces at the vertex, each planar
        let mut faces: Vec<FaceId> = vec![];
        for &e in &inc {
            for f in b.edge_faces(e) {
                if !faces.contains(&f) {
                    faces.push(f);
                }
            }
        }
        if faces.len() != 3 || faces.iter().any(|&f| !matches!(b.face(f).surface, Surface::Plane { .. })) {
            if let Some(c) = find_cyl_corner(b, v, &inc) {
                out.push(c);
            }
            continue;
        }
        let opposite = |e: EdgeId| faces.iter().copied().find(|f| !b.edge_faces(e).contains(f));
        let (Some(f0), Some(f1), Some(f2)) = (opposite(inc[0]), opposite(inc[1]), opposite(inc[2])) else { continue };
        // convexity: along edge i the face opposite to it leans toward the
        // body (convex, n.u < 0) or toward the void (concave, n.u > 0)
        let vp = b.vertex(v).p;
        let mut sgn = [0i32; 3];
        for (i, (&e, &f)) in inc.iter().zip([f0, f1, f2].iter()).enumerate() {
            let ed = b.edge(e);
            let other = if ed.v0 == v { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
            let d = b.planar_normal(f).dot((other - vp).normalize());
            sgn[i] = if d > 1e-9 { 1 } else if d < -1e-9 { -1 } else { 0 };
        }
        let concave = sgn.iter().all(|&s| s > 0);
        if !concave && !sgn.iter().all(|&s| s < 0) {
            continue; // mixed or degenerate vertex: no spherical corner
        }
        out.push(Corner { vertex: v, edges: [inc[0], inc[1], inc[2]], faces: [f0, f1, f2], concave, curved: false });
    }
    out
}

/// The tool `H - ball` for `corner` (see the module doc), its sphere face
/// named `name`. `Err(blendUnsupported)` for a concave / degenerate corner.
pub fn sphere_corner_tool(b: &Body, corner: &Corner, r: f64, feature: &str, name: &FaceName, tol: f64) -> Result<Body, BlendError> {
    if corner.curved {
        return cyl_corner_tool(b, corner, r, feature, name);
    }
    let vp = b.vertex(corner.vertex).p;
    let n: Vec<Vec3> = corner.faces.iter().map(|&f| b.planar_normal(f)).collect();
    // ball centre: r below each of the three planes
    // concave: the ball sits on the void side (r above each plane), the tool
    // is the block MINUS the ball, added to the body
    let sg = if corner.concave { 1.0 } else { -1.0 };
    let c = crate::intersect::solve3([n[0], n[1], n[2]], [sg * r, sg * r, sg * r]).map(|d| vp + d).ok_or_else(|| BlendError::code("blendUnsupported"))?;
    // edge directions away from the vertex
    let mut u = [Vec3::ZERO; 3];
    for (i, &e) in corner.edges.iter().enumerate() {
        let ed = b.edge(e);
        let other = if ed.v0 == corner.vertex { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
        u[i] = (other - vp).normalize();
        // the ball centre lies along every edge of the vertex
        if (c - vp).dot(u[i]) <= 0.0 {
            return Err(BlendError::code("blendUnsupported"));
        }
    }
    // the corner block: out(i) = plane of the face opposite to edge i, pushed
    // out by `m`; inn(i) = plane through c perpendicular to edge i
    // concave: the block is pushed into the material instead (normal -n), only by half the radius: the material
    // behind a face may be thinner than r + 1 (a pocket ceiling 3.4 under the top, r = 3.048), and a block that
    // reaches through it adds a lump on the far side
    let m = if corner.concave { 0.5 * r } else { r + 1.0 };
    let outer: [(Vec3, Vec3); 3] = std::array::from_fn(|i| (n[i] * -sg, vp - n[i] * (sg * m))); // (outward normal, point)
    sphere_block_minus_ball(c, u, outer, r, feature, name)
}

/// The ball centre and the directions of the three edges away from the vertex of a convex corner of three planar faces
/// that are not mutually perpendicular (a wall angle other than 90 degrees), `None` for any other corner. The tools of the
/// three edges end on the planes through the centre square to them ([`super::analytic::CornerTrim`]), so each one is the
/// same solid whatever order the edges are built in: a tool that runs on past the vertex cuts into the wedge of an obtuse
/// wall angle, and one built after another tool has cut the corner ends in the wrong place.
pub fn oblique_corner_centre(b: &Body, corner: &Corner, r: f64) -> Option<(Vec3, [Vec3; 3])> {
    if corner.concave || corner.curved {
        return None;
    }
    let vp = b.vertex(corner.vertex).p;
    let n: Vec<Vec3> = corner.faces.iter().map(|&f| b.planar_normal(f).normalize()).collect();
    if (0..3).all(|i| n[i].dot(n[(i + 1) % 3]).abs() < 1e-6) {
        return None;
    }
    let c = crate::intersect::solve3([n[0], n[1], n[2]], [-r, -r, -r]).map(|d| vp + d)?;
    let mut u = [Vec3::ZERO; 3];
    for (i, &e) in corner.edges.iter().enumerate() {
        let ed = b.edge(e);
        let other = if ed.v0 == corner.vertex { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
        u[i] = (other - vp).normalize();
        if (c - vp).dot(u[i]) <= 1e-9 || (c - vp).dot(u[i]) >= (other - vp).len() {
            return None;
        }
    }
    Some((c, u))
}

/// The tool of [`sphere_corner_tool`] from its planes: `c` the ball centre, `u[i]` the direction of edge `i` away from
/// the vertex (the block's inner plane `i` is the plane through `c` perpendicular to it, the block on the vertex side),
/// `outer[i]` the block's outer plane opposite to edge `i` as (outward normal of the block, a point on it). The
/// planes need not be mutually perpendicular: the ball touches the strip of edge `i` along the circle in the inner
/// plane `i`, so the contact point with the face opposite to edge `i` lies on the line of the other two inner planes.
pub fn sphere_block_minus_ball(c: Vec3, u: [Vec3; 3], outer: [(Vec3, Vec3); 3], r: f64, feature: &str, name: &FaceName) -> Result<Body, BlendError> {
    let out_pl = |i: usize| outer[i];
    let inn_pl = |i: usize| (u[i], c);
    let mut verts: Vec<Vec3> = vec![];
    for mask in 0..8usize {
        let pl: Vec<(Vec3, Vec3)> = (0..3).map(|i| if mask >> i & 1 == 1 { inn_pl(i) } else { out_pl(i) }).collect();
        let p = crate::intersect::solve3([pl[0].0, pl[1].0, pl[2].0], [pl[0].0.dot(pl[0].1), pl[1].0.dot(pl[1].1), pl[2].0.dot(pl[2].1)])
            .ok_or_else(|| BlendError::code("blendUnsupported"))?;
        verts.push(p);
    }
    // The block minus the ball octant at its corner `c = verts[7]`, built by
    // hand (a revolved sphere has no usable boundary for the boolean): the
    // corner vertex is replaced by the three points `a[i]` where the ball
    // touches the body's faces; the inner faces get a quarter-circle notch
    // and the octant becomes the (cavity-facing) sphere patch.
    let cc = verts[7];
    let e: Vec<Vec3> = (0..3).map(|i| (verts[7 ^ (1 << i)] - cc).normalize()).collect();
    let a: Vec<Vec3> = (0..3).map(|i| cc + e[i] * r).collect();
    let mut tb = ToolBuilder::new();
    let ids_v: Vec<usize> = (0..8).map(|mask| if mask == 7 { usize::MAX } else { tb.vertex(verts[mask]) }).collect();
    let ids_a: Vec<usize> = a.iter().map(|&p| tb.vertex(p)).collect();
    // outer faces: the quads of the vertices with bit i clear
    for i in 0..3 {
        let quad: Vec<usize> = (0..8usize).filter(|mask| mask >> i & 1 == 0).map(|mask| ids_v[mask]).collect();
        let segs = tb.cycle_by_angle(&quad, outer[i].0);
        tb.plane_face(outer[i].0, segs, FaceName::side(feature, &format!("k{}0", i)));
    }
    // inner faces: A_j, mask_j, opposite, mask_k, A_k and the arc back
    for i in 0..3 {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        let mj = 7 ^ (1 << j);
        let mk = 7 ^ (1 << k);
        let opp = 7 ^ (1 << j) ^ (1 << k);
        let pts = [ids_a[j], ids_v[mj], ids_v[opp], ids_v[mk], ids_a[k]];
        let mut segs: Vec<Seg> = (0..4).map(|t| Seg { a: pts[t], b: pts[t + 1], arc: None }).collect();
        segs.push(Seg { a: pts[4], b: pts[0], arc: Some((cc, r)) });
        tb.plane_face(u[i], segs, FaceName::side(feature, &format!("k{}1", i)));
    }
    // the sphere patch: the three arcs; outward normal toward the centre
    let segs = vec![
        Seg { a: ids_a[0], b: ids_a[1], arc: Some((cc, r)) },
        Seg { a: ids_a[1], b: ids_a[2], arc: Some((cc, r)) },
        Seg { a: ids_a[2], b: ids_a[0], arc: Some((cc, r)) },
    ];
    tb.sphere_face(cc, r, segs, name.clone());
    Ok(tb.finish())
}

/// Give the coedges of the sphere patch `name` exact straight pcurves again
/// (the boolean refits them by projection, which is off by ~1e-4 near the
/// pole). Valid for the orthogonal corner, where [`sphere_corner_tool`]
/// puts the pole at a patch corner so every arc is a meridian or the equator.
pub fn fix_sphere_pcurves(body: &mut Body, name: &str) {
    let Some(f) = body.find_face(name) else { return };
    let Surface::Sphere { frame, r } = body.face(f).surface.clone() else { return };
    let c = frame.o;
    let uo = |p: Vec3| (p - c).dot(frame.y).atan2((p - c).dot(frame.x));
    let is_pole = |p: Vec3| (p - c).normalize().dot(frame.z).abs() > 1.0 - 1e-9;
    let mut fixes: Vec<(usize, ck_geom::curve2::Curve2)> = vec![];
    for &l in &body.face(f).loops {
        for &cid in &body.lp(l).coedges {
            let ed = body.edge(body.coedge(cid).edge);
            if !matches!(ed.curve, Curve::Circle { .. }) {
                return;
            }
            let (pa, pb) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
            let (mut ua, mut ub) = (uo(pa), uo(pb));
            if is_pole(pa) {
                ua = ub;
            } else if is_pole(pb) {
                ub = ua;
            }
            let v = |p: Vec3| ((p - c) / r).dot(frame.z).clamp(-1.0, 1.0).asin();
            let mut du = ub - ua;
            while du > std::f64::consts::PI {
                du -= std::f64::consts::TAU;
            }
            while du < -std::f64::consts::PI {
                du += std::f64::consts::TAU;
            }
            let th = ed.t1 - ed.t0;
            let d = ck_geom::Vec2::new(du / th, (v(pb) - v(pa)) / th);
            let o = ck_geom::Vec2::new(ua, v(pa)) - d * ed.t0;
            // a straight pcurve is the arc only when the arc is a meridian or the equator (the orthogonal corner); at an
            // oblique corner (faces not mutually perpendicular) the arcs are slanted great circles: keep the boolean's refit
            let tm = 0.5 * (ed.t0 + ed.t1);
            let (um, vm) = (o.x + d.x * tm, o.y + d.y * tm);
            let on_sphere = c + (frame.x * um.cos() + frame.y * um.sin()) * (vm.cos() * r) + frame.z * (vm.sin() * r);
            if on_sphere.dist(ed.curve.eval(tm)) > 1e-7 * r {
                return;
            }
            fixes.push((cid.idx(), ck_geom::curve2::Curve2::Line { o, d }));
        }
    }
    for (i, pc) in fixes {
        body.coedges[i].pcurve = Some(pc);
    }
}

/// The six boundary vertices `[a01, a02, a12, a10, a20, a21]` of a 6-sided setback corner patch.
pub type SetbackHex = [Vec3; 6];

/// Evaluate the shorter circular arc from `pa` to `pb` about `c` of radius `r` at fraction `s in [0, 1]`.
fn eval_setback_arc(pa: Vec3, pb: Vec3, c: Vec3, r: f64, s: f64) -> Vec3 {
    let (ea, eb) = ((pa - c).normalize(), (pb - c).normalize());
    let th = ea.dot(eb).clamp(-1.0, 1.0).acos();
    let ez = ea.cross(eb).normalize();
    let ang = s * th;
    c + (ea * ang.cos() + ez.cross(ea) * ang.sin()) * r
}

/// Build the tensor-product cubic B-spline surface for a 6-sided setback corner patch with triple knots at
/// `u = 1/3` and `u = 2/3` so that the three top boundary segments (`Arc_2`, `L_0`, `Arc_1`), the bottom segment
/// (`Arc_0`), and the two straight side segments (`L_1`, `L_2`) are interpolated with zero corner overshoot.
fn setback_patch_surface(
    hex: &SetbackHex,
    centres: [Vec3; 3],
    radii: [f64; 3],
) -> Result<ck_geom::nurbs::bsurf::BSplineSurface, BlendError> {
    use ck_geom::fit::{averaged_knots, interpolate_with_many};
    use ck_geom::nurbs::bsurf::BSplineSurface;
    let [a01, a02, a12, a10, a20, a21] = *hex;
    let [c0, c1, c2] = centres;
    let [r0, r1, r2] = radii;
    const M: usize = 8;
    const NU_GRID: usize = 3 * M + 1; // 25
    const NV_GRID: usize = M + 1; // 9
    const H_STEPS: usize = 3 * M + 1; // 25 x 25 isotropic grid for harmonic relaxation

    let mut grid = vec![vec![Vec3::ZERO; H_STEPS]; NU_GRID];
    for i in 0..NU_GRID {
        let u = i as f64 / (NU_GRID - 1) as f64;
        grid[i][0] = eval_setback_arc(a01, a02, c0, r0, u);
        grid[i][H_STEPS - 1] = if i <= M {
            let s = i as f64 / M as f64;
            eval_setback_arc(a21, a20, c2, r2, s)
        } else if i <= 2 * M {
            let s = (i - M) as f64 / M as f64;
            a20 + (a10 - a20) * s
        } else {
            let s = (i - 2 * M) as f64 / M as f64;
            eval_setback_arc(a10, a12, c1, r1, s)
        };
    }
    for j in 0..H_STEPS {
        let v = j as f64 / (H_STEPS - 1) as f64;
        grid[0][j] = a01 + (a21 - a01) * v;
        grid[NU_GRID - 1][j] = a02 + (a12 - a02) * v;
    }
    // Initialize interior by Coons blend, then relax harmonically
    for i in 1..NU_GRID - 1 {
        let u = i as f64 / (NU_GRID - 1) as f64;
        for j in 1..H_STEPS - 1 {
            let v = j as f64 / (H_STEPS - 1) as f64;
            let bil = (a01 * (1.0 - u) + a02 * u) * (1.0 - v) + (grid[0][H_STEPS - 1] * (1.0 - u) + grid[NU_GRID - 1][H_STEPS - 1] * u) * v;
            grid[i][j] = grid[i][0] * (1.0 - v) + grid[i][H_STEPS - 1] * v + grid[0][j] * (1.0 - u) + grid[NU_GRID - 1][j] * u - bil;
        }
    }
    for _ in 0..300 {
        for i in 1..NU_GRID - 1 {
            for j in 1..H_STEPS - 1 {
                grid[i][j] = (grid[i - 1][j] + grid[i + 1][j] + grid[i][j - 1] + grid[i][j + 1]) * 0.25;
            }
        }
    }

    // Subsample v at every 3rd row (0, 3, 6, ..., 24 -> 9 rows) and interpolate each of the 3 u-thirds
    let u_local: Vec<f64> = (0..=M).map(|k| k as f64 / M as f64).collect();
    let knots_local = averaged_knots(&u_local, 3);
    let mut seg0_rows: Vec<Vec<Vec3>> = Vec::with_capacity(NV_GRID);
    let mut seg1_rows: Vec<Vec<Vec3>> = Vec::with_capacity(NV_GRID);
    let mut seg2_rows: Vec<Vec<Vec3>> = Vec::with_capacity(NV_GRID);
    for jv in 0..NV_GRID {
        let j = jv * 3;
        seg0_rows.push((0..=M).map(|i| grid[i][j]).collect());
        seg1_rows.push((M..=2 * M).map(|i| grid[i][j]).collect());
        seg2_rows.push((2 * M..=3 * M).map(|i| grid[i][j]).collect());
    }
    let c0_rows = interpolate_with_many(&seg0_rows, 3, &u_local, &knots_local).map_err(|_| BlendError::code("blendFailed"))?;
    let c1_rows = interpolate_with_many(&seg1_rows, 3, &u_local, &knots_local).map_err(|_| BlendError::code("blendFailed"))?;
    let c2_rows = interpolate_with_many(&seg2_rows, 3, &u_local, &knots_local).map_err(|_| BlendError::code("blendFailed"))?;

    let nu = (M + 1) + M + M; // 25
    let mut cols_for_v: Vec<Vec<Vec3>> = vec![Vec::with_capacity(NV_GRID); nu];
    for jv in 0..NV_GRID {
        let mut combined = Vec::with_capacity(nu);
        combined.extend_from_slice(&c0_rows[jv]);
        combined.extend_from_slice(&c1_rows[jv][1..]);
        combined.extend_from_slice(&c2_rows[jv][1..]);
        for (i, &pt) in combined.iter().enumerate() {
            cols_for_v[i].push(pt);
        }
    }
    let ctrl_cols = interpolate_with_many(&cols_for_v, 3, &u_local, &knots_local).map_err(|_| BlendError::code("blendFailed"))?;

    let mut knots_u = vec![0.0; 4];
    for &k in &knots_local[4..M + 1] {
        knots_u.push(k / 3.0);
    }
    knots_u.extend_from_slice(&[1.0 / 3.0; 3]);
    for &k in &knots_local[4..M + 1] {
        knots_u.push((1.0 + k) / 3.0);
    }
    knots_u.extend_from_slice(&[2.0 / 3.0; 3]);
    for &k in &knots_local[4..M + 1] {
        knots_u.push((2.0 + k) / 3.0);
    }
    knots_u.extend_from_slice(&[1.0; 4]);

    let nv = NV_GRID;
    let mut ctrl = Vec::with_capacity(nu * nv);
    for col in ctrl_cols {
        ctrl.extend(col);
    }
    BSplineSurface::new(3, 3, knots_u, knots_local, nu, nv, ctrl, None).map_err(|_| BlendError::code("blendFailed"))
}

/// Build the corner tool for a 3-edge blend corner with explicit setback distance `setback` (clamped safely to
/// `[min_setback, 0.45 * min_edge_len]`) and per-edge radii `radii`. When the clamped setback equals `min_setback`
/// and the three radii are equal, delegates to [`sphere_corner_tool`]; otherwise constructs the 6-sided setback
/// corner tool and returns its 6 patch vertices (`SetbackHex`) for [`fix_setback_pcurves`].
pub fn setback_corner_tool(
    b: &Body,
    corner: &Corner,
    radii: [f64; 3],
    setback: f64,
    feature: &str,
    name: &FaceName,
    tol: f64,
) -> Result<(Body, Option<SetbackHex>), BlendError> {
    if corner.curved {
        return cyl_corner_tool(b, corner, radii[0], feature, name).map(|t| (t, None));
    }
    let vp = b.vertex(corner.vertex).p;
    let n: Vec<Vec3> = corner.faces.iter().map(|&f| b.planar_normal(f).normalize()).collect();
    let sg = if corner.concave { 1.0 } else { -1.0 };
    let mut u = [Vec3::ZERO; 3];
    let mut elen = [0.0f64; 3];
    for (i, &e) in corner.edges.iter().enumerate() {
        let ed = b.edge(e);
        let other = if ed.v0 == corner.vertex { b.vertex(ed.v1).p } else { b.vertex(ed.v0).p };
        elen[i] = (other - vp).len();
        if !(elen[i] > 1e-9) {
            return Err(BlendError::code("blendUnsupported"));
        }
        u[i] = (other - vp) / elen[i];
    }
    let r_min = radii[0].min(radii[1]).min(radii[2]);
    let r_max = radii[0].max(radii[1]).max(radii[2]);
    let c_max = crate::intersect::solve3([n[0], n[1], n[2]], [sg * r_max, sg * r_max, sg * r_max])
        .map(|d| vp + d)
        .ok_or_else(|| BlendError::code("blendUnsupported"))?;
    for i in 0..3 {
        if (c_max - vp).dot(u[i]) <= 0.0 {
            return Err(BlendError::code("blendUnsupported"));
        }
    }
    let min_setback = (0..3).map(|i| (c_max - vp).dot(u[i])).fold(0.0f64, f64::max);
    let max_setback = elen.iter().map(|&l| 0.45 * l).fold(f64::INFINITY, f64::min);
    if !(max_setback >= min_setback) {
        return Err(BlendError::detail(
            "blendTooLarge",
            &format!("the rounds are too large for a setback corner here: the setback patch needs at least {min_setback:.6} mm along each edge but the shortest edge allows {max_setback:.6} mm (0.45 of its length)"),
        ));
    }
    let mut s = setback.clamp(min_setback, max_setback);
    if (s - setback).abs() > 1e-9 {
        super::rounds::SETBACK_REPORT.with(|r| r.borrow_mut().adjusted.push([setback, s, min_setback, max_setback]));
    }
    super::rounds::SETBACK_REPORT.with(|r| r.borrow_mut().used += 1);
    let equal_radii = (r_max - r_min) <= 1e-9 * (1.0 + r_max);
    let equal_nat = (0..3).all(|i| ((c_max - vp).dot(u[i]) - min_setback).abs() <= 1e-4 * (1.0 + r_max));
    if equal_radii && equal_nat && (s - min_setback).abs() <= 1e-3 * (1.0 + r_max) {
        return sphere_corner_tool(b, corner, radii[0], feature, name, tol).map(|t| (t, None));
    }
    if s <= min_setback + 1e-3 * r_max {
        s = (min_setback + 0.05 * r_max).min(max_setback);
    }

    let inn_pl: [(Vec3, Vec3); 3] = std::array::from_fn(|i| (u[i], vp + u[i] * s));
    let mut c_pt = [Vec3::ZERO; 3];
    let mut a_ij = [Vec3::ZERO; 3];
    let mut a_ik = [Vec3::ZERO; 3];
    for i in 0..3 {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        let ri = radii[i];
        c_pt[i] = crate::intersect::solve3(
            [n[j], n[k], u[i]],
            [n[j].dot(vp) + sg * ri, n[k].dot(vp) + sg * ri, u[i].dot(inn_pl[i].1)],
        )
        .ok_or_else(|| BlendError::code("blendUnsupported"))?;
        a_ij[i] = c_pt[i] - n[j] * (sg * ri);
        a_ik[i] = c_pt[i] - n[k] * (sg * ri);
    }

    let mut leg_pl = [(Vec3::ZERO, Vec3::ZERO); 3];
    for j in 0..3 {
        let (k, i) = ((j + 1) % 3, (j + 2) % 3);
        let (akj, aij) = (a_ik[k], a_ij[i]);
        let chord = aij - akj;
        if chord.len() <= 10.0 * tol {
            return Err(BlendError::code("blendUnsupported"));
        }
        let mut ln = chord.normalize().cross(n[j]).normalize();
        let mid = (akj + aij) * 0.5;
        if ln.dot(mid - vp) < 0.0 {
            ln = -ln;
        }
        leg_pl[j] = (ln, mid);
    }

    let m = if corner.concave { 0.5 * r_min } else { r_max + 1.0 };
    let outer: [(Vec3, Vec3); 3] = std::array::from_fn(|j| (n[j] * -sg, vp - n[j] * (sg * m)));
    let intersect3 = |p0: (Vec3, Vec3), p1: (Vec3, Vec3), p2: (Vec3, Vec3)| -> Result<Vec3, BlendError> {
        crate::intersect::solve3(
            [p0.0, p1.0, p2.0],
            [p0.0.dot(p0.1), p1.0.dot(p1.1), p2.0.dot(p2.1)],
        )
        .ok_or_else(|| BlendError::code("blendUnsupported"))
    };
    let o_pt = intersect3(outer[0], outer[1], outer[2])?;
    let mut e_pt = [Vec3::ZERO; 3];
    let mut b_ij = [Vec3::ZERO; 3];
    let mut b_ik = [Vec3::ZERO; 3];
    for i in 0..3 {
        let (j, k) = ((i + 1) % 3, (i + 2) % 3);
        e_pt[i] = intersect3(inn_pl[i], outer[j], outer[k])?;
        b_ij[i] = intersect3(inn_pl[i], leg_pl[j], outer[j])?;
        b_ik[i] = intersect3(inn_pl[i], leg_pl[k], outer[k])?;
    }

    let mut tb = ToolBuilder::new();
    let id_o = tb.vertex(o_pt);
    let id_e: [usize; 3] = std::array::from_fn(|i| tb.vertex(e_pt[i]));
    let id_aij: [usize; 3] = std::array::from_fn(|i| tb.vertex(a_ij[i]));
    let id_aik: [usize; 3] = std::array::from_fn(|i| tb.vertex(a_ik[i]));
    let id_bij: [usize; 3] = std::array::from_fn(|i| tb.vertex(b_ij[i]));
    let id_bik: [usize; 3] = std::array::from_fn(|i| tb.vertex(b_ik[i]));

    for j in 0..3 {
        let (k, i) = ((j + 1) % 3, (j + 2) % 3);
        let pts = [id_o, id_e[k], id_bik[k], id_bij[i], id_e[i]];
        let segs: Vec<Seg> = (0..5).map(|t| Seg { a: pts[t], b: pts[(t + 1) % 5], arc: None }).collect();
        tb.plane_face(outer[j].0, segs, FaceName::side(feature, &format!("k{}0", j)));
    }
    for i in 0..3 {
        let pts = [id_aij[i], id_bij[i], id_e[i], id_bik[i], id_aik[i]];
        let mut segs: Vec<Seg> = (0..4).map(|t| Seg { a: pts[t], b: pts[t + 1], arc: None }).collect();
        segs.push(Seg { a: id_aik[i], b: id_aij[i], arc: Some((c_pt[i], radii[i])) });
        tb.plane_face(u[i], segs, FaceName::side(feature, &format!("k{}1", i)));
    }
    for j in 0..3 {
        let (k, i) = ((j + 1) % 3, (j + 2) % 3);
        let pts = [id_aik[k], id_bik[k], id_bij[i], id_aij[i]];
        let segs: Vec<Seg> = (0..4).map(|t| Seg { a: pts[t], b: pts[(t + 1) % 4], arc: None }).collect();
        tb.plane_face(leg_pl[j].0, segs, FaceName::side(feature, &format!("k{}2", j)));
    }

    let hex: SetbackHex = [a_ij[0], a_ik[0], a_ij[1], a_ik[1], a_ij[2], a_ik[2]];
    let surf = setback_patch_surface(&hex, c_pt, radii)?;
    let patch_segs = vec![
        Seg { a: id_aij[0], b: id_aik[0], arc: Some((c_pt[0], radii[0])) },
        Seg { a: id_aik[0], b: id_aij[1], arc: None },
        Seg { a: id_aij[1], b: id_aik[1], arc: Some((c_pt[1], radii[1])) },
        Seg { a: id_aik[1], b: id_aij[2], arc: None },
        Seg { a: id_aij[2], b: id_aik[2], arc: Some((c_pt[2], radii[2])) },
        Seg { a: id_aik[2], b: id_aij[0], arc: None },
    ];
    let outward = (u[0] + u[1] + u[2]).normalize();
    tb.bspline_face(surf, patch_segs, outward, name.clone());
    let mut tool = tb.finish();
    fix_setback_pcurves(&mut tool, &name.id(), &hex);
    Ok((tool, Some(hex)))
}

/// Assign exact linear `(u, v)` boundary pcurves to the 6 coedges of a setback corner patch `name`.
pub fn fix_setback_pcurves(body: &mut Body, name: &str, hex: &SetbackHex) {
    let Some(f) = body.find_face(name) else { return };
    if !matches!(body.face(f).surface, Surface::BSpline(_)) {
        return;
    }
    let hex_uv = [
        ck_geom::Vec2::new(0.0, 0.0),
        ck_geom::Vec2::new(1.0, 0.0),
        ck_geom::Vec2::new(1.0, 1.0),
        ck_geom::Vec2::new(2.0 / 3.0, 1.0),
        ck_geom::Vec2::new(1.0 / 3.0, 1.0),
        ck_geom::Vec2::new(0.0, 1.0),
    ];
    let uv_of = |p: Vec3| -> Option<ck_geom::Vec2> {
        let (idx, d) = hex.iter().enumerate().map(|(i, &q)| (i, q.dist(p))).min_by(|a, b| a.1.total_cmp(&b.1))?;
        (d <= 1e-3 * (1.0 + p.len())).then_some(hex_uv[idx])
    };
    let mut fixes: Vec<(usize, ck_geom::curve2::Curve2)> = vec![];
    for &l in &body.face(f).loops {
        for &cid in &body.lp(l).coedges {
            let ed = body.edge(body.coedge(cid).edge);
            let (pa, pb) = (ed.curve.eval(ed.t0), ed.curve.eval(ed.t1));
            let (Some(ua), Some(ub)) = (uv_of(pa), uv_of(pb)) else { return };
            let span = ed.t1 - ed.t0;
            if span.abs() <= 1e-14 {
                return;
            }
            let d = (ub - ua) * (1.0 / span);
            let o = ua - d * ed.t0;
            fixes.push((cid.idx(), ck_geom::curve2::Curve2::Line { o, d }));
        }
    }
    for (i, pc) in fixes {
        body.coedges[i].pcurve = Some(pc);
    }
}

/// One directed loop segment of [`ToolBuilder`]: a line, or an arc of the
/// circle through both ends about `centre` (the shorter way).
#[derive(Clone)]
pub(super) struct Seg {
    pub(super) a: usize,
    pub(super) b: usize,
    pub(super) arc: Option<(Vec3, f64)>,
}

/// Minimal B-rep builder for the corner tool: planar faces with line / arc
/// edges and one spherical patch, pcurves left to the projection fallback.
pub(super) struct ToolBuilder {
    body: Body,
    verts: Vec<(Vec3, VertexId)>,
    edges: Vec<((usize, usize, bool), EdgeId)>,
    shell: ck_topo::body::ShellId,
}

impl ToolBuilder {
    pub(super) fn new() -> ToolBuilder {
        let mut body = Body::new();
        let lump = body.add_lump();
        let shell = body.add_shell(lump);
        ToolBuilder { body, verts: vec![], edges: vec![], shell }
    }
    pub(super) fn vertex(&mut self, p: Vec3) -> usize {
        let v = self.body.add_vertex(p);
        self.verts.push((p, v));
        self.verts.len() - 1
    }
    /// The vertex at `p`, reusing one already built there within `snap`, so
    /// faces built from independently extracted wires share their common edges.
    pub(super) fn vertex_shared(&mut self, p: Vec3, snap: f64) -> usize {
        if let Some(k) = self.verts.iter().position(|(q, _)| q.dist(p) <= snap) {
            return k;
        }
        self.vertex(p)
    }
    /// The quad `idx` as segments counter-clockwise about `normal`.
    fn cycle_by_angle(&self, idx: &[usize], normal: Vec3) -> Vec<Seg> {
        let pts: Vec<Vec3> = idx.iter().map(|&i| self.verts[i].0).collect();
        let ctr = pts.iter().fold(Vec3::ZERO, |acc, &p| acc + p) / pts.len() as f64;
        let ex = (pts[0] - ctr).normalize();
        let ey = normal.cross(ex);
        let mut order: Vec<usize> = (0..idx.len()).collect();
        order.sort_by(|&x, &y| {
            let ang = |i: usize| (pts[i] - ctr).dot(ey).atan2((pts[i] - ctr).dot(ex));
            ang(x).total_cmp(&ang(y))
        });
        (0..order.len()).map(|t| Seg { a: idx[order[t]], b: idx[order[(t + 1) % order.len()]], arc: None }).collect()
    }
    fn seg_points(&self, s: &Seg) -> Vec<Vec3> {
        let (pa, pb) = (self.verts[s.a].0, self.verts[s.b].0);
        match s.arc {
            None => vec![pa],
            Some((c, r)) => {
                let (ea, eb) = ((pa - c).normalize(), (pb - c).normalize());
                let th = ea.dot(eb).clamp(-1.0, 1.0).acos();
                let ez = ea.cross(eb).normalize();
                (0..8).map(|k| c + (ea * (th * k as f64 / 8.0).cos() + ez.cross(ea) * (th * k as f64 / 8.0).sin()) * r).collect()
            }
        }
    }
    /// Reverse a loop (segments and order).
    pub(super) fn reversed(segs: Vec<Seg>) -> Vec<Seg> {
        segs.into_iter().rev().map(|s| Seg { a: s.b, b: s.a, arc: s.arc }).collect()
    }
    fn edge(&mut self, s: &Seg) -> (EdgeId, bool) {
        let key = (s.a.min(s.b), s.a.max(s.b), s.arc.is_some());
        let rev = s.a > s.b;
        if let Some((_, e)) = self.edges.iter().find(|(k, _)| *k == key) {
            return (*e, rev);
        }
        // edge direction low index -> high index
        let (lo, hi) = (key.0, key.1);
        let (pa, pb) = (self.verts[lo].0, self.verts[hi].0);
        let (curve, t1) = match s.arc {
            None => {
                let len = pa.dist(pb);
                (Curve::Line { o: pa, d: (pb - pa) / len }, len)
            }
            Some((c, r)) => {
                let (ea, eb) = ((pa - c).normalize(), (pb - c).normalize());
                let th = ea.dot(eb).clamp(-1.0, 1.0).acos();
                let ez = ea.cross(eb).normalize();
                (Curve::Circle { frame: Frame { o: c, x: ea, y: ez.cross(ea), z: ez }, r }, th)
            }
        };
        let e = self.body.add_edge(curve, 0.0, t1, self.verts[lo].1, self.verts[hi].1);
        self.edges.push((key, e));
        (e, rev)
    }
    pub(super) fn plane_face(&mut self, normal: Vec3, mut segs: Vec<Seg>, name: FaceName) {
        // counter-clockwise about `normal`
        let mut area = Vec3::ZERO;
        let pts: Vec<Vec3> = segs.iter().flat_map(|s| self.seg_points(s)).collect();
        for i in 0..pts.len() {
            area = area + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        if area.dot(normal) < 0.0 {
            segs = ToolBuilder::reversed(segs);
        }
        let frame = ck_geom::frame_from(pts[0], normal, pts[1] - pts[0]);
        let f = self.body.add_face(self.shell, Surface::Plane { frame }, true, name);
        let lp = self.body.add_loop(f);
        for s in &segs {
            let (e, rev) = self.edge(s);
            let pc = ck_topo::make::plane_pcurve(&frame, &self.body.edge(e).curve.clone());
            self.body.add_coedge(lp, e, rev, pc);
        }
    }
    fn sphere_face(&mut self, c: Vec3, r: f64, mut segs: Vec<Seg>, name: FaceName) {
        // outward normal of the tool here points at the centre; the loop runs
        // counter-clockwise about it
        let pts: Vec<Vec3> = segs.iter().map(|s| self.verts[s.a].0).collect();
        let ctr = pts.iter().fold(Vec3::ZERO, |acc, &p| acc + p) / pts.len() as f64;
        let nrm = (pts[1] - pts[0]).cross(pts[2] - pts[0]);
        if nrm.dot(c - ctr) < 0.0 {
            segs = ToolBuilder::reversed(segs);
        }
        // frame: the pole at the first patch corner (so the two arcs through
        // it are meridians and the opposite arc is the equator: all straight
        // in uv when the corner is orthogonal)
        let axis = (pts[0] - c).normalize();
        let frame = ck_geom::frame_from(c, axis, pts[1] - c);
        let f = self.body.add_face(self.shell, Surface::Sphere { frame, r }, false, name);
        let lp = self.body.add_loop(f);
        let uv_of = |p: Vec3, other_u: f64| {
            let d = (p - c) / r;
            let v = d.dot(frame.z).clamp(-1.0, 1.0).asin();
            let u = if v.abs() > std::f64::consts::FRAC_PI_2 - 1e-9 { other_u } else { d.dot(frame.y).atan2(d.dot(frame.x)) };
            ck_geom::Vec2::new(u, v)
        };
        let orthogonal = (pts[1] - c).normalize().dot((pts[0] - c).normalize()).abs() < 1e-9;
        for s in &segs {
            let (e, rev) = self.edge(s);
            let pc = if orthogonal {
                let ed = self.body.edge(e).clone();
                let (pa, pb) = (self.body.vertex(ed.v0).p, self.body.vertex(ed.v1).p);
                let uo = |p: Vec3| ((p - c).dot(frame.y)).atan2((p - c).dot(frame.x));
                let (ua, ub) = (uo(pa), uo(pb));
                let pole_a = (pa - c).normalize().dot(frame.z).abs() > 1.0 - 1e-9;
                let pole_b = (pb - c).normalize().dot(frame.z).abs() > 1.0 - 1e-9;
                let (ua, ub) = if pole_a { (ub, ub) } else if pole_b { (ua, ua) } else { (ua, ub) };
                let (a, bb) = (uv_of(pa, ua), uv_of(pb, ub));
                // equator arcs: u runs the short way
                let mut du = bb.x - a.x;
                while du > std::f64::consts::PI {
                    du -= std::f64::consts::TAU;
                }
                while du < -std::f64::consts::PI {
                    du += std::f64::consts::TAU;
                }
                let th = ed.t1 - ed.t0;
                Some(ck_geom::curve2::Curve2::Line { o: a, d: ck_geom::Vec2::new(du / th, (bb.y - a.y) / th) })
            } else {
                None
            };
            self.body.add_coedge(lp, e, rev, pc);
        }
    }
    pub(super) fn finish(mut self) -> Body {
        ck_topo::region::rebuild_lumps(&mut self.body);
        self.body
    }

    /// The loop of `segs` on the spline `surf`, oriented counter-clockwise
    /// about `outward` (the loop's polygon area decides, as in
    /// [`ToolBuilder::plane_face`]); `sense` puts the face's outward normal on
    /// `outward`. Pcurves are left to the projection fallback (as in
    /// [`ToolBuilder::sphere_face`]).
    pub(super) fn bspline_face(&mut self, surf: ck_geom::nurbs::bsurf::BSplineSurface, mut segs: Vec<Seg>, outward: Vec3, name: FaceName) {
        let pts: Vec<Vec3> = segs.iter().flat_map(|s| self.seg_points(s)).collect();
        let mut area = Vec3::ZERO;
        for i in 0..pts.len() {
            area = area + pts[i].cross(pts[(i + 1) % pts.len()]);
        }
        if area.dot(outward) < 0.0 {
            segs = ToolBuilder::reversed(segs);
        }
        let ((u0, u1), (v0, v1)) = surf.domain();
        let sense = surf.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).dot(outward) >= 0.0;
        let f = self.body.add_face(self.shell, Surface::BSpline(Box::new(surf)), sense, name);
        let lp = self.body.add_loop(f);
        for s in &segs {
            let (e, rev) = self.edge(s);
            self.body.add_coedge(lp, e, rev, None);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::build::{apply_tool, blend_edges, find_edge, Spec};
    use crate::boolean::tests::box_body;
    use ck_topo::mass::mass_props;

    #[test]
    fn box_corner_sphere() {
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let sel = [
            ("F2.C1".to_string(), "F2.S.e1".to_string(), Vec3::new(20., 10., 0.)),
            ("F2.C1".to_string(), "F2.S.e2".to_string(), Vec3::new(40., 10., -10.)),
            ("F2.S.e1".to_string(), "F2.S.e2".to_string(), Vec3::new(40., 5., 0.)),
        ];
        let mitred = blend_edges(&b, &sel, Spec::Fillet(2.0, crate::blend::build::FilletProfile::Circular), "F3", 1e-6).unwrap();
        let ids: Vec<EdgeId> = sel.iter().map(|(a, bb, at)| find_edge(&b, a, bb, *at).unwrap()).collect();
        let corners = find_corner(&b, &ids);
        assert_eq!(corners.len(), 1);
        let name = FaceName::other("F3", "V0");
        let tool = sphere_corner_tool(&b, &corners[0], 2.0, "F3", &name, 1e-6).unwrap();
        assert!(ck_topo::check::check(&tool, 1e-6).is_empty());
        // block 5^3 minus the ball octant
        assert!((mass_props(&tool).volume - (125.0 - std::f64::consts::PI * 4.0 / 3.0)).abs() < 1e-9);
        let (mut r, _) = apply_tool(&mitred, &tool, Op::Cut, &name, vec![], 1e-6).unwrap();
        fix_sphere_pcurves(&mut r, &name.id());
        assert!(ck_topo::check::check(&r, 1e-6).is_empty());
        let faces: Vec<String> = r.face_ids().map(|f| r.face_id_str(f)).collect();
        assert_eq!(faces[faces.len() - 4..], ["F3.B0", "F3.B1", "F3.B2", "F3.V0"]);
        // oracle volume of filletCorner3 (exact: 8000 - 64(1-pi/4)4/4... see tools/brep_cases.mjs)
        assert!((mass_props(&r).volume - 7941.250719950454).abs() < 1e-6, "{}", mass_props(&r).volume);
    }

    #[test]
    fn box_corner_explicit_setback_varies_patch_area_and_volume() {
        use crate::blend::build::{fillet_unequal, fillet_with_opts, BlendOpts, BlendOptsGuard};
        let b = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let sel = [
            ("F2.C1".to_string(), "F2.S.e1".to_string(), Vec3::new(20., 10., 0.)),
            ("F2.C1".to_string(), "F2.S.e2".to_string(), Vec3::new(40., 10., -10.)),
            ("F2.S.e1".to_string(), "F2.S.e2".to_string(), Vec3::new(40., 5., 0.)),
        ];

        let mut areas = vec![];
        let mut vols = vec![];
        for sb in [2.0, 3.0, 4.0] {
            let opts = BlendOpts { start_offset: None, end_offset: None, setback: Some(sb) };
            let out = fillet_with_opts(&b, &sel, 2.0, opts, "F3", 1e-6).unwrap();
            assert!(ck_topo::check::check(&out, 1e-6).is_empty(), "check failed for setback={sb}");
            assert!(ck_check::validate(&out, 1e-6).is_empty(), "validate failed for setback={sb}: {:?}", ck_check::validate(&out, 1e-6));
            let v0 = out.find_face("F3.V0").expect("corner patch F3.V0");
            let patch_area = ck_topo::mass::faces_moments(&out, &[v0], Vec3::ZERO).area.abs();
            let vol = mass_props(&out).volume;
            areas.push(patch_area);
            vols.push(vol);
        }
        assert!(areas[0] < areas[1] && areas[1] < areas[2], "patch areas should grow with setback: {areas:?}");
        assert!(vols[0] > vols[1] && vols[1] > vols[2], "volumes should shrink with larger convex setback: {vols:?}");

        // Unequal radii at a 3-edge corner with explicit setback
        let _g = BlendOptsGuard::new(BlendOpts { start_offset: None, end_offset: None, setback: Some(3.5) });
        let out_unequal = fillet_unequal(&b, &sel, &[1.5, 2.0, 2.5], "F3", 1e-6).unwrap();
        assert!(ck_check::validate(&out_unequal, 1e-6).is_empty(), "unequal setback validate: {:?}", ck_check::validate(&out_unequal, 1e-6));
        assert!(out_unequal.find_face("F3.V0").is_some());
    }
}

/// A convex corner of two planes and a cylinder whose three edges are all selected: a rim arc round the cylinder's axis (between
/// the cylinder and the top plane, square to the axis), the line where the two planes meet, and a line along the cylinder (between
/// the cylinder and the wall plane, which is parallel to the axis). `None` for any other shape.
fn find_cyl_corner(b: &Body, v: VertexId, inc: &[EdgeId]) -> Option<Corner> {
    use super::torus_corner::edge_concave;
    let arcs: Vec<EdgeId> = inc.iter().copied().filter(|&e| matches!(b.edge(e).curve, Curve::Circle { .. })).collect();
    if arcs.len() != 1 || inc.iter().any(|&e| b.edge(e).v0 == b.edge(e).v1) {
        return None;
    }
    if inc.iter().any(|&e| edge_concave(b, e) != Some(false)) {
        return None;
    }
    let e_arc = arcs[0];
    let lines: Vec<EdgeId> = inc.iter().copied().filter(|&e| e != e_arc).collect();
    if lines.iter().any(|&e| !matches!(b.edge(e).curve, Curve::Line { .. })) {
        return None;
    }
    let mut faces: Vec<FaceId> = vec![];
    for &e in inc {
        for f in b.edge_faces(e) {
            if !faces.contains(&f) {
                faces.push(f);
            }
        }
    }
    if faces.len() != 3 {
        return None;
    }
    let cyls: Vec<FaceId> = faces.iter().copied().filter(|&f| matches!(b.face(f).surface, Surface::Cylinder { .. })).collect();
    if cyls.len() != 1 || faces.iter().filter(|&&f| matches!(b.face(f).surface, Surface::Plane { .. })).count() != 2 {
        return None;
    }
    let fc = cyls[0];
    let on_c = |e: EdgeId| b.edge_faces(e).contains(&fc);
    let (e_vert, e_top) = if on_c(lines[0]) && !on_c(lines[1]) {
        (lines[0], lines[1])
    } else if on_c(lines[1]) && !on_c(lines[0]) {
        (lines[1], lines[0])
    } else {
        return None;
    };
    if !on_c(e_arc) {
        return None;
    }
    let opp = |e: EdgeId| faces.iter().copied().find(|f| !b.edge_faces(e).contains(f));
    let (f_w, f_c, f_t) = (opp(e_arc)?, opp(e_top)?, opp(e_vert)?);
    if f_c != fc {
        return None;
    }
    let Surface::Cylinder { frame, .. } = &b.face(fc).surface else { return None };
    let ax = frame.z.normalize();
    let Curve::Line { d: dv, .. } = b.edge(e_vert).curve else { return None };
    let Curve::Line { d: dt, .. } = b.edge(e_top).curve else { return None };
    let Some((_, circ_axis, _)) = b.edge(e_arc).curve.circle_info().map(|(c, a, r)| (c, a.normalize(), r)) else { return None };
    // the vertical edge and the arc's axis along the cylinder's, the top plane square to it, the wall plane parallel to it
    let nt = b.planar_normal(f_t).normalize();
    let nw = b.planar_normal(f_w).normalize();
    if dv.normalize().cross(ax).len() > 1e-7 || circ_axis.cross(ax).len() > 1e-7 || nt.cross(ax).len() > 1e-7 || nw.dot(ax).abs() > 1e-7 || dt.normalize().dot(ax).abs() > 1e-7 {
        return None;
    }
    // a convex cylinder (its outward normal points away from the axis at the vertex)
    let vp = b.vertex(v).p;
    let (u, w) = b.face(fc).surface.closest_param(vp);
    let rad = (vp - frame.o) - ax * (vp - frame.o).dot(ax);
    if b.outward_normal(fc, u, w).dot(rad) <= 0.0 {
        return None;
    }
    Some(Corner { vertex: v, edges: [e_arc, e_top, e_vert], faces: [f_w, f_c, f_t], concave: false, curved: true })
}

/// Direction of the edge `e` leaving the vertex `v` (the tangent of an arc at `v`).
pub(super) fn leaving(b: &Body, e: EdgeId, v: VertexId) -> Vec3 {
    let ed = b.edge(e);
    let sgn = if ed.t1 >= ed.t0 { 1.0 } else { -1.0 };
    let d = if ed.v0 == v { ed.curve.tangent(ed.t0) * sgn } else { -ed.curve.tangent(ed.t1) * sgn };
    d.normalize()
}

/// Geometry of the vertex blend of [`find_cyl_corner`]: the rolling ball of radius `r` touches the two planes and the cylinder at
/// `c` (`r` inside each: on the line of the two offset planes, `R - r` from the axis, the root on the vertex side of every edge),
/// the directions `u` of the three edges at `c` (the arc's is the tangent of the circle through `c`), and the block's outer planes.
pub(super) struct CylGeom {
    pub c: Vec3,
    pub u: [Vec3; 3],
    pub outer: [(Vec3, Vec3); 3],
    /// How far the ball centre is from the vertex along each edge: a length for the two lines, the angle (radians) for the arc.
    pub along: [f64; 3],
}

/// Shortest length an edge must keep past the end of its tool at the cylinder corner. A rest of 0.1 mm or less leaves sliver faces
/// and edges a hair off the cylinder (`corner_stress`: rests from 0.12 mm up build, in every pick order), so such a radius is
/// refused; 0.25 keeps a margin.
const MIN_REST: f64 = 0.25;

pub(super) fn cyl_corner_geom(b: &Body, corner: &Corner, r: f64) -> Result<CylGeom, BlendError> {
    let unsupported = || BlendError::code("blendUnsupported");
    let refuse = |m: String| BlendError::detail("blendUnsupported", &m);
    let [e_arc, e_top, e_vert] = corner.edges;
    let [f_w, f_c, f_t] = corner.faces;
    let vp = b.vertex(corner.vertex).p;
    let Surface::Cylinder { frame, r: rc } = b.face(f_c).surface.clone() else { return Err(unsupported()) };
    let ax = frame.z.normalize();
    let (nw, nt) = (b.planar_normal(f_w).normalize(), b.planar_normal(f_t).normalize());
    let d = nw.cross(nt).normalize();
    let p0 = vp + crate::intersect::solve3([nw, nt, d], [-r, -r, 0.0]).ok_or_else(unsupported)?;
    let q = (p0 - frame.o) - ax * (p0 - frame.o).dot(ax);
    let (qd, qq) = (q.dot(d), q.dot(q) - (rc - r) * (rc - r));
    let disc = qd * qd - qq;
    let too_big = || refuse(format!("the radius {r} is too large for the cylinder (radius {rc:.3}) at this corner: the rolling ball cannot touch the wall, the top and the cylinder at once"));
    if !(disc >= 0.0) || rc - r <= 0.0 {
        return Err(too_big());
    }
    let away = |e: EdgeId| leaving(b, e, corner.vertex);
    let (u_top, u_vert, u_arc_v) = (away(e_top), away(e_vert), away(e_arc));
    let radv = (vp - frame.o) - ax * (vp - frame.o).dot(ax);
    if radv.len() < 1e-12 {
        return Err(unsupported());
    }
    let radv = radv.normalize();
    let mut best: Option<(Vec3, [Vec3; 3], f64)> = None;
    for sgn in [-1.0, 1.0] {
        let c = p0 + d * (-qd + sgn * disc.sqrt());
        let radc = (c - frame.o) - ax * (c - frame.o).dot(ax);
        if radc.len() < 1e-12 {
            continue;
        }
        let radc = radc.normalize();
        let t = ax.cross(radc);
        let u_arc = if t.dot(u_arc_v) >= 0.0 { t } else { -t };
        let u = [u_arc, u_top, u_vert];
        if u.iter().all(|x| (c - vp).dot(*x) > 0.0) {
            best = Some((c, u, radv.dot(radc).clamp(-1.0, 1.0).acos()));
        }
    }
    let Some((c, u, phi)) = best else { return Err(too_big()) };
    let along = [phi, (c - vp).dot(u_top), (c - vp).dot(u_vert)];
    // each edge must go on past the end of its tool by a real length: the tools end on the planes through the ball's centre, and
    // a rest shorter than that leaves slivers (the vertical edge of a step as tall as the radius is the usual case)
    let axial = (vp - frame.o).dot(ax);
    let rim_r = ((vp - frame.o) - ax * axial).len();
    for (i, (e, name)) in [(e_arc, "rim arc"), (e_top, "wall-top edge"), (e_vert, "vertical edge")].into_iter().enumerate() {
        let len = b.edge_length(e);
        let used = if i == 0 { along[0] * rim_r } else { along[i] };
        if len - used < MIN_REST {
            return Err(refuse(format!("the {name} at this corner is {len:.3} mm long, too short for a round of radius {r}: the corner patch needs the edge to run on {:.3} mm past the ball's contact line", used + MIN_REST)));
        }
    }
    let m = r + 1.0;
    let outer: [(Vec3, Vec3); 3] = [(nw, vp + nw * m), (radv, vp + radv * m), (nt, vp + nt * m)];
    Ok(CylGeom { c, u, outer, along })
}

/// The vertex blend of [`find_cyl_corner`], see [`cyl_corner_geom`]: the corner block minus the ball. The tools of the three
/// edge fillets end on the planes through `c` square to their edges (`rounds::cyl_corner_trims`), which for the arc is the radial
/// plane through `c`; `sphere_block_minus_ball` takes the block's planes as they are (they need not be perpendicular).
fn cyl_corner_tool(b: &Body, corner: &Corner, r: f64, feature: &str, name: &FaceName) -> Result<Body, BlendError> {
    let g = cyl_corner_geom(b, corner, r)?;
    sphere_block_minus_ball(g.c, g.u, g.outer, r, feature, name)
}
