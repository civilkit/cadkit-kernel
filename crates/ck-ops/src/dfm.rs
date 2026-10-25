//! Design-for-manufacture queries (extension requests `dfmMinWall`, `dfmMinInternalRadius`, `dfmPocketRatio`,
//! `dfmOverhang`, `dfmConstantSection`). Read-only, deterministic. Wire shapes: docs/spec-agent-interface.md ("DFM queries").
//!
//! Every reply names the rule (`rule.id`, the limit the caller gave), a `pass` flag and, for each violation, the value, the
//! persistent face (and edge) names and a point on the part. A body a query cannot judge is answered with a structured
//! refusal (`supported:false`, `refusal:{code, reason, faces}`) instead of a guess:
//!
//! * `emptyBody`: the body has no faces.
//! * `nonAnalyticFaces`: the query classifies faces by their exact surface kind (radius, floor, wall direction) and the
//!   body has B-spline / screw / offset faces (or extrusions / revolutions of a spline profile), e.g. a lofted or an
//!   imported free-form body. `faces` lists them.
//! * `tessellationFailed`: the query reads the display mesh (overhang) and some faces could not be meshed.
//!
//! Reuse: wall thickness is the `thickness` query (ray method), internal corners / overhang / concavity are the
//! `checkProfile` rule helpers, section areas are the `section` query.

use crate::profile::{rules, Ctx};
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_json::{Json, Obj};
use ck_topo::body::{Body, FaceId};
use std::cmp::Ordering;

fn r6(x: f64) -> f64 {
    if x.is_finite() {
        let r = (x * 1e6).round() / 1e6;
        if r == 0.0 {
            0.0
        } else {
            r
        }
    } else {
        0.0
    }
}

fn pt(p: Vec3) -> Json {
    Json::Arr(vec![Json::Num(r6(p.x)), Json::Num(r6(p.y)), Json::Num(r6(p.z))])
}

fn names(v: &[String]) -> Json {
    Json::Arr(v.iter().map(|s| Json::from(s.as_str())).collect())
}

fn pos_num(msg: &Json, ty: &str, key: &str) -> Result<f64, String> {
    msg.get_f64(key).filter(|x| x.is_finite() && *x > 0.0).ok_or_else(|| format!("{ty}: {key} must be a positive number"))
}

fn dir_arg(msg: &Json, ty: &str, key: &str) -> Result<Option<Vec3>, String> {
    match msg.get(key) {
        None | Some(Json::Null) => Ok(None),
        Some(j) => match j.as_vec3() {
            Some(a) if a.iter().all(|x| x.is_finite()) && Vec3::from_array(a).len() > 1e-12 => Ok(Some(Vec3::from_array(a).normalize())),
            _ => Err(format!("{ty}: {key} must be a non-zero [x, y, z]")),
        },
    }
}

fn req_dir(msg: &Json, ty: &str, key: &str) -> Result<Vec3, String> {
    dir_arg(msg, ty, key)?.ok_or_else(|| format!("{ty}: {key} must be a non-zero [x, y, z]"))
}

fn rule(id: &str, limit_key: &str, limit: Option<f64>, unit: &str, doc: &str) -> Json {
    let mut o = Obj::new().put("id", id);
    if let Some(l) = limit {
        o = o.put(limit_key, r6(l));
    }
    o.put("unit", unit).put("description", doc).build()
}

fn refusal(ty_rule: &str, code: &str, reason: &str, faces: Vec<String>) -> Json {
    Obj::new()
        .put("rule", Obj::new().put("id", ty_rule).build())
        .put("supported", false)
        .put("pass", Json::Null)
        .put("refusal", Obj::new().put("code", code).put("reason", reason).put("faces", names(&faces)).build())
        .build()
}

fn is_analytic(s: &Surface) -> bool {
    match s {
        Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. } => true,
        Surface::Extrusion { profile, .. } | Surface::Revolution { profile, .. } => {
            matches!(**profile, Curve::Line { .. } | Curve::Circle { .. } | Curve::Ellipse { .. } | Curve::Conic { .. })
        }
        Surface::BSpline(_) | Surface::Screw { .. } | Surface::Offset { .. } => false,
    }
}

/// `Some(refusal)` when the body cannot be judged by a query that classifies faces by exact surface kind.
fn needs_analytic(body: &Body, id: &str) -> Option<Json> {
    if body.num_faces() == 0 {
        return Some(refusal(id, "emptyBody", "the body has no faces", vec![]));
    }
    let bad: Vec<String> = body.face_ids().filter(|&f| !is_analytic(&body.face(f).surface)).map(|f| body.face_id_str(f)).collect();
    if bad.is_empty() {
        None
    } else {
        Some(refusal(
            id,
            "nonAnalyticFaces",
            "the body has free-form (B-spline, screw or offset) faces; this query needs analytic surfaces (planes, cylinders, cones, spheres, tori, extrusions and revolutions of lines / circles)",
            bad,
        ))
    }
}

fn face_point(cx: &Ctx, fi: usize) -> Vec3 {
    let f = cx.fids[fi];
    let s = &cx.body.face(f).surface;
    let c = cx.centroid(fi);
    let (u, v) = s.closest_param(c);
    let p = s.eval(u, v);
    if p.is_finite() {
        p
    } else {
        c
    }
}

// ---------------------------------------------------------------- dfmMinWall

/// `{type:"dfmMinWall", threshold, opts?:{samples}}`: the thinnest wall (inward ray method of `thickness`) against a
/// minimum wall thickness; every location below it with the two faces and the points.
pub fn min_wall_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    const ID: &str = "minWall";
    let thr = pos_num(msg, "dfmMinWall", "threshold")?;
    if body.num_faces() == 0 {
        return Ok(refusal(ID, "emptyBody", "the body has no faces", vec![]));
    }
    let samples = msg
        .get("opts")
        .and_then(|o| o.get("samples"))
        .or_else(|| msg.get("samples"))
        .and_then(|j| j.as_usize())
        .unwrap_or(6000)
        .clamp(200, 200000);
    let sub = Obj::new().put("samples", samples as f64).put("top", 200.0).put("below", thr).put("method", "ray").build();
    let th = crate::analysis::thickness_reply(body, &sub)?;
    let min = th.get_f64("minThickness");
    let mut violations = vec![];
    for t in th.get_arr("thin").unwrap_or(&[]) {
        let v = t.get_f64("thickness").unwrap_or(f64::INFINITY);
        if v >= thr - 1e-9 {
            continue;
        }
        let faces = [t.get_str("face").unwrap_or(""), t.get_str("farFace").unwrap_or("")];
        violations.push(
            Obj::new()
                .put("rule", ID)
                .put("thickness", r6(v))
                .put("limit", r6(thr))
                .put("faces", Json::Arr(faces.iter().map(|s| Json::from(*s)).collect()))
                .put("point", t.get("point").cloned().unwrap_or(Json::Null))
                .put("farPoint", t.get("farPoint").cloned().unwrap_or(Json::Null))
                .build(),
        );
    }
    let loc = th.get("location");
    let minimum = match (min, loc) {
        (Some(m), Some(l)) => Obj::new()
            .put("thickness", r6(m))
            .put("faces", Json::Arr(vec![Json::from(l.get_str("face").unwrap_or("")), Json::from(l.get_str("farFace").unwrap_or(""))]))
            .put("point", l.get("point").cloned().unwrap_or(Json::Null))
            .put("farPoint", l.get("farPoint").cloned().unwrap_or(Json::Null))
            .build(),
        _ => Json::Null,
    };
    let pass = min.map_or(true, |m| m >= thr - 1e-9);
    Ok(Obj::new()
        .put("rule", rule(ID, "min", Some(thr), "mm", "Wall thickness (inward ray from surface samples to the opposite wall) must be at least the threshold. Sampled: a sliver thinner than the sample spacing can be missed."))
        .put("supported", true)
        .put("pass", pass)
        .put("threshold", r6(thr))
        .put("minimum", minimum)
        .put("violations", Json::Arr(violations))
        .put("samples", th.get_f64("samples").unwrap_or(0.0))
        .put("truncated", th.get("truncated").cloned().unwrap_or(Json::Bool(false)))
        .build())
}

// ---------------------------------------------------------------- dfmMinInternalRadius

struct Corner {
    radius: f64,
    kind: &'static str,
    faces: Vec<String>,
    edge: Option<String>,
    at: Vec3,
}

fn corner_json(c: &Corner, tool: f64) -> Json {
    let mut o = Obj::new()
        .put("rule", "minInternalRadius")
        .put("radius", r6(c.radius))
        .put("limit", r6(tool))
        .put("kind", c.kind)
        .put("faces", names(&c.faces));
    if let Some(e) = &c.edge {
        o = o.put("edge", e.as_str());
    }
    o.put("point", pt(c.at)).build()
}

/// `{type:"dfmMinInternalRadius", toolRadius, direction?}`: the smallest concave corner radius of walls parallel to the tool
/// axis (`direction`, default +Z) against the cutter radius. Concave cylinders give their exact radius, sharp concave edges
/// radius 0; full cylinders (drilled holes) are not corners.
pub fn min_internal_radius_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    const ID: &str = "minInternalRadius";
    let tool = pos_num(msg, "dfmMinInternalRadius", "toolRadius")?;
    let axis = dir_arg(msg, "dfmMinInternalRadius", "direction")?.unwrap_or(Vec3::Z);
    if let Some(r) = needs_analytic(body, ID) {
        return Ok(r);
    }
    let cx = Ctx::new(body, 1000);
    let cos5 = 5f64.to_radians().cos();
    let mut corners: Vec<Corner> = vec![];
    for fi in 0..cx.fids.len() {
        let f = cx.fids[fi];
        let Surface::Cylinder { frame, r } = &body.face(f).surface else { continue };
        if frame.z.dot(axis).abs() < cos5 || !rules::is_concave_cylinder(&cx, f) || rules::full_cylinder(&cx, f) {
            continue;
        }
        corners.push(Corner { radius: *r, kind: "fillet", faces: vec![cx.names[fi].clone()], edge: None, at: face_point(&cx, fi) });
    }
    for ne in cx.edges() {
        let ed = body.edge(ne.first);
        let Curve::Line { d, .. } = &ed.curve else { continue };
        if d.normalize().dot(axis).abs() < cos5 {
            continue;
        }
        let Some((true, ang)) = rules::edge_concavity(&cx, ne.first) else { continue };
        if ang < 5f64.to_radians() {
            continue;
        }
        let mut faces: Vec<String> = ne.faces.iter().map(|&f| body.face_id_str(f)).collect();
        faces.sort();
        faces.dedup();
        corners.push(Corner { radius: 0.0, kind: "sharp", faces, edge: Some(ne.key.clone()), at: body.edge_mid(ne.first) });
    }
    corners.sort_by(|a, b| a.radius.partial_cmp(&b.radius).unwrap_or(Ordering::Equal).then_with(|| a.faces.cmp(&b.faces)).then_with(|| a.edge.cmp(&b.edge)));
    let viol: Vec<Json> = corners.iter().filter(|c| c.radius < tool - 1e-9).map(|c| corner_json(c, tool)).collect();
    let minimum = corners.first().map_or(Json::Null, |c| corner_json(c, tool));
    Ok(Obj::new()
        .put("rule", rule(ID, "toolRadius", Some(tool), "mm", "Concave corners parallel to the tool axis must have a radius of at least the cutter radius (sharp concave edges are radius 0)."))
        .put("supported", true)
        .put("pass", viol.is_empty())
        .put("toolRadius", r6(tool))
        .put("direction", pt(axis))
        .put("minimum", minimum)
        .put("corners", Json::Arr(corners.iter().take(200).map(|c| corner_json(c, tool)).collect()))
        .put("violations", Json::Arr(viol))
        .build())
}

// ---------------------------------------------------------------- dfmPocketRatio

/// Narrowest caliper width of a planar point set (exact: rotating calipers on the convex hull), in the plane of unit `n`.
fn min_width(pts: &[Vec3], n: Vec3) -> f64 {
    let u = n.any_perp().normalize();
    let v = n.cross(u).normalize();
    let mut p: Vec<(f64, f64)> = pts.iter().map(|q| (q.dot(u), q.dot(v))).collect();
    p.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
    p.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-12 && (a.1 - b.1).abs() < 1e-12);
    if p.len() < 3 {
        return 0.0;
    }
    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let mut h: Vec<(f64, f64)> = vec![];
    for &q in &p {
        while h.len() >= 2 && cross(h[h.len() - 2], h[h.len() - 1], q) <= 1e-12 {
            h.pop();
        }
        h.push(q);
    }
    let lower = h.len() + 1;
    for &q in p.iter().rev().skip(1) {
        while h.len() >= lower && cross(h[h.len() - 2], h[h.len() - 1], q) <= 1e-12 {
            h.pop();
        }
        h.push(q);
    }
    h.pop();
    let n = h.len();
    if n < 3 {
        return 0.0;
    }
    let mut best = f64::INFINITY;
    for i in 0..n {
        let (a, b) = (h[i], h[(i + 1) % n]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = (dx * dx + dy * dy).sqrt();
        if l < 1e-12 {
            continue;
        }
        let w = h.iter().map(|q| ((q.0 - a.0) * dy - (q.1 - a.1) * dx).abs() / l).fold(0.0, f64::max);
        best = best.min(w);
    }
    if best.is_finite() {
        best
    } else {
        0.0
    }
}

struct Pocket {
    floor: String,
    walls: Vec<String>,
    depth: f64,
    width: f64,
    at: Vec3,
}

/// Closed pockets and blind holes: a planar floor whose every boundary edge is a concave corner (or a tangent concave fillet).
fn find_pockets(cx: &Ctx) -> Vec<Pocket> {
    let b = cx.body;
    let tan = 5f64.to_radians();
    let mut out = vec![];
    for fi in 0..cx.fids.len() {
        let f = cx.fids[fi];
        if !matches!(b.face(f).surface, Surface::Plane { .. }) {
            continue;
        }
        let n = b.outward_normal(f, 0.0, 0.0);
        if !n.is_finite() || n.len() < 0.5 {
            continue;
        }
        let n = n.normalize();
        let (mut ok, mut concave) = (true, 0usize);
        let mut wall_faces: Vec<FaceId> = vec![];
        let mut blends: Vec<FaceId> = vec![];
        for e in b.face_edges(f) {
            if b.is_seam(e) {
                continue;
            }
            let other: Vec<FaceId> = b.edge_faces(e).into_iter().filter(|&g| g != f).collect();
            let Some((conc, ang)) = rules::edge_concavity(cx, e) else {
                ok = false;
                break;
            };
            if ang >= tan {
                if !conc {
                    ok = false;
                    break;
                }
                concave += 1;
                wall_faces.extend(other);
            } else if other.len() == 1 && rules::is_concave_cylinder(cx, other[0]) {
                concave += 1;
                blends.push(other[0]);
            } else {
                ok = false;
                break;
            }
        }
        if !ok || concave == 0 {
            continue;
        }
        for &g in &blends {
            for h in b.face_neighbors(g) {
                if h != f && !blends.contains(&h) {
                    wall_faces.push(h);
                }
            }
        }
        wall_faces.sort_by_key(|g| g.idx());
        wall_faces.dedup();
        let o = b.edge_ids().find(|&e| b.edge_faces(e).contains(&f)).map(|e| b.vertex(b.edge(e).v0).p);
        let Some(o) = o else { continue };
        let mut depth = 0.0f64;
        for &g in wall_faces.iter().chain(blends.iter()) {
            for e in b.face_edges(g) {
                let ed = b.edge(e);
                for v in [ed.v0, ed.v1] {
                    depth = depth.max((b.vertex(v).p - o).dot(n));
                }
            }
        }
        if depth <= 1e-9 {
            continue;
        }
        // floor outline: a single circle is exact (diameter); otherwise the hull of the floor mesh
        let outline: Vec<_> = b.face_edges(f).into_iter().filter(|&e| !b.is_seam(e)).collect();
        let width = match outline.as_slice() {
            [e] if matches!(b.edge(*e).curve, Curve::Circle { .. }) => match &b.edge(*e).curve {
                Curve::Circle { r, .. } => 2.0 * r,
                _ => 0.0,
            },
            // straight outline: the exact vertices; otherwise the display mesh of the floor
            es if es.iter().all(|&e| matches!(b.edge(e).curve, Curve::Line { .. })) => {
                let pts: Vec<Vec3> = es.iter().flat_map(|&e| [b.vertex(b.edge(e).v0).p, b.vertex(b.edge(e).v1).p]).collect();
                min_width(&pts, n)
            }
            _ => {
                let pts: Vec<Vec3> = cx.tris().iter().filter(|t| t.face == fi).flat_map(|t| [t.a, t.b, t.c]).collect();
                min_width(&pts, n)
            }
        };
        if width <= 1e-9 {
            continue;
        }
        let mut walls: Vec<String> = wall_faces.iter().map(|&g| b.face_id_str(g)).collect();
        walls.sort();
        walls.dedup();
        out.push(Pocket { floor: cx.names[fi].clone(), walls, depth, width, at: face_point(cx, fi) });
    }
    out
}

/// `{type:"dfmPocketRatio", maxRatio}`: depth / width of every closed pocket (and blind hole with a flat bottom). Depth is the
/// height of the walls above the planar floor, width the floor outline's narrowest caliper width. Open slots (a floor with a
/// convex edge) and pockets with a conical or free-form bottom are not counted.
pub fn pocket_ratio_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    const ID: &str = "pocketRatio";
    let max = pos_num(msg, "dfmPocketRatio", "maxRatio")?;
    if let Some(r) = needs_analytic(body, ID) {
        return Ok(r);
    }
    let cx = Ctx::new(body, 1000);
    let mut ps = find_pockets(&cx);
    ps.sort_by(|a, b| (b.depth / b.width).partial_cmp(&(a.depth / a.width)).unwrap_or(Ordering::Equal).then_with(|| a.floor.cmp(&b.floor)));
    let js = |p: &Pocket| {
        let ratio = p.depth / p.width;
        Obj::new()
            .put("rule", ID)
            .put("floor", p.floor.as_str())
            .put("walls", names(&p.walls))
            .put("faces", names(&[vec![p.floor.clone()], p.walls.clone()].concat()))
            .put("depth", r6(p.depth))
            .put("width", r6(p.width))
            .put("ratio", r6(ratio))
            .put("limit", r6(max))
            .put("fails", ratio > max + 1e-9)
            .put("point", pt(p.at))
            .build()
    };
    let all: Vec<Json> = ps.iter().map(js).collect();
    let viol: Vec<Json> = all.iter().filter(|j| j.get_bool("fails") == Some(true)).cloned().collect();
    Ok(Obj::new()
        .put("rule", rule(ID, "maxRatio", Some(max), "", "Pocket depth divided by width must not exceed the limit."))
        .put("supported", true)
        .put("pass", viol.is_empty())
        .put("maxRatio", r6(max))
        .put("maximum", all.first().cloned().unwrap_or(Json::Null))
        .put("pockets", Json::Arr(all))
        .put("violations", Json::Arr(viol))
        .build())
}

// ---------------------------------------------------------------- dfmOverhang

/// `{type:"dfmOverhang", direction, angle, minArea?}`: faces whose downward-facing surface is steeper than `angle` degrees
/// from vertical (0 = wall, 90 = flat ceiling; same convention as the `overhang` profile rule) against the build direction;
/// overhanging area per face and a rough support volume (columns from each overhanging triangle down to the first surface
/// below it or the build plate). Faces on the build plate are ignored.
pub fn overhang_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    const ID: &str = "overhang";
    let up = req_dir(msg, "dfmOverhang", "direction")?;
    let lim = msg.get_f64("angle").filter(|a| a.is_finite() && (0.0..=90.0).contains(a)).ok_or("dfmOverhang: angle must be a number in [0, 90]")?;
    let min_area = match msg.get("minArea") {
        None | Some(Json::Null) => 0.0,
        Some(j) => j.as_f64().filter(|x| x.is_finite() && *x >= 0.0).ok_or("dfmOverhang: minArea must be a number >= 0")?,
    };
    if body.num_faces() == 0 {
        return Ok(refusal(ID, "emptyBody", "the body has no faces", vec![]));
    }
    let cx = Ctx::new(body, 1000);
    let tris = cx.tris();
    if !cx.skipped().is_empty() {
        let f: Vec<String> = cx.skipped().iter().filter_map(|&i| cx.names.get(i).cloned()).collect();
        return Ok(refusal(ID, "tessellationFailed", "some faces could not be meshed, so their overhang cannot be measured", f));
    }
    let (lo, _) = cx.height_range(up);
    let tol = 1e-4 * cx.diag() + 1e-6;
    let n = cx.fids.len();
    let (mut area, mut worst, mut big) = (vec![0.0; n], vec![0.0f64; n], vec![(0.0f64, Vec3::ZERO); n]);
    let mut support = vec![0.0f64; n];
    let eps = 1e-6 * cx.diag();
    for t in tris {
        let s = t.n.dot(up);
        let on_plate = s < -0.9 && [t.a, t.b, t.c].iter().all(|p| p.dot(up) <= lo + tol);
        if s >= 0.0 || on_plate {
            continue;
        }
        let alpha = (-s).clamp(0.0, 1.0).asin().to_degrees();
        if alpha <= lim + 1e-6 {
            continue;
        }
        area[t.face] += t.area;
        worst[t.face] = worst[t.face].max(alpha);
        let c = (t.a + t.b + t.c) * (1.0 / 3.0);
        if t.area > big[t.face].0 {
            big[t.face] = (t.area, c);
        }
        let h = match cx.cast(c - up * eps, -up, cx.diag() * 2.0) {
            Some((d, _)) => d + eps,
            None => (c.dot(up) - lo).max(0.0),
        };
        support[t.face] += t.area * (-s) * h;
    }
    let mut order: Vec<usize> = (0..n).filter(|&i| area[i] > 1e-9 && area[i] >= min_area).collect();
    order.sort_by(|&a, &b| area[b].partial_cmp(&area[a]).unwrap_or(Ordering::Equal).then_with(|| cx.names[a].cmp(&cx.names[b])));
    let mut face_area = vec![0.0; n];
    for t in tris {
        face_area[t.face] += t.area;
    }
    let faces: Vec<Json> = order
        .iter()
        .map(|&i| {
            Obj::new()
                .put("rule", ID)
                .put("face", cx.names[i].as_str())
                .put("faces", names(&[cx.names[i].clone()]))
                .put("overhangAngle", r6(worst[i]))
                .put("limit", r6(lim))
                .put("overhangArea", r6(area[i]))
                .put("supportVolume", r6(support[i]))
                .put("faceArea", r6(face_area[i]))
                .put("point", pt(big[i].1))
                .build()
        })
        .collect();
    let total: f64 = order.iter().map(|&i| area[i]).sum();
    let support_total: f64 = order.iter().map(|&i| support[i]).sum();
    Ok(Obj::new()
        .put("rule", rule(ID, "angle", Some(lim), "deg", "Downward-facing surface steeper than the angle, measured from vertical against the build direction (0 = wall, 90 = flat ceiling), needs support. Faces on the build plate are ignored."))
        .put("supported", true)
        .put("pass", faces.is_empty())
        .put("direction", pt(up))
        .put("angle", r6(lim))
        .put("faces", Json::Arr(faces))
        .put("overhangArea", r6(total))
        .put("supportVolume", r6(support_total))
        .put("supportVolumeMethod", "columns from overhanging triangles to the first surface below or the build plate; a rough estimate")
        .build())
}

// ---------------------------------------------------------------- dfmConstantSection

/// Lowest height along unit `a` of edge `e`: exact for lines and circles, sampled otherwise.
fn edge_low(b: &Body, e: ck_topo::body::EdgeId, a: Vec3) -> f64 {
    let ed = b.edge(e);
    match &ed.curve {
        Curve::Line { .. } => b.vertex(ed.v0).p.dot(a).min(b.vertex(ed.v1).p.dot(a)),
        Curve::Circle { frame, r } => {
            let h = |t: f64| (frame.o + (frame.x * t.cos() + frame.y * t.sin()) * *r).dot(a);
            let (ax, ay) = (frame.x.dot(a), frame.y.dot(a));
            let low_t = ay.atan2(ax) + std::f64::consts::PI; // cos(t - phi) = -1
            let mut best = h(ed.t0).min(h(ed.t1));
            let span = ed.t1 - ed.t0;
            if span > 0.0 {
                let k = ((ed.t0 - low_t) / std::f64::consts::TAU).ceil();
                let t = low_t + k * std::f64::consts::TAU;
                if t <= ed.t1 {
                    best = best.min(h(t));
                }
            }
            best
        }
        _ => b.sample_edge(e, 1e-4).iter().map(|p| p.dot(a)).fold(f64::INFINITY, f64::min),
    }
}

/// `{type:"dfmConstantSection", axis}`: is the body a constant extrusion along `axis`? True when every face is a side wall
/// parallel to the axis (plane, cylinder, extrusion) or a plane cap perpendicular to it at the two ends. Otherwise the first
/// station along the axis where the section changes: an interior cap (a step, a blind pocket floor) or the lowest point of a
/// face that is not parallel to the axis (taper, cone, sphere).
pub fn constant_section_reply(body: &Body, msg: &Json) -> Result<Json, String> {
    const ID: &str = "constantSection";
    let a = req_dir(msg, "dfmConstantSection", "axis")?;
    if let Some(r) = needs_analytic(body, ID) {
        return Ok(r);
    }
    let cx = Ctx::new(body, 1000);
    if !cx.skipped().is_empty() {
        let f: Vec<String> = cx.skipped().iter().filter_map(|&i| cx.names.get(i).cloned()).collect();
        return Ok(refusal(ID, "tessellationFailed", "some faces could not be meshed, so the axis extent cannot be measured", f));
    }
    let (lo, hi) = cx.height_range(a);
    let tol = 1e-6 * cx.diag() + 1e-9;
    let sin_tol = 1e-7;
    let parallel = |v: Vec3| v.normalize().cross(a).len() < sin_tol;
    // (height of the change, face index, interior cap?)
    let mut changes: Vec<(f64, usize)> = vec![];
    for fi in 0..cx.fids.len() {
        let f = cx.fids[fi];
        let s = &body.face(f).surface;
        let side = match s {
            Surface::Plane { frame } => {
                let n = frame.z;
                if n.dot(a).abs() < sin_tol {
                    true
                } else if n.cross(a).len() < sin_tol {
                    let h = frame.o.dot(a);
                    if h > lo + tol && h < hi - tol {
                        changes.push((h, fi));
                    }
                    continue;
                } else {
                    false
                }
            }
            Surface::Cylinder { frame, .. } => parallel(frame.z),
            Surface::Extrusion { dir, .. } => parallel(*dir),
            Surface::Revolution { profile, axis, .. } => matches!(**profile, Curve::Line { d, .. } if parallel(d)) && parallel(*axis),
            _ => false,
        };
        if side {
            continue;
        }
        let mesh_low = cx.tris().iter().filter(|t| t.face == fi).flat_map(|t| [t.a, t.b, t.c]).map(|p| p.dot(a)).fold(f64::INFINITY, f64::min);
        // the boundary edges give the exact lowest point of a hole wall, cone rim, taper edge; the mesh covers the interior
        let low = body.face_edges(f).into_iter().map(|e| edge_low(body, e, a)).fold(mesh_low, f64::min);
        changes.push((if low.is_finite() { low } else { lo }, fi));
    }
    let length = hi - lo;
    let mut o = Obj::new()
        .put("rule", rule(ID, "axis", None, "", "The body must be a constant extrusion along the axis: one cross-section over its whole length."))
        .put("supported", true)
        .put("axis", pt(a))
        .put("start", r6(lo))
        .put("end", r6(hi))
        .put("length", r6(length));
    let tess = ck_tess::mesh::tessellate_memo(body);
    let area_at = |h: f64| crate::section::section(body, &tess, a * h, a);
    if changes.is_empty() {
        let s = area_at(0.5 * (lo + hi));
        return Ok(o.put("pass", true).put("constant", true).put("station", Json::Null).put("sectionArea", r6(s.area)).put("loops", s.loop_count() as f64).build());
    }
    let first = changes.iter().map(|c| c.0).fold(f64::INFINITY, f64::min);
    let mut at_faces: Vec<String> = changes.iter().filter(|c| c.0 <= first + tol).map(|c| cx.names[c.1].clone()).collect();
    at_faces.sort();
    at_faces.dedup();
    let fi = changes.iter().filter(|c| c.0 <= first + tol).map(|c| c.1).min().unwrap_or(0);
    let p = {
        let pts: Vec<Vec3> = cx.tris().iter().filter(|t| t.face == fi).flat_map(|t| [t.a, t.b, t.c]).collect();
        pts.iter().copied().min_by(|p, q| (p.dot(a) - first).abs().partial_cmp(&(q.dot(a) - first).abs()).unwrap_or(Ordering::Equal)).unwrap_or(cx.centroid(fi))
    };
    let mut station = Obj::new().put("position", r6(first)).put("offset", r6(first - lo)).put("faces", names(&at_faces)).put("point", pt(p));
    if first > lo + tol {
        station = station.put("sectionAreaBefore", r6(area_at(0.5 * (lo + first)).area));
    }
    if hi > first + tol {
        station = station.put("sectionAreaAfter", r6(area_at(0.5 * (first + hi)).area));
    }
    o = o.put("pass", false).put("constant", false).put("station", station.build());
    Ok(o.build())
}
