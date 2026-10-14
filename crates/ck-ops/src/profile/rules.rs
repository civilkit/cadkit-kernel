//! The rule catalogue. Every rule is `fn(&Ctx, &Params, Severity) -> Vec<Issue>`
//! and reports worst-first. Accuracy notes are in docs/spec-agent-interface.md.

use super::ctx::Ctx;
use super::{Issue, PKind, PVal, ParamDef, Params, RuleDef, Severity};
use crate::agent::dir_label;
use ck_core::linalg::Vec3;
use ck_geom::curve::Curve;
use ck_geom::surface::Surface;
use ck_topo::body::{EdgeId, FaceId, Role};
use std::collections::HashMap;
use std::f64::consts::TAU;

const fn num(name: &'static str, default: f64, unit: &'static str, min: Option<f64>, max: Option<f64>, doc: &'static str) -> ParamDef {
    ParamDef { name, kind: PKind::Num, default: PVal::Num(default), unit, min, max, doc }
}
const fn dir(name: &'static str, default: [f64; 3], doc: &'static str) -> ParamDef {
    ParamDef { name, kind: PKind::Vec3, default: PVal::Vec(default), unit: "", min: None, max: None, doc }
}

pub static RULES: &[RuleDef] = &[
    RuleDef {
        id: "openShell",
        title: "Watertight",
        doc: "The body must be a closed, consistently oriented shell: no open or non-manifold edges, no inside-out solids. Exact (topology and signed volume).",
        severity: Severity::Error,
        params: &[],
        run: open_shell,
    },
    RuleDef {
        id: "bodyCount",
        title: "Single solid",
        doc: "The part should be one solid; separate solids print / machine as separate pieces. Exact (lump count).",
        severity: Severity::Warn,
        params: &[num("max", 1.0, "", Some(1.0), None, "Most solids allowed")],
        run: body_count,
    },
    RuleDef {
        id: "minWall",
        title: "Minimum wall thickness",
        doc: "Local thickness: rays cast inward from surface samples to the opposite wall. Reports each pair of faces whose measured thickness is under the limit. Sampled (see opts.samples): a very thin sliver between samples can be missed.",
        severity: Severity::Error,
        params: &[num("min", 1.2, "mm", Some(0.0), None, "Thinnest wall allowed")],
        run: min_wall,
    },
    RuleDef {
        id: "overhang",
        title: "Overhang",
        doc: "Downward-facing surface steeper than the limit, measured from vertical (0 = wall, 90 = flat ceiling), against the build direction. Faces lying on the build plate are ignored. Mesh based: planar faces are exact, curved faces are measured per triangle.",
        severity: Severity::Warn,
        params: &[
            num("angle", 45.0, "deg", Some(0.0), Some(90.0), "Largest overhang printed without support, from vertical"),
            dir("direction", [0.0, 1.0, 0.0], "Build direction (up)"),
            num("minArea", 0.5, "mm2", Some(0.0), None, "Ignore overhanging patches smaller than this"),
        ],
        run: overhang,
    },
    RuleDef {
        id: "bridge",
        title: "Bridge span",
        doc: "Flat downward-facing faces (within flatTolerance of horizontal) above the plate: the span is the face's narrowest horizontal width (the slicer can bridge along the long direction). Estimate from the face triangulation.",
        severity: Severity::Warn,
        params: &[
            num("maxSpan", 10.0, "mm", Some(0.0), None, "Longest unsupported span"),
            dir("direction", [0.0, 1.0, 0.0], "Build direction (up)"),
            num("flatTolerance", 10.0, "deg", Some(0.0), Some(45.0), "How far from horizontal a face still counts as a bridge"),
        ],
        run: bridge,
    },
    RuleDef {
        id: "minFeature",
        title: "Minimum feature size",
        doc: "Hole and pin diameters from cylinder radii (exact), and gaps between parallel facing walls by outward rays (sampled).",
        severity: Severity::Warn,
        params: &[
            num("minDiameter", 0.8, "mm", Some(0.0), None, "Smallest hole or pin diameter"),
            num("minGap", 0.8, "mm", Some(0.0), None, "Narrowest gap between two facing walls"),
        ],
        run: min_feature,
    },
    RuleDef {
        id: "plateContact",
        title: "Build plate contact",
        doc: "Flat area touching the build plate (downward faces within about 6 degrees of horizontal at the lowest height). Warns when it is under the minimum.",
        severity: Severity::Warn,
        params: &[
            num("minArea", 20.0, "mm2", Some(0.0), None, "Smallest acceptable contact area"),
            dir("direction", [0.0, 1.0, 0.0], "Build direction (up)"),
        ],
        run: plate_contact,
    },
    RuleDef {
        id: "minInternalRadius",
        title: "Minimum internal radius",
        doc: "Vertical internal corners (parallel to the tool axis) smaller than the cutter radius: sharp concave edges are radius 0, concave cylinders give their exact radius. Drilled holes are not corners.",
        severity: Severity::Warn,
        params: &[
            num("minRadius", 1.0, "mm", Some(0.0), None, "Smallest cutter radius"),
            dir("direction", [0.0, 0.0, 1.0], "Tool axis (from the tool towards the part)"),
        ],
        run: min_internal_radius,
    },
    RuleDef {
        id: "undercut",
        title: "Undercut / tool access",
        doc: "Faces a tool moving along the direction cannot reach: facing away from it (exact per triangle) or hidden under another part of the body (rays, sampled). The stock underside is ignored.",
        severity: Severity::Error,
        params: &[dir("direction", [0.0, 0.0, 1.0], "Approach direction of the tool"), num("minArea", 0.5, "mm2", Some(0.0), None, "Ignore unreachable patches smaller than this")],
        run: undercut,
    },
    RuleDef {
        id: "draft",
        title: "Draft angle",
        doc: "Draft of each wall against the pull direction: asin(n . pull). Planes, cones and axis-parallel cylinders are exact; other faces use the area-weighted triangle average. A face is flagged when at least half of its area is under the limit.",
        severity: Severity::Warn,
        params: &[
            num("minAngle", 1.0, "deg", Some(0.0), Some(45.0), "Least draft allowed"),
            dir("direction", [0.0, 0.0, 1.0], "Mould pull direction"),
            num("minArea", 0.5, "mm2", Some(0.0), None, "Ignore faces smaller than this"),
        ],
        run: draft,
    },
    RuleDef {
        id: "uniformWall",
        title: "Uniform wall thickness",
        doc: "Wall thickness (as minWall) compared with the part's median: pairs of faces whose median thickness differs from it by more than the tolerance. Sampled.",
        severity: Severity::Warn,
        params: &[num("tolerance", 0.25, "", Some(0.0), Some(10.0), "Allowed deviation from the median thickness, as a fraction")],
        run: uniform_wall,
    },
];

// ---------------------------------------------------------------- helpers

fn f2(x: f64) -> String {
    format!("{:.2}", x)
}

fn unit_dir(d: Vec3) -> Vec3 {
    d.normalize()
}

fn names(cx: &Ctx, idx: &[usize]) -> Vec<String> {
    let mut out: Vec<String> = vec![];
    for &i in idx {
        let n = cx.names[i].clone();
        if !out.contains(&n) {
            out.push(n);
        }
    }
    out
}

/// Sum of triangle areas per face index.
fn face_areas(cx: &Ctx) -> Vec<f64> {
    let mut a = vec![0.0; cx.fids.len()];
    for t in cx.tris() {
        a[t.face] += t.area;
    }
    a
}

/// Triangles of the build plate: facing down and entirely at the lowest height.
fn on_plate(t: &super::ctx::Tri, up: Vec3, lo: f64, tol: f64) -> bool {
    t.n.dot(up) < -0.9 && [t.a, t.b, t.c].iter().all(|p| p.dot(up) <= lo + tol)
}

fn plate_tol(cx: &Ctx) -> f64 {
    1e-4 * cx.diag() + 1e-6
}

// ---------------------------------------------------------------- sanity

fn open_shell(cx: &Ctx, _p: &Params, sev: Severity) -> Vec<Issue> {
    use ck_topo::check::Issue as T;
    let mut open: Vec<EdgeId> = vec![];
    let mut nonman: Vec<EdgeId> = vec![];
    for i in ck_topo::check::check(cx.body, 1e-6) {
        match i {
            T::OpenEdge(e) => open.push(e),
            T::NonManifoldEdge(e, _) => nonman.push(e),
            _ => {}
        }
    }
    let mut out = vec![];
    for (list, what, advice) in [(&open, "open", "close the gap or cap the hole"), (&nonman, "non-manifold", "separate the bodies that share these edges")] {
        if list.is_empty() {
            continue;
        }
        let mut faces: Vec<String> = vec![];
        for &e in list.iter() {
            for f in cx.body.edge_faces(e) {
                let n = cx.body.face_id_str(f);
                if !faces.contains(&n) {
                    faces.push(n);
                }
            }
        }
        let edges: Vec<String> = list.iter().take(50).filter_map(|&e| cx.edge_key_of(e)).collect();
        let mut i = Issue::new("openShell", sev, format!("Not watertight: {} {} edge(s) on {} face(s); {}.", list.len(), what, faces.len(), advice));
        i.faces = faces;
        i.edges = edges;
        i.value = Some(list.len() as f64);
        i.limit = Some(0.0);
        i.unit = "edges";
        out.push(i);
    }
    let lumps = cx.lump_volumes();
    let bad: Vec<usize> = lumps.iter().enumerate().filter(|(_, v)| **v <= 0.0).map(|(i, _)| i).collect();
    if !bad.is_empty() {
        let ls: Vec<_> = cx.body.lump_ids().collect();
        let faces: Vec<String> = bad
            .iter()
            .filter_map(|&k| ck_topo::mass::lump_faces(cx.body, ls[k]).first().map(|&f| cx.body.face_id_str(f)))
            .collect();
        let mut i = Issue::new("openShell", sev, format!("{} solid(s) have zero or negative volume (inside-out or degenerate shell); reverse the faces or rebuild the feature.", bad.len()));
        i.faces = faces;
        i.value = Some(lumps[bad[0]]);
        i.limit = Some(0.0);
        i.unit = "mm3";
        out.push(i);
    }
    out
}

fn body_count(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let n = cx.body.num_lumps();
    let max = p.num("max");
    if (n as f64) <= max {
        return vec![];
    }
    let lumps = cx.lump_volumes();
    let ids: Vec<_> = cx.body.lump_ids().collect();
    let faces: Vec<String> = ids.iter().filter_map(|&l| ck_topo::mass::lump_faces(cx.body, l).first().map(|&f| cx.body.face_id_str(f))).collect();
    let vols: Vec<String> = lumps.iter().map(|v| f2(*v)).collect();
    let mut i = Issue::new("bodyCount", sev, format!("{} separate solids (volumes {} mm3), at most {} expected: unite them or print / machine them as separate parts.", n, vols.join(", "), max));
    i.faces = faces;
    i.value = Some(n as f64);
    i.limit = Some(max);
    i.unit = "solids";
    vec![i]
}

// ---------------------------------------------------------------- walls

/// Samples grouped by unordered face pair: (a, b) -> thicknesses.
fn wall_pairs(cx: &Ctx) -> Vec<((usize, usize), Vec<f64>, Vec3)> {
    let mut m: HashMap<(usize, usize), (Vec<f64>, Vec3, f64)> = HashMap::new();
    for w in cx.walls() {
        let k = (w.src.min(w.hit), w.src.max(w.hit));
        let e = m.entry(k).or_insert((vec![], w.p, f64::INFINITY));
        if w.t < e.2 {
            e.1 = w.p;
            e.2 = w.t;
        }
        e.0.push(w.t);
    }
    let mut v: Vec<_> = m.into_iter().map(|(k, (ts, p, _))| (k, ts, p)).collect();
    v.sort_by_key(|x| x.0);
    v
}

fn min_wall(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let lim = p.num("min");
    let mut out: Vec<(f64, Issue)> = vec![];
    for ((a, b), ts, at) in wall_pairs(cx) {
        let t = ts.iter().cloned().fold(f64::INFINITY, f64::min);
        if t >= lim {
            continue;
        }
        let nm = names(cx, &[a, b]);
        let mut i = Issue::new(
            "minWall",
            sev,
            format!("Wall between {} is {} mm thick, under the {} mm minimum: thicken it by {} mm.", nm.join(" and "), f2(t), f2(lim), f2(lim - t)),
        );
        i.faces = nm;
        i.value = Some(t);
        i.limit = Some(lim);
        i.unit = "mm";
        i.at = Some(at);
        out.push((t, i));
    }
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}

fn median(v: &mut Vec<f64>) -> f64 {
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    v[v.len() / 2]
}

fn uniform_wall(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let tol = p.num("tolerance");
    // A wall is a pair of faces much wider than they are far apart: the distance
    // must be at most half the square root of the smaller face area. This drops
    // the end faces of a plate (60 mm apart, 2 mm tall) from the comparison.
    let is_wall = |a: usize, b: usize, t: f64| {
        let area = |i: usize| cx.area(i);
        t <= 0.5 * area(a).min(area(b)).sqrt()
    };
    let pairs: Vec<_> = wall_pairs(cx)
        .into_iter()
        .map(|((a, b), mut ts, at)| {
            ts.retain(|&t| is_wall(a, b, t));
            ((a, b), ts, at)
        })
        .filter(|(_, ts, _)| !ts.is_empty())
        .collect();
    let mut all: Vec<f64> = pairs.iter().flat_map(|(_, ts, _)| ts.iter().copied()).collect();
    if all.is_empty() {
        return vec![];
    }
    let nominal = median(&mut all);
    let (lo, hi) = (nominal * (1.0 - tol), nominal * (1.0 + tol));
    let mut out: Vec<(f64, Issue)> = vec![];
    for ((a, b), mut ts, at) in pairs {
        let t = median(&mut ts);
        if t >= lo && t <= hi {
            continue;
        }
        let nm = names(cx, &[a, b]);
        let (bound, word) = if t > hi { (hi, "over") } else { (lo, "under") };
        let dev = (t - nominal).abs() / nominal * 100.0;
        let mut i = Issue::new(
            "uniformWall",
            sev,
            format!(
                "Wall between {} is {} mm thick versus a nominal {} mm ({:.0}% {}, allowed +-{:.0}%): {}.",
                nm.join(" and "),
                f2(t),
                f2(nominal),
                dev,
                word,
                tol * 100.0,
                if t > hi { "core it out or add ribs instead of mass, to avoid sinks and warping" } else { "thicken it or thin the rest, to avoid short shots" }
            ),
        );
        i.faces = nm;
        i.value = Some(t);
        i.limit = Some(bound);
        i.unit = "mm";
        i.at = Some(at);
        out.push(((t - nominal).abs(), i));
    }
    out.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}

// ---------------------------------------------------------------- FDM

fn overhang(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let up = unit_dir(p.dir("direction"));
    let lim = p.num("angle");
    let min_area = p.num("minArea");
    let (lo, _) = cx.height_range(up);
    let tol = plate_tol(cx);
    let n = cx.fids.len();
    let (mut over, mut worst) = (vec![0.0; n], vec![0.0f64; n]);
    for t in cx.tris() {
        let s = t.n.dot(up);
        if s >= 0.0 || on_plate(t, up, lo, tol) {
            continue;
        }
        let alpha = (-s).clamp(0.0, 1.0).asin().to_degrees();
        if alpha > lim + 1e-6 {
            over[t.face] += t.area;
            worst[t.face] = worst[t.face].max(alpha);
        }
    }
    let areas = face_areas(cx);
    let mut order: Vec<usize> = (0..n).filter(|&i| over[i] >= min_area.max(1e-9)).collect();
    order.sort_by(|&a, &b| over[b].partial_cmp(&over[a]).unwrap_or(std::cmp::Ordering::Equal));
    order
        .into_iter()
        .map(|f| {
            let mut i = Issue::new(
                "overhang",
                sev,
                format!(
                    "{} overhangs up to {} deg from vertical (limit {} deg) over {} of {} mm2 against build direction {}: add a chamfer or fillet of at most {} deg, reorient the part, or allow supports.",
                    cx.names[f], f2(worst[f]), f2(lim), f2(over[f]), f2(areas[f]), dir_label(up), f2(lim)
                ),
            );
            i.faces = vec![cx.names[f].clone()];
            i.value = Some(worst[f]);
            i.limit = Some(lim);
            i.unit = "deg";
            i.at = Some(cx.centroid(f));
            i
        })
        .collect()
}

fn bridge(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let up = unit_dir(p.dir("direction"));
    let max_span = p.num("maxSpan");
    let flat = p.num("flatTolerance").to_radians().cos();
    let (lo, _) = cx.height_range(up);
    let tol = plate_tol(cx);
    let n = cx.fids.len();
    let mut pts: Vec<Vec<Vec3>> = vec![vec![]; n];
    let mut area = vec![0.0; n];
    for t in cx.tris() {
        if t.n.dot(up) > -flat || on_plate(t, up, lo, tol) {
            continue;
        }
        pts[t.face].extend([t.a, t.b, t.c]);
        area[t.face] += t.area;
    }
    let u1 = up.any_perp().normalize();
    let u2 = up.cross(u1).normalize();
    let mut out: Vec<(f64, Issue)> = vec![];
    for f in 0..n {
        if pts[f].is_empty() || area[f] < 1.0 {
            continue;
        }
        // narrowest width over 90 directions in the horizontal plane
        let mut span = f64::INFINITY;
        for k in 0..90 {
            let a = std::f64::consts::PI * k as f64 / 90.0;
            let d = u1 * a.cos() + u2 * a.sin();
            let (mut mn, mut mx) = (f64::INFINITY, f64::NEG_INFINITY);
            for q in &pts[f] {
                let h = q.dot(d);
                mn = mn.min(h);
                mx = mx.max(h);
            }
            span = span.min(mx - mn);
        }
        if span <= max_span {
            continue;
        }
        let mut i = Issue::new(
            "bridge",
            sev,
            format!(
                "{} is a flat ceiling spanning {} mm unsupported (limit {} mm, {} mm2): shorten the span below {} mm, add a rib or pillar, or use supports.",
                cx.names[f], f2(span), f2(max_span), f2(area[f]), f2(max_span)
            ),
        );
        i.faces = vec![cx.names[f].clone()];
        i.value = Some(span);
        i.limit = Some(max_span);
        i.unit = "mm";
        i.at = Some(cx.centroid(f));
        out.push((span, i));
    }
    out.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}

/// A cylinder face whose boundary is only full circles (a barrel or a pin side).
fn full_cylinder(cx: &Ctx, f: FaceId) -> bool {
    let b = cx.body;
    let mut sweep = 0.0;
    for e in b.face_edges(f) {
        if b.is_seam(e) {
            continue;
        }
        let ed = b.edge(e);
        if !matches!(ed.curve, Curve::Circle { .. }) {
            return false;
        }
        sweep += (ed.t1 - ed.t0).abs();
    }
    sweep >= 2.0 * TAU - 1e-3
}

/// Material outside the cylinder (a bore, a pocket corner): the outward normal points at the axis.
fn is_concave_cylinder(cx: &Ctx, f: FaceId) -> bool {
    let b = cx.body;
    let face = b.face(f);
    let Surface::Cylinder { frame, .. } = &face.surface else { return false };
    let Some(&e) = b.face_edges(f).first() else { return false };
    let p = b.edge_mid(e);
    let (u, v) = face.surface.closest_param(p);
    let d = p - frame.o;
    b.outward_normal(f, u, v).dot(d - frame.z * d.dot(frame.z)) < 0.0
}

fn min_feature(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let (min_d, min_gap) = (p.num("minDiameter"), p.num("minGap"));
    let mut out: Vec<(f64, Issue)> = vec![];
    let mut covered = vec![false; cx.fids.len()];
    for fi in 0..cx.fids.len() {
        let face = cx.body.face(cx.fids[fi]);
        let Surface::Cylinder { r, .. } = &face.surface else { continue };
        let d = 2.0 * r;
        if d >= min_d || matches!(face.name.role, Role::Blend(_)) || !full_cylinder(cx, cx.fids[fi]) {
            continue;
        }
        let concave = is_concave_cylinder(cx, cx.fids[fi]);
        covered[fi] = true;
        let (what, fix) = if concave { ("Hole", "enlarge it or drill it after printing") } else { ("Pin", "thicken it or make it a separate part") };
        let mut i = Issue::new("minFeature", sev, format!("{} {} has diameter {} mm, under the {} mm minimum: {}.", what, cx.names[fi], f2(d), f2(min_d), fix));
        i.faces = vec![cx.names[fi].clone()];
        i.value = Some(d);
        i.limit = Some(min_d);
        i.unit = "mm";
        i.at = Some(cx.centroid(fi));
        out.push((d, i));
    }
    // gaps: rays leaving the surface that meet a wall facing back at them
    let tris = cx.tris();
    let eps = 1e-6 * cx.diag();
    let mut pairs: HashMap<(usize, usize), (f64, Vec3)> = HashMap::new();
    for s in cx.samples() {
        if covered[s.face] {
            continue;
        }
        if let Some((t, ti)) = cx.cast(s.p + s.n * eps, s.n, min_gap.max(1e-9) * 1.0001) {
            let h = &tris[ti];
            if h.n.dot(s.n) < -0.7 && h.face != s.face && !covered[h.face] && t > eps {
                let k = (s.face.min(h.face), s.face.max(h.face));
                let e = pairs.entry(k).or_insert((f64::INFINITY, s.p));
                if t < e.0 {
                    *e = (t, s.p);
                }
            }
        }
    }
    let mut gaps: Vec<_> = pairs.into_iter().collect();
    gaps.sort_by_key(|g| g.0);
    for ((a, b), (t, at)) in gaps {
        if t >= min_gap {
            continue;
        }
        let nm = names(cx, &[a, b]);
        let mut i = Issue::new("minFeature", sev, format!("Gap between {} is {} mm wide, under the {} mm minimum: widen it by {} mm or close it.", nm.join(" and "), f2(t), f2(min_gap), f2(min_gap - t)));
        i.faces = nm;
        i.value = Some(t);
        i.limit = Some(min_gap);
        i.unit = "mm";
        i.at = Some(at);
        out.push((t, i));
    }
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}

fn plate_contact(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let up = unit_dir(p.dir("direction"));
    let min_area = p.num("minArea");
    let (lo, _) = cx.height_range(up);
    let tol = plate_tol(cx);
    let mut area = vec![0.0; cx.fids.len()];
    for t in cx.tris() {
        if t.n.dot(up) < -0.995 && [t.a, t.b, t.c].iter().all(|q| q.dot(up) <= lo + tol) {
            area[t.face] += t.area;
        }
    }
    let total: f64 = area.iter().sum();
    if total >= min_area {
        return vec![];
    }
    let faces: Vec<String> = (0..area.len()).filter(|&i| area[i] > 0.0).map(|i| cx.names[i].clone()).collect();
    let msg = if faces.is_empty() {
        format!("No flat face rests on the build plate (build direction {}): the part will not stick without a brim or raft; orient a flat face down.", dir_label(up))
    } else {
        format!("Only {} mm2 of flat area touches the build plate (minimum {} mm2): enlarge the base, add a brim, or reorient.", f2(total), f2(min_area))
    };
    let mut i = Issue::new("plateContact", sev, msg);
    i.faces = faces;
    i.value = Some(total);
    i.limit = Some(min_area);
    i.unit = "mm2";
    vec![i]
}

// ---------------------------------------------------------------- CNC

/// Is the edge between its two faces a concave (internal) one? None when
/// it is not a two-face manifold edge. Returns (concave, angle between normals).
fn edge_concavity(cx: &Ctx, e: EdgeId) -> Option<(bool, f64)> {
    let b = cx.body;
    let cs = b.edge_coedges(e);
    if cs.len() != 2 {
        return None;
    }
    let (fa, fb) = (b.coedge_face(cs[0]), b.coedge_face(cs[1]));
    if fa == fb {
        return None;
    }
    let ed = b.edge(e);
    let tm = 0.5 * (ed.t0 + ed.t1);
    let pt = ed.curve.eval(tm);
    let tan = ed.curve.tangent(tm).normalize();
    let nrm = |f: FaceId| {
        let (u, v) = b.face(f).surface.closest_param(pt);
        b.outward_normal(f, u, v)
    };
    let (na, nb) = (nrm(fa), nrm(fb));
    let da = if b.coedge(cs[0]).rev { -tan } else { tan };
    let db = if b.coedge(cs[1]).rev { -tan } else { tan };
    // face interior lies to the left of the loop direction seen from outside
    let convex_a = nb.dot(na.cross(da)) < 0.0;
    let convex_b = na.dot(nb.cross(db)) < 0.0;
    if convex_a != convex_b {
        return None;
    }
    Some((!convex_a, na.dot(nb).clamp(-1.0, 1.0).acos()))
}

fn min_internal_radius(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let axis = unit_dir(p.dir("direction"));
    let min_r = p.num("minRadius");
    let cos5 = 5f64.to_radians().cos();
    let mut out: Vec<(f64, Issue)> = vec![];
    // exact radii of concave cylinders parallel to the tool axis
    for fi in 0..cx.fids.len() {
        let Surface::Cylinder { frame, r } = &cx.body.face(cx.fids[fi]).surface else { continue };
        if *r >= min_r || frame.z.dot(axis).abs() < cos5 || !is_concave_cylinder(cx, cx.fids[fi]) || full_cylinder(cx, cx.fids[fi]) {
            continue;
        }
        let mut i = Issue::new(
            "minInternalRadius",
            sev,
            format!("Internal corner {} has radius {} mm, under the {} mm cutter radius: enlarge it to at least {} mm.", cx.names[fi], f2(*r), f2(min_r), f2(min_r)),
        );
        i.faces = vec![cx.names[fi].clone()];
        i.value = Some(*r);
        i.limit = Some(min_r);
        i.unit = "mm";
        i.at = Some(cx.centroid(fi));
        out.push((*r, i));
    }
    // sharp concave edges along the tool axis: radius 0
    for ne in cx.edges().iter().filter(|_| min_r > 0.0) {
        let ed = cx.body.edge(ne.first);
        let Curve::Line { d, .. } = &ed.curve else { continue };
        if d.normalize().dot(axis).abs() < cos5 {
            continue;
        }
        let Some((true, ang)) = edge_concavity(cx, ne.first) else { continue };
        if ang < 5f64.to_radians() {
            continue; // tangent: a smooth blend, not a corner
        }
        let faces: Vec<String> = ne.faces.iter().map(|&f| cx.body.face_id_str(f)).collect();
        let mut i = Issue::new(
            "minInternalRadius",
            sev,
            format!("Sharp internal corner {} (radius 0) between {}: a {} mm radius cutter cannot reach it; add a fillet of at least {} mm or a corner relief.", ne.key, faces.join(" and "), f2(min_r), f2(min_r)),
        );
        i.faces = faces;
        i.edges = vec![ne.key.clone()];
        i.value = Some(0.0);
        i.limit = Some(min_r);
        i.unit = "mm";
        i.at = Some(cx.body.edge_mid(ne.first));
        out.push((0.0, i));
    }
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}

fn undercut(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let d = unit_dir(p.dir("direction"));
    let min_area = p.num("minArea");
    let n = cx.fids.len();
    // the stock underside is the lowest height along the approach direction
    let (lo, _) = cx.height_range(d);
    let tol = plate_tol(cx);
    let (mut away, mut up_area) = (vec![0.0; n], vec![0.0; n]);
    for t in cx.tris() {
        if t.n.dot(d) < -1e-3 {
            if !on_plate(t, d, lo, tol) {
                away[t.face] += t.area;
            }
        } else {
            up_area[t.face] += t.area;
        }
    }
    let eps = 1e-5 * cx.diag();
    let (mut seen, mut hidden) = (vec![0usize; n], vec![0usize; n]);
    for s in cx.samples() {
        if s.n.dot(d) < -1e-3 {
            continue;
        }
        seen[s.face] += 1;
        if cx.cast(s.p + (s.n + d) * eps, d, cx.diag() * 2.0).is_some() {
            hidden[s.face] += 1;
        }
    }
    let mut out: Vec<(f64, Issue)> = vec![];
    for f in 0..n {
        let occ = if seen[f] > 0 { hidden[f] as f64 / seen[f] as f64 * up_area[f] } else { 0.0 };
        let total = away[f] + occ;
        if total < min_area.max(1e-9) {
            continue;
        }
        let why = match (away[f] > 0.0, occ > 0.0) {
            (true, true) => format!("{} mm2 faces away from the tool and {} mm2 is hidden under other material", f2(away[f]), f2(occ)),
            (true, false) => format!("{} mm2 faces away from the tool", f2(away[f])),
            _ => format!("{} mm2 is hidden under other material", f2(occ)),
        };
        let mut i = Issue::new(
            "undercut",
            sev,
            format!("{} cannot be reached by a 3-axis tool approaching from {}: {}. Machine it from another setup, open the overhang, or remove the undercut.", cx.names[f], dir_label(d), why),
        );
        i.faces = vec![cx.names[f].clone()];
        i.value = Some(total);
        i.limit = Some(min_area);
        i.unit = "mm2";
        i.at = Some(cx.centroid(f));
        out.push((total, i));
    }
    out.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}

// ---------------------------------------------------------------- injection

fn draft(cx: &Ctx, p: &Params, sev: Severity) -> Vec<Issue> {
    let pull = unit_dir(p.dir("direction"));
    let min_a = p.num("minAngle");
    let min_area = p.num("minArea");
    let sin_min = min_a.to_radians().sin();
    let n = cx.fids.len();
    let (mut area, mut flat, mut wsum) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
    for t in cx.tris() {
        let s = t.n.dot(pull).abs().min(1.0);
        area[t.face] += t.area;
        wsum[t.face] += t.area * s.asin().to_degrees();
        if s < sin_min {
            flat[t.face] += t.area;
        }
    }
    let mut out: Vec<(f64, Issue)> = vec![];
    for f in 0..n {
        if area[f] < min_area.max(1e-9) || flat[f] < 0.5 * area[f] {
            continue;
        }
        let mean = wsum[f] / area[f];
        let mut i = Issue::new(
            "draft",
            sev,
            format!(
                "{} has {} deg of draft against pull direction {} (minimum {} deg) over {} mm2: tilt it by at least {} deg more, or split the mould at a parting line here.",
                cx.names[f], f2(mean), dir_label(pull), f2(min_a), f2(area[f]), f2(min_a - mean)
            ),
        );
        i.faces = vec![cx.names[f].clone()];
        i.value = Some(mean);
        i.limit = Some(min_a);
        i.unit = "deg";
        i.at = Some(cx.centroid(f));
        out.push((area[f], i));
    }
    out.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));
    out.into_iter().map(|x| x.1).collect()
}
