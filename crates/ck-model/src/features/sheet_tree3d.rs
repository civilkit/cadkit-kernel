//! Sheet metal, stage 3: the folded solid of a panel tree (docs/sheet-metal.md, "Stage 3").
//!
//! Every panel is the extrusion of its flat polygon by the thickness; every bend is the extrusion of the section of a bend of the
//! panel it hangs on (`sheet_features::strip`). Panels and bends are built in the flat layout and moved to their folded place by
//! the chain of bend transforms from the root. Roles (`P<n>.front`, `P<n>.side.<tag>`, `B<n>.front`, ...) are made from the same
//! corner points.

use super::sheet::{Rule, P};
use super::sheet_features::{strip, unsupported, xform_role, Model, Sec, Step};
use super::sheet_roles::{plane_role, RoleGeo};
use super::sheet_tree::{Flat3, Poly, TBend, Tree};
use crate::regen::FeatureError;
use ck_core::linalg::{Frame, Mat3, Vec3};
use ck_core::xform::Xform;
use ck_geom::Vec2;
use ck_topo::body::Body;
use ck_topo::make::{ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

fn v2(p: P) -> Vec2 {
    Vec2::new(p[0], p[1])
}

pub fn frame_of(fl: &Flat3) -> Frame {
    Frame { o: fl.o, x: fl.x, y: fl.y, z: fl.x.cross(fl.y) }
}

pub fn ext_opts(fl: &Flat3, t: f64) -> ExtrudeOpts {
    if fl.n.dot(fl.x.cross(fl.y)) > 0.0 {
        ExtrudeOpts::blind(t)
    } else {
        ExtrudeOpts::flipped(t)
    }
}

/// The section frame of a bend: `x` back over the parent, `y` the side the bend turns to, along the bend line `d3`.
pub fn bend_sec(fl: &Flat3, t: f64, b: &TBend) -> Sec {
    let nu3 = fl.dir(b.nu);
    let d3 = fl.dir(b.d);
    let h_axis = fl.n * if b.up { 1.0 } else { -1.0 };
    let o3 = fl.point(b.o, if b.up { 0.0 } else { t });
    Sec { o: o3, xs: -nu3, ys: h_axis, d3 }
}

/// The rigid motion that takes the flat far side of a bend to its folded place, in the frame of the parent.
pub fn bend_xform(fl: &Flat3, t: f64, b: &TBend) -> Xform {
    let sec = bend_sec(fl, t, b);
    let (r, a) = (b.radius, b.angle);
    let bb = -b.z[0];
    let c = Vec2::new(bb, r + t);
    let (s, co) = a.sin_cos();
    let e_end = c + Vec2::new(-s, -co) * (r + t);
    let (w2, n2) = (Vec2::new(-co, s), Vec2::new(s, co));
    let zs = sec.zs();
    let dir3 = |v: Vec2| sec.xs * v.x + sec.ys * v.y;
    let p0 = sec.pt(Vec2::new(-b.z[1], 0.0), 0.0);
    let e3 = sec.pt(e_end, 0.0);
    let flat = Mat3::from_cols(dir3(Vec2::new(-1.0, 0.0)), dir3(Vec2::new(0.0, 1.0)), zs);
    let folded = Mat3::from_cols(dir3(w2), dir3(n2), zs);
    let rot = folded.mul_mat(&flat.transpose());
    Xform::new(rot, e3 - rot.mul_vec(p0))
}

/// The transform of every panel from the flat layout to the folded solid.
pub fn xforms(tree: &Tree, t: f64) -> Vec<Xform> {
    fn go(tree: &Tree, t: f64, i: usize, memo: &mut Vec<Option<Xform>>) -> Xform {
        if let Some(x) = memo[i] {
            return x;
        }
        let x = match tree.panels[i].via {
            None => Xform::IDENTITY,
            Some(k) => {
                let b = &tree.bends[k];
                go(tree, t, b.parent, memo).compose(&bend_xform(&tree.flat, t, b))
            }
        };
        memo[i] = Some(x);
        x
    }
    let mut memo = vec![None; tree.panels.len()];
    (0..tree.panels.len()).map(|i| go(tree, t, i, &mut memo)).collect()
}

fn circumcenter(a: P, b: P, c: P) -> Option<P> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() < 1e-12 {
        return None;
    }
    let (a2, b2, c2) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
    Some([(a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d, (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d])
}

/// The profile loop of a panel ring: lines, and true arcs for the runs of edges tagged `ra<n>` (round corner reliefs).
pub fn ring_loop(poly: &Poly) -> ProfileLoop {
    let n = poly.len();
    let arc = |i: usize| poly[i].1.starts_with("ra");
    // a group starts where the previous edge has another tag
    let start = (0..n).find(|&i| !(arc(i) && arc((i + n - 1) % n) && poly[i].1 == poly[(i + n - 1) % n].1)).unwrap_or(0);
    let mut segs = vec![];
    let mut count = 0;
    let mut i = start;
    while count < n {
        if arc(i) {
            let mut m = 1;
            while count + m < n && arc((i + m) % n) && poly[(i + m) % n].1 == poly[i].1 {
                m += 1;
            }
            let (pa, pm, pb) = (poly[i].0, poly[(i + m / 2) % n].0, poly[(i + m) % n].0);
            match circumcenter(pa, pm, pb) {
                Some(c) => {
                    let ccw = (pm[0] - pa[0]) * (pb[1] - pm[1]) - (pm[1] - pa[1]) * (pb[0] - pm[0]) > 0.0;
                    let tag = poly[i].1.as_str();
                    if ccw {
                        segs.push(ProfileSeg::arc(tag, v2(c), v2(pa), v2(pb)));
                    } else {
                        segs.push(ProfileSeg::arc(tag, v2(c), v2(pb), v2(pa)).reversed());
                    }
                }
                None => {
                    for q in 0..m {
                        segs.push(ProfileSeg::line(&poly[i].1, v2(poly[(i + q) % n].0), v2(poly[(i + q + 1) % n].0)));
                    }
                }
            }
            i = (i + m) % n;
            count += m;
        } else {
            segs.push(ProfileSeg::line(&poly[i].1, v2(poly[i].0), v2(poly[(i + 1) % n].0)));
            i = (i + 1) % n;
            count += 1;
        }
    }
    ProfileLoop::new(segs)
}

/// A hole: a full circle when its points are equally spaced on one, else a polygon.
pub fn hole_loop(h: &[P], tag: &str) -> ProfileLoop {
    let n = h.len();
    if n >= 12 {
        let c = [h.iter().map(|p| p[0]).sum::<f64>() / n as f64, h.iter().map(|p| p[1]).sum::<f64>() / n as f64];
        let r = (h[0][0] - c[0]).hypot(h[0][1] - c[1]);
        let round = r > 1e-9 && h.iter().all(|p| ((p[0] - c[0]).hypot(p[1] - c[1]) - r).abs() < 1e-7 * r.max(1.0));
        let step = |a: P, b: P| {
            let (u, v) = ([a[0] - c[0], a[1] - c[1]], [b[0] - c[0], b[1] - c[1]]);
            (u[0] * v[1] - u[1] * v[0]).atan2(u[0] * v[0] + u[1] * v[1])
        };
        let s0 = step(h[0], h[1]);
        let even = (0..n).all(|i| (step(h[i], h[(i + 1) % n]) - s0).abs() < 1e-6);
        if round && even {
            return ProfileLoop::new(vec![ProfileSeg::circle(tag, v2(c), r)]);
        }
    }
    ProfileLoop::new((0..n).map(|i| ProfileSeg::line(&format!("{tag}{i}"), v2(h[i]), v2(h[(i + 1) % n]))).collect())
}

fn panel_body(tree: &Tree, i: usize, t: f64, id: &str) -> Result<Body, FeatureError> {
    let pn = &tree.panels[i];
    let holes: Vec<ProfileLoop> = pn.holes.iter().enumerate().map(|(k, h)| hole_loop(h, &format!("h{k}"))).collect();
    let reg = ProfileRegion { outer: ring_loop(&pn.poly), holes };
    let prof = Profile::new(frame_of(&tree.flat), vec![reg]).normalized();
    ck_topo::make::extrude(id, &prof, ext_opts(&tree.flat, t)).map_err(|e| FeatureError::new(&e))
}

/// The solid pieces of the tree, in an order in which every piece touches the ones before it, with their roles.
pub fn parts(m: &Model, tree: &Tree, roles: &mut Vec<RoleGeo>) -> Result<Vec<Body>, FeatureError> {
    let t = m.sheet.rule.thickness;
    let fl = &tree.flat;
    let xfs = xforms(tree, t);
    // walk: root, then for every bend (in order) the bend and the panel behind it
    enum Item {
        Panel(usize),
        Bend(usize),
    }
    fn visit(tree: &Tree, i: usize, out: &mut Vec<Item>) {
        out.push(Item::Panel(i));
        for (k, b) in tree.bends.iter().enumerate() {
            if b.parent == i {
                out.push(Item::Bend(k));
                visit(tree, b.child, out);
            }
        }
    }
    let mut items = vec![];
    for (i, p) in tree.panels.iter().enumerate() {
        if p.via.is_none() {
            visit(tree, i, &mut items);
        }
    }
    let mut out = vec![];
    for it in items {
        match it {
            Item::Panel(i) => {
                let pn = &tree.panels[i];
                if pn.area() < 1e-9 {
                    continue;
                }
                let mut body = panel_body(tree, i, t, &format!("{}.p{}", m.sheet.base, i))?;
                body.transform(&xfs[i]);
                out.push(body);
                let nn = pn.poly.len();
                let world = |p: P, h: f64| xfs[i].apply_point(fl.point(p, h));
                for (name, h) in [("front", 0.0), ("back", t)] {
                    let pts: Vec<Vec3> = pn.poly.iter().map(|(p, _)| world(*p, h)).collect();
                    roles.extend(plane_role(&format!("P{i}.{name}"), &pts));
                }
                for j in 0..nn {
                    let tag = &pn.poly[j].1;
                    if Tree::is_zone_tag(tag) || tag.starts_with("ra") || tag.starts_with("rl") {
                        continue;
                    }
                    let (a, b2) = (pn.poly[j].0, pn.poly[(j + 1) % nn].0);
                    let pts = [world(a, 0.0), world(b2, 0.0), world(b2, t), world(a, t)];
                    let name = if tag == "tip" { format!("P{i}.tip") } else { format!("P{i}.side.{tag}") };
                    roles.extend(plane_role(&name, &pts));
                }
            }
            Item::Bend(k) => {
                let b = &tree.bends[k];
                let sec = bend_sec(fl, t, b);
                let lo = (b.v0[0] + b.v1[0]) * 0.5;
                let hi = (b.v0[1] + b.v1[1]) * 0.5;
                if hi - lo < 1e-9 {
                    continue;
                }
                let step = Step { angle: b.angle, flip: false, leg: 0.0, bend_along: [lo, hi], leg_along: [lo, hi] };
                let id = |c: char, _j: usize| format!("{}.{}{}", b.feature, c, k);
                let mut sub_roles = vec![];
                let (mut bodies, _) = strip(&sec, &id, -b.z[0], b.radius, t, &[step], k + 1, 0, b.up, &mut sub_roles)?;
                for bd in bodies.iter_mut() {
                    bd.transform(&xfs[b.parent]);
                }
                for r in sub_roles {
                    roles.push(xform_role(&xfs[b.parent], r));
                }
                out.append(&mut bodies);
            }
        }
    }
    if out.is_empty() {
        return Err(unsupported("the sheet has no material", "check the sheet features"));
    }
    Ok(out)
}

/// The flat sheet: the flat pattern extruded by the thickness.
pub fn slab(m: &Model, tree: &Tree, rule: &Rule) -> Result<Body, FeatureError> {
    let fp = tree.flat_pattern(rule, true);
    let loops: Vec<ProfileLoop> = fp
        .outline
        .iter()
        .enumerate()
        .map(|(k, l)| {
            let n = l.len();
            ProfileLoop::new((0..n).map(|i| ProfileSeg::line(&format!("u{k}_{i}"), v2(l[i]), v2(l[(i + 1) % n]))).collect())
        })
        .collect();
    let mut it = loops.into_iter();
    let outer = it.next().ok_or_else(|| unsupported("empty flat pattern", "check the sheet"))?;
    let reg = ProfileRegion { outer, holes: it.collect() };
    let prof = Profile::new(frame_of(&tree.flat), vec![reg]).normalized();
    ck_topo::make::extrude(&format!("{}.flat", m.sheet.base), &prof, ext_opts(&tree.flat, rule.thickness)).map_err(|e| FeatureError::new(&e))
}

/// The panel and flat position of a world point lying on the folded sheet (within half a thickness of a panel's mid-surface
/// and inside its polygon, `tol` mm outside it allowed).
pub fn locate(tree: &Tree, xfs: &[Xform], t: f64, w: Vec3) -> Option<(usize, P)> {
    let mut best: Option<(f64, usize, P)> = None;
    for (i, pn) in tree.panels.iter().enumerate() {
        if pn.area() < 1e-9 {
            continue;
        }
        let local = xfs[i].inverse_rigid().apply_point(w);
        let (p, h) = tree.flat.flat_of(local);
        if h < -0.75 * t || h > 1.75 * t {
            continue;
        }
        let ring = super::sheet_tree::pts_of(&pn.poly);
        if super::sheet_tree::inside(&ring, p, 1e-6) {
            let score = (h - 0.5 * t).abs();
            if best.map_or(true, |b| score < b.0) {
                best = Some((score, i, p));
            }
        }
    }
    best.map(|b| (b.1, b.2))
}

/// The flat position of a world point on the folded panel `i` (its place before the panel was moved by the bends).
pub fn flat_on(tree: &Tree, xfs: &[Xform], i: usize, w: Vec3) -> P {
    let local = xfs[i].inverse_rigid().apply_point(w);
    tree.flat.flat_of(local).0
}

/// The wall of a panel nearest to a world point, and whether the point is on the cap (back) side of that panel.
pub fn nearest_wall(tree: &Tree, xfs: &[Xform], t: f64, w: Vec3) -> Option<(usize, usize, bool)> {
    let mut best: Option<(f64, usize, usize, bool)> = None;
    for (i, pn) in tree.panels.iter().enumerate() {
        if pn.area() < 1e-9 {
            continue;
        }
        let n = pn.poly.len();
        let nrm = xfs[i].apply_vec(tree.flat.n);
        for j in 0..n {
            let tag = &pn.poly[j].1;
            if Tree::is_zone_tag(tag) || tag.starts_with("ra") || tag.starts_with("rl") {
                continue;
            }
            let a = xfs[i].apply_point(tree.flat.point(pn.poly[j].0, t * 0.5));
            let b = xfs[i].apply_point(tree.flat.point(pn.poly[(j + 1) % n].0, t * 0.5));
            let ab = b - a;
            let l2 = ab.dot(ab);
            let s = if l2 < 1e-24 { 0.0 } else { ((w - a).dot(ab) / l2).clamp(0.0, 1.0) };
            let c = a + ab * s;
            let dist = w.dist(c);
            if best.map_or(true, |b| dist < b.0 - 1e-9) {
                best = Some((dist, i, j, (w - c).dot(nrm) >= -1e-9));
            }
        }
    }
    best.map(|b| (b.1, b.2, b.3))
}

/// The tools that cut the bends: for every cut that crosses a bend zone, the wedges of the bend (sectors about its axis, extruded
/// along it) that the cut maps to. The cut is a region of the flat pattern; `u` across the zone is the angle `u / BA * a`, so a
/// region bounded by lines along and across the bend is one exact wedge and any other boundary is a staircase of wedges (slices
/// 0.25 mm of zone wide).
pub fn cut_tools(m: &Model, tree: &Tree) -> Result<Vec<Body>, FeatureError> {
    let t = m.sheet.rule.thickness;
    let xfs = xforms(tree, t);
    let mut out = vec![];
    for (k, b) in tree.bends.iter().enumerate() {
        let (lo, hi) = ((b.v0[0] + b.v1[0]) * 0.5, (b.v0[1] + b.v1[1]) * 0.5);
        let (u0, u1) = (b.z[0], b.z[1]);
        if u1 - u0 < 1e-9 {
            continue;
        }
        for (ci, region) in tree.cuts.iter().enumerate() {
            // the region in the (u, v) coordinates of the bend
            let q: Vec<P> = region.iter().map(|p| [(p[0] - b.o[0]) * b.nu[0] + (p[1] - b.o[1]) * b.nu[1], (p[0] - b.o[0]) * b.d[0] + (p[1] - b.o[1]) * b.d[1]]).collect();
            let (qmin_u, qmax_u) = q.iter().fold((f64::MAX, f64::MIN), |a, p| (a.0.min(p[0]), a.1.max(p[0])));
            if qmax_u <= u0 + 1e-9 || qmin_u >= u1 - 1e-9 {
                continue;
            }
            let n = (((u1 - u0) / 0.25).ceil() as usize).clamp(1, 60);
            let du = (u1 - u0) / n as f64;
            // slices: the v intervals of the region at the middle of each
            let mut slabs: Vec<(f64, f64, Vec<(f64, f64)>)> = vec![];
            for i in 0..n {
                let um = u0 + (i as f64 + 0.5) * du;
                let mut vs: Vec<f64> = vec![];
                let m_ = q.len();
                for j in 0..m_ {
                    let (a, c) = (q[j], q[(j + 1) % m_]);
                    if (a[0] - um) * (c[0] - um) < 0.0 {
                        vs.push(a[1] + (c[1] - a[1]) * (um - a[0]) / (c[0] - a[0]));
                    }
                }
                vs.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
                let iv: Vec<(f64, f64)> = vs.chunks(2).filter(|c| c.len() == 2).map(|c| (c[0].max(lo), c[1].min(hi))).filter(|c| c.1 - c.0 > 1e-6).collect();
                if iv.is_empty() {
                    continue;
                }
                let (ua, ub) = (u0 + i as f64 * du, u0 + (i + 1) as f64 * du);
                match slabs.last_mut() {
                    Some(last) if (last.1 - ua).abs() < 1e-9 && last.2.len() == iv.len() && last.2.iter().zip(&iv).all(|(a, b2)| (a.0 - b2.0).abs() < 1e-6 && (a.1 - b2.1).abs() < 1e-6) => last.1 = ub,
                    _ => slabs.push((ua, ub, iv)),
                }
            }
            let sec = bend_sec(&tree.flat, t, b);
            let (r, ang) = (b.radius, b.angle);
            let c = Vec2::new(-b.z[0], r + t);
            for (si, (ua, ub, ivs)) in slabs.iter().enumerate() {
                let (ta, tb) = ((ua - u0) / (u1 - u0) * ang, (ub - u0) / (u1 - u0) * ang);
                let steps = (((tb - ta) / (std::f64::consts::PI / 6.0)).ceil() as usize).max(1);
                let step = (tb - ta) / steps as f64;
                let rr = (r + t + 2.0) / (step * 0.5).cos().max(0.2);
                let mut pts = vec![c];
                for s in 0..=steps {
                    let th = ta + step * s as f64;
                    pts.push(c + Vec2::new(-th.sin(), -th.cos()) * rr);
                }
                let nn = pts.len();
                let segs: Vec<ProfileSeg> = (0..nn).map(|i| ProfileSeg::line(&format!("w{i}"), pts[i], pts[(i + 1) % nn])).collect();
                let prof = Profile::new(sec.frame(), vec![ProfileRegion { outer: ProfileLoop::new(segs), holes: vec![] }]).normalized();
                for (ii, (va, vb)) in ivs.iter().enumerate() {
                    let mut tool = ck_topo::make::extrude(&format!("cut{ci}.b{k}.{si}.{ii}"), &prof, sec.span(*va, *vb)).map_err(|e| FeatureError::new(&e))?;
                    tool.transform(&xfs[b.parent]);
                    out.push(tool);
                }
            }
        }
    }
    Ok(out)
}
