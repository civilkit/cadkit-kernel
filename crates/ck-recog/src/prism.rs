//! Prismatic base: side walls parallel to a principal axis form closed 2D cycles (the sketch profile); cap
//! planes at the two extremes; fillets / chamfers between a cap and a wall are recognised and left out of the
//! profile.

use crate::model::*;
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_topo::body::{EdgeId, FaceId};
use std::collections::{HashMap, HashSet};
use std::f64::consts::TAU;

#[derive(Debug, Clone)]
pub enum Geom {
    Line,
    Arc { c: (f64, f64), ccw: bool },
}

#[derive(Debug, Clone)]
pub struct Seg {
    pub a: (f64, f64),
    pub b: (f64, f64),
    pub g: Geom,
    pub face: FaceId,
}

#[derive(Debug, Clone)]
pub enum Cycle {
    Poly(Vec<Seg>),
    Circle { c: (f64, f64), r: f64, face: FaceId },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlendKind {
    Fillet(f64),
    Chamfer(f64),
}

#[derive(Debug, Clone)]
pub struct Blend {
    pub faces: Vec<FaceId>,
    pub kind: BlendKind,
    pub cap: FaceId,
    pub wall: FaceId,
    /// A point on the sharp edge (the wall rail moved to the cap level).
    pub at: Vec3,
}

pub struct Prism {
    pub ai: usize,
    pub spec: PlaneSpec,
    pub zmin: f64,
    pub zmax: f64,
    pub cycles: Vec<Cycle>,
    pub top: Option<FaceId>,
    pub bottom: Option<FaceId>,
    pub walls: Vec<FaceId>,
    pub blends: Vec<Blend>,
    pub leftover: Vec<FaceId>,
}

fn ang(c: (f64, f64), p: (f64, f64)) -> f64 {
    (p.1 - c.1).atan2(p.0 - c.0)
}

fn wrap(a: f64) -> f64 {
    let r = a % TAU;
    if r < 0.0 {
        r + TAU
    } else {
        r
    }
}

fn seg_points(s: &Seg, out: &mut Vec<(f64, f64)>) {
    out.push(s.a);
    if let Geom::Arc { c, ccw } = &s.g {
        let r = ((s.a.0 - c.0).powi(2) + (s.a.1 - c.1).powi(2)).sqrt();
        let (ta, tb) = (ang(*c, s.a), ang(*c, s.b));
        let mut sweep = if *ccw { wrap(tb - ta) } else { -wrap(ta - tb) };
        if sweep.abs() < 1e-12 {
            sweep = if *ccw { TAU } else { -TAU };
        }
        for i in 1..8 {
            let t = ta + sweep * i as f64 / 8.0;
            out.push((c.0 + r * t.cos(), c.1 + r * t.sin()));
        }
    }
}

fn area(segs: &[Seg]) -> f64 {
    let mut pts = vec![];
    for s in segs {
        seg_points(s, &mut pts);
    }
    let mut a = 0.0;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        a += p.0 * q.1 - q.0 * p.1;
    }
    a / 2.0
}

fn reverse(segs: &mut Vec<Seg>) {
    segs.reverse();
    for s in segs.iter_mut() {
        std::mem::swap(&mut s.a, &mut s.b);
        if let Geom::Arc { ccw, .. } = &mut s.g {
            *ccw = !*ccw;
        }
    }
}

pub fn is_wall(k: &Kind, nv: Vec3) -> bool {
    match k {
        Kind::Plane { n, .. } => n.dot(nv).abs() < 1e-7,
        Kind::Cyl { a, .. } => par(*a, nv),
        _ => false,
    }
}

fn normal_at(m: &Model, f: FaceId, p: Vec3) -> Vec3 {
    let (u, v) = m.b.face(f).surface.closest_param(p);
    m.b.outward_normal(f, u, v)
}

pub fn analyze(m: &Model, claimed: &HashSet<FaceId>, ai: usize) -> Option<Prism> {
    let spec = PlaneSpec::for_axis(ai);
    let nv = spec.zs;
    let (mut zmin, mut zmax) = (f64::INFINITY, f64::NEG_INFINITY);
    for e in m.b.edge_ids() {
        for p in [m.p0(e), m.p1(e)] {
            zmin = zmin.min(p.dot(nv));
            zmax = zmax.max(p.dot(nv));
        }
    }
    if !(zmax - zmin > 1e-6) {
        return None;
    }
    let mut role: HashMap<FaceId, u8> = HashMap::new(); // 1 wall, 2 top, 3 bottom
    let mut top = None;
    let mut bottom = None;
    let mut walls = vec![];
    for &f in &m.faces {
        if claimed.contains(&f) {
            continue;
        }
        let k = m.kind(f);
        if is_wall(k, nv) {
            role.insert(f, 1);
            walls.push(f);
        } else if let Kind::Plane { n, d } = k {
            let t = n.dot(nv);
            if t > 1.0 - 1e-7 && (d - zmax).abs() < 1e-5 && top.is_none() {
                role.insert(f, 2);
                top = Some(f);
            } else if t < -1.0 + 1e-7 && (-d - zmin).abs() < 1e-5 && bottom.is_none() {
                role.insert(f, 3);
                bottom = Some(f);
            }
        }
    }
    if walls.is_empty() {
        return None;
    }
    // blends
    let mut blends: Vec<Blend> = vec![];
    let mut blended: HashSet<FaceId> = HashSet::new();
    for &f in &m.faces {
        if claimed.contains(&f) || role.contains_key(&f) {
            continue;
        }
        if let Some(b) = detect_blend(m, f, nv, zmax, zmin, &role) {
            blended.insert(f);
            blends.push(b);
        }
    }
    // corner patches: spheres touching only blend strips of the same radius
    for &f in &m.faces {
        if claimed.contains(&f) || role.contains_key(&f) || blended.contains(&f) {
            continue;
        }
        if let Kind::Sphere { r, .. } = m.kind(f) {
            let nb: Vec<FaceId> = m.b.face_neighbors(f);
            let ok = !nb.is_empty()
                && nb.iter().all(|g| {
                    blends.iter().any(|b| b.faces.contains(g) && matches!(b.kind, BlendKind::Fillet(rr) if (rr - r).abs() < 1e-6))
                        || blended.contains(g)
                });
            if ok {
                blended.insert(f);
                if let Some(b) = blends.iter_mut().find(|b| nb.iter().any(|g| b.faces.contains(g))) {
                    b.faces.push(f);
                }
            }
        }
    }
    let leftover: Vec<FaceId> = m
        .faces
        .iter()
        .copied()
        .filter(|f| !claimed.contains(f) && !role.contains_key(f) && !blended.contains(f))
        .collect();

    // wall cycles
    let vertical = |e: EdgeId| matches!(m.curve(e), Curve::Line { d, .. } if par(*d, nv));
    let vedges = |w: FaceId| -> Vec<EdgeId> {
        m.edges(w)
            .into_iter()
            .filter(|&e| vertical(e) && m.across(e, w).map_or(false, |g| role.get(&g) == Some(&1)))
            .collect()
    };
    let pos = |e: EdgeId| spec.xy(m.p0(e));
    let mut visited: HashSet<FaceId> = HashSet::new();
    let mut cycles: Vec<Cycle> = vec![];
    for &w in &walls {
        if visited.contains(&w) {
            continue;
        }
        let ves = vedges(w);
        if ves.is_empty() {
            let Kind::Cyl { o, r, .. } = m.kind(w) else { return None };
            visited.insert(w);
            cycles.push(Cycle::Circle { c: spec.xy(*o), r: *r, face: w });
            continue;
        }
        if ves.len() != 2 {
            return None;
        }
        let mut segs: Vec<Seg> = vec![];
        let mut cur = w;
        let mut enter = ves[0];
        loop {
            visited.insert(cur);
            let v = vedges(cur);
            if v.len() != 2 {
                return None;
            }
            let exit = if v[0] == enter { v[1] } else { v[0] };
            let (a, b) = (pos(enter), pos(exit));
            let g = match m.kind(cur) {
                Kind::Plane { .. } => Geom::Line,
                Kind::Cyl { o, .. } => {
                    let c = spec.xy(*o);
                    // a horizontal arc edge of the wall tells which side of the chord the arc is on
                    let mid = m.edges(cur).into_iter().find_map(|e| match m.curve(e) {
                        Curve::Circle { frame, .. } if par(frame.z, nv) && !vertical(e) => {
                            let ed = m.b.edge(e);
                            Some(spec.xy(m.curve(e).eval(0.5 * (ed.t0 + ed.t1))))
                        }
                        _ => None,
                    })?;
                    let (ta, tb, tm) = (ang(c, a), ang(c, b), ang(c, mid));
                    let ccw = wrap(tm - ta) < wrap(tb - ta);
                    Geom::Arc { c, ccw }
                }
                _ => return None,
            };
            segs.push(Seg { a, b, g, face: cur });
            let Some(next) = m.across(exit, cur) else { return None };
            enter = exit;
            cur = next;
            if cur == w {
                break;
            }
            if visited.contains(&cur) || segs.len() > walls.len() {
                return None;
            }
        }
        cycles.push(Cycle::Poly(segs));
    }
    // orientation: largest cycle counter-clockwise, the others clockwise
    let area_of = |c: &Cycle| match c {
        Cycle::Poly(s) => area(s),
        Cycle::Circle { r, .. } => std::f64::consts::PI * r * r,
    };
    let big = (0..cycles.len()).max_by(|&i, &j| area_of(&cycles[i]).abs().partial_cmp(&area_of(&cycles[j]).abs()).unwrap())?;
    for (i, c) in cycles.iter_mut().enumerate() {
        if let Cycle::Poly(s) = c {
            let a = area(s);
            if (i == big && a < 0.0) || (i != big && a > 0.0) {
                reverse(s);
            }
        }
    }
    // the outer cycle first
    cycles.swap(0, big);
    Some(Prism { ai, spec, zmin, zmax, cycles, top, bottom, walls, blends, leftover })
}

fn detect_blend(m: &Model, f: FaceId, nv: Vec3, zmax: f64, zmin: f64, role: &HashMap<FaceId, u8>) -> Option<Blend> {
    let rail_tol = 1e-4;
    let all = m.edges(f);
    // rails per neighbour
    let mut cap: Option<(FaceId, EdgeId)> = None;
    let mut wall: Option<(FaceId, EdgeId)> = None;
    let mut kind = None;
    let mut cap_wall_dist = (0.0, 0.0);
    match m.kind(f) {
        Kind::Cyl { a, r, .. } if a.dot(nv).abs() < 1e-7 => {
            for &e in &all {
                if let Curve::Line { d, .. } = m.curve(e) {
                    if par(*d, *a) {
                        let g = m.across(e, f)?;
                        match role.get(&g) {
                            Some(2) | Some(3) => cap = cap.or(Some((g, e))),
                            Some(1) if matches!(m.kind(g), Kind::Plane { .. }) => wall = wall.or(Some((g, e))),
                            _ => {}
                        }
                    }
                }
            }
            kind = Some(BlendKind::Fillet(*r));
        }
        Kind::Torus { a, minor, .. } if par(*a, nv) => {
            for &e in &all {
                if matches!(m.curve(e), Curve::Circle { .. }) {
                    let g = m.across(e, f)?;
                    match role.get(&g) {
                        Some(2) | Some(3) => cap = cap.or(Some((g, e))),
                        Some(1) if matches!(m.kind(g), Kind::Cyl { .. }) => wall = wall.or(Some((g, e))),
                        _ => {}
                    }
                }
            }
            kind = Some(BlendKind::Fillet(*minor));
        }
        Kind::Plane { n, .. } if n.dot(nv).abs() > 1e-6 && n.dot(nv).abs() < 1.0 - 1e-6 => {
            for &e in &all {
                if let Curve::Line { .. } = m.curve(e) {
                    let g = m.across(e, f)?;
                    match role.get(&g) {
                        Some(2) | Some(3) => cap = cap.or(Some((g, e))),
                        Some(1) if matches!(m.kind(g), Kind::Plane { .. }) => wall = wall.or(Some((g, e))),
                        _ => {}
                    }
                }
            }
            if let (Some((cg, ce)), Some((wg, we))) = (cap, wall) {
                let (Kind::Plane { n: nc, d: dc }, Kind::Plane { n: nw, d: dw }) = (m.kind(cg), m.kind(wg)) else { return None };
                let (Curve::Line { d: d1, .. }, Curve::Line { d: d2, .. }) = (m.curve(ce), m.curve(we)) else { return None };
                if !par(*d1, *d2) {
                    return None;
                }
                cap_wall_dist = ((nw.dot(m.p0(ce)) - dw).abs(), (nc.dot(m.p0(we)) - dc).abs());
                kind = Some(BlendKind::Chamfer(cap_wall_dist.0));
            }
        }
        Kind::Cone { a, .. } if par(*a, nv) => {
            for &e in &all {
                if let Curve::Circle { .. } = m.curve(e) {
                    let g = m.across(e, f)?;
                    match role.get(&g) {
                        Some(2) | Some(3) => cap = cap.or(Some((g, e))),
                        Some(1) if matches!(m.kind(g), Kind::Cyl { .. }) => wall = wall.or(Some((g, e))),
                        _ => {}
                    }
                }
            }
            if let (Some((_, ce)), Some((wg, we))) = (cap, wall) {
                let Kind::Cyl { r: rw, .. } = m.kind(wg) else { return None };
                let Curve::Circle { r: rc, .. } = m.curve(ce) else { return None };
                let hc = m.p0(ce).dot(nv);
                let hw = m.p0(we).dot(nv);
                cap_wall_dist = ((rw - rc).abs(), (hc - hw).abs());
                kind = Some(BlendKind::Chamfer(cap_wall_dist.0));
            }
        }
        _ => {}
    }
    let kind = kind?;
    let (cg, _ce) = cap?;
    let (wg, we) = wall?;
    if cg == wg {
        return None;
    }
    let mut at_src = m.mid(we);
    if matches!(m.curve(we), Curve::Circle { .. }) {
        let ed = m.b.edge(we);
        at_src = m.curve(we).eval(0.5 * (ed.t0 + ed.t1));
    }
    match kind {
        BlendKind::Fillet(_) => {
            // tangent to both neighbours at the wall rail
            let nf = normal_at(m, f, at_src);
            let nw = normal_at(m, wg, at_src);
            if nf.cross(nw).len() > rail_tol {
                return None;
            }
        }
        BlendKind::Chamfer(_) => {
            if (cap_wall_dist.0 - cap_wall_dist.1).abs() > 1e-5 || cap_wall_dist.0 < 1e-6 {
                return None;
            }
        }
    }
    let top = role.get(&cg) == Some(&2);
    let level = if top { zmax } else { zmin };
    let at = at_src + nv * (level - at_src.dot(nv));
    Some(Blend { faces: vec![f], kind, cap: cg, wall: wg, at })
}
