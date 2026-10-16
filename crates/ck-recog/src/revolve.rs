//! Solid of revolution: every face is a plane normal to the axis, or a cylinder / cone / sphere / torus about it.
//! The half profile is the chain of the faces' generatrices, linked through their circular edges.

use crate::model::*;
use crate::prism::{Geom, Seg};
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_topo::body::FaceId;
use std::collections::HashSet;

pub struct Rev {
    pub ai: usize,
    pub spec: PlaneSpec,
    pub level: f64,
    /// Closed profile (counter-clockwise), including the closing line on the axis.
    pub segs: Vec<Seg>,
    pub axis: ((f64, f64), (f64, f64)),
    pub faces: Vec<FaceId>,
}

struct Link {
    n1: usize,
    n2: usize,
    arc: Option<((f64, f64), bool)>, // centre (s, rho), convex
    face: FaceId,
}

fn node(nodes: &mut Vec<(f64, f64)>, p: (f64, f64)) -> usize {
    if let Some(i) = nodes.iter().position(|q| (q.0 - p.0).abs() < 1e-6 && (q.1 - p.1).abs() < 1e-6) {
        return i;
    }
    nodes.push(p);
    nodes.len() - 1
}

pub fn analyze(m: &Model) -> Option<Rev> {
    // candidate axis from the first curved face
    let (ao, a) = m.faces.iter().find_map(|&f| match m.kind(f) {
        Kind::Cyl { o, a, .. } | Kind::Cone { o, a, .. } => Some((*o, *a)),
        Kind::Torus { c, a, .. } => Some((*c, *a)),
        _ => None,
    })?;
    let (ai, _) = principal(a)?;
    let a = axis_vec(ai);
    let on_axis = |p: Vec3| dist_to_line(p, ao, a) < 1e-6 * (1.0 + p.len());
    let s_of = |p: Vec3| (p - ao).dot(a);
    let mut nodes: Vec<(f64, f64)> = vec![];
    let mut links: Vec<Link> = vec![];
    for &f in &m.faces {
        let k = m.kind(f).clone();
        let mut circ: Vec<(f64, f64)> = vec![];
        for e in m.edges(f) {
            match m.curve(e) {
                Curve::Circle { frame, r } if par(frame.z, a) && on_axis(frame.o) => {
                    let p = (s_of(frame.o), *r);
                    if !circ.iter().any(|q| (q.0 - p.0).abs() < 1e-6 && (q.1 - p.1).abs() < 1e-6) {
                        circ.push(p);
                    }
                }
                _ => {
                    if m.b.edge_length(e) > 1e-7 {
                        return None;
                    }
                }
            }
        }
        let mut arc = None;
        match &k {
            Kind::Plane { n, .. } => {
                if !par(*n, a) {
                    return None;
                }
                if circ.len() == 1 {
                    let s = circ[0].0;
                    circ.push((s, 0.0));
                }
            }
            Kind::Cyl { o, a: ca, .. } => {
                if !(par(*ca, a) && on_axis(*o)) {
                    return None;
                }
            }
            Kind::Cone { o, a: ca, r0, half, .. } => {
                if !(par(*ca, a) && on_axis(*o)) {
                    return None;
                }
                if circ.len() == 1 {
                    let v = -r0 / half.tan();
                    circ.push((s_of(*o + *ca * v), 0.0));
                }
            }
            Kind::Sphere { c, r, convex } => {
                if !on_axis(*c) {
                    return None;
                }
                let sc = s_of(*c);
                match circ.len() {
                    0 => {
                        circ.push((sc - r, 0.0));
                        circ.push((sc + r, 0.0));
                    }
                    1 => {
                        let area = m.area(f);
                        let h = circ[0].0 - sc;
                        // cap beyond the circle on its own side, else the complement
                        let cap = 2.0 * std::f64::consts::PI * r * (r - h.abs());
                        let pole_same = (area - cap).abs() < (area - (4.0 * std::f64::consts::PI * r * r - cap)).abs();
                        let sign = if (h >= 0.0) == pole_same { 1.0 } else { -1.0 };
                        circ.push((sc + sign * r, 0.0));
                    }
                    _ => {}
                }
                arc = Some(((sc, 0.0), *convex));
            }
            Kind::Torus { c, a: ta, major, convex, .. } => {
                if !(par(*ta, a) && on_axis(*c)) {
                    return None;
                }
                arc = Some(((s_of(*c), *major), *convex));
            }
            Kind::Other(_) => return None,
        }
        if circ.len() != 2 {
            return None;
        }
        let n1 = node(&mut nodes, circ[0]);
        let n2 = node(&mut nodes, circ[1]);
        links.push(Link { n1, n2, arc, face: f });
    }
    // walk the chain
    let mut deg = vec![0usize; nodes.len()];
    for l in &links {
        deg[l.n1] += 1;
        deg[l.n2] += 1;
    }
    if deg.iter().any(|&d| d > 2) {
        return None;
    }
    let ends: Vec<usize> = (0..nodes.len()).filter(|&i| deg[i] == 1).collect();
    if !(ends.is_empty() || ends.len() == 2) {
        return None;
    }
    if ends.len() == 2 && !(nodes[ends[0]].1 < 1e-6 && nodes[ends[1]].1 < 1e-6) {
        return None;
    }
    let start = ends.first().copied().unwrap_or(links[0].n1);
    let mut used: HashSet<usize> = HashSet::new();
    let mut order: Vec<(usize, usize, usize)> = vec![]; // (link, from, to)
    let mut cur = start;
    loop {
        let Some(li) = (0..links.len()).find(|i| !used.contains(i) && (links[*i].n1 == cur || links[*i].n2 == cur)) else { break };
        used.insert(li);
        let l = &links[li];
        let to = if l.n1 == cur { l.n2 } else { l.n1 };
        order.push((li, cur, to));
        cur = to;
    }
    if used.len() != links.len() {
        return None;
    }
    let ni = if ai == 2 { 1 } else { 2 };
    let spec = PlaneSpec::for_axis(ni);
    let level = spec.level(ao);
    // in-plane unit direction normal to the axis
    let perp = if par(spec.xs, a) { spec.ys } else { spec.xs };
    let world = |p: (f64, f64)| ao + a * p.0 + perp * p.1;
    let xy = |p: (f64, f64)| spec.xy(world(p));
    let mut segs: Vec<Seg> = vec![];
    for (li, from, to) in &order {
        let l = &links[*li];
        let g = match l.arc {
            None => Geom::Line,
            Some((c, convex)) => Geom::Arc { c: xy(c), ccw: convex },
        };
        // arcs are stored for the as-built travel direction; flipped below if needed
        let _ = l.face;
        segs.push(Seg { a: xy(nodes[*from]), b: xy(nodes[*to]), g, face: l.face });
    }
    if ends.len() == 2 {
        let (a0, b0) = (xy(nodes[cur]), xy(nodes[start]));
        segs.push(Seg { a: a0, b: b0, g: Geom::Line, face: links[order[0].0].face });
    }
    // orientation: with an axis closure the material lies on the +rho side of it; else the chord polygon area
    let mut ar = 0.0;
    if ends.len() == 2 {
        let last = segs.last().unwrap();
        let d = (last.b.0 - last.a.0, last.b.1 - last.a.1);
        let p2 = spec.xy(perp);
        ar = d.0 * p2.1 - d.1 * p2.0;
    } else {
        let pts: Vec<(f64, f64)> = segs.iter().map(|s| s.a).collect();
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            ar += p.0 * q.1 - q.0 * p.1;
        }
    }
    if ar.abs() < 1e-12 {
        return None;
    }
    if ar < 0.0 {
        segs.reverse();
        for s in segs.iter_mut() {
            std::mem::swap(&mut s.a, &mut s.b);
        }
    }
    // arcs: convex = counter-clockwise in a counter-clockwise loop, in the final travel direction
    for s in segs.iter_mut() {
        if let Geom::Arc { ccw, .. } = &mut s.g {
            let convex = *ccw;
            *ccw = convex;
        }
    }
    let (mut smin, mut smax) = (f64::INFINITY, f64::NEG_INFINITY);
    for n in &nodes {
        smin = smin.min(n.0);
        smax = smax.max(n.0);
    }
    let axis = (xy((smin, 0.0)), xy((smax, 0.0)));
    Some(Rev { ai, spec, level, segs, axis, faces: m.faces.clone() })
}
