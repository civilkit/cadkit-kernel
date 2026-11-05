//! Hole recognition: groups of coaxial concave cylinders / cones (plus the planar floors and steps between
//! them) that open onto planar mouths.

use crate::model::*;
use ck_geom::curve::Curve;
use ck_geom::Vec3;
use ck_topo::body::FaceId;
use std::collections::HashSet;

#[derive(Debug, Clone)]
pub struct Hole {
    pub faces: Vec<FaceId>,
    pub entry_face: FaceId,
    /// Unit direction into the material.
    pub axis_in: Vec3,
    /// Centre of the mouth circle on the entry plane.
    pub centre: Vec3,
    pub d: f64,
    pub through: bool,
    /// Depth of the main bore end below the entry plane (blind holes).
    pub depth: f64,
    pub tip: bool,
    /// Counterbore (diameter, depth).
    pub cbore: Option<(f64, f64)>,
    /// Countersink (head diameter, included angle in degrees, cone depth).
    pub csk: Option<(f64, f64, f64)>,
}

impl Hole {
    pub fn kind(&self) -> &'static str {
        if self.cbore.is_some() {
            "cbore"
        } else if self.csk.is_some() {
            "csk"
        } else {
            "simple"
        }
    }
}

struct Stage {
    cyl: bool,
    s0: f64,
    s1: f64,
    r0: f64,
    r1: f64,
}

fn coaxial(o: Vec3, a: Vec3, o2: Vec3, a2: Vec3) -> bool {
    par(a, a2) && dist_to_line(o2, o, a) < 1e-6 * (1.0 + o2.len())
}

/// Axis data of a hole-wall candidate.
fn axis_of(k: &Kind) -> Option<(Vec3, Vec3)> {
    match k {
        Kind::Cyl { o, a, convex: false, .. } => Some((*o, *a)),
        Kind::Cone { o, a, convex: false, .. } => Some((*o, *a)),
        _ => None,
    }
}

fn s_range(m: &Model, f: FaceId, o: Vec3, a: Vec3) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for e in m.b.face_edges(f) {
        for p in [m.p0(e), m.p1(e)] {
            let s = (p - o).dot(a);
            lo = lo.min(s);
            hi = hi.max(s);
        }
    }
    (lo, hi)
}

fn plane_is_hole_structure(m: &Model, g: FaceId, o: Vec3, a: Vec3) -> bool {
    let Kind::Plane { n, .. } = m.kind(g) else { return false };
    if !par(*n, a) {
        return false;
    }
    let es = m.edges(g);
    if es.is_empty() {
        return false;
    }
    for &e in &es {
        match m.curve(e) {
            Curve::Circle { frame, .. } => {
                if !par(frame.z, a) || dist_to_line(frame.o, o, a) > 1e-6 * (1.0 + o.len()) {
                    return false;
                }
            }
            _ => return false,
        }
    }
    // a concave edge with any coaxial concave wall: the wall extends from the edge in the direction of the plane's
    // outward normal
    for &e in &es {
        let Some(via) = m.across(e, g) else { continue };
        let Some((o2, a2)) = axis_of(m.kind(via)) else { continue };
        if !coaxial(o, a, o2, a2) {
            continue;
        }
        let (lo, hi) = s_range(m, via, o, a);
        let mid = 0.5 * (lo + hi);
        let s_e = (m.p0(e) - o).dot(a);
        let dir = if mid > s_e { a } else { -a };
        if n.dot(dir) > 0.5 {
            return true;
        }
    }
    false
}

pub fn find_holes(m: &Model, claimed: &mut HashSet<FaceId>) -> Vec<Hole> {
    find_holes_in(m, claimed, None)
}

/// Holes read from the faces of `only` alone: a hole feature's own faces, when later features cut into its walls
/// (cross holes) or its far end (a counterbore from the other side). Faces outside `only` do not join a group, and a
/// neighbour that is no plane across the axis is ignored where the strict reading would refuse the group.
pub fn find_holes_in(m: &Model, claimed: &mut HashSet<FaceId>, only: Option<&HashSet<FaceId>>) -> Vec<Hole> {
    let mut out = vec![];
    let mut seen: HashSet<FaceId> = HashSet::new();
    for &c in &m.faces {
        if seen.contains(&c) || claimed.contains(&c) {
            continue;
        }
        if only.map_or(false, |s| !s.contains(&c)) {
            continue;
        }
        let Kind::Cyl { o, a, convex: false, .. } = m.kind(c).clone() else { continue };
        let mut group = vec![c];
        let mut inset: HashSet<FaceId> = HashSet::new();
        inset.insert(c);
        let mut queue = vec![c];
        while let Some(f) = queue.pop() {
            for e in m.edges(f) {
                let Some(g) = m.across(e, f) else { continue };
                if inset.contains(&g) || claimed.contains(&g) || only.map_or(false, |s| !s.contains(&g)) {
                    continue;
                }
                let ok = match axis_of(m.kind(g)) {
                    Some((o2, a2)) => coaxial(o, a, o2, a2),
                    None => plane_is_hole_structure(m, g, o, a),
                };
                // a plane may be reached from a cone only through the cylinder test above
                if ok {
                    inset.insert(g);
                    group.push(g);
                    queue.push(g);
                }
            }
        }
        for &g in &group {
            seen.insert(g);
        }
        if let Some(h) = classify(m, &group, &inset, o, a, only.is_some()) {
            for &g in &h.faces {
                claimed.insert(g);
            }
            out.push(h);
        }
    }
    out
}

fn classify(m: &Model, group: &[FaceId], inset: &HashSet<FaceId>, o: Vec3, a: Vec3, tolerant: bool) -> Option<Hole> {
    // mouths
    struct Mouth {
        plane: FaceId,
        n: Vec3,
        centre: Vec3,
        r: f64,
    }
    let mut mouths: Vec<Mouth> = vec![];
    for &f in group {
        for e in m.edges(f) {
            let Some(g) = m.across(e, f) else { return None };
            if inset.contains(&g) {
                continue;
            }
            let Kind::Plane { n, .. } = m.kind(g) else {
                if tolerant {
                    continue;
                }
                return None;
            };
            if !par(*n, a) {
                if tolerant {
                    continue;
                }
                return None;
            }
            let Curve::Circle { frame, r } = m.curve(e) else {
                if tolerant {
                    continue;
                }
                return None;
            };
            if !coaxial(o, a, frame.o, frame.z) {
                return None;
            }
            if !mouths.iter().any(|mo| mo.plane == g && (mo.r - r).abs() < 1e-6) {
                mouths.push(Mouth { plane: g, n: *n, centre: frame.o, r: *r });
            }
        }
    }
    if mouths.is_empty() || mouths.len() > 2 {
        return None;
    }
    if mouths.len() == 2 && mouths[0].plane == mouths[1].plane {
        return None;
    }
    let mut ei = 0;
    if mouths.len() == 2 {
        let (m0, m1) = (&mouths[0], &mouths[1]);
        ei = if (m0.r - m1.r).abs() > 1e-6 {
            if m0.r > m1.r {
                0
            } else {
                1
            }
        } else if (m0.n.x, m0.n.y, m0.n.z) > (m1.n.x, m1.n.y, m1.n.z) {
            0
        } else {
            1
        };
    }
    let entry = &mouths[ei];
    let a_in = (-entry.n).normalize();
    let centre = entry.centre;
    let s_of = |p: Vec3| (p - centre).dot(a_in);

    let mut stages: Vec<Stage> = vec![];
    for &f in group {
        match m.kind(f) {
            Kind::Cyl { r, .. } => {
                let (lo, hi) = s_range(m, f, centre, a_in);
                stages.push(Stage { cyl: true, s0: lo, s1: hi, r0: *r, r1: *r });
            }
            Kind::Cone { o: co, a: ca, r0, half, .. } => {
                let mut pts: Vec<(f64, f64)> = vec![];
                for e in m.edges(f) {
                    if let Curve::Circle { frame, r } = m.curve(e) {
                        pts.push((s_of(frame.o), *r));
                    }
                }
                if pts.len() == 1 {
                    let v = -r0 / half.tan();
                    pts.push((s_of(*co + *ca * v), 0.0));
                }
                if pts.len() != 2 {
                    return None;
                }
                pts.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
                stages.push(Stage { cyl: false, s0: pts[0].0, s1: pts[1].0, r0: pts[0].1, r1: pts[1].1 });
            }
            _ => {}
        }
    }
    stages.sort_by(|x, y| x.s0.partial_cmp(&y.s0).unwrap().then(x.s1.partial_cmp(&y.s1).unwrap()));
    let through = mouths.len() == 2;
    let mut i = 0;
    let mut csk = None;
    let mut cbore = None;
    if !stages.is_empty() && !stages[0].cyl && stages.len() > 1 && stages[1].cyl && stages[0].r0 > stages[0].r1 + 1e-6 {
        let st = &stages[0];
        let cd = st.s1 - st.s0;
        if cd < 1e-6 {
            return None;
        }
        csk = Some((2.0 * st.r0, 2.0 * ((st.r0 - st.r1) / cd).atan().to_degrees(), cd));
        i = 1;
    }
    if csk.is_none() && stages.len() > i + 1 && stages[i].cyl && stages[i + 1].cyl && stages[i].r0 > stages[i + 1].r0 + 1e-6 {
        cbore = Some((2.0 * stages[i].r0, stages[i].s1));
        i += 1;
    }
    let main = stages.get(i)?;
    if !main.cyl {
        return None;
    }
    let mut tip = false;
    let rest = &stages[i + 1..];
    match rest.len() {
        0 => {}
        1 if !rest[0].cyl && rest[0].r1 < 1e-6 && !through => tip = true,
        _ => return None,
    }
    Some(Hole {
        faces: group.to_vec(),
        entry_face: entry.plane,
        axis_in: a_in,
        centre,
        d: 2.0 * main.r0,
        through,
        depth: main.s1,
        tip,
        cbore,
        csk,
    })
}
