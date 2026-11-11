//! Sheet metal, stage 4: `sheetConvert`, a solid shell turned into a sheet body (docs/sheet-metal.md, "Stage 4").
//!
//! The solid must be a bent plate of constant thickness: pairs of parallel planar faces `t` apart (the panels), pairs of coaxial
//! cylinders `t` apart in radius (the bends), and the planar walls that close the edges. The faces are walked from the largest
//! panel across the bends; every panel is unfolded into the flat layout of the root panel and the result is the panel tree of
//! the sheet (`sheet_tree.rs`), from which the body is rebuilt and checked against the volume of the original.

use super::sheet::{Rule, Sheet, P};
use super::sheet_features::{rule_of, unsupported, Model};
use super::sheet_tree::{ccw, BendKind, Flat3, Panel, Poly, TBend, Tree};
use super::Brep;
use crate::regen::{codes, FeatureError, RegenState};
use crate::Feature;
use ck_core::linalg::{Frame, Vec3};
use ck_geom::surface::Surface;
use ck_topo::body::{Body, EdgeId, FaceId};
use ck_topo::make::Profile;

/// A planar face: its outward normal, plane offset, outer ring (3D) with the edge leaving each point, and the holes.
struct Pf {
    id: FaceId,
    n: Vec3,
    d: f64,
    ring: Vec<Vec3>,
    seg: Vec<EdgeId>,
    holes: Vec<Vec<Vec3>>,
}

struct Cyl {
    id: FaceId,
    o: Vec3,
    axis: Vec3,
    r: f64,
}

fn ring_of(body: &Body, f: FaceId) -> (Vec<(Vec<Vec3>, Vec<EdgeId>)>, Vec<f64>) {
    let mut out = vec![];
    let mut areas = vec![];
    let n = body.planar_normal(f);
    for &l in &body.face(f).loops {
        let mut pts = vec![];
        let mut seg = vec![];
        for &c in &body.lp(l).coedges {
            let ce = body.coedge(c);
            let mut s = body.sample_edge(ce.edge, 0.01);
            if ce.rev {
                s.reverse();
            }
            s.pop();
            for p in s {
                pts.push(p);
                seg.push(ce.edge);
            }
        }
        let m = pts.len();
        let a = (0..m).map(|i| pts[i].cross(pts[(i + 1) % m]).dot(n)).sum::<f64>() * 0.5;
        areas.push(a);
        out.push((pts, seg));
    }
    (out, areas)
}

fn planar(body: &Body, f: FaceId) -> Option<Pf> {
    if !matches!(body.face(f).surface, Surface::Plane { .. }) {
        return None;
    }
    let n = body.planar_normal(f);
    let (rings, areas) = ring_of(body, f);
    if rings.is_empty() {
        return None;
    }
    let outer = (0..rings.len()).max_by(|&a, &b| areas[a].abs().partial_cmp(&areas[b].abs()).unwrap_or(std::cmp::Ordering::Equal))?;
    let d = n.dot(rings[outer].0[0]);
    let mut holes = vec![];
    let mut main = None;
    for (i, (p, s)) in rings.into_iter().enumerate() {
        if i == outer {
            main = Some((p, s));
        } else {
            holes.push(p);
        }
    }
    let (ring, seg) = main?;
    Some(Pf { id: f, n, d, ring, seg, holes })
}

fn centroid(p: &[Vec3]) -> Vec3 {
    p.iter().fold(Vec3::ZERO, |a, q| a + *q) * (1.0 / p.len().max(1) as f64)
}

fn area_of(p: &[Vec3], n: Vec3) -> f64 {
    let m = p.len();
    ((0..m).map(|i| p[i].cross(p[(i + 1) % m]).dot(n)).sum::<f64>() * 0.5).abs()
}

/// Is the point (projected along `n`) inside the planar ring?
fn in_ring(ring: &[Vec3], n: Vec3, p: Vec3) -> bool {
    let u = if n.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
    let ex = (u - n * u.dot(n)).normalize();
    let ey = n.cross(ex);
    let q: Vec<P> = ring.iter().map(|v| [v.dot(ex), v.dot(ey)]).collect();
    super::sheet_tree::inside(&q, [p.dot(ex), p.dot(ey)], 1e-9)
}

/// Do two antiparallel faces face each other over some area?
fn overlap(a: &Pf, b: &Pf) -> bool {
    let probe = |x: &Pf, y: &Pf| x.ring.iter().any(|p| in_ring(&y.ring, y.n, *p)) || in_ring(&y.ring, y.n, centroid(&x.ring));
    probe(a, b) || probe(b, a)
}

struct Panel3 {
    front: usize,
    back: usize,
}

pub fn exec_sheet_convert(br: &mut Brep, state: &mut RegenState, f: &Feature) -> Result<(), FeatureError> {
    let body = br.body.clone().ok_or_else(|| FeatureError::new(codes::NOTHING_TO_CUT))?;
    let hint = "convert a bent plate of constant thickness: planar panels and cylindrical bends";
    let mut rule = rule_of(br, f)?;
    let faces: Vec<FaceId> = body.face_ids().collect();
    let planes: Vec<Pf> = faces.iter().filter_map(|&x| planar(&body, x)).collect();
    // thickness: given, or the smallest gap between facing planar faces
    let given = f.raw.get_f64("thickness").filter(|t| *t > 0.0);
    let t = match given {
        Some(t) => t,
        None => {
            let mut best = f64::MAX;
            for (i, a) in planes.iter().enumerate() {
                for b in planes.iter().skip(i + 1) {
                    if a.n.dot(b.n) < -1.0 + 1e-9 {
                        let dist = (a.d + b.d).abs();
                        if dist > 1e-6 && dist < best && overlap(a, b) {
                            best = dist;
                        }
                    }
                }
            }
            if best == f64::MAX {
                return Err(unsupported("the body has no pair of parallel faces to take the thickness from", hint));
            }
            best
        }
    };
    rule.thickness = t;
    if f.raw.get("rule").is_none() {
        rule.name = "Converted".into();
    }
    // panels: pairs of facing faces t apart
    let mut partner: Vec<Option<usize>> = vec![None; planes.len()];
    for i in 0..planes.len() {
        for j in i + 1..planes.len() {
            if partner[i].is_none() && partner[j].is_none() && planes[i].n.dot(planes[j].n) < -1.0 + 1e-9 && ((planes[i].d + planes[j].d).abs() - t).abs() < 1e-4 * t.max(1.0) && overlap(&planes[i], &planes[j]) {
                partner[i] = Some(j);
                partner[j] = Some(i);
            }
        }
    }
    let plane_of: std::collections::HashMap<usize, usize> = planes.iter().enumerate().map(|(i, p)| (p.id.idx(), i)).collect();
    // bends: coaxial cylinder pairs t apart in radius
    let cyls: Vec<Cyl> = faces
        .iter()
        .filter_map(|&x| match &body.face(x).surface {
            Surface::Cylinder { frame, r } => Some(Cyl { id: x, o: frame.o, axis: frame.z.normalize(), r: *r }),
            _ => None,
        })
        .collect();
    let mut cpart: Vec<Option<usize>> = vec![None; cyls.len()];
    for i in 0..cyls.len() {
        for j in i + 1..cyls.len() {
            let (a, b) = (&cyls[i], &cyls[j]);
            let coaxial = a.axis.cross(b.axis).len() < 1e-9 && (b.o - a.o).cross(a.axis).len() < 1e-6;
            if cpart[i].is_none() && cpart[j].is_none() && coaxial && ((a.r - b.r).abs() - t).abs() < 1e-4 * t.max(1.0) {
                cpart[i] = Some(j);
                cpart[j] = Some(i);
            }
        }
    }
    if let Some(c) = cyls.iter().enumerate().find(|(i, _)| cpart[*i].is_none()) {
        let _ = c;
        return Err(unsupported("a cylindrical face has no partner t away: the body is not a bent plate of constant thickness", hint));
    }
    let cyl_of: std::collections::HashMap<usize, usize> = cyls.iter().enumerate().map(|(i, c)| (c.id.idx(), i)).collect();
    // the root: the panel (front, back) with the largest face
    let root = (0..planes.len())
        .filter(|&i| partner[i].is_some())
        .max_by(|&a, &b| area_of(&planes[a].ring, planes[a].n).partial_cmp(&area_of(&planes[b].ring, planes[b].n)).unwrap_or(std::cmp::Ordering::Equal))
        .ok_or_else(|| unsupported("no panel found: no two parallel faces are one thickness apart", hint))?;
    let rootp = Panel3 { front: root, back: partner[root].unwrap_or(root) };
    let nrm = -planes[rootp.front].n;
    let e1 = (planes[rootp.front].ring[1] - planes[rootp.front].ring[0]).normalize();
    let flat = Flat3 { o: planes[rootp.front].ring[0], x: e1, y: nrm.cross(e1), n: nrm };
    let to_flat = move |p: Vec3| -> P {
        let r = p - flat.o;
        [r.dot(flat.x), r.dot(flat.y)]
    };
    // walk
    struct Item {
        p3: Panel3,
        /// Maps a 3D point of this panel to flat coordinates.
        map: Box<dyn Fn(Vec3) -> P>,
        index: usize,
        /// The bend (cylinder pair, bend index) the panel hangs on.
        via: Option<(usize, usize)>,
    }
    let mut tree = Tree { flat, panels: vec![], bends: vec![], folds: 0, reliefs: 0, cuts: vec![] };
    let mut seen_panel = vec![false; planes.len()];
    let mut seen_bend = vec![false; cyls.len()];
    let mut queue: std::collections::VecDeque<Item> = std::collections::VecDeque::new();
    let mut wall_no = 0usize;
    // panel polygons are made when a panel is taken from the queue; its bends are found from the edges of its front ring
    seen_panel[rootp.front] = true;
    seen_panel[rootp.back] = true;
    queue.push_back(Item { p3: rootp, map: Box::new(to_flat), index: 0, via: None });
    tree.panels.push(Panel { poly: vec![], holes: vec![], via: None });
    let mut k = rule.k_factor;
    if !(0.0..=1.0).contains(&k) {
        k = 0.44;
    }
    let mut first_radius: Option<f64> = None;
    while let Some(item) = queue.pop_front() {
        let pf = &planes[item.p3.front];
        let n = pf.ring.len();
        let mut tags: Vec<String> = vec![String::new(); n];
        let mut i = 0;
        // the edges of the ring grouped by the edge they sample; those shared with a cylinder are the sides of bends
        let mut children: Vec<(usize, usize, usize)> = vec![]; // (bend cyl idx, first ring point, count)
        while i < n {
            let e = pf.seg[i];
            let mut j = i;
            while j + 1 < n && pf.seg[j + 1] == e {
                j += 1;
            }
            let others: Vec<FaceId> = body.edge_faces(e).into_iter().filter(|x| *x != pf.id).collect();
            let cyl = others.iter().find_map(|x| cyl_of.get(&x.idx()).copied());
            let neighbour_panel = others.iter().find_map(|x| plane_of.get(&x.idx()).copied().filter(|pi| partner[*pi].is_some() && *pi != item.p3.back));
            if neighbour_panel.is_some() {
                return Err(unsupported("two panels meet at a sharp corner (no bend radius between them)", "convert bodies whose panels are joined by cylindrical bends"));
            }
            match cyl {
                Some(ci) => children.push((ci, i, j - i + 1)),
                None => {
                    wall_no += 1;
                    for t_ in tags.iter_mut().take(j + 1).skip(i) {
                        *t_ = format!("w{wall_no}");
                    }
                }
            }
            i = j + 1;
        }
        // bends hanging on this panel
        for (ci, first, count) in children {
            let bend_cyl = ci.min(cpart[ci].unwrap_or(ci));
            let (cx, cy) = (cyls[ci].r, cyls[cpart[ci].unwrap_or(ci)].r);
            let already = seen_bend[bend_cyl];
            if already {
                // the bend this panel hangs on: its side of the zone
                if let Some((vc, vbi)) = item.via {
                    if vc == bend_cyl {
                        for t_ in tags.iter_mut().skip(first).take(count) {
                            *t_ = format!("z{vbi}b");
                        }
                    }
                }
                continue;
            }
            seen_bend[bend_cyl] = true;
            let (inner, outer) = if cx < cy { (cx, cy) } else { (cy, cx) };
            let a_pt = pf.ring[first];
            let b_pt = pf.ring[(first + count) % n];
            let axis = cyls[ci].axis;
            let a_dir = if (b_pt - a_pt).dot(axis) >= 0.0 { axis } else { -axis };
            // the child: the planar panel face next to this cylinder (the front side continues on this cylinder)
            let cface = cyls[ci].id;
            let child_front = body.face_neighbors(cface).into_iter().find_map(|x| plane_of.get(&x.idx()).copied().filter(|pi| *pi != item.p3.front && partner[*pi].is_some() && !seen_panel[*pi]));
            let Some(cf) = child_front else { return Err(unsupported("a bend does not lead to another panel", hint)) };
            let cb = partner[cf].unwrap_or(cf);
            seen_panel[cf] = true;
            seen_panel[cb] = true;
            let child = &planes[cf];
            let nc = -child.n;
            let np = -pf.n;
            let ang = np.dot(nc).clamp(-1.0, 1.0).acos();
            let angle = if ang > std::f64::consts::PI - 1e-6 { std::f64::consts::PI } else { ang };
            if angle < 1e-6 {
                return Err(unsupported("a bend of angle 0", hint));
            }
            let up = (cyls[ci].o - a_pt).dot(np) > 0.0;
            let ba = (inner + k * t) * angle;
            let fa = (item.map)(a_pt);
            let fb = (item.map)(b_pt);
            let dv = [fb[0] - fa[0], fb[1] - fa[1]];
            let dl = dv[0].hypot(dv[1]);
            let d = [dv[0] / dl, dv[1] / dl];
            // away from the parent's polygon
            let cen = (item.map)(centroid(&pf.ring));
            let mut nu = [-d[1], d[0]];
            if (cen[0] - fa[0]) * nu[0] + (cen[1] - fa[1]) * nu[1] > 0.0 {
                nu = [-nu[0], -nu[1]];
            }
            // the child's tangent line: its ring points on the cylinder face edge
            let cring = &child.ring;
            let cn = cring.len();
            let tang = (0..cn).find(|&q| body.edge_faces(child.seg[q]).contains(&cface));
            let Some(tq) = tang else { return Err(unsupported("a bend does not touch the next panel", hint)) };
            let c0 = cring[tq];
            let mut ep = nc.cross(a_dir);
            if (centroid(cring) - c0).dot(ep) < 0.0 {
                ep = -ep;
            }
            let o_flat = fa;
            let ba_ = ba;
            let a_pt_c = a_pt;
            let map_child = move |p: Vec3| -> P {
                let v = (p - a_pt_c).dot(a_dir);
                let u = ba_ + (p - c0).dot(ep);
                [o_flat[0] + d[0] * v + nu[0] * u, o_flat[1] + d[1] * v + nu[1] * u]
            };
            let bi = tree.bends.len();
            let cidx = tree.panels.len();
            tree.panels.push(Panel { poly: vec![], holes: vec![], via: Some(bi) });
            tree.bends.push(TBend {
                feature: f.id.clone(),
                parent: item.index,
                child: cidx,
                o: fa,
                d,
                nu,
                z: [0.0, ba],
                v0: [0.0, dl],
                v1: [0.0, dl],
                angle,
                radius: inner,
                up,
                kind: BendKind::Fold,
                attach: format!("z{bi}a"),
                attach_child: format!("z{bi}b"),
                hem: false,
                batch: 0,
                edge: 0,
            });
            let _ = outer;
            first_radius.get_or_insert(inner);
            for t_ in tags.iter_mut().skip(first).take(count) {
                *t_ = format!("z{bi}a");
            }
            queue.push_back(Item { p3: Panel3 { front: cf, back: cb }, map: Box::new(map_child), index: cidx, via: Some((bend_cyl, bi)) });
        }
        // the polygon of this panel
        let poly: Poly = (0..n).map(|q| ((item.map)(pf.ring[q]), tags[q].clone())).collect();
        let holes: Vec<Vec<P>> = pf.holes.iter().map(|h| h.iter().map(|p| (item.map)(*p)).collect()).collect();
        tree.panels[item.index].poly = ccw(poly);
        tree.panels[item.index].holes = holes;
    }
    if seen_panel.iter().zip(&partner).any(|(s, p)| !*s && p.is_some()) {
        return Err(unsupported("the body has panels that no bend connects to the first one", hint));
    }
    tree.refresh();
    let radius = first_radius.unwrap_or(t);
    rule.bend_radius = radius;
    if rule.relief_width < t {
        rule.relief_width = t;
    }
    let frame = Frame { o: tree.flat.o, x: tree.flat.x, y: tree.flat.y, z: tree.flat.x.cross(tree.flat.y) };
    let dir = if tree.flat.n.dot(frame.z) >= 0.0 { 1.0 } else { -1.0 };
    let sheet = Sheet { base: f.id.clone(), rule: rule.clone(), outline: vec![], tags: vec![], line: vec![], holes: vec![], flanges: vec![], folds: vec![], fold_root: None };
    let mut model = Model { sheet, frame, dir, profile: Profile::new(frame, vec![]), sig: (0, 0), batches: 0, contour: None, unfolded: false, roles: vec![], tree: Some(tree) };
    let vol0 = ck_topo::mass::mass_props(&body).volume;
    super::sheet_features::rebuild_checked(br, &mut model, vol0)?;
    br.sheet.rules.retain(|r| r.name != rule.name);
    br.sheet.rules.push(rule);
    br.sheet.active = Some(br.sheet.rules.len() - 1);
    br.sheet.model = Some(model);
    super::sheet_features::report_pub(br, state, f, "sheetConvert");
    Ok(())
}

#[allow(dead_code)]
fn _unused(_: &Rule) {}
