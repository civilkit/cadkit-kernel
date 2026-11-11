//! Sheet metal, stage 3: the panel tree (docs/sheet-metal.md, "Stage 3"). The pure 2D part.
//!
//! A sheet is a tree of flat panels joined by bends. Everything is kept in the flat layout (the sheet laid out in the plane of
//! the base): a panel is a polygon in flat coordinates, a bend is a strip between two panels (a line, a zone `[z0, z1]` measured
//! along its normal `nu`, which points from the parent panel to the child) with the angle, radius and direction of the bend.
//! The flat pattern is the union of the panels and the zone strips; the folded solid is the panels and bends moved by the
//! chain of bend transforms down the tree (`sheet_tree3d.rs`). Edge flanges, hems, chained bends and folds all are the same
//! thing here: a strip on a panel edge or across a panel, so they combine freely.
//!
//! Legacy sheets (a base with edge flanges, a folded base, a contour flange) convert to a tree exactly (`from_sheet`,
//! `from_contour`): same panels, same bend rows, same flat pattern.

use super::sheet::{area, chain_of, clean, Bend, BendRow, FlatPattern, HeightFrom, Next, Position, ReliefShape, Rule, Sheet, P};
use super::sheet_contour::Contour;
use super::sheet_fold::{self, zone_of, FoldPos};
use ck_core::linalg::Vec3;

pub type Poly = Vec<(P, String)>;

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}
fn mul(a: P, k: f64) -> P {
    [a[0] * k, a[1] * k]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn len(a: P) -> f64 {
    a[0].hypot(a[1])
}
fn unit(a: P) -> P {
    let l = len(a).max(1e-300);
    [a[0] / l, a[1] / l]
}

/// Where the flat layout sits in space: flat `(x, y)` at height `h` above the front face is `o + x px + y py + n h`
/// (`n` is the unit direction from the front face to the back face; it need not be `x cross y`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Flat3 {
    pub o: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub n: Vec3,
}

impl Flat3 {
    pub fn point(&self, p: P, h: f64) -> Vec3 {
        self.o + self.x * p[0] + self.y * p[1] + self.n * h
    }
    pub fn dir(&self, v: P) -> Vec3 {
        self.x * v[0] + self.y * v[1]
    }
    /// The flat coordinates and the height of a point (inverse of [`Flat3::point`] for points on the layout).
    pub fn flat_of(&self, w: Vec3) -> (P, f64) {
        let r = w - self.o;
        ([r.dot(self.x), r.dot(self.y)], r.dot(self.n))
    }
}

pub fn poly_area(p: &Poly) -> f64 {
    let n = p.len();
    (0..n).map(|i| cross(p[i].0, p[(i + 1) % n].0)).sum::<f64>() * 0.5
}

pub fn pts_of(p: &Poly) -> Vec<P> {
    p.iter().map(|v| v.0).collect()
}

/// Drop repeated vertices (the later vertex keeps the tag of the edge that leaves it).
fn tidy(v: Poly) -> Poly {
    let mut out: Poly = vec![];
    for q in v {
        match out.last_mut() {
            Some(l) if len(sub(l.0, q.0)) <= 1e-9 => {
                l.1 = q.1;
            }
            _ => out.push(q),
        }
    }
    while out.len() > 1 && len(sub(out[0].0, out[out.len() - 1].0)) <= 1e-9 {
        out.pop();
    }
    out
}

/// The same polygon, counter-clockwise (the tag of an edge travels with it when the ring is reversed).
pub fn ccw(p: Poly) -> Poly {
    if poly_area(&p) >= 0.0 {
        return p;
    }
    let n = p.len();
    (0..n).map(|j| (p[n - 1 - j].0, p[(n + n - 2 - j) % n].1.clone())).collect()
}

fn seg_dist(p: P, a: P, b: P) -> f64 {
    let ab = sub(b, a);
    let l2 = dot(ab, ab);
    let s = if l2 < 1e-24 { 0.0 } else { (dot(sub(p, a), ab) / l2).clamp(0.0, 1.0) };
    len(sub(p, add(a, mul(ab, s))))
}

/// `p` is inside the ring or within `tol` of its boundary.
pub fn inside(ring: &[P], p: P, tol: f64) -> bool {
    let n = ring.len();
    if n < 3 {
        return false;
    }
    let mut c = false;
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        if seg_dist(p, a, b) <= tol {
            return true;
        }
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
            c = !c;
        }
    }
    c
}

#[derive(Clone, Copy, PartialEq)]
enum K {
    In,
    Out,
    Enter,
    Exit,
}

struct V {
    p: P,
    tag: String,
    k: K,
    t: f64,
}

/// Clip a (possibly concave) polygon by the half-plane `n . p - c >= 0`; the new edges along the clip line get `tag`. The result is
/// a list of polygons: a concave polygon can fall into several pieces.
pub fn clip_half(poly: &Poly, n: P, c: f64, tag: &str) -> Vec<Poly> {
    let m = poly.len();
    if m < 3 {
        return vec![];
    }
    const E: f64 = 1e-9;
    let dist = |p: P| dot(n, p) - c;
    let tdir = [-n[1], n[0]];
    let mut ring: Vec<V> = vec![];
    for i in 0..m {
        let (a, ta) = (poly[i].0, &poly[i].1);
        let b = poly[(i + 1) % m].0;
        let (da, db) = (dist(a), dist(b));
        let (ia, ib) = (da >= -E, db >= -E);
        ring.push(V { p: a, tag: ta.clone(), k: if ia { K::In } else { K::Out }, t: 0.0 });
        if ia != ib {
            let s = (da / (da - db)).clamp(0.0, 1.0);
            let x = add(a, mul(sub(b, a), s));
            ring.push(V { p: x, tag: ta.clone(), k: if ia { K::Exit } else { K::Enter }, t: dot(tdir, x) });
        }
    }
    let cross_idx: Vec<usize> = ring.iter().enumerate().filter(|(_, v)| matches!(v.k, K::Enter | K::Exit)).map(|(i, _)| i).collect();
    if cross_idx.is_empty() {
        return if ring.iter().all(|v| v.k == K::In) { vec![poly.clone()] } else { vec![] };
    }
    let mut order = cross_idx.clone();
    order.sort_by(|&a, &b| ring[a].t.partial_cmp(&ring[b].t).unwrap_or(std::cmp::Ordering::Equal));
    let mut mate = vec![usize::MAX; ring.len()];
    for pair in order.chunks(2) {
        if pair.len() == 2 {
            mate[pair[0]] = pair[1];
            mate[pair[1]] = pair[0];
        }
    }
    let mut used = vec![false; ring.len()];
    let mut out: Vec<Poly> = vec![];
    for &s in &cross_idx {
        if ring[s].k != K::Enter || used[s] {
            continue;
        }
        let mut res: Poly = vec![];
        let mut cur = s;
        let mut guard = 0;
        let ok = loop {
            guard += 1;
            if guard > ring.len() + 2 {
                break false;
            }
            used[cur] = true;
            let mut i = cur;
            let mut steps = 0;
            loop {
                steps += 1;
                if steps > ring.len() + 2 || ring[i].k == K::Out {
                    break;
                }
                if ring[i].k == K::Exit {
                    res.push((ring[i].p, tag.to_string()));
                    break;
                }
                res.push((ring[i].p, ring[i].tag.clone()));
                i = (i + 1) % ring.len();
            }
            if ring[i].k != K::Exit {
                break false;
            }
            let next = mate[i];
            if next == usize::MAX || ring[next].k != K::Enter {
                break false;
            }
            if next == s {
                break true;
            }
            if used[next] {
                break false;
            }
            cur = next;
        };
        if !ok {
            continue;
        }
        let res = tidy(res);
        if res.len() >= 3 && poly_area(&res) > 1e-9 {
            out.push(res);
        }
    }
    out
}

/// The outer boundary loops of a union of polygons (shared edges cancel): outer loops counter-clockwise, largest first, then the
/// holes (clockwise). Vertices of one polygon lying on an edge of another split that edge.
pub fn merge_loops(polys: &[Vec<P>]) -> Vec<Vec<P>> {
    const T: f64 = 1e-7;
    let mut pts: Vec<P> = vec![];
    let canon = |p: P, pts: &mut Vec<P>| -> usize {
        if let Some(i) = pts.iter().position(|q| len(sub(*q, p)) <= T) {
            return i;
        }
        pts.push(p);
        pts.len() - 1
    };
    let mut rings: Vec<Vec<usize>> = vec![];
    for p in polys {
        if p.len() < 3 || area(p).abs() < 1e-9 {
            continue;
        }
        let mut q = p.clone();
        if area(&q) < 0.0 {
            q.reverse();
        }
        let ids: Vec<usize> = q.iter().map(|v| canon(*v, &mut pts)).collect();
        rings.push(ids);
    }
    let mut edges: Vec<(usize, usize)> = vec![];
    for ring in &rings {
        let n = ring.len();
        for i in 0..n {
            let (a, b) = (ring[i], ring[(i + 1) % n]);
            if a == b {
                continue;
            }
            let (pa, pb) = (pts[a], pts[b]);
            let l = len(sub(pb, pa));
            let d = unit(sub(pb, pa));
            let mut on: Vec<(f64, usize)> = vec![];
            for (k, q) in pts.iter().enumerate() {
                if k == a || k == b {
                    continue;
                }
                let s = dot(sub(*q, pa), d);
                if s > T && s < l - T && cross(d, sub(*q, pa)).abs() <= T {
                    on.push((s, k));
                }
            }
            on.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap_or(std::cmp::Ordering::Equal));
            let mut prev = a;
            for (_, k) in on {
                edges.push((prev, k));
                prev = k;
            }
            edges.push((prev, b));
        }
    }
    // cancel opposite pairs
    let mut alive = vec![true; edges.len()];
    let mut by_start: std::collections::HashMap<(usize, usize), Vec<usize>> = std::collections::HashMap::new();
    for (i, e) in edges.iter().enumerate() {
        by_start.entry(*e).or_default().push(i);
    }
    for i in 0..edges.len() {
        if !alive[i] {
            continue;
        }
        let (a, b) = edges[i];
        if let Some(list) = by_start.get_mut(&(b, a)) {
            while let Some(j) = list.pop() {
                if alive[j] && j != i {
                    alive[i] = false;
                    alive[j] = false;
                    break;
                }
            }
        }
    }
    let rest: Vec<(usize, usize)> = edges.iter().zip(&alive).filter(|(_, a)| **a).map(|(e, _)| *e).collect();
    let mut out_of: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
    for (i, e) in rest.iter().enumerate() {
        out_of.entry(e.0).or_default().push(i);
    }
    let mut used = vec![false; rest.len()];
    let mut loops: Vec<Vec<P>> = vec![];
    for s in 0..rest.len() {
        if used[s] {
            continue;
        }
        let mut lp: Vec<usize> = vec![];
        let mut cur = s;
        let mut guard = 0;
        loop {
            guard += 1;
            if guard > rest.len() + 2 {
                break;
            }
            used[cur] = true;
            lp.push(rest[cur].0);
            let at = rest[cur].1;
            if at == rest[s].0 {
                break;
            }
            let din = unit(sub(pts[rest[cur].1], pts[rest[cur].0]));
            let cands: Vec<usize> = out_of.get(&at).map(|v| v.iter().copied().filter(|&j| !used[j]).collect()).unwrap_or_default();
            // the most counter-clockwise turn keeps touching regions apart
            let best = cands.into_iter().max_by(|&x, &y| {
                let ang = |j: usize| {
                    let dout = unit(sub(pts[rest[j].1], pts[rest[j].0]));
                    cross(din, dout).atan2(dot(din, dout))
                };
                ang(x).partial_cmp(&ang(y)).unwrap_or(std::cmp::Ordering::Equal)
            });
            match best {
                Some(j) => cur = j,
                None => break,
            }
        }
        if lp.len() >= 3 {
            loops.push(lp.into_iter().map(|i| pts[i]).collect());
        }
    }
    let mut outers: Vec<Vec<P>> = loops.iter().filter(|l| area(l) > 0.0).cloned().collect();
    outers.sort_by(|a, b| area(b).partial_cmp(&area(a)).unwrap_or(std::cmp::Ordering::Equal));
    outers.extend(loops.into_iter().filter(|l| area(l) <= 0.0));
    outers
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BendKind {
    Flange,
    Fold,
}

/// A panel: a flat polygon (CCW, the tag of the edge leaving each vertex), its holes, and the bend it hangs on.
#[derive(Debug, Clone, PartialEq)]
pub struct Panel {
    pub poly: Poly,
    pub holes: Vec<Vec<P>>,
    /// The bend this panel hangs on (`None`: the root panel).
    pub via: Option<usize>,
}

impl Panel {
    pub fn area(&self) -> f64 {
        poly_area(&self.poly)
    }
    pub fn edge_with(&self, tag: &str) -> Option<usize> {
        self.poly.iter().position(|v| v.1 == tag)
    }
}

/// A bend: the strip `u` in `[z0, z1]` along `nu` from the line through `o` along `d`, between its parent and its child panel.
#[derive(Debug, Clone, PartialEq)]
pub struct TBend {
    pub feature: String,
    pub parent: usize,
    pub child: usize,
    pub o: P,
    pub d: P,
    pub nu: P,
    pub z: [f64; 2],
    /// Extent along `d` at `u = z0` (the parent side) and at `u = z1` (the child side).
    pub v0: [f64; 2],
    pub v1: [f64; 2],
    pub angle: f64,
    pub radius: f64,
    pub up: bool,
    pub kind: BendKind,
    /// Tag of the parent edge the strip hangs on, and of the child edge on its far side.
    pub attach: String,
    pub attach_child: String,
    pub hem: bool,
    /// Flange batch (legs of one batch keep a gap at the corners).
    pub batch: usize,
    /// Row `edge` of the bend table (the outline edge for a flange, the segment for a contour).
    pub edge: usize,
}

impl TBend {
    pub fn allowance(&self, rule: &Rule) -> f64 {
        (self.radius + rule.k_factor * rule.thickness) * self.angle
    }
    /// A flat point at `u` along the normal and `v` along the line.
    pub fn at(&self, u: f64, v: f64) -> P {
        add(add(self.o, mul(self.nu, u)), mul(self.d, v))
    }
    /// The zone strip as a polygon (CCW).
    pub fn zone_poly(&self) -> Vec<P> {
        let q = vec![self.at(self.z[0], self.v0[0]), self.at(self.z[0], self.v0[1]), self.at(self.z[1], self.v1[1]), self.at(self.z[1], self.v1[0])];
        if area(&q) < 0.0 {
            q.into_iter().rev().collect()
        } else {
            q
        }
    }
    /// The middle of the zone between the two ends: the start and end of the bend line.
    pub fn line(&self) -> (P, P) {
        let um = (self.z[0] + self.z[1]) * 0.5;
        let lo = (self.v0[0] + self.v1[0]) * 0.5;
        let hi = (self.v0[1] + self.v1[1]) * 0.5;
        (self.at(um, lo), self.at(um, hi))
    }
}

/// The sheet as a tree of panels.
#[derive(Debug, Clone, PartialEq)]
pub struct Tree {
    pub flat: Flat3,
    pub panels: Vec<Panel>,
    pub bends: Vec<TBend>,
    /// Folds made so far (for the tags of the edges along their zones).
    pub folds: usize,
    /// Corner reliefs made so far (for the tags of their edges).
    pub reliefs: usize,
    /// Cut-outs (`sheetCut`) in flat coordinates: already removed from the panels; the bends they cross carry them to the flat
    /// pattern and the solid.
    pub cuts: Vec<Vec<P>>,
}

fn rect_poly(o: P, d: P, nu: P, v: [f64; 2], u: [f64; 2], tags: [&str; 4]) -> Poly {
    // corners (v0,u0) (v1,u0) (v1,u1) (v0,u1); edge tags: zone side, end, tip, start
    let at = |vv: f64, uu: f64| add(add(o, mul(nu, uu)), mul(d, vv));
    ccw(vec![(at(v[0], u[0]), tags[0].into()), (at(v[1], u[0]), tags[1].into()), (at(v[1], u[1]), tags[2].into()), (at(v[0], u[1]), tags[3].into())])
}

impl Tree {
    /// The tree of a sheet made of a base with edge flanges or with folds.
    pub fn from_sheet(sheet: &Sheet, flat: Flat3) -> Result<Tree, String> {
        if !sheet.straight() {
            // curved outline: a plate with its arcs (runs of samples tagged `ra...`, true arcs in the solid); flanges on its straight edges
            if !sheet.flanges.is_empty() || !sheet.folds.is_empty() {
                return Err("the base outline has curved edges".into());
            }
            let poly: Poly = (0..sheet.outline.len()).map(|i| (sheet.outline[i], if sheet.line[i] { sheet.tags[i].clone() } else { format!("ra{}", sheet.tags[i]) })).collect();
            return Ok(Tree { flat, panels: vec![Panel { poly, holes: sheet.holes.clone(), via: None }], bends: vec![], folds: 0, reliefs: 0, cuts: vec![] });
        }
        if !sheet.folds.is_empty() {
            return Tree::from_folds(sheet, flat);
        }
        let q = sheet.straight_part();
        let n = q.len();
        let poly: Poly = (0..n).map(|i| (q[i], sheet.tags[i].clone())).collect();
        let mut tree = Tree { flat, panels: vec![Panel { poly, holes: sheet.holes.clone(), via: None }], bends: vec![], folds: 0, reliefs: 0, cuts: vec![] };
        for f in &sheet.flanges {
            let ch = sheet.chain(f);
            if ch.is_empty() {
                continue;
            }
            let e = sheet.edge(f.edge);
            let [x0, x1] = sheet.zone(f.edge, &q);
            let nu = [-e.m[0], -e.m[1]];
            let (mut parent, mut u_cur) = (0usize, -ch[0].b);
            for (j, bd) in ch.iter().enumerate() {
                let z = [u_cur, u_cur + bd.allowance];
                let ve = if j == 0 { [x0, x1] } else { f.leg };
                let (angle, up) = if j == 0 { (f.angle, f.up) } else { (f.then[j - 1].angle, f.up ^ f.then[j - 1].flip) };
                let child = tree.panels.len();
                let poly = rect_poly(e.a, e.d, nu, f.leg, [z[1], z[1] + bd.leg], ["z", "end", "tip", "start"]);
                tree.panels.push(Panel { poly, holes: vec![], via: Some(tree.bends.len()) });
                tree.bends.push(TBend {
                    feature: f.feature.clone(),
                    parent,
                    child,
                    o: e.a,
                    d: e.d,
                    nu,
                    z,
                    v0: ve,
                    v1: ve,
                    angle,
                    radius: f.radius,
                    up,
                    kind: BendKind::Flange,
                    attach: if j == 0 { sheet.tags[f.edge].clone() } else { "tip".into() },
                    attach_child: "z".into(),
                    hem: f.hem && j == 0,
                    batch: f.batch,
                    edge: f.edge,
                });
                parent = child;
                u_cur = z[1] + bd.leg;
            }
        }
        Ok(tree)
    }

    fn from_folds(sheet: &Sheet, flat: Flat3) -> Result<Tree, String> {
        let root = sheet.fold_root.ok_or("a folded base without a fixed panel")?;
        let lay = sheet_fold::layout(sheet, root)?;
        let mut panels: Vec<Panel> = lay
            .cells
            .iter()
            .map(|c| {
                let ring = pts_of(&c.poly);
                let holes = sheet.holes.iter().filter(|h| h.iter().all(|p| inside(&ring, *p, 1e-7))).cloned().collect();
                Panel { poly: c.poly.clone(), holes, via: None }
            })
            .collect();
        let mut bends: Vec<Option<TBend>> = vec![None; sheet.folds.len()];
        for (i, par) in lay.parent.iter().enumerate() {
            if let Some((pi, k)) = par {
                let f = &sheet.folds[*k];
                let ch = sheet_fold::chord(sheet, *k)?;
                bends[*k] = Some(TBend {
                    feature: f.feature.clone(),
                    parent: *pi,
                    child: i,
                    o: f.o,
                    d: f.d,
                    nu: f.nu,
                    z: f.z,
                    v0: ch,
                    v1: ch,
                    angle: f.angle,
                    radius: f.radius,
                    up: f.up,
                    kind: BendKind::Fold,
                    attach: format!("z{k}a"),
                    attach_child: format!("z{k}b"),
                    hem: false,
                    batch: 0,
                    edge: 0,
                });
                panels[i].via = Some(*k);
            }
        }
        let bends: Option<Vec<TBend>> = bends.into_iter().collect();
        let bends = bends.ok_or("the folds do not hang together as a chain of bends")?;
        Ok(Tree { flat, panels, bends, folds: sheet.folds.len(), reliefs: 0, cuts: vec![] })
    }

    /// The tree of a contour flange: one panel per segment, one bend per corner. `flat` must be the layout of the strip (see
    /// `Model::flat3`): `x` along the chain from its start, `y` along the extrusion.
    pub fn from_contour(c: &Contour, rule: &Rule, flat: Flat3) -> Result<Tree, String> {
        let plan = c.plan(rule)?;
        let (y0, y1) = if c.right == c.reverse { (-c.distance, 0.0) } else { (0.0, c.distance) };
        let mut panels: Vec<Panel> = vec![];
        let mut bends: Vec<TBend> = vec![];
        let mut x = 0.0;
        let n = c.segments();
        for i in 0..n {
            let leg = plan.legs[i];
            let first = i == 0;
            let last = i + 1 == n;
            let b_end = plan.bends.iter().find(|b| b.vertex == i + 1);
            // the edge along the extrusion start (flat y = 0) is `start`, the one at the far end of the extrusion `end`
            let (t0, t1) = if y0 == 0.0 { ("start", "end") } else { ("end", "start") };
            let ring: Poly = ccw(vec![
                ([x, y0], if first { "cs".to_string() } else { "z".to_string() }),
                ([x, y1], t1.to_string()),
                ([x + leg, y1], if last { "ce".to_string() } else { "z".to_string() }),
                ([x + leg, y0], t0.to_string()),
            ]);
            let idx = panels.len();
            panels.push(Panel { poly: ring, holes: vec![], via: if i == 0 { None } else { Some(bends.len() - 1) } });
            x += leg;
            if let Some(b) = b_end {
                bends.push(TBend {
                    feature: c.feature.clone(),
                    parent: idx,
                    child: idx + 1,
                    o: [x, 0.0],
                    d: [0.0, 1.0],
                    nu: [1.0, 0.0],
                    z: [0.0, b.allowance],
                    v0: [y0, y1],
                    v1: [y0, y1],
                    angle: b.angle,
                    radius: rule.bend_radius,
                    up: b.left_turn != c.right,
                    kind: BendKind::Flange,
                    attach: "z".into(),
                    attach_child: "z".into(),
                    hem: false,
                    batch: 0,
                    edge: i,
                });
                x += b.allowance;
            } else if !last {
                // a vertex without a bend (collinear): the next segment continues straight; the panel is separate but no bend
                return Err("a contour flange chain with a straight vertex".into());
            }
        }
        Ok(Tree { flat, panels, bends, folds: 0, reliefs: 0, cuts: vec![] })
    }

    // ------------------------------------------------------------------------------------------ flat pattern

    /// Recompute the extents of the bends along the edges they hang on (after an edge of a panel moved or was cut).
    pub fn refresh(&mut self) {
        for k in 0..self.bends.len() {
            let b = self.bends[k].clone();
            if let Some(e) = self.extent_on(b.parent, &b.attach, &b, b.z[0], b.v0) {
                self.bends[k].v0 = e;
                if b.kind == BendKind::Flange {
                    self.bends[k].v1 = e;
                }
            }
            if b.kind == BendKind::Fold {
                if let Some(e) = self.extent_on(b.child, &b.attach_child, &b, b.z[1], b.v1) {
                    self.bends[k].v1 = e;
                }
            }
        }
    }

    /// Extent along the bend line of the edges of `panel` with `tag` that lie on the line `u = zu`; of several, the one that
    /// overlaps `prefer` most.
    fn extent_on(&self, panel: usize, tag: &str, b: &TBend, zu: f64, prefer: [f64; 2]) -> Option<[f64; 2]> {
        let poly = &self.panels.get(panel)?.poly;
        let n = poly.len();
        // the hull of the edges with this tag on the line that touch the old extent (a cut across the edge splits it in two)
        let mut hull: Option<[f64; 2]> = None;
        for i in 0..n {
            if poly[i].1 != tag {
                continue;
            }
            let (pa, pb) = (poly[i].0, poly[(i + 1) % n].0);
            let u = |p: P| dot(sub(p, b.o), b.nu);
            if (u(pa) - zu).abs() > 1e-6 || (u(pb) - zu).abs() > 1e-6 {
                continue;
            }
            let (va, vb) = (dot(sub(pa, b.o), b.d), dot(sub(pb, b.o), b.d));
            let ext = [va.min(vb), va.max(vb)];
            if ext[1].min(prefer[1]) - ext[0].max(prefer[0]) < -1e-6 && hull.is_some() {
                continue;
            }
            hull = Some(match hull {
                Some(h) => [h[0].min(ext[0]), h[1].max(ext[1])],
                None => ext,
            });
        }
        hull
    }

    /// The flat pattern: panels and zones united.
    pub fn flat_pattern(&self, rule: &Rule, unfolded: bool) -> FlatPattern {
        let mut polys: Vec<Vec<P>> = vec![];
        for p in &self.panels {
            if p.area() > 1e-9 {
                polys.push(pts_of(&p.poly));
            }
        }
        let mut cut_holes: Vec<Vec<P>> = vec![];
        for b in &self.bends {
            let (pieces, holes) = self.zone_pieces(b);
            polys.extend(pieces);
            cut_holes.extend(holes);
        }
        let mut loops: Vec<Vec<P>> = merge_loops(&polys).into_iter().map(clean).collect();
        if loops.is_empty() {
            loops.push(vec![]);
        }
        loops.extend(cut_holes);
        for p in &self.panels {
            for h in &p.holes {
                loops.push(h.clone());
            }
        }
        let mut bends = vec![];
        let mut zones = vec![];
        for (k, b) in self.bends.iter().enumerate() {
            let (s, e) = b.line();
            let lo = (b.v0[0] + b.v1[0]) * 0.5;
            let hi = (b.v0[1] + b.v1[1]) * 0.5;
            bends.push(BendRow {
                index: k + 1,
                up: b.up,
                angle: b.angle.to_degrees(),
                included_angle: 180.0 - b.angle.to_degrees(),
                radius: b.radius,
                allowance: b.allowance(rule),
                length: hi - lo,
                start: s,
                end: e,
                feature: b.feature.clone(),
                edge: b.edge,
            });
            let q = b.zone_poly();
            let bb = q.iter().fold([f64::MAX, f64::MIN, f64::MAX, f64::MIN], |a, p| [a[0].min(p[0]), a[1].max(p[0]), a[2].min(p[1]), a[3].max(p[1])]);
            zones.push(bb);
        }
        let mut fp = FlatPattern::new(rule, loops, bends, zones);
        fp.unfolded = unfolded;
        fp
    }

    /// The zone of a bend less the cut-outs: the pieces left, and the holes a cut leaves inside it.
    pub fn zone_pieces(&self, b: &TBend) -> (Vec<Vec<P>>, Vec<Vec<P>>) {
        let zone = b.zone_poly();
        let mut pieces: Vec<Poly> = vec![zone.iter().map(|p| (*p, String::new())).collect()];
        let mut holes = vec![];
        for c in &self.cuts {
            let mut next = vec![];
            for pc in pieces {
                let (o, h) = subtract(&pc, c, "cut");
                next.extend(o);
                holes.extend(h);
            }
            pieces = next;
        }
        (pieces.iter().map(pts_of).collect(), holes)
    }

    /// Cut a region (flat coordinates) out of every panel it touches; the bends it crosses keep it for their zone.
    pub fn cut(&mut self, region: &[P]) -> Result<(), String> {
        let n = self.cuts.len();
        let tag = format!("cut{n}");
        let mut touched = false;
        let mut updates: Vec<(usize, Poly, Vec<Vec<P>>)> = vec![];
        for (i, pn) in self.panels.iter().enumerate() {
            if pn.area() < 1e-9 {
                continue;
            }
            let (outers, holes) = subtract(&pn.poly, region, &tag);
            let overlap = (pn.area() - outers.iter().map(poly_area).sum::<f64>()).abs() > 1e-9 || !holes.is_empty();
            if !overlap {
                continue;
            }
            touched = true;
            match outers.len() {
                0 => return Err(format!("the cut removes the whole panel P{i}")),
                1 => {}
                _ => return Err(format!("the cut splits panel P{i} in pieces: cut a smaller region, or fold after cutting")),
            }
            updates.push((i, outers.into_iter().next().unwrap_or_default(), holes));
        }
        for b in &self.bends {
            let zone: Poly = b.zone_poly().into_iter().map(|p| (p, String::new())).collect();
            let (o, h) = subtract(&zone, region, "cut");
            if (poly_area(&zone) - o.iter().map(poly_area).sum::<f64>()).abs() > 1e-9 || !h.is_empty() {
                touched = true;
            }
        }
        if !touched {
            return Err("the cut misses the sheet".into());
        }
        for (i, poly, holes) in updates {
            self.panels[i].poly = poly;
            self.panels[i].holes.extend(holes);
        }
        self.cuts.push(region.to_vec());
        self.refresh();
        Ok(())
    }

    /// The panel indices from the root to `panel`, the bends on the way.
    pub fn path(&self, mut panel: usize) -> Vec<usize> {
        let mut out = vec![];
        while let Some(k) = self.panels[panel].via {
            out.push(k);
            panel = self.bends[k].parent;
        }
        out.reverse();
        out
    }

    /// The mirror image of the tree in a flat reflection (`pt` maps points, `vec` directions; a reflection that keeps the layout
    /// plane, so the bends keep their angle and side). The panels run the other way round after a reflection, so every ring is
    /// reversed (the tag of an edge travels with it); a bend keeps its zone and its extents because its line and normal are
    /// reflected together.
    pub fn reflected(&self, flat: Flat3, pt: impl Fn(P) -> P, vec: impl Fn(P) -> P) -> Tree {
        let ring = |r: &Vec<P>| -> Vec<P> { r.iter().rev().map(|p| pt(*p)).collect() };
        let panels = self
            .panels
            .iter()
            .map(|p| {
                let m: Poly = p.poly.iter().map(|(q, t)| (pt(*q), t.clone())).collect();
                let n = m.len();
                let poly: Poly = (0..n).map(|j| (m[n - 1 - j].0, m[(n + n - 2 - j) % n].1.clone())).collect();
                Panel { poly, holes: p.holes.iter().map(&ring).collect(), via: p.via }
            })
            .collect();
        let bends = self.bends.iter().map(|b| TBend { o: pt(b.o), d: vec(b.d), nu: vec(b.nu), ..b.clone() }).collect();
        Tree { flat, panels, bends, folds: self.folds, reliefs: self.reliefs, cuts: self.cuts.iter().map(&ring).collect() }
    }

    /// The tree and its mirror image `other` (the same tree reflected across the flat line through `c` with unit normal `m`, in
    /// the same layout) as one sheet: the two base plates meet along an edge of the base that lies on the line, and that edge must
    /// not carry a bend. The plates become one panel and every other panel and bend of the image hangs on it.
    pub fn joined_mirror(&self, other: &Tree, c: P, m: P) -> Result<Tree, String> {
        const TOL: f64 = 1e-6;
        let roots = |t: &Tree| t.panels.iter().enumerate().filter(|(_, p)| p.via.is_none()).map(|(i, _)| i).collect::<Vec<_>>();
        let (r, r2) = match (roots(self).as_slice(), roots(other).as_slice()) {
            ([a], [b]) => (*a, *b),
            _ => return Err("the sheet has more than one base panel".into()),
        };
        let dist = |p: P| dot(sub(p, c), m);
        let (pn, pn2) = (&self.panels[r], &other.panels[r2]);
        let n = pn.poly.len();
        let on: Vec<usize> = (0..n).filter(|&i| dist(pn.poly[i].0).abs() <= TOL).collect();
        let e = match on.as_slice() {
            [a, b] if (*a + 1) % n == *b => *a,
            [a, b] if (*b + 1) % n == *a => *b,
            [] => return Err("the base plate does not reach the mirror plane with an edge".into()),
            _ => return Err("the base plate touches the mirror plane at more than one straight edge".into()),
        };
        let side = pn.poly.iter().map(|(q, _)| dist(*q)).find(|d| d.abs() > TOL).unwrap_or(0.0).signum();
        if side == 0.0 || pn.poly.iter().any(|(q, _)| dist(*q) * side < -TOL) {
            return Err("the base plate lies across the mirror plane".into());
        }
        let tag = pn.poly[e].1.clone();
        if pn.poly.iter().filter(|v| v.1 == tag).count() != 1 || self.bends.iter().any(|b| b.parent == r && b.attach == tag) {
            return Err("the edge of the base plate on the mirror plane carries a bend".into());
        }
        let (a, b) = (pn.poly[e].0, pn.poly[(e + 1) % n].0);
        let n2 = pn2.poly.len();
        let j = (0..n2)
            .find(|&j| len(sub(pn2.poly[j].0, b)) <= TOL && len(sub(pn2.poly[(j + 1) % n2].0, a)) <= TOL)
            .ok_or_else(|| "the mirror image of the base plate does not meet the plate along its edge".to_string())?;
        // B .. A of the plate, then the image from A round to B
        let mut ring: Poly = (1..=n).map(|k| pn.poly[(e + k) % n].clone()).collect();
        ring.last_mut().map(|l| l.1 = pn2.poly[(j + 1) % n2].1.clone());
        ring.extend((2..n2).map(|k| pn2.poly[(j + k) % n2].clone()));
        // the two plates continue straight over the seam: no corner there
        let mut k = 0;
        while k < ring.len() && ring.len() > 3 {
            let (pv, cu, nx) = (ring[(k + ring.len() - 1) % ring.len()].clone(), ring[k].clone(), ring[(k + 1) % ring.len()].0);
            let (d1, d2) = (sub(cu.0, pv.0), sub(nx, cu.0));
            let at_seam = len(sub(cu.0, a)) <= TOL || len(sub(cu.0, b)) <= TOL;
            if at_seam && pv.1 == cu.1 && cross(d1, d2).abs() <= 1e-9 * len(d1) * len(d2) && dot(d1, d2) > 0.0 {
                ring.remove(k);
            } else {
                k += 1;
            }
        }
        let mut holes = pn.holes.clone();
        holes.extend(pn2.holes.iter().cloned());
        let mut panels = self.panels.clone();
        panels[r] = Panel { poly: ring, holes, via: None };
        let (off, boff) = (self.panels.len(), self.bends.len());
        let map = |i: usize| if i == r2 { r } else { off + i - usize::from(i > r2) };
        for (i, p) in other.panels.iter().enumerate().filter(|(i, _)| *i != r2) {
            let _ = i;
            panels.push(Panel { via: p.via.map(|k| k + boff), ..p.clone() });
        }
        let mut bends = self.bends.clone();
        bends.extend(other.bends.iter().map(|b| TBend { parent: map(b.parent), child: map(b.child), ..b.clone() }));
        let mut cuts = self.cuts.clone();
        cuts.extend(other.cuts.iter().cloned());
        Ok(Tree { flat: self.flat, panels, bends, folds: self.folds.max(other.folds), reliefs: self.reliefs.max(other.reliefs), cuts })
    }

    /// The tag of an edge of a panel that no wall roles should name (the strip side of a bend, tags of zones).
    pub fn is_zone_tag(tag: &str) -> bool {
        tag == "z" || (tag.starts_with('z') && tag[1..].chars().next().map_or(false, |c| c.is_ascii_digit()))
    }

    /// Edge `i` of `panel` has a bend hanging on it (as parent), if any.
    pub fn bend_on_edge(&self, panel: usize, tag: &str) -> Option<usize> {
        self.bends.iter().position(|b| b.parent == panel && b.attach == tag)
    }

    // ------------------------------------------------------------------------------------------ flanges

    /// Move the edge `e` of a panel inward by `b`; its neighbours follow (the corners are where the moved line meets them).
    fn move_edge(&mut self, p: usize, e: usize, b: f64) {
        if b.abs() < 1e-12 {
            return;
        }
        let poly = &mut self.panels[p].poly;
        let n = poly.len();
        let (a, bb) = (poly[e].0, poly[(e + 1) % n].0);
        let d = unit(sub(bb, a));
        let m = [-d[1], d[0]];
        let s0 = add(a, mul(m, b));
        let pe = (e + n - 1) % n;
        let dprev = unit(sub(a, poly[pe].0));
        let dnext = unit(sub(poly[(e + 2) % n].0, bb));
        let meet = |p0: P, d0: P| -> P {
            let den = cross(d0, d);
            if den.abs() < 1e-9 {
                return add(p0, mul(m, b));
            }
            let s = cross(sub(s0, p0), d) / den;
            add(p0, mul(d0, s))
        };
        let na = meet(poly[pe].0, dprev);
        let nb = meet(bb, dnext);
        poly[e].0 = na;
        poly[(e + 1) % n].0 = nb;
    }

    /// Add the flanges of one `edgeFlange` / `hem` feature. Every request names a panel and an edge of it (by index into its ring).
    pub fn add_flanges(&mut self, rule: &Rule, feature: &str, batch: usize, reqs: &[FlangeReq]) -> Result<(), String> {
        let t = rule.thickness;
        struct Pending {
            panel: usize,
            tag: String,
            o: P,
            d: P,
            nu: P,
            ch: Vec<Bend>,
            req: usize,
            len0: f64,
            zone: [f64; 2],
            leg: [f64; 2],
            span: Option<[f64; 2]>,
            reach: f64,
            carved: bool,
        }
        let mut pend: Vec<Pending> = vec![];
        for (ri, r) in reqs.iter().enumerate() {
            let n = self.panels[r.panel].poly.len();
            if r.edge >= n {
                return Err(format!("panel {} has no edge {}", r.panel, r.edge));
            }
            let tag = self.panels[r.panel].poly[r.edge].1.clone();
            if Tree::is_zone_tag(&tag) {
                return Err("that edge is the side of a bend zone".into());
            }
            let partial = r.span.is_some() || r.profile.is_some();
            if !partial {
                if self.bend_on_edge(r.panel, &tag).is_some() || pend.iter().any(|p| p.panel == r.panel && p.tag == tag) {
                    return Err(format!("the edge `{tag}` of panel {} already has a flange", r.panel));
                }
                if self.panels[r.panel].poly.iter().filter(|v| v.1 == tag).count() > 1 {
                    return Err(format!("the edge `{tag}` of panel {} is split by a partial flange: give the flange its span (start and length)", r.panel));
                }
            }
            let ch = chain_of(t, rule.k_factor, r.radius, r.angle, r.height, r.from, r.pos, r.hem, &r.then).map_err(|(j, min)| format!("flangeHeightTooLow:{j}:{min}"))?;
            let (a, bb) = (self.panels[r.panel].poly[r.edge].0, self.panels[r.panel].poly[(r.edge + 1) % n].0);
            let d = unit(sub(bb, a));
            let nu = [d[1], -d[0]];
            let reach = r.height - ch[0].leg;
            // the span: given, else the root of the profile (its width at the end of the bend)
            let mut span = r.span;
            if let (None, Some(pr)) = (span, &r.profile) {
                span = Some(profile_root(pr, reach)?);
            }
            if let Some(sp) = span {
                if !(sp[1] - sp[0] > 1e-6) {
                    return Err("the flange has no width".into());
                }
            }
            pend.push(Pending { panel: r.panel, tag, o: a, d, nu, ch, req: ri, len0: len(sub(bb, a)), zone: [0.0, 0.0], leg: [0.0, 0.0], span, reach, carved: false });
        }
        // partial width: carve the strip the flange hangs on out of its edge (a notch beside it where the bend starts inside)
        let dup: Vec<bool> = (0..pend.len()).map(|i| pend.iter().enumerate().any(|(j, q)| j != i && q.panel == pend[i].panel && q.tag == pend[i].tag)).collect();
        for (pi, p) in pend.iter_mut().enumerate() {
            let Some(sp) = p.span else { continue };
            let r = &reqs[p.req];
            let whole = {
                let poly = &self.panels[p.panel].poly;
                let same: Vec<usize> = (0..poly.len()).filter(|&i| poly[i].1 == p.tag).collect();
                same.len() == 1 && {
                    let e = same[0];
                    let (va, vb) = (dot(sub(poly[e].0, p.o), p.d), dot(sub(poly[(e + 1) % poly.len()].0, p.o), p.d));
                    sp[0] <= va + 1e-4 && sp[1] >= vb - 1e-4
                }
            };
            if whole && !self.bend_on_edge(p.panel, &p.tag).is_some() && !dup[pi] {
                continue;
            }
            self.reliefs += 1;
            let strip_tag = format!("{}_s{}", p.tag, self.reliefs);
            self.carve_strip(p.panel, &p.tag, (p.o, p.d), sp, p.ch[0].b, r.relief, &strip_tag, self.reliefs)?;
            p.tag = strip_tag;
            p.carved = true;
        }
        // move the edges in
        for p in &pend {
            if p.carved {
                continue;
            }
            let e = self.panels[p.panel].edge_with(&p.tag).ok_or("edge lost")?;
            self.move_edge(p.panel, e, p.ch[0].b);
        }
        // corner reliefs between flanges beside each other
        if rule.corner_relief_shape != ReliefShape::Notch {
            let tags: Vec<(usize, String)> = pend.iter().map(|p| (p.panel, p.tag.clone())).collect();
            let mut panels: Vec<usize> = tags.iter().map(|x| x.0).collect();
            panels.sort_unstable();
            panels.dedup();
            for pn in panels {
                self.relieve_corners(pn, rule, &tags)?;
            }
        }
        // zone extents
        for p in pend.iter_mut() {
            let e = self.panels[p.panel].edge_with(&p.tag).ok_or("edge lost")?;
            let n = self.panels[p.panel].poly.len();
            let (a, bb) = (self.panels[p.panel].poly[e].0, self.panels[p.panel].poly[(e + 1) % n].0);
            let (va, vb) = (dot(sub(a, p.o), p.d), dot(sub(bb, p.o), p.d));
            if vb - va < 1e-6 {
                return Err(format!("the flange on `{}` leaves no width", p.tag));
            }
            p.zone = [va, vb];
            p.leg = [va, vb];
        }
        // legs: the zone, and with a notch corner the ends beside a flange of the same feature (both bends 90 degrees, edges at
        // right angles) pulled back so that the two legs stay `gap` apart
        if rule.corner_relief_shape == ReliefShape::Notch {
            let right = |a: f64| (a - std::f64::consts::FRAC_PI_2).abs() < 1e-6;
            let mut legs: Vec<[f64; 2]> = vec![];
            for p in pend.iter() {
                let me = &reqs[p.req];
                if me.profile.is_some() {
                    legs.push(p.zone);
                    continue;
                }
                let poly = &self.panels[p.panel].poly;
                let n = poly.len();
                let e = poly.iter().position(|v| v.1 == p.tag).unwrap_or(0);
                let nb = |j: usize| -> Option<f64> {
                    let tg = &poly[j].1;
                    let q = pend.iter().find(|q| q.panel == p.panel && q.tag == *tg)?;
                    let o = &reqs[q.req];
                    if !right(o.angle) || !right(me.angle) || dot(p.d, q.d).abs() > 1e-6 {
                        return None;
                    }
                    Some(q.ch[0].b - o.radius + rule.gap / std::f64::consts::SQRT_2)
                };
                let mut z = p.zone;
                if let Some(c) = nb((e + n - 1) % n) {
                    z[0] = c.min(z[1]);
                }
                if let Some(c) = nb((e + 1) % n) {
                    z[1] = (p.len0 - c).max(z[0]);
                }
                legs.push(z);
            }
            for (p, l) in pend.iter_mut().zip(legs) {
                p.leg = l;
            }
        }
        // the panels and bends, flange by flange
        let first_new = self.panels.len();
        for p in pend.iter() {
            let r = &reqs[p.req];
            let (mut parent, mut u_cur) = (p.panel, -p.ch[0].b);
            for (j, bd) in p.ch.iter().enumerate() {
                let z = [u_cur, u_cur + bd.allowance];
                let ve = if j == 0 { p.zone } else { p.leg };
                let (angle, up) = if j == 0 { (r.angle, r.up) } else { (r.then[j - 1].angle, r.up ^ r.then[j - 1].flip) };
                let child = self.panels.len();
                let poly = match (&r.profile, j) {
                    (Some(pr), 0) => profile_leg(pr, p.o, p.d, p.nu, z[1], p.reach)?,
                    _ => rect_poly(p.o, p.d, p.nu, p.leg, [z[1], z[1] + bd.leg], ["z", "end", "tip", "start"]),
                };
                let mut holes: Vec<Vec<P>> = vec![];
                if j == 0 && r.profile.is_some() {
                    for h in &r.profile_holes {
                        if h.iter().any(|q| q[1] < p.reach + 1e-7) {
                            return Err("a hole in the flange reaches the bend: keep it beyond the end of the bend".into());
                        }
                        holes.push(h.iter().map(|q| add(add(p.o, mul(p.d, q[0])), mul(p.nu, z[1] + (q[1] - p.reach)))).collect());
                    }
                }
                self.panels.push(Panel { poly, holes, via: Some(self.bends.len()) });
                self.bends.push(TBend {
                    feature: feature.to_string(),
                    parent,
                    child,
                    o: p.o,
                    d: p.d,
                    nu: p.nu,
                    z,
                    v0: ve,
                    v1: ve,
                    angle,
                    radius: r.radius,
                    up,
                    kind: BendKind::Flange,
                    attach: if j == 0 { p.tag.clone() } else { "tip".into() },
                    attach_child: "z".into(),
                    hem: r.hem && j == 0,
                    batch,
                    edge: r.edge,
                });
                parent = child;
                u_cur = z[1] + bd.leg;
            }
        }
        self.refresh();
        // the new legs must not run into anything in the flat layout
        for i in first_new..self.panels.len() {
            if self.panels[i].area() < 1e-6 {
                continue;
            }
            for j in 0..self.panels.len() {
                if j == i || self.panels[j].area() < 1e-6 || (j >= first_new && j < i) {
                    continue;
                }
                let ov = overlap_area(&self.panels[j].poly, &self.panels[i].poly);
                if ov > 1e-4 {
                    return Err(format!("the flange leg P{i} overlaps panel P{j} in the flat pattern ({ov:.3} mm2): give the corner a relief, or shorten the flange"));
                }
            }
        }
        Ok(())
    }

    /// Carve the strip `span` (v along the edge line `(o, d)`) out of the edge of `pn` tagged `tag` that holds it: the strip becomes
    /// an edge of its own tagged `strip_tag`, `b` inside the old line (the bend starts `b` inside). Where the strip stops inside
    /// the edge, walls (tagged `rls<k>..`) join it to the old line, with the rectangular relief `(width, depth)` if given; where it
    /// reaches the end of the edge, the neighbouring edge follows the strip (the corner is where the two lines meet).
    #[allow(clippy::too_many_arguments)]
    fn carve_strip(&mut self, pn: usize, tag: &str, (o, d): (P, P), span: [f64; 2], b: f64, relief: Option<(f64, f64)>, strip_tag: &str, k: usize) -> Result<(), String> {
        const EPS: f64 = 1e-4;
        let n = self.panels[pn].poly.len();
        let m = [-d[1], d[0]];
        let at = |v: f64, u: f64| add(add(o, mul(d, v)), mul(m, u));
        let poly = &self.panels[pn].poly;
        let e = (0..n)
            .find(|&i| {
                let (pa, pb) = (poly[i].0, poly[(i + 1) % n].0);
                let (ua, ub) = (dot(sub(pa, o), m), dot(sub(pb, o), m));
                let (va, vb) = (dot(sub(pa, o), d), dot(sub(pb, o), d));
                poly[i].1 == tag && ua.abs() < EPS && ub.abs() < EPS && va <= span[0] + EPS && vb >= span[1] - EPS
            })
            .ok_or_else(|| format!("the span {:.3} to {:.3} is not on a free part of the edge `{tag}` (outside it, or a flange is there already)", span[0], span[1]))?;
        // the edge first
        let mut ring: Poly = (0..n).map(|i| poly[(e + i) % n].clone()).collect();
        let (a, bb) = (ring[0].0, ring[1].0);
        let (ea, eb) = (dot(sub(a, o), d), dot(sub(bb, o), d));
        let span = [span[0].max(ea), span[1].min(eb)];
        let touch0 = span[0] - ea < EPS;
        let touch1 = eb - span[1] < EPS;
        let meet = |p0: P, d0: P| -> P {
            // where the line through `p0` along `d0` meets the line `b` inside the edge
            let den = cross(d0, d);
            if den.abs() < 1e-9 {
                return add(p0, mul(m, b));
            }
            let s0 = at(0.0, b);
            let s = cross(sub(s0, p0), d) / den;
            add(p0, mul(d0, s))
        };
        let (rw, rd) = relief.filter(|r| r.0 > 1e-9 && r.1 > 1e-9).map_or((0.0, 0.0), |r| r);
        let depth = b + rd;
        let tg = |c: &str| format!("rls{k}{c}");
        let mut s: Poly = vec![];
        if touch0 {
            let prev = ring[n - 1].0;
            let moved = if b.abs() < 1e-12 { a } else { meet(prev, unit(sub(a, prev))) };
            s.push((moved, strip_tag.to_string()));
        } else {
            let wa = if rd > 0.0 { (span[0] - rw).max(ea) } else { span[0] };
            s.push((a, tag.to_string()));
            s.push((at(wa, 0.0), tg("a")));
            s.push((at(wa, depth.max(b)), tg("b")));
            if rd > 0.0 {
                s.push((at(span[0], depth), tg("c")));
            }
            s.push((at(span[0], b), strip_tag.to_string()));
        }
        let next_tag = ring[1].1.clone();
        if touch1 {
            let nx = ring[2 % n].0;
            let moved = if b.abs() < 1e-12 { bb } else { meet(nx, unit(sub(bb, nx))) };
            s.push((moved, next_tag));
        } else {
            let we = if rd > 0.0 { (span[1] + rw).min(eb) } else { span[1] };
            s.push((at(span[1], b), tg("d")));
            if rd > 0.0 {
                s.push((at(span[1], depth), tg("e")));
            }
            s.push((at(we, depth.max(b)), tg("f")));
            s.push((at(we, 0.0), tag.to_string()));
            s.push((bb, next_tag));
        }
        s.extend(ring.drain(2..));
        self.panels[pn].poly = tidy(s);
        Ok(())
    }

    /// Corners of a panel where two flanged edges meet get the rule's relief shape (at least one of the two flanges is new).
    fn relieve_corners(&mut self, pn: usize, rule: &Rule, new_tags: &[(usize, String)]) -> Result<(), String> {
        let mut done: Vec<(String, String)> = vec![];
        loop {
            let poly = &self.panels[pn].poly;
            let n = poly.len();
            let flanged = |tag: &str| self.bend_on_edge(pn, tag).is_some() || new_tags.iter().any(|(p, t)| *p == pn && t == tag);
            let is_new = |tag: &str| new_tags.iter().any(|(p, t)| *p == pn && t == tag);
            let found = (0..n).find(|&k| {
                let (tp, tn) = (&poly[(k + n - 1) % n].1, &poly[k].1);
                tp != tn && flanged(tp) && flanged(tn) && (is_new(tp) || is_new(tn)) && !done.contains(&(tp.clone(), tn.clone()))
            });
            let Some(k) = found else { return Ok(()) };
            done.push((poly[(k + n - 1) % n].1.clone(), poly[k].1.clone()));
            self.cut_corner(pn, k, rule)?;
        }
    }

    /// Replace the corner at vertex `k` by the relief shape of the rule. A corner that is not convex is left alone.
    fn cut_corner(&mut self, pn: usize, k: usize, rule: &Rule) -> Result<(), String> {
        let poly = &self.panels[pn].poly;
        let n = poly.len();
        let (vp, vk, vn) = (poly[(k + n - 1) % n].0, poly[k].0, poly[(k + 1) % n].0);
        let (dprev, dnext) = (unit(sub(vk, vp)), unit(sub(vn, vk)));
        if cross(dprev, dnext) <= 1e-9 {
            return Ok(());
        }
        let (lp, ln) = (len(sub(vk, vp)), len(sub(vn, vk)));
        let tag_k = poly[k].1.clone();
        let id = self.reliefs;
        let rho = rule.corner_relief * 0.5;
        let fit = |x: f64| -> Result<(), String> {
            if x < lp - 1e-6 && x < ln - 1e-6 {
                Ok(())
            } else {
                Err(format!("the corner relief ({x:.3} mm from the corner) is longer than the edge beside it: make the relief smaller"))
            }
        };
        let path: Vec<(P, String)> = match rule.corner_relief_shape {
            ReliefShape::Notch => return Ok(()),
            ReliefShape::Rectangular => {
                fit(rho)?;
                let p1 = sub(vk, mul(dprev, rho));
                vec![(p1, format!("rl{id}a")), (add(p1, mul(dnext, rho)), format!("rl{id}b")), (add(vk, mul(dnext, rho)), tag_k)]
            }
            ReliefShape::Round => {
                fit(rho)?;
                let a1 = (-dprev[1]).atan2(-dprev[0]);
                let a2 = dnext[1].atan2(dnext[0]);
                let mut span = (a1 - a2) % (2.0 * std::f64::consts::PI);
                if span < 0.0 {
                    span += 2.0 * std::f64::consts::PI;
                }
                let steps = ((span / std::f64::consts::FRAC_PI_2 * 16.0).ceil() as usize).max(4);
                let tag = format!("ra{id}");
                (0..=steps)
                    .map(|s| {
                        let a = a1 - span * s as f64 / steps as f64;
                        (add(vk, mul([a.cos(), a.sin()], rho)), if s == steps { tag_k.clone() } else { tag.clone() })
                    })
                    .collect()
            }
            ReliefShape::Tear => {
                let w = rule.relief_width.max(1e-3);
                let bis = unit(sub(dnext, dprev));
                let perp = [-bis[1], bis[0]];
                let ql = add(vk, mul(perp, w * 0.5));
                let qr = sub(vk, mul(perp, w * 0.5));
                let meet = |p0: P, d0: P, p1: P, d1: P| -> Option<P> {
                    let den = cross(d0, d1);
                    if den.abs() < 1e-9 {
                        return None;
                    }
                    Some(add(p0, mul(d0, cross(sub(p1, p0), d1) / den)))
                };
                let p1 = meet(ql, bis, vp, dprev).ok_or("the tear relief does not meet the edge")?;
                let p2 = meet(qr, bis, vk, dnext).ok_or("the tear relief does not meet the edge")?;
                if dot(sub(vk, p1), dprev) < 1e-9 || dot(sub(p2, vk), dnext) < 1e-9 {
                    return Err("the tear relief is wider than the corner it cuts".into());
                }
                fit(dot(sub(vk, p1), dprev).max(dot(sub(p2, vk), dnext)))?;
                let depth = rho.max(dot(sub(p1, vk), bis).max(dot(sub(p2, vk), bis)) + w);
                let (e1, e2) = (add(ql, mul(bis, depth)), add(qr, mul(bis, depth)));
                vec![(p1, format!("rl{id}a")), (e1, format!("rl{id}b")), (e2, format!("rl{id}c")), (p2, tag_k)]
            }
        };
        self.reliefs += 1;
        self.panels[pn].poly.splice(k..k + 1, path);
        Ok(())
    }

    // ------------------------------------------------------------------------------------------ corners

    /// Round (`radius`) or chamfer (`distance`) the corner at vertex `k` of a panel. Both edges must be free walls (no bend on them)
    /// and the corner convex.
    pub fn round_corner(&mut self, pn: usize, k: usize, chamfer: bool, size: f64) -> Result<(), String> {
        let poly = &self.panels[pn].poly;
        let n = poly.len();
        if k >= n {
            return Err(format!("panel {pn} has no vertex {k}"));
        }
        let (vp, vk, vn) = (poly[(k + n - 1) % n].0, poly[k].0, poly[(k + 1) % n].0);
        let (tp, tk) = (poly[(k + n - 1) % n].1.clone(), poly[k].1.clone());
        for tg in [&tp, &tk] {
            if Tree::is_zone_tag(tg) || self.bend_on_edge(pn, tg).is_some() {
                return Err(format!("the corner of panel P{pn} touches the bend side `{tg}`: round the corners of free walls"));
            }
        }
        let (dprev, dnext) = (unit(sub(vk, vp)), unit(sub(vn, vk)));
        let c = cross(dprev, dnext);
        if c <= 1e-9 {
            return Err("only convex corners can be rounded".into());
        }
        let turn = c.atan2(dot(dprev, dnext));
        let td = if chamfer { size } else { size * (turn / 2.0).tan() };
        if td >= len(sub(vk, vp)) - 1e-6 || td >= len(sub(vn, vk)) - 1e-6 {
            return Err(format!("the corner size {size} needs {td:.3} mm of each wall: the walls beside the corner are shorter"));
        }
        let id = self.reliefs;
        self.reliefs += 1;
        let (p1, p2) = (sub(vk, mul(dprev, td)), add(vk, mul(dnext, td)));
        let path: Vec<(P, String)> = if chamfer {
            vec![(p1, format!("rl{id}a")), (p2, tk)]
        } else {
            let ctr = add(p1, mul([-dprev[1], dprev[0]], size));
            let a1 = (p1[1] - ctr[1]).atan2(p1[0] - ctr[0]);
            let steps = ((turn / std::f64::consts::FRAC_PI_2 * 16.0).ceil() as usize).max(4);
            let tag = format!("ra{id}");
            (0..=steps)
                .map(|s| {
                    let a = a1 + turn * s as f64 / steps as f64;
                    (add(ctr, mul([a.cos(), a.sin()], size)), if s == steps { tk.clone() } else { tag.clone() })
                })
                .collect()
        };
        self.panels[pn].poly.splice(k..k + 1, path);
        Ok(())
    }

    // ------------------------------------------------------------------------------------------ folds

    /// Fold a panel along a line. The panel falls into the root piece (the side that stays: the one the panel hangs on, else the
    /// side of `fixed`, else the larger), the bend zone, and the pieces on the other side of the line, which move.
    pub fn add_fold(&mut self, rule: &Rule, feature: &str, req: &FoldReq) -> Result<(), String> {
        let p = req.panel;
        let t = rule.thickness;
        let ba = (req.radius + rule.k_factor * t) * req.angle;
        let sb = (req.radius + t) * (req.angle / 2.0).tan();
        let d = unit(req.d);
        let nu0 = [-d[1], d[0]];
        let u0 = |q: P| dot(sub(q, req.o), nu0);
        let forced: Option<f64> = match self.panels[p].via {
            Some(k) => {
                let b = &self.bends[k];
                Some(u0(b.at(b.z[1], (b.v1[0] + b.v1[1]) * 0.5)))
            }
            None => req.fixed.map(u0),
        };
        if let Some(u) = forced {
            if u.abs() < 1e-9 {
                return Err("the side that stays lies on the fold line".into());
            }
        }
        let sides: Vec<f64> = match forced {
            Some(u) => vec![u.signum()],
            None => {
                let off = dot(nu0, req.o);
                let area_of = |n: P, c: f64| clip_half(&self.panels[p].poly, n, c, "").iter().map(poly_area).sum::<f64>();
                let (ap, an) = (area_of(nu0, off), area_of([-nu0[0], -nu0[1]], -off));
                if ap > an + 1e-9 {
                    vec![1.0, -1.0]
                } else {
                    vec![-1.0, 1.0]
                }
            }
        };
        let mut last = String::new();
        for s in sides {
            match self.fold_plan(feature, req, d, nu0, s, zone_of(req.position, ba, sb), forced) {
                Ok(plan) => return self.apply_fold(feature, req, plan),
                Err(e) => last = e,
            }
        }
        Err(last)
    }

    #[allow(clippy::too_many_arguments)]
    fn fold_plan(&self, _feature: &str, req: &FoldReq, d: P, nu0: P, s: f64, z: [f64; 2], forced: Option<f64>) -> Result<FoldPlan, String> {
        let p = req.panel;
        let nu = if s > 0.0 { [-nu0[0], -nu0[1]] } else { nu0 };
        let off = dot(nu, req.o);
        let u = |q: P| dot(nu, q) - off;
        if let Some(f) = forced {
            if -s * f > z[0] - 1e-9 {
                return Err("the side that stays lies inside the bend zone".into());
            }
        }
        let (ta, tb) = (format!("z{}a", self.folds), format!("z{}b", self.folds));
        let poly = &self.panels[p].poly;
        let a = clip_half(poly, [-nu[0], -nu[1]], -(z[0] + off), &ta);
        let b = clip_half(poly, nu, z[1] + off, &tb);
        let strips: Vec<Poly> = clip_half(poly, nu, z[0] + off, "s").iter().flat_map(|q| clip_half(q, [-nu[0], -nu[1]], -(z[1] + off), "s")).collect();
        if a.is_empty() || b.is_empty() || strips.is_empty() {
            return Err("the fold line does not cross the panel".into());
        }
        if a.len() > 1 {
            return Err("the fold line crosses the panel in several separate places, so the bends cannot all hang on one root piece: choose the side that stays (the fixed point) so that the bends hang on one root piece, or fold with a line that crosses the panel once".into());
        }
        let mut conn: Vec<(usize, [f64; 2], [f64; 2])> = vec![];
        for sp in &strips {
            if sp.iter().any(|(q, _)| (u(*q) - z[0]).abs() > 1e-6 && (u(*q) - z[1]).abs() > 1e-6) {
                return Err("a corner of the panel lies inside the bend zone: move the fold line".into());
            }
            let ext = |zu: f64| -> [f64; 2] {
                let vs: Vec<f64> = sp.iter().filter(|(q, _)| (u(*q) - zu).abs() <= 1e-6).map(|(q, _)| dot(sub(*q, req.o), d)).collect();
                [vs.iter().cloned().fold(f64::MAX, f64::min), vs.iter().cloned().fold(f64::MIN, f64::max)]
            };
            let (e0, e1) = (ext(z[0]), ext(z[1]));
            if e0[0] > e0[1] || e1[0] > e1[1] {
                return Err("the bend zone does not reach both sides of the fold".into());
            }
            let j = b.iter().position(|bp| {
                let n = bp.len();
                (0..n).any(|i| {
                    if bp[i].1 != tb {
                        return false;
                    }
                    let (qa, qb) = (bp[i].0, bp[(i + 1) % n].0);
                    if (u(qa) - z[1]).abs() > 1e-6 || (u(qb) - z[1]).abs() > 1e-6 {
                        return false;
                    }
                    let (va, vb) = (dot(sub(qa, req.o), d), dot(sub(qb, req.o), d));
                    va.max(vb).min(e1[1]) - va.min(vb).max(e1[0]) > 1e-6
                })
            });
            match j {
                Some(j) => conn.push((j, e0, e1)),
                None => return Err("a bend zone does not reach a piece on the other side".into()),
            }
        }
        for j in 0..b.len() {
            let n = conn.iter().filter(|c| c.0 == j).count();
            if n > 1 {
                return Err(format!(
                    "this fold line cuts a piece beyond it from the root piece in {n} bends (it crosses both arms of a shape such as a V or a U): the piece would be joined to the root twice, through the bend and through the part you fold, and a sheet cannot be bent that way. Move the fold line so that it crosses one arm only, or fold each arm with its own line"
                ));
            }
            if n != 1 {
                return Err("a piece beyond the fold does not hang on the root piece by a bend: move the fold line so that every piece beyond it is joined across the line".into());
            }
        }
        Ok(FoldPlan { nu, z, a: a.into_iter().next().unwrap_or_default(), b, conn })
    }

    fn apply_fold(&mut self, feature: &str, req: &FoldReq, plan: FoldPlan) -> Result<(), String> {
        let p = req.panel;
        let FoldPlan { nu, z, a, b, conn } = plan;
        let d = unit(req.d);
        // holes go to the piece that holds them
        let holes = std::mem::take(&mut self.panels[p].holes);
        let rings: Vec<Vec<P>> = std::iter::once(pts_of(&a)).chain(b.iter().map(pts_of)).collect();
        let mut hole_of: Vec<Vec<Vec<P>>> = vec![vec![]; rings.len()];
        // a hole that reaches into the bend zone (or across it) is no hole of one piece: it is cut out after the fold, like
        // a cut drawn across the bend
        let mut across: Vec<Vec<P>> = vec![];
        for h in holes.iter() {
            match rings.iter().position(|r| h.iter().all(|q| inside(r, *q, 1e-7))) {
                Some(i) => hole_of[i].push(h.clone()),
                None => across.push(h.clone()),
            }
        }
        // bends hanging on the panel go with the piece they hang on
        let mut moves: Vec<(usize, usize)> = vec![];
        for (k, bd) in self.bends.iter().enumerate() {
            if bd.parent != p {
                continue;
            }
            let (qa, qb) = (bd.at(bd.z[0], bd.v0[0]), bd.at(bd.z[0], bd.v0[1]));
            match rings.iter().position(|r| inside(r, qa, 1e-6) && inside(r, qb, 1e-6)) {
                Some(i) => moves.push((k, i)),
                None => {
                    self.panels[p].holes = holes.clone();
                    return Err(format!("the fold crosses the {} that hangs on this panel", if bd.kind == BendKind::Fold { "fold" } else { "flange" }));
                }
            }
        }
        self.folds += 1;
        self.panels[p].poly = a;
        self.panels[p].holes = hole_of[0].clone();
        let idx: Vec<usize> = b
            .into_iter()
            .enumerate()
            .map(|(j, poly)| {
                self.panels.push(Panel { poly, holes: hole_of[j + 1].clone(), via: None });
                self.panels.len() - 1
            })
            .collect();
        let fidx = self.folds - 1;
        for (j, e0, e1) in conn {
            let child = idx[j];
            self.panels[child].via = Some(self.bends.len());
            self.bends.push(TBend {
                feature: feature.to_string(),
                parent: p,
                child,
                o: req.o,
                d,
                nu,
                z,
                v0: e0,
                v1: e1,
                angle: req.angle,
                radius: req.radius,
                up: req.up,
                kind: BendKind::Fold,
                attach: format!("z{fidx}a"),
                attach_child: format!("z{fidx}b"),
                hem: false,
                batch: 0,
                edge: 0,
            });
        }
        for (k, i) in moves {
            if i > 0 {
                self.bends[k].parent = idx[i - 1];
            }
        }
        for h in across {
            // the hole is a clockwise ring; the cut takes the region counter-clockwise
            let area2: f64 = (0..h.len()).map(|i| cross(h[i], h[(i + 1) % h.len()])).sum();
            let region: Vec<P> = if area2 < 0.0 { h.iter().rev().cloned().collect() } else { h.clone() };
            self.cut(&region).map_err(|e| format!("a hole of the panel meets the bend zone and cannot be cut across it: {e}"))?;
        }
        Ok(())
    }
}

struct FoldPlan {
    nu: P,
    z: [f64; 2],
    a: Poly,
    b: Vec<Poly>,
    conn: Vec<(usize, [f64; 2], [f64; 2])>,
}

/// A fold to add.
#[derive(Debug, Clone)]
pub struct FoldReq {
    pub panel: usize,
    /// A point on the fold line and its direction.
    pub o: P,
    pub d: P,
    pub angle: f64,
    pub radius: f64,
    pub up: bool,
    pub position: FoldPos,
    /// A point on the side that stays (used only for a panel that hangs on no bend).
    pub fixed: Option<P>,
}

/// Area two polygons have in common (`b` is clipped against the edges of `a`, which must be convex and counter-clockwise; the
/// other may be concave).
pub fn overlap_area(a: &Poly, b: &Poly) -> f64 {
    let (convex, other) = if is_convex(a) { (a, b) } else if is_convex(b) { (b, a) } else { return 0.0 };
    let n = convex.len();
    let mut pieces: Vec<Poly> = vec![other.clone()];
    for i in 0..n {
        let (p0, p1) = (convex[i].0, convex[(i + 1) % n].0);
        let dv = unit(sub(p1, p0));
        let nrm = [-dv[1], dv[0]];
        let c = dot(nrm, p0);
        pieces = pieces.iter().flat_map(|q| clip_half(q, nrm, c, "")).collect();
        if pieces.is_empty() {
            return 0.0;
        }
    }
    pieces.iter().map(poly_area).sum()
}

/// The profile of a shaped flange cut at the end of the bend (`h = reach`): the pieces beyond it, in `(v, h)`.
fn profile_cut(pr: &Poly, reach: f64) -> Vec<Poly> {
    let hmin = pr.iter().map(|p| p.0[1]).fold(f64::MAX, f64::min);
    if hmin < reach - 1e-9 {
        clip_half(pr, [0.0, 1.0], reach, "z")
    } else {
        vec![pr.clone()]
    }
}

/// The width of a profile at the end of the bend: `[v0, v1]` of the edges that lie on the line `h = reach`.
fn profile_root(pr: &Poly, reach: f64) -> Result<[f64; 2], String> {
    let mut best: Option<[f64; 2]> = None;
    for piece in profile_cut(pr, reach) {
        let n = piece.len();
        for i in 0..n {
            let (a, b) = (piece[i].0, piece[(i + 1) % n].0);
            if (a[1] - reach).abs() < 1e-7 && (b[1] - reach).abs() < 1e-7 && (a[0] - b[0]).abs() > 1e-9 {
                let ext = [a[0].min(b[0]), a[0].max(b[0])];
                best = Some(best.map_or(ext, |h| [h[0].min(ext[0]), h[1].max(ext[1])]));
            }
        }
    }
    best.ok_or_else(|| "the profile has no edge on the end of the bend: it must reach down to the flange's root".to_string())
}

/// The leg of a shaped flange as a flat polygon: the part of the profile beyond the end of the bend (`h >= reach`), laid out from
/// the end of the bend zone (`u0` along the edge's outward normal). The root edge gets the tag `z`.
fn profile_leg(pr: &Poly, o: P, d: P, nu: P, u0: f64, reach: f64) -> Result<Poly, String> {
    let pieces = profile_cut(pr, reach);
    let best = pieces.into_iter().max_by(|a, b| poly_area(a).abs().total_cmp(&poly_area(b).abs())).ok_or("the profile does not reach beyond the bend")?;
    if poly_area(&best).abs() < 1e-9 {
        return Err("the profile does not reach beyond the bend".into());
    }
    let n = best.len();
    let flat: Poly = (0..n)
        .map(|i| {
            let (a, b) = (best[i].0, best[(i + 1) % n].0);
            let root = (a[1] - reach).abs() < 1e-7 && (b[1] - reach).abs() < 1e-7;
            (add(add(o, mul(d, a[0])), mul(nu, u0 + (a[1] - reach))), if root { "z".to_string() } else { best[i].1.clone() })
        })
        .collect();
    Ok(ccw(tidy(flat)))
}

fn is_convex(p: &Poly) -> bool {
    let n = p.len();
    (0..n).all(|i| cross(sub(p[(i + 1) % n].0, p[i].0), sub(p[(i + 2) % n].0, p[(i + 1) % n].0)) >= -1e-9)
}

/// An edge flange (or hem) to add.
#[derive(Debug, Clone)]
pub struct FlangeReq {
    pub panel: usize,
    pub edge: usize,
    pub height: f64,
    pub angle: f64,
    pub radius: f64,
    pub from: HeightFrom,
    pub pos: Position,
    pub up: bool,
    pub hem: bool,
    pub then: Vec<Next>,
    /// Partial width: the part `[v0, v1]` of the edge the flange covers, `v` measured along the edge from its start vertex
    /// (the ring runs counter-clockwise). `None`: the whole edge (or, with a profile, the root of the profile).
    pub span: Option<[f64; 2]>,
    /// A shaped leg: the outline of the flange in `(v, h)` (`v` along the edge from its start vertex, `h` from the height
    /// reference line, outward), the tag of each edge leaving a point (`ra...`: a run of samples of one arc). The leg is the
    /// part of it beyond the end of the bend; `height` is its largest `h`.
    pub profile: Option<Poly>,
    /// Holes in the shaped leg, in the same `(v, h)` coordinates (a hole made of equally spaced points on a circle is a true circle).
    pub profile_holes: Vec<Vec<P>>,
    /// Relief at the ends of a partial flange that stop inside the edge: `(width, depth)` of a rectangle cut into the plate
    /// beside the zone, `depth` beyond the bend's start line.
    pub relief: Option<(f64, f64)>,
}


fn inside_strict(ring: &[P], p: P) -> bool {
    let n = ring.len();
    let mut c = false;
    for i in 0..n {
        let (a, b) = (ring[i], ring[(i + 1) % n]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
            c = !c;
        }
    }
    c
}

/// Split the segment `a -> b` at the points where it meets the closed ring `other` (crossings, and vertices of the ring on it).
fn split_params(a: P, b: P, other: &[P]) -> Vec<f64> {
    let r = sub(b, a);
    let l = len(r);
    let mut ts: Vec<f64> = vec![];
    let n = other.len();
    for i in 0..n {
        let (c, d) = (other[i], other[(i + 1) % n]);
        let s = sub(d, c);
        let den = cross(r, s);
        if den.abs() > 1e-12 {
            let t = cross(sub(c, a), s) / den;
            let u = cross(sub(c, a), r) / den;
            if t > 1e-9 && t < 1.0 - 1e-9 && (-1e-9..=1.0 + 1e-9).contains(&u) {
                ts.push(t);
            }
        }
        // a vertex of the ring on this segment
        let tv = dot(sub(c, a), r) / (l * l).max(1e-300);
        if tv > 1e-9 && tv < 1.0 - 1e-9 && cross(r, sub(c, a)).abs() / l.max(1e-300) < 1e-9 {
            ts.push(tv);
        }
    }
    ts.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
    ts.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
    ts
}

/// `p` less the region `r`: the outer pieces (counter-clockwise, edges keep their tags, the edges of the cut get `tag`) and the
/// holes the cut leaves inside (clockwise rings of points).
pub fn subtract(p: &Poly, r: &[P], tag: &str) -> (Vec<Poly>, Vec<Vec<P>>) {
    let pr: Vec<P> = pts_of(p);
    let mut rr: Vec<P> = r.to_vec();
    if area(&rr) < 0.0 {
        rr.reverse();
    }
    if pr.len() < 3 || rr.len() < 3 {
        return (vec![p.clone()], vec![]);
    }
    // pieces: (a, b, tag)
    let mut edges: Vec<(P, P, String)> = vec![];
    let mut push_split = |a: P, b: P, other: &[P], tg: &str, rev: bool, keep: &dyn Fn(P, P) -> bool, edges: &mut Vec<(P, P, String)>| {
        let ts = split_params(a, b, other);
        let mut prev = 0.0;
        let pts: Vec<f64> = ts.into_iter().chain(std::iter::once(1.0)).collect();
        for t in pts {
            let (pa, pb) = (add(a, mul(sub(b, a), prev)), add(a, mul(sub(b, a), t)));
            prev = t;
            if len(sub(pb, pa)) < 1e-9 {
                continue;
            }
            if keep(pa, pb) {
                if rev {
                    edges.push((pb, pa, tg.to_string()));
                } else {
                    edges.push((pa, pb, tg.to_string()));
                }
            }
        }
    };
    let np = pr.len();
    for i in 0..np {
        let (a, b) = (pr[i], pr[(i + 1) % np]);
        // keep the part of the edge of `p` whose inner side is not inside the cut
        let keep = |pa: P, pb: P| {
            let m = mul(add(pa, pb), 0.5);
            let d = unit(sub(pb, pa));
            !inside_strict(&rr, add(m, mul([-d[1], d[0]], 1e-6)))
        };
        push_split(a, b, &rr, &p[i].1, false, &keep, &mut edges);
    }
    let nr = rr.len();
    for i in 0..nr {
        let (a, b) = (rr[i], rr[(i + 1) % nr]);
        // keep the part of the cut's edge that lies inside `p` (material on its outer side), reversed
        let keep = |pa: P, pb: P| {
            let m = mul(add(pa, pb), 0.5);
            let d = unit(sub(pb, pa));
            inside_strict(&pr, add(m, mul([d[1], -d[0]], 1e-6)))
        };
        push_split(a, b, &pr, tag, true, &keep, &mut edges);
    }
    // trace the loops
    let mut used = vec![false; edges.len()];
    let mut outers: Vec<Poly> = vec![];
    let mut holes: Vec<Vec<P>> = vec![];
    for s in 0..edges.len() {
        if used[s] {
            continue;
        }
        let mut lp: Poly = vec![];
        let mut cur = s;
        let mut guard = 0;
        let closed = loop {
            guard += 1;
            if guard > edges.len() + 2 {
                break false;
            }
            used[cur] = true;
            lp.push((edges[cur].0, edges[cur].2.clone()));
            let at = edges[cur].1;
            if len(sub(at, edges[s].0)) < 1e-7 {
                break true;
            }
            let din = unit(sub(edges[cur].1, edges[cur].0));
            let best = (0..edges.len()).filter(|&j| !used[j] && len(sub(edges[j].0, at)) < 1e-7).max_by(|&x, &y| {
                let ang = |j: usize| {
                    let dout = unit(sub(edges[j].1, edges[j].0));
                    cross(din, dout).atan2(dot(din, dout))
                };
                ang(x).partial_cmp(&ang(y)).unwrap_or(std::cmp::Ordering::Equal)
            });
            match best {
                Some(j) => cur = j,
                None => break false,
            }
        };
        if !closed || lp.len() < 3 {
            continue;
        }
        let lp = tidy(lp);
        let a = poly_area(&lp);
        if a > 1e-9 {
            outers.push(lp);
        } else if a < -1e-9 {
            holes.push(pts_of(&lp));
        }
    }
    (outers, holes)
}
