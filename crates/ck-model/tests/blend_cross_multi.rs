//! Crossing rounds over several walls: a plate whose outline is a polygon (sketch on `top`, extruded H along +y, the caps
//! `F2.C0` at y = 0 and `F2.C1` at y = H, the walls `F2.S.e<i>`) with the top and bottom edges of several walls rounded so
//! that the two rounds of a wall are wider together than the wall is high (r_bottom + r_top > H): they cut into each other
//! and meet in a sharp crease (ck-ops `blend::cross`, warning `blendRoundsCross`).
//!
//! Oracle (walls perpendicular to the caps). Take the plate with walls in the set S rounded: the solid is the intersection
//! of the per-wall single-edge results, so its section at height z (0 at C0, H at C1) is the outline polygon with every
//! wall of S moved inward by m(z) = max(d_b(z), d_t(H - z)), where d_r(t) = r - sqrt(r^2 - (r - t)^2) for 0 <= t <= r and 0
//! beyond (how far a round of radius r has cut into the wall at distance t from its cap). The other walls stay. So
//! V = integral_0^H area(clip(polygon, walls of S moved in by m(z))) dz. With z = r_b (1 - cos phi) the bottom branch is
//! smooth (d = r_b (1 - sin phi), dz = r_b sin phi dphi) and the top branch likewise; the two branches meet where
//! d_b(z*) = d_t(H - z*) (bisection), or do not touch (r_b + r_t <= H, the middle is the full outline). Simpson on a few
//! thousand steps per branch (the area is piecewise quadratic in m). For all walls of a rectangle the area is the exact
//! (a - 2m)(b - 2m); for a convex polygon A(m) = A0 - P m + m^2 sum cot(theta_k / 2), checked below.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;

#[derive(Debug, Clone, PartialEq)]
struct Out {
    errs: Vec<String>,
    params: Vec<String>,
    faces: usize,
    volume: f64,
    issues: Vec<String>,
    lumps: usize,
    open_edges: usize,
    tess_failed: usize,
    names: Vec<String>,
    warns: Vec<String>,
}

type Pt = (f64, f64);

/// The user's plate: sketch (0,0), (40,4), (34,30), (-4,24) on the top plane.
const QUAD: [Pt; 4] = [(0.0, 0.0), (40.0, 4.0), (34.0, 30.0), (-4.0, 24.0)];
const BOX: [Pt; 4] = [(0.0, 0.0), (40.0, 0.0), (40.0, 30.0), (0.0, 30.0)];

fn plate(pts: &[Pt], h: f64) -> String {
    let n = pts.len();
    let pj: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{}","x":{x},"y":{y}}}"#, i + 1)).collect();
    let en: Vec<String> = (0..n).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{}","p2":"p{}"}}"#, i + 1, i + 1, (i + 1) % n + 1)).collect();
    format!(
        r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{}],"entities":[{}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}}"#,
        pj.join(","),
        en.join(",")
    )
}

/// Pick of the edge between wall `i` (0-based) and cap `cap` (0: y = 0, 1: y = h), with its own radius.
fn wall_pick(pts: &[Pt], h: f64, i: usize, cap: usize, r: f64) -> String {
    let (a, b) = (pts[i], pts[(i + 1) % pts.len()]);
    let (mx, my) = ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
    let y = if cap == 0 { 0.0 } else { h };
    format!(r#"{{"faces":["F2.S.e{}","F2.C{cap}"],"at":[{mx},{y},{}],"radius":{r}}}"#, i + 1, -my)
}

fn fillet(r: f64, picks: &[String]) -> String {
    format!(r#"{{"type":"fillet","id":"F3","radius":{r},"edges":[{}]}}"#, picks.join(","))
}

/// Both rounds of every wall in `walls`: radius `rb` at the cap y = 0 and `rt` at the cap y = h. `order` shuffles the picks.
fn picks_for(pts: &[Pt], h: f64, walls: &[usize], rb: f64, rt: f64, order: Order) -> Vec<String> {
    let mut v: Vec<(usize, usize)> = vec![];
    for &w in walls {
        v.push((w, 0));
    }
    for &w in walls {
        v.push((w, 1));
    }
    match order {
        Order::BottomThenTop => {}
        Order::Interleaved => v.sort_by_key(|&(w, c)| (w, c)),
        Order::Reversed => v.reverse(),
        Order::TopThenBottom => v.sort_by_key(|&(w, c)| (1 - c, w)),
    }
    v.iter().map(|&(w, c)| wall_pick(pts, h, w, c, if c == 0 { rb } else { rt })).collect()
}

#[derive(Clone, Copy, Debug)]
enum Order {
    BottomThenTop,
    Interleaved,
    Reversed,
    TopThenBottom,
}

fn run_once(base: &str, feats: &[String]) -> Out {
    let mut all = vec![base.to_string()];
    all.extend(feats.iter().cloned());
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, all.join(","));
    let part = Part::parse(&text).unwrap();
    let mut br = Brep::new();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    let errs = res.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.code)).collect();
    let params = res.errors.iter().map(|e| e.error.params.iter().map(|(k, v)| format!("{k}={}", v.to_string())).collect::<Vec<_>>().join("; ")).collect();
    let b = br.body.as_ref().expect("a body");
    let mesh = ck_tess::mesh::tessellate(b).mesh;
    Out {
        errs,
        params,
        faces: b.num_faces(),
        volume: ck_topo::mass::mass_props(b).volume,
        issues: ck_check::validate(b, 1e-6).iter().map(|i| format!("{} {}", i.code, i.detail)).collect(),
        lumps: b.num_lumps(),
        open_edges: b.edge_ids().filter(|&e| b.edge_coedges(e).len() != 2).count(),
        tess_failed: mesh.failed.len(),
        names: b.face_ids().map(|f| b.face_id_str(f)).collect(),
        warns: res.warnings.iter().map(|w| format!("{}: {} {}", w.feature, w.error.code, w.error.params.iter().map(|(k, v)| format!("{k}={}", v.to_string())).collect::<Vec<_>>().join("; "))).collect(),
    }
}

/// Valid, closed, one solid, meshes, repeatable; the volume to 1e-6 relative when `want` is given.
fn built(tag: &str, base: &str, feats: &[String], want: Option<f64>) -> Out {
    built_tol(tag, base, feats, want, 1e-6)
}

/// [`built`] with the relative volume tolerance `tol`.
fn built_tol(tag: &str, base: &str, feats: &[String], want: Option<f64>, tol: f64) -> Out {
    let o = run_once(base, feats);
    println!("{tag}: faces {} volume {:.9} want {want:?} errs {:?} warns {:?}", o.faces, o.volume, o.errs, o.warns);
    assert!(o.errs.is_empty(), "{tag}: errors {:?} {:?}", o.errs, o.params);
    assert!(o.issues.is_empty(), "{tag}: validate {:?}", o.issues);
    assert_eq!(o.lumps, 1, "{tag}: one solid");
    assert_eq!(o.open_edges, 0, "{tag}: closed");
    assert_eq!(o.tess_failed, 0, "{tag}: tessellates");
    if let Some(w) = want {
        assert!((o.volume - w).abs() <= tol * w, "{tag}: volume {} vs {w} (rel {:e})", o.volume, (o.volume - w).abs() / w);
    }
    let again = run_once(base, feats);
    assert_eq!(again, o, "{tag}: a repeat run differs");
    o
}

fn refused(tag: &str, base: &str, feats: &[String]) -> Out {
    let o = run_once(base, feats);
    println!("{tag}: errs {:?} {:?}", o.errs, o.params);
    assert_eq!(o.errs, vec!["F3: blendOverflow".to_string()], "{tag}: {:?}", o.params);
    o
}

// ---------------------------------------------------------------- the oracle

fn area(p: &[Pt]) -> f64 {
    (0..p.len()).map(|i| p[i].0 * p[(i + 1) % p.len()].1 - p[(i + 1) % p.len()].0 * p[i].1).sum::<f64>().abs() / 2.0
}

/// The convex polygon `poly` (any orientation) with the walls in `walls` moved inward by `m`.
fn moved_in(poly: &[Pt], walls: &[usize], m: f64) -> f64 {
    let n = poly.len();
    let ccw = (0..n).map(|i| poly[i].0 * poly[(i + 1) % n].1 - poly[(i + 1) % n].0 * poly[i].1).sum::<f64>() > 0.0;
    let mut cur: Vec<Pt> = poly.to_vec();
    for &w in walls {
        let (a, b) = (poly[w], poly[(w + 1) % n]);
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = dx.hypot(dy);
        // inward normal
        let (nx, ny) = if ccw { (-dy / l, dx / l) } else { (dy / l, -dx / l) };
        let side = |p: Pt| (p.0 - a.0) * nx + (p.1 - a.1) * ny - m; // >= 0 kept
        let mut out = vec![];
        for i in 0..cur.len() {
            let (p, q) = (cur[i], cur[(i + 1) % cur.len()]);
            let (sp, sq) = (side(p), side(q));
            if sp >= 0.0 {
                out.push(p);
            }
            if (sp >= 0.0) != (sq >= 0.0) {
                let t = sp / (sp - sq);
                out.push((p.0 + t * (q.0 - p.0), p.1 + t * (q.1 - p.1)));
            }
        }
        cur = out;
        if cur.len() < 3 {
            return 0.0;
        }
    }
    area(&cur)
}

fn depth(r: f64, t: f64) -> f64 {
    if t >= r {
        0.0
    } else {
        r - (r * r - (r - t) * (r - t)).max(0.0).sqrt()
    }
}

/// Simpson of the section area over z in [z0, z1] on the branch of radius `r` (`from_top`: z measured from C1).
fn branch(poly: &[Pt], walls: &[usize], r: f64, t0: f64, t1: f64) -> f64 {
    // t = r (1 - cos phi), t in [t0, t1]
    let (p0, p1) = ((1.0 - t0 / r).clamp(-1.0, 1.0).acos(), (1.0 - t1 / r).clamp(-1.0, 1.0).acos());
    let n = 4000;
    let f = |phi: f64| moved_in(poly, walls, r * (1.0 - phi.sin())) * r * phi.sin();
    let hh = (p1 - p0) / n as f64;
    let mut s = f(p0) + f(p1);
    for i in 1..n {
        s += f(p0 + hh * i as f64) * if i % 2 == 1 { 4.0 } else { 2.0 };
    }
    s * hh / 3.0
}

/// Volume of the plate `poly` x `h` with the walls in `walls` rounded at C0 with `rb` and at C1 with `rt` (see the module docs).
fn oracle(poly: &[Pt], h: f64, walls: &[usize], rb: f64, rt: f64) -> f64 {
    assert!(rb <= h && rt <= h);
    let a0 = area(poly);
    if rb + rt <= h {
        return a0 * (h - rb - rt) + branch(poly, walls, rb, 0.0, rb) + branch(poly, walls, rt, 0.0, rt);
    }
    // z* where d_b(z) = d_t(h - z)
    let (mut lo, mut hi) = (0.0, h);
    for _ in 0..200 {
        let z = (lo + hi) / 2.0;
        if depth(rb, z) > depth(rt, h - z) {
            lo = z;
        } else {
            hi = z;
        }
    }
    let zs = (lo + hi) / 2.0;
    branch(poly, walls, rb, 0.0, zs) + branch(poly, walls, rt, 0.0, h - zs)
}

#[test]
fn oracle_self_checks() {
    // rectangle, all walls: (a - 2m)(b - 2m); the closed form for A(m) with the cot sum; and the plain box of r <= H / 2
    let walls = [0, 1, 2, 3];
    let m = 3.0;
    assert!((moved_in(&BOX, &walls, m) - (40.0 - 2.0 * m) * (30.0 - 2.0 * m)).abs() < 1e-9);
    let (a0, p) = (area(&QUAD), (0..4).map(|i| ((QUAD[(i + 1) % 4].0 - QUAD[i].0).hypot(QUAD[(i + 1) % 4].1 - QUAD[i].1))).sum::<f64>());
    let cot_sum: f64 = (0..4)
        .map(|i| {
            let (a, b, c) = (QUAD[(i + 3) % 4], QUAD[i], QUAD[(i + 1) % 4]);
            let (u, v) = ((a.0 - b.0, a.1 - b.1), (c.0 - b.0, c.1 - b.1));
            let th = (u.0 * v.0 + u.1 * v.1).atan2((u.0 * v.1 - u.1 * v.0).abs());
            // th = pi/2 - angle... use the interior angle
            let ang = (u.0 * v.1 - u.1 * v.0).abs().atan2(u.0 * v.0 + u.1 * v.1);
            let _ = th;
            1.0 / (ang / 2.0).tan()
        })
        .sum();
    assert!((moved_in(&QUAD, &walls, m) - (a0 - p * m + m * m * cot_sum)).abs() < 1e-9);
    // r = H / 2 on a 4 x 10 x 6 prism: both top edges, the single-section formula w h l - l (1 - pi/4) 2 r^2 does not apply
    // (that one is for two edges of one face); instead the rounded-walls plate of height 5 with r = 2.5 is a pair of quarter
    // discs per wall: for the two long walls of a rectangle the section is 5 - 2 (1 - pi/4) 2.5^2 per unit length of the 30 side
    let v = oracle(&BOX, 5.0, &[0, 2], 2.5, 2.5);
    let want = 40.0 * 30.0 * 5.0 - 40.0 * (2.0 * 2.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 2.5 * 2.5);
    assert!((v - want).abs() < 1e-6, "{v} vs {want}");
}

// ---------------------------------------------------------------- the user's plate

/// Relative volume tolerance against the oracle. The crossing build is the intersection of single-edge rounds, so its
/// corners are the exact mitre and the volume agrees to 1e-6. Not so for the ORDINARY build below the tie (`crossing`
/// false) when two rounded walls meet at a corner that is not a right angle (the user's plate: 86 to 94 degrees): main
/// leaves up to 4e-4 more material there (0.03 r^3 per corner, r = 2.0: 4.0e-4, 2.4: 7e-4, 2.5: 8e-4), a corner effect of
/// the ordinary builder that this change leaves alone; 5e-3 bounds it and the printed volumes record it.
fn tol_for(pts: &[Pt], walls: &[usize], crossing: bool) -> f64 {
    let n = pts.len();
    if crossing || pts == &BOX[..] {
        return 1e-6;
    }
    if walls.len() > 1 && walls.iter().any(|&w| walls.contains(&((w + 1) % n))) {
        5e-3
    } else {
        1e-6
    }
}

fn rel_name(o: &Out, n: &str) -> bool {
    o.names.iter().any(|x| x == n || x.starts_with(&format!("{n}~")))
}

#[test]
fn quad_plate_all_eight_edges_over_the_tie() {
    let h = 5.0;
    let base = plate(&QUAD, h);
    let walls = [0, 1, 2, 3];
    for r in [2.0, 2.4, 2.5, 2.6, 3.0, 4.0, 5.0] {
        let feats = [fillet(r, &picks_for(&QUAD, h, &walls, r, r, Order::BottomThenTop))];
        let o = built_tol(&format!("quad r={r}"), &base, &feats, Some(oracle(&QUAD, h, &walls, r, r)), tol_for(&QUAD, &walls, r > 2.5));
        println!("quad r={r}: {} faces {:?}", o.faces, o.names);
        if r > 2.5 {
            assert_eq!(o.warns.len(), 1, "{:?}", o.warns);
            let w = &o.warns[0];
            assert!(w.starts_with("F3: blendRoundsCross") && w.contains("meetRadius=2.5") && w.contains("width=5"), "{w}");
            // the walls are gone: every one of the 8 rounds stays
            for k in 0..8 {
                assert!(rel_name(&o, &format!("F3.B{k}")), "r={r}: B{k} missing in {:?}", o.names);
            }
            assert!(!o.names.iter().any(|n| n.starts_with("F2.S.e")), "r={r}: a wall is left: {:?}", o.names);
        } else {
            assert!(o.warns.is_empty(), "{:?}", o.warns);
        }
    }
}

/// Not selected in the user's case: the four vertical corner edges (wall to wall). The 8 picks are all wall-to-cap edges.
#[test]
fn quad_plate_selection_is_the_eight_cap_edges() {
    let picks = picks_for(&QUAD, 5.0, &[0, 1, 2, 3], 3.0, 3.0, Order::BottomThenTop);
    assert_eq!(picks.len(), 8);
    for p in &picks {
        assert!(p.contains("\"F2.C0\"") || p.contains("\"F2.C1\""), "{p}");
    }
    let base = plate(&QUAD, 5.0);
    let o = run_once(&base, &[fillet(3.0, &picks)]);
    assert!(o.errs.is_empty(), "{:?}", o.errs);
    // adding the four vertical edges to the same selection is a different (corner) problem: it must not build an invalid body
    let mut all = picks.clone();
    for i in 0..4 {
        let (a, b) = (QUAD[i], QUAD[(i + 1) % 4]);
        let _ = b;
        all.push(format!(r#"{{"faces":["F2.S.e{}","F2.S.e{}"],"at":[{},2.5,{}],"radius":3}}"#, (i + 3) % 4 + 1, i + 1, a.0, -a.1));
    }
    let o = run_once(&base, &[fillet(3.0, &all)]);
    println!("12 edges r=3: errs {:?} {:?} warns {:?} faces {}", o.errs, o.params, o.warns, o.faces);
    if o.errs.is_empty() {
        assert!(o.issues.is_empty() && o.lumps == 1 && o.open_edges == 0, "{:?}", o.issues);
    }
}

#[test]
fn quad_plate_unequal_top_and_bottom() {
    let h = 5.0;
    let base = plate(&QUAD, h);
    let walls = [0, 1, 2, 3];
    for (rb, rt) in [(3.0, 2.6), (2.6, 3.0), (4.0, 1.5), (2.0, 3.5)] {
        let feats = [fillet(rb, &picks_for(&QUAD, h, &walls, rb, rt, Order::BottomThenTop))];
        built_tol(&format!("quad rb={rb} rt={rt}"), &base, &feats, Some(oracle(&QUAD, h, &walls, rb, rt)), tol_for(&QUAD, &walls, rb + rt > h));
    }
}

#[test]
fn quad_plate_pick_orders() {
    let h = 5.0;
    let base = plate(&QUAD, h);
    let walls = [0, 1, 2, 3];
    for r in [2.6, 3.0] {
        let want = oracle(&QUAD, h, &walls, r, r);
        let mut first: Option<(usize, f64)> = None;
        for order in [Order::BottomThenTop, Order::Interleaved, Order::Reversed, Order::TopThenBottom] {
            let feats = [fillet(r, &picks_for(&QUAD, h, &walls, r, r, order))];
            let o = built_tol(&format!("quad r={r} {order:?}"), &base, &feats, Some(want), tol_for(&QUAD, &walls, true));
            match first {
                None => first = Some((o.faces, o.volume)),
                Some((f, v)) => {
                    assert_eq!(o.faces, f, "{order:?}");
                    assert!((o.volume - v).abs() < 1e-6 * v, "{order:?}: {} vs {v}", o.volume);
                }
            }
        }
    }
}

#[test]
fn quad_plate_wall_subsets() {
    let h = 5.0;
    let base = plate(&QUAD, h);
    for walls in [vec![0], vec![1], vec![0, 2], vec![1, 3], vec![0, 1], vec![1, 2], vec![0, 1, 2], vec![1, 2, 3]] {
        for r in [2.6, 3.0] {
            for order in [Order::BottomThenTop, Order::Reversed] {
                let feats = [fillet(r, &picks_for(&QUAD, h, &walls, r, r, order))];
                built_tol(&format!("quad walls {walls:?} r={r} {order:?}"), &base, &feats, Some(oracle(&QUAD, h, &walls, r, r)), tol_for(&QUAD, &walls, r > 2.5));
            }
        }
    }
}

#[test]
fn box_plate() {
    let h = 5.0;
    let base = plate(&BOX, h);
    let walls = [0, 1, 2, 3];
    for r in [2.4, 2.5, 2.6, 3.0, 4.5] {
        let feats = [fillet(r, &picks_for(&BOX, h, &walls, r, r, Order::BottomThenTop))];
        built(&format!("box r={r}"), &base, &feats, Some(oracle(&BOX, h, &walls, r, r)));
    }
    for walls in [vec![0, 2], vec![1, 3], vec![0, 1], vec![0, 1, 2]] {
        let feats = [fillet(3.0, &picks_for(&BOX, h, &walls, 3.0, 3.0, Order::Interleaved))];
        built(&format!("box walls {walls:?} r=3"), &base, &feats, Some(oracle(&BOX, h, &walls, 3.0, 3.0)));
    }
}

/// What stays refused: a round taller than its wall (r > H), here with all walls.
#[test]
fn limits_stay_refused() {
    let h = 5.0;
    let walls = [0, 1, 2, 3];
    for (pts, name) in [(&QUAD, "quad"), (&BOX, "box")] {
        let base = plate(pts, h);
        for r in [5.5, 6.0] {
            refused(&format!("{name} r={r}"), &base, &[fillet(r, &picks_for(pts, h, &walls, r, r, Order::BottomThenTop))]);
        }
    }
}
