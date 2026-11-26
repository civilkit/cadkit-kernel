//! Shared helpers of `blend_spline_corner` and `blend_spline_corner_families`: profiles made of splines and straight lines (a D, a
//! circle with a flat, a stadium of two arcs), the picks of their rims and vertical corner edges, and the independent volume estimate
//! (Pappus on the rim, plus the corner area times the height for a rounded vertical edge).
#![allow(dead_code)]
use ck_geom::curve::Curve;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use std::f64::consts::PI;

pub const H: f64 = 20.0;

/// One run of a closed profile: a line to the next corner, or an interpolating spline (`cv` false) / control-vertex spline through
/// the interior points to the next corner.
#[derive(Clone)]
pub struct Run {
    pub pts: Vec<(f64, f64)>,
    pub cv: bool,
}

/// A profile: its corner points and what runs between them (`None` a line).
pub struct Shape {
    pub name: String,
    pub corners: Vec<(f64, f64)>,
    pub runs: Vec<Option<Run>>,
}

fn arc_pts(rx: f64, ry: f64, cx: f64, cy: f64, a0: f64, a1: f64, n: usize) -> Vec<(f64, f64)> {
    (0..=n).map(|i| {
        let t = a0 + (a1 - a0) * i as f64 / n as f64;
        (cx + rx * t.cos(), cy + ry * t.sin())
    }).collect()
}

/// A D: an elliptic arc over the chord (-w/2..w/2, 0), `hgt` high, closed by the chord. `n` points.
pub fn d_shape(name: &str, w: f64, hgt: f64, n: usize) -> Shape {
    let a = arc_pts(w / 2.0, hgt, 0.0, 0.0, 0.0, PI, n);
    // the interpolating spline runs from the right end over the top to the left end; the line closes it
    let corners = vec![a[0], a[n]];
    let inner: Vec<(f64, f64)> = a[1..n].iter().enumerate().map(|(i, p)| (p.0, p.1 + if i % 2 == 1 { 0.01 * hgt } else { 0.0 })).collect();
    Shape { name: name.into(), corners, runs: vec![Some(Run { pts: inner, cv: false }), None] }
}

/// A D whose spline leaves the chord tangentially at both ends (control-vertex spline, first and last legs along the chord).
pub fn d_tangent(name: &str, w: f64, hgt: f64) -> Shape {
    let (a, b) = ((w / 2.0, 0.0), (-w / 2.0, 0.0));
    let k = w * 0.28;
    // from a (right) back along the chord direction first: the line b -> a arrives at a heading +x, the spline leaves a heading +x
    let pts = vec![(a.0 + k, 0.0), (a.0 + k, hgt * 1.3), (b.0 - k, hgt * 1.3), (b.0 - k, 0.0)];
    Shape { name: name.into(), corners: vec![a, b], runs: vec![Some(Run { pts, cv: true }), None] }
}

/// A circle of radius `r` with a flat: the arc over `[-(pi - cut), pi - cut]` (interpolating spline) closed by the chord.
pub fn circle_flat(name: &str, r: f64, cut: f64, n: usize) -> Shape {
    let a = arc_pts(r, r, 0.0, 0.0, -(PI - cut), PI - cut, n);
    let corners = vec![a[0], a[n]];
    let inner: Vec<(f64, f64)> = a[1..n].to_vec();
    Shape { name: name.into(), corners, runs: vec![Some(Run { pts: inner, cv: false }), None] }
}

/// Two arcs (top and bottom) joined by two straight sides: a stadium-like blob with two flats.
pub fn two_flats(name: &str, w: f64, hh: f64, bulge: f64, n: usize) -> Shape {
    let (c0, c1, c2, c3) = ((w / 2.0, -hh / 2.0), (w / 2.0, hh / 2.0), (-w / 2.0, hh / 2.0), (-w / 2.0, -hh / 2.0));
    let top: Vec<(f64, f64)> = (1..n).map(|i| { let s = i as f64 / n as f64; (c1.0 + (c2.0 - c1.0) * s, hh / 2.0 + bulge * (PI * s).sin()) }).collect();
    let bot: Vec<(f64, f64)> = (1..n).map(|i| { let s = i as f64 / n as f64; (c3.0 + (c0.0 - c3.0) * s, -hh / 2.0 - bulge * (PI * s).sin() * 0.8) }).collect();
    Shape { name: name.into(), corners: vec![c0, c1, c2, c3], runs: vec![None, Some(Run { pts: top, cv: false }), None, Some(Run { pts: bot, cv: false })] }
}

/// Two flats with G1 control-vertex arcs (the spline leaves each flat along it).
pub fn two_flats_tangent(name: &str, w: f64, hh: f64, bulge: f64) -> Shape {
    let (c0, c1, c2, c3) = ((w / 2.0, -hh / 2.0), (w / 2.0, hh / 2.0), (-w / 2.0, hh / 2.0), (-w / 2.0, -hh / 2.0));
    let k = hh * 0.3;
    let top = vec![(c1.0, c1.1 + k), (c1.0 - w * 0.2, c1.1 + bulge * 1.6), (c2.0 + w * 0.2, c1.1 + bulge * 1.6), (c2.0, c2.1 + k)];
    let bot = vec![(c3.0, c3.1 - k), (c3.0 + w * 0.2, c3.1 - bulge * 1.6), (c0.0 - w * 0.2, c3.1 - bulge * 1.6), (c0.0, c0.1 - k)];
    Shape { name: name.into(), corners: vec![c0, c1, c2, c3], runs: vec![None, Some(Run { pts: top, cv: true }), None, Some(Run { pts: bot, cv: true })] }
}

pub fn sketch(sh: &Shape) -> String {
    let mut pts: Vec<String> = vec![];
    let mut ents: Vec<String> = vec![];
    let nc = sh.corners.len();
    for (i, c) in sh.corners.iter().enumerate() {
        pts.push(format!(r#"{{"id":"c{i}","x":{:.9},"y":{:.9}}}"#, c.0, c.1));
    }
    for (i, run) in sh.runs.iter().enumerate() {
        let (a, b) = (format!("c{i}"), format!("c{}", (i + 1) % nc));
        match run {
            None => ents.push(format!(r#"{{"id":"e{i}","type":"line","p1":"{a}","p2":"{b}"}}"#)),
            Some(r) => {
                let mut ids = vec![format!(r#""{a}""#)];
                for (j, p) in r.pts.iter().enumerate() {
                    pts.push(format!(r#"{{"id":"s{i}_{j}","x":{:.9},"y":{:.9}}}"#, p.0, p.1));
                    ids.push(format!(r#""s{i}_{j}""#));
                }
                ids.push(format!(r#""{b}""#));
                let deg = if r.cv { r#","degree":3"# } else { "" };
                ents.push(format!(r#"{{"id":"e{i}","type":"{}","pts":[{}]{deg}}}"#, if r.cv { "cvspline" } else { "spline" }, ids.join(",")));
            }
        }
    }
    format!(
        r#"{{"type":"sketch","id":"F1","name":"S","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"O","x":0,"y":0,"fixed":true}},{}],"entities":[{}],"constraints":[]}}"#,
        pts.join(","),
        ents.join(",")
    )
}

pub fn doc(sh: &Shape, extra: &str) -> String {
    format!(r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"extrude","op":"add","sketch":"F1","end":"blind","depth":{H},"id":"F2"}}{extra}]}}"#, sketch(sh))
}

pub fn regen_doc(d: &str) -> (Option<Body>, Vec<String>) {
    let part = Part::parse(d).unwrap();
    let mut br = Brep::new();
    let res = regen::regen(&part, &RegenOpts::default(), &mut br);
    (br.body, res.errors.iter().map(|e| format!("{}: {} {:?}", e.feature, e.error.code, e.error.params)).collect())
}

pub fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

fn pick_json(b: &Body, e: ck_topo::body::EdgeId, radius: Option<f64>) -> String {
    let fs = b.edge_faces(e);
    let ed = b.edge(e);
    let (p0, p1) = (b.vertex(ed.v0).p, b.vertex(ed.v1).p);
    let p = if matches!(ed.curve, Curve::Line { .. }) && ed.v0 != ed.v1 { (p0 + p1) * 0.5 } else { p0 };
    let rad = radius.map(|r| format!(r#","radius":{r}"#)).unwrap_or_default();
    format!(r#"{{"faces":["{}","{}"],"at":[{:.9},{:.9},{:.9}]{rad}}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), p.x, p.y, p.z)
}

/// Picks of the rim edges of cap `face`.
pub fn rim_picks(b: &Body, face: &str, radius: Option<f64>) -> Vec<String> {
    let f = b.find_face(face).unwrap();
    b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).map(|e| pick_json(b, e, radius)).collect()
}

/// Picks of the vertical edges of the extrusion (edges joining two wall faces).
pub fn vertical_picks(b: &Body, radius: Option<f64>) -> Vec<String> {
    let caps: Vec<_> = ["F2.C0", "F2.C1"].iter().filter_map(|n| b.find_face(n)).collect();
    b.edge_ids().filter(|&e| {
        let fs = b.edge_faces(e);
        fs.len() == 2 && !fs.iter().any(|f| caps.contains(f)) && matches!(b.edge(e).curve, Curve::Line { .. })
    }).map(|e| pick_json(b, e, radius)).collect()
}

pub fn rim_length(b: &Body, face: &str) -> f64 {
    let f = b.find_face(face).unwrap();
    b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).map(|e| b.edge_length(e)).sum()
}

/// Exterior angles of the corners of cap `face` (0 where the rim is tangent-continuous).
pub fn exterior_angles(b: &Body, face: &str) -> Vec<f64> {
    let f = b.find_face(face).unwrap();
    let es: Vec<_> = b.edge_ids().filter(|&e| b.edge_faces(e).len() == 2 && b.edge_faces(e).contains(&f)).collect();
    let mut out = vec![];
    let mut seen = vec![];
    for &e in &es {
        for v in [b.edge(e).v0, b.edge(e).v1] {
            if seen.contains(&v) {
                continue;
            }
            seen.push(v);
            let mut dirs = vec![];
            for &g in &es {
                let ed = b.edge(g);
                for (vv, t, sgn) in [(ed.v0, ed.t0, 1.0), (ed.v1, ed.t1, -1.0)] {
                    if vv == v {
                        let [_, d] = ed.curve.derivs1_arr(t);
                        dirs.push(d.normalize() * sgn);
                    }
                }
            }
            if dirs.len() == 2 {
                let c = dirs[0].dot(dirs[1]).clamp(-1.0, 1.0);
                // the outgoing tangents point away from the vertex: straight through means dot = -1
                out.push(PI - c.acos());
            }
        }
    }
    out
}

/// Section area and the distance of its centroid from the rim for a round `s` (or an equal chamfer).
pub fn section(chamfer: bool, s: f64) -> (f64, f64) {
    if chamfer {
        (s * s / 2.0, s / 3.0)
    } else {
        let a = (1.0 - PI / 4.0) * s * s;
        (a, s * (5.0 / 6.0 - PI / 4.0) / (1.0 - PI / 4.0))
    }
}

/// Dense points of cap edge `e` walking away from vertex `v`, with the cumulative arc length.
fn walk(b: &Body, e: ck_topo::body::EdgeId, v: ck_topo::body::VertexId, win: f64) -> Vec<(ck_geom::Vec3, f64)> {
    let ed = b.edge(e);
    let n = 60000;
    let (ta, tb) = if ed.v0 == v { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
    let mut out = vec![];
    let mut s = 0.0;
    let mut prev = ed.curve.eval(ta);
    let total = b.edge_length(e);
    let frac = (30.0 * win / total).min(1.0);
    for i in 0..=n {
        let p = ed.curve.eval(ta + (tb - ta) * frac * i as f64 / n as f64);
        s += p.dist(prev);
        prev = p;
        out.push((p, s));
    }
    out
}

/// What cutting one corner of the top cap (a round of radius `size`, or an equal chamfer) removes: the area, the change of the rim's
/// length, and the exterior angles of the corners the cut leaves (a chamfer leaves two).
pub struct Cut {
    pub area: f64,
    pub dlen: f64,
    pub sharp: Vec<f64>,
}

pub fn corner_cuts(b0: &Body, size: f64, chamfer: bool) -> Vec<Cut> {
    let f = b0.find_face("F2.C1").unwrap();
    let es: Vec<_> = b0.edge_ids().filter(|&e| b0.edge_faces(e).len() == 2 && b0.edge_faces(e).contains(&f)).collect();
    let ncap = b0.outward_normal(f, 0.0, 0.0);
    let th = exterior_angles(b0, "F2.C1");
    let _ = th;
    let mut cuts = vec![];
    let mut seen = vec![];
    for &e in &es {
        for v in [b0.edge(e).v0, b0.edge(e).v1] {
            if seen.contains(&v) {
                continue;
            }
            seen.push(v);
            let inc: Vec<_> = es.iter().copied().filter(|&g| b0.edge(g).v0 == v || b0.edge(g).v1 == v).collect();
            if inc.len() != 2 {
                continue;
            }
            let (wa, wb) = (walk(b0, inc[0], v, size), walk(b0, inc[1], v, size));
            let tan = |w: &Vec<(ck_geom::Vec3, f64)>| (w[8].0 - w[0].0).normalize();
            let c = tan(&wa).dot(tan(&wb)).clamp(-1.0, 1.0);
            let theta = PI - c.acos();
            if theta < 1e-3 {
                continue;
            }
            let p0 = wa[0].0;
            let area = |pts: &[ck_geom::Vec3]| {
                let mut a = ck_geom::Vec3::ZERO;
                for i in 0..pts.len() {
                    a = a + pts[i].cross(pts[(i + 1) % pts.len()]);
                }
                0.5 * a.dot(ncap).abs()
            };
            if chamfer {
                let ia = wa.iter().position(|x| x.1 >= size).unwrap();
                let ib = wb.iter().position(|x| x.1 >= size).unwrap();
                let (pa, pb) = (wa[ia].0, wb[ib].0);
                let mut poly = vec![p0];
                poly.extend(wa[..=ia].iter().map(|x| x.0).skip(1));
                poly.extend(wb[..=ib].iter().rev().map(|x| x.0));
                let chord = pa.dist(pb);
                let ch = (pb - pa).normalize();
                let (ta, tb) = ((wa[ia + 8].0 - wa[ia].0).normalize(), (wb[ib + 8].0 - wb[ib].0).normalize());
                // exterior angles between the chord (traversed a -> b) and the two walls
                let e1 = (-ta).dot(ch).clamp(-1.0, 1.0).acos();
                let e2 = ch.dot(tb).clamp(-1.0, 1.0).acos();
                cuts.push(Cut { area: area(&poly), dlen: chord - 2.0 * size, sharp: vec![e1, e2] });
            } else {
                // the line side is the one with a straight curve
                let (wl, ws) = if matches!(b0.edge(inc[0]).curve, Curve::Line { .. }) { (&wa, &wb) } else { (&wb, &wa) };
                let dl = (wl[8].0 - wl[0].0).normalize();
                let mut nin = ncap.cross(dl);
                if nin.dot(ws[40].0 - p0) < 0.0 {
                    nin = -nin;
                }
                let cen = |lam: f64| p0 + dl * lam + nin * size;
                let gap = |lam: f64| ws.iter().map(|x| x.0.dist(cen(lam))).fold(f64::INFINITY, f64::min) - size;
                let (mut lo, mut hi) = (0.0, wl.last().unwrap().1);
                for _ in 0..100 {
                    let m = 0.5 * (lo + hi);
                    if gap(m) < 0.0 { lo = m } else { hi = m }
                }
                let lam = 0.5 * (lo + hi);
                let c0 = cen(lam);
                let k = (0..ws.len()).min_by(|&i, &j| ws[i].0.dist(c0).total_cmp(&ws[j].0.dist(c0))).unwrap();
                let (foot, cont) = (p0 + dl * lam, ws[k].0);
                let (u0, u1) = ((foot - c0).normalize(), (cont - c0).normalize());
                let phi = u0.dot(u1).clamp(-1.0, 1.0).acos();
                let zc = u0.cross(u1).normalize();
                let mut poly = vec![p0, foot];
                for i in 1..200 {
                    let a = phi * i as f64 / 200.0;
                    poly.push(c0 + (u0 * a.cos() + zc.cross(u0) * a.sin()) * size);
                }
                poly.push(cont);
                poly.extend(ws[..k].iter().rev().map(|x| x.0));
                cuts.push(Cut { area: area(&poly), dlen: size * phi - lam - ws[k].1, sharp: vec![] });
            }
        }
    }
    cuts
}

/// Expected removed volume. `rims`: rim faces rounded with `s`; `vert`: the vertical corner edges cut with this size (None: the
/// corners stay sharp, the rim strips meet in a mitre) and whether that cut is a chamfer. Corners with an exterior angle below 1e-3
/// are tangent joins (nothing to cut there).
pub fn expected(b0: &Body, rims: &[&str], chamfer: bool, s: f64, vert: Option<(f64, bool)>) -> f64 {
    let th: Vec<f64> = exterior_angles(b0, "F2.C1");
    let mut sharp: Vec<f64> = th.iter().copied().filter(|&t| t > 1e-3).collect();
    let mut out = 0.0;
    let mut dlen = 0.0;
    if let Some((size, vch)) = vert {
        let cuts = corner_cuts(b0, size, vch);
        out += H * cuts.iter().map(|c| c.area).sum::<f64>();
        dlen = cuts.iter().map(|c| c.dlen).sum();
        sharp = cuts.iter().flat_map(|c| c.sharp.clone()).collect();
    }
    let (a, xr) = section(chamfer, s);
    for r in rims {
        let len = rim_length(b0, r) + dlen;
        let miter: f64 = sharp.iter().map(|t| 2.0 * (t / 2.0).tan()).sum();
        let smooth_turn = 2.0 * PI - sharp.iter().sum::<f64>();
        out += a * (len - xr * (miter + smooth_turn));
    }
    out
}

pub struct Outcome {
    pub body: Option<Body>,
    pub errors: Vec<String>,
    pub secs: f64,
}

/// Fillet / chamfer on the rims `rims` (size `s`; a chamfer when `chamfer`) and the vertical edges `vert` = (size, chamfer): picked in
/// the same feature when `together` (both of one kind; the vertical picks carry their own radius), else the vertical edges first in a
/// feature of their own and then the rim, picked afresh on that body.
pub fn run(sh: &Shape, rims: &[&str], chamfer: bool, s: f64, vert: Option<(f64, bool)>, together: bool) -> Outcome {
    let (b0, e0) = regen_doc(&doc(sh, ""));
    let b0 = b0.expect("base");
    assert!(e0.is_empty(), "{e0:?}");
    let feat = |id: &str, picks: Vec<String>, size: f64, ch: bool| {
        let (ty, field) = if ch { ("chamfer", "distance") } else { ("fillet", "radius") };
        format!(r#",{{"id":"{id}","type":"{ty}","{field}":{size},"edges":[{}]}}"#, picks.join(","))
    };
    let rim: Vec<String> = rims.iter().flat_map(|f| rim_picks(&b0, f, None)).collect();
    let t = std::time::Instant::now();
    let extra = match vert {
        None => feat("F3", rim, s, chamfer),
        Some((rv, vch)) if together => {
            assert_eq!(vch, chamfer, "a feature is one kind");
            let mut p = rim;
            p.extend(vertical_picks(&b0, if vch { None } else { Some(rv) }));
            feat("F3", p, s, chamfer)
        }
        Some((rv, vch)) => {
            let first = feat("F3", vertical_picks(&b0, None), rv, vch);
            if rims.is_empty() {
                let (body, errors) = regen_doc(&doc(sh, &first));
                return Outcome { body, errors, secs: t.elapsed().as_secs_f64() };
            }
            let (b1, e1) = regen_doc(&doc(sh, &first));
            let Some(b1) = b1.filter(|_| e1.is_empty()) else {
                return Outcome { body: None, errors: e1, secs: t.elapsed().as_secs_f64() };
            };
            let rim2: Vec<String> = rims.iter().flat_map(|f| rim_picks(&b1, f, None)).collect();
            format!("{first}{}", feat("F4", rim2, s, chamfer))
        }
    };
    let (body, errors) = regen_doc(&doc(sh, &extra));
    Outcome { body, errors, secs: t.elapsed().as_secs_f64() }
}

/// Largest curvature along the rim edges of the top cap (1/mm).
pub fn corner_curvature(b0: &Body) -> f64 {
    let f = b0.find_face("F2.C1").unwrap();
    let mut worst = 0.0f64;
    for e in b0.edge_ids().filter(|&e| b0.edge_faces(e).len() == 2 && b0.edge_faces(e).contains(&f)) {
        let ed = b0.edge(e);
        for i in 0..=400 {
            let t = ed.t0 + (ed.t1 - ed.t0) * i as f64 / 400.0;
            let [_, g, g2] = ed.curve.derivs2_arr(t);
            worst = worst.max(g.cross(g2).len() / g.len().powi(3));
        }
    }
    worst
}

/// Outcome of one case judged: `Ok(())` built and right, `Err(why)`.
pub fn judge(sh: &Shape, rims: &[&str], chamfer: bool, s: f64, vert: Option<(f64, bool)>, together: bool, o: &Outcome) -> Result<(), String> {
    let (b0, _) = regen_doc(&doc(sh, ""));
    let b0 = b0.unwrap();
    let Some(b) = o.body.as_ref().filter(|_| o.errors.is_empty()) else { return Err(format!("refused {:?}", o.errors)) };
    let bad = ck_check::validate(b, 1e-6);
    if !bad.is_empty() {
        return Err(format!("invalid {:?}", bad.iter().map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>()));
    }
    let want = expected(&b0, rims, chamfer, s, vert);
    let got = vol(&b0) - vol(b);
    // (the mass integral of the whole body is good to a few 1e-6 of its volume: that is the floor of a small removal)
    // (the mitre term of the estimate takes the walls as straight over the mitre's reach, `size tan(theta/2)`: where the size is more than
    // half the end's radius of curvature that is only good to a few percent)
    let loose = if s * corner_curvature(&b0) > 0.5 && vert.is_none() { 0.03 } else { 0.01 };
    if (got - want).abs() > loose * want.abs() + 5e-6 * vol(&b0) {
        let cuts = vert.map(|(sz, vc)| corner_cuts(&b0, sz, vc).iter().map(|c| format!("[area {:.4} dlen {:.4} sharp {:?}]", c.area, c.dlen, c.sharp)).collect::<String>()).unwrap_or_default();
        return Err(format!("volume: removed {got:.4}, expected {want:.4} (rim length {:.3}, cuts {cuts})", rim_length(&b0, "F2.C1")));
    }
    let _ = together;
    Ok(())
}

/// The rim first (feature F3, size `s`), then the vertical edges (F4, size `rv`), picked on the rounded body.
pub fn run_rim_first(sh: &Shape, rim_face: &str, s: f64, rv: f64) -> Outcome {
    let (b0, _) = regen_doc(&doc(sh, ""));
    let b0 = b0.unwrap();
    let t = std::time::Instant::now();
    let first = format!(r#",{{"id":"F3","type":"fillet","radius":{s},"edges":[{}]}}"#, rim_picks(&b0, rim_face, None).join(","));
    let (b1, e1) = regen_doc(&doc(sh, &first));
    let Some(b1) = b1.filter(|_| e1.is_empty()) else { return Outcome { body: None, errors: e1, secs: t.elapsed().as_secs_f64() } };
    // the vertical edges now: straight edges between two wall faces
    let picks: Vec<String> = b1
        .edge_ids()
        .filter(|&e| {
            let fs = b1.edge_faces(e);
            fs.len() == 2 && matches!(b1.edge(e).curve, Curve::Line { .. }) && fs.iter().all(|&f| b1.face_id_str(f).contains(".S."))
        })
        .map(|e| pick_json(&b1, e, None))
        .collect();
    let second = format!(r#"{first},{{"id":"F4","type":"fillet","radius":{rv},"edges":[{}]}}"#, picks.join(","));
    let (body, errors) = regen_doc(&doc(sh, &second));
    Outcome { body, errors, secs: t.elapsed().as_secs_f64() }
}
