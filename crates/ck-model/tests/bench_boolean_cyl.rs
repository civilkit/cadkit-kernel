//! Timing of a cylinder cut into a box, native (no wasm). Ignored by default; run with
//!   cargo-queue test --profile fast -p ck-model --test bench_boolean_cyl -- --ignored --nocapture
//! (`--profile fast` is release speed without LTO; use `--release` for the shipped optimisation level.)
//! Part (a): the ck-ops `boolean` call alone on prebuilt bodies (box 100 x 60 x 40, cylinder r 10: through, blind, boss; cut, union,
//! intersect). Part (b): the ck-model feature path (regen of the whole part: sketch circle + extrude cut, hole feature, 1/10/50 holes in
//! a plate). Part (c): tessellation of the result at the export presets. Each figure is the median of 20 runs after 3 warm-up runs.
//! The wasm path is `tools/bench_boolean_cyl.mjs`.
use ck_geom::{Frame, Vec2, Vec3};
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_ops::boolean::{boolean, Op};
use ck_topo::body::Body;
use ck_topo::mass::mass_props;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};
use std::time::Instant;

const RUNS: usize = 20;
const WARM: usize = 3;

fn median_ms(mut f: impl FnMut()) -> f64 {
    for _ in 0..WARM {
        f();
    }
    let mut t: Vec<f64> = (0..RUNS)
        .map(|_| {
            let t0 = Instant::now();
            f();
            t0.elapsed().as_secs_f64() * 1e3
        })
        .collect();
    t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    (t[RUNS / 2 - 1] + t[RUNS / 2]) / 2.0
}

fn top_at(y: f64) -> Frame {
    Frame { o: Vec3::new(0.0, y, 0.0), x: Vec3::X, y: Vec3::new(0.0, 0.0, -1.0), z: Vec3::Y }
}
fn v(x: f64, y: f64) -> Vec2 {
    Vec2::new(x, y)
}
fn cyl(r: f64, s0: f64, s1: f64) -> Body {
    let lp = ProfileLoop::new(vec![ProfileSeg::circle("e1", v(0.0, 0.0), r)]);
    extrude("C", &Profile::new(top_at(0.0), vec![ProfileRegion { outer: lp, holes: vec![] }]), ExtrudeOpts { s0, s1, dir: None, draft: 0.0 }).unwrap()
}
fn boxb(w: f64, d: f64, h: f64) -> Body {
    let p = [v(-w / 2.0, -d / 2.0), v(w / 2.0, -d / 2.0), v(w / 2.0, d / 2.0), v(-w / 2.0, d / 2.0)];
    let lp = ProfileLoop::new((0..4).map(|i| ProfileSeg::line(&format!("e{}", i + 1), p[i], p[(i + 1) % 4])).collect());
    extrude("B", &Profile::new(top_at(0.0), vec![ProfileRegion { outer: lp, holes: vec![] }]), ExtrudeOpts { s0: 0.0, s1: h, dir: None, draft: 0.0 }).unwrap()
}

struct Exec(Brep);
impl regen::FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.0.exec(s, f)
    }
}
fn regen_vol(part: &Part) -> f64 {
    let mut ex = Exec(Brep::new());
    let res = regen::regen(part, &RegenOpts::default(), &mut ex);
    assert!(res.errors.is_empty(), "{:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
    mass_props(ex.0.body.as_ref().expect("body")).volume
}

fn rect(w: f64, d: f64) -> String {
    let p = [(-w / 2.0, -d / 2.0), (w / 2.0, -d / 2.0), (w / 2.0, d / 2.0), (-w / 2.0, d / 2.0)];
    let pts: Vec<String> = p.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
    let es: Vec<String> = (0..4).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, i + 1, (i + 1) % 4)).collect();
    format!(
        r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}},{{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":HH}}"#,
        pts.join(","),
        es.join(",")
    )
}
fn base(w: f64, d: f64, h: f64) -> String {
    format!(r#"{},{{"type":"plane","id":"PT","mode":"offset","base":{{"kind":"std","id":"front"}},"offset":{h},"name":"top"}}"#, rect(w, d).replace("HH", &h.to_string()))
}
fn circles(pts: &[(f64, f64)], r: f64) -> String {
    let p: Vec<String> = pts.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"c{i}","x":{x},"y":{y}}}"#)).collect();
    let e: Vec<String> = (0..pts.len()).map(|i| format!(r#"{{"type":"circle","id":"k{i}","c":"c{i}","r":{r}}}"#)).collect();
    format!(r#"{{"type":"sketch","id":"S2","plane":{{"kind":"feature","id":"PT"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, p.join(","), e.join(","))
}
fn pt_sketch(id: &str, x: f64, y: f64) -> String {
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"feature","id":"PT"}},"points":[{{"id":"p1","x":{x},"y":{y}}}],"entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}}"#)
}
fn grid(n: usize, w: f64, d: f64) -> Vec<(f64, f64)> {
    let cols = ((n as f64 * w / d).sqrt()).ceil() as usize;
    let rows = (n + cols - 1) / cols;
    (0..n).map(|i| (-w / 2.0 + w / cols as f64 * (0.5 + (i % cols) as f64), -d / 2.0 + d / rows as f64 * (0.5 + (i / cols) as f64))).collect()
}
fn part(features: &str) -> Part {
    Part::parse(&format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#)).unwrap()
}

#[test]
#[ignore]
fn bench_boolean_cyl() {
    let pi = std::f64::consts::PI;
    println!("--- (a) ck_ops::boolean::boolean on prebuilt bodies, box 100x60x40, tol 1e-6 (ms, median of {RUNS})");
    let bx = boxb(100.0, 60.0, 40.0);
    let boxv = mass_props(&bx).volume;
    let cases: [(&str, Body, Op, f64); 6] = [
        ("cut, cylinder r10 through", cyl(10.0, -5.0, 45.0), Op::Cut, -pi * 100.0 * 40.0),
        ("cut, cylinder r10 blind 20", cyl(10.0, 20.0, 45.0), Op::Cut, -pi * 100.0 * 20.0),
        ("union, cylinder r10 through (sticks out both ends)", cyl(10.0, -5.0, 45.0), Op::Union, pi * 100.0 * 10.0),
        ("union, boss r10 h20 on top", cyl(10.0, 40.0, 60.0), Op::Union, pi * 100.0 * 20.0),
        ("intersect, cylinder r10 through", cyl(10.0, -5.0, 45.0), Op::Intersect, 0.0),
        ("intersect, cylinder r10 inside (s 10..30)", cyl(10.0, 10.0, 30.0), Op::Intersect, 0.0),
    ];
    for (name, tool, op, dv) in &cases {
        let out = boolean(&bx, tool, *op, 1e-6).expect(name);
        let got = mass_props(&out.body).volume;
        let want = match op {
            Op::Intersect => pi * 100.0 * if name.contains("inside") { 20.0 } else { 40.0 },
            _ => boxv + dv,
        };
        assert!((got - want).abs() < 2e-3 * want, "{name}: volume {got} want {want}");
        let ms = median_ms(|| {
            std::hint::black_box(boolean(&bx, tool, *op, 1e-6).unwrap());
        });
        println!("{name:<56} {ms:8.3} ms  faces {}", out.body.num_faces());
    }
    println!("--- (a2) building the operand bodies (extrude of a rectangle / of a circle)");
    println!("{:<56} {:8.3} ms", "box extrude", median_ms(|| drop(std::hint::black_box(boxb(100.0, 60.0, 40.0)))));
    println!("{:<56} {:8.3} ms", "cylinder extrude", median_ms(|| drop(std::hint::black_box(cyl(10.0, -5.0, 45.0)))));

    println!("--- (b) ck-model regen of the whole part, cold (fresh state each run), ms");
    let bs = base(100.0, 60.0, 40.0);
    let bsp = base(300.0, 200.0, 10.0);
    let hole = |d: f64, end: &str| format!(r#"{{"type":"hole","id":"F3","sketch":"S2","kind":"simple","diameter":{d},"end":"{end}"{}}}"#, if end == "blind" { r#","depth":20"# } else { "" });
    let mut rows: Vec<(String, String, f64)> = vec![
        ("box only".into(), bs.clone(), boxv),
        ("extrude cut r10 through".into(), format!(r#"{bs},{},{{"type":"extrude","id":"F3","op":"cut","sketch":"S2","end":"through"}}"#, circles(&[(0.0, 0.0)], 10.0)), boxv - pi * 100.0 * 40.0),
        ("extrude cut r10 blind 20".into(), format!(r#"{bs},{},{{"type":"extrude","id":"F3","op":"cut","sketch":"S2","end":"blind","depth":20}}"#, circles(&[(0.0, 0.0)], 10.0)), boxv - pi * 100.0 * 20.0),
        ("extrude boss r10 h20 (union)".into(), format!(r#"{bs},{},{{"type":"extrude","id":"F3","op":"add","sketch":"S2","end":"blind","depth":20}}"#, circles(&[(0.0, 0.0)], 10.0)), boxv + pi * 100.0 * 20.0),
        ("hole feature d20 through".into(), format!(r#"{bs},{},{}"#, pt_sketch("S2", 0.0, 0.0), hole(20.0, "through")), boxv - pi * 100.0 * 40.0),
        ("hole feature d20 blind 20".into(), format!(r#"{bs},{},{}"#, pt_sketch("S2", 0.0, 0.0), hole(20.0, "blind")), boxv - pi * 100.0 * 20.0),
    ];
    let plate_v = 300.0 * 200.0 * 10.0;
    rows.push(("plate only 300x200x10".into(), bsp.clone(), plate_v));
    for n in [1usize, 10, 50] {
        rows.push((
            format!("plate, {n} circles in ONE extrude cut"),
            format!(r#"{bsp},{},{{"type":"extrude","id":"F3","op":"cut","sketch":"S2","end":"through"}}"#, circles(&grid(n, 260.0, 160.0), 5.0)),
            plate_v - n as f64 * pi * 25.0 * 10.0,
        ));
    }
    for n in [10usize, 50] {
        let (c, r) = (10, (n + 9) / 10);
        let dir2 = if r > 1 { format!(r#","dir2":{{"kind":"axis","axis":"y"}},"spacing2":25,"count2":{r}"#) } else { String::new() };
        rows.push((
            format!("plate, {n} holes: hole feature + linear pattern"),
            format!(r#"{bsp},{},{},{{"type":"lpattern","id":"F4","seeds":["F3"],"dir":{{"kind":"axis","axis":"x"}},"spacing":26,"count":{c}{dir2}}}"#, pt_sketch("S2", -130.0, -80.0), hole(10.0, "through")),
            plate_v - n as f64 * pi * 25.0 * 10.0,
        ));
    }
    let seq: String = grid(50, 260.0, 160.0)
        .iter()
        .enumerate()
        .map(|(i, (x, y))| format!(r#"{},{{"type":"hole","id":"H{i}","sketch":"H{i}s","kind":"simple","diameter":10,"end":"through"}}"#, pt_sketch(&format!("H{i}s"), *x, *y)))
        .collect::<Vec<_>>()
        .join(",");
    rows.push(("plate, 50 separate hole features".into(), format!("{bsp},{seq}"), plate_v - 50.0 * pi * 25.0 * 10.0));
    for (name, feats, want) in &rows {
        let p = part(feats);
        let got = regen_vol(&p);
        let tol = if name.contains("pattern") || name.contains("separate") || name.contains("circles") { 5e-3 } else { 2e-3 };
        assert!((got - want).abs() < tol * want, "{name}: volume {got} want {want}");
        let ms = median_ms(|| drop(std::hint::black_box(regen_vol(&p))));
        println!("{name:<56} {ms:8.3} ms");
    }

    println!("--- (c) tessellation of the cut result (box minus through cylinder), by preset (the tolerance knob)");
    let cut = boolean(&bx, &cyl(10.0, -5.0, 45.0), Op::Cut, 1e-6).unwrap().body;
    for (q, _, _) in ck_tess::mesh::EXPORT_QUALITIES {
        let (tol, _) = ck_tess::mesh::export_tol(&cut, Some(q), None).unwrap();
        let tris = ck_tess::mesh::tessellate_with(&cut, &tol).mesh.tri_face.len();
        let ms = median_ms(|| drop(std::hint::black_box(ck_tess::mesh::tessellate_with(&cut, &tol))));
        println!("{q:<10} chord {:.5} angle {:.2}: {ms:8.3} ms, {tris} triangles", tol.chord, tol.angle);
    }
    println!("{:<10} {:.3} ms (tessellate, regen tolerance)", "default", median_ms(|| drop(std::hint::black_box(ck_tess::mesh::tessellate(&cut)))));
}
