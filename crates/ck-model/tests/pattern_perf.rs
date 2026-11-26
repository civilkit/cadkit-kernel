//! Patterns of cut / hole / boss features: the piece-wise box rule of `pattern::accumulate` must give exactly what the old whole-tool
//! box rule gave (same faces and names, same volume), and a grid pattern must not go through a union per copy.
//! The correctness test is always on; the timing test is `#[ignore]`d:
//!   cargo-queue test --profile fast -p ck-model --test pattern_perf -- --ignored --nocapture
use ck_model::features::{pattern::LEGACY_ACCUMULATE, Brep};
use ck_model::regen::{self, RegenOpts};
use ck_model::Part;
use ck_topo::mass::mass_props;
use std::sync::atomic::Ordering;
use std::time::Instant;

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

fn part(features: &str) -> Part {
    Part::parse(&format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#)).unwrap()
}

type Out = (Vec<String>, Vec<String>, f64, Vec<String>, String);

/// (error codes, warning codes, volume, face names sorted, bbox) of a regen.
fn run(p: &Part) -> Out {
    let mut ex = Exec(Brep::new());
    let res = regen::regen(p, &RegenOpts::default(), &mut ex);
    let errs = res.errors.iter().map(|e| e.error.describe()).collect();
    let warns = res.warnings.iter().map(|e| e.error.describe()).collect();
    let Some(b) = ex.0.body.as_ref() else { return (errs, warns, 0.0, vec![], String::new()) };
    let mut names: Vec<String> = b.face_ids().map(|f| format!("{:?}", b.face(f).name)).collect();
    names.sort();
    (errs, warns, mass_props(b).volume, names, format!("{:?}", b.bbox()))
}

fn hole(id: &str, sketch: &str, d: f64, end: &str, depth: f64) -> String {
    format!(r#"{{"type":"hole","id":"{id}","sketch":"{sketch}","kind":"simple","diameter":{d},"end":"{end}"{}}}"#, if end == "blind" { format!(r#","depth":{depth}"#) } else { String::new() })
}

/// n holes: one hole feature + a linear pattern of `cols` per row (spacing 26 / 25) on a 600 x 500 x 10 plate.
fn holes_grid(n: usize) -> String {
    let cols = if n > 50 { 20 } else { 10.min(n) };
    let rows = (n + cols - 1) / cols;
    let dir2 = if rows > 1 { format!(r#","dir2":{{"kind":"axis","axis":"y"}},"spacing2":25,"count2":{rows}"#) } else { String::new() };
    format!(
        r#"{},{},{},{{"type":"lpattern","id":"F4","seeds":["F3"],"dir":{{"kind":"axis","axis":"x"}},"spacing":26,"count":{cols}{dir2}}}"#,
        base(600.0, 500.0, 10.0),
        pt_sketch("S2", -250.0, -200.0),
        hole("F3", "S2", 10.0, "through", 0.0)
    )
}

fn cases() -> Vec<(String, String)> {
    let b = base(300.0, 200.0, 10.0);
    let lp = |seeds: &str, sp: f64, c: usize, extra: &str| format!(r#",{{"type":"lpattern","id":"F4","seeds":[{seeds}],"dir":{{"kind":"axis","axis":"x"}},"spacing":{sp},"count":{c}{extra}}}"#);
    let thru = |x: f64, y: f64| format!("{},{}", pt_sketch("S2", x, y), hole("F3", "S2", 10.0, "through", 0.0));
    let boss = |x: f64, y: f64| format!(r#"{},{{"type":"extrude","id":"F3","op":"add","sketch":"S2","end":"blind","depth":5}}"#, circles(&[(x, y)], 5.0));
    let g = |sp2: f64, c2: usize| format!(r#","dir2":{{"kind":"axis","axis":"y"}},"spacing2":{sp2},"count2":{c2}"#);
    let mut v: Vec<(String, String)> = vec![];
    for n in [2usize, 3, 10, 50] {
        v.push((format!("grid {n}"), holes_grid(n)));
    }
    v.push(("row 6".into(), format!("{b},{}{}", thru(-120.0, 0.0), lp(r#""F3""#, 30.0, 6, ""))));
    v.push(("overlap row (spacing 6, d 10)".into(), format!("{b},{}{}", thru(-100.0, 0.0), lp(r#""F3""#, 6.0, 8, ""))));
    v.push(("overlap grid".into(), format!("{b},{}{}", thru(-100.0, -20.0), lp(r#""F3""#, 26.0, 5, &g(6.0, 4)))));
    v.push(("crossing the plate edge".into(), format!("{b},{}{}", thru(100.0, 0.0), lp(r#""F3""#, 26.0, 6, &g(25.0, 3)))));
    v.push((
        "circular".into(),
        format!(r#"{b},{},{{"type":"cpattern","id":"F4","seeds":["F3"],"axis":{{"kind":"axis","axis":"z"}},"count":8,"angle":360,"equal":true}}"#, thru(60.0, 0.0)),
    ));
    v.push(("boss grid".into(), format!("{b},{}{}", boss(-100.0, -60.0), lp(r#""F3""#, 26.0, 8, &g(25.0, 5)))));
    v.push(("boss overlap".into(), format!("{b},{}{}", boss(-100.0, -60.0), lp(r#""F3""#, 8.0, 6, &g(25.0, 3)))));
    v.push((
        "two seeds, different depths".into(),
        format!(
            "{b},{},{},{},{}{}",
            pt_sketch("S2", -120.0, -60.0),
            hole("F3", "S2", 10.0, "blind", 4.0),
            pt_sketch("S3", -120.0, -40.0),
            hole("F5", "S3", 8.0, "blind", 7.0),
            lp(r#""F3","F5""#, 26.0, 8, &g(50.0, 3))
        ),
    ));
    v
}

#[test]
fn pattern_pieces_match_the_whole_box_rule() {
    for (name, feats) in cases() {
        let p = part(&feats);
        LEGACY_ACCUMULATE.store(true, Ordering::Relaxed);
        let old = run(&p);
        LEGACY_ACCUMULATE.store(false, Ordering::Relaxed);
        let new = run(&p);
        assert_eq!(old.0, new.0, "{name}: errors");
        assert_eq!(old.1, new.1, "{name}: warnings");
        assert!((old.2 - new.2).abs() <= 1e-9 * old.2.abs().max(1.0), "{name}: volume {} vs {}", old.2, new.2);
        assert_eq!(old.3.len(), new.3.len(), "{name}: face count");
        assert_eq!(old.3, new.3, "{name}: face names");
        assert_eq!(old.4, new.4, "{name}: bbox");
        println!("{name}: {} faces, volume {:.6}, errors {:?}, warnings {:?}", new.3.len(), new.2, new.0, new.1);
    }
    let got = run(&part(&holes_grid(50)));
    let want = 600.0 * 500.0 * 10.0 - 50.0 * std::f64::consts::PI * 25.0 * 10.0;
    assert!(got.0.is_empty() && (got.2 - want).abs() < 5e-3 * want, "grid 50 volume {} want {want}", got.2);
}

fn median_ms(mut f: impl FnMut()) -> f64 {
    f();
    let mut t: Vec<f64> = (0..10)
        .map(|_| {
            let t0 = Instant::now();
            f();
            t0.elapsed().as_secs_f64() * 1e3
        })
        .collect();
    t.sort_by(|a, b| a.partial_cmp(b).unwrap());
    (t[4] + t[5]) / 2.0
}

/// Old vs new, medians of 10 (the old rule at 200 holes is skipped when CK_NO_OLD is set).
#[test]
#[ignore]
fn pattern_timing() {
    let skip_old = std::env::var("CK_NO_OLD").is_ok();
    for n in [10usize, 50, 200] {
        let p = part(&holes_grid(n));
        LEGACY_ACCUMULATE.store(true, Ordering::Relaxed);
        let old = if skip_old && n == 200 { f64::NAN } else { median_ms(|| drop(std::hint::black_box(run(&p)))) };
        LEGACY_ACCUMULATE.store(false, Ordering::Relaxed);
        let new = median_ms(|| drop(std::hint::black_box(run(&p))));
        println!("{n:>4} holes: old {old:10.1} ms   new {new:10.1} ms");
    }
}
