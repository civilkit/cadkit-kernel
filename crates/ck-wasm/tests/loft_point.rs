//! Loft with a point (apex) end section: a sketch holding ONE point entity as the first and/or last
//! section (docs/spec-agent-interface.md, loft "Point sections"). Cone / pyramid volumes, a hull of
//! closed-spline stations ending in points, the near-degenerate end, STEP round trips, a pocket away
//! from the tip and the display mesh at the pole (watertight, unit normals).
use ck_json::Json;
use std::collections::HashMap;
use std::f64::consts::PI;

fn call(msg: &str) -> Json {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap()
}

fn num(j: &Json, k: &str) -> f64 {
    j.get(k).and_then(Json::as_f64).unwrap_or(f64::NAN)
}

/// Feature list builder: stations along the right-plane normal.
struct P {
    feats: Vec<String>,
    n: usize,
}

impl P {
    fn new() -> P {
        P { feats: vec![], n: 0 }
    }
    fn id(&mut self) -> String {
        self.n += 1;
        format!("F{}", self.n)
    }
    /// A sketch at offset `x` with the given points / entities JSON; returns its id.
    fn station(&mut self, x: f64, points: &str, entities: &str) -> String {
        let (pl, sk) = (self.id(), self.id());
        self.feats.push(format!(r#"{{"type":"plane","id":"{pl}","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":{x}}}"#));
        self.feats.push(format!(r#"{{"type":"sketch","id":"{sk}","plane":{{"kind":"feature","id":"{pl}"}},"points":[{points}],"entities":[{entities}],"constraints":[]}}"#));
        sk
    }
    fn circle(&mut self, x: f64, r: f64) -> String {
        self.station(x, r#"{"id":"c","x":0,"y":0}"#, &format!(r#"{{"type":"circle","id":"e1","c":"c","r":{r}}}"#))
    }
    fn square(&mut self, x: f64, a: f64) -> String {
        let c = [(-a, -a), (a, -a), (a, a), (-a, a)];
        let pts: Vec<String> = c.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
        let ents: Vec<String> = (0..4).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, i + 1, (i + 1) % 4)).collect();
        self.station(x, &pts.join(","), &ents.join(","))
    }
    fn point(&mut self, x: f64, px: f64, py: f64) -> String {
        self.station(x, &format!(r#"{{"id":"p","x":{px},"y":{py}}}"#), r#"{"type":"point","id":"e1","p":"p"}"#)
    }
    /// A closed 10-point spline hull section.
    fn hull(&mut self, x: f64, w: f64, d: f64, h: f64) -> String {
        let p = [(0.0, h), (0.55 * w, h + 0.4), (w, h - 1.0), (0.97 * w, -0.3 * d), (0.6 * w, -0.8 * d), (0.0, -d), (-0.6 * w, -0.8 * d), (-0.97 * w, -0.3 * d), (-w, h - 1.0), (-0.55 * w, h + 0.4)];
        let pts: Vec<String> = p.iter().enumerate().map(|(i, (x, y))| format!(r#"{{"id":"p{i}","x":{x},"y":{y}}}"#)).collect();
        let ids: Vec<String> = (0..p.len()).map(|i| format!(r#""p{i}""#)).collect();
        self.station(x, &pts.join(","), &format!(r#"{{"type":"spline","id":"e1","pts":[{}],"closed":true}}"#, ids.join(",")))
    }
    fn loft(&mut self, sks: &[String]) -> String {
        let id = self.id();
        let s: Vec<String> = sks.iter().map(|s| format!(r#""{s}""#)).collect();
        self.feats.push(format!(r#"{{"type":"loft","id":"{id}","op":"add","sketches":[{}]}}"#, s.join(",")));
        id
    }
    fn text(&self) -> String {
        format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, self.feats.join(","))
    }
}

struct Res {
    errors: Vec<String>,
    volume: f64,
    faces: usize,
    issues: usize,
}

fn run(p: &P, tag: &str) -> Res {
    let r = call(&format!(r#"{{"type":"regen","tag":"{tag}","part":{},"opts":{{"solveSketches":true}}}}"#, p.text()));
    let res = r.get("result").unwrap();
    let errors: Vec<String> = res.get_arr("errors").map(|a| a.iter().map(|e| ck_json::to_string(e)).collect()).unwrap_or_default();
    let faces = res.get("body").and_then(|b| b.get_arr("faces")).map_or(0, |f| f.len());
    let m = call(&format!(r#"{{"type":"massProps","tag":"{tag}"}}"#));
    let v = call(&format!(r#"{{"type":"validate","tag":"{tag}"}}"#));
    Res {
        errors,
        volume: m.get("result").map_or(f64::NAN, |m| num(m, "volume")),
        faces,
        issues: v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len()),
    }
}

fn close(a: f64, b: f64, rel: f64) -> bool {
    (a - b).abs() <= rel * b.abs()
}

fn step_roundtrip(tag: &str) -> (f64, usize) {
    let st = call(&format!(r#"{{"type":"step","tag":"{tag}","name":"rt"}}"#));
    let text = st.get("result").and_then(Json::as_str).expect("step text").to_string();
    let msg = ck_json::to_string(&Json::Obj(vec![
        ("type".into(), Json::Str("importStep".into())),
        ("tag".into(), Json::Str("imp".into())),
        ("name".into(), Json::Str("rt".into())),
        ("data".into(), Json::Str(text)),
    ]));
    let imp = call(&msg);
    assert!(imp.get("result").and_then(|r| r.get("body")).map_or(false, |b| !matches!(b, Json::Null)), "import failed {:?}", imp.get("result"));
    let m = call(r#"{"type":"massProps","tag":"imp"}"#);
    let v = call(r#"{"type":"validate","tag":"imp"}"#);
    (num(m.get("result").unwrap(), "volume"), v.get("result").and_then(|r| r.get_arr("issues")).map_or(usize::MAX, |i| i.len()))
}

#[test]
fn circle_to_point_is_a_cone() {
    let mut p = P::new();
    let (a, b) = (p.circle(0.0, 8.0), p.point(30.0, 0.0, 0.0));
    p.loft(&[a, b]);
    let r = run(&p, "cone");
    let want = PI * 64.0 * 30.0 / 3.0;
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(close(r.volume, want, 1e-6), "{} vs {want}", r.volume);
    assert_eq!((r.faces, r.issues), (3, 0));
    let (vi, ii) = step_roundtrip("cone");
    assert!(close(vi, want, 1e-6) && ii == 0, "step {vi} {ii}");
}

#[test]
fn point_first_and_oblique_apex() {
    let mut p = P::new();
    let (a, b) = (p.point(0.0, 5.0, 3.0), p.circle(30.0, 8.0));
    p.loft(&[a, b]);
    let r = run(&p, "cone2");
    let want = PI * 64.0 * 30.0 / 3.0;
    assert!(r.errors.is_empty() && r.issues == 0 && close(r.volume, want, 1e-6), "{:?} {} {}", r.errors, r.volume, r.issues);
    let (vi, ii) = step_roundtrip("cone2");
    assert!(close(vi, want, 1e-6) && ii == 0, "step {vi} {ii}");
}

#[test]
fn square_to_point_is_a_pyramid() {
    let mut p = P::new();
    let (a, b) = (p.square(0.0, 5.0), p.point(12.0, 0.0, 0.0));
    p.loft(&[a, b]);
    let r = run(&p, "pyr");
    let want = 100.0 * 12.0 / 3.0;
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(close(r.volume, want, 1e-6), "{} vs {want}", r.volume);
    assert_eq!((r.faces, r.issues), (5, 0));
    let (vi, ii) = step_roundtrip("pyr");
    assert!(close(vi, want, 1e-6) && ii == 0, "step {vi} {ii}");
}

fn hull(p: &mut P, bow: Option<(f64, f64, f64)>, stern: Option<(f64, f64, f64)>) -> String {
    let mut sks = vec![];
    sks.push(match bow {
        Some((w, d, h)) => p.hull(-60.0, w, d, h),
        None => p.point(-60.0, 0.0, 3.0),
    });
    for (x, w, d, h) in [(-30.0, 14.0, 9.0, 4.0), (0.0, 18.0, 12.0, 5.0), (30.0, 14.0, 9.0, 4.0)] {
        sks.push(p.hull(x, w, d, h));
    }
    sks.push(match stern {
        Some((w, d, h)) => p.hull(60.0, w, d, h),
        None => p.point(60.0, 0.0, 5.0),
    });
    p.loft(&sks)
}

#[test]
fn hull_of_spline_stations_ends_in_two_points() {
    let mut p = P::new();
    hull(&mut p, None, None);
    let r = run(&p, "hull");
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert!(r.volume > 20000.0 && r.volume < 60000.0, "{}", r.volume);
    assert_eq!((r.faces, r.issues), (2, 0), "two lateral halves of the closed spline, no caps");
    let (vi, ii) = step_roundtrip("hull");
    assert!(close(vi, r.volume, 1e-6) && ii == 0, "step {vi} {ii}");
    // one point end and one spline end: the spline end keeps its cap
    let mut p = P::new();
    hull(&mut p, None, Some((2.0, 3.0, 7.0)));
    let r = run(&p, "hull1");
    assert!(r.errors.is_empty() && r.issues == 0 && r.volume > 0.0 && r.faces == 3, "{:?} {} {} {}", r.errors, r.volume, r.issues, r.faces);
}

#[test]
fn near_degenerate_end_is_valid_or_a_clean_error() {
    // a stern section 0.3 mm across: the section itself crosses itself (its offsets do not scale)
    let mut p = P::new();
    hull(&mut p, Some((4.0, 4.0, 3.0)), Some((0.15, 0.2, 0.5)));
    let r = run(&p, "tiny");
    if r.errors.is_empty() {
        assert!(r.issues == 0 && r.volume > 0.0, "ok:true with an invalid body ({} issues)", r.issues);
    } else {
        assert!(r.errors.iter().all(|e| e.contains("loft")), "{:?}", r.errors);
    }
    // a clean tiny circle section (0.01 of the length): snapped to a point, valid
    let mut p = P::new();
    let (a, b) = (p.circle(0.0, 8.0), p.circle(30.0, 0.01));
    p.loft(&[a, b]);
    let r = run(&p, "tinycircle");
    assert!(r.errors.is_empty() && r.issues == 0, "{:?} {}", r.errors, r.issues);
    assert!(close(r.volume, PI * 64.0 * 30.0 / 3.0, 1e-6), "{}", r.volume);
}

#[test]
fn misplaced_point_sections_are_refused() {
    let mut p = P::new();
    let (a, b, c) = (p.circle(0.0, 5.0), p.point(10.0, 0.0, 0.0), p.circle(20.0, 5.0));
    p.loft(&[a, b, c]);
    let r = run(&p, "mid");
    assert!(r.errors.iter().any(|e| e.contains("loftFailed")), "{:?}", r.errors);
}

#[test]
fn pocket_away_from_the_tip() {
    let mut p = P::new();
    let hl = hull(&mut p, None, None);
    let _ = hl;
    let (pl, sk) = (p.id(), p.id());
    p.feats.push(format!(r#"{{"type":"plane","id":"{pl}","mode":"offset","base":{{"kind":"std","id":"right"}},"offset":-10}}"#));
    p.feats.push(format!(r#"{{"type":"sketch","id":"{sk}","plane":{{"kind":"feature","id":"{pl}"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":3}}],"constraints":[]}}"#));
    let id = p.id();
    p.feats.push(format!(r#"{{"type":"extrude","id":"{id}","op":"cut","sketch":"{sk}","end":"blind","depth":8}}"#));
    let base = {
        let mut q = P::new();
        hull(&mut q, None, None);
        run(&q, "hullbase").volume
    };
    let r = run(&p, "pocket");
    assert!(r.errors.is_empty(), "{:?}", r.errors);
    assert_eq!(r.issues, 0);
    assert!(r.volume < base && r.volume > 0.9 * base, "{} vs {base}", r.volume);
}

/// The display mesh of the lofted bodies: watertight (every welded edge used twice) and unit normals,
/// including at the pole.
#[test]
fn tip_mesh_is_watertight_with_unit_normals() {
    // (name, builder, apex x positions): only the mesh within 10% of the length of an apex is audited
    // for cracks (the rest of the lateral faces is the ordinary free-form tessellation)
    for (name, build, apexes) in [
        ("cone", Box::new(|p: &mut P| { let (a, b) = (p.circle(0.0, 8.0), p.point(30.0, 0.0, 0.0)); p.loft(&[a, b]); }) as Box<dyn Fn(&mut P)>, vec![(30.0f32, 3.0f32)]),
        ("pyramid", Box::new(|p: &mut P| { let (a, b) = (p.square(0.0, 5.0), p.point(12.0, 0.0, 0.0)); p.loft(&[a, b]); }), vec![(12.0, 1.2)]),
        ("hull", Box::new(|p: &mut P| { hull(p, None, None); }), vec![(-60.0, 8.0), (60.0, 8.0)]),
    ] {
        let mut p = P::new();
        build(&mut p);
        let part = ck_model::Part::parse(&p.text()).unwrap();
        let mut ex = Ex(ck_model::features::Brep::new());
        ck_model::regen::regen(&part, &ck_model::regen::RegenOpts::default(), &mut ex);
        let body = ex.0.body.take().expect("body");
        let t = ck_tess::mesh::tessellate(&body);
        assert!(t.mesh.failed.is_empty(), "{name}: failed faces {:?}", t.mesh.failed);
        let (pos, nrm) = (&t.mesh.positions, &t.mesh.normals);
        assert_eq!(pos.len(), nrm.len());
        let mut worst: f32 = 0.0;
        for n in nrm.chunks(3) {
            let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!(l.is_finite(), "{name}: non-finite normal");
            worst = worst.max((l - 1.0).abs());
        }
        assert!(worst < 1e-3, "{name}: normal length off by {worst}");
        // welded edge multiplicity
        let key = |i: usize| ((pos[3 * i] * 1e3).round() as i64, (pos[3 * i + 1] * 1e3).round() as i64, (pos[3 * i + 2] * 1e3).round() as i64);
        let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), i32> = HashMap::new();
        for t in 0..pos.len() / 9 {
            for k in 0..3 {
                let (a, b) = (key(3 * t + k), key(3 * t + (k + 1) % 3));
                if a == b {
                    continue;
                }
                *edges.entry(if a < b { (a, b) } else { (b, a) }).or_default() += 1;
            }
        }
        let near = |k: (i64, i64, i64)| apexes.iter().any(|&(x, r)| ((k.0 as f32 / 1e3) - x).abs() < r);
        let open = edges.iter().filter(|(e, &c)| c != 2 && near(e.0) && near(e.1)).count();
        if open > 0 {
            for (e, c) in edges.iter().filter(|(e, &c)| c != 2 && near(e.0) && near(e.1)).take(8) {
                eprintln!("{name}: edge {:?} used {c}x", e);
            }
        }
        assert_eq!(open, 0, "{name}: {open} of {} mesh edges are not shared by exactly two triangles (cracks)", edges.len());
    }
}

struct Ex(ck_model::features::Brep);
impl ck_model::regen::FeatureExec for Ex {
    fn face_count(&self) -> Option<usize> {
        self.0.face_count()
    }
    fn face_frame(&mut self, s: &ck_model::regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.0.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut ck_model::regen::RegenState, f: &ck_model::Feature) -> Result<(), ck_model::regen::FeatureError> {
        self.0.exec(s, f)
    }
}
