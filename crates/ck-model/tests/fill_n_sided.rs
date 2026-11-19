//! `fill` with `patch: "nSided"`: a closed rim filled by N free-form faces round a centre vertex.
//! The default `fill` (one planar / ruled face) is pinned elsewhere and is not touched.
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts, RegenResult};
use ck_model::Part;

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

const R: f64 = 10.0;
const H: f64 = 5.0;

fn corner(n: usize, i: usize) -> (f64, f64) {
    let a = std::f64::consts::TAU * i as f64 / n as f64;
    (R * a.cos(), R * a.sin())
}

/// A dish: the walls of a regular `n`-gon prism (height `H`) knitted with its floor, open at the top;
/// then `fill_extra` is the member list of the fill feature `F5` (after `"type":"fill","id":"F5"`).
/// `edges` = how many of the `n` rim edges the fill picks.
fn part(n: usize, edges: usize, fill_extra: &str) -> String {
    let mut pts = String::new();
    let mut ents = String::new();
    for i in 0..n {
        let (x, y) = corner(n, i);
        pts += &format!(r#"{}{{"id":"p{i}","x":{x},"y":{y}}}"#, if i > 0 { "," } else { "" });
        ents += &format!(r#"{}{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, if i > 0 { "," } else { "" }, i + 1, (i + 1) % n);
    }
    let picks: Vec<String> = (0..edges)
        .map(|i| {
            let (a, b) = (corner(n, i), corner(n, (i + 1) % n));
            format!(r#"{{"faces":["F2.S.e{}"],"at":[{},{},{H}]}}"#, i + 1, (a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0)
        })
        .collect();
    format!(
        r#"{{"format":"ck-part","version":1,"name":"t","features":[
  {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{pts}],"entities":[{ents}],"constraints":[]}},
  {{"type":"surfaceExtrude","id":"F2","sketch":"S1","end":"blind","depth":{H}}},
  {{"type":"surfacePlanar","id":"F3","sketch":"S1"}},
  {{"type":"knit","id":"F4","surfaces":["F2","F3"]}},
  {{"type":"fill","id":"F5","edges":[{}],{fill_extra}}}
 ]}}"#,
        picks.join(",")
    )
}

fn run(src: &str) -> (RegenResult, Exec) {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (res, ex)
}

fn codes(res: &RegenResult) -> Vec<String> {
    res.errors.iter().map(|e| e.error.code.clone()).collect()
}

fn prism_volume(n: usize) -> f64 {
    0.5 * n as f64 * R * R * (std::f64::consts::TAU / n as f64).sin() * H
}

#[test]
fn n_sided_fill_closes_the_dish() {
    for n in [3usize, 4, 5, 6] {
        for cont in ["G0", "G1"] {
            let (res, ex) = run(&part(n, n, &format!(r#""merge":true,"patch":"nSided","continuity":"{cont}""#)));
            assert!(res.errors.is_empty(), "n={n} {cont}: {:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
            let b = ex.0.body.as_ref().expect("the closed shell is the solid body");
            assert!(ex.0.surfs.is_empty(), "n={n} {cont}: no sheet left");
            // n walls + floor + n patches, named F5.N0 .. F5.N{n-1}
            assert_eq!(b.num_faces(), 2 * n + 1, "n={n} {cont}");
            for k in 0..n {
                assert!(b.find_face(&format!("F5.N{k}")).is_some(), "n={n} {cont}: F5.N{k}");
            }
            // the walls, the floor and the n patches: the seams are shared by exactly two faces
            for e in b.edge_ids() {
                assert_eq!(b.edge_coedges(e).len(), 2, "n={n} {cont}: every edge has two coedges");
            }
            let issues = ck_check::validate(b, 1e-6);
            assert!(issues.is_empty(), "n={n} {cont}: {issues:?}");
            let v = ck_topo::mass::mass_props(b).volume;
            let prism = prism_volume(n);
            if cont == "G0" {
                assert!((v - prism).abs() < 1e-3 * prism, "n={n} G0: flat rim gives the prism, {v} vs {prism}");
            } else {
                // leaving the vertical walls tangentially bulges the cap outwards: more, but not a dome of the full radius
                assert!(v > prism && v < prism + std::f64::consts::PI * R * R * R, "n={n} G1: volume {v} vs prism {prism}");
            }
        }
    }
}

#[test]
fn n_sided_fill_default_is_unchanged() {
    // without `patch` the fill is the one planar face `F5.F`
    let (res, ex) = run(&part(4, 4, r#""merge":true"#));
    assert!(res.errors.is_empty());
    let b = ex.0.body.as_ref().unwrap();
    assert_eq!(b.num_faces(), 6);
    assert!(b.find_face("F5.F").is_some());
}

#[test]
fn n_sided_fill_as_a_sheet() {
    // without merge the patches are a sheet of their own
    let (res, ex) = run(&part(5, 5, r#""merge":false,"patch":"nSided""#));
    assert!(res.errors.is_empty(), "{:?}", codes(&res));
    let (_, sheet) = ex.0.surfs.iter().find(|(id, _)| id == "F5").expect("sheet F5");
    assert_eq!(sheet.num_faces(), 5);
}

#[test]
fn n_sided_fill_refusals() {
    let cases: [(&str, usize, &str, &str); 6] = [
        ("open ring", 3, r#""merge":true,"patch":"nSided""#, "fillNotClosed"),
        ("G2", 4, r#""merge":true,"patch":"nSided","continuity":"G2""#, "fillContinuity"),
        ("zero tolerance", 4, r#""merge":true,"patch":"nSided","patchTolerance":0"#, "fillInvalid"),
        ("negative tolerance", 4, r#""merge":true,"patch":"nSided","patchTolerance":-1"#, "fillInvalid"),
        ("unknown patch", 4, r#""merge":true,"patch":"coons""#, "fillInvalid"),
        ("two edges of four", 2, r#""merge":true,"patch":"nSided""#, "fillNotClosed"),
    ];
    for (name, picks, extra, want) in cases {
        let (res, ex) = run(&part(4, picks, extra));
        assert_eq!(codes(&res), vec![want.to_string()], "{name}: {:?}", res.errors.iter().map(|e| e.error.describe()).collect::<Vec<_>>());
        assert!(ex.0.body.is_none(), "{name}: the dish stays an open sheet");
    }
}

#[test]
fn n_sided_fill_regen_is_deterministic() {
    let src = part(5, 5, r#""merge":true,"patch":"nSided","continuity":"G1""#);
    let (r1, e1) = run(&src);
    let (r2, e2) = run(&src);
    assert!(r1.errors.is_empty() && r2.errors.is_empty());
    let (b1, b2) = (e1.0.body.as_ref().unwrap(), e2.0.body.as_ref().unwrap());
    let ids = |b: &ck_topo::body::Body| b.face_ids().map(|f| b.face_id_str(f)).collect::<Vec<_>>();
    assert_eq!(ids(b1), ids(b2));
    assert_eq!(ck_topo::mass::mass_props(b1).volume, ck_topo::mass::mass_props(b2).volume);
    // an edit that does not touch the fill leaves its result alone; dropping the fill's `patch` returns the default
    let (r3, e3) = run(&src.replace(r#""patch":"nSided","#, ""));
    assert!(r3.errors.is_empty());
    assert!(e3.0.body.as_ref().unwrap().find_face("F5.F").is_some());
}
