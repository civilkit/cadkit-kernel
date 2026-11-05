//! Rim corners of the petal ring (`docs/blend-reference-petal.md`): the face census and the corner tori of the
//! fillet table's `daisy-ring-n5-*-r0.8-ref` cases.
//!
//! `BLEND_CENSUS=<id>[,<id>]` prints the census (and every torus) of those fixture cases instead of the checks.
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl FeatureExec for Exec {
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

const CASES: &str = include_str!("fixtures/blend_cases.json");

/// The fixture case `id` built: (body, regen errors, reference volume).
fn build(id: &str) -> (Option<ck_topo::body::Body>, Vec<String>, Option<f64>) {
    let all = ck_json::parse(CASES).unwrap();
    let case = all.get("cases").unwrap().as_arr().unwrap().iter().find(|c| c.get_str("id") == Some(id)).unwrap().clone();
    let mut feats = all.get("bases").unwrap().get(case.get_str("base").unwrap()).unwrap().as_arr().unwrap().to_vec();
    if let Some(pre) = case.get("pre") {
        feats.push(pre.clone());
    }
    feats.push(case.get("fillet").unwrap().clone());
    let doc = format!(r#"{{"format":"ck-part","version":1,"features":{}}}"#, ck_json::to_string(&Json::Arr(feats)));
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.errors.iter().map(|e| e.error.code.clone()).collect();
    (ex.0.body.clone(), errs, case.get("reference").and_then(|r| r.get_f64("volume")))
}

/// Face count per surface kind, in a fixed order.
fn census(b: &ck_topo::body::Body) -> Vec<(&'static str, usize)> {
    let mut out: Vec<(&'static str, usize)> = ["plane", "cylinder", "torus", "cone", "sphere", "spline"].iter().map(|k| (*k, 0)).collect();
    for f in b.face_ids() {
        let k = b.face(f).surface.kind();
        match out.iter_mut().find(|(x, _)| *x == k) {
            Some((_, n)) => *n += 1,
            None => out.push((k, 1)),
        }
    }
    out
}

/// (major, minor) of every torus face.
fn tori(b: &ck_topo::body::Body) -> Vec<(f64, f64)> {
    b.face_ids()
        .filter_map(|f| match &b.face(f).surface {
            ck_geom::surface::Surface::Torus { major, minor, .. } => Some((*major, *minor)),
            _ => None,
        })
        .collect()
}

#[test]
fn rim_corner_census() {
    if let Ok(ids) = std::env::var("BLEND_CENSUS") {
        for id in ids.split(',').filter(|s| !s.is_empty()) {
            let (b, errs, rv) = build(id);
            let Some(b) = b.filter(|_| errs.is_empty()) else {
                println!("{id}: errors {errs:?}");
                continue;
            };
            let v = ck_topo::mass::mass_props(&b).volume;
            println!("{id}: faces {} {:?} volume {v:.4} reference {rv:?} issues {:?}", b.face_ids().count(), census(&b), ck_check::validate(&b, 1e-6).iter().map(|i| i.code).collect::<Vec<_>>());
            for f in b.face_ids() {
                println!("  {} {}", b.face_id_str(f), match &b.face(f).surface {
                    ck_geom::surface::Surface::Torus { frame, major, minor } => format!("torus o={:?} z={:?} R={major:.6} r={minor:.6}", frame.o, frame.z),
                    ck_geom::surface::Surface::Cylinder { frame, r } => format!("cylinder o={:?} z={:?} r={r:.6}", frame.o, frame.z),
                    s => s.kind().to_string(),
                });
            }
        }
        return;
    }
    // (case, faces per kind [plane, cylinder, torus], corner tori of major radius 2r): the census of the reference
    // results (`docs/blend-reference-petal.md`; T: no corner face, the rim fillets meet along their intersection)
    for (id, kinds, corners) in [("daisy-ring-n5-T-r0.8-ref", [12, 20, 10], 0), ("daisy-ring-n5-VT-r0.8-ref", [12, 30, 20], 10), ("daisy-ring-n5-VTB-r0.8-ref", [12, 32, 23], 12)] {
        let (b, errs, rv) = build(id);
        assert!(errs.is_empty(), "{id}: {errs:?}");
        let b = b.unwrap();
        assert!(ck_check::validate(&b, 1e-6).is_empty(), "{id}: {:?}", ck_check::validate(&b, 1e-6));
        let c = census(&b);
        assert_eq!([c[0].1, c[1].1, c[2].1], kinds, "{id}: {c:?}");
        assert_eq!(b.face_ids().count(), kinds.iter().sum::<usize>(), "{id}: {c:?}");
        let r = 0.8;
        let n = tori(&b).iter().filter(|(big, small)| (big - 2.0 * r).abs() < 1e-9 && (small - r).abs() < 1e-9).count();
        assert_eq!(n, corners, "{id}: {:?}", tori(&b));
        let v = ck_topo::mass::mass_props(&b).volume;
        let rv = rv.unwrap();
        assert!((v - rv).abs() < 2e-3, "{id}: volume {v} vs reference {rv}");
    }
}
