//! Long helical sweeps: the BenchCAD torsion spring (a 12-turn helix sweep united with two tangent
//! legs) used to take minutes (every closest-point query scanned all 577 knot spans, the SSI of the
//! legs' caps marched every turn, the containment test integrated the whole sweep). The budgets
//! below are an order of magnitude over the current time (profile `fast`) and far under the old one.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use std::time::Instant;

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

fn regen_file(name: &str, upto: Option<u32>) -> (ck_topo::body::Body, String, f64) {
    let text = std::fs::read_to_string(format!("{}/../../tools/bench/benchcad/{name}.ckpart.json", env!("CARGO_MANIFEST_DIR"))).unwrap();
    let part = Part::parse(&text).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts { up_to: upto, ..RegenOpts::default() }, &mut ex);
    let dt = t.elapsed().as_secs_f64();
    (ex.0.body.clone().expect("a body"), r.to_json().to_string(), dt)
}

#[test]
fn torsion_spring_regens_fast_with_two_lumps() {
    let (b, errs, dt) = regen_file("torsion_spring_000054", None);
    assert!(errs.contains(r#""errors":[]"#), "{}", &errs[..errs.len().min(300)]);
    assert_eq!(b.num_lumps(), 2, "coil and the far leg are disjoint solids");
    let m = ck_topo::mass::mass_props(&b);
    // closed form: coil A 2 pi R N with N = height / pitch (not 12), plus the two legs (cylinders), minus
    // the sliver where the second leg's cap meets the coil's start cap
    let a = std::f64::consts::PI * 3.52 * 3.52;
    let coil = a * std::f64::consts::TAU * 56.25 * (206.84 / 17.24);
    let want = coil + a * 220.5 + a * 162.87;
    assert!((m.volume - want).abs() < 5.0, "volume {} vs closed form {want}", m.volume);
    assert!(ck_check::validate(&b, 1e-6).is_empty());
    assert!(dt < 120.0, "regen took {dt:.1} s");
}

#[test]
fn helix_sweep_alone_is_quick_and_exact() {
    let (b, errs, dt) = regen_file("torsion_spring_000054", Some(4));
    assert!(errs.contains(r#""errors":[]"#));
    let m = ck_topo::mass::mass_props(&b);
    let a = std::f64::consts::PI * 3.52 * 3.52;
    let want = a * std::f64::consts::TAU * 56.25 * (206.84 / 17.24);
    assert!((m.volume - want).abs() / want < 2e-5, "volume {} vs {want}", m.volume);
    assert!(dt < 60.0, "sweep took {dt:.1} s");
}

fn run_text(text: &str) -> (ck_topo::body::Body, String, f64) {
    let part = Part::parse(text).unwrap();
    let mut ex = Exec(Brep::new());
    let t = Instant::now();
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (ex.0.body.clone().expect("a body"), r.to_json().to_string(), t.elapsed().as_secs_f64())
}

/// Two turns of a helix (R 20, pitch 7) sampled 12 points per turn, as a `sketch3d` spline; the profile sits
/// on a plane normal to the path at its start. Pinned against the original kernel: volume, area, centre of
/// mass and principal moments of the sweep (the rectangle fixes the frame the section is carried by).
fn spline_sweep_part(profile: &str) -> String {
    let pts: Vec<String> = (0..=24)
        .map(|i| {
            let th = std::f64::consts::TAU * 2.0 * i as f64 / 24.0;
            format!(r#"{{"id":"p{}","x":{},"y":{},"z":{}}}"#, i + 1, 20.0 * th.cos(), 7.0 * th / std::f64::consts::TAU, -20.0 * th.sin())
        })
        .collect();
    let ids: Vec<String> = (0..=24).map(|i| format!(r#""p{}""#, i + 1)).collect();
    format!(
        r#"{{"format":"ck-part","version":1,"name":"t","features":[
        {{"type":"sketch3d","id":"P","points":[{}],"entities":[{{"type":"spline","id":"e1","pts":[{}]}}],"constraints":[]}},
        {{"type":"plane","id":"PN","mode":"normal","path":"P"}},
        {{"type":"sketch","id":"S","plane":{{"kind":"feature","id":"PN"}},{profile}}},
        {{"type":"sweep","id":"W","op":"add","sketch":"S","path":{{"kind":"sketch","id":"P"}}}}]}}"#,
        pts.join(","),
        ids.join(",")
    )
}

#[test]
fn spline_path_sweep_matches_the_original_kernel() {
    let rel = |a: f64, b: f64| (a - b).abs() / b.abs().max(1.0);
    // circle r 2
    let (b, errs, _) = run_text(&spline_sweep_part(r#""points":[{"id":"p1","x":0,"y":0}],"entities":[{"type":"circle","id":"e1","c":"p1","r":2}],"constraints":[]"#));
    assert!(errs.contains(r#""errors":[]"#), "{}", &errs[..errs.len().min(300)]);
    let m = ck_topo::mass::mass_props(&b);
    assert!(rel(m.volume, 3161.5405511200897) < 2e-5 && rel(m.area, 3186.6732905888784) < 1e-4, "{} {}", m.volume, m.area);
    assert!((m.com.y - 7.0).abs() < 1e-6 && (m.com.x + 0.020853723389439395).abs() < 5e-4);
    // 3 x 1 rectangle: the original carries the section by a rotation-minimizing frame (a Frenet frame gives
    // com y 7.0005 and principal moments 4e-4 off)
    let rect = r#""points":[{"id":"a","x":-1.5,"y":-0.5},{"id":"b","x":1.5,"y":-0.5},{"id":"c","x":1.5,"y":0.5},{"id":"d","x":-1.5,"y":0.5}],"entities":[{"type":"line","id":"e1","p1":"a","p2":"b"},{"type":"line","id":"e2","p1":"b","p2":"c"},{"type":"line","id":"e3","p1":"c","p2":"d"},{"type":"line","id":"e4","p1":"d","p2":"a"}],"constraints":[]"#;
    let (b, errs, _) = run_text(&spline_sweep_part(rect));
    assert!(errs.contains(r#""errors":[]"#));
    assert_eq!(b.face_ids().count(), 6, "4 sides + 2 caps");
    let m = ck_topo::mass::mass_props(&b);
    assert!(rel(m.volume, 754.7622097443573) < 2e-5 && rel(m.area, 2018.6992252964665) < 1e-4, "{} {}", m.volume, m.area);
    assert!((m.com.y - 7.009439429732686).abs() < 3e-4 && (m.com.z + 0.0010921331183723908).abs() < 3e-4, "{:?}", m.com);
    for (got, want) in m.principal.iter().zip([161691.6717217116, 164211.93377658154, 304971.9553681787]) {
        assert!(rel(*got, want) < 1e-4, "{:?}", m.principal);
    }
}

#[test]
fn coil_spring_spline_sweep_with_flat_ground_ends() {
    // variable-pitch coil swept along one spline (sketch3d with tangent end constraints), both ends cut flat:
    // wire 6.94 mm on a 6 mm pitch overlaps itself, the solid is self-overlapping like the dataset's
    let (b, errs, dt) = regen_file("coil_spring_000026", None);
    assert!(errs.contains(r#""errors":[]"#), "{}", &errs[..errs.len().min(300)]);
    assert_eq!(b.num_lumps(), 1);
    let m = ck_topo::mass::mass_props(&b);
    // 92549.4 for the tube, minus 2 x 1621 for the ends (an independent clipped-mesh integral and a
    // closed-form quadrature of the ideal screw agree to 1e-4): 89306
    assert!((m.volume - 89306.2).abs() < 40.0, "volume {}", m.volume);
    assert!((m.volume - 89283.67).abs() / 89283.67 < 1e-3, "dataset volume 89283.67");
    assert!(dt < 180.0, "regen took {dt:.1} s");
}
