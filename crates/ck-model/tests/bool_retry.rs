//! Opt-in bounded boolean perturbation retry (`retryPerturb: true`).
//!
//! Verifies:
//! 1. Without `retryPerturb`, behavior is 100% unchanged (no perturbation, no `booleanPerturbed` warning).
//! 2. With `retryPerturb: true`, grazing/tangent curved-on-curved booleans that otherwise fail
//!    succeed with a valid solid (`ck_check::validate`), and emit a structured `booleanPerturbed`
//!    warning whose displacement (`<= 1e-4 mm`) and rotation (`<= 2e-6 rad`) are strictly bounded.
//! 3. Clean, non-failing booleans with `retryPerturb: true` do NOT perturb and emit NO warning.
//! 4. `combine` features also support `retryPerturb: true`, and `retryPerturb` triggers no `unknownField`.

use std::f64::consts::PI;

use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts, RegenResult};
use ck_model::Part;
use ck_ops::boolean::perturb::{MAX_ROTATION_RAD, MAX_TRANSLATION_MM};

struct Run {
    result: RegenResult,
    brep: Brep,
}

impl Run {
    fn run(features: &str) -> Run {
        let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
        let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
        let mut brep = Brep::new();
        let result = regen::regen(&part, &RegenOpts::default(), &mut brep);
        Run { result, brep }
    }

    fn error_codes(&self) -> Vec<String> {
        self.result.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.code)).collect()
    }

    fn warning_codes(&self) -> Vec<String> {
        self.result.warnings.iter().map(|w| format!("{}:{}", w.feature, w.error.code)).collect()
    }

    fn body(&self) -> &ck_topo::body::Body {
        self.brep.body.as_ref().expect("expected solid body")
    }

    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(self.body()).volume
    }
}

fn param_num(params: &[(String, Json)], key: &str) -> f64 {
    params
        .iter()
        .find(|(k, _)| k == key)
        .and_then(|(_, v)| v.as_f64())
        .unwrap_or_else(|| panic!("missing numeric param `{key}` in {params:?}"))
}

fn assert_bounded_perturbation_warning(run: &Run, feature_id: &str) {
    assert_eq!(run.warning_codes(), vec![format!("{feature_id}:booleanPerturbed")], "warnings: {:?}", run.result.warnings);
    let w = &run.result.warnings[0].error;
    let dx = param_num(&w.params, "dx");
    let dy = param_num(&w.params, "dy");
    let dz = param_num(&w.params, "dz");
    let dist = param_num(&w.params, "distance");
    let rot = param_num(&w.params, "rotation");
    let attempt = param_num(&w.params, "attempt");

    let mag = (dx * dx + dy * dy + dz * dz).sqrt();
    assert!((mag - dist).abs() < 1e-12, "mag={mag} dist={dist}");
    assert!(dist <= MAX_TRANSLATION_MM * (1.0 + 1e-12), "displacement {dist} exceeded {MAX_TRANSLATION_MM}");
    assert!(rot <= MAX_ROTATION_RAD * (1.0 + 1e-12), "rotation {rot} exceeded {MAX_ROTATION_RAD}");
    assert!(dist > 0.0 || rot > 0.0, "perturbation must be non-zero");
    assert!(attempt >= 1.0, "attempt must be >= 1, got {attempt}");
    assert!(
        w.params.iter().any(|(k, v)| k == "detail" && v.as_str().is_some_and(|s| s.contains("perturbation"))),
        "expected detail string in {:?}",
        w.params
    );
}

/// Viviani-style internal tangency: sphere of radius 10 at the origin, cut by a vertical
/// cylinder of radius 5 centered at (5, 0) (internally tangent to the sphere at (10, 0, 0)).
fn viviani_cut(extra: &str) -> String {
    format!(
        r#"{{"type":"sphere","id":"F1","op":"add","center":[0,0,0],"radius":10}},
           {{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":5,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":5}}],"constraints":[]}},
           {{"type":"extrude","id":"F2","op":"cut","sketch":"S2","end":"throughBoth","depth":30{extra}}}"#
    )
}

/// Internally tangent vertical cylinders: r=10 at (0,0) cut by r=5 at (5,0) (tangent along x=10).
fn tangent_cylinders_cut(extra: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":10}}],"constraints":[]}},
           {{"type":"extrude","id":"F1","op":"add","sketch":"S1","end":"blind","depth":10}},
           {{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":5,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":5}}],"constraints":[]}},
           {{"type":"extrude","id":"F2","op":"cut","sketch":"S2","end":"blind","depth":10,"flip":true{extra}}}"#
    )
}

#[test]
fn default_without_retry_perturb_is_unchanged() {
    for (name, features) in [
        ("viviani_omitted", viviani_cut("")),
        ("viviani_false", viviani_cut(r#","retryPerturb":false"#)),
        ("cyls_omitted", tangent_cylinders_cut("")),
        ("cyls_false", tangent_cylinders_cut(r#","retryPerturb":false"#)),
    ] {
        let r = Run::run(&features);
        assert_eq!(r.error_codes(), vec!["F2:booleanFailed"], "{name}");
        assert!(r.warning_codes().is_empty(), "{name}: {:?}", r.warning_codes());
    }
}

#[test]
fn retry_perturb_rescues_viviani_tangent_cut_with_valid_solid_and_bounded_warning() {
    let r = Run::run(&viviani_cut(r#","retryPerturb":true"#));
    assert!(r.error_codes().is_empty(), "errors: {:?}", r.error_codes());
    let issues = ck_check::validate(r.body(), 1e-6);
    assert!(issues.is_empty(), "validation issues: {issues:?}");
    assert!(ck_topo::check::check(r.body(), 1e-6).is_empty());

    // Analytical volume of sphere (R=10) minus internally tangent Viviani cylinder (r=5):
    // V_sphere = 4/3 * pi * R^3, V_removed = 2 * R^3 * (pi/3 - 4/9) -> V_rem = (2/3 * pi + 8/9) * R^3.
    let expected = (2.0 / 3.0 * PI + 8.0 / 9.0) * 1000.0;
    assert!((r.volume() - expected).abs() < 0.2, "vol={} expected={expected}", r.volume());

    assert_bounded_perturbation_warning(&r, "F2");
}

#[test]
fn retry_perturb_rescues_tangent_cylinders_cut_with_valid_solid() {
    let r = Run::run(&tangent_cylinders_cut(r#","retryPerturb":true"#));
    assert!(r.error_codes().is_empty(), "errors: {:?}", r.error_codes());
    let issues = ck_check::validate(r.body(), 1e-6);
    assert!(issues.is_empty(), "validation issues: {issues:?}");
    assert!(ck_topo::check::check(r.body(), 1e-6).is_empty());

    let expected = PI * (10.0 * 10.0 - 5.0 * 5.0) * 10.0;
    assert!((r.volume() - expected).abs() < 0.1, "vol={} expected={expected}", r.volume());

    assert_bounded_perturbation_warning(&r, "F2");
}

#[test]
fn clean_boolean_with_retry_perturb_does_not_perturb_or_warn() {
    // Non-tangent concentric hole cut in a cylinder.
    let mk = |extra: &str| {
        format!(
            r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":10}}],"constraints":[]}},
               {{"type":"extrude","id":"F1","op":"add","sketch":"S1","end":"blind","depth":10}},
               {{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":4}}],"constraints":[]}},
               {{"type":"extrude","id":"F2","op":"cut","sketch":"S2","end":"blind","depth":10,"flip":true{extra}}}"#
        )
    };
    let plain = Run::run(&mk(""));
    let opt_in = Run::run(&mk(r#","retryPerturb":true"#));
    assert!(plain.error_codes().is_empty(), "{:?}", plain.error_codes());
    assert!(opt_in.error_codes().is_empty(), "{:?}", opt_in.error_codes());
    assert!(opt_in.warning_codes().is_empty(), "clean boolean must not warn: {:?}", opt_in.warning_codes());
    assert!((plain.volume() - opt_in.volume()).abs() < 1e-12, "plain={} opt_in={}", plain.volume(), opt_in.volume());
    assert!(ck_check::validate(opt_in.body(), 1e-6).is_empty());
}

#[test]
fn combine_feature_supports_retry_perturb() {
    let mk = |extra: &str| {
        format!(
            r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":10}}],"constraints":[]}},
               {{"type":"extrude","id":"F1","op":"newBody","body":"Host","sketch":"S1","end":"blind","depth":10}},
               {{"type":"sketch","id":"S2","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":5,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":5}}],"constraints":[]}},
               {{"type":"extrude","id":"F2","op":"newBody","body":"Tool","sketch":"S2","end":"blind","depth":10}},
               {{"type":"combine","id":"C1","op":"cut","target":{{"body":"Host"}},"tools":[{{"body":"Tool"}}]{extra}}}"#
        )
    };
    let plain = Run::run(&mk(""));
    assert_eq!(plain.error_codes(), vec!["C1:booleanFailed"]);
    assert!(plain.warning_codes().is_empty());

    let retried = Run::run(&mk(r#","retryPerturb":true"#));
    assert!(retried.error_codes().is_empty(), "{:?}", retried.error_codes());
    assert!(ck_check::validate(retried.body(), 1e-6).is_empty());
    let expected = PI * (10.0 * 10.0 - 5.0 * 5.0) * 10.0;
    assert!((retried.volume() - expected).abs() < 0.1, "vol={} expected={expected}", retried.volume());
    assert_bounded_perturbation_warning(&retried, "C1");
}
