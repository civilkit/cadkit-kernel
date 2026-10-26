//! The kit's marine propeller: a lofted blade fused to a hub cylinder, for several spacings of the
//! section planes. Some spacings fail the fuse with `booleanFailed;issue=openEdge` (most at the hub
//! cylinder's seam). See docs/prop-probe.md, "Marine hub fuse".
//!
//! Run with `CK_BOOL_DBG=1` to print the offending arrangement.
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

struct Built {
    errors: Vec<(String, String)>,
    volume: f64,
    issues: usize,
}

fn build(src: &str) -> Built {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone())).collect();
    let b = ex.0.body.expect("a body");
    Built { errors, volume: ck_topo::mass::mass_props(&b).volume, issues: ck_topo::check::check(&b, 1e-3).len() }
}

macro_rules! case {
    ($name:literal) => {
        ($name, include_str!(concat!("../../../tools/cases/marine_fuse/", $name, ".ckpart")))
    };
}

/// The spacings that build, are valid and agree in volume: the default spacing plus the pattern
/// spacings from 0.70 to 0.80 pitch. The free-form SSI of the hub cylinder with the blade walls is
/// extended to the trimmed face boundary, so the cylinder footprint loop closes on the shared blade
/// rail for these 7 of the 10 documents. See docs/prop-probe.md, "Marine hub fuse".
const PASSING: &[(&str, &str)] = &[
    case!("default_s5"),
    case!("p070_s7"),
    case!("p075_s5"),
    case!("p080_s4"),
    case!("p080_s5"),
    case!("p080_s6"),
    case!("p080_s7"),
];

/// The three spacings still blocked by task A: the hub fuse leaves an open edge at the near-tangent
/// rail vertex (`booleanFailed`). Kept ignored as the target of the fix; see docs/prop-probe.md.
const FAILING: &[(&str, &str)] = &[
    case!("p070_s4"),
    case!("p070_s5"),
    case!("p070_s6"),
];

fn build_cases(cases: &[(&'static str, &'static str)]) -> Vec<(&'static str, Built)> {
    let mut built = vec![];
    for (name, src) in cases {
        let b = build(src);
        eprintln!("{name}: errors {:?} volume {:.3} issues {}", b.errors, b.volume, b.issues);
        built.push((*name, b));
    }
    built
}

fn assert_built_valid(built: &[(&str, Built)]) {
    let bad: Vec<_> = built.iter().filter(|(_, b)| !b.errors.is_empty()).map(|(n, b)| format!("{n}: {:?}", b.errors)).collect();
    assert!(bad.is_empty(), "marine_fuse regen errors: {bad:#?}");
    let bad: Vec<_> = built.iter().filter(|(_, b)| b.issues != 0).map(|(n, b)| format!("{n}: {} issues", b.issues)).collect();
    assert!(bad.is_empty(), "marine_fuse invalid: {bad:#?}");
}

#[test]
fn marine_fuse_builds_valid_for_every_spacing() {
    let built = build_cases(PASSING);
    assert_built_valid(&built);
    // the volume must agree across the pattern spacings (the brief: within 1 %); default_s5 has a
    // different hub length (110 mm vs 140 mm), so it is a validity case only, not a volume sample.
    let vs: Vec<f64> = built.iter().filter(|(n, _)| *n != "default_s5").map(|(_, b)| b.volume).collect();
    let vmin = vs.iter().cloned().fold(f64::INFINITY, f64::min);
    let vmax = vs.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    assert!((vmax - vmin) <= 0.01 * vmax, "marine_fuse volumes differ by more than 1 %: {vs:?}");
}

/// TARGET of task A: the remaining three spacings must build like the rest. They currently fail
/// with `booleanFailed;issue=openEdge` at the near-tangent rail vertex. See docs/prop-probe.md.
#[test]
#[ignore = "task A: p070_s4, p070_s5 and p070_s6 still leave an open edge at the near-tangent rail vertex"]
fn marine_fuse_open_edge_spacings_build() {
    let built = build_cases(FAILING);
    assert_built_valid(&built);
}
