//! The conduit snap-clip (brepjs playground example, hand translation in tools/bench/brepjs): fillet F7 r 1.44 on the
//! bore of a slotted ring. The reference rounds only the two straight lip edges where the slot walls meet the bore (its
//! STEP has two r 1.44 cylinders and nothing else), and our build of that selection matches its volume and area to 1e-6.
//! An earlier translation (kept as `fixtures/conduit_snap_clip_4edge.ckpart.json`, also shipped as an app sample) also
//! picked the two bore-end arcs; that builds 1.4 % below the reference, which looked like a wrong blend but is the right
//! answer to a different question: the arcs alone remove exactly the analytic volume of a r 1.44 round swept round the
//! bore over the arc's span (the span grows with the radius because the slot walls are not radial planes).
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

const BENCH: &str = include_str!("../../../tools/bench/brepjs/conduit-snap-clip.ckpart.json");
const FOUR: &str = include_str!("fixtures/conduit_snap_clip_4edge.ckpart.json");
const LINES: &str = r#"{"faces":["F3.S.e2","F6.S.e1"],"at":[0,19.67681447869857,4.96]},{"faces":["F3.S.e2","F6.S.e3"],"at":[0,19.67681447869857,-4.96]}"#;

/// (volume, area) of the regenerated part, with no feature errors and a valid body.
fn build(part: Part) -> (f64, f64) {
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect();
    assert!(errs.is_empty(), "{errs:?}");
    let b = ex.0.body.expect("body");
    let iss = ck_check::validate(&b, 1e-6);
    assert!(iss.is_empty(), "{:?}", iss.iter().map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
    let m = ck_topo::mass::mass_props(&b);
    (m.volume, m.area)
}

fn without_f7(src: &str) -> Part {
    let mut p = Part::parse(src).unwrap();
    p.features.iter_mut().filter(|f| f.id == "F7").for_each(|f| f.suppressed = true);
    p
}

/// Analytic volume a r round removes from the convex edge where a bore of radius `bore` meets an end face square to it,
/// over the arc left between two slot walls at +-`half` (planes parallel to the bore axis): the round's section swept round
/// the axis, each radius `rho` over the angle `2 pi - 2 asin(half / rho)` that the slot leaves of the circle there.
fn arc_round(bore: f64, r: f64, half: f64) -> f64 {
    let n = 200_000;
    let h = r / n as f64;
    (0..n)
        .map(|i| {
            let rho = bore + (i as f64 + 0.5) * h;
            let depth = r - (r * r - (bore + r - rho).powi(2)).sqrt();
            rho * (2.0 * std::f64::consts::PI - 2.0 * (half / rho).asin()) * depth * h
        })
        .sum()
}

#[test]
fn lip_fillet_matches_reference() {
    // the reference (brepjs 20.5.0 / OCCT): tools/bench/brepjs/manifest.json
    let (v, a) = build(Part::parse(BENCH).unwrap());
    assert!((v - 2687.6807671338415).abs() < 1e-3, "volume {v}");
    assert!((a - 2329.112149254925).abs() < 1e-3, "area {a}");
}

#[test]
fn bore_end_arcs_remove_the_analytic_volume() {
    let (v0, _) = build(without_f7(FOUR));
    let arcs = FOUR.replacen(&format!(",{LINES}"), "", 1);
    assert_ne!(arcs, FOUR);
    let (va, _) = build(Part::parse(&arcs).unwrap());
    let want = 2.0 * arc_round(8.0, 1.44, 4.96);
    assert!((v0 - va - want).abs() < 2e-3, "arcs removed {} want {want}", v0 - va);
    // all four edges: the arcs and the lips, less the overlap at the four corners where they meet
    let (v4, _) = build(Part::parse(FOUR).unwrap());
    let (vl, _) = build(Part::parse(BENCH).unwrap());
    let lips = v0 - vl;
    assert!(v0 - v4 > want && v0 - v4 < want + lips, "four edges removed {} (arcs {want}, lips {lips})", v0 - v4);
}
