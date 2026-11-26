//! Bottle tutorial part (a lofted base, a stem, an eight-edge round on the base, then a 2 mm shell open at the stem top).
//! Fixtures are the part JSON of the tutorial with the thumbnail dropped.
use ck_core::linalg::Vec3;
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
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

/// F8 (the loft of the 63 x 76 R7 base to the 76 x 95 R12 section 114 up) and the round F17 on its 8 bottom edges.
const BASE: &str = include_str!("fixtures/bottle_base.ckpart.json");
/// The stem part before the round and the shell: F8 to F16, then the shell F18.
const STEM_SHELL: &str = include_str!("fixtures/bottle_stem_shell.ckpart.json");

fn regen_doc(doc: &str, radius: Option<f64>, up_to: Option<u32>) -> (Vec<String>, Exec) {
    let doc = match radius {
        Some(r) => doc.replace("\"radius\":5", &format!("\"radius\":{r}")),
        None => doc.to_string(),
    };
    let part = Part::parse(&doc).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts { up_to, ..RegenOpts::default() }, &mut ex);
    let errs = res.errors.iter().map(|e| format!("{} {}", e.feature, e.error.describe())).collect();
    (errs, ex)
}

fn base_volume() -> f64 {
    // F8 alone: the first 12 features (sketches and planes first)
    let (errs, ex) = regen_doc(BASE, None, Some(12));
    assert!(errs.is_empty(), "{errs:?}");
    ck_topo::mass::mass_props(ex.0.body.as_ref().unwrap()).volume
}

/// The inside/outside test of the base loft's rounded corner: a ray from an interior point near the wall crosses the
/// corner wall (one rational cell) twice, and the far crossing used to be lost, so the point read as outside.
#[test]
fn points_inside_the_lofted_corner_are_inside() {
    let (errs, ex) = regen_doc(BASE, None, Some(12));
    assert!(errs.is_empty(), "{errs:?}");
    let b = ex.0.body.as_ref().unwrap();
    let solid = ck_ops::boolean::classify::Solid::new(b, 1e-4);
    let cont = ck_ops::contain::Containment::new(b, 1e-4);
    // (the wall's radius from the corner's axis at z = 1.2 is about 7.1; these lie 0.4 to 0.6 mm inside it)
    for p in [[28.5531, -36.2642, 1.2064], [26.1801, -37.3188, 1.0656], [28.0, -35.0, 1.2], [29.5, -33.0, 1.2]] {
        let q = Vec3::new(p[0], p[1], p[2]);
        assert!(matches!(solid.classify(q, Vec3::Z), ck_ops::boolean::classify::Class::In), "Solid::classify {p:?}");
        assert!(matches!(cont.classify_point(q, 1e-4), ck_ops::contain::Loc::In), "Containment {p:?}");
    }
}

/// A 3 mm and a 1 mm round of the 8 bottom edges of the lofted base build (they were refused with "has the point ...
/// outside the solid"): the round removes material and the result is a valid solid.
#[test]
fn bottom_round_of_the_lofted_base_builds_at_3_and_1_mm() {
    let v0 = base_volume();
    for r in [3.0, 1.0] {
        let (errs, ex) = regen_doc(BASE, Some(r), None);
        assert!(errs.is_empty(), "r={r}: {errs:?}");
        let b = ex.0.body.as_ref().unwrap();
        assert!(ck_topo::check::check(b, 1e-6).is_empty(), "r={r}: {:?}", ck_topo::check::check(b, 1e-6));
        let removed = v0 - ck_topo::mass::mass_props(b).volume;
        // (1 - pi/4) r^2 per mm of the 250 mm rim, within what the tilted wall changes)
        let want = (1.0 - std::f64::consts::FRAC_PI_4) * r * r * 250.0;
        assert!(removed > 0.8 * want && removed < 1.2 * want, "r={r}: removed {removed}, about {want}");
    }
}

/// The 5 mm round of the same edges (the tutorial's size). The ring tool built, but the sliver of one corner wall between
/// the rim and the contact curve (F8.S.e5, or e8 on the retries) read as outside it and stayed in the result, leaving three
/// open edges and a non-manifold seam: the ray used for the inside test passed within 0.005 mm of the contact curve, the
/// corner of the tool's section, where the fitted faces cross it once or twice by rounding. A ray that close to an edge of
/// the ring tool (or that grazes its face) is now ambiguous and the next ray decides.
#[test]
fn bottom_round_of_the_lofted_base_builds_at_5_mm() {
    let v0 = base_volume();
    let (errs, ex) = regen_doc(BASE, None, None);
    assert!(errs.is_empty(), "{errs:?}");
    let b = ex.0.body.as_ref().unwrap();
    assert!(ck_topo::check::check(b, 1e-6).is_empty(), "{:?}", ck_topo::check::check(b, 1e-6));
    let removed = v0 - ck_topo::mass::mass_props(b).volume;
    let want = (1.0 - std::f64::consts::FRAC_PI_4) * 25.0 * 250.0;
    assert!(removed > 0.8 * want && removed < 1.2 * want, "removed {removed}, about {want}");
}

/// The single 4-profile loft with `railSmoothing: "local"` (76 x 95 x 241, no overshoot), shelled 2 mm open at the top: the
/// shell refuses at once (a wall of the loft curves with a 0.02 mm radius: the top section is nearly a point) instead of
/// searching for minutes.
#[test]
fn the_single_local_loft_refuses_a_2_mm_shell_at_once() {
    const SINGLE: &str = include_str!("fixtures/bottle_single_loft_local_shell.ckpart.json");
    let t = std::time::Instant::now();
    let (errs, ex) = regen_doc(SINGLE, None, None);
    let secs = t.elapsed().as_secs_f64();
    let b = ex.0.body.as_ref().unwrap();
    let (lo, hi) = b.bbox();
    assert!(((hi.x - lo.x) - 76.0).abs() < 0.1 && ((hi.y - lo.y) - 95.0).abs() < 0.1 && ((hi.z - lo.z) - 241.0).abs() < 0.1, "loft box {lo:?} {hi:?}");
    assert_eq!(errs.len(), 1, "{errs:?}");
    assert!(errs[0].starts_with("F9 shellTooThick") && errs[0].contains("curves too tightly"), "{errs:?}");
    // The point of the test is "refused in seconds, not minutes" (it used to search for minutes). A box shared with other
    // jobs ran it in 38 s while parallel tests took the cores, so the bound is a multiple of the measured time, not a tight one;
    // the time is printed so the real figure stays visible in the logs.
    eprintln!("single local loft shell refusal: {secs:.1} s");
    assert!(secs < 120.0, "{secs} s");
}

/// Total length of the solid along the line through `p` in direction `dir` (from `-r` to `r`, step `h`), by point classification.
fn solid_length(b: &ck_topo::body::Body, p: Vec3, dir: Vec3, r: f64, h: f64) -> (f64, usize) {
    let cont = ck_ops::contain::Containment::new(b, 1e-4);
    let (mut len, mut runs, mut prev) = (0.0, 0, false);
    let n = (2.0 * r / h) as usize;
    for k in 0..=n {
        let q = p + dir * (-r + h * k as f64);
        let inside = matches!(cont.classify_point(q, 1e-4), ck_ops::contain::Loc::In);
        if inside {
            len += h;
            if !prev {
                runs += 1;
            }
        }
        prev = inside;
    }
    (len, runs)
}

/// The 2 mm shell open at the stem top on the two-loft bottle (it failed with "EDGE_OFF_SURFACE ... pcurve deviates 6.7e-4": the
/// lofted faces meet their neighbours tangent only to the loft's fit tolerance, so their 2 mm offsets miss each other by 7e-4
/// at the crease, and the one edge of the cavity there cannot lie on both). The wall is built, valid, and 2 mm thick.
#[test]
fn the_2_mm_shell_of_the_two_loft_bottle_builds_with_a_wall_of_2_mm() {
    let (errs, ex) = regen_doc(STEM_SHELL, None, Some(16));
    assert!(errs.is_empty(), "{errs:?}");
    let body = ex.0.body.as_ref().unwrap();
    let open = body.find_face("F16.C1").unwrap();
    let t = 2.0;
    let shelled = ck_ops::shell::shell(body, &[open], t, false, ck_core::linalg::EPS_LINEAR).expect("the 2 mm shell builds");
    assert!(ck_check::validate(&shelled, 1e-6).is_empty(), "{:?}", ck_check::validate(&shelled, 1e-6));
    // volume: a wall t thick over the closed surface (less the opened cap), less what the convex curvature gives back
    let (mb, ms) = (ck_topo::mass::mass_props(body), ck_topo::mass::mass_props(&shelled));
    let wall = ms.volume;
    let upper = t * mb.area;
    assert!(wall > 0.85 * upper && wall < 1.0 * upper, "wall volume {wall}, area x t = {upper}");
    // thickness by ray casts through the side walls at three heights: two runs (the two walls), each about t across
    // (the walls lean by a few degrees, so a horizontal chord is a little longer than t)
    for z in [20.0, 60.0, 100.0] {
        let (len, runs) = solid_length(&shelled, Vec3::new(0.0, 0.0, z), Vec3::X, 60.0, 0.02);
        assert_eq!(runs, 2, "z={z}");
        assert!(len > 2.0 * t * 0.97 && len < 2.0 * t * 1.25, "z={z}: wall chords add up to {len}, want about {}", 2.0 * t);
    }
}

/// The tutorial chain: the two-loft bottle, a round of its 8 bottom edges (F17), the 2 mm shell open at the stem top (F18), a round
/// of the stem's outer top edge (F19). The shell used to refuse (the 5 mm round's offset surface did not fit: its continuation past
/// the face folds over; the 2 mm shell's creases between lofted faces lie 7e-4 off their faces) and the rim round then refused the
/// shelled body for the same crease ("the blend changed geometry it does not touch"): the crease's tolerance was lost in the rebuild.
const CHAIN: &str = include_str!("fixtures/bottle_chain.ckpart.json");

#[test]
fn the_bottle_chain_of_rounds_and_a_2_mm_shell_builds() {
    for (base, rim) in [(5.0, 2.0), (3.5, 0.4)] {
        let doc = CHAIN.replace("\"radius\":5", &format!("\"radius\":{base}")).replace("\"radius\":2,", &format!("\"radius\":{rim},"));
        let (errs, ex) = regen_doc(&doc, None, None);
        assert!(errs.is_empty(), "base {base} rim {rim}: {errs:?}");
        let b = ex.0.body.as_ref().unwrap();
        assert!(ck_check::validate(b, 1e-6).is_empty(), "base {base} rim {rim}: {:?}", ck_check::validate(b, 1e-6));
        // wall: 2 mm over the closed outer surface (half of the wall's two sides), a little less where the surface is convex
        let m = ck_topo::mass::mass_props(b);
        let outer = 0.5 * m.area;
        assert!(m.volume > 0.85 * 2.0 * outer && m.volume < 1.05 * 2.0 * outer, "base {base}: wall volume {} for outer area {outer}", m.volume);
        for z in [30.0, 60.0, 100.0] {
            let (len, runs) = solid_length(b, Vec3::new(0.0, 0.0, z), Vec3::X, 60.0, 0.02);
            assert_eq!(runs, 2, "base {base} z={z}");
            assert!(len > 3.88 && len < 5.0, "base {base} z={z}: the two walls add up to {len}, want about 4");
        }
    }
}
