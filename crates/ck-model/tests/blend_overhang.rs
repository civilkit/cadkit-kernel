//! A block that overhangs a wall (sketched on a cube's top face, reaching past the cube's side wall): the
//! four edges where the block's underside, the cube's side wall, the cube top and the block's wall meet are
//! two collinear pairs that each change from convex to concave at the shared vertex. Filleting all four
//! makes the stripes wrap into each other through one B-spline corner patch (four blend faces + the patch,
//! no steps or slivers), G1 to each stripe along its arc, volume on the analytic edge-only estimate
//! `V0 -/+ (1 - pi/4) r^2 * (edge length)` within 2e-4. Selecting the concave pair and the convex pair
//! separately works the same way; three of the four still build.
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

const FIXTURE: &str = include_str!("fixtures/overhang/bad_overhang_fillets.ckpart");
const FIX_MZ: &str = include_str!("fixtures/overhang/bad_overhang_mz.ckpart");
const FIX_UNDER: &str = include_str!("fixtures/overhang/bad_overhang_under.ckpart");
const FIX_TWO: &str = include_str!("fixtures/overhang/bad_overhang_two.ckpart");
const FIX_SKEW: &str = include_str!("fixtures/overhang/bad_overhang_skew.ckpart");

/// The fixture's fillet edge picks 0..3: cube top / block wall (concave), cube top / cube wall (convex),
/// underside / block wall (convex), cube wall / underside (concave).
const PICKS: [&str; 4] = [
    r#"{"faces": ["F2.C1", "F6.S.e3"], "at": [16.9805, 27, 9.193]}"#,
    r#"{"faces": ["F2.C1", "F2.S.e2"], "at": [25, 27, -7.9035]}"#,
    r#"{"faces": ["F6.C0", "F6.S.e3"], "at": [34.2355, 27, 9.193]}"#,
    r#"{"faces": ["F2.S.e2", "F6.C0"], "at": [25, 27, 17.0965]}"#,
];

/// the fixture with the fillet's edge list replaced by picks `which`; `None` keeps the whole fillet off
fn part_with(which: &[usize]) -> String {
    let i = FIXTURE.find("\"edges\": [").expect("edges") + "\"edges\": [".len();
    let j = i + FIXTURE[i..].find("], \"id\": \"F7\"").expect("fillet id");
    let picks: Vec<&str> = which.iter().map(|&k| PICKS[k]).collect();
    format!("{}{}{}", &FIXTURE[..i], picks.join(", "), &FIXTURE[j..])
}

/// the fixture with picks `which`, each carrying its own `radius` (`None` keeps the
/// feature's default of 2): the unequal path (`fillet_unequal`) reads `edges[i].radius`
fn part_with_radii(which: &[usize], radii: &[Option<f64>]) -> String {
    let i = FIXTURE.find("\"edges\": [").expect("edges") + "\"edges\": [".len();
    let j = i + FIXTURE[i..].find("], \"id\": \"F7\"").expect("fillet id");
    let picks: Vec<String> = which
        .iter()
        .zip(radii)
        .map(|(&k, r)| match r {
            None => PICKS[k].to_string(),
            Some(r) => {
                let s = PICKS[k];
                format!("{}{}, \"radius\": {r}}}", &s[..s.len() - 1], "")
            }
        })
        .collect();
    format!("{}{}{}", &FIXTURE[..i], picks.join(", "), &FIXTURE[j..])
}

enum Out {
    Ok { vol: f64, body: ck_topo::body::Body },
    Err { code: String, params: Vec<(String, String)> },
}

fn run(src: &str, drop_last: bool) -> Out {
    let mut part = Part::parse(src).unwrap();
    if drop_last {
        part.features.pop();
    }
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if let Some(e) = r.errors.first() {
        let params = e.error.params.iter().map(|(k, v)| (k.clone(), format!("{v:?}"))).collect();
        return Out::Err { code: e.error.code.clone(), params };
    }
    let b = ex.0.body.clone().unwrap();
    let issues = ck_topo::check::check(&b, 1e-6);
    assert!(issues.is_empty(), "invalid body: {issues:?}");
    Out::Ok { vol: ck_topo::mass::mass_props(&b).volume, body: b }
}

/// The wrap budget for a cube+block body with `patches` overhang corners: 12 original
/// planes + 4 stripes and 1 spline patch per corner, nothing else; every stripe seams
/// G1 to its patch.
fn assert_wraps(b: &ck_topo::body::Body, patches: usize) {
    use ck_topo::body::Role;
    let names: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
    assert_eq!(names.len(), 12 + 5 * patches, "faces {names:?}");
    let f7: Vec<String> = names.into_iter().filter(|n| n.starts_with("F7.")).collect();
    assert_eq!(f7.len(), 5 * patches, "F7 faces {f7:?} (steps or slivers leaked)");
    let mut patch_faces = vec![];
    for k in 0..patches {
        let p = b.find_face(&format!("F7.P{k}")).unwrap_or_else(|| panic!("corner patch P{k} missing"));
        assert!(matches!(b.face(p).surface, ck_geom::surface::Surface::BSpline(_)));
        patch_faces.push(p);
    }
    let blends: Vec<ck_topo::body::FaceId> = f7
        .iter()
        .filter_map(|n| b.find_face(n))
        .filter(|&f| matches!(b.face(f).name.role, Role::Blend(_)))
        .collect();
    assert_eq!(blends.len(), 4 * patches, "blend count");
    for bf in blends {
        let (pf, seam) = patch_faces
            .iter()
            .find_map(|&pf| b.face_edges(pf).into_iter().find(|&e| b.face_edges(bf).contains(&e)).map(|e| (pf, e)))
            .expect("patch|stripe seam");
        let ed = b.edge(seam);
        let pt = ed.curve.eval(0.5 * (ed.t0 + ed.t1));
        let (u1, w1) = b.face(pf).surface.closest_param(pt);
        let (u2, w2) = b.face(bf).surface.closest_param(pt);
        let d = b.outward_normal(pf, u1, w1).normalize().dot(b.outward_normal(bf, u2, w2).normalize()).clamp(-1.0, 1.0);
        let ang = d.acos();
        assert!(ang < 0.02, "{} seam tangent deviation {ang} rad", b.face_id_str(bf));
    }
}

fn vol(o: Out) -> f64 {
    match o {
        Out::Ok { vol, .. } => vol,
        Out::Err { code, params } => panic!("unexpected error {code} {params:?}"),
    }
}

#[test]
fn all_four_edges_wrap_into_one_corner_patch() {
    let (v, b) = match run(FIXTURE, false) {
        Out::Ok { vol, body } => (vol, body),
        Out::Err { code, params } => panic!("unexpected error {code} {params:?}"),
    };
    let v0 = vol(run(FIXTURE, true));
    // concave picks 0, 3 add material, convex ones 1, 2 remove it; every stripe is trimmed a
    // radius short of the corner, which cancels between the two sums
    let k = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0;
    let len = [25.0 - 8.961, 9.193 + 25.0, 43.471 - 25.0, 25.0 - 9.193];
    let want = v0 + k * ((len[0] + len[3]) - (len[1] + len[2]));
    assert!((v - want).abs() < 2e-4 * v0, "volume {v} vs analytic {want}");
    // OCCT topology: 12 planes + 4 blend cylinders + 1 corner patch, nothing else
    assert_wraps(&b, 1);
}

/// Unequal radii on the two pairs (picks 0, 2 = 2; picks 1, 3 = 3): each stripe is cut at
/// the other pair's radius along its own edge, so the hexagon still closes and the patch
/// wraps. OCCT builds the same 17 faces with volume 80462.854136; the edge-only estimate
/// weights each edge by its own radius squared.
#[test]
fn unequal_radii_on_the_two_pairs_still_wrap() {
    let src = part_with_radii(&[0, 1, 2, 3], &[None, Some(3.0), None, Some(3.0)]);
    let (v, b) = match run(&src, false) {
        Out::Ok { vol, body } => (vol, body),
        Out::Err { code, params } => panic!("unexpected error {code} {params:?}"),
    };
    let v0 = vol(run(&src, true));
    let k = 1.0 - std::f64::consts::FRAC_PI_4;
    let len = [25.0 - 8.961, 9.193 + 25.0, 43.471 - 25.0, 25.0 - 9.193];
    // picks 0, 3 concave (r 2, 3); picks 1, 2 convex (r 3, 2)
    let want = v0 + k * (4.0 * (len[0] - len[2]) + 9.0 * (len[3] - len[1]));
    assert!((v - want).abs() < 2e-4 * v0, "volume {v} vs analytic {want}");
    // the OCCT reference solid for these radii
    assert!((v - 80462.854136).abs() < 2e-4 * v, "volume {v} vs OCCT 80462.854136");
    assert_wraps(&b, 1);
}

#[test]
fn concave_pair_and_convex_pair_alone_match_the_analytic_volume() {
    let v0 = vol(run(FIXTURE, true));
    // block 8.961..43.471 x 9.193..47.245 (sketch y -47.245..-9.193) x 9.9 on a 50 x 50 x 27 cube
    let exact = 50.0 * 50.0 * 27.0 + (43.471 - 8.961) * (47.245 - 9.193) * 9.9;
    assert!((v0 - exact).abs() < 1e-6 * exact, "base {v0} vs {exact}");
    let k = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0;
    let len = [25.0 - 8.961, 9.193 + 25.0, 43.471 - 25.0, 25.0 - 9.193];
    // concave edges (picks 0, 3) add material, convex ones (1, 2) remove it
    let concave = vol(run(&part_with(&[0, 3]), false));
    let want = v0 + k * (len[0] + len[3]);
    assert!((concave - want).abs() < 2e-4 * v0, "concave pair {concave} vs {want}");
    let convex = vol(run(&part_with(&[1, 2]), false));
    let want = v0 - k * (len[1] + len[2]);
    assert!((convex - want).abs() < 2e-4 * v0, "convex pair {convex} vs {want}");
}

#[test]
fn three_of_the_four_edges_still_build() {
    let v0 = vol(run(FIXTURE, true));
    let v = vol(run(&part_with(&[0, 1, 2]), false));
    assert!(v < v0 && v > v0 - 60.0, "{v} vs {v0}");
}


// ---------------------------------------------------------------- Task 2: generalisation

#[test]
fn the_mirrored_overhang_wraps_the_same_way() {
    let (v, b) = match run(FIX_MZ, false) {
        Out::Ok { vol, body } => (vol, body),
        Out::Err { code, params } => panic!("unexpected error {code} {params:?}"),
    };
    let v0 = vol(run(FIX_MZ, true));
    let k = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0;
    let len = [25.0 - 8.961, 9.193 + 25.0, 43.471 - 25.0, 25.0 - 9.193];
    let want = v0 + k * ((len[0] + len[3]) - (len[1] + len[2]));
    assert!((v - want).abs() < 2e-4 * v0, "mirror volume {v} vs analytic {want}");
    assert_wraps(&b, 1);
}

#[test]
fn a_block_under_the_ledge_wraps_the_same_way() {
    let (v, b) = match run(FIX_UNDER, false) {
        Out::Ok { vol, body } => (vol, body),
        Out::Err { code, params } => panic!("unexpected error {code} {params:?}"),
    };
    let v0 = vol(run(FIX_UNDER, true));
    let k = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0;
    let len = [25.0 - 8.961, 9.193 + 25.0, 43.471 - 25.0, 25.0 - 9.193];
    let want = v0 + k * ((len[0] + len[3]) - (len[1] + len[2]));
    assert!((v - want).abs() < 2e-4 * v0, "under-ledge volume {v} vs analytic {want}");
    assert_wraps(&b, 1);
}

#[test]
fn a_block_overhanging_two_walls_gets_two_corner_patches() {
    let (v, b) = match run(FIX_TWO, false) {
        Out::Ok { vol, body } => (vol, body),
        Out::Err { code, params } => panic!("unexpected error {code} {params:?}"),
    };
    let v0 = vol(run(FIX_TWO, true));
    let k = (1.0 - std::f64::consts::FRAC_PI_4) * 4.0;
    // eight edges: picks 0,3 and 5,7 concave, 1,2,4,6 convex; lens of picks 4..7 are
    // the second corner's (a) cube-top|x-wall 33.961, (b) top|z-wall 16.039,
    // (c) under|z-wall 22.245, (d) under|x-wall 15.807
    let want = v0 + k * ((16.039 + 15.807 + 16.039 + 15.807) - (34.193 + 18.471 + 33.961 + 22.245));
    assert!((v - want).abs() < 2e-4 * v0, "two-corner volume {v} vs analytic {want}");
    assert_wraps(&b, 2);
}

#[test]
fn a_skewed_corner_is_refused_with_a_hint() {
    match run(FIX_SKEW, false) {
        Out::Err { code, params } => {
            assert_eq!(code, "blendCorner");
            let hint = params.iter().find(|(k, _)| k == "hint").map(|(_, v)| v.as_str()).unwrap_or("");
            assert!(hint.contains("separate features"), "hint: {hint}");
            assert!(params.iter().any(|(k, v)| k == "detail" && v.contains("not perpendicular")), "{params:?}");
        }
        Out::Ok { vol, .. } => panic!("expected a refusal, got a solid of volume {vol}"),
    }
}
