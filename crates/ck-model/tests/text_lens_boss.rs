//! A far-away thin lens (a huge-radius spherical shell revolved onto the dial) must not flip point
//! classification near the dial top: the lens face loops are sphere caps whose uv polygon starts on the
//! pole, and without the pole line the closing edge cut a diagonal across the strip — points under the
//! pole read In / Out by chord count, and the boss fuse then failed with `booleanFailed;issue=openEdge`.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

const DOC: &str = include_str!("fixtures/text_lens_boss.json");

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

fn regen_part(src: &str) -> Result<ck_topo::body::Body, Vec<regen::RegenError>> {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    if res.errors.is_empty() {
        Ok(ex.0.body.unwrap())
    } else {
        Err(res.errors)
    }
}

/// The boss extrude (F7) fuses onto the dial under the lens: it must not fail `booleanFailed`,
/// and the result must validate clean.
#[test]
fn boss_fuse_with_far_away_lens() {
    let body = regen_part(DOC).expect("regen errors");
    assert_eq!(body.num_lumps(), 2, "dial + lens stay separate lumps");
    assert_eq!(body.face_ids().count(), 20);
    let issues: Vec<String> = ck_check::validate(&body, 1e-3)
        .iter()
        .map(|i| format!("{} {}", i.code, i.detail))
        .collect();
    assert!(issues.is_empty(), "validate issues: {issues:?}");
}

/// Boss-wall samples on both sides of the dial top (z = 1.9): below reads In, above reads Out —
/// identical with and without the lens features in the part.
#[test]
fn classify_across_dial_top_is_stable_with_lens() {
    let mut j: ck_json::Json = ck_json::parse(DOC).unwrap();
    if let ck_json::Json::Obj(kvs) = &mut j {
        if let Some(f) = kvs.iter_mut().find(|(k, _)| k == "features") {
            if let ck_json::Json::Arr(a) = &mut f.1 {
                a.truncate(4); // dial + lens
            }
        }
    }
    let lens = regen_part(&ck_json::to_string(&j)).expect("dial+lens regen");
    let mut j: ck_json::Json = ck_json::parse(DOC).unwrap();
    if let ck_json::Json::Obj(kvs) = &mut j {
        if let Some(f) = kvs.iter_mut().find(|(k, _)| k == "features") {
            if let ck_json::Json::Arr(a) = &mut f.1 {
                a.truncate(2); // dial only
            }
        }
    }
    let dial = regen_part(&ck_json::to_string(&j)).expect("dial regen");
    let pts = [
        (-9.4538f64, -8.8401f64, 1.8268f64, ck_ops::boolean::classify::Class::In),
        (-9.4538, -8.8401, 2.0229, ck_ops::boolean::classify::Class::Out),
        (-10.1858, -9.2858, 1.8357, ck_ops::boolean::classify::Class::In),
        (-10.1858, -9.2858, 2.0141, ck_ops::boolean::classify::Class::Out),
        (0.0, 0.0, 1.0, ck_ops::boolean::classify::Class::In),
        (0.0, 0.0, 5.0, ck_ops::boolean::classify::Class::Out),
    ];
    for (x, y, z, want) in pts {
        let q = ck_core::linalg::Vec3::new(x, y, z);
        let n = ck_core::linalg::Vec3::new(0.0, 1.0, 0.0);
        for (b, lbl) in [(&lens, "with lens"), (&dial, "dial only")] {
            let got = ck_ops::boolean::classify::Solid::new(b, 1e-6).classify(q, n);
            assert_eq!(got, want, "{lbl} at {q:?}");
        }
    }
}
