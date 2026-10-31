//! The plate of solidcore #1528 / PR #1757: a 42.5 x 20 plate with a half-round end (r 10 about y = 10), 27.5 thick,
//! with a blind r 5 hole along +x through the round end, off its axis (y = 12.5), shelled open at x = 42.5. It came
//! back valid=false (a cavity vertex t/4 off its edge) for t < 1.2, and the refusal above named a thickness (1.18) that
//! gave such a body. Every answer must be a valid body of the right volume or a refusal, and a thickness a refusal
//! offers must build a valid body.
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

const PLATE: &str = r#"[{"type":"sketch","id":"F1","plane":{"kind":"std","id":"front"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p1","x":0,"y":0},{"id":"p2","x":42.5,"y":0},{"id":"p3","x":42.5,"y":0},{"id":"p4","x":42.5,"y":20},{"id":"p5","x":42.5,"y":20},{"id":"p6","x":0,"y":20},{"id":"p7","x":0,"y":10},{"id":"p8","x":0,"y":20},{"id":"p9","x":0,"y":0}],"entities":[{"id":"e1","type":"line","p1":"p1","p2":"p2"},{"id":"e2","type":"line","p1":"p3","p2":"p4"},{"id":"e3","type":"line","p1":"p5","p2":"p6"},{"id":"e4","type":"arc","c":"p7","p1":"p8","p2":"p9"}],"constraints":[]},{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":27.5},{"type":"plane","id":"F3","mode":"offset","base":{"kind":"std","id":"right"},"offset":-10},{"type":"sketch","id":"F4","plane":{"kind":"feature","id":"F3"},"points":[{"id":"O","x":0,"y":0,"fixed":true},{"id":"p1","x":-12.5,"y":12.5}],"entities":[{"id":"e1","type":"circle","c":"p1","r":5}],"constraints":[]},{"type":"extrude","id":"F5","op":"cut","sketch":"F4","end":"blind","depth":17.5,"flip":true}]"#;

/// Errors (code, offered thickness), volume and validation issues of the plate shelled by `t`.
fn shell(t: f64) -> (Vec<(String, Option<f64>)>, f64, Vec<String>) {
    let feats = format!(r#"{},{{"type":"shell","id":"F6","faces":["F2.S.e2"],"thickness":{t}}}]"#, &PLATE[..PLATE.len() - 1]);
    let src = format!(r#"{{"format":"ck-part","version":1,"features":{feats}}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r
        .errors
        .iter()
        .map(|e| {
            let offered = e.error.params.iter().find_map(|(k, v)| match (k.as_str(), v) {
                ("maxSize", Json::Obj(_)) => v.get("value").and_then(|x| x.as_f64()),
                ("maxThickness", Json::Num(x)) if *x > 0.0 => Some(*x),
                _ => None,
            });
            (format!("{}", e.error.describe()), offered)
        })
        .collect();
    let b = ex.0.body.as_ref().unwrap();
    (errs, ck_topo::mass::mass_props(b).volume, ck_check::validate(b, 1e-6).iter().map(|i| format!("{} {} {}", i.code, i.entity, i.detail)).collect())
}

#[test]
fn drilled_round_end_plate_is_valid_or_refused() {
    // independent volumes (numeric integral, briefs/shell-drilled-arc-plate-invalid-or-refused.md)
    for (t, want) in [(0.5, 2803.9955), (1.0, 5489.2415), (1.18, 6425.9380), (2.0, 10400.2124)] {
        let (errs, v, issues) = shell(t);
        eprintln!("t={t}: errors {errs:?} volume {v} issues {issues:?}");
        if errs.is_empty() {
            assert!(issues.is_empty(), "t={t}: ok but invalid {issues:?}");
            assert!((v - want).abs() < 1e-5 * want, "t={t}: volume {v} want {want}");
        } else if let Some(m) = errs[0].1 {
            let (e2, _, i2) = shell(m);
            assert!(e2.is_empty() && i2.is_empty(), "t={t}: offered {m} gives {e2:?} {i2:?}");
        }
    }
}
