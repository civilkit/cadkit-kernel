//! `frame`: structural members swept along a skeleton, corners mitred / butted / teed (features/frame.rs). The checks are
//! independent of the implementation: each member's volume against section area x centre-line length (a plane through the
//! centre line cuts a symmetric prism so that what one end loses the other gains), stock lengths and cut angles worked out by
//! hand from the skeleton, members that do not overlap (the union of all of them has the summed volume), and valid solids.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;
use ck_topo::body::Body;
use ck_json::Json;

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

struct Out {
    body: Option<Body>,
    errors: Vec<String>,
    /// the cut list of the last frame: (segment, length, angles, ends, volume)
    members: Vec<(String, f64, [f64; 2], [String; 2], f64)>,
}

fn run(features: &str) -> Out {
    let src = format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#);
    let part = Part::from_json(&ck_json::parse(&src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let wire = r.to_json();
    let mut members = vec![];
    if let Some(fr) = wire.get_arr("frames").and_then(|a| a.last()) {
        for m in fr.get_arr("members").unwrap() {
            let a = m.get_arr("angles").unwrap();
            let e = m.get_arr("ends").unwrap();
            members.push((
                m.get_str("segment").unwrap().to_string(),
                m.get_f64("length").unwrap(),
                [a[0].as_f64().unwrap(), a[1].as_f64().unwrap()],
                [e[0].as_str().unwrap().to_string(), e[1].as_str().unwrap().to_string()],
                m.get_f64("volume").unwrap(),
            ));
        }
    }
    Out { body: ex.0.body.clone(), errors: r.errors.iter().map(|e| format!("{}:{} {:?}", e.feature, e.error.code, e.error.params)).collect(), members }
}

/// A sketch on the front plane (x, y) with straight lines between the given points: `(id, [x0, y0], [x1, y1])`.
fn skeleton(lines: &[(&str, [f64; 2], [f64; 2])]) -> String {
    let mut pts = vec![];
    let mut ents = vec![];
    for (k, (id, a, b)) in lines.iter().enumerate() {
        pts.push(format!(r#"{{"id":"a{k}","x":{},"y":{}}}"#, a[0], a[1]));
        pts.push(format!(r#"{{"id":"b{k}","x":{},"y":{}}}"#, b[0], b[1]));
        ents.push(format!(r#"{{"type":"line","id":"{id}","p1":"a{k}","p2":"b{k}"}}"#));
    }
    format!(r#"{{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, pts.join(","), ents.join(","))
}

const RECT: [(&str, [f64; 2], [f64; 2]); 4] = [("e1", [0.0, 0.0], [600.0, 0.0]), ("e2", [600.0, 0.0], [600.0, 400.0]), ("e3", [600.0, 400.0], [0.0, 400.0]), ("e4", [0.0, 400.0], [0.0, 0.0])];
const SHS40: &str = r#"{"id":"shs-40x40x2","label":"SHS 40x40x2","kind":"rectTube","w":40,"h":40,"t":2}"#;
const A_SHS40: f64 = 40.0 * 40.0 - 36.0 * 36.0;

fn frame(extra: &str) -> String {
    format!(r#"{{"type":"frame","id":"F2","sketch":"S1","profile":{SHS40}{extra}}}"#)
}

fn volume(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}

/// The whole body is valid, has one lump per member, and the members do not overlap: uniting them all keeps the summed volume.
fn check_body(o: &Out, n: usize) {
    let b = o.body.as_ref().expect("a body");
    assert!(ck_check::validate(b, 1e-6).is_empty(), "{:?}", ck_check::validate(b, 1e-6).iter().map(|i| i.code).collect::<Vec<_>>());
    assert_eq!(b.num_lumps(), n);
    let sum: f64 = o.members.iter().map(|m| m.4).sum();
    assert!((volume(b) - sum).abs() < 1e-6 * sum, "{} vs {sum}", volume(b));
    // union of the separate member solids, one at a time
    let lumps: Vec<Body> = b.lump_ids().map(|l| ck_ops::interference::extract_lump(b, l)).collect();
    let mut acc = lumps[0].clone();
    for l in &lumps[1..] {
        acc = ck_ops::boolean::boolean(&acc, l, ck_ops::boolean::Op::Union, 1e-6).expect("union").body;
    }
    let u = volume(&acc);
    assert!((u - sum).abs() < 1e-5 * sum, "members overlap: union {u} vs sum {sum}");
}

fn near(a: f64, b: f64, tol: f64) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn rectangle_mitred() {
    let o = run(&format!("{},{}", skeleton(&RECT), frame("")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.members.len(), 4);
    for (seg, len, ang, ends, vol) in &o.members {
        let cl = if seg == "e1" || seg == "e3" { 600.0 } else { 400.0 };
        assert!(near(*len, cl + 40.0, 1e-6), "{seg} stock {len}");
        assert!(near(ang[0], 45.0, 1e-6) && near(ang[1], 45.0, 1e-6), "{seg} {ang:?}");
        assert_eq!(ends, &["mitre".to_string(), "mitre".to_string()]);
        assert!(near(*vol, A_SHS40 * cl, 1e-6 * A_SHS40 * cl), "{seg} volume {vol} vs {}", A_SHS40 * cl);
    }
    check_body(&o, 4);
}

#[test]
fn rectangle_butted_and_tee() {
    let mut lines = RECT.to_vec();
    lines.push(("e5", [300.0, 0.0], [300.0, 400.0]));
    let o = run(&format!("{},{}", skeleton(&lines), frame(r#","corner":"butt""#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    // e1 runs through both corners; e2 runs through at the top; e3 through at the left; e4 butts at both; e5 is a tee at both ends
    let want = [("e1", 640.0, ["through", "through"]), ("e2", 400.0, ["butt", "through"]), ("e3", 600.0, ["butt", "through"]), ("e4", 360.0, ["butt", "butt"]), ("e5", 360.0, ["tee", "tee"])];
    for (seg, len, ends) in want {
        let m = o.members.iter().find(|m| m.0 == seg).unwrap();
        assert!(near(m.1, len, 1e-6), "{seg} stock {}", m.1);
        assert_eq!([m.3[0].as_str(), m.3[1].as_str()], ends, "{seg}");
        assert_eq!(m.2, [0.0, 0.0], "{seg} square cuts");
        assert!(near(m.4, A_SHS40 * len, 1e-6 * A_SHS40 * len), "{seg} volume");
    }
    check_body(&o, 5);
}

#[test]
fn triangle_round_tube_mitres_at_sixty_degrees() {
    // equilateral triangle, side 500: each corner is 60 deg between members, so each cut is 60 deg from square
    let h = 500.0 * 3f64.sqrt() / 2.0;
    let tri = [("e1", [0.0, 0.0], [500.0, 0.0]), ("e2", [500.0, 0.0], [250.0, h]), ("e3", [250.0, h], [0.0, 0.0])];
    let prof = r#"{"id":"chs-33.7x2.6","kind":"roundTube","d":33.7,"t":2.6}"#;
    let o = run(&format!(r#"{},{{"type":"frame","id":"F2","sketch":"S1","profile":{prof}}}"#, skeleton(&tri)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let a = std::f64::consts::PI / 4.0 * (33.7f64.powi(2) - (33.7f64 - 5.2).powi(2));
    for m in &o.members {
        assert!(near(m.2[0], 60.0, 1e-6) && near(m.2[1], 60.0, 1e-6), "{:?}", m.2);
        // long point: the tube radius / tan(30 deg) beyond each node
        assert!(near(m.1, 500.0 + 2.0 * 16.85 * 3f64.sqrt(), 1e-6), "stock {}", m.1);
        assert!(near(m.4, a * 500.0, 1e-5 * a * 500.0), "volume {} vs {}", m.4, a * 500.0);
    }
    check_body(&o, 3);
}

#[test]
fn tslot_cube_from_world_lines() {
    // a 300 cube of 20 mm T-slot from world lines: three members at each corner, the first runs through and the others butt
    let c = |x: u8, y: u8, z: u8| [x as f64 * 300.0, y as f64 * 300.0, z as f64 * 300.0];
    let edges: [([f64; 3], [f64; 3]); 12] = [
        (c(0, 0, 0), c(1, 0, 0)), (c(1, 0, 0), c(1, 1, 0)), (c(1, 1, 0), c(0, 1, 0)), (c(0, 1, 0), c(0, 0, 0)),
        (c(0, 0, 1), c(1, 0, 1)), (c(1, 0, 1), c(1, 1, 1)), (c(1, 1, 1), c(0, 1, 1)), (c(0, 1, 1), c(0, 0, 1)),
        (c(0, 0, 0), c(0, 0, 1)), (c(1, 0, 0), c(1, 0, 1)), (c(1, 1, 0), c(1, 1, 1)), (c(0, 1, 0), c(0, 1, 1)),
    ];
    let lines: Vec<String> = edges.iter().map(|(a, b)| format!("[[{},{},{}],[{},{},{}]]", a[0], a[1], a[2], b[0], b[1], b[2])).collect();
    let prof = r#"{"id":"tslot-2020","kind":"tslot","s":20,"slot":6.2,"lip":1.8,"inner":11,"depth":6.1,"bore":4.2}"#;
    let o = run(&format!(r#"{{"type":"frame","id":"F1","lines":[{}],"profile":{prof},"corner":"butt"}}"#, lines.join(",")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.members.len(), 12);
    let b = o.body.as_ref().unwrap();
    assert!(ck_check::validate(b, 1e-6).is_empty());
    assert_eq!(b.num_lumps(), 12);
    // every stock length is 300 plus or minus the half section at each end (through +10, butt -10)
    for m in &o.members {
        let d: f64 = m.3.iter().map(|e| if e == "through" { 10.0 } else { -10.0 }).sum();
        assert!(near(m.1, 300.0 + d, 1e-6), "{} {} {:?}", m.0, m.1, m.3);
    }
}

#[test]
fn per_member_profile_and_joint_override() {
    // e2 is a flat bar; the e1/e2 corner is butted with e2 running through, the others stay mitred
    let o = run(&format!(
        "{},{}",
        skeleton(&RECT),
        frame(r#","members":[{"segment":"e2","profile":{"id":"fb-40x40","kind":"rect","w":40,"h":40}}],"joints":[{"segments":["e1","e2"],"corner":"butt","through":"e2"}]"#)
    ));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let e1 = o.members.iter().find(|m| m.0 == "e1").unwrap();
    assert_eq!([e1.3[0].as_str(), e1.3[1].as_str()], ["mitre", "butt"]);
    assert!(near(e1.1, 600.0 + 20.0 - 20.0, 1e-6), "{}", e1.1);
    let e2 = o.members.iter().find(|m| m.0 == "e2").unwrap();
    assert_eq!([e2.3[0].as_str(), e2.3[1].as_str()], ["through", "mitre"]);
    assert!(near(e2.4, 1600.0 * (400.0 + 20.0) , 1e-3), "solid bar volume {}", e2.4);
    check_body(&o, 4);
}

#[test]
fn bad_payloads_are_frame_invalid() {
    let o = run(r#"{"type":"frame","id":"F1"}"#);
    assert!(o.errors.iter().any(|e| e.starts_with("F1:frameInvalid")), "{:?}", o.errors);
    let o = run(&format!(r#"{},{{"type":"frame","id":"F2","sketch":"S1","profile":{{"kind":"zed","w":3}}}}"#, skeleton(&RECT)));
    assert!(o.errors.iter().any(|e| e.starts_with("F2:frameInvalid") && e.contains("zed")), "{:?}", o.errors);
    let o = run(&format!(r#"{},{{"type":"frame","id":"F2","sketch":"S1","profile":{{"kind":"rectTube","w":40,"h":40,"t":25}}}}"#, skeleton(&RECT)));
    assert!(o.errors.iter().any(|e| e.starts_with("F2:frameInvalid")), "{:?}", o.errors);
    let _ = Json::Null;
}

#[test]
fn editing_the_skeleton_moves_the_members() {
    // the frame follows its sketch: a longer bottom/top changes the stock lengths, nothing else
    let wide = [("e1", [0.0, 0.0], [800.0, 0.0]), ("e2", [800.0, 0.0], [800.0, 400.0]), ("e3", [800.0, 400.0], [0.0, 400.0]), ("e4", [0.0, 400.0], [0.0, 0.0])];
    let o = run(&format!("{},{}", skeleton(&wide), frame("")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let e1 = o.members.iter().find(|m| m.0 == "e1").unwrap();
    assert!(near(e1.1, 840.0, 1e-6));
}

/// Area of an I section with flanges `w` x `tf`, web `tw`, depth `h` and four root fillets of radius `r`.
fn a_ibeam(w: f64, h: f64, tf: f64, tw: f64, r: f64) -> f64 {
    2.0 * w * tf + (h - 2.0 * tf) * tw + 4.0 * (1.0 - std::f64::consts::FRAC_PI_4) * r * r
}

const UB310: &str = r#"{"id":"310ub40","label":"310UB40.4","kind":"ibeam","w":165,"h":304,"tf":10.2,"tw":6.1,"r":11.4}"#;

fn ibeam_frame(profile: &str, extra: &str) -> String {
    format!(r#"{{"type":"frame","id":"F2","sketch":"S1","profile":{profile}{extra}}}"#)
}

#[test]
fn ibeam_rectangle_mitred() {
    let a = a_ibeam(165.0, 304.0, 10.2, 6.1, 11.4);
    let o = run(&format!("{},{}", skeleton(&RECT), ibeam_frame(UB310, "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert_eq!(o.members.len(), 4);
    for (seg, len, ang, ends, vol) in &o.members {
        let cl = if seg == "e1" || seg == "e3" { 600.0 } else { 400.0 };
        // the flanges (165 wide) lie in the plane of the frame: the long point is half a flange beyond each node
        assert!(near(*len, cl + 165.0, 1e-6), "{seg} stock {len}");
        assert!(near(ang[0], 45.0, 1e-6) && near(ang[1], 45.0, 1e-6), "{seg} {ang:?}");
        assert_eq!(ends, &["mitre".to_string(), "mitre".to_string()]);
        assert!(near(*vol, a * cl, 1e-5 * a * cl), "{seg} volume {vol} vs {}", a * cl);
    }
    check_body(&o, 4);
}

#[test]
fn ibeam_rectangle_butted_and_tee() {
    let a = a_ibeam(165.0, 304.0, 10.2, 6.1, 11.4);
    let mut lines = RECT.to_vec();
    lines.push(("e5", [300.0, 0.0], [300.0, 400.0]));
    let o = run(&format!("{},{}", skeleton(&lines), ibeam_frame(UB310, r#","corner":"butt""#)));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    let m5 = o.members.iter().find(|m| m.0 == "e5").unwrap();
    assert_eq!([m5.3[0].as_str(), m5.3[1].as_str()], ["tee", "tee"]);
    for (seg, len, _, _, vol) in &o.members {
        assert!(near(*vol, a * len, 1e-4 * a * len), "{seg} volume {vol} vs {}", a * len);
    }
    check_body(&o, 5);
}

#[test]
fn channel_with_own_flange_and_web_thickness_and_root_radius() {
    // a parallel flange channel 230 deep, 75 wide, flange 12, web 6.5, root radius 12: two root fillets
    let prof = r#"{"id":"230pfc","kind":"channel","w":75,"h":230,"tf":12,"tw":6.5,"r":12}"#;
    let a = 2.0 * 75.0 * 12.0 + (230.0 - 24.0) * 6.5 + 2.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 144.0;
    // one straight member: square cuts, so its volume is the section area x its length (a mitre of an unsymmetric section is not)
    let o = run(&format!("{},{}", skeleton(&[("e1", [0.0, 0.0], [600.0, 0.0])]), ibeam_frame(prof, "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    assert!(near(o.members[0].4, a * 600.0, 1e-5 * a * 600.0), "volume {} vs {}", o.members[0].4, a * 600.0);
    check_body(&o, 1);
    // mitred in a rectangle it still builds, valid and without overlap
    let o = run(&format!("{},{}", skeleton(&RECT), ibeam_frame(prof, "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
    check_body(&o, 4);
    // the old form (one thickness) still reads
    let o = run(&format!("{},{}", skeleton(&RECT), ibeam_frame(r#"{"id":"c","kind":"channel","w":50,"h":100,"t":5}"#, "")));
    assert!(o.errors.is_empty(), "{:?}", o.errors);
}

#[test]
fn bad_ibeam_is_frame_invalid() {
    for prof in [
        r#"{"kind":"ibeam","w":100,"h":200,"tf":10}"#,                        // no web
        r#"{"kind":"ibeam","w":100,"h":200,"tf":10,"tw":120}"#,               // web wider than the flange
        r#"{"kind":"ibeam","w":100,"h":200,"tf":100,"tw":6}"#,                // flanges meet
        r#"{"kind":"ibeam","w":100,"h":200,"tf":10,"tw":6,"r":200}"#,         // root radius does not fit
    ] {
        let o = run(&format!("{},{}", skeleton(&RECT), ibeam_frame(prof, "")));
        assert!(o.errors.iter().any(|e| e.starts_with("F2:frameInvalid")), "{prof}: {:?}", o.errors);
    }
}
