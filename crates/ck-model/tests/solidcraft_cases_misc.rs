//! Cases from tangency-labs/solidcraft (Apache-2.0 for solidcore/, see THIRD_PARTY.md) that our kernel got wrong, rebuilt for our feature tree:
//! a blind hole that ends where it comes out of a skewed wall (#17873), unreadable hole fields (#9088),
//! a pattern of a feature that leaves no tool (#16945), STEP solids the reader leaves out (#16857) and a
//! distance dimension edited up from zero (#14298). Sizes and expected volumes are closed forms.
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

struct Out {
    ex: Exec,
    errors: Vec<(String, String, Json)>,
}

fn run(features: &str) -> Out {
    let part = Part::parse(&format!(r#"{{"format":"ck-part","version":1,"features":[{features}]}}"#)).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errors = r.errors.iter().map(|e| (e.feature.clone(), e.error.code.clone(), Json::Obj(e.error.params.clone()))).collect();
    Out { ex, errors }
}

impl Out {
    fn codes(&self) -> Vec<String> {
        self.errors.iter().map(|e| format!("{}:{}", e.0, e.1)).collect()
    }
    fn param(&self, key: &str) -> String {
        let e = self.errors.first().expect("an error");
        ck_json::to_string(e.2.get(key).unwrap_or(&Json::Null))
    }
    fn volume(&self) -> f64 {
        ck_topo::mass::mass_props(self.ex.0.body.as_ref().expect("a body")).volume
    }
    fn faces(&self) -> Vec<String> {
        let b = self.ex.0.body.as_ref().expect("a body");
        b.face_ids().map(|f| b.face(f).name.id()).collect()
    }
}

fn quad(id: &str, plane: &str, pts: &[[f64; 2]]) -> String {
    let points: Vec<String> = pts.iter().enumerate().map(|(i, p)| format!(r#"{{"id":"p{i}","x":{},"y":{}}}"#, p[0], p[1])).collect();
    let ents: Vec<String> = (0..pts.len()).map(|i| format!(r#"{{"type":"line","id":"e{}","p1":"p{i}","p2":"p{}"}}"#, i + 1, (i + 1) % pts.len())).collect();
    format!(r#"{{"type":"sketch","id":"{id}","plane":{{"kind":"std","id":"{plane}"}},"points":[{}],"entities":[{}],"constraints":[]}}"#, points.join(","), ents.join(","))
}

/// A 3 mm wall, 40 x 30, whose two sides lean 3.1e-6 and 1.31e-5 off parallel to the axis of the hole (as a sketch can leave
/// them); the hole is drilled from the near side at x = 20 to a depth that reaches the far side there. (The sketch frame of
/// the near side keeps its x axis on the world X, so its plane origin sits 5e-5 off that side at x = 20; the depth counts
/// from the plane.)
const EXIT_DEPTH: f64 = 3.000262;
fn skewed_wall(hole: &str) -> Out {
    let (near, far) = (3.1e-6, 1.31e-5);
    let wall = quad("S1", "front", &[[0.0, 5.5], [40.0, 5.5 + 40.0 * near], [40.0, 8.5 + 40.0 * far], [0.0, 8.5]]);
    let depth = EXIT_DEPTH;
    run(&format!(
        r#"{wall},{{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":30}},
        {{"type":"sketch","id":"HS","plane":{{"kind":"face","id":"E1.S.e1"}},"points":[{{"id":"p1","x":20,"y":15}}],"entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},
        {{"type":"hole","id":"H","sketch":"HS","diameter":8,"end":"blind","depth":{depth}{hole}}}"#
    ))
}

#[test]
fn a_blind_hole_to_the_far_side_of_a_skewed_wall_comes_out_whole() {
    let (near, far) = (3.1e-6, 1.31e-5);
    let want = 30.0 * 40.0 * (3.0 + 20.0 * (far - near)) - std::f64::consts::PI * 16.0 * 3.0002;
    for (name, hole) in [("plain", ""), ("drill point", r#","tip":true"#), ("countersink", r#","kind":"countersink","cskDiameter":10,"cskAngle":90"#)] {
        let o = skewed_wall(hole);
        assert!(o.codes().is_empty(), "{name}: {:?}", o.codes());
        // no floor is left across the exit: every face of the hole is its wall, or the countersink cone
        let floors: Vec<String> = o.faces().into_iter().filter(|n| n.starts_with("H.") && !n.contains("h1") && !n.contains("K0")).collect();
        let skin: Vec<_> = floors.iter().filter(|n| n.ends_with("h2")).collect();
        if hole.is_empty() || hole.contains("tip") {
            assert!(skin.is_empty(), "{name}: a film stays over the exit: {floors:?}");
            assert!((o.volume() - want).abs() < 1e-2, "{name}: {} want {want}", o.volume());
        } else {
            // the countersink adds its cone volume above the through hole's
            assert!(skin.is_empty(), "{name}: a film stays over the exit: {floors:?}");
            assert!(o.volume() < want - 10.0, "{name}: {}", o.volume());
        }
    }
}

#[test]
fn a_blind_hole_short_of_the_far_side_keeps_its_floor() {
    // 0.5 mm of wall left under the floor: an ordinary blind hole, not a through hole
    let (near, far) = (3.1e-6, 1.31e-5);
    let depth = 2.5 + 20.0 * (far - near);
    let wall = quad("S1", "front", &[[0.0, 5.5], [40.0, 5.5 + 40.0 * near], [40.0, 8.5 + 40.0 * far], [0.0, 8.5]]);
    let o = run(&format!(
        r#"{wall},{{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":30}},
        {{"type":"sketch","id":"HS","plane":{{"kind":"face","id":"E1.S.e1"}},"points":[{{"id":"p1","x":20,"y":15}}],"entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},
        {{"type":"hole","id":"H","sketch":"HS","diameter":8,"end":"blind","depth":{depth}}}"#
    ));
    assert!(o.codes().is_empty(), "{:?}", o.codes());
    let want = 30.0 * 40.0 * (3.0 + 20.0 * (far - near)) - std::f64::consts::PI * 16.0 * depth;
    assert!((o.volume() - want).abs() < 1e-2, "{} want {want}", o.volume());
    assert!(o.faces().iter().any(|n| n.ends_with("h2")), "the floor is there: {:?}", o.faces());
}

fn plate_hole(hole: &str) -> Out {
    let plate = quad("S1", "front", &[[0.0, 0.0], [40.0, 0.0], [40.0, 30.0], [0.0, 30.0]]);
    run(&format!(
        r#"{plate},{{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":10}},
        {{"type":"sketch","id":"HS","plane":{{"kind":"face","id":"E1.C1"}},"points":[{{"id":"p1","x":10,"y":10}}],"entities":[{{"type":"point","id":"e1","p":"p1"}}],"constraints":[]}},
        {{"type":"hole","id":"H","sketch":"HS"{hole}}}"#
    ))
}

#[test]
fn a_hole_with_unreadable_fields_is_refused_by_name_and_drills_nothing() {
    assert!(plate_hole(r#","diameter":5,"end":"blind","depth":5"#).codes().is_empty());
    for (what, hole, field) in [
        ("flip as text", r#","diameter":5,"end":"blind","depth":5,"flip":"sideways""#, "flip"),
        ("tip as text", r#","diameter":5,"end":"blind","depth":5,"tip":"x""#, "tip"),
        ("kind unknown", r#","diameter":5,"kind":"slot","end":"through""#, "kind"),
        ("kind a number", r#","diameter":5,"kind":42,"end":"through""#, "kind"),
        ("end unknown", r#","diameter":5,"end":"sideways""#, "end"),
        ("depth as text", r#","diameter":5,"end":"blind","depth":"deep""#, "depth"),
        ("depth a list", r#","diameter":5,"end":"blind","depth":[1,2]"#, "depth"),
        ("depth zero", r#","diameter":5,"end":"blind","depth":0"#, "depth"),
        ("depth negative", r#","diameter":5,"end":"blind","depth":-4"#, "depth"),
        ("blind without depth", r#","diameter":5"#, "depth"),
    ] {
        let o = plate_hole(hole);
        assert_eq!(o.codes(), ["H:holeInvalid"], "{what}");
        assert_eq!(o.param("field"), format!("\"{field}\""), "{what}");
        assert!(!o.param("detail").is_empty() && o.param("hint") != "null", "{what}");
        assert!((o.volume() - 12_000.0).abs() < 1e-9, "{what}: the plate is drilled");
    }
}

#[test]
fn a_pattern_of_a_fillet_names_the_cause() {
    let plate = quad("S1", "front", &[[0.0, 0.0], [60.0, 0.0], [60.0, 40.0], [0.0, 40.0]]);
    let o = run(&format!(
        r#"{plate},{{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":10}},
        {{"type":"fillet","id":"F2","radius":1,"edges":[{{"faces":["E1.C1","E1.S.e1"],"at":[30,0,10]}}]}},
        {{"type":"lpattern","id":"P1","seeds":["F2"],"dir":{{"kind":"axis","axis":"x"}},"count":2,"spacing":5}}"#
    ));
    assert_eq!(o.codes(), ["P1:missingReference"]);
    assert_eq!(o.param("seed"), "\"F2\"");
    assert!(o.param("detail").contains("fillet") && o.param("detail").contains("no tool"), "{}", o.param("detail"));
    assert!(o.param("hint").contains("copies"), "{}", o.param("hint"));
    // an id that is no feature says so too
    let o = run(&format!(
        r#"{plate},{{"type":"extrude","id":"E1","op":"add","sketch":"S1","end":"blind","depth":10}},
        {{"type":"lpattern","id":"P1","seeds":["F99"],"dir":{{"kind":"axis","axis":"x"}},"count":2,"spacing":5}}"#
    ));
    assert_eq!(o.codes(), ["P1:missingReference"]);
    assert!(o.param("detail").contains("names no feature"), "{}", o.param("detail"));
}

/// Three boxes as one STEP file; with the middle solid's shell unreadable the file reads two solids. The last one
/// keeps the number of its place in the file, so a later feature that names it still finds it (#16857).
#[test]
fn a_solid_left_out_of_a_step_file_keeps_the_names_of_the_others() {
    let boxes = ["-5", "15", "35"]
        .iter()
        .enumerate()
        .map(|(i, x)| {
            let x: f64 = x.parse().unwrap();
            let h = 10.0 * (i as f64 + 1.0);
            format!(
                r#"{},{{"type":"extrude","id":"E{i}","op":"{}","sketch":"S{i}","end":"blind","depth":{h}{}}}"#,
                quad(&format!("S{i}"), "front", &[[x, -5.0], [x + 10.0, -5.0], [x + 10.0, 5.0], [x, 5.0]]),
                if i == 0 { "add" } else { "newBody" },
                if i == 0 { String::new() } else { format!(r#","body":"B{i}""#) }
            )
        })
        .collect::<Vec<_>>()
        .join(",");
    let src = run(&boxes);
    assert!(src.codes().is_empty(), "{:?}", src.codes());
    let step = ck_io::step::write_step(src.ex.0.body.as_ref().unwrap(), "three", &ck_io::step::StepOpts { unix_ms: 0.0 });
    let solids: Vec<(String, String)> = step
        .match_indices("MANIFOLD_SOLID_BREP('")
        .map(|(at, _)| {
            let rest = &step[at + "MANIFOLD_SOLID_BREP('".len()..];
            let name = rest[..rest.find('\'').unwrap()].to_string();
            let shell = rest[rest.find("',#").unwrap() + 3..].split(')').next().unwrap().to_string();
            (name, shell)
        })
        .collect();
    assert_eq!(solids.len(), 3, "{solids:?}");
    let broken = step.replacen(&format!("',#{})", solids[1].1), "',#999999)", 1);
    assert_ne!(broken, step);
    let named = |text: &str| -> Vec<(String, f64)> {
        let o = run(&format!(
            r#"{{"type":"importStep","id":"I","step":{},"body":"Part"}}"#,
            ck_json::to_string(&Json::Str(text.to_string()))
        ));
        let b = o.ex.0.body.as_ref().unwrap();
        let mut v: Vec<(String, f64)> = b
            .lump_ids()
            .map(|l| {
                let faces = ck_topo::mass::lump_faces(b, l);
                let top = b.bbox_of_faces(&faces).1.z;
                (b.lump(l).name.clone().unwrap_or_default(), top)
            })
            .collect();
        v.sort_by(|a, b| a.0.cmp(&b.0));
        v
    };
    let whole = named(&step);
    assert_eq!(whole.iter().map(|w| w.0.as_str()).collect::<Vec<_>>(), ["Part", "Part 2", "Part 3"]);
    let left = named(&broken);
    // the third solid (30 tall) is still "Part 3"; "Part 2" is the one that was left out
    assert_eq!(left.iter().map(|w| w.0.as_str()).collect::<Vec<_>>(), ["Part", "Part 3"], "{left:?}");
    let top = |v: &[(String, f64)], n: &str| v.iter().find(|w| w.0 == n).unwrap().1;
    assert!((top(&left, "Part 3") - top(&whole, "Part 3")).abs() < 1e-9 && (top(&left, "Part") - top(&whole, "Part")).abs() < 1e-9, "{left:?} vs {whole:?}");
}

/// `|d| = value` has no slope at `d = 0`: a distance dimension edited up from zero never solved.
#[test]
fn a_distance_edited_up_from_zero_solves_on_the_positive_side() {
    use ck_model::sketch::solver::solve_sketch;
    use ck_model::sketch::solver::SolveOpts;
    let make = |value: f64, b: f64| {
        let src = format!(
            r#"{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[
              {{"id":"a","x":0,"y":0,"fixed":true}},{{"id":"b","x":{b},"y":0}},{{"id":"c","x":{b},"y":5}},{{"id":"d","x":0,"y":5}}],
             "entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},
              {{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}}],
             "constraints":[{{"type":"horizontal","id":"k1","refs":["e1"]}},{{"type":"vertical","id":"k2","refs":["e2"]}},
              {{"type":"horizontal","id":"k3","refs":["e3"]}},{{"type":"vertical","id":"k4","refs":["e4"]}},
              {{"type":"hdist","id":"k5","refs":["a","b"],"value":{value},"offset":null}}]}}"#
        );
        ck_model::Sketch::from_json(&ck_json::parse(&src).unwrap())
    };
    let solved = |value: f64, b: f64| {
        let mut s = make(value, b);
        solve_sketch(&mut s, &SolveOpts::default()).unwrap();
        (s.point_xy("b").unwrap(), s.point_xy("c").unwrap())
    };
    // from the width it had
    let (b, c) = solved(25.0, 10.0);
    assert!((b[0] - 25.0).abs() < 1e-6 && (c[0] - 25.0).abs() < 1e-6, "{b:?} {c:?}");
    // from zero width
    let (b, c) = solved(25.0, 0.0);
    assert!((b[0] - 25.0).abs() < 1e-6 && (c[0] - 25.0).abs() < 1e-6, "{b:?} {c:?}");
    // a point on the left of its anchor stays on the left
    let (b, _) = solved(25.0, -10.0);
    assert!((b[0] + 25.0).abs() < 1e-6, "{b:?}");
    // a negative value takes the other side
    let (b, _) = solved(-25.0, 10.0);
    assert!((b[0] + 25.0).abs() < 1e-6, "{b:?}");
}
