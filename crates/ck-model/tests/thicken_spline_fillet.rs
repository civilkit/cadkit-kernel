//! Fillets on a thickened closed-spline wall (surface extrude of a closed spline, thicken).
//!
//! The thickened wall is two faces per side (the closed spline's wall and each cap are split in two at the seam vertices),
//! so the rim of one side is a ring of two edges whose faces change at the seams. The chain walk and the tangent propagation
//! follow the rim across the seams, but every fillet on this wall is refused at once with the reason: the booleans on this body
//! fail at its rims (a plain cut through the wall is refused too, see the ignored test), and a ring that does close comes out
//! of the boolean with the untouched half of the inner wall flipped. Refusal tests assert the reason and that it is fast.
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts};
use ck_model::Part;

struct Exec(Brep);
impl FeatureExec for Exec {
    fn face_count(&self) -> Option<usize> { self.0.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.0.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> { self.0.exec(s, f) }
}

fn run(src: &str) -> (Vec<String>, Option<ck_topo::body::Body>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec(Brep::new());
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    (r.errors.iter().map(|e| format!("{}: {}", e.feature, e.error.describe())).collect(), ex.0.body.take())
}

fn shape(name: &str) -> String {
    let pts: &[(f64, f64)] = match name {
        // the crash-corpus spline (tight bend ~3.6)
        "A" => &[(0., 0.), (-20., 20.), (-38.36019069184176, 0.), (-49.385516678534486, 29.163921571572125), (22.893597317859196, 36.38761478368862), (30.401642912057962, 7.866202711800999)],
        // the user's part, exactly as the file lists it: the closed spline's fit points are origin, p1..p5
        "U" => &[(0., 0.), (-25., 18.5), (-52., 0.), (-67.5, 19.), (-35., 35.), (9.5, 31.)],
        // the same without the origin point (p1..p5 only)
        "V" => &[(-25., 18.5), (-52., 0.), (-67.5, 19.), (-35., 35.), (9.5, 31.)],
        "C" => &[(-52.642, 7.545), (-44.476, 4.602), (-36.193, 2.010), (-27.678, 0.360), (-19.013, 0.147), (-10.572, 2.022), (-3.110, 6.406), (3.535, 11.989), (10.204, 17.543), (17.603, 22.047), (25.967, 24.186), (34.514, 23.015), (42.211, 19.062), (48.921, 13.567), (55.028, 7.400), (60.815, 0.943), (60.489, -7.167), (52.676, -9.367), (46.238, -3.696), (40.201, 2.540), (33.502, 8.039), (25.304, 9.972), (17.802, 5.726), (11.201, 0.090), (4.479, -5.399), (-2.928, -9.902), (-11.102, -12.774), (-19.694, -13.919), (-28.363, -13.648), (-36.921, -12.226), (-45.282, -9.900), (-53.478, -7.042), (-60.827, -2.819), (-60.434, 5.262)],
        "D" => &[(-53.177, 9.652), (-44.883, 6.522), (-36.721, 3.055), (-28.365, 0.103), (-19.636, -1.331), (-10.951, 0.103), (-3.525, 4.878), (2.902, 10.984), (9.413, 16.999), (16.949, 21.606), (25.644, 22.782), (33.887, 19.712), (40.707, 14.074), (46.585, 7.439), (52.146, 0.532), (57.959, -6.161), (60.502, -13.971), (53.205, -17.343), (46.627, -11.562), (40.978, -4.728), (35.369, 2.140), (29.116, 8.409), (20.987, 10.290), (13.935, 4.980), (7.596, -1.222), (0.837, -6.950), (-6.916, -11.205), (-15.527, -13.197), (-24.373, -12.962), (-33.020, -11.040), (-41.353, -8.016), (-49.515, -4.548), (-57.768, -1.323), (-60.499, 6.240)],
        // a long kidney / S with one tight concave bend
        _ => &[(-60., 0.), (-35., 12.), (-10., 4.), (8., -6.), (14., -22.), (30., -34.), (55., -30.), (62., -8.), (40., -14.), (28., -4.), (6., 16.), (-20., 28.), (-48., 22.)],
    };
    let mut points = String::new();
    let mut ids = vec![];
    for (i, (x, y)) in pts.iter().enumerate() {
        points += &format!(r#"{{"id":"p{i}","x":{x},"y":{y}}},"#);
        ids.push(format!(r#""p{i}""#));
    }
    points.pop();
    format!(
        r#"{{"id":"F1","type":"sketch","plane":{{"kind":"std","id":"top"}},"points":[{points}],"entities":[{{"id":"e1","type":"spline","pts":[{}],"closed":true}}],"constraints":[]}},{{"type":"surfaceExtrude","sketch":"F1","end":"blind","depth":{depth},"id":"F2"}}"#,
        ids.join(","), depth = if name == "U" || name == "V" { 10 } else { 11 }
    )
}

fn base(shape_name: &str, t: f64, side: &str) -> String {
    if side == "solid" {
        return format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, shape(shape_name).replace("surfaceExtrude", "extrude").replace(r#""end":"blind""#, r#""op":"add","end":"blind""#));
    }
    format!(r#"{{"format":"ck-part","version":1,"features":[{},{{"type":"thicken","surface":"F2","thickness":{t},"side":"{side}","id":"F3"}}]}}"#, shape(shape_name))
}

const WALL_VOLUME: f64 = 13511.14;

fn wall() -> ck_topo::body::Body {
    let (e, b) = run(&base("C", 4.0, "back"));
    assert!(e.is_empty(), "{e:?}");
    b.unwrap()
}

fn is_outer(b: &ck_topo::body::Body, ed: ck_topo::body::EdgeId) -> bool {
    b.edge_faces(ed).iter().any(|&f| b.face_id_str(f).starts_with("F2.S.e1~1"))
}

/// The rim edges (between a wall face and a cap) of one side (`top`: y = 11) and one wall (`outer` / inner), as fillet picks.
fn rim_picks(b: &ck_topo::body::Body, top: bool, outer: bool, first_only: bool) -> Vec<String> {
    let mut out = vec![];
    for ed in b.edge_ids() {
        let fs = b.edge_faces(ed);
        let m = b.edge_mid(ed);
        let wall_and_cap = fs.len() == 2 && b.face_id_str(fs[0]).starts_with("F2.S") != b.face_id_str(fs[1]).starts_with("F2.S");
        if wall_and_cap && (m.y > 5.5) == top && is_outer(b, ed) == outer && !(first_only && !out.is_empty()) {
            out.push(format!(r#"{{"faces":["{}","{}"],"at":[{},{},{}]}}"#, b.face_id_str(fs[0]), b.face_id_str(fs[1]), m.x, m.y, m.z));
        }
    }
    out
}

fn fillet_of(picks: &[String], r: f64, propagate: bool) -> (Vec<String>, f64) {
    let prop = if propagate { r#","propagate":true"# } else { "" };
    let src = base("C", 4.0, "back").replace("}]}", &format!(r#"}},{{"type":"fillet","id":"F4","radius":{r}{prop},"edges":[{}]}}]}}"#, picks.join(",")));
    let t0 = std::time::Instant::now();
    let (errs, b) = run(&src);
    assert!(b.is_some_and(|bd| (ck_topo::mass::mass_props(&bd).volume - WALL_VOLUME).abs() < 1.0), "the body is left as it was");
    (errs, t0.elapsed().as_secs_f64())
}

fn assert_refused_for_the_wall(what: &str, picks: &[String], r: f64, propagate: bool) {
    assert!(!picks.is_empty(), "{what}: no picks");
    let (errs, secs) = fillet_of(picks, r, propagate);
    assert!(errs.iter().any(|e| e.starts_with("F4: blendFailed") && e.contains("closed-spline wall") && e.contains("hint")), "{what}: {errs:?}");
    assert!(secs < 20.0, "{what}: refused after {secs:.1} s");
}

/// The wall of a thickened closed spline is split in two faces per side, and the halves are one smooth skin.
#[test]
fn thickened_spline_wall_is_split_into_smooth_halves() {
    let b = wall();
    assert_eq!(b.face_ids().count(), 8);
    let rim: Vec<_> = b.edge_ids().filter(|&ed| {
        let fs = b.edge_faces(ed);
        fs.len() == 2 && b.face_id_str(fs[0]).starts_with("F2.S.e1~1") != b.face_id_str(fs[1]).starts_with("F2.S.e1~1") && b.edge_mid(ed).y > 5.0
    }).collect();
    assert_eq!(rim.len(), 2);
    let (f0, f1) = (b.edge_faces(rim[0]), b.edge_faces(rim[1]));
    assert!(f0.iter().all(|f| !f1.contains(f)), "the halves share no face");
    assert!(ck_ops::blend::freeform::continues_smoothly(&b, [f0[0], f0[1]], [f1[0], f1[1]]).is_some());
}

/// One half of a rim, the full outer or inner ring of either side, every rim edge, and one pick with the tangent chain on:
/// all refused at once with the reason, at both a small and a large radius.
#[test]
fn every_fillet_on_the_wall_is_refused_with_the_reason() {
    let b = wall();
    for r in [0.8, 1.96] {
        assert_refused_for_the_wall("half a rim", &rim_picks(&b, true, true, true), r, false);
        assert_refused_for_the_wall("outer ring, top", &rim_picks(&b, true, true, false), r, false);
        assert_refused_for_the_wall("inner ring, top", &rim_picks(&b, true, false, false), r, false);
        assert_refused_for_the_wall("outer ring, bottom", &rim_picks(&b, false, true, false), r, false);
        assert_refused_for_the_wall("inner ring, bottom", &rim_picks(&b, false, false, false), r, false);
        let mut all = rim_picks(&b, true, true, false);
        all.extend(rim_picks(&b, true, false, false));
        all.extend(rim_picks(&b, false, true, false));
        all.extend(rim_picks(&b, false, false, false));
        assert_refused_for_the_wall("every rim edge", &all, r, false);
        assert_refused_for_the_wall("one pick, tangent chain on", &rim_picks(&b, true, true, true), r, true);
    }
}

/// A plain cut through the thickened wall (a box through it). The caps on the y = 0 side used to be built with their loops
/// clockwise about the outward normal, so the boolean could not split them (openEdge at the cap rim).
#[test]
fn plain_cut_through_the_wall_works() {
    let v0 = ck_topo::mass::mass_props(&wall()).volume;
    for (name, x0, x1, flip) in [("middle", -10.0, 10.0, false), ("middle flipped", -10.0, 10.0, true), ("quarter", 20.0, 30.0, false), ("west quarter", -40.0, -30.0, true)] {
        let src = base("C", 4.0, "back").replace("}]}", &format!(
            r#"}},{{"id":"G1","type":"sketch","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":{x0},"y":-5}},{{"id":"q2","x":{x1},"y":-5}},{{"id":"q3","x":{x1},"y":20}},{{"id":"q4","x":{x0},"y":20}}],"entities":[{{"id":"l1","type":"line","p1":"q1","p2":"q2"}},{{"id":"l2","type":"line","p1":"q2","p2":"q3"}},{{"id":"l3","type":"line","p1":"q3","p2":"q4"}},{{"id":"l4","type":"line","p1":"q4","p2":"q1"}}],"constraints":[]}},{{"type":"extrude","id":"G2","op":"cut","sketch":"G1","end":"blind","depth":30,"flip":{flip}}}]}}"#));
        let (errs, b) = run(&src);
        let b = b.unwrap();
        let v = ck_topo::mass::mass_props(&b).volume;
        // (a flipped direction may miss the wall: `cutNoEffect` is not the defect)
        if errs.iter().any(|e| e.contains("cutNoEffect")) {
            continue;
        }
        assert!(errs.is_empty(), "{name}: {errs:?}");
        assert!(v < v0 && ck_check::validate(&b, 1e-6).is_empty(), "{name}: volume {v} against {v0}");
    }
}

/// Diagnostic (a no-op unless DIAG is set): the thickened wall's own consistency, and one plain cut with the boolean trace.
#[test]
fn diag() {
    if std::env::var_os("DIAG").is_none() {
        return;
    }
    let b = wall();
    eprintln!("DIAG check {:?}", ck_topo::check::check(&b, 1e-6));
    eprintln!("DIAG validate {:?}", ck_check::validate(&b, 1e-6).iter().map(|i| format!("{}:{}", i.code, i.detail)).collect::<Vec<_>>());
    for ed in b.edge_ids() {
        let e = b.edge(ed);
        eprintln!(
            "DIAG edge e{} {} v{}->v{} coedges={} faces={:?} dev={:.2e} t=[{:.3},{:.3}] a=({:.3},{:.3},{:.3}) b=({:.3},{:.3},{:.3})",
            ed.idx(), e.curve.kind(), e.v0.idx(), e.v1.idx(), b.edge_coedges(ed).len(),
            b.edge_faces(ed).iter().map(|&f| b.face_id_str(f)).collect::<Vec<_>>(), ck_check::face_deviation(&b, ed, 16), e.t0, e.t1,
            b.vertex(e.v0).p.x, b.vertex(e.v0).p.y, b.vertex(e.v0).p.z, b.vertex(e.v1).p.x, b.vertex(e.v1).p.y, b.vertex(e.v1).p.z
        );
    }
    for f in b.face_ids() {
        let loops = ck_ops::boolean::classify::face_loops_uv(&b, f);
        let fc = b.face(f);
        for l in &loops {
            let (mut u0, mut u1, mut v0, mut v1) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
            for q in l { u0 = u0.min(q.x); u1 = u1.max(q.x); v0 = v0.min(q.y); v1 = v1.max(q.y); }
            eprintln!("DIAG face {} sense={} area={:.4} n={} u[{:.3},{:.3}] v[{:.3},{:.3}] first={:?} {:?}", b.face_id_str(f), fc.sense, ck_ops::boolean::classify::poly_area(l), l.len(), u0, u1, v0, v1, l[0], l[l.len() / 2]);
        }
    }
    std::env::set_var("CK_BOOL_DBG", "1");
    let (x0, x1) = (20.0, 30.0);
    let src = base("C", 4.0, "back").replace("}]}", &format!(
        r#"}},{{"id":"G1","type":"sketch","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":{x0},"y":-5}},{{"id":"q2","x":{x1},"y":-5}},{{"id":"q3","x":{x1},"y":20}},{{"id":"q4","x":{x0},"y":20}}],"entities":[{{"id":"l1","type":"line","p1":"q1","p2":"q2"}},{{"id":"l2","type":"line","p1":"q2","p2":"q3"}},{{"id":"l3","type":"line","p1":"q3","p2":"q4"}},{{"id":"l4","type":"line","p1":"q4","p2":"q1"}}],"constraints":[]}},{{"type":"extrude","id":"G2","op":"cut","sketch":"G1","end":"blind","depth":30,"flip":false}}]}}"#));
    let (errs, _) = run(&src);
    eprintln!("DIAG cut errs {errs:?}");
}

#[test]
fn diag_fillets() {
    if std::env::var_os("DIAG").is_none() {
        return;
    }
    std::env::remove_var("CK_BOOL_DBG");
    let b = wall();
    let v0 = ck_topo::mass::mass_props(&b).volume;
    for r in [0.8, 1.96] {
        let mut all = rim_picks(&b, true, true, false);
        all.extend(rim_picks(&b, true, false, false));
        all.extend(rim_picks(&b, false, true, false));
        all.extend(rim_picks(&b, false, false, false));
        let cases: Vec<(&str, Vec<String>, bool)> = vec![
            ("half rim", rim_picks(&b, true, true, true), false),
            ("outer top", rim_picks(&b, true, true, false), false),
            ("inner top", rim_picks(&b, true, false, false), false),
            ("outer bottom", rim_picks(&b, false, true, false), false),
            ("inner bottom", rim_picks(&b, false, false, false), false),
            ("all rims", all, false),
            ("one pick propagate", rim_picks(&b, true, true, true), true),
        ];
        for (name, picks, prop) in cases {
            let propj = if prop { r#","propagate":true"# } else { "" };
            let src = base("C", 4.0, "back").replace("}]}", &format!(r#"}},{{"type":"fillet","id":"F4","radius":{r}{propj},"edges":[{}]}}]}}"#, picks.join(",")));
            let t0 = std::time::Instant::now();
            let (errs, body) = run(&src);
            let bd = body.unwrap();
            let v = ck_topo::mass::mass_props(&bd).volume;
            let inv = ck_check::validate(&bd, 1e-6).len();
            eprintln!("DIAGF r={r} {name}: {:.1}s vol {:.3} (was {:.3}) invalid={} errs={}", t0.elapsed().as_secs_f64(), v, v0, inv, errs.iter().map(|e| e.chars().take(200).collect::<String>()).collect::<Vec<_>>().join(" | "));
        }
    }
}

fn with_tool(shape_name: &str, op: &str, tool: &str) -> String {
    base(shape_name, 4.0, "back").replace("}]}", &format!("}},{tool},{{\"type\":\"{op}\",\"id\":\"G2\",\"sketch\":\"G1\",\"end\":\"blind\",\"depth\":30,\"op\":\"{}\"}}]}}", if op == "extrude" { "cut" } else { "add" }))
}

fn rect_sketch(x0: f64, x1: f64, y0: f64, y1: f64) -> String {
    format!(r#"{{"id":"G1","type":"sketch","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"q1","x":{x0},"y":{y0}}},{{"id":"q2","x":{x1},"y":{y0}}},{{"id":"q3","x":{x1},"y":{y1}}},{{"id":"q4","x":{x0},"y":{y1}}}],"entities":[{{"id":"l1","type":"line","p1":"q1","p2":"q2"}},{{"id":"l2","type":"line","p1":"q2","p2":"q3"}},{{"id":"l3","type":"line","p1":"q3","p2":"q4"}},{{"id":"l4","type":"line","p1":"q4","p2":"q1"}}],"constraints":[]}}"#)
}

fn circle_sketch(cx: f64, cy: f64, r: f64) -> String {
    format!(r#"{{"id":"G1","type":"sketch","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"c","x":{cx},"y":{cy}}}],"entities":[{{"id":"k1","type":"circle","c":"c","r":{r}}}],"constraints":[]}}"#)
}

/// Booleans on the thickened wall of the C kidney and the D shape: a box cut through it (both walls at the crossing), a cylinder
/// hole, and a block united across it. Every result is a valid solid and the volume moves the right way.
#[test]
fn booleans_on_the_thickened_wall() {
    for shape_name in ["C", "D"] {
        let (e, b0) = run(&base(shape_name, 4.0, "back"));
        assert!(e.is_empty(), "{e:?}");
        let v0 = ck_topo::mass::mass_props(&b0.unwrap()).volume;
        let cases: Vec<(&str, String, bool)> = vec![
            ("box cut", with_tool(shape_name, "extrude", &rect_sketch(20.0, 30.0, -5.0, 20.0)), true),
            ("cylinder hole", with_tool(shape_name, "extrude", &circle_sketch(25.0, 5.5, 3.0)), true),
            ("block union", with_tool(shape_name, "extrude", &rect_sketch(20.0, 30.0, -5.0, 20.0)).replace(r#""op":"cut""#, r#""op":"add""#), false),
        ];
        for (name, src, cut) in cases {
            let (errs, b) = run(&src);
            let b = b.unwrap();
            let v = ck_topo::mass::mass_props(&b).volume;
            assert!(errs.is_empty(), "{shape_name} {name}: {errs:?}");
            assert!(ck_check::validate(&b, 1e-6).is_empty(), "{shape_name} {name}: invalid");
            assert!(if cut { v < v0 - 1.0 } else { v > v0 + 1.0 }, "{shape_name} {name}: volume {v} against {v0}");
            eprintln!("BOOLCASE {shape_name} {name}: {v:.3} (wall {v0:.3})");
        }
    }
}

