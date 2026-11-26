//! The timeline mapping on dumps written by hand (no third-party data): sketch + extrude, a cut, a symmetric extrude,
//! a through-all cut, a plate with a hole picked by its outline, a revolve about an origin axis, placeholders, and the
//! guard against a reference body.

use ck_f3d_dump::Dump;
use ck_f3d_features::{Measure, Status, Verdict, guard, timeline_to_features};
use serde_json::{Value, json};

/// A sketch item on a plane (cm): `origin`, `x`, `y` and its geometry.
fn sketch(index: i64, name: &str, origin: [f64; 3], x: [f64; 3], y: [f64; 3], z: [f64; 3], points: Value, curves: Value) -> Value {
    json!({
        "index": index, "name": name, "objectType": "Sketch", "component": "Root",
        "detail": {
            "name": name, "component": "Root", "isVisible": true,
            "model_frame": {"origin": origin, "x_axis": x, "y_axis": y, "z_axis": z},
            "points": points, "curves": curves,
        }
    })
}

fn xy(index: i64, name: &str, points: Value, curves: Value) -> Value {
    sketch(index, name, [0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], points, curves)
}

/// A rectangle (cm): points `p<base>..` and lines `c<base>..`.
fn rect(base: usize, x0: f64, y0: f64, x1: f64, y1: f64) -> (Vec<Value>, Vec<Value>) {
    let at = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]];
    let points = (0..4).map(|i| json!({"id": format!("p{}", base + i), "xyz": [at[i][0], at[i][1], 0.0]})).collect();
    let curves = (0..4)
        .map(|i| {
            json!({"id": format!("c{}", base + i), "type": "SketchLine",
                   "startSketchPoint": format!("p{}", base + i), "endSketchPoint": format!("p{}", base + (i + 1) % 4)})
        })
        .collect();
    (points, curves)
}

fn circle(point: &str, curve: &str, cx: f64, cy: f64, r: f64) -> (Value, Value) {
    (
        json!({"id": point, "xyz": [cx, cy, 0.0]}),
        json!({"id": curve, "type": "SketchCircle", "centerSketchPoint": point, "radius": r}),
    )
}

fn profile(sketch_index: i64, name: &str) -> Value {
    json!({"kind": "profile", "sketch": name, "sketch_timeline_index": sketch_index})
}

fn distance(cm: f64) -> Value {
    json!({"_type": "DistanceExtentDefinition", "distance": {"kind": "parameter", "name": "d", "value": cm}})
}

fn extrude(index: i64, name: &str, sketch_index: i64, sketch_name: &str, operation: &str, extent: Value) -> Value {
    let mut detail = json!({
        "operation": operation, "profile": [profile(sketch_index, sketch_name)],
        "extentType": "OneSideFeatureExtentType", "extentOne": extent, "isSolid": true,
    });
    if let Some(o) = detail.as_object_mut() {
        o.insert("taperAngleOne".into(), json!({"kind": "parameter", "name": "t", "value": 0.0}));
    }
    json!({"index": index, "name": name, "objectType": "ExtrudeFeature", "component": "Root", "detail": detail})
}

fn dump(items: Vec<Value>) -> Dump {
    let doc = json!({"schema": "mitcad-f3d-dump", "schema_version": 2, "timeline": {"available": true, "items": items}});
    Dump::from_json(&doc.to_string()).expect("a dump")
}

fn plate_items() -> Vec<Value> {
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    vec![
        xy(0, "Sketch1", json!(p), json!(c)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
    ]
}

/// Regenerates the features: their body's volume (mm^3) and box.
fn built(features: &Value) -> Measure {
    let part = json!({"format": "ck-part", "version": 1, "features": features});
    let (m, errors) = guard::regenerate(&part).expect("regenerates");
    assert!(errors.is_empty(), "{errors:?}");
    m.expect("a body")
}

fn near(a: f64, b: f64) {
    assert!((a - b).abs() <= 1e-6 * b.abs().max(1.0), "{a} vs {b}");
}

#[test]
fn a_sketch_and_an_extrude_map_and_regenerate_to_the_plate() {
    let import = timeline_to_features(&dump(plate_items()));
    assert_eq!(import.mapped(), 2, "{:?}", import.notes);
    assert_eq!(import.placeholders(), 0);
    let sk = &import.features[0];
    assert_eq!(sk["type"], "sketch");
    assert_eq!(sk["plane"], json!({"kind": "std", "id": "front"}));
    // cm to mm
    assert_eq!(sk["points"][1]["x"], json!(40.0));
    assert_eq!(import.features[1]["type"], "extrude");
    assert_eq!(import.features[1]["op"], "add");
    assert_eq!(import.features[1]["depth"], json!(10.0));
    let m = built(&import.part()["features"]);
    near(m.volume, 40.0 * 20.0 * 10.0);
    assert_eq!(m.max, [40.0, 20.0, 10.0].map(|v: f64| v));
    assert_eq!(import.part()["format"], "ck-part");
}

#[test]
fn the_guard_verifies_a_matching_body_and_reports_the_delta_of_a_wrong_one() {
    let import = timeline_to_features(&dump(plate_items()));
    let truth = Measure { volume: 8000.0, min: [0.0; 3], max: [40.0, 20.0, 10.0], solids: 1, unread: 0 };
    let ok = guard::verify(&import, Some(&truth));
    assert_eq!(ok.verdict, Verdict::Verified, "{:?}", ok.reason);
    let taller = Measure { volume: 9000.0, min: [0.0; 3], max: [40.0, 20.0, 11.25], solids: 1, unread: 0 };
    let bad = guard::verify(&import, Some(&taller));
    assert_eq!(bad.verdict, Verdict::Unverified);
    near(bad.volume_delta.expect("a delta"), -1000.0 / 9000.0);
    near(bad.box_delta.expect("a box delta"), 1.25);
    assert!(bad.reason.as_deref().is_some_and(|r| r.contains("differs")));
    let none = guard::verify(&import, None);
    assert_eq!(none.verdict, Verdict::Unverified);
    // an imported body the reader could not build whole is not the design's: equal numbers say nothing
    let partial = Measure { unread: 1, ..truth };
    let c = guard::verify(&import, Some(&partial));
    assert_eq!(c.verdict, Verdict::Unverified);
    assert!(c.reason.as_deref().is_some_and(|r| r.contains("not the whole design")), "{:?}", c.reason);
}

#[test]
fn a_cut_from_the_top_removes_a_pocket() {
    // a second sketch on the top face (z = 10 mm), a 10 x 10 mm pocket cut 5 mm down: a negative distance
    let mut items = plate_items();
    let (p, c) = rect(0, 1.0, 0.5, 2.0, 1.5);
    items.push(sketch(2, "Sketch2", [0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], json!(p), json!(c)));
    items.push(extrude(3, "Extrude2", 2, "Sketch2", "CutFeatureOperation", distance(-0.5)));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    // the plane of the second sketch is not a standard one: a plane feature comes first
    assert!(import.features.iter().any(|f| f["type"] == "plane"));
    let m = built(&import.part()["features"]);
    near(m.volume, 8000.0 - 10.0 * 10.0 * 5.0);
}

#[test]
fn a_symmetric_extrude_of_a_circle_is_a_mid_plane_cylinder() {
    let (p, c) = circle("p0", "c0", 0.0, 0.0, 1.0);
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["extentType"] = json!("SymmetricFeatureExtentType");
    e["detail"]["extentOne"] = json!({"_type": "SymmetricExtentDefinition", "isFullLength": true,
        "distance": {"kind": "parameter", "name": "d", "value": 2.0}});
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!([p]), json!([c])), e]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert_eq!(import.features[1]["end"], "mid");
    assert_eq!(import.features[1]["depth"], json!(20.0));
    let m = built(&import.part()["features"]);
    near(m.volume, std::f64::consts::PI * 100.0 * 20.0);
    near(m.min[2], -10.0);
    near(m.max[2], 10.0);
}

#[test]
fn a_through_all_cut_goes_through_the_plate() {
    let mut items = plate_items();
    let (p, c) = circle("p0", "c0", 2.0, 1.0, 0.25);
    items.push(xy(2, "Sketch2", json!([p]), json!([c])));
    let mut cut = extrude(3, "Extrude2", 2, "Sketch2", "CutFeatureOperation", distance(1.0));
    cut["detail"]["extentOne"] = json!({"_type": "ThroughAllExtentDefinition", "isPositiveDirection": true});
    items.push(cut);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert_eq!(import.features[3]["end"], "through");
    let m = built(&import.part()["features"]);
    near(m.volume, 8000.0 - std::f64::consts::PI * 2.5 * 2.5 * 10.0);
}

#[test]
fn a_plate_with_a_hole_is_picked_by_its_outline() {
    let (mut p, mut c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let (hp, hc) = circle("p9", "c9", 2.0, 1.0, 0.5);
    p.push(hp);
    c.push(hc);
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["_f3d_profile_loops"] = json!([{
        "loops": [
            {"outer": true, "curves": [{"id": "c0"}, {"id": "c1"}, {"id": "c2"}, {"id": "c3"}]},
            {"outer": false, "curves": [{"id": "c9"}]},
        ],
        "children": [],
    }]);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), e]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    // the plate (with its hole) is the one region of the sketch: the whole sketch, or picked by its outline
    assert!(import.features[1].get("regions").is_none_or(|r| r == &json!(["c0"])), "{:?}", import.features[1]);
    let m = built(&import.part()["features"]);
    near(m.volume, (40.0 * 20.0 - std::f64::consts::PI * 25.0) * 10.0);
}

/// Three squares side by side in a row, 20 mm each (A, B, C; the edges between them are one curve).
fn three_squares() -> (Value, Value) {
    let at = [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0], [0.0, 2.0], [4.0, 0.0], [4.0, 2.0], [6.0, 0.0], [6.0, 2.0]];
    let points: Vec<Value> = at.iter().enumerate().map(|(i, p)| json!({"id": format!("p{i}"), "xyz": [p[0], p[1], 0.0]})).collect();
    let lines = [(0, 1), (1, 2), (2, 3), (3, 0), (1, 4), (4, 5), (5, 2), (4, 6), (6, 7), (7, 5)];
    let curves: Vec<Value> = lines
        .iter()
        .enumerate()
        .map(|(i, (a, b))| json!({"id": format!("c{i}"), "type": "SketchLine", "startSketchPoint": format!("p{a}"), "endSketchPoint": format!("p{b}")}))
        .collect();
    (json!(points), json!(curves))
}

fn loop_of(ids: &[&str]) -> Value {
    json!({"outer": true, "curves": ids.iter().map(|i| json!({"id": i})).collect::<Vec<_>>()})
}

#[test]
fn a_profile_of_two_regions_is_picked_by_its_children() {
    // the file's profile is A and B (split by the curve between them): its merged outline is the outline of both, and
    // each child is one region. Only the merged outline would pick one square.
    let (p, c) = three_squares();
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["_f3d_profile_loops"] = json!([{
        "loops": [loop_of(&["c0", "c4", "c5", "c6", "c2", "c3"])],
        "children": [[loop_of(&["c0", "c1", "c2", "c3"])], [loop_of(&["c4", "c5", "c6", "c1"])]],
    }]);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", p, c), e]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 2.0 * 400.0 * 10.0);
    assert_eq!(m.max[0], 40.0);
}

#[test]
fn a_region_with_no_curve_of_its_own_is_picked_by_a_point_inside_it() {
    // two rectangles that cross: A 4 x 2 cm and B 4 x 4 cm, B's left edge cutting A. The overlap's four edges (A's
    // bottom, right and top, B's left) each also bound a neighbouring region, so no entity id picks it alone.
    let (mut p, mut c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let (p2, c2) = rect(4, 2.0, -1.0, 6.0, 3.0);
    p.extend(p2);
    c.extend(c2);
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["_f3d_profile_loops"] = json!([{"loops": [loop_of(&["c0", "c1", "c2", "c7"])], "children": []}]);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), e]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let ex = &import.features[1];
    assert!(ex.get("regions").is_none(), "{ex:?}");
    let at = ex["regionAt"].as_array().expect("a point");
    assert_eq!(at.len(), 1);
    let m = built(&import.part()["features"]);
    near(m.volume, 20.0 * 20.0 * 10.0);
    // the overlap is the only region that was extruded
    near(m.min[0], 20.0);
    near(m.max[0], 40.0);
}

#[test]
fn as_many_profiles_as_regions_with_one_outline_pick_every_one_of_them() {
    // two crossing circles: the lens and the two crescents are all outlined by c0 and c1. A file that picks three
    // profiles with that outline picks them all (the whole sketch)
    let (p0, c0) = circle("p0", "c0", 0.0, 0.0, 2.0);
    let (p1, c1) = circle("p1", "c1", 2.0, 0.0, 2.0);
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    let one = json!({"loops": [loop_of(&["c0", "c1"])], "children": []});
    e["detail"]["_f3d_profile_loops"] = json!([one.clone(), one.clone(), one]);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!([p0, p1]), json!([c0, c1])), e]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    // the union of the two discs, 20 mm deep... the sketch is in cm
    let r = 20.0_f64;
    let d = 20.0_f64;
    let lens = 2.0 * r * r * (d / (2.0 * r)).acos() - d / 2.0 * (4.0 * r * r - d * d).sqrt();
    near(m.volume, (2.0 * std::f64::consts::PI * r * r - lens) * 10.0);
}

#[test]
fn lines_drawn_along_an_edge_cut_it_as_they_do_in_fusion() {
    // a 40 x 40 mm plate and a notch drawn from the top edge: one of its sides (c4) lies on the edge, so Fusion cuts the edge
    // at its ends and the notch is a region of its own. The file's profile is the notch: its three sides and the piece of the
    // edge (c2) between them.
    let (mut p, mut c) = rect(0, 0.0, 0.0, 4.0, 4.0);
    let at = [[1.0, 4.0], [2.0, 4.0], [2.0, 3.0], [1.0, 3.0]];
    for (i, a) in at.iter().enumerate() {
        p.push(json!({"id": format!("p{}", 10 + i), "xyz": [a[0], a[1], 0.0]}));
        c.push(json!({"id": format!("c{}", 4 + i), "type": "SketchLine",
            "startSketchPoint": format!("p{}", 10 + i), "endSketchPoint": format!("p{}", 10 + (i + 1) % 4)}));
    }
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["_f3d_profile_loops"] = json!([{"loops": [loop_of(&["c5", "c6", "c7", "c2"])], "children": []}]);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), e]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert_eq!(import.features[0]["splitOverlaps"], json!(true));
    near(built(&import.part()["features"]).volume, 10.0 * 10.0 * 10.0);
    // a sketch without such lines does not ask for it
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 4.0);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0))]));
    assert!(import.features[0].get("splitOverlaps").is_none());
}

#[test]
fn regions_outlined_by_the_same_curves_are_not_guessed() {
    // two circles that cross: the lens and the two crescents are all outlined by the two circles' ids
    let (p0, c0) = circle("p0", "c0", 0.0, 0.0, 2.0);
    let (p1, c1) = circle("p1", "c1", 2.0, 0.0, 2.0);
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["_f3d_profile_loops"] = json!([{"loops": [loop_of(&["c0", "c1"])], "children": []}]);
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!([p0, p1]), json!([c0, c1])), e]));
    assert_eq!(import.placeholders(), 1);
    assert!(import.notes.iter().any(|n| n.text.contains("several regions")), "{:?}", import.notes);
}

#[test]
fn a_revolve_about_an_origin_axis_writes_a_helper_axis_sketch() {
    // a 10 x 10 mm square from x = 10 to 20 mm, a full turn about the Y axis: a ring, outer 20, inner 10, 10 high
    let (p, c) = rect(0, 1.0, 0.0, 2.0, 1.0);
    let revolve = json!({
        "index": 1, "name": "Revolve1", "objectType": "RevolveFeature", "component": "Root",
        "detail": {
            "operation": "NewBodyFeatureOperation", "profile": [profile(0, "Sketch1")], "isSolid": true,
            "axis": {"kind": "construction_axis", "name": "Y", "origin": "Y"},
            "extentDefinition": {"_type": "AngleExtentDefinition",
                "angle": {"kind": "parameter", "name": "a", "value": std::f64::consts::TAU}},
        }
    });
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), revolve]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert_eq!(import.features.iter().filter(|f| f["type"] == "sketch").count(), 2);
    let m = built(&import.part()["features"]);
    near(m.volume, std::f64::consts::PI * (400.0 - 100.0) * 10.0);
}

#[test]
fn a_revolve_of_regions_on_both_sides_of_the_axis_revolves_each() {
    // two squares, one from x = 10 to 20 mm and one from x = -40 to -30 mm, a full turn about the Y axis: two rings.
    // (a region that itself crosses the axis is still refused by the kernel: profileCrossesAxis)
    let (mut p, mut c) = rect(0, 1.0, 0.0, 2.0, 1.0);
    let (p2, c2) = rect(4, -4.0, 0.0, -3.0, 1.0);
    p.extend(p2);
    c.extend(c2);
    let revolve = json!({
        "index": 1, "name": "Revolve1", "objectType": "RevolveFeature", "component": "Root",
        "detail": {
            "operation": "NewBodyFeatureOperation", "profile": [profile(0, "Sketch1"), profile(0, "Sketch1")], "isSolid": true,
            "axis": {"kind": "construction_axis", "name": "Y", "origin": "Y"},
            "extentDefinition": {"_type": "AngleExtentDefinition",
                "angle": {"kind": "parameter", "name": "a", "value": std::f64::consts::TAU}},
        }
    });
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), revolve]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, std::f64::consts::PI * ((400.0 - 100.0) + (1600.0 - 900.0)) * 10.0);
}

#[test]
fn an_extrude_of_the_letters_of_a_sketch_text_is_not_mapped() {
    // the letters' outlines are the font's and are not in the file: "every region of the sketch" would be another body
    let mut items = plate_items();
    let mut cut = extrude(2, "Extrude2", 0, "Sketch1", "CutFeatureOperation", distance(-0.2));
    cut["detail"]["_f3d_text_profiles"] = json!({"only": true, "profiles": 9, "texts": ["t0"]});
    items.push(cut);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 1);
    assert!(import.notes.iter().any(|n| n.text.contains("letters of a sketch text")), "{:?}", import.notes);
}

#[test]
fn what_does_not_map_is_a_suppressed_placeholder_naming_the_feature() {
    let mut items = plate_items();
    items.push(json!({"index": 2, "name": "Fillet1", "objectType": "FilletFeature", "component": "Root", "detail": {"edgeSets": []}}));
    items.push(json!({"index": 3, "name": "Pattern1", "objectType": "RectangularPatternFeature", "component": "Root", "detail": {}}));
    let mut tapered = extrude(4, "Extrude2", 0, "Sketch1", "JoinFeatureOperation", distance(1.0));
    tapered["detail"]["taperAngleTwo"] = json!({"kind": "parameter", "name": "t", "value": 0.1});
    items.push(tapered);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.mapped(), 2);
    assert_eq!(import.placeholders(), 3);
    let placeholders: Vec<&Value> = import.features.iter().filter(|f| f["type"] == "placeholder").collect();
    assert_eq!(placeholders.len(), 3);
    assert!(placeholders.iter().all(|p| p["suppressed"] == json!(true)));
    assert_eq!(placeholders[0]["source"], "FilletFeature");
    assert_eq!(placeholders[1]["source"], "RectangularPatternFeature");
    assert!(placeholders[2]["note"].as_str().is_some_and(|n| n.contains("tapered")));
    assert_eq!(import.items[2].status, Status::Placeholder);
    // the placeholders draw nothing: the plate is still the plate
    let m = built(&import.part()["features"]);
    near(m.volume, 8000.0);
}

#[test]
fn an_extrude_of_a_sketch_that_did_not_map_is_a_placeholder() {
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let mut spline = c.clone();
    spline[0] = json!({"id": "c0", "type": "SketchEllipticalArc", "centerSketchPoint": "p0"});
    let items = vec![
        xy(0, "Sketch1", json!(p), json!(spline)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
    ];
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 2);
    assert!(import.items[0].note.as_deref().is_some_and(|n| n.contains("EllipticalArc")));
    assert!(import.items[1].note.as_deref().is_some_and(|n| n.contains("sketch")));
}

#[test]
fn a_suppressed_item_is_written_suppressed_and_builds_nothing() {
    let mut items = plate_items();
    items[1]["isSuppressed"] = json!(true);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.mapped(), 2);
    assert_eq!(import.features[1]["suppressed"], json!(true));
}

fn in_component(mut item: Value, comp: &str) -> Value {
    item["component"] = json!(comp);
    if let Some(d) = item.get_mut("detail").and_then(Value::as_object_mut) {
        d.insert("component".into(), json!(comp));
    }
    item
}

fn two_component_dump(arm_occurrences: Value) -> Dump {
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let items = vec![
        xy(0, "Sketch1", json!(p), json!(c)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
        in_component(xy(2, "Sketch2", json!(rect(0, 0.0, 0.0, 4.0, 2.0).0), json!(rect(0, 0.0, 0.0, 4.0, 2.0).1)), "Arm"),
        in_component(extrude(3, "Extrude2", 2, "Sketch2", "NewBodyFeatureOperation", distance(1.0)), "Arm"),
    ];
    let doc = json!({
        "schema": "mitcad-f3d-dump", "schema_version": 2,
        "components": [{"name": "Root", "id": "r", "is_root": true}, {"name": "Arm", "id": "a"}],
        "occurrences": arm_occurrences,
        "timeline": {"available": true, "items": items},
    });
    Dump::from_json(&doc.to_string()).expect("a dump")
}

#[test]
fn a_component_placed_once_is_built_where_its_occurrence_is() {
    let moved = json!([{"name": "Arm:1", "component": "Arm",
        "transform2": [[1.0, 0.0, 0.0, 10.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]}]);
    let import = timeline_to_features(&two_component_dump(moved));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 16000.0);
    near(m.max[0], 140.0);
    assert_eq!(m.solids, 2);
}

#[test]
fn a_component_placed_twice_is_built_once_and_copied_to_the_other_occurrence() {
    let at = |x: f64| json!([[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    let twice = json!([{"name": "Arm:1", "component": "Arm", "transform2": at(10.0)}, {"name": "Arm:2", "component": "Arm", "transform2": at(30.0)}]);
    let import = timeline_to_features(&two_component_dump(twice));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert!(import.features.iter().any(|f| f["type"] == "moveBody" && f["copy"] == true));
    let m = built(&import.part()["features"]);
    // the root plate, the arm at x 100 and its copy at x 300 (mm)
    near(m.volume, 24000.0);
    near(m.max[0], 340.0);
    assert_eq!(m.solids, 3);
}

#[test]
fn a_component_placed_a_quarter_turn_from_its_first_occurrence_is_copied_turned() {
    let at = json!([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    // a quarter turn about z, then 5 cm along y
    let turned = json!([[0.0, -1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 5.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    let two = json!([{"name": "Arm:1", "component": "Arm", "transform2": at}, {"name": "Arm:2", "component": "Arm", "transform2": turned}]);
    let import = timeline_to_features(&two_component_dump(two));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 24000.0);
    // the 40 x 20 arm turned a quarter about z lies at x -20..0, y 50..90
    near(m.min[0], -20.0);
    near(m.max[1], 90.0);
}

#[test]
fn a_component_with_a_scaled_occurrence_stays_a_placeholder() {
    let id = json!([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    let big = json!([[2.0, 0.0, 0.0, 0.0], [0.0, 2.0, 0.0, 0.0], [0.0, 0.0, 2.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    let two = json!([{"name": "Arm:1", "component": "Arm", "transform2": id}, {"name": "Arm:2", "component": "Arm", "transform2": big}]);
    let import = timeline_to_features(&two_component_dump(two));
    assert_eq!(import.placeholders(), 2);
}

#[test]
fn a_fillet_names_its_edge_by_the_faces_the_kernel_gave_it() {
    let mut items = plate_items();
    // the vertical edge at the origin corner, from its middle point and length (cm)
    items.push(json!({"index": 2, "name": "Fillet1", "objectType": "FilletFeature", "component": "Root", "detail": {
        "edgeSets": [{"radius": {"kind": "parameter", "name": "r", "value": 0.2},
            "edges": [{"kind": "edge", "mid_point": [0.0, 0.0, 0.5], "length": 1.0}]}]}}));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 8000.0 - (1.0 - std::f64::consts::FRAC_PI_4) * 4.0 * 10.0);
}

#[test]
fn a_fillet_whose_edge_is_not_in_the_history_is_a_placeholder() {
    let mut items = plate_items();
    items.push(json!({"index": 2, "name": "Fillet1", "objectType": "FilletFeature", "component": "Root", "detail": {
        "edgeSets": [{"radius": {"kind": "parameter", "name": "r", "value": 0.2},
            "edges": [{"kind": "edge", "mid_point": [9.0, 9.0, 9.0], "length": 1.0}]}]}}));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 1);
}

#[test]
fn a_fitted_spline_and_an_ellipse_map() {
    let pts = json!([
        {"id": "p0", "xyz": [0.0, 0.0, 0.0]}, {"id": "p1", "xyz": [2.0, 1.0, 0.0]}, {"id": "p2", "xyz": [4.0, 0.0, 0.0]},
        {"id": "p3", "xyz": [10.0, 10.0, 0.0]}
    ]);
    let curves = json!([
        {"id": "c0", "type": "SketchFittedSpline", "fitPoints": ["p0", "p1", "p2"]},
        {"id": "c1", "type": "SketchLine", "startSketchPoint": "p2", "endSketchPoint": "p0"},
        {"id": "c2", "type": "SketchEllipse", "centerSketchPoint": "p3", "majorAxisRadius": 2.0, "minorAxisRadius": 1.0, "majorAxis": [1.0, 0.0, 0.0]},
    ]);
    let items = vec![
        xy(0, "Sketch1", pts, curves),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
    ];
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let sketch = &import.features[0];
    let kinds: Vec<&str> = sketch["entities"].as_array().unwrap().iter().filter_map(|e| e["type"].as_str()).collect();
    assert_eq!(kinds, ["spline", "line", "ellipse"]);
}

fn face_extrude(index: i64, operation: &str, cm: f64) -> Value {
    let face = json!({"kind": "face", "objectType": "BRepFace", "point_on_face": [2.0, 1.0, 1.0], "normal_at_point": [0.0, 0.0, 1.0]});
    json!({"index": index, "name": "Extrude2", "objectType": "ExtrudeFeature", "component": "Root", "detail": {
        "operation": operation, "profile": [face], "extentType": "OneSideFeatureExtentType",
        "extentOne": distance(cm), "isSolid": true,
        "taperAngleOne": {"kind": "parameter", "name": "t", "value": 0.0},
    }})
}

#[test]
fn an_extrude_of_a_face_of_the_body_is_a_press_pull_of_that_face() {
    let mut items = plate_items();
    items.push(face_extrude(2, "JoinFeatureOperation", 0.5));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert!(import.features.iter().any(|f| f["type"] == "pressPull" && f["distance"] == json!(5.0)));
    let m = built(&import.part()["features"]);
    near(m.volume, 40.0 * 20.0 * 15.0);
    near(m.max[2], 15.0);
}

/// An item of a type that carries no detail the mapping reads.
fn bare(index: i64, name: &str, kind: &str) -> Value {
    json!({"index": index, "name": name, "objectType": kind, "component": "Root", "detail": {}})
}

#[test]
fn an_item_that_only_places_or_copies_a_component_does_not_block_a_face_extrude() {
    let mut items = plate_items();
    items.push(bare(2, "Copy", "CopyPasteOccurrence"));
    items.push(face_extrude(3, "JoinFeatureOperation", 0.5));
    let import = timeline_to_features(&dump(items));
    // the copy stays a placeholder, the press-pull after it still maps
    assert_eq!(import.placeholders(), 1, "{:?}", import.notes);
    assert!(import.features.iter().any(|f| f["type"] == "pressPull"), "{:?}", import.notes);
    // but an item that changes the body does block it
    let mut items = plate_items();
    items.push(bare(2, "Move", "MoveFeature"));
    items.push(face_extrude(3, "JoinFeatureOperation", 0.5));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 2, "{:?}", import.notes);
}

#[test]
fn a_cut_of_a_face_of_the_body_runs_into_it() {
    let mut items = plate_items();
    items.push(face_extrude(2, "CutFeatureOperation", -0.3));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 40.0 * 20.0 * 7.0);
}

#[test]
fn a_cut_of_a_face_with_only_a_sliver_of_material_behind_it_is_kept_suppressed() {
    // a plate 4 x 2 cm and 1e-6 cm (1e-5 mm) thick: a cut of its top face 3 mm deep removes that sliver, which the
    // kernel cannot build; the press-pull is kept suppressed and the history is the plate
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let mut face = face_extrude(2, "CutFeatureOperation", -0.3);
    face["detail"]["profile"][0]["point_on_face"] = json!([2.0, 1.0, 1e-6]);
    let items = vec![
        xy(0, "Sketch1", json!(p), json!(c)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1e-6)),
        face,
    ];
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let pp = import.features.iter().find(|f| f["type"] == "pressPull").expect("a press-pull");
    assert_eq!(pp["suppressed"], json!(true));
    // a thick plate is not a sliver: the same cut is built
    let mut items = plate_items();
    items.push(face_extrude(2, "CutFeatureOperation", -0.3));
    let import = timeline_to_features(&dump(items));
    let pp = import.features.iter().find(|f| f["type"] == "pressPull").expect("a press-pull");
    assert!(pp.get("suppressed").is_none());
}

#[test]
fn an_extrude_of_a_face_that_is_not_in_the_history_is_a_placeholder() {
    let mut items = plate_items();
    let mut f = face_extrude(2, "JoinFeatureOperation", 0.5);
    f["detail"]["profile"][0]["point_on_face"] = json!([2.0, 1.0, 3.0]);
    items.push(f);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 1);
    assert!(import.notes.iter().any(|n| n.text.contains("not a planar face")), "{:?}", import.notes);
}

#[test]
fn an_extrude_to_a_face_ends_on_that_face() {
    // a plate 4 x 2 x 1 cm, and a second sketch on the bottom plane extruded up to the plate's top face is the plate again
    let mut items = plate_items();
    let (p, c) = rect(10, 0.5, 0.5, 1.5, 1.5);
    items.push(xy(2, "Sketch2", json!(p), json!(c)));
    let face = json!({"kind": "face", "objectType": "BRepFace", "point_on_face": [2.0, 1.0, 1.0], "normal_at_point": [0.0, 0.0, 1.0]});
    let mut e = extrude(3, "Extrude2", 2, "Sketch2", "NewBodyFeatureOperation", json!({}));
    e["detail"]["extentOne"] = json!({"_type": "ToEntityExtentDefinition", "entity": face});
    items.push(e);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert!(import.features.iter().any(|f| f["end"] == "upToFace"));
    let m = built(&import.part()["features"]);
    // the first plate, and a 10 x 10 x 10 mm block inside it that stays its own body
    assert!(m.solids >= 1);
    near(m.max[2], 10.0);
}

fn param(v: f64) -> Value {
    json!({"kind": "parameter", "name": "p", "value": v})
}

fn feature_ref(index: i64, kind: &str) -> Value {
    json!({"kind": "feature", "objectType": kind, "name": format!("Item{index}"), "timeline_index": index})
}

fn item(index: i64, name: &str, kind: &str, detail: Value) -> Value {
    json!({"index": index, "name": name, "objectType": kind, "component": "Root", "detail": detail})
}

/// A 40 x 40 x 10 mm plate (items 0, 1) and a through-all hole of radius 2.5 mm at (cx, cy) cm (items 2, 3).
fn plate_with_hole(cx: f64, cy: f64) -> Vec<Value> {
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 4.0);
    let (hp, hc) = circle("h0", "hc0", cx, cy, 0.25);
    let mut cut = extrude(3, "Cut", 2, "Sketch2", "CutFeatureOperation", distance(1.0));
    cut["detail"]["extentOne"] = json!({"_type": "ThroughAllExtentDefinition", "isPositiveDirection": true});
    vec![
        xy(0, "Sketch1", json!(p), json!(c)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
        xy(2, "Sketch2", json!([hp]), json!([hc])),
        cut,
    ]
}

const PLATE: f64 = 40.0 * 40.0 * 10.0;
fn hole(n: f64) -> f64 {
    n * std::f64::consts::PI * 2.5 * 2.5 * 10.0
}

#[test]
fn a_tapered_blind_extrude_is_the_parts_draft() {
    let (p, c) = rect(0, 0.0, 0.0, 2.0, 2.0);
    let mut e = extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0));
    e["detail"]["taperAngleOne"] = param(10f64.to_radians());
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), e.clone()]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    near(import.features[1]["draft"].as_f64().expect("a draft"), 10.0);
    let top = 20.0 - 2.0 * 10.0 * 10f64.to_radians().tan();
    let want = 10.0 / 3.0 * (400.0 + top * top + 20.0 * top);
    near(built(&import.part()["features"]).volume, want);
    // a negative angle flares outward
    e["detail"]["taperAngleOne"] = param(-10f64.to_radians());
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), e.clone()]));
    assert_eq!(import.features[1]["draftOut"], json!(true));
    let top = 20.0 + 2.0 * 10.0 * 10f64.to_radians().tan();
    near(built(&import.part()["features"]).volume, 10.0 / 3.0 * (400.0 + top * top + 20.0 * top));
    // the angle is measured with the extrusion: on a negative length (against the sketch normal) a positive angle widens
    // and a negative one narrows (the corpus' tapered pins: -3.8 mm, -0.5 degrees, narrower at the far end)
    let mut back = e.clone();
    back["detail"]["extentOne"] = distance(-1.0);
    back["detail"]["taperAngleOne"] = param(-10f64.to_radians());
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), back.clone()]));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    assert!(import.features[1].get("draftOut").is_none(), "{:?}", import.features[1]);
    let top = 20.0 - 2.0 * 10.0 * 10f64.to_radians().tan();
    near(built(&import.part()["features"]).volume, 10.0 / 3.0 * (400.0 + top * top + 20.0 * top));
    back["detail"]["taperAngleOne"] = param(10f64.to_radians());
    let import = timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), back]));
    assert_eq!(import.features[1]["draftOut"], json!(true));
    // a taper of the second side, or of a symmetric extrude, stays a placeholder
    let mut s = e.clone();
    s["detail"]["taperAngleOne"] = param(0.0);
    s["detail"]["taperAngleTwo"] = param(0.1);
    assert_eq!(timeline_to_features(&dump(vec![xy(0, "Sketch1", json!(p), json!(c)), s])).placeholders(), 1);
}

fn circular_pattern(index: i64, seeds: &[i64], count: f64, angle: f64, axis_origin: [f64; 3]) -> Value {
    item(index, "C-Pattern1", "CircularPatternFeature", json!({
        "inputEntities": seeds.iter().map(|i| feature_ref(*i, "ExtrudeFeature")).collect::<Vec<_>>(),
        "patternEntityType": "FeaturesPatternType", "quantity": param(count), "totalAngle": param(angle),
        "isSymmetric": false, "patternComputeOption": "IdenticalPatternCompute", "suppressedElementsIds": [],
        "_f3d_axis": {"direction": [0.0, 0.0, 1.0], "origin": axis_origin},
    }))
}

#[test]
fn a_circular_pattern_of_a_cut_about_an_axis_repeats_it() {
    let mut items = plate_with_hole(3.0, 2.0);
    items.push(circular_pattern(4, &[3], 4.0, std::f64::consts::TAU, [2.0, 2.0, 0.0]));
    let import = timeline_to_features(&dump(items.clone()));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let f = import.features.iter().find(|f| f["type"] == "cpattern").expect("a pattern");
    assert_eq!(f["axis"]["point"], json!([20.0, 20.0, 0.0]));
    near(built(&import.part()["features"]).volume, PLATE - hole(4.0));
    // a partial turn is not guessed; neither is a pattern of a sketch (not a feature that makes a tool)
    items[4] = circular_pattern(4, &[3], 4.0, 3.0, [2.0, 2.0, 0.0]);
    assert_eq!(timeline_to_features(&dump(items.clone())).placeholders(), 1);
    items[4] = circular_pattern(4, &[2], 4.0, std::f64::consts::TAU, [2.0, 2.0, 0.0]);
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
}

/// A gear-like disc: a disc of radius 3 cm, a tooth (a 1 x 0.4 cm rectangle across its rim) joined to it (items 0 to 3), the
/// tooth's two outer vertical edges filleted (item 4, when `blend`), and a circular pattern of six (item 5) of the tooth
/// (item 3) and, when `blend`, of its fillet (item 4).
fn tooth_items(blend: bool, inputs: &[(i64, &str)]) -> Vec<Value> {
    let (cp, cc) = circle("p0", "c0", 0.0, 0.0, 3.0);
    let (tp, tc) = rect(10, 2.5, -0.2, 3.5, 0.2);
    let mut items = vec![
        xy(0, "Base Circle", json!([cp]), json!([cc])),
        extrude(1, "Extrude1", 0, "Base Circle", "NewBodyFeatureOperation", distance(1.0)),
        xy(2, "Tooth", json!(tp), json!(tc)),
        extrude(3, "Extrude2", 2, "Tooth", "JoinFeatureOperation", distance(1.0)),
    ];
    if blend {
        items.push(item(4, "Fillet1", "FilletFeature", json!({"edgeSets": [{"radius": {"kind": "parameter", "name": "r", "value": 0.05},
            "edges": [{"kind": "edge", "mid_point": [3.5, 0.2, 0.5], "length": 1.0}, {"kind": "edge", "mid_point": [3.5, -0.2, 0.5], "length": 1.0}]}]})));
    }
    items.push(item(5, "C-Pattern1", "CircularPatternFeature", json!({
        "inputEntities": inputs.iter().map(|(i, k)| feature_ref(*i, k)).collect::<Vec<_>>(),
        "patternEntityType": "FeaturesPatternType", "quantity": param(6.0), "totalAngle": param(std::f64::consts::TAU),
        "isSymmetric": false, "patternComputeOption": "IdenticalPatternCompute", "suppressedElementsIds": [],
        "_f3d_axis": {"direction": [0.0, 0.0, 1.0], "origin": [0.0, 0.0, 0.0]},
    })));
    items
}

#[test]
fn a_circular_pattern_of_a_tooth_and_its_fillet_blends_every_copy() {
    // the fillet is not a seed (a part repeats a feature's tool body and a blend leaves none): the copies of the tooth are
    // blended alike, on the file's edges turned by each step
    let sharp = timeline_to_features(&dump(tooth_items(false, &[(3, "ExtrudeFeature")])));
    assert_eq!(sharp.placeholders(), 0, "{:?}", sharp.notes);
    let v_sharp = built(&sharp.part()["features"]).volume;
    let blended = timeline_to_features(&dump(tooth_items(true, &[(3, "ExtrudeFeature"), (4, "FilletFeature")])));
    assert_eq!(blended.placeholders(), 0, "{:?}", blended.notes);
    let fillets: Vec<&Value> = blended.features.iter().filter(|f| f["type"] == "fillet").collect();
    assert_eq!(fillets.len(), 6, "the tooth's own and one for each of the five other copies");
    assert!(fillets[1..].iter().all(|f| f["edges"].as_array().map(Vec::len) == Some(2)));
    // twelve convex vertical edges of radius 0.5 mm and height 10 mm each lose (1 - pi/4) r^2 h
    let cut = 12.0 * (1.0 - std::f64::consts::FRAC_PI_4) * 0.25 * 10.0;
    let v = built(&blended.part()["features"]).volume;
    assert!((v - (v_sharp - cut)).abs() < 1e-4, "{v} against {}", v_sharp - cut);
    // a pattern of the blend alone has nothing of its own to repeat
    let alone = timeline_to_features(&dump(tooth_items(true, &[(4, "FilletFeature")])));
    assert_eq!(alone.placeholders(), 1);
    // a fillet that was left out of the history is not repeated either
    let mut items = tooth_items(true, &[(3, "ExtrudeFeature"), (4, "FilletFeature")]);
    items[4]["detail"]["edgeSets"][0]["edges"][0]["mid_point"] = json!([9.0, 9.0, 9.0]);
    let broken = timeline_to_features(&dump(items));
    assert_eq!(broken.placeholders(), 2, "{:?}", broken.notes);
}

#[test]
fn a_chamfer_of_a_full_circle_is_found_by_any_point_of_the_edge() {
    // the file's circle edge starts at x = -r, so its middle is at +r; the kernel's starts at +r, so its middle is at -r
    let (cp, cc) = circle("p0", "c0", 0.0, 0.0, 2.0);
    let items = vec![
        xy(0, "Sketch1", json!([cp]), json!([cc])),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
        item(2, "Chamfer1", "ChamferFeature", json!({"chamferType": "EqualDistanceChamferType", "edgeSets": [{
            "distance": {"kind": "parameter", "name": "d", "value": 0.1},
            "edges": [{"kind": "edge", "length": 4.0 * std::f64::consts::PI, "start_point": [-2.0, 0.0, 1.0], "end_point": [-2.0, 0.0, 1.0], "mid_point": [2.0, 0.0, 1.0]}]}]})),
    ];
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    // a 1 mm chamfer of the top edge removes a triangle of area 0.5 mm^2 all round at the radius 20 mm (Pappus)
    let cylinder = std::f64::consts::PI * 400.0 * 10.0;
    let removed = 0.5 * 1.0 * 1.0 * 2.0 * std::f64::consts::PI * (20.0 - 1.0 / 3.0);
    let v = built(&import.part()["features"]).volume;
    assert!((v - (cylinder - removed)).abs() < 1e-3, "{v} against {}", cylinder - removed);
}

#[test]
fn a_rectangular_pattern_of_a_cut_makes_a_grid() {
    let mut items = plate_with_hole(0.5, 0.5);
    let pattern = |kind: &str, d1: f64, d2: f64| {
        item(4, "R-Pattern1", "RectangularPatternFeature", json!({
            "inputEntities": [feature_ref(3, "ExtrudeFeature")], "patternEntityType": "FeaturesPatternType",
            "directionOne": [1.0, 0.0, 0.0], "directionTwo": [0.0, 1.0, 0.0],
            "quantityOne": param(3.0), "quantityTwo": param(2.0), "distanceOne": param(d1), "distanceTwo": param(d2),
            "patternDistanceType": kind, "patternComputeOption": "IdenticalPatternCompute", "suppressedElementsIds": [],
        }))
    };
    // 3 x 2 holes at 1 cm and 2 cm: x 0.5, 1.5, 2.5, y 0.5, 2.5, all inside the plate
    items.push(pattern("SpacingPatternDistanceType", 1.0, 2.0));
    let import = timeline_to_features(&dump(items.clone()));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    near(built(&import.part()["features"]).volume, PLATE - hole(6.0));
    // the extent is from the first to the last copy: 2 cm over 3 copies is 1 cm apart
    items[4] = pattern("ExtentPatternDistanceType", 2.0, 2.0);
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    near(built(&import.part()["features"]).volume, PLATE - hole(6.0));
}

fn mirror_item(index: i64, kind: &str, input: Value, combine: bool) -> Value {
    item(index, "Mirror1", "MirrorFeature", json!({
        "inputEntities": [input], "patternEntityType": kind, "isCombine": combine,
        "mirrorPlane": {"kind": "construction_plane", "name": "Plane1", "timeline_index": null, "geometry": {
            "type": "Plane", "origin": [2.0, 0.0, 0.0], "normal": [1.0, 0.0, 0.0], "uDirection": [0.0, 1.0, 0.0], "vDirection": [0.0, 0.0, 1.0]}},
    }))
}

#[test]
fn a_mirror_of_a_feature_reflects_its_tool_and_a_mirror_of_the_one_body_joins_the_image() {
    let mut items = plate_with_hole(1.0, 1.0);
    items.push(mirror_item(4, "FeaturesPatternType", feature_ref(3, "ExtrudeFeature"), true));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = import.features.iter().find(|f| f["type"] == "mirror").expect("a mirror");
    assert_eq!(m["body"], json!(false));
    near(built(&import.part()["features"]).volume, PLATE - hole(2.0));
    // the whole body mirrored in the plane x = 40 mm (its right face) joins an image of the same size
    let mut items = plate_items();
    let mut m = mirror_item(2, "BodiesPatternType", json!({"kind": "body", "objectType": "BRepBody"}), true);
    m["detail"]["mirrorPlane"]["geometry"]["origin"] = json!([4.0, 0.0, 0.0]);
    items.push(m.clone());
    let import = timeline_to_features(&dump(items.clone()));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    near(built(&import.part()["features"]).volume, 2.0 * 8000.0);
    // an image that stays a body of its own is not mapped
    m["detail"]["isCombine"] = json!(false);
    items[2] = m;
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
}

fn move_item(index: i64, transform: Value, inputs: Value) -> Value {
    item(index, "Move1", "MoveFeature", json!({
        "definition": {"_type": "MoveFeatureFreeMoveDefinition", "transform": transform.clone()},
        "inputEntities": inputs, "transform": transform,
    }))
}

#[test]
fn a_move_of_the_one_body_is_a_move_body() {
    let body = json!([{"kind": "body", "objectType": "BRepBody"}]);
    let mut items = plate_items();
    // a plate 40 x 20 x 10 mm turned a quarter about z, then lifted 30 mm and shifted 5 mm along x
    let quarter = json!([[0.0, -1.0, 0.0, 0.5], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 3.0], [0.0, 0.0, 0.0, 1.0]]);
    items.push(move_item(2, quarter, body.clone()));
    let import = timeline_to_features(&dump(items.clone()));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 8000.0);
    // x in [-20, 0] + 5, y in [0, 40], z in [30, 40]
    for (got, want) in m.min.iter().zip([-15.0, 0.0, 30.0]).chain(m.max.iter().zip([5.0, 40.0, 40.0])) {
        assert!((got - want).abs() < 1e-6, "{:?} {:?}", m.min, m.max);
    }
    // an identity moves nothing and a move of another kind of input is not guessed
    let id = json!([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    items[2] = move_item(2, id, body);
    assert_eq!(timeline_to_features(&dump(items.clone())).placeholders(), 0);
    items[2] = move_item(2, json!([[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]), json!([{"kind": "face"}]));
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
}

#[test]
fn a_fixed_spline_is_a_control_point_spline_of_the_sketch() {
    // a D shape: a line and a degree 3 B-spline arch through four control points, closed by the line
    let nurbs = json!({"degree": 3, "isPeriodic": false, "isRational": false, "controlPointCount": 4,
        "controlPoints": [[2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]], "knots": [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]});
    let points = json!([{"id": "p0", "xyz": [0.0, 0.0, 0.0]}, {"id": "p1", "xyz": [2.0, 0.0, 0.0]}]);
    let curves = json!([
        {"id": "c0", "type": "SketchLine", "startSketchPoint": "p0", "endSketchPoint": "p1"},
        {"id": "c1", "type": "SketchFixedSpline", "startSketchPoint": null, "endSketchPoint": null, "degree": 3,
         "geometry": {"type": "NurbsCurve3D", "nurbs": nurbs}},
    ]);
    let items = vec![xy(0, "Sketch1", points, curves), extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0))];
    let import = timeline_to_features(&dump(items.clone()));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let sk = &import.features[0];
    let spline = sk["entities"].as_array().expect("entities").iter().find(|e| e["type"] == "cvspline").expect("a cvspline");
    assert_eq!(spline["degree"], json!(3));
    assert_eq!(spline["pts"].as_array().map(Vec::len), Some(4));
    assert_eq!(sk["points"][2]["x"], json!(20.0));
    // the arch is a cubic Bezier 20 mm wide and 7.5 mm high at its crest: its area is below the 150 mm^2 box and above half of it
    let v = built(&import.part()["features"]).volume;
    assert!(v > 750.0 && v < 1500.0, "{v}");
    // a rational one with no weights is not guessed
    let mut r = items.clone();
    r[0]["detail"]["curves"][1]["geometry"]["nurbs"]["isRational"] = json!(true);
    assert_eq!(timeline_to_features(&dump(r)).placeholders(), 2);
    // knots that do not fit
    let mut r = items;
    r[0]["detail"]["curves"][1]["geometry"]["nurbs"]["knots"] = json!([0.0, 1.0]);
    assert_eq!(timeline_to_features(&dump(r)).placeholders(), 2);
}

/// A rational spline is the part's `cvspline` with `weights`: a quarter disc of radius 20 mm whose arc is the rational quadratic
/// Bezier of a circle (weights 1, sqrt(2)/2, 1) has the area of a quarter circle; the same control points without weights are a
/// parabola that bulges out to 6 % more area.
#[test]
fn a_rational_spline_is_a_weighted_cvspline_and_makes_the_exact_arc() {
    let w = std::f64::consts::FRAC_1_SQRT_2;
    let nurbs = |weights: Value, rational: bool| {
        json!({"degree": 2, "isPeriodic": false, "isRational": rational, "controlPointCount": 3,
            "controlPoints": [[2.0, 0.0, 0.0], [2.0, 2.0, 0.0], [0.0, 2.0, 0.0]], "weights": weights, "knots": [0.0, 0.0, 0.0, 1.0, 1.0, 1.0]})
    };
    let build = |n: Value| {
        let points = json!([{"id": "p0", "xyz": [0.0, 0.0, 0.0]}, {"id": "p1", "xyz": [2.0, 0.0, 0.0]}, {"id": "p2", "xyz": [0.0, 2.0, 0.0]}]);
        let curves = json!([
            {"id": "c0", "type": "SketchLine", "startSketchPoint": "p0", "endSketchPoint": "p1"},
            {"id": "c1", "type": "SketchFixedSpline", "startSketchPoint": null, "endSketchPoint": null, "degree": 2,
             "geometry": {"type": "NurbsCurve3D", "nurbs": n}},
            {"id": "c2", "type": "SketchLine", "startSketchPoint": "p2", "endSketchPoint": "p0"},
        ]);
        let items = vec![xy(0, "Sketch1", points, curves), extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0))];
        timeline_to_features(&dump(items))
    };
    let import = build(nurbs(json!([1.0, w, 1.0]), true));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let spline = import.features[0]["entities"].as_array().expect("entities").iter().find(|e| e["type"] == "cvspline").expect("a cvspline").clone();
    assert_eq!(spline["weights"].as_array().map(Vec::len), Some(3));
    let exact = std::f64::consts::PI * 20.0 * 20.0 / 4.0 * 10.0;
    let v = built(&import.part()["features"]).volume;
    assert!((v - exact).abs() / exact < 2e-3, "{v} against {exact}");
    // the same control points without weights are the parabola, a different (larger) body
    let plain = build(nurbs(json!([]), false));
    assert_eq!(plain.placeholders(), 0);
    let vp = built(&plain.part()["features"]).volume;
    assert!((vp - exact).abs() / exact > 0.03, "{vp} against {exact}");
    // weights that do not fit (a count, a zero, a negative one) are refused, not guessed
    for bad in [json!([1.0, w]), json!([1.0, 0.0, 1.0]), json!([1.0, -w, 1.0])] {
        assert_eq!(build(nurbs(bad, true)).placeholders(), 2);
    }
}

/// The lazy decoder sends the design dump as JSON (`serde_json::to_string(&Dump)`) and the main kernel maps the dump read
/// back from it (docs/f3d-lazy.md): the mapping must not depend on which side of that trip the dump is.
#[test]
fn a_dump_sent_as_json_maps_the_same() {
    let d = dump(plate_items());
    let text = serde_json::to_string(&d).expect("serialises");
    let back = Dump::from_json(&text).expect("reads back");
    assert_eq!(d, back);
    let (a, b) = (timeline_to_features(&d), timeline_to_features(&back));
    assert_eq!(a.part(), b.part());
    assert_eq!(a.mapped(), b.mapped());
    assert!(a.mapped() > 0);
}

// ---- bodies selected by the file's names (slice 7) ----

/// A body the way the decoder names one: the item that made it and its index among that item's bodies.
fn body_ref(producer: i64, index: i64) -> Value {
    json!({"kind": "body", "objectType": "BRepBody", "_f3d": {"producer": producer, "body_index": index, "found": "BREP.x.smbh before state 3"}})
}

/// Two plates as bodies of their own: A (item 1) 4 x 2 cm and B (item 3) 3 to 7 x 0 to 2 cm, both 1 cm thick, overlapping in
/// 1 x 2 cm.
fn two_plates() -> Vec<Value> {
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let (q, d) = rect(0, 3.0, 0.0, 7.0, 2.0);
    vec![
        xy(0, "Sketch1", json!(p), json!(c)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
        xy(2, "Sketch2", json!(q), json!(d)),
        extrude(3, "Extrude2", 2, "Sketch2", "NewBodyFeatureOperation", distance(1.0)),
    ]
}

fn combine_item(index: i64, op: &str, target: Value, tools: Vec<Value>, keep: bool) -> Value {
    item(index, "Combine1", "CombineFeature", json!({
        "operation": op, "targetBody": target, "toolBodies": tools, "isKeepToolBodies": keep, "isNewComponent": false,
    }))
}

#[test]
fn a_combine_joins_cuts_or_intersects_the_bodies_the_file_names() {
    // A = 8000 mm^3, B = 8000 mm^3, overlap 1 x 2 x 1 cm = 2000 mm^3
    for (op, want) in [("JoinFeatureOperation", 14000.0), ("CutFeatureOperation", 6000.0), ("IntersectFeatureOperation", 2000.0)] {
        let mut items = two_plates();
        items.push(combine_item(4, op, body_ref(1, 0), vec![body_ref(3, 0)], false));
        let import = timeline_to_features(&dump(items));
        assert_eq!(import.placeholders(), 0, "{op}: {:?}", import.notes);
        let c = import.features.iter().find(|f| f["type"] == "combine").expect("a combine");
        assert_eq!(c["target"], json!({"body": "B1"}));
        assert_eq!(c["tools"], json!([{"body": "B3"}]));
        near(built(&import.part()["features"]).volume, want);
    }
}

#[test]
fn a_combine_that_keeps_its_tools_leaves_them_a_body_of_their_own() {
    let mut items = two_plates();
    items.push(combine_item(4, "CutFeatureOperation", body_ref(1, 0), vec![body_ref(3, 0)], true));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    // the cut target 6000 and the kept tool 8000
    near(built(&import.part()["features"]).volume, 14000.0);
}

#[test]
fn a_consumed_tool_is_no_longer_a_body_the_file_can_name() {
    let mut items = two_plates();
    items.push(combine_item(4, "JoinFeatureOperation", body_ref(1, 0), vec![body_ref(3, 0)], false));
    // a second combine that names the consumed tool
    items.push(combine_item(5, "CutFeatureOperation", body_ref(1, 0), vec![body_ref(3, 0)], false));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.mapped(), 5);
    assert_eq!(import.placeholders(), 1, "{:?}", import.notes);
}

#[test]
fn a_combine_is_not_guessed_when_a_body_is_not_one_the_history_made() {
    let base = two_plates();
    // a tool made by an item the history does not have
    let mut items = base.clone();
    items.push(combine_item(4, "JoinFeatureOperation", body_ref(1, 0), vec![body_ref(9, 0)], false));
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
    // a body of the item with an index it does not have: the plate is one lump, so a second index names nothing
    let mut items = base.clone();
    items.push(combine_item(4, "JoinFeatureOperation", body_ref(1, 0), vec![body_ref(3, 1)], false));
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
    // a body with no producer, a target that is its own tool, and a combine into a new component
    let mut items = base.clone();
    items.push(combine_item(4, "JoinFeatureOperation", body_ref(1, 0), vec![json!({"kind": "body", "objectType": "BRepBody"})], false));
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
    let mut items = base.clone();
    items.push(combine_item(4, "JoinFeatureOperation", body_ref(1, 0), vec![body_ref(1, 0)], false));
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
    let mut items = base;
    let mut c = combine_item(4, "JoinFeatureOperation", body_ref(1, 0), vec![body_ref(3, 0)], false);
    c["detail"]["isNewComponent"] = json!(true);
    items.push(c);
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
}

#[test]
fn an_extrude_with_several_separate_profiles_is_not_one_body_the_file_can_select() {
    // one extrude of two separate squares makes two lumps under the one name B1: (item 1, index 0) selects neither
    let (mut p, mut c) = rect(0, 0.0, 0.0, 1.0, 1.0);
    let (q, d) = rect(4, 3.0, 0.0, 4.0, 1.0);
    p.extend(q);
    c.extend(d);
    let mut items = vec![xy(0, "Sketch1", json!(p), json!(c)), extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0))];
    let (r, e) = rect(0, 0.0, 0.0, 1.0, 1.0);
    items.push(xy(2, "Sketch2", json!(r), json!(e)));
    items.push(extrude(3, "Extrude2", 2, "Sketch2", "NewBodyFeatureOperation", distance(1.0)));
    items.push(combine_item(4, "JoinFeatureOperation", body_ref(3, 0), vec![body_ref(1, 0)], false));
    let import = timeline_to_features(&dump(items));
    let last = import.items.last().expect("an item");
    assert_eq!(last.status, Status::Placeholder, "{:?}", import.notes);
    assert!(last.note.as_deref().unwrap_or("").contains("exactly one"), "{:?}", last.note);
}

#[test]
fn a_paste_of_a_body_is_a_copy_in_place_and_a_move_by_name_moves_only_that_body() {
    let mut items = plate_items();
    items.push(item(2, "Paste1", "CopyPasteBodies", json!({"sourceBody": [body_ref(1, 0)]})));
    // the pasted body (item 2, index 0) moved 10 cm along x
    let shift = json!([[1.0, 0.0, 0.0, 10.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    items.push(move_item(3, shift, json!([body_ref(2, 0)])));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    near(m.volume, 16000.0);
    // the original stays at x 0 to 40 mm, the copy is at 100 to 140 mm
    assert!((m.min[0] - 0.0).abs() < 1e-6 && (m.max[0] - 140.0).abs() < 1e-6, "{:?} {:?}", m.min, m.max);
}

#[test]
fn a_move_of_a_body_the_history_does_not_name_is_not_guessed() {
    let mut items = two_plates();
    let shift = json!([[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
    // two bodies: the one-body rule would move both, the name moves one
    items.push(move_item(4, shift.clone(), json!([body_ref(3, 0)])));
    let import = timeline_to_features(&dump(items.clone()));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let m = built(&import.part()["features"]);
    // plate A at x 0 to 40, plate B (30 to 70 mm) moved to 40 to 80 mm
    assert!((m.min[0] - 0.0).abs() < 1e-6 && (m.max[0] - 80.0).abs() < 1e-6, "{:?} {:?}", m.min, m.max);
    items[4] = move_item(4, shift, json!([body_ref(7, 0)]));
    assert_eq!(timeline_to_features(&dump(items)).placeholders(), 1);
}

#[test]
fn a_cut_acts_only_on_the_participant_bodies_the_file_lists() {
    // a through-all hole of radius 2.5 mm at x 3.5 cm: inside the overlap of A (0 to 4) and B (3 to 7)
    let mut items = two_plates();
    let (hp, hc) = circle("h0", "hc0", 3.5, 1.0, 0.25);
    items.push(xy(4, "Sketch3", json!([hp]), json!([hc])));
    let mut cut = extrude(5, "Cut", 4, "Sketch3", "CutFeatureOperation", distance(1.0));
    cut["detail"]["extentOne"] = json!({"_type": "ThroughAllExtentDefinition", "isPositiveDirection": true});
    cut["detail"]["participantBodies"] = json!([body_ref(1, 0)]);
    items.push(cut);
    // a combine afterwards makes the design name its bodies (the join of the two)
    items.push(combine_item(6, "JoinFeatureOperation", body_ref(1, 0), vec![body_ref(3, 0)], true));
    let import = timeline_to_features(&dump(items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let f = import.features.iter().find(|f| f["type"] == "extrude" && f["op"] == "cut").expect("a cut");
    assert_eq!(f["target"], json!([{"body": "B1"}]));
}

#[test]
fn a_combine_inside_a_component_of_a_design_with_bodies_in_several_components() {
    // the root has a plate; the component Arm (placed 100 mm along y) has two overlapping plates and joins them
    let (p, c) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let (q, d) = rect(0, 3.0, 0.0, 7.0, 2.0);
    let (ap, ac) = rect(0, 0.0, 0.0, 4.0, 2.0);
    let mut items = vec![
        xy(0, "Sketch1", json!(p), json!(c)),
        extrude(1, "Extrude1", 0, "Sketch1", "NewBodyFeatureOperation", distance(1.0)),
        in_component(xy(2, "Sketch2", json!(ap), json!(ac)), "Arm"),
        in_component(extrude(3, "Extrude2", 2, "Sketch2", "NewBodyFeatureOperation", distance(1.0)), "Arm"),
        in_component(xy(4, "Sketch3", json!(q), json!(d)), "Arm"),
        in_component(extrude(5, "Extrude3", 4, "Sketch3", "NewBodyFeatureOperation", distance(1.0)), "Arm"),
    ];
    items.push(in_component(combine_item(6, "JoinFeatureOperation", body_ref(3, 0), vec![body_ref(5, 0)], false), "Arm"));
    let doc = |items: &[Value]| {
        let at = json!([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 10.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]);
        let doc = json!({
            "schema": "mitcad-f3d-dump", "schema_version": 2,
            "components": [{"name": "Root", "id": "r", "is_root": true}, {"name": "Arm", "id": "a"}],
            "occurrences": [{"name": "Arm:1", "component": "Arm", "transform2": at}],
            "timeline": {"available": true, "items": items},
        });
        Dump::from_json(&doc.to_string()).expect("a dump")
    };
    let import = timeline_to_features(&doc(&items));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    // the root plate 8000 and the two Arm plates united 14000
    near(built(&import.part()["features"]).volume, 22000.0);
    // a later join in Arm must not name the tool the combine consumed ("Arm body 2") among its component's bodies
    let (jp, jc) = rect(0, 0.0, 0.0, 1.0, 1.0);
    let mut later = items.clone();
    later.push(in_component(xy(7, "Sketch4", json!(jp), json!(jc)), "Arm"));
    later.push(in_component(extrude(8, "Extrude4", 7, "Sketch4", "JoinFeatureOperation", distance(2.0)), "Arm"));
    let import = timeline_to_features(&doc(&later));
    assert_eq!(import.placeholders(), 0, "{:?}", import.notes);
    let j = import.features.iter().rev().find(|f| f["type"] == "extrude").expect("the join");
    assert_eq!(j["target"], json!({"body": "Arm body"}));
    // the 1 x 1 cm column rises 1 cm above the 1 cm plate: 1000 mm^3 more
    near(built(&import.part()["features"]).volume, 23000.0);
    // a body of another component than the combine's is not selected
    items[6] = in_component(combine_item(6, "JoinFeatureOperation", body_ref(3, 0), vec![body_ref(1, 0)], false), "Arm");
    assert_eq!(timeline_to_features(&doc(&items)).placeholders(), 1);
}
