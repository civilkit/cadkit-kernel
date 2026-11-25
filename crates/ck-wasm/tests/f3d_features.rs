//! `f3dFeatures` on the main kernel (docs/f3d-lazy.md): a design dump (what the lazy decoder's `f3dDump` sends, here written by hand)
//! is mapped to a ck-part feature list and checked against the reference body by regenerating it in the kernel.

use ck_json::Json;

fn kernel(msg: &str) -> Json {
    let env = format!("{{\"id\":1,\"msg\":{msg}}}");
    let r = ck_server::handle(env.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    let v = ck_json::parse(j).unwrap();
    assert_eq!(v.get_bool("ok"), Some(true), "{v:?}");
    v.get("result").unwrap().clone()
}

/// A 4 x 2 cm sketch on the front plane, extruded 1 cm as a new body: 8000 mm^3, box 40 x 20 x 10 mm.
const DUMP: &str = r#"{"schema":"mitcad-f3d-dump","schema_version":2,"timeline":{"available":true,"items":[
 {"index":0,"name":"Sketch1","objectType":"Sketch","component":"Root","detail":{"name":"Sketch1","component":"Root","isVisible":true,
  "model_frame":{"origin":[0,0,0],"x_axis":[1,0,0],"y_axis":[0,1,0],"z_axis":[0,0,1]},
  "points":[{"id":"p0","xyz":[0,0,0]},{"id":"p1","xyz":[4,0,0]},{"id":"p2","xyz":[4,2,0]},{"id":"p3","xyz":[0,2,0]}],
  "curves":[{"id":"c0","type":"SketchLine","startSketchPoint":"p0","endSketchPoint":"p1"},{"id":"c1","type":"SketchLine","startSketchPoint":"p1","endSketchPoint":"p2"},
            {"id":"c2","type":"SketchLine","startSketchPoint":"p2","endSketchPoint":"p3"},{"id":"c3","type":"SketchLine","startSketchPoint":"p3","endSketchPoint":"p0"}]}},
 {"index":1,"name":"Extrude1","objectType":"ExtrudeFeature","component":"Root","detail":{"operation":"NewBodyFeatureOperation",
  "profile":[{"kind":"profile","sketch":"Sketch1","sketch_timeline_index":0}],"extentType":"OneSideFeatureExtentType",
  "extentOne":{"_type":"DistanceExtentDefinition","distance":{"kind":"parameter","name":"d","value":1.0}},"isSolid":true,
  "taperAngleOne":{"kind":"parameter","name":"t","value":0.0}}}]}}"#;

fn run(reference: &str) -> Json {
    let dump: String = DUMP.split_whitespace().collect::<Vec<_>>().join(" ");
    ck_wasm::engine::install();
    let msg = format!("{{\"type\":\"f3dFeatures\",\"designs\":[{{\"label\":\"d\",\"dump\":{dump},\"reference\":{reference}}},{{\"label\":\"none\",\"dump\":null}}]}}");
    kernel(&msg)
}

#[test]
fn the_dump_maps_and_verifies_against_the_reference() {
    let r = run(r#"{"volume":8000,"min":[0,0,0],"max":[40,20,10],"solids":1}"#);
    let designs = r.get("designs").and_then(|d| d.as_arr()).unwrap();
    assert_eq!(designs.len(), 1, "the design without a dump is left out");
    let d = &designs[0];
    assert_eq!(d.get_f64("mapped"), Some(2.0));
    assert_eq!(d.get_f64("placeholders"), Some(0.0));
    assert_eq!(d.get("check").unwrap().get_str("verdict"), Some("verified"), "{:?}", d.get("check"));
}

#[test]
fn a_reference_that_differs_is_unverified_with_the_reason() {
    let r = run(r#"{"volume":9000,"min":[0,0,0],"max":[40,20,11.25],"solids":1}"#);
    let d = &r.get("designs").and_then(|d| d.as_arr()).unwrap()[0];
    let check = d.get("check").unwrap();
    assert_eq!(check.get_str("verdict"), Some("unverified"));
    assert!(check.get_str("reason").is_some());
    // no reference at all: unverified, never verified
    let r = run("null");
    let check = r.get("designs").and_then(|d| d.as_arr()).unwrap()[0].get("check").unwrap().clone();
    assert_eq!(check.get_str("verdict"), Some("unverified"));
}
