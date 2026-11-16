//! The Z-up standard planes `xz` and `yz` (Front and Right in a Z-up view): y runs up (+Z) in both, so a sketch on them stands
//! upright. A 10 x 4 rectangle (sketch x 0..10, y 0..4) extruded 3 mm on each plane: where the box lands says which way the
//! plane's x, y and normal run. The three older planes keep the frames every saved part was drawn with.
#[path = "robust_common/mod.rs"]
mod common;

fn com(plane: &str) -> [f64; 3] {
    let src = format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"{plane}"}},"points":[{{"id":"a","x":0,"y":0}},{{"id":"b","x":10,"y":0}},{{"id":"c","x":10,"y":4}},{{"id":"d","x":0,"y":4}}],
          "entities":[{{"type":"line","id":"l1","p1":"a","p2":"b"}},{{"type":"line","id":"l2","p1":"b","p2":"c"}},{{"type":"line","id":"l3","p1":"c","p2":"d"}},{{"type":"line","id":"l4","p1":"d","p2":"a"}}],"constraints":[]}},
        {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":3}}]}}"#
    );
    let b = common::regen_body(&src).unwrap_or_else(|| panic!("{plane}: no body"));
    let m = ck_topo::mass::mass_props(&b);
    assert!((m.volume - 120.0).abs() < 1e-6, "{plane}: volume {}", m.volume);
    [m.com.x, m.com.y, m.com.z]
}

fn near(a: [f64; 3], b: [f64; 3]) {
    assert!((0..3).all(|i| (a[i] - b[i]).abs() < 1e-6), "{a:?} != {b:?}");
}

#[test]
fn the_three_older_planes_are_unchanged() {
    near(com("front"), [5.0, 2.0, 1.5]);
    near(com("top"), [5.0, 1.5, -2.0]);
    near(com("right"), [1.5, 2.0, -5.0]);
}

#[test]
fn xz_and_yz_have_y_running_up() {
    // xz is seen from -Y: x right, y up (+Z), the extrusion grows toward -Y
    near(com("xz"), [5.0, -1.5, 2.0]);
    // yz is seen from +X: x = +Y, y up (+Z), the extrusion grows toward +X
    near(com("yz"), [1.5, 5.0, 2.0]);
}
