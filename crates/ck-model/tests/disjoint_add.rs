//! Task 5 repro: an `add` while the body holds a separate (disjoint) lump.
//! Two disjoint extrudes, then a revolve add that fuses to one lump, then a
//! feature that refers to a face of that revolve — the failure the GUI saw
//! when a library part was inserted into a document that already had a solid.
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

fn regen_text(text: &str) -> (Option<ck_topo::body::Body>, Vec<String>) {
    let part = Part::parse(text).unwrap();
    let mut ex = Exec(Brep::new());
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    (ex.0.body.clone(), errs)
}

/// An 8x8x5 slab on the `top` plane at x in [x0, x0+8], y in [0, 8].
fn slab(id: &str, x0: f64) -> String {
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":{x0},"y":0}},{{"id":"p2","x":{},"y":0}},{{"id":"p3","x":{},"y":8}},{{"id":"p4","x":{x0},"y":8}}],"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},{{"type":"extrude","id":"E{id}","op":"add","sketch":"S{id}","end":"blind","depth":5}}"#,
        x0 + 8.0,
        x0 + 8.0
    )
}

/// A cylinder (revolved rectangle about its left edge's axis) whose axis sits
/// on the front plane (world y=0) through the slab's side: it bites into the
/// slab at x0 (z in [-4,0], y in [0,3]) and sticks out of its top, so an `add`
/// and a `cut` both change the volume. The top cap `FC.C1` is planar, which
/// sketch-on-face planes require.
fn cylinder_add(id: &str, cx: f64, op: &str) -> String {
    format!(
        r#"{{"type":"sketch","id":"S{id}","plane":{{"kind":"std","id":"front"}},"points":[{{"id":"a","x":{cx},"y":-4}},{{"id":"b","x":{},"y":-4}},{{"id":"c","x":{},"y":2}},{{"id":"d","x":{cx},"y":2}}],"entities":[{{"type":"line","id":"e1","p1":"a","p2":"b"}},{{"type":"line","id":"e2","p1":"b","p2":"c"}},{{"type":"line","id":"e3","p1":"c","p2":"d"}},{{"type":"line","id":"e4","p1":"d","p2":"a"}},{{"type":"line","id":"ax","p1":"a","p2":"d","construction":true}}],"constraints":[]}},{{"type":"revolve","id":"F{id}","op":"{op}","sketch":"S{id}","axis":"ax","angle":360}}"#,
        cx + 3.0,
        cx + 3.0
    )
}

fn doc(feats: &[String]) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"name":"repro","features":[{}]}}"#,
        feats.join(",")
    )
}

#[test]
fn revolve_add_on_a_two_lump_body() {
    // two disjoint slabs, then a cylinder that fuses to the first
    let text = doc(&[slab("1", 0.0), slab("2", 30.0), cylinder_add("C", 4.0, "add")]);
    let (b, errs) = regen_text(&text);
    assert!(errs.is_empty(), "errors: {errs:?}");
    let b = b.expect("a body");
    assert_eq!(b.num_lumps(), 2, "slab2 must stay separate");
    let mp = ck_topo::mass::mass_props(&b);
    // cylinder r=3 h=6 (54 pi) overlaps the slab only where z in [0,2] and
    // y in [0,3] (half the disc over 2 units = 9 pi): the add contributes 45 pi
    let expect = 8.0 * 8.0 * 5.0 * 2.0 + std::f64::consts::PI * 45.0;
    assert!((mp.volume - expect).abs() < 1e-3, "volume {} vs {expect}", mp.volume);
    assert!(ck_check::validate(&b, 1e-6).is_empty());
}

#[test]
fn face_reference_after_a_revolve_add_on_two_lumps() {
    // ...then a feature that names a face of that revolve: a sketch whose plane
    // is the revolve's planar cap. The face name must resolve after the add
    // united the revolve into a body that still holds a separate lump.
    let feats = [
        slab("1", 0.0).to_string(),
        slab("2", 30.0).to_string(),
        cylinder_add("C", 4.0, "add"),
    ];
    let mut all = feats.to_vec();
    // list the faces the revolve produced so the test can name one
    let (b, errs) = regen_text(&doc(&all));
    assert!(errs.is_empty(), "errors: {errs:?}");
    let b = b.expect("a body");
    let rev: Vec<String> = b.face_ids().map(|f| b.face_id_str(f).to_string()).filter(|s| s.starts_with("FC.")).collect();
    assert!(!rev.is_empty(), "the revolve should have faces");
    // sketch-on-face planes need a planar face: the revolve's top cap
    let lateral = rev.iter().find(|s| s.contains(".C")).cloned().unwrap_or_else(|| rev[0].clone());
    all.push(format!(
        r#"{{"type":"sketch","id":"SH","plane":{{"kind":"face","id":"{lateral}"}},"points":[{{"id":"q1","x":-1,"y":0}},{{"id":"q2","x":1,"y":0}},{{"id":"q3","x":1,"y":2}},{{"id":"q4","x":-1,"y":2}}],"entities":[{{"type":"line","id":"h1","p1":"q1","p2":"q2"}},{{"type":"line","id":"h2","p1":"q2","p2":"q3"}},{{"type":"line","id":"h3","p1":"q3","p2":"q4"}},{{"type":"line","id":"h4","p1":"q4","p2":"q1"}}],"constraints":[]}}"#
    ));
    let (b, errs) = regen_text(&doc(&all));
    assert!(errs.is_empty(), "errors after face reference ({lateral}): {errs:?}");
    let b = b.expect("a body");
    assert_eq!(b.num_lumps(), 2, "slab2 must stay separate");
    assert!(ck_check::validate(&b, 1e-6).is_empty());
}

#[test]
fn cut_add_on_a_two_lump_body() {
    // a cut while a separate lump exists (the other direction of the symptom)
    let text = doc(&[
        slab("1", 0.0),
        slab("2", 30.0),
        cylinder_add("C", 4.0, "cut"),
    ]);
    let (b, errs) = regen_text(&text);
    assert!(errs.is_empty(), "errors: {errs:?}");
    let b = b.expect("a body");
    let mp = ck_topo::mass::mass_props(&b);
    // the cut removes the overlap: half the disc over z in [0,2] = 9 pi
    let expect = 8.0 * 8.0 * 5.0 * 2.0 - std::f64::consts::PI * 9.0;
    assert!((mp.volume - expect).abs() < 1e-3, "volume {} vs {expect}", mp.volume);
    assert!(ck_check::validate(&b, 1e-6).is_empty());
}
