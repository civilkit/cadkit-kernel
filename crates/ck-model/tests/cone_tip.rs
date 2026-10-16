//! A circle extruded with a draft (the GUI's "Cone") and a draft feature on a cylinder wall: at the height
//! where the taper closes to a point the cone gets one apex vertex; past it the error is `draftTooLarge`
//! with `maxHeight`; the regen result reports the same number as `limits` so a UI need not recompute it.
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, Tracer};
use ck_model::Part;
use std::f64::consts::PI;

#[derive(Default)]
struct Noop;
impl Tracer for Noop {
    fn begin(&mut self, _label: &str) -> usize {
        0
    }
    fn relabel(&mut self, _id: usize, _label: &str) {}
    fn end(&mut self, _id: usize) {}
}
struct Exec {
    brep: Brep,
    labels: Noop,
}
impl FeatureExec for Exec {
    fn tracer(&mut self) -> Option<&mut dyn Tracer> {
        Some(&mut self.labels)
    }
    fn face_count(&self) -> Option<usize> {
        self.brep.face_count()
    }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> {
        self.brep.face_frame(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

struct Out {
    brep: Brep,
    errors: Vec<regen::RegenError>,
    wire: Json,
}

fn run(src: &str) -> Out {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    Out { wire: r.to_json(), brep: ex.brep, errors: r.errors }
}

fn param(o: &Out, k: &str) -> Option<Json> {
    o.errors.first().and_then(|e| e.error.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone()))
}

fn limit(o: &Out, id: &str) -> Option<f64> {
    let Json::Obj(m) = &o.wire else { return None };
    let (_, Json::Arr(l)) = m.iter().find(|(k, _)| k == "limits")? else { return None };
    l.iter().find_map(|e| {
        let a = e.as_arr()?;
        (a[0].as_str() == Some(id)).then(|| a[1].get("maxHeight").and_then(|v| v.as_f64())).flatten()
    })
}

const R: f64 = 10.0;

fn cone_doc(depth: f64, draft_deg: f64, extra: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],
         "entities":[{{"type":"circle","id":"e1","c":"c","r":{R}}}],"constraints":[]}},
        {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth},"draft":{draft_deg:.17}}}{extra}]}}"#
    )
}

fn volume(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn assert_valid(o: &Out, what: &str) {
    assert!(o.errors.is_empty(), "{what}: {:?}", o.errors);
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-6);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
}

fn cone_height(h: f64) -> f64 {
    // the draft that makes the taper close exactly at height `h`
    (R / h).atan().to_degrees()
}

#[test]
fn extrude_draft_exact_tip_is_a_cone_with_one_apex() {
    for h in [10.0, 12.0, 37.5, 0.8] {
        let o = run(&cone_doc(h, cone_height(h), ""));
        assert_valid(&o, &format!("tip at {h}"));
        let b = o.brep.body.as_ref().unwrap();
        assert_eq!(b.face_ids().count(), 2, "base disc and cone only");
        // a single apex vertex, and no edge shorter than the tolerance
        let apex = b.vertex_ids().filter(|&v| (b.vertex(v).p.y.abs() - h).abs() < 1e-6).count();
        assert_eq!(apex, 1, "one apex vertex: {:?}", b.vertex_ids().map(|v| b.vertex(v).p).collect::<Vec<_>>());
        for e in b.edge_ids() {
            assert!(b.edge_length(e) > 1e-6, "zero-length edge {e:?}");
        }
        let want = PI * R * R * h / 3.0;
        assert!((volume(&o) - want).abs() < 1e-6 * want, "h={h}: volume {} want {want}", volume(&o));
        let tip = limit(&o, "F2").expect("limits.F2");
        assert!((tip - h).abs() < 1e-9 * h, "limit {tip} vs {h}");
    }
}

#[test]
fn extrude_draft_just_below_tip_keeps_the_flat_top() {
    let a = cone_height(10.0);
    for h in [9.9, 9.999, 5.0] {
        let o = run(&cone_doc(h, a, ""));
        assert_valid(&o, &format!("h {h}"));
        let b = o.brep.body.as_ref().unwrap();
        assert_eq!(b.face_ids().count(), 3, "base, top and cone");
        let r1 = R - h * a.to_radians().tan();
        let want = PI * h / 3.0 * (R * R + R * r1 + r1 * r1);
        assert!((volume(&o) - want).abs() < 1e-6 * want, "h={h}: {} vs {want}", volume(&o));
        let tip = limit(&o, "F2").expect("limits.F2");
        assert!((tip - 10.0).abs() < 1e-9, "limit {tip}");
    }
}

#[test]
fn extrude_draft_past_tip_reports_max_height() {
    let a = cone_height(10.0);
    for h in [10.001, 10.5, 40.0] {
        let o = run(&cone_doc(h, a, ""));
        let e = &o.errors.first().unwrap_or_else(|| panic!("h={h} should fail")).error;
        assert_eq!(e.code, "draftTooLarge", "{e:?}");
        let max = param(&o, "maxHeight").and_then(|j| j.as_f64()).expect("maxHeight");
        assert!((max - 10.0).abs() < 1e-9, "maxHeight {max}");
        let d = param(&o, "detail").and_then(|j| j.as_str().map(str::to_string)).unwrap();
        assert!(d.starts_with("the taper closes to a point at height 10"), "{d}");
        assert!(!d.contains("shrinks to nothing"), "{d}");
        // the reported height builds
        assert_valid(&run(&cone_doc(max, a, "")), "at maxHeight");
    }
}

#[test]
fn draft_feature_on_a_cylinder_wall_reaches_the_tip() {
    // F2: straight cylinder R=10, height 10 from the top plane; F3 drafts its wall about the top plane
    let src = |deg: f64| {
        let base = cone_doc(10.0, 0.0, "").replace(r#","draft":0.00000000000000000"#, "");
        let i = base.rfind("]}").unwrap();
        format!(
            r#"{},{{"type":"draft","id":"F3","neutral":{{"kind":"std","id":"top"}},"faces":["F2.S.e1"],"angle":{deg:.17}}}]}}"#,
            &base[..i]
        )
    };
    let o = run(&src(45.0));
    assert_valid(&o, "draft feature at the tip");
    let want = PI * R * R * 10.0 / 3.0;
    assert!((volume(&o) - want).abs() < 1e-6 * want, "{} vs {want}", volume(&o));
    let o = run(&src(50.0));
    let e = &o.errors.first().expect("past the tip fails").error;
    assert_eq!(e.code, "draftTooLarge");
    let max = param(&o, "maxHeight").and_then(|j| j.as_f64()).expect("maxHeight");
    let want = R / 50f64.to_radians().tan();
    assert!((max - want).abs() < 1e-9, "{max} vs {want}");
    assert_valid(&run(&src(40.0)), "draft feature below the tip");
}

#[test]
fn polygon_taper_past_collapse_reports_max_height() {
    // a 20 x 20 square collapses at height 10 / tan(draft); the deepest build is just below it
    let doc = |depth: f64| {
        format!(
            r#"{{"format":"ck-part","version":1,"features":[
            {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},
             "points":[{{"id":"p1","x":-10,"y":-10}},{{"id":"p2","x":10,"y":-10}},{{"id":"p3","x":10,"y":10}},{{"id":"p4","x":-10,"y":10}}],
             "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
                         {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
            {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{depth},"draft":10}}]}}"#
        )
    };
    let tip = 10.0 / 10f64.to_radians().tan();
    let o = run(&doc(tip * 1.2));
    assert_eq!(o.errors.first().expect("fails").error.code, "draftTooLarge");
    let max = param(&o, "maxHeight").and_then(|j| j.as_f64()).expect("maxHeight");
    assert!(max < tip && max > tip * 0.999, "maxHeight {max} vs {tip}");
    assert_valid(&run(&doc(max)), "at maxHeight");
}
