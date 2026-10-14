//! `regions` selection of `extrude` / `revolve` (docs/spec-agent-interface.md, "Region selection").

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

/// Run a part; (errors as "F3:code {params}", volume, sorted face ids).
fn run(src: &str) -> (Vec<String>, f64, Vec<String>) {
    let part = Part::from_json(&ck_json::parse(src).unwrap()).unwrap();
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let r = regen::regen(&part, &RegenOpts::default(), &mut ex);
    let errs: Vec<String> = r.errors.iter().map(|e| format!("{}:{}", e.feature, e.error.describe())).collect();
    match ex.brep.body.as_ref() {
        Some(b) => {
            let iss = ck_check::validate(b, 1e-6);
            assert!(iss.is_empty(), "{:?}", iss.iter().take(3).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
            let mut ids: Vec<String> = b.face_ids().map(|f| b.face_id_str(f)).collect();
            ids.sort();
            (errs, ck_topo::mass::mass_props(b).volume, ids)
        }
        None => (errs, 0.0, vec![]),
    }
}

/// Rectangle 40x20 (e1..e4) with two circles inside: A (10,10) r4 = e5, B (30,10) r4 = e6,
/// plus a construction line e7. Regions: rectangle with two holes, disc A, disc B.
fn two_circles(features: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[
          {{"id":"p1","x":0,"y":0}},{{"id":"p2","x":40,"y":0}},{{"id":"p3","x":40,"y":20}},{{"id":"p4","x":0,"y":20}},
          {{"id":"ca","x":10,"y":10}},{{"id":"cb","x":30,"y":10}},{{"id":"l1","x":0,"y":30}},{{"id":"l2","x":5,"y":30}}],
         "entities":[
          {{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
          {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}},
          {{"type":"circle","id":"e5","c":"ca","r":4}},{{"type":"circle","id":"e6","c":"cb","r":4}},
          {{"type":"line","id":"e7","p1":"l1","p2":"l2","construction":true}}],"constraints":[]}},
        {features}]}}"#
    )
}

fn ext(op: &str, regions: &str, extra: &str) -> String {
    format!(r#"{{"type":"extrude","id":"F2","op":"{op}","sketch":"F1","end":"blind","depth":10{regions}{extra}}}"#)
}
fn sel(r: &str) -> String {
    format!(r#","regions":{r}"#)
}

const DISC: f64 = PI * 16.0 * 10.0;
const PLATE: f64 = 8000.0 - 2.0 * DISC;

fn vol(features: &str) -> f64 {
    let (e, v, _) = run(&two_circles(features));
    assert!(e.is_empty(), "{e:?}");
    v
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-3 * b.max(1.0), "{a} vs {b}");
}

#[test]
fn select_one_circle_either_circle_or_both() {
    close(vol(&ext("add", &sel(r#"["e5"]"#), "")), DISC);
    close(vol(&ext("add", &sel(r#"["e6"]"#), "")), DISC);
    close(vol(&ext("add", &sel(r#"["e5","e6"]"#), "")), 2.0 * DISC);
    // duplicates ignored
    close(vol(&ext("add", &sel(r#"["e5","e5"]"#), "")), DISC);
}

#[test]
fn select_rectangle_keeps_its_holes_and_leaves_islands_out() {
    // the circles are holes of the rectangle region AND regions of their own: only the plate is built
    close(vol(&ext("add", &sel(r#"["e1"]"#), "")), PLATE);
    close(vol(&ext("add", &sel(r#"["e3"]"#), "")), PLATE);
    // naming a hole entity as well adds that island region back
    close(vol(&ext("add", &sel(r#"["e1","e5"]"#), "")), PLATE + DISC);
    // absent = every region of the sketch, as before (circles inside the rectangle are only its holes)
    close(vol(&ext("add", "", "")), PLATE);
}

#[test]
fn a_lone_selected_region_is_named_as_if_alone_in_the_sketch() {
    let (_, _, a) = run(&two_circles(&ext("add", &sel(r#"["e6"]"#), "")));
    assert!(a.iter().all(|n| !n.contains('~')), "{a:?}");
    let (_, _, both) = run(&two_circles(&ext("add", &sel(r#"["e5","e6"]"#), "")));
    assert!(both.iter().any(|n| n.contains('~')), "{both:?}");
}

#[test]
fn bad_selections_are_structured_errors() {
    let err = |r: &str| run(&two_circles(&ext("add", &sel(r), ""))).0;
    assert_eq!(err(r#"["e99"]"#), ["F2:extrudeRegionUnknown {\"ids\":[\"e99\"]}"]);
    assert_eq!(err(r#"["e5","zz"]"#), ["F2:extrudeRegionUnknown {\"ids\":[\"zz\"]}"]);
    assert_eq!(err("[]"), ["F2:extrudeRegionNone {\"regions\":[]}"]);
    // a real entity that bounds no region
    assert_eq!(err(r#"["e7"]"#), ["F2:extrudeRegionNone {\"regions\":[\"e7\"]}"]);
    assert_eq!(err("5").len(), 1);
    assert_eq!(err("[1]").len(), 1);
}

/// Concentric circles: C (r8) = e5 with D (r3) = e6 inside: regions ring and disc.
fn concentric(features: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],
         "entities":[{{"type":"circle","id":"e5","c":"c","r":8}},{{"type":"circle","id":"e6","c":"c","r":3}}],"constraints":[]}},
        {features}]}}"#
    )
}

#[test]
fn cut_one_region_through_a_boss_made_from_the_other() {
    let ring = PI * (64.0 - 9.0);
    let disc = PI * 9.0;
    // ring boss 10, then a 20 high disc boss through its hole: ring*10 + disc*20
    let f = |cut: &str| {
        format!(
            r#"{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10,"regions":["e5"]}},
               {{"type":"extrude","id":"F3","op":"add","sketch":"F1","end":"blind","depth":20,"regions":["e6"]}}{cut}"#
        )
    };
    let (e, v, _) = run(&concentric(&f("")));
    assert!(e.is_empty(), "{e:?}");
    close(v, ring * 10.0 + disc * 20.0);
    // cut the disc 15 deep (flip = along +normal): disc part left 5
    let cut = r#",{"type":"extrude","id":"F4","op":"cut","sketch":"F1","end":"blind","depth":15,"flip":true,"regions":["e6"]}"#;
    let (e, v, _) = run(&concentric(&f(cut)));
    assert!(e.is_empty(), "{e:?}");
    close(v, ring * 10.0 + disc * 5.0);
    // cut the ring 4 deep: ring left 6
    let cut = r#",{"type":"extrude","id":"F4","op":"cut","sketch":"F1","end":"blind","depth":4,"flip":true,"regions":["e5"]}"#;
    let (e, v, _) = run(&concentric(&f(cut)));
    assert!(e.is_empty(), "{e:?}");
    close(v, ring * 6.0 + disc * 20.0);
}

#[test]
fn draft_flip_and_mid_work_with_a_selection() {
    let v = vol(&ext("add", &sel(r#"["e5"]"#), r#","draft":5"#));
    assert!(v < DISC && v > 0.7 * DISC, "{v}");
    close(vol(&ext("add", &sel(r#"["e5"]"#), r#","flip":true"#)), DISC);
    let mid = ext("add", &sel(r#"["e5"]"#), "").replace("\"blind\"", "\"mid\"");
    close(vol(&mid), DISC);
}

#[test]
fn revolve_selection() {
    // two rectangles at different radii from the axis line; revolve only the far one
    let src = |regions: &str| {
        format!(
            r#"{{"format":"ck-part","version":1,"features":[
            {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"front"}},"points":[
              {{"id":"a1","x":10,"y":0}},{{"id":"a2","x":20,"y":0}},{{"id":"a3","x":20,"y":5}},{{"id":"a4","x":10,"y":5}},
              {{"id":"b1","x":30,"y":0}},{{"id":"b2","x":40,"y":0}},{{"id":"b3","x":40,"y":5}},{{"id":"b4","x":30,"y":5}},
              {{"id":"x1","x":0,"y":0}},{{"id":"x2","x":0,"y":10}}],
             "entities":[
              {{"type":"line","id":"a","p1":"a1","p2":"a2"}},{{"type":"line","id":"b","p1":"a2","p2":"a3"}},
              {{"type":"line","id":"c","p1":"a3","p2":"a4"}},{{"type":"line","id":"d","p1":"a4","p2":"a1"}},
              {{"type":"line","id":"f","p1":"b1","p2":"b2"}},{{"type":"line","id":"g","p1":"b2","p2":"b3"}},
              {{"type":"line","id":"h","p1":"b3","p2":"b4"}},{{"type":"line","id":"i","p1":"b4","p2":"b1"}},
              {{"type":"line","id":"ax","p1":"x1","p2":"x2","construction":true}}],"constraints":[]}},
            {{"type":"revolve","id":"F2","op":"add","sketch":"F1","axis":"ax","angle":360{regions}}}]}}"#
        )
    };
    let ring = |r0: f64, r1: f64| PI * (r1 * r1 - r0 * r0) * 5.0;
    let (e, v, _) = run(&src(""));
    assert!(e.is_empty(), "{e:?}");
    close(v, ring(10.0, 20.0) + ring(30.0, 40.0));
    let (e, v, _) = run(&src(r#","regions":["g"]"#));
    assert!(e.is_empty(), "{e:?}");
    close(v, ring(30.0, 40.0));
    let (e, _, _) = run(&src(r#","regions":["zz"]"#));
    assert_eq!(e, ["F2:extrudeRegionUnknown {\"ids\":[\"zz\"]}"]);
}

#[test]
fn island_inside_a_hole_is_its_own_region() {
    // r10 (e1) with hole r5 (e2) and an island r2 (e3) inside that hole
    let src = |regions: &str| {
        format!(
            r#"{{"format":"ck-part","version":1,"features":[
            {{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],
             "entities":[{{"type":"circle","id":"e1","c":"c","r":10}},{{"type":"circle","id":"e2","c":"c","r":5}},{{"type":"circle","id":"e3","c":"c","r":2}}],"constraints":[]}},
            {{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":1{regions}}}]}}"#
        )
    };
    let a = |r0: f64, r1: f64| PI * (r1 * r1 - r0 * r0);
    let both = a(5.0, 10.0) + a(0.0, 2.0);
    for (sel, want) in [("", both), (r#","regions":["e1"]"#, a(5.0, 10.0)), (r#","regions":["e3"]"#, a(0.0, 2.0)), (r#","regions":["e1","e3"]"#, both)] {
        let (e, v, _) = run(&src(sel));
        assert!(e.is_empty(), "{sel}: {e:?}");
        close(v, want);
    }
    // e2 only bounds the ring's hole: it selects the filled r5 disc
    let (e, v, _) = run(&src(r#","regions":["e2"]"#));
    assert!(e.is_empty(), "{e:?}");
    close(v, a(0.0, 5.0));
}
