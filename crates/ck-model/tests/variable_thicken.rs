//! Variable thickness for `thicken`: `thicknessAt {along: [x,y,z], stops: [[s,t], ...]}` evaluates
//! the thickness at every point of the sheet along `along` (linear between stops, clamped to the
//! end stops), so the solid's volume is the field integrated over the sheet. A flat 100x50 sheet
//! under a ramp 1..3 along x is exactly 10000 mm3; the cap7 blade sheet under a root-4 / tip-1
//! taper is 1.25x the constant t=2 body (the field is linear in z and the loft's sections are pure
//! translates, so its sheet-average is 2.5 against the constant's 2); a constant stop list routes
//! to the plain `thicken` path — bit-identical volume and face names. Stop lists the kernel cannot
//! use (short of the sheet, non-positive, beyond the sheet's curvature) are structured errors with
//! a `hint`, never a trap.
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
    brep: Brep,
    errors: Vec<regen::RegenError>,
}

fn run(src: &str) -> Out {
    let part = Part::parse(src).unwrap();
    let mut ex = Exec(Brep::new());
    let opts = RegenOpts::from_json(Some(&ck_json::parse(r#"{"solveSketches":true}"#).unwrap()));
    let r = regen::regen(&part, &opts, &mut ex);
    Out { brep: ex.0, errors: r.errors }
}

fn vol(o: &Out) -> f64 {
    ck_topo::mass::mass_props(o.brep.body.as_ref().unwrap()).volume
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-9)
}

fn param(e: &regen::FeatureError, k: &str) -> Option<Json> {
    e.params.iter().find(|(n, _)| n == k).map(|(_, v)| v.clone())
}

fn check_valid(o: &Out, what: &str) {
    let b = o.brep.body.as_ref().unwrap();
    let iss = ck_check::validate(b, 1e-3);
    assert!(iss.is_empty(), "{what}: {:?}", iss.iter().take(4).map(|i| format!("{} {}", i.code, i.detail)).collect::<Vec<_>>());
}

fn names(o: &Out) -> Vec<String> {
    let b = o.brep.body.as_ref().unwrap();
    b.face_ids().map(|f| b.face_id_str(f)).collect()
}

/// The prop probe case's part document, its `thicken` feature edited by `edit`.
fn case_part_edit(id: &str, edit: impl FnOnce(&mut Json)) -> String {
    let p = format!("{}/../../tools/cases/prop_probe/{id}.json", env!("CARGO_MANIFEST_DIR"));
    let mut j = ck_json::parse(&std::fs::read_to_string(&p).unwrap()).unwrap();
    let t = find_thicken(&mut j).expect("a thicken feature");
    edit(t);
    format!(
        r#"{{"format":"ck-part","version":1,"features":{}}}"#,
        ck_json::to_string(j.get("features").expect("features"))
    )
}

fn find_thicken<'a>(j: &'a mut Json) -> Option<&'a mut Json> {
    let hit = matches!(j, Json::Obj(m) if m.iter().any(|(k, v)| k == "type" && v.as_str() == Some("thicken")));
    if hit {
        return Some(j);
    }
    match j {
        Json::Arr(a) => a.iter_mut().find_map(find_thicken),
        Json::Obj(m) => m.iter_mut().find_map(|(_, v)| find_thicken(v)),
        _ => None,
    }
}

/// The case's part document with `thicknessAt: <at>` and no `thickness`.
fn case_part_at(id: &str, at: &str) -> String {
    case_part_edit(id, |t| {
        if let Json::Obj(m) = find_thicken(t).expect("a thicken feature") {
            m.retain(|(k, _)| k != "thickness");
            m.push(("thicknessAt".to_string(), ck_json::parse(at).unwrap()));
        }
    })
}

/// A flat 100 x 50 rectangle sheet on the front plane (world x, y), thickened by `<fields>`.
fn flat_doc(fields: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},
          "points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":100,"y":0}},{{"id":"p3","x":100,"y":50}},{{"id":"p4","x":0,"y":50}}],
          "entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},
                      {{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],
          "constraints":[]}},
        {{"type":"surfacePlanar","id":"F2","sketch":"S1"}},
        {{"type":"thicken","id":"F3","surface":"F2","side":"front",{fields}}}]}}"#
    )
}

/// A cylinder sheet (r=10, extruded 40 from the front plane), thickened by `<fields>`.
fn tube_doc(side: &str, fields: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
        {{"type":"sketch","id":"S1","plane":{{"kind":"std","id":"front"}},
          "points":[{{"id":"c","x":0,"y":0}}],
          "entities":[{{"type":"circle","id":"e1","c":"c","r":10}}],"constraints":[]}},
        {{"type":"surfaceExtrude","id":"F2","sketch":"S1","end":"blind","depth":40}},
        {{"type":"thicken","id":"F3","surface":"F2","side":"{side}",{fields}}}]}}"#
    )
}

/// A flat sheet under a linear ramp 1..3 across its 100 mm: volume = area x mean thickness,
/// the trapezoid of the ramp: 100 * 50 * 2 = 10000 exactly.
#[test]
fn flat_sheet_ramp_volume_is_the_integral() {
    let o = run(&flat_doc(r#""thicknessAt":{"along":[1,0,0],"stops":[[0,1],[100,3]]}"#));
    assert!(o.errors.is_empty(), "{:?}", o.errors.iter().map(|e| (&e.feature, &e.error.code)).collect::<Vec<_>>());
    check_valid(&o, "flat ramp");
    let v = vol(&o);
    let want = 10000.0;
    eprintln!("flat ramp volume {v:.6} want {want}");
    assert!(rel(v, want) <= 1e-3, "volume {v} want {want} (rel {})", rel(v, want));
}

/// The cap7 blade sheet, root 4 tip 1 along z: the field averages 2.5 over the sheet (linear in
/// z, sections are pure translates), so against the constant t=2 body the volume ratio is 1.25.
#[test]
fn blade_ramp_root_4_tip_1_is_the_integral() {
    let r = run(&case_part_at(
        "cap7_thicken_open_loft",
        r#"{"along":[0,0,1],"stops":[[0,4],[48,1]]}"#,
    ));
    assert!(
        r.errors.is_empty(),
        "{:?}",
        r.errors
            .iter()
            .map(|e| (&e.feature, &e.error.code, param(&e.error, "detail")))
            .collect::<Vec<_>>()
    );
    check_valid(&r, "blade ramp");
    let c = run(&case_part_edit("cap7_thicken_open_loft", |_| {}));
    assert!(c.errors.is_empty(), "{:?}", c.errors.iter().map(|e| (&e.feature, &e.error.code)).collect::<Vec<_>>());
    check_valid(&c, "blade constant");
    let (a, b) = (vol(&r), vol(&c));
    eprintln!("blade ramp {a:.3} constant t=2 {b:.3} ratio {:.4}", a / b);
    assert!(rel(a / b, 1.25) <= 1e-3, "ramp/constant ratio {} want 1.25 (volumes {a} / {b})", a / b);
}

/// A constant stop list routes to the plain `thicken` path: the body and its face names are the
/// plain `thickness: 2` body's, bit for bit (naming of faces unchanged for constant thickness).
#[test]
fn constant_stop_list_is_the_constant_body() {
    let a = run(&case_part_at(
        "cap7_thicken_open_loft",
        r#"{"along":[0,0,1],"stops":[[0,2],[48,2]]}"#,
    ));
    let b = run(&case_part_edit("cap7_thicken_open_loft", |_| {}));
    assert!(a.errors.is_empty() && b.errors.is_empty(), "{:?} {:?}", a.errors, b.errors);
    check_valid(&a, "constant via thicknessAt");
    check_valid(&b, "plain thickness");
    assert_eq!(vol(&a), vol(&b), "bit-identical volume");
    assert_eq!(names(&a), names(&b), "identical face names");
}

/// Stops that do not reach the end of the sheet are refused, with the span they cover, the span
/// the sheet has, and a hint to add stops at the ends.
#[test]
fn stops_must_cover_the_sheet() {
    let o = run(&flat_doc(r#""thicknessAt":{"along":[1,0,0],"stops":[[0,1],[50,3]]}"#));
    assert_eq!(o.errors.len(), 1, "{:?}", o.errors);
    let e = &o.errors[0].error;
    assert_eq!(e.code, "invalidValue");
    let d = param(e, "detail").expect("detail");
    let d = d.as_str().unwrap_or_default();
    assert!(d.contains("cover") && d.contains("100"), "{d}");
    assert!(param(e, "hint").is_some(), "hint on {d}");
}

/// A zero or negative stop thickness is refused with a hint, on the wire.
#[test]
fn zero_or_negative_stop_thickness_is_refused() {
    for stops in [r#"[[0,0],[100,3]]"#, r#"[[0,1],[100,-1]]"#] {
        let o = run(&flat_doc(&format!(r#""thicknessAt":{{"along":[1,0,0],"stops":{stops}}}"#)));
        assert_eq!(o.errors.len(), 1, "{stops}: {:?}", o.errors);
        let e = &o.errors[0].error;
        assert_eq!(e.code, "invalidValue", "{stops}");
        assert!(param(e, "detail").is_some(), "{stops}: detail");
        assert!(param(e, "hint").is_some(), "{stops}: hint");
    }
}

/// Thickness larger than the sheet's local curvature allows: whichever `side` the plain
/// `thickness: 50` refuses (the extrude's natural normal decides), the ramp reaching 50 there
/// refuses the same way — `offsetTooLarge` with `minRadius`, `maxThickness` and a hint.
#[test]
fn ramp_beyond_local_curvature_is_refused_with_a_hint() {
    let mut refusals = 0;
    for side in ["front", "back"] {
        let c = run(&tube_doc(side, r#""thickness":50"#));
        let r = run(&tube_doc(side, r#""thicknessAt":{"along":[0,0,1],"stops":[[-40,1],[40,50]]}"#));
        match c.errors.first() {
            Some(ce) => {
                assert_eq!(ce.error.code, "offsetTooLarge", "{side}: {:?}", ce.error);
                let re = r.errors.first().unwrap_or_else(|| panic!("{side}: the ramp must refuse where 50 does"));
                assert_eq!(re.error.code, "offsetTooLarge", "{side}");
                let mr = param(&re.error, "minRadius").expect("minRadius");
                assert!(mr.as_f64().unwrap_or(0.0) > 0.0, "{side}: minRadius {mr}");
                let mt = param(&re.error, "maxThickness").expect("maxThickness");
                assert!(mt.as_f64().unwrap_or(0.0) > 0.0, "{side}: maxThickness {mt}");
                assert!(param(&re.error, "hint").is_some(), "{side}: hint");
                refusals += 1;
            }
            None => {
                assert!(r.errors.is_empty(), "{side}: the plain thickness works, the ramp must too: {:?}", r.errors);
            }
        }
    }
    assert!(refusals >= 1, "the ramp refuses on a side (the constant 50 anchors which)");
}
