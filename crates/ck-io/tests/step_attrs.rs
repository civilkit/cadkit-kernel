//! Tests for STEP AP214/AP242 colors, body/product names, and assembly structure
//! (NEXT_ASSEMBLY_USAGE_OCCURRENCE + ITEM_DEFINED_TRANSFORMATION + CONTEXT_DEPENDENT_SHAPE_REPRESENTATION).

use ck_core::xform::Xform;
use ck_geom::{Frame, Vec2, Vec3};
use ck_io::step::{read_step, write_step, write_step_with_meta, StepExportMeta, StepOpts};
use ck_topo::body::Body;
use ck_topo::make::{extrude, ExtrudeOpts, Profile, ProfileLoop, ProfileRegion, ProfileSeg};

fn make_box(feature: &str, x0: f64, y0: f64, dx: f64, dy: f64, dz: f64) -> Body {
    let pts = [
        Vec2::new(x0, y0),
        Vec2::new(x0 + dx, y0),
        Vec2::new(x0 + dx, y0 + dy),
        Vec2::new(x0, y0 + dy),
    ];
    let outer = ProfileLoop::new(
        (0..4)
            .map(|i| ProfileSeg::line(["e1", "e2", "e3", "e4"][i], pts[i], pts[(i + 1) % 4]))
            .collect(),
    );
    let profile = Profile::new(Frame::identity(), vec![ProfileRegion { outer, holes: vec![] }]);
    extrude(feature, &profile, ExtrudeOpts::blind(dz)).unwrap()
}

fn lump_bbox(body: &Body, lump: ck_topo::body::LumpId) -> (Vec3, Vec3) {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for f in ck_topo::mass::lump_faces(body, lump) {
        for &lp in &body.face(f).loops {
            for &ce in &body.lp(lp).coedges {
                let p = body.vertex(body.coedge_start(ce)).p;
                lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
                hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
            }
        }
    }
    (lo, hi)
}

#[test]
fn roundtrip_named_colored_multibody_and_face_override() {
    let mut body = make_box("F1", 0.0, 0.0, 20.0, 10.0, 5.0); // vol 1000
    let b2 = make_box("F2", 40.0, 0.0, 10.0, 10.0, 8.0); // vol 800
    let new_lumps = body.append(&b2);
    let l1 = body.lump_ids().next().unwrap();
    let l2 = new_lumps[0];
    body.lumps[l1.idx()].name = Some("BasePlate".to_string());
    body.lumps[l2.idx()].name = Some("TopCap".to_string());

    let red = [0.85, 0.15, 0.10];
    let blue = [0.10, 0.45, 0.90];
    let green = [0.20, 0.80, 0.25];
    let face0 = ck_topo::mass::lump_faces(&body, l1)[0];

    let mut meta = StepExportMeta::default();
    meta.body_colors.insert(l1, red);
    meta.body_colors.insert(l2, blue);
    meta.face_colors.insert(face0, green);

    let step = write_step_with_meta(&body, "assembly_part", &StepOpts { unix_ms: 0.0 }, &meta, None);
    assert!(step.contains("MANIFOLD_SOLID_BREP('BasePlate',"), "{step}");
    assert!(step.contains("MANIFOLD_SOLID_BREP('TopCap',"), "{step}");
    assert!(step.contains("COLOUR_RGB("));
    assert!(step.contains("FILL_AREA_STYLE_COLOUR("));
    assert!(step.contains("FILL_AREA_STYLE("));
    assert!(step.contains("SURFACE_STYLE_FILL_AREA("));
    assert!(step.contains("SURFACE_SIDE_STYLE("));
    assert!(step.contains("SURFACE_STYLE_USAGE("));
    assert!(step.contains("PRESENTATION_STYLE_ASSIGNMENT("));
    assert!(step.contains("STYLED_ITEM("));
    assert!(step.contains("MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION("));

    let imp = read_step(&step).expect("read_step");
    assert_eq!(imp.solids, 2);
    assert_eq!(imp.parts.len(), 2);
    assert_eq!(imp.body.num_lumps(), 2);

    assert_eq!(imp.parts[0].name, "BasePlate");
    assert_eq!(imp.parts[1].name, "TopCap");
    let lump_names: Vec<Option<String>> = imp.body.lump_ids().map(|l| imp.body.lump(l).name.clone()).collect();
    assert_eq!(lump_names, vec![Some("BasePlate".into()), Some("TopCap".into())]);

    let c0 = imp.parts[0].color.expect("BasePlate color");
    let c1 = imp.parts[1].color.expect("TopCap color");
    for i in 0..3 {
        assert!((c0[i] - red[i]).abs() < 1e-9, "c0={c0:?}");
        assert!((c1[i] - blue[i]).abs() < 1e-9, "c1={c1:?}");
    }
    assert_eq!(imp.parts[0].face_colors.len(), 1);
    let fc = *imp.parts[0].face_colors.values().next().unwrap();
    for i in 0..3 {
        assert!((fc[i] - green[i]).abs() < 1e-9, "fc={fc:?}");
    }
    assert!(imp.parts[1].face_colors.is_empty());

    let mp = ck_topo::mass::mass_props(&imp.body);
    assert!((mp.volume - 1800.0).abs() < 1e-6, "volume={}", mp.volume);
    let lump_vols: Vec<f64> = ck_topo::mass::lump_moments(&imp.body).iter().map(|m| m.volume).collect();
    assert!((lump_vols[0] - 1000.0).abs() < 1e-6);
    assert!((lump_vols[1] - 800.0).abs() < 1e-6);
    assert!(ck_check::validate(&imp.body, 1e-6).is_empty());
}

#[test]
fn product_name_fallback_and_predefined_color() {
    let body = make_box("F1", 0.0, 0.0, 10.0, 10.0, 10.0);
    let base = write_step(&body, "Bracket_42", &StepOpts { unix_ms: 0.0 });
    // Clear the MANIFOLD_SOLID_BREP name so PRODUCT('Bracket_42',...) must supply it,
    // and attach a DRAUGHTING_PRE_DEFINED_COLOUR('cyan') style to the solid.
    let solid_line = base.lines().find(|l| l.contains("MANIFOLD_SOLID_BREP(")).unwrap();
    let solid_id: u32 = solid_line.strip_prefix('#').unwrap().split('=').next().unwrap().parse().unwrap();
    let mut step = base.replace("MANIFOLD_SOLID_BREP('Bracket_42',", "MANIFOLD_SOLID_BREP('',");
    let style_block = format!(
        "#9001=DRAUGHTING_PRE_DEFINED_COLOUR('cyan');\n\
         #9002=FILL_AREA_STYLE_COLOUR('',#9001);\n\
         #9003=FILL_AREA_STYLE('',(#9002));\n\
         #9004=SURFACE_STYLE_FILL_AREA(#9003);\n\
         #9005=SURFACE_SIDE_STYLE('',(#9004));\n\
         #9006=SURFACE_STYLE_USAGE(.BOTH.,#9005);\n\
         #9007=PRESENTATION_STYLE_ASSIGNMENT((#9006));\n\
         #9008=STYLED_ITEM('',(#9007),#{solid_id});\n\
         #9009=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(#9008),#14);\n\
         ENDSEC;\n"
    );
    step = step.replacen("ENDSEC;\nEND-ISO-10303-21;", &format!("{style_block}END-ISO-10303-21;"), 1);

    let imp = read_step(&step).expect("read_step");
    assert_eq!(imp.solids, 1);
    assert_eq!(imp.parts[0].name, "Bracket_42");
    let l0 = imp.body.lump_ids().next().unwrap();
    assert_eq!(imp.body.lump(l0).name.as_deref(), Some("Bracket_42"));
    assert_eq!(imp.parts[0].color, Some([0.0, 1.0, 1.0]));
}

/// Build a self-contained AP214 assembly STEP text reusing a 10x10x10 cube solid
/// in two placed occurrences (`Cube_Origin` and `Cube_Moved`), plus an optional
/// nested subassembly level.
fn build_assembly_step(nested: bool) -> String {
    let cube = make_box("F1", 0.0, 0.0, 10.0, 10.0, 10.0);
    let base = write_step(&cube, "UnitCube", &StepOpts { unix_ms: 0.0 });
    // Extract the DATA lines from `base` (entities #1..#N), find the ADVANCED_BREP_SHAPE_REPRESENTATION
    // and MANIFOLD_SOLID_BREP ids.
    let mut absr_id = 0u32;
    let mut solid_id = 0u32;
    for l in base.lines() {
        if l.contains("=ADVANCED_BREP_SHAPE_REPRESENTATION(") {
            absr_id = l.strip_prefix('#').unwrap().split('=').next().unwrap().parse().unwrap();
        }
        if l.contains("=MANIFOLD_SOLID_BREP(") {
            solid_id = l.strip_prefix('#').unwrap().split('=').next().unwrap().parse().unwrap();
        }
    }
    assert!(absr_id > 0 && solid_id > 0);
    // In `write_step`: #1=APP_CTX, #3=PROD_CTX, #7=PD_CTX, #8=PD(UnitCube), #14=GEOM_CTX, #15=world AXIS2_PLACEMENT_3D.
    let mut extra = format!(
        "#5001=COLOUR_RGB('',0.9,0.5,0.1);\n\
         #5002=FILL_AREA_STYLE_COLOUR('',#5001);\n\
         #5003=FILL_AREA_STYLE('',(#5002));\n\
         #5004=SURFACE_STYLE_FILL_AREA(#5003);\n\
         #5005=SURFACE_SIDE_STYLE('',(#5004));\n\
         #5006=SURFACE_STYLE_USAGE(.BOTH.,#5005);\n\
         #5007=PRESENTATION_STYLE_ASSIGNMENT((#5006));\n\
         #5008=STYLED_ITEM('',(#5007),#{solid_id});\n\
         #5009=MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION('',(#5008),#14);\n"
    );

    if !nested {
        // Root assembly (#6004 = PD, #6006 = SHAPE_REPRESENTATION) with 2 occurrences of #8 (UnitCube)
        extra.push_str(&format!(
            "#6001=PRODUCT('RootAssy','RootAssy','',(#3));\n\
             #6002=PRODUCT_DEFINITION_FORMATION('','',#6001);\n\
             #6003=PRODUCT_DEFINITION('design','',#6002,#7);\n\
             #6004=PRODUCT_DEFINITION_SHAPE('','',#6003);\n\
             #6005=CARTESIAN_POINT('',(0.,0.,0.));\n\
             #6006=DIRECTION('',(0.,0.,1.));\n\
             #6007=DIRECTION('',(1.,0.,0.));\n\
             #6008=AXIS2_PLACEMENT_3D('origin',#6005,#6006,#6007);\n\
             #6009=SHAPE_REPRESENTATION('RootAssy',(#6008),#14);\n\
             #6010=SHAPE_DEFINITION_REPRESENTATION(#6004,#6009);\n\
             #6020=NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','Cube_Origin','',#6003,#8,$);\n\
             #6021=PRODUCT_DEFINITION_SHAPE('','',#6020);\n\
             #6022=ITEM_DEFINED_TRANSFORMATION('','',#6008,#6008);\n\
             #6023=(REPRESENTATION_RELATIONSHIP('','',#{absr_id},#6009)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#6022)SHAPE_REPRESENTATION_RELATIONSHIP());\n\
             #6024=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#6023,#6021);\n\
             #6030=CARTESIAN_POINT('',(50.,20.,10.));\n\
             #6031=DIRECTION('',(0.,0.,1.));\n\
             #6032=DIRECTION('',(0.,1.,0.));\n\
             #6033=AXIS2_PLACEMENT_3D('moved',#6030,#6031,#6032);\n\
             #6034=NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','Cube_Moved','',#6003,#8,$);\n\
             #6035=PRODUCT_DEFINITION_SHAPE('','',#6034);\n\
             #6036=ITEM_DEFINED_TRANSFORMATION('','',#6008,#6033);\n\
             #6037=(REPRESENTATION_RELATIONSHIP('','',#{absr_id},#6009)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#6036)SHAPE_REPRESENTATION_RELATIONSHIP());\n\
             #6038=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#6037,#6035);\n"
        ));
    } else {
        // Two-level assembly: TopAssy (#7003) -> SubAssy (#6003, shifted +Y 100) -> UnitCube (#8, shifted +X 25)
        extra.push_str(&format!(
            "#6001=PRODUCT('SubAssy','SubAssy','',(#3));\n\
             #6002=PRODUCT_DEFINITION_FORMATION('','',#6001);\n\
             #6003=PRODUCT_DEFINITION('design','',#6002,#7);\n\
             #6004=PRODUCT_DEFINITION_SHAPE('','',#6003);\n\
             #6005=CARTESIAN_POINT('',(0.,0.,0.));\n\
             #6006=DIRECTION('',(0.,0.,1.));\n\
             #6007=DIRECTION('',(1.,0.,0.));\n\
             #6008=AXIS2_PLACEMENT_3D('origin',#6005,#6006,#6007);\n\
             #6009=SHAPE_REPRESENTATION('SubAssy',(#6008),#14);\n\
             #6010=SHAPE_DEFINITION_REPRESENTATION(#6004,#6009);\n\
             #6011=CARTESIAN_POINT('',(25.,0.,0.));\n\
             #6012=AXIS2_PLACEMENT_3D('sub_offset',#6011,#6006,#6007);\n\
             #6020=NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','Block_A','',#6003,#8,$);\n\
             #6021=PRODUCT_DEFINITION_SHAPE('','',#6020);\n\
             #6022=ITEM_DEFINED_TRANSFORMATION('','',#6008,#6012);\n\
             #6023=(REPRESENTATION_RELATIONSHIP('','',#{absr_id},#6009)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#6022)SHAPE_REPRESENTATION_RELATIONSHIP());\n\
             #6024=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#6023,#6021);\n\
             #7001=PRODUCT('TopAssy','TopAssy','',(#3));\n\
             #7002=PRODUCT_DEFINITION_FORMATION('','',#7001);\n\
             #7003=PRODUCT_DEFINITION('design','',#7002,#7);\n\
             #7004=PRODUCT_DEFINITION_SHAPE('','',#7003);\n\
             #7005=SHAPE_REPRESENTATION('TopAssy',(#6008),#14);\n\
             #7006=SHAPE_DEFINITION_REPRESENTATION(#7004,#7005);\n\
             #7010=CARTESIAN_POINT('',(0.,100.,0.));\n\
             #7011=AXIS2_PLACEMENT_3D('top_offset',#7010,#6006,#6007);\n\
             #7020=NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','Sub_1','',#7003,#6003,$);\n\
             #7021=PRODUCT_DEFINITION_SHAPE('','',#7020);\n\
             #7022=ITEM_DEFINED_TRANSFORMATION('','',#6008,#7011);\n\
             #7023=(REPRESENTATION_RELATIONSHIP('','',#6009,#7005)REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#7022)SHAPE_REPRESENTATION_RELATIONSHIP());\n\
             #7024=CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#7023,#7021);\n"
        ));
    }

    base.replacen("ENDSEC;\nEND-ISO-10303-21;", &format!("{extra}ENDSEC;\nEND-ISO-10303-21;"), 1)
}

#[test]
fn assembly_nauo_places_instanced_occurrences_with_names_and_colors() {
    let step = build_assembly_step(false);
    let imp = read_step(&step).expect("read assembly step");
    assert_eq!(imp.solids, 2);
    assert_eq!(imp.parts.len(), 2);
    assert_eq!(imp.body.num_lumps(), 2);

    assert_eq!(imp.parts[0].name, "Cube_Origin");
    assert_eq!(imp.parts[1].name, "Cube_Moved");

    let expected_color = [0.9, 0.5, 0.1];
    for p in &imp.parts {
        let c = p.color.expect("inherited solid color");
        for i in 0..3 {
            assert!((c[i] - expected_color[i]).abs() < 1e-9);
        }
    }

    let lumps: Vec<_> = imp.body.lump_ids().collect();
    assert_eq!(imp.body.lump(lumps[0]).name.as_deref(), Some("Cube_Origin"));
    assert_eq!(imp.body.lump(lumps[1]).name.as_deref(), Some("Cube_Moved"));

    // Occurrence 0: [0..10, 0..10, 0..10]
    let (lo0, hi0) = lump_bbox(&imp.body, lumps[0]);
    assert!((lo0 - Vec3::new(0.0, 0.0, 0.0)).len() < 1e-6, "lo0={lo0:?}");
    assert!((hi0 - Vec3::new(10.0, 10.0, 10.0)).len() < 1e-6, "hi0={hi0:?}");

    // Occurrence 1: rotated +90 deg about Z then translated to (50, 20, 10) -> [40..50, 20..30, 10..20]
    let (lo1, hi1) = lump_bbox(&imp.body, lumps[1]);
    assert!((lo1 - Vec3::new(40.0, 20.0, 10.0)).len() < 1e-6, "lo1={lo1:?}");
    assert!((hi1 - Vec3::new(50.0, 30.0, 20.0)).len() < 1e-6, "hi1={hi1:?}");

    let mp = ck_topo::mass::mass_props(&imp.body);
    assert!((mp.volume - 2000.0).abs() < 1e-6, "volume={}", mp.volume);
    assert!(ck_check::validate(&imp.body, 1e-6).is_empty());
}

#[test]
fn nested_subassembly_composes_transforms() {
    let step = build_assembly_step(true);
    let imp = read_step(&step).expect("read nested assembly step");
    assert_eq!(imp.solids, 1);
    assert_eq!(imp.parts.len(), 1);
    assert_eq!(imp.parts[0].name, "Block_A");

    let l0 = imp.body.lump_ids().next().unwrap();
    let (lo, hi) = lump_bbox(&imp.body, l0);
    // Composed translation: (0, 100, 0) + (25, 0, 0) = (25, 100, 0)
    assert!((lo - Vec3::new(25.0, 100.0, 0.0)).len() < 1e-6, "lo={lo:?}");
    assert!((hi - Vec3::new(35.0, 110.0, 10.0)).len() < 1e-6, "hi={hi:?}");

    let mp = ck_topo::mass::mass_props(&imp.body);
    assert!((mp.volume - 1000.0).abs() < 1e-6);
    assert!(ck_check::validate(&imp.body, 1e-6).is_empty());
}

#[test]
fn plain_write_step_emits_no_style_entities() {
    let mut b = make_box("F1", 0.0, 0.0, 5.0, 5.0, 5.0);
    b.transform(&Xform::IDENTITY);
    let step = write_step(&b, "plain", &StepOpts { unix_ms: 0.0 });
    assert!(!step.contains("STYLED_ITEM"));
    assert!(!step.contains("COLOUR_RGB"));
}
