//! First-class Mate Connectors (`type: "connector"`) and connector-based
//! assembly joints (`fastened`, `revolute`, `slider`, `cylindrical`, `planar`).

use ck_core::linalg::{Frame, Vec3};
use ck_json::Json;
use ck_model::features::Brep;
use ck_model::regen::{self, FeatureExec, RegenOpts, RegenResult, Tracer};
use ck_model::Part;

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
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<Frame> {
        self.brep.face_frame(s, face)
    }
    fn face_connector_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<Frame> {
        self.brep.face_connector_frame(s, face)
    }
    fn edge_line(&mut self, s: &regen::RegenState, faces: &[String], at: Option<[f64; 3]>) -> Option<(Vec3, Vec3)> {
        self.brep.edge_line(s, faces, at)
    }
    fn edge_seg(&mut self, s: &regen::RegenState, faces: &[String], at: Option<Vec3>) -> Option<ck_ops::sweep::PathSeg> {
        self.brep.edge_seg(s, faces, at)
    }
    fn vertex_point(&mut self, s: &regen::RegenState, faces: &[String], at: Option<Vec3>) -> Option<Vec3> {
        self.brep.vertex_point(s, faces, at)
    }
    fn face_surf(&mut self, s: &regen::RegenState, face: &str) -> Option<regen::FaceSurf> {
        self.brep.face_surf(s, face)
    }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> {
        self.brep.exec(s, f)
    }
}

fn run_part(json: &str) -> (RegenResult, Brep) {
    let part = Part::parse(json).expect("valid part json");
    let mut ex = Exec { brep: Brep::new(), labels: Noop };
    let res = regen::regen(&part, &RegenOpts::default(), &mut ex);
    (res, ex.brep)
}

fn assert_vec_near(a: Vec3, b: Vec3, tol: f64, label: &str) {
    let d = a.dist(b);
    assert!(d <= tol, "{label}: expected {b:?}, got {a:?} (dist={d})");
}

fn box_part_with_connector(w: f64, d: f64, h: f64, conn_json: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"p1","x":0,"y":0}},{{"id":"p2","x":{w},"y":0}},{{"id":"p3","x":{w},"y":{d}}},{{"id":"p4","x":0,"y":{d}}}],
"entities":[{{"type":"line","id":"e1","p1":"p1","p2":"p2"}},{{"type":"line","id":"e2","p1":"p2","p2":"p3"}},{{"type":"line","id":"e3","p1":"p3","p2":"p4"}},{{"type":"line","id":"e4","p1":"p4","p2":"p1"}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}},
{conn_json}
]}}"#
    )
}

fn cyl_part_with_connector(r: f64, h: f64, conn_json: &str) -> String {
    format!(
        r#"{{"format":"ck-part","version":1,"features":[
{{"type":"sketch","id":"F1","plane":{{"kind":"std","id":"top"}},"points":[{{"id":"c","x":0,"y":0}}],"entities":[{{"type":"circle","id":"e1","c":"c","r":{r}}}],"constraints":[]}},
{{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":{h}}},
{conn_json}
]}}"#
    )
}

#[test]
fn connector_anchors_and_wire_format() {
    // 1. Planar face anchor: top cap of 20x10x30 box (sketch on top plane -> Y up, depth 30 -> top cap at y=30, x in [0,20], z in [-10,0])
    let part = box_part_with_connector(
        20.0,
        10.0,
        30.0,
        r#"{"type":"connector","id":"C1","name":"TopCenter","anchor":{"kind":"face","id":"F2.C1"}}"#,
    );
    let (res, _brep) = run_part(&part);
    assert!(res.errors.is_empty(), "errors: {:?}", res.errors);
    assert_eq!(res.connectors.len(), 1);
    let (id, name, fr) = &res.connectors[0];
    assert_eq!(id, "C1");
    assert_eq!(name, "TopCenter");
    // Top cap centroid is (10, 30, -5), outward normal is +Y
    assert_vec_near(fr.o, Vec3::new(10.0, 30.0, -5.0), 1e-6, "planar face centroid");
    assert_vec_near(fr.z, Vec3::new(0.0, 1.0, 0.0), 1e-6, "planar face normal");
    assert!((fr.x.cross(fr.y).dot(fr.z) - 1.0).abs() < 1e-9, "right-handed frame");

    // Check wire JSON format
    let j = res.to_json();
    let conns = j.get("connectors").and_then(|v| v.as_arr()).expect("connectors array on wire");
    assert_eq!(conns.len(), 1);
    let entry = conns[0].as_arr().unwrap();
    assert_eq!(entry[0].as_str(), Some("C1"));
    assert_eq!(entry[1].get_str("name"), Some("TopCenter"));
    assert!(entry[1].get("o").is_some());
    assert!(entry[1].get("x").is_some());
    assert!(entry[1].get("y").is_some());
    assert!(entry[1].get("z").is_some());

    // 2. Part WITHOUT connectors must NOT serialize "connectors" key (keeps oracle goldens byte-identical)
    let plain_part = r#"{"format":"ck-part","version":1,"features":[
{"type":"sketch","id":"F1","plane":{"kind":"std","id":"top"},"points":[{"id":"p1","x":0,"y":0},{"id":"p2","x":10,"y":0},{"id":"p3","x":10,"y":10},{"id":"p4","x":0,"y":10}],
"entities":[{"type":"line","id":"e1","p1":"p1","p2":"p2"},{"type":"line","id":"e2","p1":"p2","p2":"p3"},{"type":"line","id":"e3","p1":"p3","p2":"p4"},{"type":"line","id":"e4","p1":"p4","p2":"p1"}],"constraints":[]},
{"type":"extrude","id":"F2","op":"add","sketch":"F1","end":"blind","depth":10}]}"#;
    let (plain_res, _) = run_part(plain_part);
    assert!(plain_res.to_json().get("connectors").is_none(), "empty connectors must be omitted from wire");

    // 3. Cylindrical face anchor: cylinder of r=5, h=20 on top plane (axis along Y, center at (0, 10, 0))
    let cyl = cyl_part_with_connector(
        5.0,
        20.0,
        r#"{"type":"connector","id":"C_CYL","name":"ShaftAxis","anchor":{"kind":"face","id":"F2.S.e1"}}"#,
    );
    let (cyl_res, _) = run_part(&cyl);
    assert!(cyl_res.errors.is_empty(), "cyl errors: {:?}", cyl_res.errors);
    let (_, _, cfr) = &cyl_res.connectors[0];
    assert_vec_near(cfr.o, Vec3::new(0.0, 10.0, 0.0), 1e-6, "cyl face center");
    assert!((cfr.z.y.abs() - 1.0).abs() < 1e-6, "cyl axis along Y: {:?}", cfr.z);

    // 4. Std plane + explicit frame anchors
    let planes_part = r#"{"format":"ck-part","version":1,"features":[
{"type":"connector","id":"C_STD","name":"FrontPlane","anchor":{"kind":"std","id":"front"}},
{"type":"connector","id":"C_FRAME","name":"Custom","anchor":{"kind":"frame","origin":[1,2,3],"xDir":[0,1,0],"yDir":[0,0,1]}}
]}"#;
    let (p_res, _) = run_part(planes_part);
    assert!(p_res.errors.is_empty(), "errors: {:?}", p_res.errors);
    assert_eq!(p_res.connectors.len(), 2);
    let f_std = p_res.connectors.iter().find(|c| c.0 == "C_STD").unwrap().2;
    assert_vec_near(f_std.o, Vec3::ZERO, 1e-9, "std front origin");
    assert_vec_near(f_std.z, Vec3::Z, 1e-9, "std front normal");
    let f_custom = p_res.connectors.iter().find(|c| c.0 == "C_FRAME").unwrap().2;
    assert_vec_near(f_custom.o, Vec3::new(1.0, 2.0, 3.0), 1e-9, "frame origin");
    assert_vec_near(f_custom.x, Vec3::Y, 1e-9, "frame x");
    assert_vec_near(f_custom.y, Vec3::Z, 1e-9, "frame y");
    assert_vec_near(f_custom.z, Vec3::X, 1e-9, "frame z = x cross y");
}

#[test]
fn connector_modifiers_and_frozen_frame_survival() {
    // Test offset + flip + rotateZ on std front plane (o=0, x=X, y=Y, z=Z)
    let part = r#"{"format":"ck-part","version":1,"features":[
{"type":"connector","id":"C1","name":"Modified","anchor":{"kind":"std","id":"front"},"offset":[5,10,15],"flip":true,"rotateZ":90}
]}"#;
    let (res, _) = run_part(part);
    assert!(res.errors.is_empty(), "errors: {:?}", res.errors);
    let fr = res.connectors[0].2;
    // offset -> (5, 10, 15); flip -> x=-X, y=Y, z=-Z; rotateZ(+90 about z=-Z) -> x=+Y, y=+X, z=-Z
    assert_vec_near(fr.o, Vec3::new(5.0, 10.0, 15.0), 1e-9, "offset origin");
    assert_vec_near(fr.z, Vec3::new(0.0, 0.0, -1.0), 1e-9, "flipped z");
    assert_vec_near(fr.x, Vec3::new(0.0, 1.0, 0.0), 1e-9, "flipped + rotZ 90 x");
    assert_vec_near(fr.y, Vec3::new(1.0, 0.0, 0.0), 1e-9, "flipped + rotZ 90 y");

    // Test frozenFrame survival when referenced face disappears
    let broken = r#"{"format":"ck-part","version":1,"features":[
{"type":"connector","id":"C_FROZEN","name":"Survived","anchor":{"kind":"face","id":"F99.C1"},
 "frozenFrame":{"o":[7,8,9],"x":[1,0,0],"y":[0,1,0],"z":[0,0,1]}}
]}"#;
    let (res2, _) = run_part(broken);
    assert!(res2.errors.is_empty(), "frozen connector should not error: {:?}", res2.errors);
    assert_eq!(res2.warnings.len(), 1, "expected detachedReference warning");
    assert_eq!(res2.warnings[0].error.code, regen::codes::DETACHED_REFERENCE);
    assert_eq!(res2.connectors.len(), 1);
    assert_vec_near(res2.connectors[0].2.o, Vec3::new(7.0, 8.0, 9.0), 1e-9, "frozen origin");
}

fn assembly_run(features: &[String]) -> (RegenResult, Brep, Option<Json>) {
    let text = format!(r#"{{"format":"ck-part","version":1,"features":[{}]}}"#, features.join(","));
    let (res, mut brep) = run_part(&text);
    let report = brep.finalize_assembly(false);
    (res, brep, report)
}

fn world_conn(brep: &Brep, comp_id: &str, conn_id: &str) -> Frame {
    let c = brep.asm.components.iter().find(|c| c.id == comp_id).expect("component");
    let (_, _, local) = c.connectors.iter().find(|x| x.0 == conn_id).expect("connector");
    c.place.frame(local)
}

fn comp_dof(report: &Option<Json>, comp_id: &str) -> (usize, usize, usize) {
    let cs = report.as_ref().unwrap().get("components").unwrap().as_arr().unwrap();
    let c = cs.iter().find(|c| c.get_str("id") == Some(comp_id)).unwrap();
    (
        c.get_f64("dof").unwrap() as usize,
        c.get_f64("translational").unwrap() as usize,
        c.get_f64("rotational").unwrap() as usize,
    )
}

#[test]
fn assembly_fastened_connector_mate() {
    let base_part = box_part_with_connector(
        20.0,
        20.0,
        10.0,
        r#"{"type":"connector","id":"C_TOP","name":"Top","anchor":{"kind":"face","id":"F2.C1"}}"#,
    );
    let block_part = box_part_with_connector(
        10.0,
        10.0,
        10.0,
        r#"{"type":"connector","id":"C_BOT","name":"Bottom","anchor":{"kind":"face","id":"F2.C0"}}"#,
    );

    // 1. Basic fastened with flip: true (C_BOT normal -Y opposes C_TOP normal +Y -> block sits right on top of base)
    let feats = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"Base","part":{base_part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"Block","part":{block_part},"dx":50,"dy":40,"dz":30}}"#),
        r#"{"type":"mate","id":"M1","kind":"fastened","a":{"kind":"connector","id":"C_TOP","component":"I1"},"b":{"kind":"connector","id":"C_BOT","component":"I2"},"flip":true}"#.to_string(),
    ];
    let (res, brep, report) = assembly_run(&feats);
    assert!(res.errors.is_empty(), "errors: {:?}", res.errors);
    let f1 = world_conn(&brep, "I1", "C_TOP");
    let f2 = world_conn(&brep, "I2", "C_BOT");
    assert_vec_near(f2.o, f1.o, 1e-6, "fastened origin");
    assert_vec_near(f2.z, -f1.z, 1e-6, "fastened flipped z");
    assert_vec_near(f2.x, -f1.x, 1e-6, "fastened flipped x");
    assert_eq!(comp_dof(&report, "I2"), (0, 0, 0), "fastened must leave 0 DOF");

    // Verify Component::to_json exposes transformed connectors
    let comps = brep.components_json();
    let i2_json = comps.iter().find(|c| c.get_str("id") == Some("I2")).unwrap();
    let i2_conns = i2_json.get("connectors").and_then(|v| v.as_arr()).expect("component connectors on wire");
    assert_eq!(i2_conns.len(), 1);
    assert_eq!(i2_conns[0].as_arr().unwrap()[0].as_str(), Some("C_BOT"));

    // 2. Fastened with offset [2, 3, 5] and rotateZ: 90
    let feats_mod = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"Base","part":{base_part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"Block","part":{block_part},"dx":50,"dy":40,"dz":30}}"#),
        r#"{"type":"mate","id":"M1","kind":"fastened","a":{"kind":"connector","id":"C_TOP","component":"I1"},"b":{"kind":"connector","id":"C_BOT","component":"I2"},"offset":[2,3,5],"rotateZ":90}"#.to_string(),
    ];
    let (res2, brep2, report2) = assembly_run(&feats_mod);
    assert!(res2.errors.is_empty(), "errors: {:?}", res2.errors);
    let f1 = world_conn(&brep2, "I1", "C_TOP");
    let f2 = world_conn(&brep2, "I2", "C_BOT");
    let expected_o = f1.o + f1.x.scale(2.0) + f1.y.scale(3.0) + f1.z.scale(5.0);
    assert_vec_near(f2.o, expected_o, 1e-6, "fastened offset origin");
    assert_vec_near(f2.z, f1.z, 1e-6, "fastened aligned z");
    assert_vec_near(f2.x, f1.y, 1e-6, "fastened rotateZ 90 x");
    assert_eq!(comp_dof(&report2, "I2"), (0, 0, 0));

    // 3. Fastened with rotateZ: 180 (pure antiparallel X singularity check)
    let feats_180 = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"Base","part":{base_part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"Block","part":{base_part}}}"#),
        r#"{"type":"mate","id":"M1","kind":"fastened","a":{"kind":"connector","id":"C_TOP","component":"I1"},"b":{"kind":"connector","id":"C_TOP","component":"I2"},"rotateZ":180}"#.to_string(),
    ];
    let (res3, brep3, _) = assembly_run(&feats_180);
    assert!(res3.errors.is_empty(), "rotateZ 180 errors: {:?}", res3.errors);
    let f1 = world_conn(&brep3, "I1", "C_TOP");
    let f2 = world_conn(&brep3, "I2", "C_TOP");
    assert_vec_near(f2.o, f1.o, 1e-6, "180 origin");
    assert_vec_near(f2.z, f1.z, 1e-6, "180 z");
    assert_vec_near(f2.x, -f1.x, 1e-6, "180 x");
}

#[test]
fn assembly_revolute_slider_cylindrical_planar_connector_mates() {
    let part = box_part_with_connector(
        20.0,
        20.0,
        10.0,
        r#"{"type":"connector","id":"C1","name":"Top","anchor":{"kind":"face","id":"F2.C1"}}"#,
    );

    // 1. Revolute: 1 rotational DOF about Z
    let rev_feats = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part},"dx":30,"dy":20,"dz":10,"axis":{{"kind":"axis","axis":"y"}},"angle":45}}"#),
        r#"{"type":"mate","id":"M1","kind":"revolute","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"}}"#.to_string(),
    ];
    let (res, brep, report) = assembly_run(&rev_feats);
    assert!(res.errors.is_empty(), "revolute errors: {:?}", res.errors);
    let f1 = world_conn(&brep, "I1", "C1");
    let f2 = world_conn(&brep, "I2", "C1");
    assert_vec_near(f2.o, f1.o, 1e-6, "revolute origin");
    assert_vec_near(f2.z, f1.z, 1e-6, "revolute z");
    assert_eq!(comp_dof(&report, "I2"), (1, 0, 1), "revolute must leave 1 rotational DOF");

    // 1b. Revolute with min/max angle limit clamping (started at 45 deg, clamped to [-20, 20])
    let rev_lim = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part},"axis":{{"kind":"axis","axis":"y"}},"angle":45}}"#),
        r#"{"type":"mate","id":"M1","kind":"revolute","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"},"min":-20,"max":20}"#.to_string(),
    ];
    let (res_rl, brep_rl, _) = assembly_run(&rev_lim);
    assert!(res_rl.errors.is_empty(), "revolute limit errors: {:?}", res_rl.errors);
    let f1 = world_conn(&brep_rl, "I1", "C1");
    let f2 = world_conn(&brep_rl, "I2", "C1");
    let ang_deg = f1.z.dot(f1.x.cross(f2.x)).atan2(f1.x.dot(f2.x)).to_degrees();
    assert!((ang_deg.abs() - 20.0).abs() < 1e-5, "expected clamped to ±20 deg, got {ang_deg}");

    // 2. Slider: 1 translational DOF along Z, plus min/max clamping
    let sld_feats = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part},"dx":15,"dy":50,"dz":12}}"#),
        r#"{"type":"mate","id":"M1","kind":"slider","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"},"min":0,"max":25}"#.to_string(),
    ];
    let (res_s, brep_s, report_s) = assembly_run(&sld_feats);
    assert!(res_s.errors.is_empty(), "slider errors: {:?}", res_s.errors);
    let f1 = world_conn(&brep_s, "I1", "C1");
    let f2 = world_conn(&brep_s, "I2", "C1");
    assert_vec_near(f2.z, f1.z, 1e-6, "slider z");
    assert_vec_near(f2.x, f1.x, 1e-6, "slider x locked");
    let diff = f2.o - f1.o;
    assert!(diff.dot(f1.x).abs() < 1e-6 && diff.dot(f1.y).abs() < 1e-6, "slider on Z line");
    assert!((diff.dot(f1.z) - 25.0).abs() < 1e-5, "slider clamped to max=25, got {}", diff.dot(f1.z));
    assert_eq!(comp_dof(&report_s, "I2"), (1, 1, 0), "slider must leave 1 translational DOF");

    // 3. Cylindrical: 2 DOF (1 translational + 1 rotational about Z)
    let cyl_feats = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part},"dx":15,"dy":30,"dz":12}}"#),
        r#"{"type":"mate","id":"M1","kind":"cylindrical","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"}}"#.to_string(),
    ];
    let (res_c, brep_c, report_c) = assembly_run(&cyl_feats);
    assert!(res_c.errors.is_empty(), "cylindrical errors: {:?}", res_c.errors);
    let f1 = world_conn(&brep_c, "I1", "C1");
    let f2 = world_conn(&brep_c, "I2", "C1");
    assert_vec_near(f2.z, f1.z, 1e-6, "cylindrical z");
    let diff = f2.o - f1.o;
    assert!(diff.dot(f1.x).abs() < 1e-6 && diff.dot(f1.y).abs() < 1e-6, "cylindrical on Z line");
    assert_eq!(comp_dof(&report_c, "I2"), (2, 1, 1), "cylindrical must leave 2 DOF (1T, 1R)");

    // 4. Planar: 3 DOF (2 translational + 1 rotational)
    let pln_feats = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part},"dx":15,"dy":30,"dz":12}}"#),
        r#"{"type":"mate","id":"M1","kind":"planar","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"}}"#.to_string(),
    ];
    let (res_p, brep_p, report_p) = assembly_run(&pln_feats);
    assert!(res_p.errors.is_empty(), "planar errors: {:?}", res_p.errors);
    let f1 = world_conn(&brep_p, "I1", "C1");
    let f2 = world_conn(&brep_p, "I2", "C1");
    assert_vec_near(f2.z, f1.z, 1e-6, "planar z");
    assert!((f2.o - f1.o).dot(f1.z).abs() < 1e-6, "planar origins coplanar");
    assert_eq!(comp_dof(&report_p, "I2"), (3, 2, 1), "planar must leave 3 DOF (2T, 1R)");
}

#[test]
fn connector_mate_error_cases() {
    let part = box_part_with_connector(
        10.0,
        10.0,
        10.0,
        r#"{"type":"connector","id":"C1","name":"Top","anchor":{"kind":"face","id":"F2.C1"}}"#,
    );

    // Same body -> mateSameBody
    let same = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part}}}"#),
        r#"{"type":"mate","id":"M1","kind":"fastened","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I1"}}"#.to_string(),
    ];
    let (res, _, _) = assembly_run(&same);
    assert_eq!(res.errors.len(), 1);
    assert_eq!(res.errors[0].error.code, "mateSameBody");

    // Both fixed -> mateFixed
    let both_fixed = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part},"fixed":true}}"#),
        r#"{"type":"mate","id":"M1","kind":"fastened","a":{"kind":"connector","id":"C1","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"}}"#.to_string(),
    ];
    let (res2, _, _) = assembly_run(&both_fixed);
    assert_eq!(res2.errors.len(), 1);
    assert_eq!(res2.errors[0].error.code, "mateFixed");

    // Missing connector -> missingReference
    let missing = vec![
        format!(r#"{{"type":"insertPart","id":"I1","name":"A","part":{part},"fixed":true}}"#),
        format!(r#"{{"type":"insertPart","id":"I2","name":"B","part":{part}}}"#),
        r#"{"type":"mate","id":"M1","kind":"fastened","a":{"kind":"connector","id":"C_NOPE","component":"I1"},"b":{"kind":"connector","id":"C1","component":"I2"}}"#.to_string(),
    ];
    let (res3, _, _) = assembly_run(&missing);
    assert_eq!(res3.errors.len(), 1);
    assert_eq!(res3.errors[0].error.code, regen::codes::MISSING_REFERENCE);
}
