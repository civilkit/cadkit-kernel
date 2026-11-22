//! A face whose display mesh is a closed but WRONG region (silent wrong geometry: the volume is right, `validate` may only
//! name a neighbour as TESSELLATION_OPEN). For every face of the fixtures: the mesh area is within 3 percent of the exact
//! face area and `validate` reports no TESSELLATION_OPEN. Fixtures (tests/fixtures/tess_closed_region):
//! - A: a full sphere read from a STEP record (seam edge only) then split by a boolean: the meridian seam coedges
//!   carried v in [3 pi / 2, 2 pi] beside equator coedges at v = 0 (the loop did not close in uv),
//! - B1: closed B-spline faces after a boolean (loft, coil), the two seam coedges on one isoline: the tessellator chains the loop through the closed direction,
//! - B2: a closed rational dome read from STEP whose face meets the seam at two places (two seam pairs in one loop),
//! - B3: a linear extrusion of a closed spline, straight from import,
//! - ok_control_*: sphere and hemisphere imports that always meshed right.
use ck_json::Json;

fn call(msg: &Json) -> (Json, Vec<u8>) {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg.clone())]));
    let r = ck_server::handle(body.as_bytes());
    let (j, bin) = ck_server::reply::decode(&r).unwrap();
    (ck_json::parse(j).unwrap(), bin.to_vec())
}

fn obj(kv: Vec<(&str, Json)>) -> Json {
    Json::Obj(kv.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

fn result(r: &Json) -> &Json {
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{}", ck_json::to_string(r));
    r.get("result").unwrap()
}

/// Loads the fixture on `tag` and returns the faces whose mesh area is off by more than 3 percent, plus the TESSELLATION_OPEN issues.
fn problems(tag: &str, file: &str, text: &str) -> Vec<String> {
    ck_wasm::engine::install();
    let (r, _) = if file.ends_with(".json") {
        call(&obj(vec![("type", "regen".into()), ("tag", tag.into()), ("part", ck_json::parse(text).unwrap())]))
    } else {
        call(&obj(vec![("type", "importStep".into()), ("tag", tag.into()), ("name", "p".into()), ("data", text.into())]))
    };
    assert_eq!(r.get("ok"), Some(&Json::Bool(true)), "{file}: {}", ck_json::to_string(&r));
    let mut out = vec![];
    let (v, _) = call(&obj(vec![("type", "validate".into()), ("tag", tag.into())]));
    if let Some(issues) = result(&v).get("issues").and_then(|i| i.as_arr()) {
        for i in issues {
            if i.get_str("code") == Some("TESSELLATION_OPEN") {
                out.push(format!("validate TESSELLATION_OPEN {}", i.get_str("entity").unwrap_or("?")));
            }
        }
    }
    let (m, bin) = call(&obj(vec![("type", "exportMesh".into()), ("tag", tag.into())]));
    let res = result(&m);
    let buf = |k: &str| (res.get(k).unwrap().get_f64("offset").unwrap() as usize, res.get(k).unwrap().get_f64("len").unwrap() as usize);
    let (po, pl) = buf("positions");
    let (to, tl) = buf("triFace");
    let pos: Vec<f32> = (0..pl).map(|i| f32::from_le_bytes(bin[po + 4 * i..po + 4 * i + 4].try_into().unwrap())).collect();
    let tf: Vec<i32> = (0..tl).map(|i| i32::from_le_bytes(bin[to + 4 * i..to + 4 * i + 4].try_into().unwrap())).collect();
    let faces: Vec<Json> = res.get_arr("faces").unwrap().to_vec();
    let mut area = vec![0.0f64; faces.len()];
    for (t, &f) in tf.iter().enumerate() {
        let q = |i: usize, c: usize| pos[9 * t + 3 * i + c] as f64;
        let a = [q(1, 0) - q(0, 0), q(1, 1) - q(0, 1), q(1, 2) - q(0, 2)];
        let b = [q(2, 0) - q(0, 0), q(2, 1) - q(0, 1), q(2, 2) - q(0, 2)];
        let c = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
        area[f as usize] += 0.5 * (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
    }
    let (d, _) = call(&obj(vec![("type", "describe".into()), ("tag", tag.into()), ("names", Json::Arr(faces.clone()))]));
    let items = result(&d).get_arr("items").unwrap();
    for (i, it) in items.iter().enumerate() {
        let ex = it.get_f64("area").unwrap_or(0.0).abs();
        if ex > 1e-6 && !((area[i] / ex - 1.0).abs() < 0.03) {
            out.push(format!("{} ({}) mesh/exact = {:.3}", it.get_str("name").unwrap_or("?"), it.get_str("surface").unwrap_or("?"), area[i] / ex));
        }
    }
    out
}

macro_rules! fixture {
    ($name:ident, $file:literal) => {
        #[test]
        fn $name() {
            let p = problems(stringify!($name), $file, include_str!(concat!("fixtures/tess_closed_region/", $file)));
            assert!(p.is_empty(), "{}:\n{}", $file, p.join("\n"));
        }
    };
}

/// A swept tube's side face (a closed spline whose seam twins have no pcurve) meshed at 1 to 2 percent of its area before.
macro_rules! tube_fixture {
    ($name:ident, $file:literal) => {
        #[test]
        fn $name() {
            let p = problems(stringify!($name), $file, include_str!(concat!("../../ck-model/tests/fixtures/", $file)));
            assert!(p.is_empty(), "{}:\n{}", $file, p.join("\n"));
        }
    };
}
tube_fixture!(sweep_tube_one_rim_fillet, "sweep_tube_one_rim_fillet.ckpart.json");
tube_fixture!(sweep_tube_rim_fillet, "sweep_tube_rim_fillet.ckpart.json");

fixture!(sphere_split_by_boolean, "A_sphere_hemisphere_part.json");
fixture!(loft_seam_after_boolean_443, "B1_loft443_part.json");
fixture!(loft_seam_after_boolean_396, "B1_loft396_part.json");
fixture!(coil_seam_after_boolean_1411, "B1_coil1411_part.json");
fixture!(crossed_lofted_tubes_1615, "cases/tessellation_1615-crossed-lofted-tubes.json");
// (open: its fi1.B0 spline faces mesh 81 to 94 percent of the exact area through the server path; clean when the boolean chained the loops)
#[test]
#[ignore]
fn sphere_plane_rim_fillet_r2() {
    let p = problems("r2", "r.json", include_str!("fixtures/tess_closed_region/cases/fillets_cov-fillet-exact-sphere-plane-rim-r2.json"));
    assert!(p.is_empty(), "{}", p.join("\n"));
}
fixture!(extrusion_of_closed_spline, "B3_spl_periodic_extrusion.step");
fixture!(closed_rational_dome_two_seam_pairs, "B2_42594_closed_rational_dome.step");
fixture!(control_hemisphere_equator_loop, "ok_control_hemisphere_equator_loop.step");
fixture!(control_hemisphere_seam, "ok_control_hemisphere_seam.step");
fixture!(control_sphere_seam_only, "ok_control_sphere_seam_only.step");

// ---- diagnostic dump of the faces that are off (CK_DUMP_PARTS=<part.json>,.. cargo test -p ck-wasm --test tess_wrong_region dump -- --ignored --nocapture)
use ck_model::features::Brep;
use ck_model::regen::{self, RegenOpts};
struct Ex(Brep);
impl regen::FeatureExec for Ex {
    fn face_count(&self) -> Option<usize> { self.0.face_count() }
    fn face_frame(&mut self, s: &regen::RegenState, face: &str) -> Option<ck_core::linalg::Frame> { self.0.face_frame(s, face) }
    fn exec(&mut self, s: &mut regen::RegenState, f: &ck_model::Feature) -> Result<(), regen::FeatureError> { self.0.exec(s, f) }
}
fn body_of(text: &str) -> Option<ck_topo::body::Body> {
    let part = ck_model::Part::parse(text).ok()?;
    let mut ex = Ex(Brep::new());
    regen::regen(&part, &RegenOpts::default(), &mut ex);
    ex.0.body.take()
}
#[test]
#[ignore = "diagnostic: needs CK_* env input files, prints only; run by hand"]
fn dump() {
    let Ok(list) = std::env::var("CK_DUMP_PARTS") else { return };
    for p in list.split(',') {
        let text = std::fs::read_to_string(p).unwrap();
        let body = if p.ends_with(".step") || p.ends_with(".stp") { ck_io::step::read_step(&text).ok().map(|i| i.body) } else { body_of(&text) };
        let Some(body) = body else {
            eprintln!("PART {p} no body");
            continue;
        };
        eprintln!("PART {p}");
        let tess = ck_tess::mesh::tessellate(&body);
        for (wi, f) in body.face_ids().enumerate() {
            let exact = ck_topo::mass::face_integral::<1>(&body, f, 1, &|_, n| [n.len()])[0].abs();
            let mesh: f64 = tess.faces[wi].as_ref().map(|fm| fm.tris.iter().map(|t| 0.5 * (fm.pts[t[1] as usize] - fm.pts[t[0] as usize]).cross(fm.pts[t[2] as usize] - fm.pts[t[0] as usize]).len()).sum()).unwrap_or(0.0);
            let face = body.face(f);
            if (mesh / exact - 1.0).abs() < 0.03 || exact < 1e-2 { continue; }
            eprintln!("FACE {} {} sense {} periods {:?} exact {exact:.4} mesh {mesh:.4}", body.face_id_str(f), face.surface.kind(), face.sense, face.surface.periods());
            if let ck_geom::surface::Surface::BSpline(bs) = &face.surface {
                let ((u0, u1), (v0, v1)) = bs.domain();
                let gu = (0..5).map(|k| { let v = v0 + (v1 - v0) * (k as f64 + 0.5) / 5.0; bs.eval(u0, v).dist(bs.eval(u1, v)) }).fold(0.0, f64::max);
                let gv = (0..5).map(|k| { let u = u0 + (u1 - u0) * (k as f64 + 0.5) / 5.0; bs.eval(u, v0).dist(bs.eval(u, v1)) }).fold(0.0, f64::max);
                eprintln!("  domain u [{u0:.4},{u1:.4}] v [{v0:.4},{v1:.4}] closure gap u {gu:.3e} v {gv:.3e} closed_periods {:?}", ck_geom::pcurve::closed_periods(&face.surface));
            }
            for &l in &face.loops {
                eprintln!("  loop {} coedges", body.lp(l).coedges.len());
                for &c in &body.lp(l).coedges {
                    let ce = body.coedge(c);
                    let e = body.edge(ce.edge);
                    let (a, b) = body.coedge_params(c);
                    let sample = |t: f64| { let q = body.coedge_uv(c, t, None); format!("({:.4},{:.4})", q.x, q.y) };
                    eprintln!("    ce rev {} seam {} edge {:?} t[{:.3},{:.3}] pc {} uv {} {} {}", ce.rev, body.is_seam(ce.edge), ce.edge.idx(), e.t0, e.t1, ce.pcurve.is_some(), sample(a), sample(0.5*(a+b)), sample(b));
                }
            }
        }
    }
}

/// `CK_TESS_FILES=<file.step|part.json>,..`: lists the faces of arbitrary files whose mesh is off (diagnostic).
#[test]
#[ignore = "diagnostic: needs CK_* env input files, prints only; run by hand"]
fn audit_files() {
    let Ok(list) = std::env::var("CK_TESS_FILES") else { return };
    for (i, p) in list.split(',').enumerate() {
        let text = std::fs::read_to_string(p).unwrap();
        let t0 = std::time::Instant::now();
        let pr = problems(&format!("a{i}"), p, &text);
        eprintln!("FILE {p} {:.2}s {}", t0.elapsed().as_secs_f64(), if pr.is_empty() { "clean".to_string() } else { pr.join("; ") });
    }
}
