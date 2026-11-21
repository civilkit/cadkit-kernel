//! `meshDecimate` through the full server path: a noisy sphere (indexed, base64) comes back with fewer triangles as
//! buffers, a deviation within the tolerance and the report; `meshToSolid` with `opts.decimate` reports `import.decimation`;
//! refusals carry code, message and hint.

use ck_json::Json;

fn call(msg: &str) -> (Json, Vec<u8>) {
    ck_wasm::engine::install();
    let body = format!(r#"{{"id":1,"msg":{msg}}}"#);
    let r = ck_server::handle(body.as_bytes());
    let jl = u32::from_le_bytes([r[0], r[1], r[2], r[3]]) as usize;
    (ck_json::parse(std::str::from_utf8(&r[4..4 + jl]).unwrap()).unwrap(), r[4 + jl..].to_vec())
}

fn result(r: &Json) -> &Json {
    r.get("result").unwrap_or(r)
}

fn b64(bytes: &[u8]) -> String {
    ck_text::base64_encode(bytes)
}

/// A UV sphere of radius 10 with 0.01 noise: (positions f32, indices u32).
fn sphere(nu: usize, nv: usize) -> (Vec<f32>, Vec<u32>) {
    let mut s = 12345u64;
    let mut rnd = move || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        ((s >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * 0.02
    };
    let pt = |a: f64, b: f64| [(10.0 * b.sin() * a.cos()) as f32, (10.0 * b.sin() * a.sin()) as f32, (-10.0 * b.cos()) as f32];
    let mut p: Vec<f32> = pt(0.0, 0.0).to_vec();
    for j in 1..nv {
        for i in 0..nu {
            let q = pt(i as f64 / nu as f64 * std::f64::consts::TAU, j as f64 / nv as f64 * std::f64::consts::PI);
            p.extend(q.iter().map(|v| v + rnd() as f32));
        }
    }
    p.extend(pt(0.0, std::f64::consts::PI));
    let south = (p.len() / 3 - 1) as u32;
    let at = |i: usize, j: usize| (1 + (j - 1) * nu + i % nu) as u32;
    let mut t = vec![];
    for i in 0..nu {
        t.extend([0, at(i + 1, 1), at(i, 1)]);
        for j in 1..nv - 1 {
            t.extend([at(i, j), at(i + 1, j), at(i + 1, j + 1), at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
        t.extend([south, at(i, nv - 1), at(i + 1, nv - 1)]);
    }
    (p, t)
}

#[test]
fn decimates_a_sphere_over_the_wire() {
    let (p, t) = sphere(80, 40);
    let pb: Vec<u8> = p.iter().flat_map(|v| v.to_le_bytes()).collect();
    let tb: Vec<u8> = t.iter().flat_map(|v| v.to_le_bytes()).collect();
    let (r, bin) = call(&format!(r#"{{"type":"meshDecimate","positionsF32":"{}","indicesU32":"{}","opts":{{"tol":0.1,"targetTriangles":800}}}}"#, b64(&pb), b64(&tb)));
    let j = result(&r);
    assert_eq!(r.get("ok").and_then(|v| v.as_bool()), Some(true), "{r:?}");
    let tris = j.get_f64("triangles").unwrap() as usize;
    assert_eq!(j.get_f64("inputTriangles").unwrap() as usize, t.len() / 3);
    assert!(tris <= 800 && tris > 300, "{tris}");
    assert_eq!(j.get_str("stoppedBy"), Some("target"));
    assert!(j.get("deviation").unwrap().get_f64("max").unwrap() <= 0.1);
    assert_eq!(j.get_f64("selfIntersections"), Some(0.0));
    // the buffers
    let (pos, idx) = (j.get("positions").unwrap(), j.get("indices").unwrap());
    assert_eq!(pos.get_str("type"), Some("f32"));
    assert_eq!(idx.get_str("type"), Some("u32"));
    assert_eq!(idx.get_f64("len").unwrap() as usize, tris * 3);
    assert_eq!(pos.get_f64("len").unwrap() as usize, j.get_f64("vertices").unwrap() as usize * 3);
    assert!(bin.len() >= (idx.get_f64("len").unwrap() as usize) * 4);
}

#[test]
fn refusals_say_why() {
    let (r, _) = call(r#"{"type":"meshDecimate","positions":[0,0,0,1,0,0,0,1,0],"opts":{}}"#);
    let e = result(&r).get_arr("errors").unwrap()[0].as_arr().unwrap().to_vec();
    assert_eq!(e[1].get_str("code"), Some("meshBadInput"));
    assert!(e[1].get("params").unwrap().get_str("hint").is_some());
    let (r, _) = call(r#"{"type":"meshDecimate","opts":{"tol":0.1}}"#);
    let e = result(&r).get_arr("errors").unwrap()[0].as_arr().unwrap().to_vec();
    assert_eq!(e[1].get_str("code"), Some("meshBadInput"));
}

#[test]
fn mesh_to_solid_reports_its_decimation() {
    let (p, t) = sphere(60, 30);
    let pb: Vec<u8> = p.iter().flat_map(|v| v.to_le_bytes()).collect();
    let tb: Vec<u8> = t.iter().flat_map(|v| v.to_le_bytes()).collect();
    let (r, _) = call(&format!(
        r#"{{"type":"meshToSolid","tag":"d","positionsF32":"{}","indicesU32":"{}","opts":{{"analytic":false,"deviation":false,"maxFaces":1500,"decimate":{{"tol":0.1,"targetTriangles":1200}}}}}}"#,
        b64(&pb),
        b64(&tb)
    ));
    let j = result(&r);
    assert!(j.get_arr("errors").map_or(true, |e| e.is_empty()), "{r:?}");
    let d = j.get("import").unwrap().get("decimation").expect("import.decimation");
    assert!(d.get_f64("triangles").unwrap() <= 1200.0);
    assert!(d.get("deviation").unwrap().get_f64("max").unwrap() <= 0.1);
}
