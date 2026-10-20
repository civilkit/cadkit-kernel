//! `importSkp` request with `opts.up`: default 'z' (as in the file), 'y' = (x,y,z) -> (x,z,-y), 'yFlip' = (-x,-z,-y).
//! Needs the `skp` feature: `cargo test -p ck-wasm --features skp --test import_skp`.
#![cfg(feature = "skp")]

use ck_json::Json;

fn b64(d: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::new();
    for c in d.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            if i <= c.len() {
                s.push(T[(n >> (18 - 6 * i) & 63) as usize] as char);
            } else {
                s.push('=');
            }
        }
    }
    s
}

fn call(msg: Json) -> Json {
    let body = ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg)]));
    let r = ck_server::handle(body.as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap()
}

fn import(data: &str, up: Option<&str>) -> Json {
    ck_wasm::engine::install();
    let mut kv: Vec<(String, Json)> = vec![("type".into(), "importSkp".into()), ("tag".into(), "t".into()), ("data".into(), data.into())];
    if let Some(u) = up {
        kv.push(("opts".into(), Json::Obj(vec![("up".into(), u.into())])));
    }
    call(Json::Obj(kv))
}

fn nums(j: &Json) -> Vec<f64> {
    j.as_arr().unwrap().iter().map(|v| v.as_f64().unwrap()).collect()
}

#[test]
fn up_option_rotates_the_body_and_reports_the_matrix() {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../ck-skp/tests/fixtures/chair_and_table.skp")).unwrap();
    let data = b64(&bytes);
    let z = import(&data, None);
    let zi = z.get("result").unwrap().get("import").unwrap();
    assert_eq!(zi.get_str("up"), Some("z"), "kernel default stays as in the file");
    assert_eq!(nums(zi.get("upMatrix").unwrap()), vec![1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
    let zb = nums(zi.get("bboxMm").unwrap());
    let explicit = import(&data, Some("z"));
    assert_eq!(nums(explicit.get("result").unwrap().get("import").unwrap().get("bboxMm").unwrap()), zb);

    let y = import(&data, Some("y"));
    let yr = y.get("result").unwrap();
    let yi = yr.get("import").unwrap();
    assert_eq!(yi.get_str("up"), Some("y"));
    assert_eq!(nums(yi.get("upMatrix").unwrap()), vec![1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0]);
    let yb = nums(yi.get("bboxMm").unwrap());
    // (x, y, z) -> (x, z, -y)
    let want = [zb[0], zb[2], -zb[4], zb[3], zb[5], -zb[1]];
    for k in 0..6 {
        assert!((yb[k] - want[k]).abs() < 1e-3, "bbox {k}: {} vs {}", yb[k], want[k]);
    }
    for key in ["faces", "lumps", "openEdges", "nonManifoldEdges", "skippedFaces"] {
        assert_eq!(yi.get_f64(key), zi.get_f64(key), "{key}");
    }
    assert_eq!(yi.get("faceMaterials"), zi.get("faceMaterials"));
    // the wire body is the rotated one: same face count, box from the rotated geometry
    let wb = yr.get("body").unwrap();
    assert_eq!(wb.get_arr("faces").unwrap().len(), z.get("result").unwrap().get("body").unwrap().get_arr("faces").unwrap().len());

    let f = import(&data, Some("yFlip"));
    let fb = nums(f.get("result").unwrap().get("import").unwrap().get("bboxMm").unwrap());
    let want = [-zb[3], -zb[5], -zb[4], -zb[0], -zb[2], -zb[1]];
    for k in 0..6 {
        assert!((fb[k] - want[k]).abs() < 1e-3, "flip bbox {k}: {} vs {}", fb[k], want[k]);
    }
}

#[test]
fn bad_up_value_is_a_request_error() {
    let bytes = std::fs::read(concat!(env!("CARGO_MANIFEST_DIR"), "/../ck-skp/tests/fixtures/circle.skp")).unwrap();
    let r = import(&b64(&bytes), Some("x"));
    assert_eq!(r.get("ok"), Some(&Json::Bool(false)), "{}", ck_json::to_string(&r));
    assert!(r.get_str("error").unwrap().contains("opts.up"));
}
