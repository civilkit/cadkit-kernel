//! The split decoder (docs/f3d-lazy.md): `decodeF3d` of ck-wasm-f3d, then `importF3d` with `decoded` on the kernel, gives the
//! same reply as `importF3d` with the file's bytes. Run with `--features f3d`.

// the decoder is linked only with the `f3d` feature; the gate names every test file with --test, which a
// `required-features` target refuses, so the file is empty without it instead
#![cfg(feature = "f3d")]

use ck_json::Json;

fn b64(d: &[u8]) -> String {
    const T: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut s = String::new();
    for c in d.chunks(3) {
        let n = (c[0] as u32) << 16 | (*c.get(1).unwrap_or(&0) as u32) << 8 | *c.get(2).unwrap_or(&0) as u32;
        for i in 0..4 {
            s.push(if i <= c.len() { T[(n >> (18 - 6 * i) & 63) as usize] as char } else { '=' });
        }
    }
    s
}

fn envelope(msg: Json) -> String {
    ck_json::to_string(&Json::Obj(vec![("id".into(), Json::Num(1.0)), ("msg".into(), msg)]))
}

fn kernel(msg: Json) -> Json {
    let r = ck_server::handle(envelope(msg).as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    ck_json::parse(j).unwrap().get("result").unwrap().clone()
}

fn decoder(msg: Json) -> Json {
    let r = ck_wasm_f3d::handle(envelope(msg).as_bytes());
    let (j, _) = ck_server::reply::decode(&r).unwrap();
    let v = ck_json::parse(j).unwrap();
    assert_eq!(v.get_bool("ok"), Some(true), "{v:?}");
    v.get("result").unwrap().clone()
}

fn obj(kv: Vec<(&str, Json)>) -> Json {
    Json::Obj(kv.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
}

#[test]
fn decoded_bodies_give_the_inline_reply() {
    ck_wasm::engine::install();
    for blob in [ck_asm::testdata::cube_blob(), ck_asm::testdata::cylinder_blob_with(ck_asm::testdata::CylinderVariant { unknown_side: true, ..Default::default() })] {
        let data = b64(&blob);
        let inline = kernel(obj(vec![("type", "importF3d".into()), ("tag", "a".into()), ("data", data.clone().into())]));
        let dec = decoder(obj(vec![("type", "decodeF3d".into()), ("data", data.into())]));
        let decoded = dec.get("decoded").expect("decoded").clone();
        let lazy = kernel(obj(vec![("type", "importF3d".into()), ("tag", "b".into()), ("decoded", decoded)]));
        // the same reply (the tag is not in it)
        assert_eq!(ck_json::to_string(&inline), ck_json::to_string(&lazy));
        assert!(lazy.get("body").is_some());
    }
}

#[test]
fn an_unreadable_file_answers_the_error_from_the_decoder() {
    let dec = decoder(obj(vec![("type", "decodeF3d".into()), ("data", b64(b"not a fusion file").into())]));
    let e = dec.get("errors").unwrap().as_arr().unwrap();
    assert_eq!(e[0].as_arr().unwrap()[1].get_str("code").map(|c| c.starts_with("f3d.")), Some(true), "{dec:?}");
}

#[test]
fn ping_names_the_build() {
    let r = decoder(obj(vec![("type", "ping".into())]));
    assert_eq!(r.get_bool("f3d"), Some(true));
}

/// Regenerates `packages/kernel-client/src/fixtures/cube.smb` (a synthetic one-cube ASM blob, no third-party data) for the
/// node test: `CK_F3D_WRITE_FIXTURE=<path> cargo test -p ck-wasm --features f3d --test lazy_f3d`.
#[test]
fn write_fixture_when_asked() {
    if let Ok(p) = std::env::var("CK_F3D_WRITE_FIXTURE") {
        std::fs::write(p, ck_asm::testdata::cube_blob()).unwrap();
    }
}
