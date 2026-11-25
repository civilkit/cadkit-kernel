//! The Fusion 360 decoder as a wasm of its own (docs/f3d-lazy.md). The main kernel wasm does not link the decoder; the
//! app fetches this module the first time a `.f3d` is opened and calls it with the same ABI as the kernel:
//!
//! exports: `memory`, `ck_alloc(n) -> ptr`, `ck_request(ptr, len) -> u32`, `ck_reply() -> ptr`, `ck_commit() -> ptr`,
//!          `ck_commit_len() -> u32`
//! imports (module `env`): `ck_panic(ptr, len)`, `ck_now() -> f64`, `ck_unix_ms() -> f64`
//!
//! so `WasmHost` hosts it unchanged. Requests (the kernel's envelope `{id, msg}`, replies framed like the kernel's):
//!
//! - `decodeF3d {name?, data: base64 of the .f3d / .f3z / .smb, opts?: {history?, allBodies?}}` -> `{decoded: {solid,
//!   surfaces, info}}` (what the main kernel's `importF3d` takes as `decoded`), or `{errors: [["import", {code, params}]]}`;
//! - `f3dDump {name?, data}` -> `{designs: [{label, dump, reference, referenceError}]}` (feature `features` only), or `errors`
//!   likewise; the main kernel's `f3dFeatures` maps the dumps to ck-part features;
//! - `ping` -> `{pong: true, f3d: true, features: bool}`.

use ck_json::{Json, Obj};

pub mod b64;
pub mod decode;
#[cfg(feature = "features")]
pub mod features;

pub use decode::{decode, to_wire, DecodeError, Decoded, Options};

/// The build id `ck_commit` reports, as the kernel's (`ck <revision>`).
pub const COMMIT: &str = concat!("ck ", env!("CK_WASM_COMMIT"));

pub fn error_result(code: &str, message: &str) -> Json {
    let err = Json::Arr(vec![Json::Str("import".into()), Obj::new().put("code", code).put("params", Obj::new().put("message", message).build()).build()]);
    Obj::new().put("errors", Json::Arr(vec![err])).build()
}

fn handle_msg(msg: &Json) -> Result<Json, String> {
    let ty = msg.get_str("type").unwrap_or("");
    let bytes = |what: &str| -> Result<Vec<u8>, String> {
        match msg.get_str("data") {
            Some(d) => b64::decode(d, what),
            None => Err(format!("{what}: data (the .f3d / .f3z file or .smb blob as base64) is required")),
        }
    };
    let name = msg.get_str("name").unwrap_or("model");
    match ty {
        "ping" => Ok(Obj::new().put("pong", true).put("f3d", true).put("features", cfg!(feature = "features")).build()),
        "decodeF3d" => {
            let bytes = bytes("decodeF3d")?;
            let o = msg.get("opts");
            let opts = Options { history: o.and_then(|o| o.get_bool("history")).unwrap_or(false), all_bodies: o.and_then(|o| o.get_bool("allBodies")).unwrap_or(false) };
            Ok(match decode(&bytes, name, &opts) {
                Ok(d) => Obj::new().put("decoded", to_wire(&d)).build(),
                Err(e) => error_result(&e.code, &e.message),
            })
        }
        #[cfg(feature = "features")]
        "f3dDump" => features::request(&bytes("f3dDump")?, name),
        _ => Err(format!("unknown request type: {ty}")),
    }
}

/// One raw request body -> the framed reply (the kernel's framing: `ck_server::reply`).
pub fn handle(body: &[u8]) -> Vec<u8> {
    let r = match ck_json::parse_bytes(body) {
        Err(_) => Err("bad request body".to_string()),
        Ok(env) => match env.get("msg") {
            Some(m @ Json::Obj(_)) => handle_msg(m),
            _ => Err("request has no msg".to_string()),
        },
    };
    let o = match r {
        Ok(r) => Obj::new().put("ok", true).put("result", r).put("ms", 0.0),
        Err(e) => Obj::new().put("ok", false).put("error", e.as_str()).put("ms", 0.0),
    };
    ck_server::reply::encode(&ck_json::to_string(&o.build()), &[])
}

// The wasm ABI (`#[no_mangle]` exports `ck_alloc`, `ck_request`, ...) exists only with the `abi` feature, which
// tools/build_wasm_f3d.sh turns on for the standalone wasm. Without it this crate is a plain library (ck-wasm's
// `f3d` feature links `decode`), so native binaries do not get a second copy of the kernel's `ck_*` symbols
// (duplicate-symbol link errors with MSVC: LNK2005 / LNK1169).
#[cfg(feature = "abi")]
mod abi;

/// Whether this build carries the exported wasm ABI (the `abi` feature).
pub const EXPORTS_ABI: bool = cfg!(feature = "abi");
