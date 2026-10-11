//! The wasm ABI boundary (cdylib). Matches kernelworker.js exactly:
//!
//! exports: `memory`, `ck_alloc(n) -> ptr`, `ck_request(ptr, len) -> u32`,
//!          `ck_reply() -> ptr`, `ck_commit() -> ptr`, `ck_commit_len() -> u32`
//! imports (module `env`): `ck_panic(ptr, len)`, `ck_now() -> f64`,
//!          `ck_unix_ms() -> f64`
//!
//! Evidence (kernel.wat):
//! - `ck_alloc` (type i32->i32): one-time init, then allocates `max(n, 1)`
//!   bytes with align 1, panicking on failure;
//! - `ck_request` (i32,i32->i32): one-time init, borrows the global server
//!   (RefCell flag @8527280) and takes ownership of the request buffer
//!   (capacity `max(len, 1)` — it is freed after the call);
//! - `ck_reply`: borrows the global reply Vec (RefCell flag @8527208) and
//!   returns its data pointer;
//! - `ck_commit` returns a static string, `ck_commit_len` = 58:
//!   "class-a-surfacing da3c6f49122f7955186a15d91471699f37e7fcf5"
//!   (read back through the oracle's ck_commit/ck_commit_len).
//! - panics reach the host through `ck_panic` as the std PanicHookInfo
//!   Display text, e.g. "panicked at /rustc/<hash>/library/core/src/time.rs:
//!   962:23:\ncannot convert float seconds to Duration: value is either too
//!   big or NaN" (oracle with ck_now returning Infinity); then `panic=abort`
//!   traps (`unreachable`) and the host re-instantiates.

use std::cell::RefCell;

/// The build id the original reports.
pub const COMMIT: &str = "class-a-surfacing da3c6f49122f7955186a15d91471699f37e7fcf5";

pub mod engine;

#[cfg(target_arch = "wasm32")]
mod host {
    #[link(wasm_import_module = "env")]
    extern "C" {
        pub fn ck_panic(ptr: *const u8, len: usize);
        pub fn ck_now() -> f64;
        pub fn ck_unix_ms() -> f64;
    }
}

#[cfg(not(target_arch = "wasm32"))]
mod host {
    //! Native stand-ins so the rlib links in tests.
    pub unsafe fn ck_panic(ptr: *const u8, len: usize) {
        let s = std::str::from_utf8(std::slice::from_raw_parts(ptr, len)).unwrap_or("?");
        eprintln!("ck_panic: {s}");
    }
    pub unsafe fn ck_now() -> f64 {
        0.0
    }
    pub unsafe fn ck_unix_ms() -> f64 {
        0.0
    }
}

fn now() -> f64 {
    unsafe { host::ck_now() }
}

fn unix_ms() -> f64 {
    unsafe { host::ck_unix_ms() }
}

thread_local! {
    static REPLY: RefCell<Vec<u8>> = const { RefCell::new(Vec::new()) };
}

fn init() {
    static ONCE: std::sync::Once = std::sync::Once::new();
    ONCE.call_once(|| {
        std::panic::set_hook(Box::new(|info| {
            let s = info.to_string();
            unsafe { host::ck_panic(s.as_ptr(), s.len()) }
        }));
        ck_server::clock::set_now(now);
        ck_server::clock::set_unix_ms(unix_ms);
        engine::install();
    });
}

/// Room for a request body of `n` bytes (ownership passes to `ck_request`).
#[no_mangle]
pub extern "C" fn ck_alloc(n: usize) -> *mut u8 {
    init();
    let mut v: Vec<u8> = Vec::with_capacity(n.max(1));
    let p = v.as_mut_ptr();
    std::mem::forget(v);
    p
}

/// Run one request; returns the total reply length (see [`ck_reply`]).
///
/// # Safety
/// `ptr` must come from [`ck_alloc`] with `n >= len`.
#[no_mangle]
pub unsafe extern "C" fn ck_request(ptr: *mut u8, len: usize) -> u32 {
    init();
    // GUESS (consistent with the WAT): the buffer was allocated with
    // capacity max(len, 1) — hosts always alloc exactly `len` bytes.
    let body = Vec::from_raw_parts(ptr, len, len.max(1));
    let reply = ck_server::handle(&body);
    drop(body);
    let n = reply.len() as u32;
    REPLY.with(|r| *r.borrow_mut() = reply);
    n
}

/// Pointer to the last reply (`ck_request`'s return value bytes long).
#[no_mangle]
pub extern "C" fn ck_reply() -> *const u8 {
    REPLY.with(|r| r.borrow().as_ptr())
}

#[no_mangle]
pub extern "C" fn ck_commit() -> *const u8 {
    COMMIT.as_ptr()
}

#[no_mangle]
pub extern "C" fn ck_commit_len() -> u32 {
    COMMIT.len() as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn abi_roundtrip() {
        let body = br#"{"id":1,"msg":{"type":"ping"}}"#;
        let p = ck_alloc(body.len());
        unsafe { std::ptr::copy_nonoverlapping(body.as_ptr(), p, body.len()) };
        let n = unsafe { ck_request(p, body.len()) } as usize;
        let r = unsafe { std::slice::from_raw_parts(ck_reply(), n) };
        assert_eq!(n, 48);
        assert_eq!(&r[4..45], br#"{"ok":true,"result":{"pong":true},"ms":0}"#);
        assert_eq!(ck_commit_len(), 58);
    }
}
