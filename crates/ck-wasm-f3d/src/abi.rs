//! The exported wasm ABI of the standalone decoder (feature `abi`; see lib.rs).

use crate::{handle, COMMIT};
use std::cell::RefCell;

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
        ck_server::clock::set_now(|| unsafe { host::ck_now() });
        ck_server::clock::set_unix_ms(|| unsafe { host::ck_unix_ms() });
        // the guard regenerates with the kernel, which keeps to a wall-clock budget
        ck_core::budget::set_clock(ck_server::clock::now);
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
    let body = Vec::from_raw_parts(ptr, len, len.max(1));
    let reply = handle(&body);
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
