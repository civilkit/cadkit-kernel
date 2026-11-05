//! C libm symbols backed by the `libm` crate (see Cargo.toml). Only the functions the kernel's native test binaries
//! import from the host libm are listed (`nm -D --undefined-only`); sqrt, floor, fma and the like are exact everywhere.
macro_rules! f1 {
    ($($n:ident),*) => {$(
        #[no_mangle]
        pub extern "C" fn $n(x: f64) -> f64 { libm::$n(x) }
    )*};
}
macro_rules! f2 {
    ($($n:ident),*) => {$(
        #[no_mangle]
        pub extern "C" fn $n(x: f64, y: f64) -> f64 { libm::$n(x, y) }
    )*};
}
f1!(acos, acosh, asin, asinh, atan, cos, cosh, exp, exp2, expm1, log, log10, log1p, log2, sin, sinh, tan, tanh, cbrt);
f2!(atan2, hypot, pow);

/// # Safety
/// `s` and `c` must be valid for writes (the C contract of `sincos`).
#[no_mangle]
pub unsafe extern "C" fn sincos(x: f64, s: *mut f64, c: *mut f64) {
    // compiler-builtins on wasm32 has no fused sincos: Rust calls sin and cos separately there
    *s = libm::sin(x);
    *c = libm::cos(x);
}
