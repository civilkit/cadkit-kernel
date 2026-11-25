//! Without the `abi` feature this crate must not export the wasm ABI symbols (`ck_alloc`, `ck_request`, ...): ck-wasm links
//! it as a plain library, and a second definition of those symbols breaks the native (MSVC) link.
#[cfg(not(feature = "abi"))]
#[test]
fn abi_not_exported_without_feature() {
    assert!(!ck_wasm_f3d::EXPORTS_ABI);
    // The decode API stays available to dependents.
    let _ = ck_wasm_f3d::handle(b"{}");
}

#[cfg(feature = "abi")]
#[test]
fn abi_exported_with_feature() {
    assert!(ck_wasm_f3d::EXPORTS_ABI);
}
