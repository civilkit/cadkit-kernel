// Links the wasm2c-compiled original kernel (differential-test oracle) when
// it is present. If the static library is missing the crate still builds;
// the differential tests are compiled out (they are gated on
// `cfg(ck_oracle)`, which is only set here when the library exists).
use std::path::Path;

fn main() {
    println!("cargo:rustc-check-cfg=cfg(ck_oracle)");
    let c2 = std::env::var("CK_ORACLE_DIR")
        .unwrap_or_else(|_| "/path/to/work/c2".to_string());
    let lib = Path::new(&c2).join("libkerneloracle.a");
    println!("cargo:rerun-if-changed={}", lib.display());
    println!("cargo:rerun-if-env-changed=CK_ORACLE_DIR");
    if lib.exists() {
        println!("cargo:rustc-cfg=ck_oracle");
        println!("cargo:rustc-link-search=native={}", c2);
        println!("cargo:rustc-link-lib=static=kerneloracle");
        // the C side needs libm
        println!("cargo:rustc-link-lib=m");
    }
}
