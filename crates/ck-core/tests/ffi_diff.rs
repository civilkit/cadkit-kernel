//! Differential tests: call the ORIGINAL kernel functions (via the wasm2c
//! native build, linked as libkerneloracle.a) and compare against the Rust
//! reimplementation. These are the strongest available ground truth.

#![allow(non_camel_case_types)]
use ck_core::linalg::Vec3;

type u32_ = u32;

extern "C" {
    fn ck_oracle_init();
    fn ck_oracle_alloc(n: u32_) -> u32_;
    fn ck_oracle_mem() -> *mut u8;
    fn ck_oracle_instance() -> *mut u8;
    // kernel func[912]/func[1000]: (instance, ptr, ptr) -> f64
    fn w2c_kernel_f912(inst: *mut u8, a: u32_, b: u32_) -> f64;
    fn w2c_kernel_f1000(inst: *mut u8, a: u32_, b: u32_) -> f64;
}

unsafe fn write_vec(off: u32_, v: Vec3) {
    let m = ck_oracle_mem().add(off as usize) as *mut f64;
    *m = v.x;
    *m.add(1) = v.y;
    *m.add(2) = v.z;
}

unsafe fn write_u32(off: u32_, v: u32_) {
    *(ck_oracle_mem().add(off as usize) as *mut u32_) = v;
}

/// func[912] semantics (from WAT, verified by C probe): param0 -> pointer
/// slot; S0 = *(u32*)param0 = plane struct {origin Vec3 @0, normal @+0x48};
/// P = param1. Returns |n·(P − origin)|.
#[test]
fn point_plane_distance_matches_kernel() {
    unsafe {
        ck_oracle_init();
        let inst = ck_oracle_instance();

        let q = Vec3::new(1.0, 2.0, 3.0);
        let n = Vec3::new(0.0, 0.0, 1.0);
        let p = Vec3::new(1.0, 2.0, 7.5);
        let signed = n.dot(p - q);

        // plane struct: origin @0, normal @+0x48
        let plane = ck_oracle_alloc(0x60);
        write_vec(plane, q);
        write_vec(plane + 0x48, n);
        // param0 = pointer slot holding the plane pointer
        let s = ck_oracle_alloc(4);
        write_u32(s, plane);
        let pb = ck_oracle_alloc(0x18);
        write_vec(pb, p);

        let kernel = w2c_kernel_f912(inst, s, pb);
        assert!((kernel - signed.abs()).abs() < 1e-12, "f912={} signed={}", kernel, signed);
        assert!((kernel - p.plane_dist(n, q)).abs() < 1e-12);

        // func[1000]: same shape; sign convention checked loosely
        let k1000 = w2c_kernel_f1000(inst, s, pb);
        assert!(
            ((k1000 - signed.abs()).abs() < 1e-12) || ((k1000 - signed).abs() < 1e-12),
            "f1000={} signed={} abs={}",
            k1000,
            signed,
            signed.abs()
        );
    }
}
