//! Re-drilled hole after a pad was fused under it (a plate with a hole, a pad fused under the plate that closes the hole from
//! below, then the SAME coaxial cylinder cut again through pad and plate): the cut must remove the pad's material
//! under the hole, leave a closed shell, and tessellate watertight.
use super::tests::{box_body, cyl_body};
use super::{boolean, Op};
use ck_topo::body::Body;

const TOL: f64 = 1e-6;

fn vol(b: &Body) -> f64 {
    ck_topo::mass::mass_props(b).volume
}
fn cylv(r: f64, h: f64) -> f64 {
    std::f64::consts::PI * r * r * h
}
fn step(a: &Body, b: &Body, op: Op) -> Body {
    let out = boolean(a, b, op, TOL).expect("boolean");
    assert!(out.mismatch.is_none(), "volume mismatch {:?}", out.mismatch);
    out.body
}

/// plate 40x40x4 (z 0..4), hole r 3.15 through, pad 25x25x3 under the plate (z -3..0), drill again from z0 to z1.
fn redrill(r2: f64, z0: f64, z1: f64) -> (Body, f64) {
    let plate = box_body("P", 0.0, 0.0, 40.0, 40.0, 0.0, 4.0);
    let hole = cyl_body("H", 10.0, 10.0, 3.15, -1.0, 5.0);
    let pad = box_body("D", 5.0, 5.0, 30.0, 30.0, -3.0, 0.0);
    let drill = cyl_body("K", 10.0, 10.0, r2, z0, z1);
    let a = step(&plate, &hole, Op::Cut);
    let a = step(&a, &pad, Op::Union);
    let b = step(&a, &drill, Op::Cut);
    let want = 1600.0 * 4.0 + 625.0 * 3.0 - cylv(3.15, 7.0);
    (b, want)
}

fn check(b: &Body, want: f64) {
    let v = vol(b);
    assert!((v - want).abs() < 1e-4 * want, "volume {v} want {want}");
    let issues = ck_check::validate(b, TOL);
    assert!(issues.is_empty(), "issues {issues:?}");
    let mesh = ck_tess::mesh::tessellate(b).mesh;
    let open = ck_check::tessellation::check_mesh(b, &mesh.positions, &mesh.tri_face, &mesh.failed);
    assert!(open.is_empty(), "open mesh {open:?}");
}

#[test]
fn redrill_ends_on_plate_top() {
    let (b, want) = redrill(3.15, -6.0, 4.0);
    check(&b, want);
}

#[test]
fn redrill_past_plate_top() {
    let (b, want) = redrill(3.15, -6.0, 6.0);
    check(&b, want);
}
