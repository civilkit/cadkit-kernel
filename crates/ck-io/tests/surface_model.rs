//! A body that is not closed is written as a SHELL_BASED_SURFACE_MODEL; reading it back gives the same faces as a part
//! of its own (a surface body, not a refusal).
mod bodies;
use ck_io::step::{read_step, write_step, StepOpts};

#[test]
fn an_open_body_reads_back_as_a_part_with_all_its_faces() {
    let mut b = bodies::build("box");
    let f = b.face_ids().next().unwrap();
    ck_topo::edit::delete_face(&mut b, f);
    ck_topo::region::rebuild_lumps(&mut b);
    let text = write_step(&b, "open box", &StepOpts::default());
    assert!(text.contains("SHELL_BASED_SURFACE_MODEL"), "writer no longer writes surface models");
    let r = read_step(&text).unwrap();
    assert_eq!(r.solids, 0);
    assert_eq!(r.parts.len(), 1);
    assert_eq!((r.parts[0].faces_total, r.parts[0].faces.len()), (5, 5));
    assert!(r.parts[0].refused.is_empty());
    assert!(r.parts[0].surface_model);
    assert_eq!(r.body.num_faces(), 5);
    let a0 = ck_topo::mass::mass_props(&b).area;
    let a1 = ck_topo::mass::mass_props(&r.body).area;
    assert!((a0 - a1).abs() < 1e-6 * a0, "{a0} vs {a1}");
}
