//! The original's `simplify` stage (ck-ops/src/simplify.rs): after a boolean,
//! adjacent faces on the SAME surface (coplanar, cocylindrical, cocone, ...)
//! are merged and the edges that separated them are dropped, and collinear
//! edges meeting at a 2-valent vertex are joined.
//!
//! There is no separate pass here: it is done while the boolean assembles its
//! result, in [`crate::boolean::assemble`]: `select_with` fuses coincident
//! same-facing fragments that share an edge (any analytic surface, decided by
//! the surface/surface `Coincident` result), and `join_collinear` joins the
//! left-over collinear edges. The periodic case needs care: a merged
//! cylinder / cone must keep the seam of each fragment (a face bounded only
//! by two circles integrates to zero lateral area). Before 2026-10-13 the two
//! seams cancelled and stacking two equal coaxial cylinders gave volume
//! 2094.4 instead of 6283.2 with no error (probe `simpCylStack`).
//!
//! Tests below pin the merged results against the oracle's numbers
//! (tools/cmp_stubs.mjs simp*); ck-topo `edit::merge_coplanar` remains the
//! planar-only editing helper.

#[cfg(test)]
mod tests {
    use crate::boolean::tests::{box_body, cyl_body};
    use crate::boolean::{boolean, Op};
    use ck_topo::mass::faces_moments;
    use ck_geom::Vec3;
    use std::f64::consts::PI;

    fn vol_area(b: &ck_topo::body::Body) -> (f64, f64) {
        let fs: Vec<_> = b.face_ids().collect();
        let m = faces_moments(b, &fs, Vec3::ZERO);
        (m.volume, m.area)
    }

    #[test]
    fn stacked_equal_cylinders_merge_into_one_face_with_the_right_volume() {
        // oracle simpCylStack: 3 faces, volume 6283.18531, area 1884.95559
        let a = cyl_body("F2", 5.0, 3.0, 10.0, 0.0, 10.0);
        let b = cyl_body("F4", 5.0, 3.0, 10.0, 10.0, 20.0);
        let u = boolean(&a, &b, Op::Union, 1e-6).expect("union").body;
        assert_eq!(u.num_faces(), 3);
        let (v, area) = vol_area(&u);
        assert!((v - 2.0 * PI * 100.0 * 10.0).abs() < 1e-6, "volume {v}");
        assert!((area - 1884.95559).abs() < 1e-4, "area {area}");
    }

    #[test]
    fn coplanar_box_stack_merges() {
        // oracle simpBoxStack: 6 faces / 12 edges, volume 16000
        let a = box_body("F2", 0.0, 0.0, 40.0, 20.0, 0.0, 10.0);
        let b = box_body("F4", 0.0, 0.0, 40.0, 20.0, 10.0, 20.0);
        let u = boolean(&a, &b, Op::Union, 1e-6).expect("union").body;
        assert_eq!((u.num_faces(), u.edge_ids().count()), (6, 12));
        assert!((vol_area(&u).0 - 16000.0).abs() < 1e-6);
    }
}
