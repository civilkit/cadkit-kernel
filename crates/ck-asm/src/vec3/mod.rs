// SPDX-License-Identifier: MIT
//! Free functions on three-component vectors (`[f64; 3]`), the one copy
//! the model, the solver and the file readers share. There is no vector
//! type: points, directions and offsets stay plain arrays.
//!
//! The order of the floating-point operations is part of the contract.
//! Callers compare results with files' stored values and with earlier
//! recomputes, so every function is written as the plain component
//! expression (`a[0] * b[0] + a[1] * b[1] + a[2] * b[2]`, no fused steps,
//! no iterator sums), and the variants that differ (the cut-off below
//! which a vector has no direction, a division by the length or a
//! multiplication by its reciprocal) have their own names.
//!
//! The two-component helpers (`[f64; 2]`: sketch, drawing and plan
//! coordinates) are in [`vec2`], under the same names.

pub mod vec2;

/// A point, a direction or an offset.
pub type Vec3 = [f64; 3];

/// The shortest vector that still has a direction, for [`unit`].
pub const MIN_LENGTH: f64 = 1e-12;

/// `a + b`.
pub fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

/// `a - b`.
pub fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// `a * s`, every component multiplied.
pub fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

/// `a + b * s`, with `b * s` rounded before the sum (no fused step).
pub fn add_scaled(a: Vec3, b: Vec3, s: f64) -> Vec3 {
    [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s]
}

/// `a * sa + b * sb`, each product rounded before the sum.
pub fn combine(a: Vec3, sa: f64, b: Vec3, sb: f64) -> Vec3 {
    [
        a[0] * sa + b[0] * sb,
        a[1] * sa + b[1] * sb,
        a[2] * sa + b[2] * sb,
    ]
}

/// The dot product, summed from the first component to the last.
pub fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// The cross product.
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// The length: `dot(a, a).sqrt()`.
pub fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}

/// The distance between two points: the length of `a - b`.
pub fn distance(a: Vec3, b: Vec3) -> f64 {
    norm(sub(a, b))
}

/// The unit vector along `v`, `None` when `v` is no longer than
/// [`MIN_LENGTH`] or not finite. The components are multiplied by the
/// reciprocal of the length (see [`unit_div`] for the division).
pub fn unit(v: Vec3) -> Option<Vec3> {
    unit_above(v, MIN_LENGTH)
}

/// Like [`unit`] with a cut-off of your own: `None` unless the length is
/// finite and greater than `min`.
pub fn unit_above(v: Vec3, min: f64) -> Option<Vec3> {
    let n = norm(v);
    (n > min && n.is_finite()).then(|| scale(v, 1.0 / n))
}

/// Like [`unit`], but every component is divided by the length instead of
/// multiplied by its reciprocal. The two differ in the last bit for some
/// vectors; a caller keeps the one its results were made with.
pub fn unit_div(v: Vec3) -> Option<Vec3> {
    unit_div_above(v, MIN_LENGTH)
}

/// [`unit_div`] with a cut-off of your own.
pub fn unit_div_above(v: Vec3, min: f64) -> Option<Vec3> {
    let n = norm(v);
    (n > min && n.is_finite()).then(|| [v[0] / n, v[1] / n, v[2] / n])
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Vec3 = [0.1, -2.3, 4.7];
    const B: Vec3 = [-1.9, 0.3, 5.1];

    #[test]
    fn the_arithmetic_is_the_plain_component_expression() {
        assert_eq!(add(A, B), [A[0] + B[0], A[1] + B[1], A[2] + B[2]]);
        assert_eq!(sub(A, B), [A[0] - B[0], A[1] - B[1], A[2] - B[2]]);
        assert_eq!(scale(A, 0.3), [A[0] * 0.3, A[1] * 0.3, A[2] * 0.3]);
        assert_eq!(dot(A, B), A[0] * B[0] + A[1] * B[1] + A[2] * B[2]);
        assert_eq!(
            add_scaled(A, B, 0.7),
            [A[0] + B[0] * 0.7, A[1] + B[1] * 0.7, A[2] + B[2] * 0.7]
        );
        assert_eq!(
            combine(A, 0.7, B, -1.3),
            [
                A[0] * 0.7 + B[0] * -1.3,
                A[1] * 0.7 + B[1] * -1.3,
                A[2] * 0.7 + B[2] * -1.3
            ]
        );
        assert_eq!(norm(A), (A[0] * A[0] + A[1] * A[1] + A[2] * A[2]).sqrt());
        assert_eq!(distance(A, B), norm(sub(A, B)));
    }

    #[test]
    fn the_cross_product_is_right_handed_and_square_to_both() {
        assert_eq!(cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]), [0.0, 0.0, 1.0]);
        assert_eq!(cross([0.0, 1.0, 0.0], [0.0, 0.0, 1.0]), [1.0, 0.0, 0.0]);
        let c = cross(A, B);
        assert!(dot(c, A).abs() < 1e-12 && dot(c, B).abs() < 1e-12);
    }

    #[test]
    fn a_unit_vector_has_length_one_and_a_short_one_has_none() {
        let u = unit(A).unwrap();
        assert!((norm(u) - 1.0).abs() < 1e-15);
        assert_eq!(unit([0.0; 3]), None);
        assert_eq!(unit([1e-13, 0.0, 0.0]), None);
        assert_eq!(unit([f64::INFINITY, 0.0, 0.0]), None);
        assert_eq!(unit([f64::NAN, 0.0, 0.0]), None);
        assert!(unit([2e-12, 0.0, 0.0]).is_some());
        assert_eq!(unit([0.0, -4.0, 0.0]), Some([0.0, -1.0, 0.0]));
    }

    #[test]
    fn the_cut_off_is_the_callers() {
        assert!(unit_above([1e-13, 0.0, 0.0], 0.0).is_some());
        assert_eq!(unit_above([0.0; 3], 0.0), None);
        assert_eq!(unit_above([0.5, 0.0, 0.0], 1.0), None);
        assert!(unit_div_above([1e-13, 0.0, 0.0], 0.0).is_some());
        assert_eq!(unit_div_above([0.0; 3], 0.0), None);
        assert_eq!(unit_div_above([0.0, 0.0, 3.0], 0.0), Some([0.0, 0.0, 1.0]));
    }

    #[test]
    fn the_reciprocal_and_the_division_keep_their_own_bits() {
        // Over a range of vectors the two ways of normalising are each what
        // their expression says, whether or not they agree in the last bit.
        for i in 1..200 {
            let v = [i as f64 * 0.37, -(i as f64) * 1.13, (i as f64).sqrt()];
            let n = norm(v);
            assert_eq!(unit_above(v, 0.0), Some(scale(v, 1.0 / n)));
            assert_eq!(unit_div_above(v, 0.0), Some([v[0] / n, v[1] / n, v[2] / n]));
        }
    }
}
