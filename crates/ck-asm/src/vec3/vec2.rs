// SPDX-License-Identifier: MIT
//! Free functions on two-component vectors (`[f64; 2]`): sketch, drawing
//! and plan coordinates. The same contract as the 3D functions of the
//! crate: plain component expressions in a fixed order, and a named
//! variant for each cut-off and for a division by the length or a
//! multiplication by its reciprocal.
//!
//! Two things differ from the 3D functions, because every 2D copy these
//! replace did them so:
//!
//! - the length is `a[0].hypot(a[1])`, not `dot(a, a).sqrt()`;
//! - the unit-vector functions check only the cut-off, not that the length
//!   is finite: an infinite vector gives non-finite components.

/// A point, a direction or an offset in a plane.
pub type Vec2 = [f64; 2];

/// The shortest vector that still has a direction, for [`unit`] and
/// [`unit_div`].
pub const MIN_LENGTH: f64 = 1e-12;

/// `a + b`.
pub fn add(a: Vec2, b: Vec2) -> Vec2 {
    [a[0] + b[0], a[1] + b[1]]
}

/// `a - b`.
pub fn sub(a: Vec2, b: Vec2) -> Vec2 {
    [a[0] - b[0], a[1] - b[1]]
}

/// `a * s`, both components multiplied.
pub fn scale(a: Vec2, s: f64) -> Vec2 {
    [a[0] * s, a[1] * s]
}

/// The point at `t` from `a` to `b`: `a + (b - a) * t` per component.
pub fn lerp(a: Vec2, b: Vec2, t: f64) -> Vec2 {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// The dot product, the first component's product first.
pub fn dot(a: Vec2, b: Vec2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

/// The scalar cross product (the z of the 3D one): positive when `b` turns
/// counter-clockwise from `a`.
pub fn cross(a: Vec2, b: Vec2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// `a` rotated a quarter turn counter-clockwise.
pub fn perp(a: Vec2) -> Vec2 {
    [-a[1], a[0]]
}

/// The length: `a[0].hypot(a[1])`.
pub fn norm(a: Vec2) -> f64 {
    a[0].hypot(a[1])
}

/// The distance between two points: the length of `a - b`.
pub fn distance(a: Vec2, b: Vec2) -> f64 {
    norm(sub(a, b))
}

/// The unit vector along `v`, `None` unless its length is greater than
/// [`MIN_LENGTH`]. The components are multiplied by the reciprocal of the
/// length (see [`unit_div`] for the division).
pub fn unit(v: Vec2) -> Option<Vec2> {
    unit_above(v, MIN_LENGTH)
}

/// Like [`unit`] with a cut-off of your own: `None` unless the length is
/// greater than `min`.
pub fn unit_above(v: Vec2, min: f64) -> Option<Vec2> {
    let n = norm(v);
    (n > min).then(|| scale(v, 1.0 / n))
}

/// Like [`unit`], but both components are divided by the length instead of
/// multiplied by its reciprocal. The two differ in the last bit for some
/// vectors; a caller keeps the one its results were made with.
pub fn unit_div(v: Vec2) -> Option<Vec2> {
    unit_div_above(v, MIN_LENGTH)
}

/// [`unit_div`] with a cut-off of your own.
pub fn unit_div_above(v: Vec2, min: f64) -> Option<Vec2> {
    let n = norm(v);
    (n > min).then(|| [v[0] / n, v[1] / n])
}

#[cfg(test)]
mod tests {
    use super::*;

    const A: Vec2 = [0.1, -2.3];
    const B: Vec2 = [-1.9, 5.1];

    #[test]
    fn the_arithmetic_is_the_plain_component_expression() {
        assert_eq!(add(A, B), [A[0] + B[0], A[1] + B[1]]);
        assert_eq!(sub(A, B), [A[0] - B[0], A[1] - B[1]]);
        assert_eq!(scale(A, 0.3), [A[0] * 0.3, A[1] * 0.3]);
        assert_eq!(
            lerp(A, B, 0.7),
            [A[0] + (B[0] - A[0]) * 0.7, A[1] + (B[1] - A[1]) * 0.7]
        );
        assert_eq!(dot(A, B), A[0] * B[0] + A[1] * B[1]);
        assert_eq!(cross(A, B), A[0] * B[1] - A[1] * B[0]);
        assert_eq!(norm(A), A[0].hypot(A[1]));
        assert_eq!(distance(A, B), (A[0] - B[0]).hypot(A[1] - B[1]));
    }

    #[test]
    fn the_quarter_turn_and_the_cross_product_are_counter_clockwise() {
        assert_eq!(perp([1.0, 0.0]), [-0.0, 1.0]);
        assert_eq!(perp([0.0, 1.0]), [-1.0, 0.0]);
        assert_eq!(cross([1.0, 0.0], [0.0, 1.0]), 1.0);
        assert_eq!(cross(A, perp(A)), dot(A, A));
    }

    #[test]
    fn a_unit_vector_has_length_one_and_a_short_one_has_none() {
        let u = unit(A).unwrap();
        assert!((norm(u) - 1.0).abs() < 1e-15);
        assert_eq!(unit([0.0; 2]), None);
        assert_eq!(unit([1e-13, 0.0]), None);
        assert_eq!(unit([f64::NAN, 0.0]), None);
        assert!(unit([2e-12, 0.0]).is_some());
        assert_eq!(unit([0.0, -4.0]), Some([0.0, -1.0]));
        assert_eq!(unit_div([0.0, 0.0]), None);
        assert_eq!(unit_div([1e-13, 0.0]), None);
        assert_eq!(unit_div([3.0, 0.0]), Some([1.0, 0.0]));
    }

    #[test]
    fn an_infinite_vector_is_not_cut_off() {
        // As in the 2D copies these functions replace: only the cut-off is
        // checked.
        let u = unit([f64::INFINITY, 1.0]).unwrap();
        assert!(u[0].is_nan() && u[1] == 0.0);
        let u = unit_div([f64::INFINITY, 1.0]).unwrap();
        assert!(u[0].is_nan() && u[1] == 0.0);
    }

    #[test]
    fn the_cut_off_is_the_callers() {
        assert!(unit_above([1e-13, 0.0], 0.0).is_some());
        assert_eq!(unit_above([0.0; 2], 0.0), None);
        assert_eq!(unit_above([0.5, 0.0], 1.0), None);
        assert!(unit_div_above([1e-13, 0.0], 0.0).is_some());
        assert_eq!(unit_div_above([0.0; 2], 0.0), None);
        assert_eq!(unit_div_above([0.0, 3.0], 0.0), Some([0.0, 1.0]));
    }

    #[test]
    fn the_reciprocal_and_the_division_keep_their_own_bits() {
        for i in 1..200 {
            let v = [i as f64 * 0.37, -(i as f64).sqrt()];
            let n = v[0].hypot(v[1]);
            assert_eq!(
                unit_above(v, 0.0),
                Some([v[0] * (1.0 / n), v[1] * (1.0 / n)])
            );
            assert_eq!(unit_div_above(v, 0.0), Some([v[0] / n, v[1] / n]));
        }
    }
}
