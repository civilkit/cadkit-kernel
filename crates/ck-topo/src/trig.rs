//! `sin` / `cos` exactly as kernel.wasm computes them.
//!
//! The original was built for wasm32-unknown-unknown, where `f64::sin` /
//! `f64::cos` are compiler-builtins' libm (the musl / FreeBSD msun port):
//! WAT f4182 = cos, f4184 = sin (their polynomial constants C1..C6 /
//! S1..S6 are the msun `__cos` / `__sin` kernels). Native host libms differ
//! in the last ulp (macOS: cos(120°) = -0.49999999999999983 vs the
//! kernel's -0.4999999999999998), which moves revolve vertices and frames
//! by an ulp and breaks DIRECTION sharing in STEP output. This is a port of
//! the same algorithm so native builds reproduce the wasm bits; on wasm32
//! it computes the same thing as std.
//!
//! Verified against the oracle's f4182/f4184 (exported from an
//! instrumented kernel.wat) on 4000 arguments in tests/trig_oracle.rs.

const C1: f64 = 4.16666666666666019037e-02;
const C2: f64 = -1.38888888888741095749e-03;
const C3: f64 = 2.48015872894767294178e-05;
const C4: f64 = -2.75573143513906633035e-07;
const C5: f64 = 2.08757232129817482790e-09;
const C6: f64 = -1.13596475577881948265e-11;

const S1: f64 = -1.66666666666666324348e-01;
const S2: f64 = 8.33333333332248946124e-03;
const S3: f64 = -1.98412698298579493134e-04;
const S4: f64 = 2.75573137070700676789e-06;
const S5: f64 = -2.50507602534068634195e-08;
const S6: f64 = 1.58969099521155010221e-10;

fn k_cos(x: f64, y: f64) -> f64 {
    let z = x * x;
    let w = z * z;
    let r = z * (C1 + z * (C2 + z * C3)) + w * w * (C4 + z * (C5 + z * C6));
    let hz = 0.5 * z;
    let w = 1.0 - hz;
    w + (((1.0 - w) - hz) + (z * r - x * y))
}

fn k_sin(x: f64, y: f64, iy: i32) -> f64 {
    let z = x * x;
    let w = z * z;
    let r = S2 + z * (S3 + z * S4) + z * w * (S5 + z * S6);
    let v = z * x;
    if iy == 0 {
        x + v * (S1 + z * r)
    } else {
        x - ((z * (0.5 * y - v * r) - y) - v * S1)
    }
}

const TO_INT: f64 = 1.5 / f64::EPSILON;
const INV_PIO2: f64 = 6.36619772367581382433e-01;
const PIO2_1: f64 = 1.57079632673412561417e+00;
const PIO2_1T: f64 = 6.07710050650619224932e-11;
const PIO2_2: f64 = 6.07710050630396597660e-11;
const PIO2_2T: f64 = 2.02226624879595063154e-21;
const PIO2_3: f64 = 2.02226624871116645580e-21;
const PIO2_3T: f64 = 8.47842766036889956997e-32;

fn medium(x: f64, ix: u32) -> (i32, f64, f64) {
    let f_n = x * INV_PIO2 + TO_INT - TO_INT;
    let n = f_n as i32;
    let mut r = x - f_n * PIO2_1;
    let mut w = f_n * PIO2_1T;
    let mut y0 = r - w;
    let ey = ((y0.to_bits() >> 52) & 0x7ff) as i32;
    let ex = (ix >> 20) as i32;
    if ex - ey > 16 {
        let t = r;
        w = f_n * PIO2_2;
        r = t - w;
        w = f_n * PIO2_2T - ((t - r) - w);
        y0 = r - w;
        let ey = ((y0.to_bits() >> 52) & 0x7ff) as i32;
        if ex - ey > 49 {
            let t = r;
            w = f_n * PIO2_3;
            r = t - w;
            w = f_n * PIO2_3T - ((t - r) - w);
            y0 = r - w;
        }
    }
    let y1 = (r - y0) - w;
    (n, y0, y1)
}

/// msun `__rem_pio2` for |x| < 2^20·π/2; None beyond (callers fall back to
/// std — the large-argument path is never reached by modelling angles).
fn rem_pio2(x: f64) -> Option<(i32, f64, f64)> {
    let sign = (x.to_bits() >> 63) != 0;
    let ix = ((x.to_bits() >> 32) as u32) & 0x7fffffff;
    let small = |k: f64, n: i32| {
        if !sign {
            let z = x - k * PIO2_1;
            let y0 = z - k * PIO2_1T;
            (n, y0, (z - y0) - k * PIO2_1T)
        } else {
            let z = x + k * PIO2_1;
            let y0 = z + k * PIO2_1T;
            (-n, y0, (z - y0) + k * PIO2_1T)
        }
    };
    if ix <= 0x400f6a7a {
        if (ix & 0xfffff) == 0x921fb {
            return Some(medium(x, ix));
        }
        return Some(if ix <= 0x4002d97c { small(1.0, 1) } else { small(2.0, 2) });
    }
    if ix <= 0x401c463b {
        if ix <= 0x4015fdbc {
            if ix == 0x4012d97c {
                return Some(medium(x, ix));
            }
            return Some(small(3.0, 3));
        }
        if ix == 0x401921fb {
            return Some(medium(x, ix));
        }
        return Some(small(4.0, 4));
    }
    if ix < 0x413921fb {
        return Some(medium(x, ix));
    }
    None
}

pub fn cos(x: f64) -> f64 {
    let ix = ((x.to_bits() >> 32) as u32) & 0x7fffffff;
    if ix <= 0x3fe921fb {
        if ix < 0x3e46a09e {
            return 1.0;
        }
        return k_cos(x, 0.0);
    }
    if ix >= 0x7ff00000 {
        return x - x;
    }
    let Some((n, y0, y1)) = rem_pio2(x) else { return x.cos() };
    match n & 3 {
        0 => k_cos(y0, y1),
        1 => -k_sin(y0, y1, 1),
        2 => -k_cos(y0, y1),
        _ => k_sin(y0, y1, 1),
    }
}

pub fn sin(x: f64) -> f64 {
    let ix = ((x.to_bits() >> 32) as u32) & 0x7fffffff;
    if ix <= 0x3fe921fb {
        if ix < 0x3e500000 {
            return x;
        }
        return k_sin(x, 0.0, 0);
    }
    if ix >= 0x7ff00000 {
        return x - x;
    }
    let Some((n, y0, y1)) = rem_pio2(x) else { return x.sin() };
    match n & 3 {
        0 => k_sin(y0, y1, 1),
        1 => k_cos(y0, y1),
        2 => -k_sin(y0, y1, 1),
        _ => -k_cos(y0, y1),
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn kernel_values() {
        // oracle f4182 / f4184 (and V8's fdlibm) values
        assert_eq!(super::cos(2.0943951023931953), -0.4999999999999998);
        assert_eq!(super::sin(2.0943951023931953), 0.8660254037844387);
        assert_eq!(super::cos(std::f64::consts::FRAC_PI_2), 6.123233995736766e-17);
        assert_eq!(super::sin(std::f64::consts::PI), 1.2246467991473532e-16);
        assert_eq!(super::cos(0.0), 1.0);
    }
}
