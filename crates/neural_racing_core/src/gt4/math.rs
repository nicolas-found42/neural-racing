//! Scalar math recovered from original GT4 EE instructions.
//!
//! Range reduction explicitly rounds toward zero, as in the recovered EE code.
//! Trigonometric polynomial arithmetic preserves separate toward-zero products
//! and accumulator additions. No host libm is substituted here.

// A product or sum of the bounded f32 range-reduction operands is exact in f64.
// Round its f32 conversion toward zero instead of the host's nearest mode. This
// matters before cancellation (for example cos(10000)), not just in the last bit
// of the final result. It deliberately does not change the process rounding mode.
fn truncate_f32(value: f64) -> f32 {
    let nearest = value as f32;
    if f64::from(nearest).abs() > value.abs() {
        f32::from_bits(nearest.to_bits() - 1)
    } else {
        nearest
    }
}

fn range_add(a: f32, b: f32) -> f32 {
    truncate_f32(f64::from(a) + f64::from(b))
}

fn range_mul(a: f32, b: f32) -> f32 {
    truncate_f32(f64::from(a) * f64::from(b))
}

// EE scalar ADD.S aligns with a single guard bit; it drops the smaller
// operand's round/sticky bits before adding. ACC ADDA/SUBA in the reference
// execution use the separate toward-zero operation above. Inputs here are
// finite normal range-reduction operands (or zero), never denormals/NaNs.
fn range_scalar_add(a: f32, b: f32) -> f32 {
    if a == 0.0 || b == 0.0 {
        return a + b;
    }
    let (large, small) = if a.abs() >= b.abs() { (a, b) } else { (b, a) };
    let large_exponent = (large.to_bits() >> 23) & 255;
    let small_exponent = (small.to_bits() >> 23) & 255;
    let alignment = large_exponent - small_exponent;
    if alignment == 0 {
        return range_add(a, b);
    }
    let significand = ((small.to_bits() & 0x007f_ffff) | 0x0080_0000) << 3;
    let aligned = if alignment < 32 {
        (significand >> alignment) & !3
    } else {
        0
    };
    let unit = f64::from_bits(((large_exponent as i32 - 127 - 26 + 1023) as u64) << 52);
    let signed = if small.is_sign_negative() { -1.0 } else { 1.0 };
    truncate_f32(f64::from(large) + f64::from(aligned) * unit * signed)
}

fn sine_polynomial(value: f32, square: f32, fourth: f32) -> f32 {
    let cubic = range_mul(square, value);
    let quintic = range_mul(fourth, value);
    let septic = range_mul(fourth, cubic);
    let nonic = range_mul(quintic, fourth);
    let mut accumulator = range_add(value, 0.0);
    accumulator = range_add(accumulator, range_mul(f32::from_bits(0xbe2a_aaa4), cubic));
    accumulator = range_add(accumulator, range_mul(f32::from_bits(0x3c08_873e), quintic));
    accumulator = range_add(accumulator, range_mul(f32::from_bits(0xb94f_b21f), septic));
    range_add(accumulator, range_mul(f32::from_bits(0x362e_9c14), nonic))
}

/// Original 0x0057d8a0: first output pointer is sine, second is cosine.
/// Preserves range reduction, sign-bit construction, polynomial coefficients
/// and accumulator operation order. The supported domain is finite angles whose
/// quotient by pi fits i32, as supplied by the recovered vehicle callers.
pub fn sin_cos(angle: f32) -> (f32, f32) {
    let inverse_pi = f32::from_bits(0x3ea2_f983);
    let pi = f32::from_bits(0x4049_0fdb);
    let half_pi = f32::from_bits(0x3fc9_0fdb);
    let sign_word = (angle.to_bits() as i32) >> 31;
    let add_pi = f32::from_bits((sign_word as u32) & pi.to_bits());
    let quotient = range_mul(angle, inverse_pi) as i32;
    let periods = quotient as f32;
    let shifted = range_scalar_add(angle, add_pi);
    let period_angle = range_mul(periods, pi);
    let first = range_add(shifted, -period_angle);
    let second = range_add(range_add(pi, -shifted), period_angle);
    let sine_reduced = first.min(second);
    let sign = f32::from_bits((((sign_word ^ quotient) as u32) << 31) | 0x3f80_0000);
    let cosine_reduced = range_add(range_add(shifted, -half_pi), -period_angle);
    let cosine_square = range_mul(cosine_reduced, cosine_reduced);
    let cosine_value = range_mul(cosine_reduced, -sign);
    let sine_square = range_mul(sine_reduced, sine_reduced);
    let sine_value = range_mul(sine_reduced, sign);
    let cosine_fourth = range_mul(cosine_square, cosine_square);
    let sine_fourth = range_mul(sine_square, sine_square);
    (
        sine_polynomial(sine_value, sine_square, sine_fourth),
        sine_polynomial(cosine_value, cosine_square, cosine_fourth),
    )
}

/// Original standalone cosine entry 0x0057d1f0. Its range reduction is different
/// from the combined sine/cosine routine and is preserved separately.
pub fn cos(angle: f32) -> f32 {
    let half_pi = f32::from_bits(0x3fc9_0fdb);
    let pi = half_pi + half_pi;
    let periods = range_mul(angle, f32::from_bits(0x3ea2_f983)) as i32;
    let negative = ((angle.to_bits() as i32) < 0) as i32;
    let reduced = range_add(
        range_add(angle, -half_pi),
        -range_mul((periods - negative) as f32, pi),
    );
    let sign = f32::from_bits((((negative ^ ((!periods) & 1)) as u32) << 31) | 0x3f80_0000);
    let square = range_mul(reduced, reduced);
    sine_polynomial(range_mul(reduced, sign), square, range_mul(square, square))
}

/// Original standalone sine entry 0x0057d2b8.
pub fn sin(angle: f32) -> f32 {
    let half_pi = f32::from_bits(0x3fc9_0fdb);
    let shifted = range_scalar_add(angle, half_pi);
    let pi = half_pi + half_pi;
    let negative = ((shifted.to_bits() as i32) < 0) as i32;
    let periods = range_mul(shifted, f32::from_bits(0x3ea2_f983)) as i32;
    let reduced = range_add(angle, -range_mul((periods - negative) as f32, pi));
    let sign = f32::from_bits((((negative ^ (periods & 1)) as u32) << 31) | 0x3f80_0000);
    let square = range_mul(reduced, reduced);
    sine_polynomial(range_mul(reduced, sign), square, range_mul(square, square))
}

/// Original 0x0036cb90 with its 0x0036cb20 tail, including the fast-path endpoint
/// choices and arithmetic reduction for angles outside +/-three pi.
pub fn wrap_angle(mut angle: f32) -> f32 {
    use super::eefloat::{add, div, mul};
    let pi = f32::from_bits(0x4049_0fda);
    let tau = f32::from_bits(0x40c9_0fda);
    let three_pi = f32::from_bits(0x4116_cbe3);
    if angle < -pi {
        if angle < -three_pi {
            let periods = div(add(angle.abs(), pi), tau) as i32 as f32;
            return add(angle, mul(periods, tau));
        }
        angle = add(angle, tau);
    } else if angle >= pi {
        if angle > three_pi {
            let periods = div(add(angle.abs(), pi), tau) as i32 as f32;
            return add(angle, mul(periods, -tau));
        }
        angle = add(angle, -tau);
    }
    angle
}

/// Original 0x0036cc40: wrap each operand, subtract, then correct one turn.
pub fn angle_difference(first: f32, second: f32) -> f32 {
    use super::eefloat::add;
    let mut difference = add(wrap_angle(first), -wrap_angle(second));
    let pi = f32::from_bits(0x4049_0fda);
    let tau = f32::from_bits(0x40c9_0fda);
    if difference >= pi {
        difference = add(difference, -tau);
    } else if difference < -pi {
        difference = add(difference, tau);
    }
    difference
}

/// Original rational arctangent kernel 0x0057d008, including its two range
/// reductions and the raw input sign bit (which preserves negative zero).
pub fn atan(input: f32) -> f32 {
    use super::eefloat::{add, div};
    let mut value = input.abs();
    let reduction;
    if value > f32::from_bits(0x401a_827a) {
        reduction = 1;
        value = div(1.0, value);
    } else if value > f32::from_bits(0x3ed4_13cd) {
        reduction = -1;
        value = div(add(1.0, -value), add(value, 1.0));
    } else {
        reduction = 0;
    }
    let square = range_mul(value, value);
    let fourth = range_mul(square, square);
    let cubic = range_mul(square, value);
    let mut denominator = range_add(1.0, 0.0);
    denominator = range_add(denominator, range_mul(f32::from_bits(0x3f8e_38e4), square));
    denominator = range_add(denominator, range_mul(f32::from_bits(0x3e73_cf3d), fourth));
    let quintic = range_mul(cubic, square);
    let mut numerator = range_add(value, 0.0);
    numerator = range_add(numerator, range_mul(f32::from_bits(0x3f47_1c72), cubic));
    numerator = range_add(numerator, range_mul(f32::from_bits(0x3d8a_b356), quintic));
    let mut result = range_mul(numerator, div(1.0, denominator));
    if reduction < 0 {
        result = add(f32::from_bits(0x3f49_0fdb), -result);
    } else if reduction > 0 {
        result = add(f32::from_bits(0x3fc9_0fdb), -result);
    }
    if (input.to_bits() as i32) < 0 {
        result = -result;
    }
    result
}

/// Original 0x0057d118. Zero/zero and signed-zero quadrant behavior deliberately
/// follow GT4's comparisons rather than Rust's host `atan2` implementation.
pub fn atan2(mut y: f32, mut x: f32) -> f32 {
    use super::eefloat::{add, div};
    if x == 0.0 && y == 0.0 {
        return 0.0;
    }
    let negative_y = y < 0.0;
    if negative_y {
        y = -y;
    }
    let negative_x = x < 0.0;
    if negative_x {
        x = -x;
    }
    let y_is_large = y >= x;
    let ratio = if y_is_large { div(x, y) } else { div(y, x) };
    let mut result = atan(ratio);
    if y_is_large {
        result = add(f32::from_bits(0x3fc9_0fda), -result);
    }
    if negative_x {
        result = add(f32::from_bits(0x4049_0fda), -result);
    }
    if negative_y {
        result = -result;
    }
    result
}
