//! Bounded EE scalar-float operations for branch-sensitive recovered routines.
//!
//! Inputs and results are finite f32 values. Subnormal inputs and outputs flush
//! to signed zero as in the configured EE reference. Each intermediate rounds
//! toward zero without changing
//! process/thread FPU mode. f64 is used only to compute an exact f32 operand sum
//! or product before that rounding; this is not a higher-precision physics model.
//!
//! ADD.S preserves the EE single guard-bit alignment used by the pinned Play!
//! reference. SQRT uses an exact candidate-square comparison to select the
//! toward-zero f32 result. Physical R5900 multiplier edge quirks, flags,
//! infinities and saturation are outside this bounded implementation.

fn admitted(value: f32) -> bool {
    value.is_finite()
}

fn truncate(value: f64) -> f32 {
    let nearest = value as f32;
    if nearest.is_subnormal() {
        return f32::from_bits(nearest.to_bits() & 0x8000_0000);
    }
    assert!(
        admitted(nearest),
        "EE float result outside bounded finite domain"
    );
    if f64::from(nearest).abs() > value.abs() {
        flush(f32::from_bits(nearest.to_bits() - 1))
    } else {
        nearest
    }
}

fn flush(value: f32) -> f32 {
    if value.is_subnormal() {
        f32::from_bits(value.to_bits() & 0x8000_0000)
    } else {
        value
    }
}

pub(crate) fn mul(a: f32, b: f32) -> f32 {
    let (a, b) = (flush(a), flush(b));
    assert!(
        admitted(a) && admitted(b),
        "EE multiply input outside bounded finite domain"
    );
    truncate(f64::from(a) * f64::from(b))
}

/// Finite quotient with the EE reference's flush-to-zero and truncation policy.
/// A zero divisor or overflowing quotient is outside this bounded contract.
pub(crate) fn div(a: f32, b: f32) -> f32 {
    let (a, b) = (flush(a), flush(b));
    assert!(
        admitted(a) && admitted(b) && b != 0.0,
        "EE divide input outside bounded finite domain"
    );
    truncate(f64::from(a) / f64::from(b))
}

pub(crate) fn add(a: f32, b: f32) -> f32 {
    let (a, b) = (flush(a), flush(b));
    assert!(
        admitted(a) && admitted(b),
        "EE add input outside bounded finite domain"
    );
    if a == 0.0 || b == 0.0 {
        return a + b;
    }
    let (large, small) = if a.abs() >= b.abs() { (a, b) } else { (b, a) };
    let exponent = (large.to_bits() >> 23) & 255;
    let smaller_exponent = (small.to_bits() >> 23) & 255;
    let alignment = exponent - smaller_exponent;
    if alignment == 0 {
        return truncate(f64::from(a) + f64::from(b));
    }
    let significand = ((small.to_bits() & 0x007f_ffff) | 0x0080_0000) << 3;
    let aligned = if alignment < 32 {
        (significand >> alignment) & !3
    } else {
        0
    };
    let unit = f64::from_bits(((exponent as i32 - 127 - 26 + 1023) as u64) << 52);
    let sign = if small.is_sign_negative() { -1.0 } else { 1.0 };
    truncate(f64::from(large) + f64::from(aligned) * unit * sign)
}

pub(crate) fn sqrt(value: f32) -> f32 {
    let value = flush(value);
    assert!(
        admitted(value) && value >= 0.0,
        "EE square root input outside nonnegative finite domain"
    );
    let candidate = f64::from(value).sqrt() as f32;
    if f64::from(candidate) * f64::from(candidate) > f64::from(value) {
        f32::from_bits(candidate.to_bits() - 1)
    } else {
        candidate
    }
}
