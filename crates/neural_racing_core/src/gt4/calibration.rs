//! Numeric automatic gear-ratio builder recovered at 0x00354ff8.
//! Stock equipment invokes this after its real database/default conversion;
//! raw GEAR rows alone can contain zero ratios and are not drivable settings.

use super::eefloat::{add, div, mul};

/// Original six-byte rows at 0x0062042a + gear_count * 6, for clamped counts 5–7.
pub const AUTO_GEAR_SPREAD: [[u8; 6]; 3] = [
    [155, 165, 188, 190, 190, 200],
    [165, 175, 190, 192, 190, 200],
    [170, 180, 192, 194, 190, 200],
];

#[derive(Clone, Copy, Debug)]
pub struct TireDimensionsBytes {
    pub width_at_1c8: u8,
    pub rim_at_1ca: u8,
    pub sidewall_at_1cc: u8,
    pub rim_adjustment_at_1e2: i8,
    pub width_percent_at_1e4: u8,
}

/// Standard equipment branch of 0x0034ba30. Returns radius, width and sidewall
/// in metres. The optional custom-model geometry override is outside this API.
pub fn tire_dimensions(source: TireDimensionsBytes) -> [f32; 3] {
    let millimetres = match u16::from(source.width_at_1c8) * 5 {
        0 => 10,
        value => value,
    };
    let mut width = mul(f32::from(millimetres), f32::from_bits(0x3a83_126e));
    let rim = source.rim_at_1ca;
    let (rim_units, half) = if rim < 61 {
        (rim, 0.5)
    } else if rim % 10 == 0 {
        (rim / 10, 0.5)
    } else {
        (rim, f32::from_bits(0x3d4c_cccc))
    };
    let rim_radius = mul(mul(f32::from(rim_units), 0.0254), half);
    let mut sidewall = mul(
        mul(width, f32::from(source.sidewall_at_1cc)),
        f32::from_bits(0x3d4c_cccc),
    );
    let radius = add(rim_radius, sidewall);
    if source.rim_adjustment_at_1e2 != 0 {
        let mut adjustment = mul(mul(f32::from(source.rim_adjustment_at_1e2), 0.0254), 0.5);
        if add(rim_radius, adjustment) < 0.0254 {
            adjustment = add(0.0254, -rim_radius);
        }
        sidewall = add(sidewall, -adjustment);
        if sidewall < 0.0127 {
            sidewall = 0.0127;
        }
    }
    let percent = source.width_percent_at_1e4;
    if percent != 0 && percent != 100 {
        width = mul(width, div(f32::from(percent), 100.0));
    }
    [radius, width, sidewall]
}

#[derive(Clone, Copy, Debug)]
pub struct AutoGearParameters {
    pub gear_count_at_155: u8,
    pub redline_at_bd: u8,
    pub final_ratio_at_150: i16,
    /// Wheel radius selected by original 0x0034bc98 -> 0x0034ba30, after the
    /// caller's optional reset of expanded+0x156. This API does not infer tires.
    pub selected_wheel_radius: f32,
}

/// Original numeric body of 0x00354ff8. `speed_setting` is Eq+0x12a * 10 in
/// stock equipment. The original writes gears 0..=gear_count only; unused
/// destination entries remain untouched, so they must be initialized by callers.
pub fn build_auto_ratios(
    parameters: AutoGearParameters,
    speed_setting: i32,
    destination: &mut [i16; 12],
) {
    let count = usize::from(parameters.gear_count_at_155);
    assert!((1..=9).contains(&count));
    let spread = AUTO_GEAR_SPREAD[count.clamp(5, 7) - 5];
    let blend = div((speed_setting - 150) as f32, 250.0);
    let first_spacing = div(200.0, f32::from(spread[1]));
    let spacing_delta = add(div(200.0, f32::from(spread[0])), -first_spacing);
    let mut spacing = add(first_spacing, mul(blend, spacing_delta));
    let spacing_progression = div(200.0, f32::from(spread[3]));
    let progression_delta = add(div(200.0, f32::from(spread[2])), -spacing_progression);
    let progression = add(spacing_progression, mul(blend, progression_delta));
    let first_gear_factor = div(200.0, f32::from(spread[5]));
    let first_gear_delta = add(div(200.0, f32::from(spread[4])), -first_gear_factor);
    let first_gear_multiplier = add(first_gear_factor, mul(blend, first_gear_delta));
    let rpm = f32::from(u16::from(parameters.redline_at_bd) * 100);
    let numerator = mul(
        mul(rpm, f32::from_bits(0x3dd6_774e)),
        parameters.selected_wheel_radius,
    );
    let speed = mul(speed_setting as f32, f32::from_bits(0x3e8e_38e3));
    let final_ratio = mul(
        f32::from(parameters.final_ratio_at_150),
        f32::from_bits(0x3a83_126e),
    );
    let mut ratio = div(div(numerator, speed), final_ratio);
    for gear in (1..=count).rev() {
        if gear == 1 {
            ratio = mul(ratio, first_gear_multiplier);
        }
        let stored = mul(ratio, 1000.0);
        ratio = mul(ratio, spacing);
        spacing = mul(spacing, progression);
        destination[gear] = stored as i32 as i16;
    }
    let mut reverse = i32::from(destination[1]) * 3 + i32::from(destination[2]);
    if reverse < 0 {
        reverse += 3;
    }
    destination[0] = (reverse >> 2) as i16;
}
