//! Recovered signed wheel slip calculation, 0x00351358 and 0x003513f8.
//! This computes slip state; it is not the tire force law or wheel integrator.

const SPEED_EPSILON: f32 = f32::from_bits(0x3c8e38e3);
const KMH_TO_SPEED: f32 = f32::from_bits(0x3e8e38e3);
const PERCENT: f32 = f32::from_bits(0x3c23d70a);

/// Original bytes loaded at 0x00620d8c..0x00620d94.
pub const ORIGINAL_SLIP_BYTES: [u8; 8] = [10, 10, 0, 0, 0, 0, 0, 0];

/// Offsets are relative to 0x00620d8c. Mode zero/one preserve the source's
/// separate acceleration and braking gain/fade choices without renaming flags.
#[derive(Clone, Copy, Debug)]
pub struct SlipParameters {
    raw: [u8; 8],
}

impl SlipParameters {
    pub const fn from_bytes(raw: [u8; 8]) -> Self {
        Self { raw }
    }

    /// Original 0x00351358. Mode is the source's integer argument: zero or one.
    pub fn gain(&self, input: f32, mode: usize) -> f32 {
        assert!(mode <= 1, "GT4 slip gain mode must be zero or one");
        let mut gain = 1.0;
        if self.raw[2 + mode] != 0 {
            let speed_limit = f32::from(self.raw[6 + mode]) * KMH_TO_SPEED;
            let mut speed = input;
            if speed < 0.0 {
                speed = -speed;
            }
            if speed < speed_limit {
                let floor = f32::from(self.raw[4 + mode]) * PERCENT;
                gain = floor + ((1.0 - floor) * speed) / speed_limit;
            }
        }
        gain
    }
}

/// Sparse view of the original wheel block. Offset names avoid assigning
/// unverified units to +0x20/+0x28; callers must supply the recovered state.
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelSlipState {
    pub input_at_20: f32,
    pub input_at_28: f32,
    pub ratio_at_38: f32,
    pub gain_at_3c: f32,
    pub mode_at_44: i8,
    pub direction_at_45: i8,
}

/// Port of 0x003513f8, including its alternate branch (+0x1d8), gain (+0xfc8),
/// signed flags, asymmetric denominators and low-speed fade. No extra slip
/// epsilon, absolute-value denominator or clamp is substituted for GT4's.
pub fn update_slip(
    input: f32,
    wheel: &mut WheelSlipState,
    params: &SlipParameters,
    alternate: bool,
    alternate_gain: f32,
) {
    wheel.mode_at_44 = 0;
    let base = wheel.input_at_20;
    if alternate {
        let mut delta = input - base;
        if base >= 0.0 {
            let flag = if base < input { -1 } else { 1 };
            wheel.mode_at_44 = flag;
            wheel.direction_at_45 = flag;
        } else {
            delta = -delta;
            if base <= input {
                wheel.mode_at_44 = 1;
                wheel.direction_at_45 = -1;
            } else {
                wheel.mode_at_44 = -1;
                wheel.direction_at_45 = 1;
            }
        }
        let mut ratio = delta * alternate_gain;
        wheel.gain_at_3c = 1.0;
        if ratio > 1.0 {
            ratio = 1.0;
        } else if ratio < -1.0 {
            ratio = -1.0;
        }
        wheel.ratio_at_38 = ratio;
        return;
    }

    let gain;
    let mut ratio = 0.0;
    if base > -SPEED_EPSILON && base < SPEED_EPSILON {
        gain = params.gain(wheel.input_at_28, 0);
        wheel.direction_at_45 = if base < input {
            -1
        } else if input < base {
            1
        } else {
            0
        };
        if base != input {
            ratio = 1.0;
        }
        wheel.mode_at_44 = -1;
    } else if base > 0.0 {
        if input <= base {
            let delta = input - base;
            gain = params.gain(wheel.input_at_28, 1);
            ratio = (delta * gain) / base;
            wheel.mode_at_44 = 1;
            wheel.direction_at_45 = 1;
        } else {
            gain = params.gain(wheel.input_at_28, 0);
            if input >= SPEED_EPSILON || input <= -SPEED_EPSILON {
                ratio = ((input - base) * gain) / input;
            }
            wheel.mode_at_44 = -1;
            wheel.direction_at_45 = -1;
        }
    } else if base <= input {
        let delta = input - base;
        gain = params.gain(wheel.input_at_28, 1);
        ratio = (delta * gain) / base;
        wheel.mode_at_44 = 1;
        wheel.direction_at_45 = -1;
    } else {
        gain = params.gain(wheel.input_at_28, 0);
        if input >= SPEED_EPSILON || input <= -SPEED_EPSILON {
            ratio = ((input - base) * gain) / input;
        }
        wheel.mode_at_44 = -1;
        wheel.direction_at_45 = 1;
    }
    wheel.gain_at_3c = gain;
    if ratio > 1.0 {
        ratio = 1.0;
    } else if ratio < -1.0 {
        ratio = -1.0;
    }
    if wheel.mode_at_44 == 0 {
        wheel.ratio_at_38 = 0.0;
        return;
    }
    let (mut speed, fade_byte) = if wheel.mode_at_44 < 0 {
        (input, params.raw[0])
    } else {
        (base, params.raw[1])
    };
    if fade_byte != 0 {
        if speed < 0.0 {
            speed = -speed;
        }
        let fade_speed = f32::from(fade_byte) * KMH_TO_SPEED;
        if speed < fade_speed {
            ratio *= speed / fade_speed;
        }
    }
    wheel.ratio_at_38 = ratio;
}
