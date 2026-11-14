//! Recovered engine-curve correction: 0x00350678, 0x00350868, 0x0034e9a0.
use super::curve::Curve;

/// Original tuning bytes at loaded address 0x00620da8.
pub const ORIGINAL_CORRECTION_BYTES: [u8; 9] = [3, 30, 10, 100, 50, 2, 50, 60, 50];

/// Only the fields read by 0x00350868. Their physical meanings are unrecovered.
#[derive(Clone, Copy, Debug)]
pub struct CorrectionState {
    pub input_at_04: f32,
    pub input_at_08: f32,
    pub bypass: bool,
}

impl Default for CorrectionState {
    /// Constructor 0x003433e8 clears 32 bytes, then sets byte +0x14 to one.
    fn default() -> Self {
        Self {
            input_at_04: 0.0,
            input_at_08: 0.0,
            bypass: true,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct EngineCorrection {
    values: [f32; 9],
}

impl EngineCorrection {
    /// Arithmetic from lazy initializer 0x00350678. Rust stores the initialized
    /// result explicitly instead of sharing GT4's global once-only guard.
    pub fn from_tuning_bytes(raw: [u8; 9]) -> Self {
        let milli = f32::from_bits(0x3a83126e);
        let low = (u32::from(raw[2]) * 100_000) as f32;
        let span = (u32::from(raw[3]) * 100_000) as f32;
        Self {
            values: [
                -((u32::from(raw[0]) * 100_000) as f32),
                f32::from(raw[1]) * milli,
                low,
                span + low,
                f32::from(raw[4]) * milli,
                -((u32::from(raw[5]) * 100_000) as f32) * 0.5,
                f32::from(raw[6]) * milli,
                (u32::from(raw[7]) * 100_000) as f32,
                (u32::from(raw[8]) * 100_000) as f32,
            ],
        }
    }

    pub fn initialized_values(&self) -> &[f32; 9] {
        &self.values
    }

    /// Scalar branches and operation ordering from 0x00350868. No extra
    /// [0,1] clamp is introduced: the original correction can exceed unity.
    pub fn factor(&self, state: &CorrectionState) -> f32 {
        if state.bypass {
            return 1.0;
        }
        let p = &self.values;
        let x = state.input_at_04;
        let mut first = 1.0;
        if x < p[0] {
            first = 1.0 - p[1];
        } else if x < 0.0 {
            first = 1.0 - (x * p[1]) / p[0];
        } else if x > p[2] {
            if x < p[3] {
                first = 1.0 - ((x - p[2]) * p[4]) / (p[3] - p[2]);
            } else {
                first = 1.0 - p[4];
            }
        }
        let y = state.input_at_08;
        let second = if y < p[5] {
            p[6] + 1.0
        } else if y < 0.0 {
            (y * p[6]) / p[5] + 1.0
        } else if y > p[8] {
            let mut t = (y - p[8]) / (p[7] - p[8]);
            if 1.0 < t {
                t = 1.0;
            }
            1.0 - p[6] * t
        } else {
            1.0
        };
        first * second
    }
}

/// Wrapper 0x0034e9a0: lookup multiplied by correction. A null source state
/// selects the recovered constructor's bypassed state, not invented weather.
pub fn corrected_curve_value(
    curve: Curve<'_>,
    input: f32,
    state: Option<&CorrectionState>,
    correction: &EngineCorrection,
) -> f32 {
    let fallback = CorrectionState::default();
    let value = curve.sample(input);
    let factor = correction.factor(state.unwrap_or(&fallback));
    value * factor
}

/// Original engine-curve fields +0xdb/+0xdc/+0xe0 read by 0x0034ea68.
#[derive(Clone, Copy, Debug, Default)]
pub struct OutputLimit {
    pub enabled_at_db: bool,
    pub power_at_dc: f32,
    pub softness_at_e0: f32,
}

/// Original power/torque soft limiting at 0x0034ea68, including its condition
/// and final minimum against requested torque. No independent power cap added.
pub fn limit_engine_output(torque: f32, angular_speed: f32, limit: OutputLimit) -> f32 {
    let mut result = torque;
    if limit.enabled_at_db {
        let power = torque * angular_speed;
        let ceiling = limit.power_at_dc;
        let softness = limit.softness_at_e0;
        if ceiling * (1.0 - softness) < power {
            let mut adjusted = ceiling * (1.0 - softness * 0.5);
            if power < ceiling {
                adjusted -=
                    ((ceiling - power) * (ceiling - power)) / ((softness + softness) * ceiling);
            }
            result = adjusted / angular_speed;
            if torque <= adjusted / angular_speed {
                result = torque;
            }
        }
    }
    result
}

#[derive(Clone, Copy, Debug, Default)]
pub struct EngineOperatingState {
    pub torque_at_5ec: f32,
    pub drag_at_5f0: f32,
    pub rpm_at_5fe: i16,
    pub angular_speed_at_600: f32,
    pub limiter_at_625: u8,
}

/// 0x00355410: original RPM cutoff with 500-RPM release hysteresis, engine
/// braking subtraction and 0x0034ea68's optional configured output limiter.
pub fn operating_engine_torque(
    throttle: f32,
    state: &mut EngineOperatingState,
    redline_at_200: u16,
    limit: OutputLimit,
) -> f32 {
    if state.limiter_at_625 == 0 {
        if i32::from(state.rpm_at_5fe) >= i32::from(redline_at_200) {
            state.limiter_at_625 = 1;
        }
    } else if i32::from(state.rpm_at_5fe) < i32::from(redline_at_200) - 500 {
        state.limiter_at_625 = 0;
    }
    let mut gross = 0.0;
    if state.limiter_at_625 == 0 {
        gross = throttle * (state.torque_at_5ec + state.drag_at_5f0);
    }
    limit_engine_output(gross - state.drag_at_5f0, state.angular_speed_at_600, limit)
}

/// Naturally aspirated fast branch of 0x003554a0 with configured +0x230 == 0,
/// followed by 0x00355410. The caller separately owns 0x00354b78's fuel/history
/// bookkeeping; this routine returns the exact recovered numeric torque stage.
pub fn naturally_aspirated_torque(
    state: &mut super::drivetrain::RearDrivelineState,
    redline_at_200: u16,
    limit: OutputLimit,
) -> f32 {
    state.throttle_at_5f4 = state.throttle_at_598;
    state.flag_at_70a = 0;
    state.throttle_at_604 = state.throttle_at_598;
    let mut operating = EngineOperatingState {
        torque_at_5ec: state.curve_torque_at_5ec,
        drag_at_5f0: state.drag_at_5f0,
        rpm_at_5fe: state.rpm_at_5fe,
        angular_speed_at_600: state.angular_speed_at_600,
        limiter_at_625: state.limiter_at_625,
    };
    let torque =
        operating_engine_torque(state.throttle_at_598, &mut operating, redline_at_200, limit);
    state.limiter_at_625 = operating.limiter_at_625;
    torque
}

/// 0x34ea08, the engine drag query used each frame by 0x357700. The global
/// 0x620e14 chooses the original quadratic branch; the initialized M3 snapshot
/// has zero and therefore uses the linear branch. `terminal_angular_speed` is
/// the last original engine curve abscissa, not a guessed redline conversion.
pub fn drag_torque(
    angular_speed: f32,
    coefficient_at_d0: f32,
    quadratic_at_620e14: bool,
    terminal_angular_speed: f32,
) -> f32 {
    use super::eefloat::{add, div, mul};
    if quadratic_at_620e14 {
        let squared = mul(terminal_angular_speed, terminal_angular_speed);
        div(
            mul(
                coefficient_at_d0,
                add(
                    mul(angular_speed, angular_speed),
                    mul(squared, f32::from_bits(0x3de38e38)),
                ),
            ),
            squared,
        )
    } else {
        mul(coefficient_at_d0, angular_speed)
    }
}
