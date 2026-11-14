//! Native standard gearbox/control preparation. Offsets preserve recovered state
//! ownership; unsupported gearbox configurations fail at the public boundary.
use super::drivetrain::{
    CarrierWheelGroup, RearDrivelineConfig, RearDrivelineState, WheelTorqueState,
};

#[derive(Clone, Debug, Default)]
pub struct StandardControlState {
    pub engine: RearDrivelineState,
    pub input_kind_at_546: u8,
    pub mode_at_56a: u8,
    pub mode_at_56b: u8,
    pub brake_at_590: f32,
    pub handbrake_at_594: f32,
    pub clutch_input_at_618: f32,
    pub latch_at_62a: u8,
    pub forward_at_640: f32,
    pub rpm_display_at_738: f32,
    pub rpm_display_rate_at_73c: f32,
    pub shift_request_at_774: i8,
}

/// Original 0x350e28 through the drive-mode-zero branch of 0x350da0.
fn coupling_target(state: &StandardControlState, config: &RearDrivelineConfig) -> f32 {
    let value = state.engine.carrier_speed_at_630[1]
        * config.gear_ratios_at_274[usize::from(state.engine.gear_at_624)];
    if value <= 0.0 {
        0.0
    } else {
        value
    }
}

fn admitted(state: &StandardControlState, config: &RearDrivelineConfig) -> bool {
    config.drive_mode_at_234 == 0
        && config.gearbox_mode_at_246 == 0
        && usize::from(state.engine.gear_at_624) < config.gear_ratios_at_274.len()
}

/// 0x350fb8 selected standard FR branch, called before brake preparation by
/// 0x3511a0. It resets the two coupling accumulators in the same caller order.
pub fn prepare_standard_control(
    state: &mut StandardControlState,
    config: &RearDrivelineConfig,
    idle_rpm_at_260: u16,
) -> Result<(), &'static str> {
    if !admitted(state, config) {
        return Err("unsupported native standard-control configuration");
    }
    state.engine.coupling_input_at_610 = 0.0;
    state.engine.coupling_input_at_614 = 0.0;
    match state.engine.clutch_at_626 {
        0 => {
            if state.clutch_input_at_618 != 0.0 {
                let speed = state.forward_at_640;
                let threshold = f32::from_bits(0x4031c71b);
                if (speed > threshold && state.engine.gear_at_624 != 0)
                    || (speed < -threshold && state.engine.gear_at_624 == 0)
                {
                    if state.engine.throttle_at_598 > 0.25 {
                        state.engine.clutch_at_626 = 2;
                    }
                } else if i32::from(idle_rpm_at_260) < i32::from(state.engine.rpm_at_5fe) {
                    state.engine.clutch_at_626 = 2;
                } else {
                    state.clutch_input_at_618 = 0.0;
                }
            }
        }
        1 => {
            if state.clutch_input_at_618 == 0.0 {
                state.engine.clutch_at_626 = 0;
            }
        }
        _ => {}
    }
    if matches!(state.engine.clutch_at_626, 2 | 3) {
        let input = if state.mode_at_56a != 1 && state.mode_at_56b == 0 {
            state.input_kind_at_546
        } else {
            0
        };
        state.engine.coupling_input_at_610 = state.engine.angular_speed_at_600;
        let target = coupling_target(state, config);
        state.engine.coupling_input_at_614 = target;
        if input != 3 && input != 6 && state.engine.clutch_at_626 == 3 {
            state.engine.throttle_at_598 = if target < state.engine.coupling_input_at_610 {
                0.0
            } else {
                1.0
            };
        }
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
pub struct BrakePreparationWheel {
    pub request_at_64: f32,
    pub normal_at_48: f32,
    pub multiplier_at_1010: f32,
    pub drive_sign_at_3c: i8,
    pub brake_at_78: f32,
    pub flags_at_46: u8,
}

/// Remaining 0x3511a0 standard-gearbox wheel stores after control preparation.
pub fn prepare_brakes(
    wheels: &mut [BrakePreparationWheel],
    handbrake_at_594: f32,
    handbrake_gain_at_1018: f32,
) {
    for wheel in wheels.iter_mut() {
        wheel.brake_at_78 = wheel.request_at_64 * wheel.multiplier_at_1010;
    }
    for wheel in wheels.iter_mut() {
        if wheel.drive_sign_at_3c < 0 {
            wheel.brake_at_78 += handbrake_at_594 * handbrake_gain_at_1018;
        }
    }
    for wheel in wheels {
        wheel.flags_at_46 = if wheel.brake_at_78 != 0.0 { 2 } else { 0 };
        if wheel.normal_at_48 == 0.0 {
            wheel.flags_at_46 |= 1;
        }
    }
}

/// 0x3519b8, selected standard FR gearbox-mode-zero branch. The callback is the
/// recovered engine-output routine, invoked only with disengaged clutch. Shift
/// requests are actual 0x35a5f8 stores, not automatic gear changes.
pub fn finish_standard_control<F: FnOnce(&mut RearDrivelineState) -> f32>(
    state: &mut StandardControlState,
    config: &RearDrivelineConfig,
    idle_rpm_at_202: u16,
    redline_at_200: u16,
    wheels: &mut [WheelTorqueState],
    carriers: &[CarrierWheelGroup],
    engine_torque: F,
) -> Result<(), &'static str> {
    if !admitted(state, config)
        || carriers.iter().any(|g| {
            g.group >= 2
                || g.first_wheel >= wheels.len()
                || g.paired_wheel.is_some_and(|i| i >= wheels.len())
        })
    {
        return Err("unsupported native standard-control configuration");
    }
    let rpm_scale = f32::from_bits(0x3dd6774e);
    if state.engine.clutch_at_626 == 0 {
        let torque = engine_torque(&mut state.engine);
        state.engine.angular_speed_at_600 +=
            torque * config.engine_inverse_inertia_at_2fc * state.engine.dt_at_54c;
        let idle = f32::from(idle_rpm_at_202) * rpm_scale;
        if state.engine.angular_speed_at_600 < idle {
            state.engine.angular_speed_at_600 = idle;
        }
    } else {
        let target = coupling_target(state, config);
        let mut current = state.engine.angular_speed_at_600;
        match state.engine.clutch_at_626 {
            1 => {
                state.engine.angular_speed_at_600 = target;
                current = target;
            }
            2 | 3 => {
                let input = if state.mode_at_56a != 1 && state.mode_at_56b == 0 {
                    state.input_kind_at_546
                } else {
                    0
                };
                if matches!(input, 3 | 6) && state.clutch_input_at_618 == 1.0 {
                    let mut speed = state.engine.angular_speed_at_600
                        / config.gear_ratios_at_274[usize::from(state.engine.gear_at_624)];
                    for carrier in carriers {
                        state.engine.carrier_speed_at_630[carrier.group] = speed;
                        speed *= config.group_ratios_at_2c4[carrier.group];
                        wheels[carrier.first_wheel].speed_at_20 = speed;
                        if let Some(other) = carrier.paired_wheel {
                            wheels[other].speed_at_20 = speed;
                        }
                    }
                    state.engine.clutch_at_626 = 1;
                } else {
                    let a = state.engine.coupling_input_at_610;
                    let b = state.engine.coupling_input_at_614;
                    if (a <= b && target <= current) || (b <= a && current <= target) {
                        state.engine.clutch_at_626 = 1;
                        state.latch_at_62a = 12;
                        state.engine.angular_speed_at_600 = target;
                        current = target;
                    }
                }
                if state.engine.clutch_at_626 == 3
                    && state.engine.coupling_input_at_614 <= f32::from(idle_rpm_at_202) * rpm_scale
                {
                    state.engine.clutch_at_626 = 0;
                }
                if state.engine.clutch_at_626 == 2
                    && state.brake_at_590 > 0.75
                    && (current < (u32::from(idle_rpm_at_202) + 500) as f32 * rpm_scale
                        || state.forward_at_640.abs() < f32::from_bits(0x4031c71b))
                {
                    state.engine.clutch_at_626 = 0;
                }
            }
            _ => {}
        }
        if current < 0.0 {
            state.engine.angular_speed_at_600 = 0.0;
        } else {
            let limit = (u32::from(redline_at_200) + 10) as f32 * rpm_scale;
            if limit <= current {
                state.engine.angular_speed_at_600 = limit;
            }
        }
    }
    if state.engine.clutch_at_626 != 3 {
        state.shift_request_at_774 = 0;
    }
    let rpm = ((state.engine.angular_speed_at_600 * 60.0) / f32::from_bits(0x40c90fda)) as i32;
    state.engine.rpm_at_5fe = rpm.max(i32::from(idle_rpm_at_202) - 1) as i16;
    Ok(())
}

/// 0x350ed0: native filtered RPM display, after gearbox followthrough.
pub fn update_rpm_display(state: &mut StandardControlState) {
    let old = state.rpm_display_rate_at_73c;
    let next = old
        + (((f32::from(state.engine.rpm_at_5fe) - state.rpm_display_at_738) * 1000.0 - old * 50.0)
            * state.engine.dt_at_54c);
    state.rpm_display_rate_at_73c = next;
    state.rpm_display_at_738 += next * state.engine.dt_at_54c;
}

/// 0x351dc8 after the original 0x36c8e8 desired-gear query. A value of -1 is
/// the native no-change result. This stage records the shift request and calls
/// the native 0x36beb8 gear store; computing desired gear remains a separate
/// controller stage, not a guessed RPM threshold.
pub fn apply_desired_gear(state: &mut StandardControlState, desired: i32, owner_type_at_32: u8) {
    if desired == -1 {
        return;
    }
    if state.engine.clutch_at_626 != 0 {
        if desired == 0 || state.engine.gear_at_624 == 0 {
            state.engine.clutch_at_626 = 2;
        } else {
            // Native 3f1f88 is false for the admitted standard gearbox mode0.
            state.engine.clutch_at_626 = 3;
            if i32::from(state.engine.gear_at_624) < desired {
                state.shift_request_at_774 = 1;
            } else if desired < i32::from(state.engine.gear_at_624) {
                state.shift_request_at_774 = -1;
            }
        }
    }
    if owner_type_at_32 != 4 || desired != 0 {
        state.engine.gear_at_624 = desired as u8;
    }
}
