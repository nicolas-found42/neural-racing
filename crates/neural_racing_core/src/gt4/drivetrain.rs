//! Individually recovered forward-body and wheel-reaction steps.
//! These do not replace the unrecovered engine/clutch/differential controller.
use super::math::{cos, sin};

#[derive(Clone, Copy, Debug, Default)]
pub struct LongitudinalState {
    pub dt_at_54c: f32,
    pub inverse_dt_at_550: f32,
    pub mass_at_5d8: f32,
    pub velocity_at_640: f32,
    pub increment_at_654: f32,
    pub force_at_668: f32,
    pub extra_force_at_6a8: f32,
    pub extra_force_at_6b4: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LongitudinalWheel {
    pub angle_at_00: f32,
    pub heading_at_30: f32,
    pub flag_at_46: u8,
    pub force_at_68: f32,
    pub force_at_6c: f32,
    pub force_at_70: f32,
    pub force_at_74: f32,
    pub flag_at_dd: u8,
    /// Car +0x300 indexed by the source's wheel/axle mapping byte.
    pub extra_mass_at_300: f32,
}

/// Complete numeric wheel loops and state stores from 0x00351fa8. Original
/// metadata pointer/axle selection is expressed by the supplied wheel slice.
/// The source's zero-crossing stop applies only to its second force phase.
pub fn update_longitudinal(state: &mut LongitudinalState, wheels: &[LongitudinalWheel]) {
    let dt = state.dt_at_54c;
    let mut first_force = 0.0;
    for wheel in wheels {
        first_force += cos(wheel.angle_at_00) * wheel.force_at_70;
    }
    state.force_at_668 = (state.force_at_668 + first_force) + state.extra_force_at_6b4;
    let mut extra_mass = 0.0;
    for wheel in wheels {
        if wheel.flag_at_46 == 0 {
            extra_mass += wheel.extra_mass_at_300;
        }
    }
    let inverse_mass = 1.0 / (extra_mass + state.mass_at_5d8);
    state.increment_at_654 += (state.force_at_668 + state.extra_force_at_6a8) * inverse_mass;
    state.velocity_at_640 += state.increment_at_654 * dt;
    if state.velocity_at_640 != 0.0 {
        let mut force = 0.0;
        for wheel in wheels {
            force += cos(wheel.angle_at_00) * wheel.force_at_74;
            let (projection, component) = if wheel.flag_at_dd == 1 {
                (sin(wheel.angle_at_00), wheel.force_at_68)
            } else {
                force -= sin(wheel.heading_at_30) * wheel.force_at_68;
                (cos(wheel.heading_at_30), wheel.force_at_6c)
            };
            force -= projection * component;
        }
        let acceleration = force * inverse_mass;
        let previous = state.velocity_at_640;
        let mut next = previous + acceleration * dt;
        if 0.0 < previous * next {
            state.force_at_668 += force;
            state.increment_at_654 += acceleration;
        } else {
            next = 0.0;
            state.increment_at_654 += state.inverse_dt_at_550 * -previous;
        }
        state.velocity_at_640 = next;
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WheelTorqueState {
    pub speed_at_20: f32,
    pub force_at_7c: f32,
    pub force_at_80: f32,
    pub brake_at_78: f32,
    pub torque_at_84: f32,
}

/// One wheel of 0x0036d230. `radius_at_2c4` is selected through the original
/// axle mapping by the caller. A stopped wheel does not gain reverse brake torque.
pub fn update_reaction_torque(wheel: &mut WheelTorqueState, radius_at_2c4: f32) {
    use super::eefloat::{add, mul};
    let mut torque = mul(-radius_at_2c4, add(wheel.force_at_7c, wheel.force_at_80));
    if wheel.speed_at_20 != 0.0 {
        if wheel.speed_at_20 > 0.0 {
            torque = add(torque, -wheel.brake_at_78);
        } else {
            torque = add(torque, wheel.brake_at_78);
        }
    }
    wheel.torque_at_84 = torque;
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AxleRotationState {
    pub carrier_speed_at_630: f32,
    pub drive_torque_at_638: f32,
    pub lock_at_628: u8,
}

/// Values selected through the original metadata/group indices before the
/// numeric axle loop. A missing paired wheel represents source sentinel 0x7f.
#[derive(Clone, Copy, Debug)]
pub struct AxleRotationConfig {
    pub first_wheel: usize,
    pub paired_wheel: Option<usize>,
    pub wheel_count_at_08: u8,
    pub kind_at_10: u8,
    pub preload_at_248: f32,
    pub drive_coefficient_at_1c: f32,
    pub coast_coefficient_at_258: f32,
    pub viscous_limit_at_14: f32,
    pub inverse_inertia_at_b4: f32,
    /// Includes the original +0x235/+0x237 group override selection.
    pub rotation_ratio_at_90: f32,
}

/// Numeric axle/wheel loops from 0x0036e4b8, including the original differential
/// modes 0/1/2/3/4/5, lock transitions and brake zero-crossing behavior. Axle
/// grouping, gear ratios and carrier speeds must come from the caller's actual
/// drivetrain state; this routine does not invent a torque split or gearbox.
pub fn update_wheel_rotation(
    wheels: &mut [WheelTorqueState],
    axles: &mut [AxleRotationState],
    configs: &[AxleRotationConfig],
    dt: f32,
) {
    use super::eefloat::{add, mul};
    assert_eq!(axles.len(), configs.len());
    for (axle, config) in axles.iter_mut().zip(configs) {
        let first = config.first_wheel;
        let second = config.paired_wheel.unwrap_or(first);
        assert!(first < wheels.len() && second < wheels.len());
        assert!((1..=2).contains(&config.wheel_count_at_08));
        let difference = add(wheels[first].speed_at_20, -wheels[second].speed_at_20);
        let mut kind = config.kind_at_10;
        let mut correction = None;
        match kind {
            3 | 4 => {
                let request = axle.drive_torque_at_638;
                let candidate = if request >= 0.0 {
                    mul(request, config.drive_coefficient_at_1c)
                } else {
                    mul(config.coast_coefficient_at_258, -request)
                };
                let capacity = if candidate < config.preload_at_248 {
                    config.preload_at_248
                } else {
                    candidate
                };
                if axle.lock_at_628 != 0 {
                    let torque_difference =
                        add(wheels[first].torque_at_84, -wheels[second].torque_at_84);
                    let twice = add(capacity, capacity);
                    if torque_difference <= twice && -twice <= torque_difference {
                        kind = 1;
                    } else {
                        axle.lock_at_628 = 0;
                    }
                }
                if kind != 1 {
                    correction = Some(capacity);
                }
            }
            5 => {
                let square = mul(difference, difference);
                let bounded = if config.viscous_limit_at_14 < square {
                    config.viscous_limit_at_14
                } else {
                    square
                };
                correction = Some(mul(config.drive_coefficient_at_1c, bounded));
            }
            _ => {}
        }
        if let Some(torque) = correction {
            if difference > 0.0 {
                wheels[first].torque_at_84 = add(wheels[first].torque_at_84, -torque);
                wheels[second].torque_at_84 = add(wheels[second].torque_at_84, torque);
            } else if difference < 0.0 {
                wheels[first].torque_at_84 = add(wheels[first].torque_at_84, torque);
                wheels[second].torque_at_84 = add(wheels[second].torque_at_84, -torque);
            }
        }
        if kind == 0 {
            for i in 0..config.wheel_count_at_08 {
                let index = if i == 0 { first } else { second };
                let wheel = &mut wheels[index];
                let previous = wheel.speed_at_20;
                let mut next = add(
                    previous,
                    mul(mul(wheel.torque_at_84, config.inverse_inertia_at_b4), dt),
                );
                if wheel.brake_at_78 != 0.0 {
                    if previous > 0.0 {
                        if next < 0.0 {
                            next = 0.0;
                        }
                    } else if previous < 0.0 {
                        if next > 0.0 {
                            next = 0.0;
                        }
                    } else {
                        next = 0.0;
                    }
                }
                wheel.speed_at_20 = next;
            }
        } else if kind == 1 {
            let mean = mul(config.rotation_ratio_at_90, axle.carrier_speed_at_630);
            wheels[second].speed_at_20 = mean;
            wheels[first].speed_at_20 = mean;
        } else {
            let half_difference = mul(difference, 0.5);
            let torque_difference = add(wheels[first].torque_at_84, -wheels[second].torque_at_84);
            let mut next_half = add(
                half_difference,
                mul(
                    mul(mul(torque_difference, config.inverse_inertia_at_b4), 0.5),
                    dt,
                ),
            );
            if mul(difference, next_half) < 0.0 {
                next_half = 0.0;
                if kind == 3 || kind == 4 {
                    axle.lock_at_628 = 1;
                }
            }
            let mean = mul(config.rotation_ratio_at_90, axle.carrier_speed_at_630);
            wheels[first].speed_at_20 = add(mean, next_half);
            wheels[second].speed_at_20 = add(mean, -next_half);
        }
    }
}

/// Original scalar clutch-slip gain at 0x0036d738.
pub fn clutch_slip_gain(mut difference: f32) -> f32 {
    if difference < 0.0 {
        difference = -difference;
    }
    difference -= f32::from_bits(0x42517082);
    if difference < 0.0 {
        difference = 0.0;
    }
    let mut gain = difference / f32::from_bits(0x44517082) + 1.0;
    if gain >= 1.125 {
        gain = 1.125;
    }
    gain
}

/// Resolved wheel-to-group metadata used by 0x0036d818.
#[derive(Clone, Copy, Debug)]
pub struct CarrierWheelGroup {
    pub group: usize,
    pub first_wheel: usize,
    pub paired_wheel: Option<usize>,
}

#[derive(Clone, Debug)]
pub struct RearDrivelineConfig {
    pub drive_mode_at_234: u8,
    pub gearbox_mode_at_246: u8,
    pub transfer_at_235: bool,
    pub driven_group_at_237: usize,
    pub group_kinds_at_244: [u8; 2],
    pub wheel_drive_signs_at_3c: Vec<i8>,
    pub gear_ratios_at_274: Vec<f32>,
    pub clutch_torque_at_2e4: f32,
    pub shaft_inertias_at_2f0: [f32; 2],
    pub engine_inverse_inertia_at_2fc: f32,
    pub engine_inertia_at_240: f32,
    pub group_ratios_at_2c4: [f32; 2],
    pub group_ratios_at_2cc: [f32; 2],
}

#[derive(Clone, Debug, Default)]
pub struct RearDrivelineState {
    pub dt_at_54c: f32,
    pub throttle_at_598: f32,
    pub curve_torque_at_5ec: f32,
    pub drag_at_5f0: f32,
    pub throttle_at_5f4: f32,
    pub rpm_at_5fe: i16,
    pub angular_speed_at_600: f32,
    pub throttle_at_604: f32,
    pub coupling_input_at_610: f32,
    pub coupling_input_at_614: f32,
    pub clutch_ratio_at_61c: f32,
    pub torque_magnitude_at_620: f32,
    pub gear_at_624: u8,
    pub limiter_at_625: u8,
    pub clutch_at_626: u8,
    pub carrier_speed_at_630: [f32; 2],
    pub drive_torque_at_638: [f32; 2],
    pub wheel_flags_at_46: Vec<u8>,
    pub flag_at_70a: u8,
}

/// Configuration-specific 0x0036d818 port for the native M3 GTR ALMS path:
/// drive mode zero, gearbox mode zero. These values were recovered from the
/// selected car's actual database and loaders. Other layouts fail closed instead
/// of silently receiving a made-up carrier-speed or torque split.
///
/// `engine_torque` is the original 0x003554a0 engine stage, called only while the
/// source clutch state is nonzero. The separate 0x00354ce8 metrics/history update
/// is outside this numeric coupling stage. Wheel-to-group maps are resolved by
/// caller from original metadata, preserving source accumulation order.
pub fn update_standard_rear_driveline<F: FnOnce(&mut RearDrivelineState) -> f32>(
    state: &mut RearDrivelineState,
    config: &RearDrivelineConfig,
    wheels: &[WheelTorqueState],
    wheel_groups: &[usize],
    carriers: &[CarrierWheelGroup],
    engine_torque: F,
) -> Result<(), &'static str> {
    if config.drive_mode_at_234 != 0 || config.gearbox_mode_at_246 != 0 {
        return Err("GT4 driveline configuration is outside the recovered M3 branch");
    }
    if config.driven_group_at_237 > 1
        || wheels.len() != wheel_groups.len()
        || wheel_groups.iter().any(|&group| group > 1)
        || state.wheel_flags_at_46.len() != wheels.len()
        || config.wheel_drive_signs_at_3c.len() != wheels.len()
        || usize::from(state.gear_at_624) >= config.gear_ratios_at_274.len()
        || carriers.iter().any(|g| {
            g.group > 1
                || g.first_wheel >= wheels.len()
                || g.paired_wheel.is_some_and(|i| i >= wheels.len())
        })
    {
        return Err("invalid resolved GT4 driveline metadata");
    }
    let mut transmitted = 0.0;
    if state.clutch_at_626 != 0 {
        for (flag, sign) in state
            .wheel_flags_at_46
            .iter_mut()
            .zip(&config.wheel_drive_signs_at_3c)
        {
            if *sign < 0 {
                *flag |= 8;
            }
        }
        let torque = engine_torque(state);
        transmitted = torque;
        if state.clutch_at_626 != 1 {
            if state.clutch_at_626 == 3 {
                transmitted = config.clutch_torque_at_2e4;
            } else {
                transmitted = state.curve_torque_at_5ec * f32::from_bits(0x3d4ccccc);
                if torque > 0.0 {
                    let gain =
                        clutch_slip_gain(state.coupling_input_at_610 - state.coupling_input_at_614);
                    transmitted += gain * torque;
                }
            }
            if state.coupling_input_at_610 < state.coupling_input_at_614 {
                transmitted = -transmitted;
            }
            state.angular_speed_at_600 +=
                ((torque - transmitted) * config.engine_inverse_inertia_at_2fc) * state.dt_at_54c;
        }
    }
    state.torque_magnitude_at_620 = if transmitted < 0.0 {
        -transmitted
    } else {
        transmitted
    };
    let gear_ratio = config.gear_ratios_at_274[usize::from(state.gear_at_624)];
    transmitted *= gear_ratio;
    let mut reactions = [0.0; 2];
    for (wheel, &group) in wheels.iter().zip(wheel_groups) {
        reactions[group] += wheel.torque_at_84;
    }
    if config.transfer_at_235 {
        let other = 1 - config.driven_group_at_237;
        reactions[other] = (reactions[other] * config.group_ratios_at_2c4[other])
            * config.group_ratios_at_2cc[config.driven_group_at_237];
    }
    let mut reflected_engine_inertia = 0.0;
    if state.clutch_at_626 == 1 {
        reflected_engine_inertia = config.engine_inertia_at_240 * gear_ratio;
        if state.gear_at_624 == 0 {
            reflected_engine_inertia = -reflected_engine_inertia;
        }
    }
    // Native drive mode zero always sends the engine term to group one.
    let inertias = [
        0.0,
        config.shaft_inertias_at_2f0[1] + reflected_engine_inertia,
    ];
    state.drive_torque_at_638 = [0.0, transmitted];
    for carrier in carriers {
        let group = carrier.group;
        if config.group_kinds_at_244[group] == 0 {
            continue;
        }
        let previous = state.carrier_speed_at_630[group];
        let mut delta = 0.0;
        if inertias[group] != 0.0 {
            delta = ((state.drive_torque_at_638[group] + reactions[group]) / inertias[group])
                * state.dt_at_54c;
        }
        let mut next = previous + delta;
        let other = carrier.paired_wheel.unwrap_or(carrier.first_wheel);
        if wheels[carrier.first_wheel].brake_at_78 != 0.0 || wheels[other].brake_at_78 != 0.0 {
            if next * previous < 0.0 || previous == 0.0 {
                next = 0.0;
            }
        }
        state.carrier_speed_at_630[group] = next;
    }
    Ok(())
}

/// One original 0x3534a0 wheel contribution. The surface coefficient is read
/// from the native 0x620984 table by the caller, without reinterpreting its units.
#[derive(Clone, Copy, Debug)]
pub struct RollingLoad {
    pub normal_at_48: f32,
    pub surface_coefficient: f32,
}

/// 0x3534a0 resets the longitudinal accumulators and prepares rolling resistance.
/// The descriptor at 0x620938 is supplied as its original decoded curve.
pub fn prepare_longitudinal(
    state: &mut LongitudinalState,
    wheels: &[RollingLoad],
    resistance_curve: super::curve::Curve<'_>,
) {
    state.force_at_668 = 0.0;
    state.increment_at_654 = 0.0;
    state.extra_force_at_6b4 = 0.0;
    let mut load = state.force_at_668;
    for wheel in wheels {
        load += wheel.normal_at_48 * wheel.surface_coefficient;
    }
    let speed = state.velocity_at_640;
    let resistance = if speed > 0.0 {
        -resistance_curve.sample_linear(speed)
    } else {
        resistance_curve.sample_linear(-speed)
    };
    state.extra_force_at_6b4 = resistance * load;
}

/// Numeric axle branch of 0x36d2f8, after the optional drive-mode-seven update.
/// The selected native M3 FR configuration has no mode-six/seven group and
/// therefore performs no torque adjustment in this routine.
#[derive(Clone, Copy, Debug)]
pub struct SteeringTorqueGroup {
    pub first_wheel: usize,
    pub paired_wheel: Option<usize>,
    pub kind_at_244: u8,
    pub preload_at_248: f32,
}

pub fn adjust_steering_torque(
    forward_at_640: f32,
    steering_at_56c: f32,
    wheels: &mut [WheelTorqueState],
    groups: &[SteeringTorqueGroup],
) {
    for group in groups {
        let Some(other) = group.paired_wheel else {
            continue;
        };
        if !(6..8).contains(&group.kind_at_244)
            || forward_at_640 < f32::from_bits(0x4031c71b)
            || forward_at_640 > f32::from_bits(0x4247ffff)
        {
            continue;
        }
        let mut weight = steering_at_56c.abs() * f32::from_bits(0x40747648);
        if weight > 1.0 {
            weight = 1.0;
        }
        weight *= group.preload_at_248;
        if weight == 0.0 {
            continue;
        }
        let first = group.first_wheel;
        let mut target = (wheels[first].speed_at_20 + wheels[other].speed_at_20) * 0.5;
        if steering_at_56c > 0.0 {
            target += target * 0.125;
        } else if steering_at_56c < 0.0 {
            target -= target * 0.125;
        }
        if wheels[other].speed_at_20 < target {
            wheels[other].torque_at_84 += weight * 0.4375;
            wheels[first].torque_at_84 -= weight * 0.5625;
        } else if wheels[other].speed_at_20 > target {
            wheels[other].torque_at_84 -= weight * 0.5625;
            wheels[first].torque_at_84 += weight * 0.4375;
        }
    }
}
