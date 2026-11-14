//! Four-wheel normal-force/posture preparation at 0x0036a758 and its numeric
//! helpers. Model metadata is decoded explicitly by the caller.
use super::{
    eefloat::{add, div, mul},
    math,
    suspension::*,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct NormalWheel {
    pub suspension: SuspensionState,
    pub alignment_at_04: f32,
    pub lateral_position_at_08: f32,
    pub body_lateral_force_at_d4: f32,
    pub normal_velocity_at_d8: f32,
    pub contact_transition_at_dc: u8,
    pub position_input_at_514: f32,
    pub body_force_at_524: f32,
    pub contact_normal: [f32; 3],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct NormalGeometry {
    pub axle: usize,
    pub side: u8,
    pub fore_aft: i8,
    pub paired_wheel: Option<usize>,
    pub half_track_at_data_28: f32,
    pub axle_distance_at_data_10: f32,
    pub radius_at_data_2c4: f32,
    pub roll_arm_at_data_ffc: f32,
    pub base_alignment_at_data_fe4: f32,
    pub travel_alignment_at_data_fec: f32,
    /// The original +0x10c/+0x110 factors, selected by steer sign/wheel index.
    pub ackermann_factors: [f32; 2],
}

#[derive(Clone, Copy, Debug)]
pub struct NormalParameters {
    pub suspension: [SuspensionParameters; 2],
    pub damper: [DamperGlobals; 2],
    /// Exact 0x0035d248 spring-rate outputs (normally suspension+0x14).
    pub spring_rate: [f32; 4],
    pub context: SuspensionContext,
    pub world_velocity_at_648: [f32; 3],
    pub up_at_128: [f32; 3],
    pub pitch_rate_at_684: f32,
    pub roll_rate_at_688: f32,
    pub steer_at_56c: f32,
    pub steer_alignment_at_data_1004: f32,
    pub lateral_acceleration_at_658: f32,
    pub longitudinal_force_at_668: f32,
    pub lateral_transfer_at_data_34: f32,
    pub longitudinal_transfer_at_data_38: f32,
    pub aero_at_6ac: f32,
    pub aero_at_6b0: f32,
    pub front_wheel_count: u8,
    pub rear_wheel_count: u8,
    pub local_transfer_at_620e11: bool,
    pub height_at_data_1060: f32,
}

/// Original 0x0036a048. Multiplication order is significant for signed travel.
pub fn anti_roll(first_travel: f32, second_travel: f32, coefficient: f32) -> f32 {
    let difference = add(first_travel, -second_travel);
    if difference < 0.0 {
        mul(mul(-coefficient, difference), difference)
    } else {
        mul(mul(coefficient, difference), difference)
    }
}

/// Original 0x0036a690 numeric geometry: travel changes the effective half-track.
pub fn lateral_position(side: u8, half_track: f32, arm: f32, travel: f32) -> f32 {
    if side == 2 {
        return 0.0;
    }
    let mut distance = half_track;
    if arm != 0.0 {
        distance = add(
            distance,
            -mul(add(1.0, -math::cos(math::atan2(travel, arm))), arm),
        );
    }
    if side == 0 {
        -distance
    } else {
        distance
    }
}

/// Four-wheel numeric branch of 0x0036a578. The travel input is f13 in the
/// original mixed float/integer ABI, verified at callsite 0x0036a9e0.
pub fn alignment(
    wheel_index: usize,
    geometry: NormalGeometry,
    steer: f32,
    travel: f32,
    steering_gain: f32,
) -> f32 {
    let base = add(
        geometry.base_alignment_at_data_fe4,
        mul(geometry.travel_alignment_at_data_fec, travel),
    );
    if geometry.fore_aft <= 0 || steering_gain == 0.0 {
        return base;
    }
    let mut contribution = mul(steer, steering_gain);
    if geometry.paired_wheel.is_some() {
        let positive = contribution >= 0.0;
        let factor = if (wheel_index == 0) == positive {
            geometry.ackermann_factors[0]
        } else {
            geometry.ackermann_factors[1]
        };
        contribution = if wheel_index == 0 {
            mul(contribution, factor)
        } else {
            -mul(factor, contribution)
        };
    }
    add(base, contribution)
}

/// Normal four-wheel body of 0x0036a758. The optional original external spring
/// callback stays explicit and is called only when its source flag is enabled.
/// Motorcycle-only steering/lean and special spring-damage selection are supplied
/// through their actual caller boundaries, not approximated here.
pub fn update_normals<F: FnMut(usize, f32) -> f32>(
    wheels: &mut [NormalWheel; 4],
    geometry: &[NormalGeometry; 4],
    parameters: &NormalParameters,
    mut external_force: F,
) {
    for (wheel, geometry) in wheels.iter_mut().zip(geometry) {
        let roll = mul(geometry.half_track_at_data_28, parameters.roll_rate_at_688);
        let pitch = mul(
            geometry.axle_distance_at_data_10,
            parameters.pitch_rate_at_684,
        );
        let mut vertical = if geometry.side == 0 {
            add(roll, 0.0)
        } else if geometry.side == 1 {
            add(0.0, -roll)
        } else {
            0.0
        };
        if geometry.fore_aft > 0 {
            vertical = add(vertical, pitch);
        } else if geometry.fore_aft < 0 {
            vertical = add(vertical, -pitch);
        }
        let velocity = parameters.world_velocity_at_648;
        let up = parameters.up_at_128;
        let normal = wheel.contact_normal;
        wheel.normal_velocity_at_d8 = add(
            add(
                mul(add(velocity[0], mul(up[0], vertical)), normal[0]),
                mul(add(velocity[1], mul(up[1], vertical)), normal[1]),
            ),
            mul(add(velocity[2], mul(up[2], vertical)), normal[2]),
        );
    }
    for (i, (wheel, geometry)) in wheels.iter_mut().zip(geometry).enumerate() {
        let was_contact = wheel.suspension.normal_at_48 > 0.0;
        let axle = geometry.axle;
        update_suspension(
            wheel.position_input_at_514,
            parameters.spring_rate[i],
            &mut wheel.suspension,
            &parameters.suspension[axle],
            &parameters.damper[axle],
            &parameters.context,
            |travel| external_force(i, travel),
        );
        let has_contact = wheel.suspension.normal_at_48 > 0.0;
        wheel.contact_transition_at_dc = match (was_contact, has_contact) {
            (false, false) => 0,
            (true, true) => 1,
            (false, true) => 2,
            (true, false) => 3,
        };
        let travel = add(
            wheel.suspension.travel_at_94,
            -parameters.suspension[axle].values[4],
        );
        wheel.lateral_position_at_08 = lateral_position(
            geometry.side,
            geometry.half_track_at_data_28,
            geometry.roll_arm_at_data_ffc,
            travel,
        );
        wheel.alignment_at_04 = alignment(
            i,
            *geometry,
            parameters.steer_at_56c,
            travel,
            parameters.steer_alignment_at_data_1004,
        );
    }
    let mut roll = [0.0; 2];
    for (i, geometry) in geometry.iter().enumerate() {
        if geometry.side == 0 {
            if let Some(pair) = geometry.paired_wheel {
                roll[geometry.axle] = anti_roll(
                    wheels[i].suspension.travel_at_94,
                    wheels[pair].suspension.travel_at_94,
                    parameters.suspension[geometry.axle].values[12],
                );
            }
        }
    }
    let lateral_transfer = if parameters.local_transfer_at_620e11 {
        0.0
    } else {
        mul(
            parameters.lateral_transfer_at_data_34,
            -parameters.lateral_acceleration_at_658,
        )
    };
    let longitudinal_transfer = mul(
        parameters.longitudinal_transfer_at_data_38,
        parameters.longitudinal_force_at_668,
    );
    for (wheel, geometry) in wheels.iter_mut().zip(geometry) {
        let p = &parameters.suspension[geometry.axle].values;
        let mut force = if wheel.suspension.normal_at_48 == 0.0 {
            -p[15]
        } else {
            wheel.suspension.spring_at_9c
        };
        if geometry.paired_wheel.is_some() {
            if geometry.side == 0 {
                force = add(force, -roll[geometry.axle]);
            } else if geometry.side == 1 {
                force = add(force, roll[geometry.axle]);
            }
        }
        let count;
        let aero;
        let direction;
        if geometry.fore_aft > 0 {
            count = parameters.front_wheel_count;
            aero = parameters.aero_at_6ac;
            direction = 1.0;
        } else if geometry.fore_aft < 0 {
            count = parameters.rear_wheel_count;
            aero = parameters.aero_at_6b0;
            direction = -1.0;
        } else {
            count = 0;
            aero = 0.0;
            direction = 0.0;
        }
        if count != 0 {
            let transfer = if count == 2 {
                longitudinal_transfer
            } else {
                mul(longitudinal_transfer, div(2.0, f32::from(count)))
            };
            force = add(
                add(force, -div(aero, f32::from(count))),
                mul(direction, transfer),
            );
        }
        if geometry.paired_wheel.is_some() {
            if !parameters.local_transfer_at_620e11 {
                if geometry.side == 0 {
                    force = add(force, lateral_transfer);
                } else if geometry.side == 1 {
                    force = add(force, -lateral_transfer);
                }
            } else {
                let transfer = mul(
                    div(
                        // 36ad4c/58/5c: subtract travel, add radius, then height.
                        add(
                            parameters.height_at_data_1060,
                            add(
                                add(wheel.position_input_at_514, -p[3]),
                                geometry.radius_at_data_2c4,
                            ),
                        ),
                        geometry.half_track_at_data_28,
                    ),
                    wheel.body_lateral_force_at_d4,
                );
                if geometry.side == 0 {
                    force = add(force, -transfer);
                } else if geometry.side == 1 {
                    force = add(force, transfer);
                }
            }
        }
        wheel.body_force_at_524 = force;
    }
}
