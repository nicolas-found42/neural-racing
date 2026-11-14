//! Normal four-wheel attitude/vertical update recovered at 0x0036b0c8, its
//! 0x0036af30 reaction helper, and 0x0036c9f8 body basis construction.

use super::{
    eefloat::{add, div, mul},
    math,
};

#[derive(Clone, Copy, Debug, Default)]
pub struct BodyBasis {
    pub forward_at_110: [f32; 3],
    pub lateral_at_11c: [f32; 3],
    pub up_at_128: [f32; 3],
}

/// Original 0x0036c9f8. At zero angles the forward axis is +Y and lateral +X;
/// the game's coordinate adapter must account for that convention explicitly.
pub fn body_basis(pitch: f32, roll: f32, heading: f32) -> BodyBasis {
    let sp = math::sin(pitch);
    let cp = math::cos(pitch);
    let sr = math::sin(roll);
    let cr = math::cos(roll);
    let sh = math::sin(heading);
    let ch = math::cos(heading);
    // 36ca7c..36cadc uses separate scalar MUL/ADD/SUB, not COP1 ACC.
    let ch_sp = mul(ch, sp);
    let sh_sp = mul(sh, sp);
    BodyBasis {
        forward_at_110: [mul(-sh, cp), mul(ch, cp), sp],
        lateral_at_11c: [
            add(mul(ch, cr), -mul(sh_sp, sr)),
            add(mul(sh, cr), mul(ch_sp, sr)),
            mul(-cp, sr),
        ],
        up_at_128: [
            add(mul(ch, sr), mul(sh_sp, cr)),
            add(mul(sh, sr), -mul(ch_sp, cr)),
            mul(cp, cr),
        ],
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AttitudeWheel {
    pub axle: usize,
    /// Model+0x1c: 0 contributes positively to roll, 1 negatively.
    pub side: u8,
    /// Model+0x3c: positive front, negative rear, zero center.
    pub fore_aft: i8,
    pub steer_at_00: f32,
    pub normal_at_48: f32,
    pub drive_at_70: f32,
    pub brake_at_74: f32,
    pub damper_at_a0: f32,
    pub force_at_524: f32,
    pub radius_at_data_2c4: f32,
    pub axle_distance_at_data_10: f32,
    pub reaction_coefficient_at_data_12c: f32,
    pub static_force_at_data_344: f32,
    /// World contact normal, after the source's {x, -z, y} conversion.
    pub contact_normal: [f32; 3],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AttitudeState {
    pub world_velocity_at_648: [f32; 3],
    pub vertical_acceleration_at_65c: f32,
    pub pitch_at_66c: f32,
    pub roll_at_670: f32,
    pub heading_at_674: f32,
    pub pitch_rate_at_684: f32,
    pub roll_rate_at_688: f32,
    pub basis: BodyBasis,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct AttitudeParameters {
    pub dt_at_54c: f32,
    pub vertical_mass_at_5dc: f32,
    pub pitch_coefficients_at_data_11c: [f32; 2],
    pub roll_coefficients_at_data_124: [f32; 2],
    pub vertical_mode_at_620dec: u8,
    pub road_normal_at_158: [f32; 3],
    pub aero_force_at_6ac: f32,
    pub aero_force_at_6b0: f32,
    pub special_type_at_data_1077: u8,
    pub position_z_at_10c: f32,
    pub road_z_at_13c: f32,
    pub height_at_data_3c: f32,
    pub special_force_at_data_1084: f32,
}

/// Original 0x0036af30, returning pitch contribution and vertical contribution.
pub fn wheel_reaction(wheels: &[AttitudeWheel]) -> (f32, f32) {
    let mut pitch = 0.0;
    let mut vertical = 0.0;
    for wheel in wheels {
        let reaction = mul(
            mul(
                math::cos(wheel.steer_at_00),
                add(wheel.drive_at_70, wheel.brake_at_74),
            ),
            wheel.radius_at_data_2c4,
        );
        pitch = add(pitch, mul(reaction, wheel.reaction_coefficient_at_data_12c));
        if wheel.fore_aft > 0 {
            vertical = add(vertical, div(reaction, wheel.axle_distance_at_data_10));
        } else if wheel.fore_aft < 0 {
            vertical = add(vertical, -div(reaction, wheel.axle_distance_at_data_10));
        }
    }
    (pitch, vertical)
}

/// Four-wheel body of 0x0036b0c8. `held` is the result of the original recovery
/// helper 0x0036ae20; ordinary manual driving has car+0x5be=0 and `held=false`.
/// Motorcycle-only gyro/lean forces are outside this API.
pub fn update_attitude(
    state: &mut AttitudeState,
    wheels: &[AttitudeWheel],
    parameters: AttitudeParameters,
    held: bool,
) {
    let (reaction_pitch, mut reaction_vertical) = wheel_reaction(wheels);
    if held {
        return;
    }
    let dt = parameters.dt_at_54c;
    let mut axle_forces = [0.0; 2];
    for wheel in wheels {
        axle_forces[wheel.axle] = add(axle_forces[wheel.axle], wheel.force_at_524);
    }
    for wheel in wheels {
        axle_forces[wheel.axle] = add(
            axle_forces[wheel.axle],
            if wheel.normal_at_48 == 0.0 {
                0.0
            } else {
                wheel.damper_at_a0
            },
        );
    }
    let mut pitch_rate = add(
        state.pitch_rate_at_684,
        mul(
            add(
                add(
                    mul(axle_forces[0], parameters.pitch_coefficients_at_data_11c[0]),
                    -mul(axle_forces[1], parameters.pitch_coefficients_at_data_11c[1]),
                ),
                reaction_pitch,
            ),
            dt,
        ),
    );
    let pitch_rate_limit = f32::from_bits(0x3fc9_0fda);
    if pitch_rate > pitch_rate_limit {
        pitch_rate = pitch_rate_limit;
    } else if pitch_rate < -pitch_rate_limit {
        pitch_rate = -pitch_rate_limit;
    }
    state.pitch_at_66c = add(state.pitch_at_66c, mul(pitch_rate, dt));
    let angle_limit = f32::from_bits(0x3f49_0fd9);
    if state.pitch_at_66c < -angle_limit {
        state.pitch_at_66c = -angle_limit;
        pitch_rate = 0.0;
    } else if state.pitch_at_66c > angle_limit {
        state.pitch_at_66c = angle_limit;
        pitch_rate = 0.0;
    }
    state.pitch_rate_at_684 = pitch_rate;
    let mut roll_forces = [0.0; 2];
    for wheel in wheels {
        if wheel.side == 0 {
            roll_forces[wheel.axle] = add(roll_forces[wheel.axle], wheel.force_at_524);
        } else if wheel.side == 1 {
            roll_forces[wheel.axle] = add(roll_forces[wheel.axle], -wheel.force_at_524);
        }
    }
    for wheel in wheels {
        let damper = if wheel.normal_at_48 == 0.0 {
            0.0
        } else {
            wheel.damper_at_a0
        };
        if wheel.side == 0 {
            roll_forces[wheel.axle] = add(roll_forces[wheel.axle], damper);
        } else if wheel.side == 1 {
            roll_forces[wheel.axle] = add(roll_forces[wheel.axle], -damper);
        }
    }
    let mut roll_rate = add(
        state.roll_rate_at_688,
        mul(
            add(
                mul(roll_forces[0], parameters.roll_coefficients_at_data_124[0]),
                mul(roll_forces[1], parameters.roll_coefficients_at_data_124[1]),
            ),
            dt,
        ),
    );
    let roll_rate_limit = f32::from_bits(0x4149_0fda);
    if roll_rate > roll_rate_limit {
        roll_rate = roll_rate_limit;
    } else if roll_rate < -roll_rate_limit {
        roll_rate = -roll_rate_limit;
    }
    state.roll_at_670 = add(state.roll_at_670, mul(roll_rate, dt));
    if state.roll_at_670 < -angle_limit {
        state.roll_at_670 = -angle_limit;
        roll_rate = 0.0;
    } else if state.roll_at_670 > angle_limit {
        state.roll_at_670 = angle_limit;
        roll_rate = 0.0;
    }
    state.roll_rate_at_688 = roll_rate;
    state.basis = body_basis(state.pitch_at_66c, state.roll_at_670, state.heading_at_674);
    let gravity = f32::from_bits(0x411c_cccc);
    if parameters.vertical_mode_at_620dec == 0 {
        let mut force = 0.0;
        for wheel in wheels {
            force = add(force, wheel.force_at_524);
        }
        for wheel in wheels {
            force = add(
                force,
                if wheel.normal_at_48 == 0.0 {
                    0.0
                } else {
                    wheel.damper_at_a0
                },
            );
        }
        let acceleration = div(
            add(force, reaction_vertical),
            parameters.vertical_mass_at_5dc,
        );
        state.vertical_acceleration_at_65c =
            add(acceleration, -mul(state.basis.up_at_128[2], gravity));
        // 36b74c multiplies acceleration by dt before the per-axis MUL.
        let increment = mul(acceleration, dt);
        for i in 0..3 {
            state.world_velocity_at_648[i] = add(
                state.world_velocity_at_648[i],
                mul(state.basis.up_at_128[i], increment),
            );
        }
        state.world_velocity_at_648[2] = add(state.world_velocity_at_648[2], -mul(dt, gravity));
    } else {
        let mut force = [0.0; 3];
        for wheel in wheels {
            let normal = if parameters.vertical_mode_at_620dec == 2 {
                wheel.contact_normal
            } else {
                parameters.road_normal_at_158
            };
            let mut projection = 0.0;
            for (a, b) in normal.iter().zip(state.basis.up_at_128) {
                projection = add(projection, mul(*a, b));
            }
            let load = add(wheel.normal_at_48, -wheel.static_force_at_data_344);
            projection = mul(projection, if load < 0.0 { 0.0 } else { load });
            for i in 0..3 {
                force[i] = add(force[i], mul(projection, normal[i]));
            }
        }
        reaction_vertical = add(
            reaction_vertical,
            -add(parameters.aero_force_at_6ac, parameters.aero_force_at_6b0),
        );
        if parameters.special_type_at_data_1077 == 7 {
            let clearance = add(
                add(parameters.position_z_at_10c, -parameters.road_z_at_13c),
                -add(parameters.height_at_data_3c, f32::from_bits(0x3dcc_cccc)),
            );
            if clearance <= 0.0 {
                reaction_vertical = add(reaction_vertical, -parameters.special_force_at_data_1084);
            } else if clearance <= f32::from_bits(0x3dcc_cccc) {
                reaction_vertical = add(
                    reaction_vertical,
                    -mul(
                        parameters.special_force_at_data_1084,
                        add(1.0, -mul(clearance, 10.0)),
                    ),
                );
            }
        }
        for (f, up) in force.iter_mut().zip(state.basis.up_at_128) {
            *f = add(*f, mul(reaction_vertical, up));
        }
        for f in &mut force {
            *f = div(*f, parameters.vertical_mass_at_5dc);
        }
        force[2] = add(force[2], -gravity);
        let mut vertical = 0.0;
        for i in 0..3 {
            vertical = add(vertical, mul(force[i], state.basis.up_at_128[i]));
            state.world_velocity_at_648[i] = add(state.world_velocity_at_648[i], mul(force[i], dt));
        }
        state.vertical_acceleration_at_65c = vertical;
    }
}
