//! Actual car yaw/lateral force aggregation and world velocity update.
//!
//! These are the planar car routines 0x00352848 and 0x00352c90, rather than the
//! separate generic rigid-body integrator. Names with offsets identify original
//! state; geometry and grip values must come from the matching car descriptor.

use super::eefloat::{add, div, mul};
use super::math;

#[derive(Clone, Copy, Debug, Default)]
pub struct WheelChassisState {
    pub steer_at_00: f32,
    pub secondary_angle_at_1c: f32,
    pub rim_speed_at_20: f32,
    pub forward_velocity_at_28: f32,
    pub lateral_velocity_at_2c: f32,
    pub velocity_heading_at_30: f32,
    pub grip_at_4c: f32,
    pub lateral_force_at_68: f32,
    pub corner_drag_at_6c: f32,
    pub drive_force_at_70: f32,
    pub brake_force_at_74: f32,
    pub body_lateral_force_at_d4: f32,
    pub force_direction_at_dd: u8,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct WheelGeometry {
    /// carData+0x28[model+0x2c[i]], with model+0x1c side sign applied.
    pub signed_half_track: f32,
    /// carData+0x10[model+0x2c[i]], with model+0x3c fore/aft sign applied.
    pub signed_axle_distance: f32,
    /// Original per-wheel additional lateral force car+0x534[i].
    pub additional_lateral_force: f32,
    /// tire[surface+2] * controlGripFactor * tire[0], in native order.
    pub normalization_grip: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ChassisState {
    pub lateral_acceleration_at_658: f32,
    pub yaw_rate_at_680: f32,
    pub yaw_acceleration_at_68c: f32,
    pub normalized_lateral_at_690: f32,
    pub yaw_filter_at_740: f32,
    pub yaw_filter_rate_at_744: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct ChassisParameters {
    pub mass_at_5d8: f32,
    pub inverse_yaw_inertia_at_data_118: f32,
    pub dt_at_54c: f32,
    /// Return value of 0x003f7c80: 1 adds the secondary angle; 2 replaces steer.
    pub angle_mode: i32,
}

/// Original 0x003522b0, including its low-speed contact/rim-speed conditions.
pub fn low_speed_lateral_force(wheel: &WheelChassisState) -> f32 {
    if wheel.grip_at_4c == 0.0 {
        return 0.0;
    }
    let rim = wheel.rim_speed_at_20;
    let rim_limit = f32::from_bits(0x3fb1_c71b);
    let difference = add(wheel.forward_velocity_at_28, -rim);
    let difference_limit = f32::from_bits(0x3f0e_38e3);
    if rim < -rim_limit
        || rim > rim_limit
        || difference < -difference_limit
        || difference > difference_limit
    {
        return 0.0;
    }
    let projected = add(
        mul(math::cos(wheel.steer_at_00), wheel.lateral_velocity_at_2c),
        mul(math::sin(wheel.steer_at_00), wheel.forward_velocity_at_28),
    );
    let projected_limit = f32::from_bits(0x3f8e_38e3);
    if projected > projected_limit || projected < -projected_limit {
        return 0.0;
    }
    mul(
        mul(
            math::sin(mul(projected, f32::from_bits(0x4034_f4ab))),
            math::cos(mul(rim, f32::from_bits(0x3f90_c3bc))),
        ),
        wheel.grip_at_4c,
    )
}

/// Original 0x00350d40, called with car+0x104. This updates diagnostic/filter
/// state at car+0x740/+0x744; yaw integration still uses car+0x68c directly.
pub fn update_yaw_filter(state: &mut ChassisState) {
    let dt = f32::from_bits(0x3c88_8888);
    let spring = mul(
        add(state.yaw_acceleration_at_68c, -state.yaw_filter_at_740),
        500.0,
    );
    let damping = mul(state.yaw_filter_rate_at_744, 20.0);
    state.yaw_filter_rate_at_744 =
        add(state.yaw_filter_rate_at_744, mul(add(spring, -damping), dt));
    state.yaw_filter_at_740 = add(
        state.yaw_filter_at_740,
        mul(state.yaw_filter_rate_at_744, dt),
    );
}

/// Original 0x00352848. `geometry` is in model wheel order, exactly as `wheels`.
pub fn aggregate_forces(
    state: &mut ChassisState,
    wheels: &mut [WheelChassisState],
    geometry: &[WheelGeometry],
    parameters: ChassisParameters,
    suspended: bool,
) {
    assert_eq!(wheels.len(), geometry.len());
    if suspended {
        state.normalized_lateral_at_690 = 0.0;
        state.lateral_acceleration_at_658 = 0.0;
        for wheel in wheels {
            wheel.body_lateral_force_at_d4 = 0.0;
        }
        return;
    }
    let mut lateral_sum = 0.0;
    let mut yaw_sum = 0.0;
    for (wheel, geometry) in wheels.iter_mut().zip(geometry) {
        let original_angle = wheel.steer_at_00;
        match parameters.angle_mode {
            1 => wheel.steer_at_00 = add(wheel.steer_at_00, wheel.secondary_angle_at_1c),
            2 => wheel.steer_at_00 = wheel.secondary_angle_at_1c,
            _ => {}
        }
        let longitudinal = add(wheel.drive_force_at_70, wheel.brake_force_at_74);
        let side = wheel.lateral_force_at_68;
        let (side_lateral, side_forward) = if wheel.force_direction_at_dd == 1 {
            (
                mul(-math::cos(wheel.steer_at_00), side),
                mul(-math::sin(wheel.steer_at_00), side),
            )
        } else {
            let heading_cos = math::cos(wheel.velocity_heading_at_30);
            let heading_sin = math::sin(wheel.velocity_heading_at_30);
            (
                add(
                    mul(-heading_cos, side),
                    mul(heading_sin, wheel.corner_drag_at_6c),
                ),
                add(
                    mul(-heading_sin, side),
                    mul(-heading_cos, wheel.corner_drag_at_6c),
                ),
            )
        };
        let cosine = math::cos(wheel.steer_at_00);
        let sine = math::sin(wheel.steer_at_00);
        let low_speed = low_speed_lateral_force(wheel);
        let lateral = add(
            add(
                add(
                    side_lateral,
                    mul(-cosine, geometry.additional_lateral_force),
                ),
                -mul(cosine, low_speed),
            ),
            -mul(sine, longitudinal),
        );
        wheel.body_lateral_force_at_d4 = lateral;
        lateral_sum = add(lateral_sum, lateral);
        let forward = add(
            add(
                add(side_forward, mul(-sine, geometry.additional_lateral_force)),
                -mul(sine, low_speed),
            ),
            mul(cosine, longitudinal),
        );
        wheel.steer_at_00 = original_angle;
        yaw_sum = add(
            yaw_sum,
            add(
                mul(geometry.signed_half_track, forward),
                -mul(geometry.signed_axle_distance, lateral),
            ),
        );
    }
    state.yaw_acceleration_at_68c = mul(yaw_sum, parameters.inverse_yaw_inertia_at_data_118);
    update_yaw_filter(state);
    state.yaw_rate_at_680 = add(
        state.yaw_rate_at_680,
        mul(state.yaw_acceleration_at_68c, parameters.dt_at_54c),
    );
    let tau = f32::from_bits(0x40c9_0fda);
    if state.yaw_rate_at_680 > tau {
        state.yaw_rate_at_680 = tau;
    } else if state.yaw_rate_at_680 < -tau {
        state.yaw_rate_at_680 = -tau;
    }
    let lateral_acceleration = div(lateral_sum, parameters.mass_at_5d8);
    state.lateral_acceleration_at_658 = lateral_acceleration;
    let mut grip_sum = 0.0;
    for wheel in geometry {
        grip_sum = add(grip_sum, wheel.normalization_grip);
    }
    state.normalized_lateral_at_690 = div(mul(lateral_acceleration, wheels.len() as f32), grip_sum);
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VelocityState {
    pub forward_at_640: f32,
    pub lateral_at_644: f32,
    pub world_xy_at_648: [f32; 2],
    pub longitudinal_acceleration_at_654: f32,
    pub lateral_acceleration_at_658: f32,
    pub vertical_rate_at_668: f32,
    pub yaw_rate_at_680: f32,
    pub normalized_lateral_at_690: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct VelocityContext {
    pub stop_at_5be: i16,
    pub held_at_5c0: bool,
    pub mode_at_56b: u8,
    pub mode_at_56a: u8,
    pub brake_at_590: f32,
    pub brake_at_594: f32,
    pub dt_at_54c: f32,
    pub forward_basis_xy_at_110: [f32; 2],
    pub lateral_basis_xy_at_11c: [f32; 2],
}

/// Original 0x00352c90, including brake-at-rest and explicit held-mode behavior.
/// `wheel_forces_at_60` is the native wheel+0x60 array, not wheel+0x70.
pub fn update_world_velocity(
    state: &mut VelocityState,
    context: VelocityContext,
    wheel_forces_at_60: &[f32],
    suspended: bool,
) {
    if context.stop_at_5be != 0 {
        state.world_xy_at_648 = [0.0; 2];
        state.vertical_rate_at_668 = 0.0;
        state.yaw_rate_at_680 = 0.0;
        return;
    }
    if suspended {
        return;
    }
    let can_hold_at_rest = !matches!(context.mode_at_56b, 1 | 2 | 4 | 8)
        && !matches!(context.mode_at_56a, 1 | 2)
        && state.forward_at_640 > -f32::from_bits(0x3e8e_38e3)
        && state.forward_at_640 < f32::from_bits(0x3e8e_38e3)
        && wheel_forces_at_60.iter().all(|&force| force == 0.0)
        && (context.brake_at_590 > 0.0 || context.brake_at_594 > 0.0);
    if context.held_at_5c0 || context.mode_at_56b == 8 || can_hold_at_rest {
        *state = VelocityState::default();
        return;
    }
    for i in 0..2 {
        state.world_xy_at_648[i] = state.world_xy_at_648[i]
            + state.longitudinal_acceleration_at_654
                * context.dt_at_54c
                * context.forward_basis_xy_at_110[i]
            + state.lateral_acceleration_at_658
                * context.dt_at_54c
                * context.lateral_basis_xy_at_11c[i];
    }
}
