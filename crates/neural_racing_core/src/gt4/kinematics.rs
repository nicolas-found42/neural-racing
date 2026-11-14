//! Contact velocity projection from 0x0035d920 / the four-wheel branch of
//! 0x0035d6b0. Road and body bases are distinct in the original car state.

use super::eefloat::{add, mul};

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactVelocity {
    pub forward_at_28: f32,
    pub lateral_at_2c: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactGeometry {
    /// carData+0x10 with the NEGATIVE of model+0x3c's fore/aft sign.
    pub yaw_lateral_lever: f32,
    /// carData+0x28 with model+0x1c's side sign (0 negative, 1 positive).
    pub signed_half_track: f32,
    /// carData+0x40, indexed by the wheel's axle metadata.
    pub height: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ContactContext {
    pub world_velocity_at_648: [f32; 3],
    pub road_forward_at_140: [f32; 3],
    pub road_lateral_at_14c: [f32; 3],
    pub yaw_rate_at_680: f32,
    pub roll_rate_at_688: f32,
    pub body_up_z_at_130: f32,
    pub roll_coupling_at_620784: bool,
}

/// Returns car+0x640/+0x644 and writes the wheel+0x28/+0x2c velocities.
/// The separate model-type-1 (motorcycle) lean branch is outside this API.
pub fn update_contact_velocities(
    wheels: &mut [ContactVelocity],
    geometry: &[ContactGeometry],
    context: ContactContext,
) -> (f32, f32) {
    assert_eq!(wheels.len(), geometry.len());
    let mut forward = 0.0;
    let mut lateral = 0.0;
    for i in 0..3 {
        forward = add(
            forward,
            mul(
                context.road_forward_at_140[i],
                context.world_velocity_at_648[i],
            ),
        );
        lateral = add(
            lateral,
            mul(
                context.road_lateral_at_14c[i],
                context.world_velocity_at_648[i],
            ),
        );
    }
    for (wheel, geometry) in wheels.iter_mut().zip(geometry) {
        let mut lateral_increment = mul(geometry.yaw_lateral_lever, context.yaw_rate_at_680);
        if context.roll_coupling_at_620784 {
            lateral_increment = add(
                lateral_increment,
                -mul(
                    mul(context.roll_rate_at_688, geometry.height),
                    context.body_up_z_at_130,
                ),
            );
        }
        wheel.lateral_at_2c = add(lateral, lateral_increment);
        wheel.forward_at_28 = add(
            forward,
            mul(geometry.signed_half_track, context.yaw_rate_at_680),
        );
    }
    (forward, lateral)
}
