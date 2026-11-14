//! Tire scalar routines recovered from the GT4 per-wheel force pipeline.
//! These helpers alone do not reproduce suspension, wheel speed or body dynamics.

use super::slip::WheelSlipState;

/// 0x00352218 folds the supplied angle and emits a separate force sign. The
/// source performs one fold only; inputs beyond +/-pi are not repeatedly wrapped.
pub fn fold_slip_angle(mut angle: f32) -> (f32, f32) {
    let pi = f32::from_bits(0x40490fda);
    let half_pi = f32::from_bits(0x3fc90fda);
    let mut direction = 1.0;
    if angle < -half_pi {
        angle = super::eefloat::add(angle, pi);
    } else if angle < 0.0 {
        angle = -angle;
        direction = -1.0;
    } else if angle >= half_pi {
        angle = super::eefloat::add(pi, -angle);
        direction = -1.0;
    }
    (angle, direction)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ForceSplit {
    pub at_70: f32,
    pub at_74: f32,
    pub at_7c: f32,
    pub at_80: f32,
}

/// One loop body of 0x003518e8, after the recovered slip/gain calculation.
/// `at_54` and `at_4c` are the original wheel fields; gain configuration bytes
/// are original globals 0x620d8e / 0x620d8f. Their value two changes the output
/// selection as in the original; it is not treated as a boolean enable.
pub fn split_longitudinal_force(
    wheel: &WheelSlipState,
    at_54: f32,
    at_4c: f32,
    gain_mode_0: u8,
    gain_mode_1: u8,
) -> ForceSplit {
    use super::eefloat::mul;
    let mut force = mul(at_54, at_4c);
    if wheel.direction_at_45 < 0 {
        force = -force;
    }
    let corrected = mul(wheel.gain_at_3c, force);
    let (braking, accelerating, braking_corrected, accelerating_corrected) = if wheel.mode_at_44 < 1
    {
        (0.0, force, 0.0, corrected)
    } else {
        (force, 0.0, corrected, 0.0)
    };
    ForceSplit {
        at_7c: braking_corrected,
        at_80: accelerating_corrected,
        at_74: if gain_mode_0 == 2 {
            accelerating_corrected
        } else {
            accelerating
        },
        at_70: if gain_mode_1 == 2 {
            braking_corrected
        } else {
            braking
        },
    }
}

use super::curve::{Curve, PairedCurve};

/// The byte stores from 0x00442490. Bytes not written by that routine retain
/// their incoming value; initialization of the destination is the caller's job.
pub fn expand_compound_into(dst: &mut [u8; 0xa8], compound: &[u8; 90], volume: &[u8; 18]) {
    dst[0] = compound[1];
    dst[1] = compound[0];
    dst[2] = compound[10];
    dst[8] = 8;
    dst[9] = 6;
    dst[10] = 6;
    dst[11] = 4;
    dst[12] = 6;
    for (to, from, count) in [
        (0x10, 11, 8),
        (0x18, 19, 8),
        (0x20, 39, 6),
        (0x30, 45, 6),
        (0x40, 57, 6),
        (0x28, 63, 6),
        (0x38, 69, 6),
        (0x48, 81, 6),
        (0x5c, 2, 4),
        (0x60, 6, 4),
        (0x50, 27, 6),
        (0x56, 33, 6),
        (3, 87, 3),
    ] {
        dst[to..to + count].copy_from_slice(&compound[from..from + count]);
    }
    dst[0x64..0x6e].copy_from_slice(&volume[..10]);
    dst[0x6e..0x73].copy_from_slice(&volume[13..18]);
    dst[0x78] = volume[10];
    dst[0x79] = volume[12];
    dst[0x7a] = volume[11];
}

#[derive(Clone, Debug)]
pub struct Table {
    pub x: Vec<f32>,
    pub y: Vec<f32>,
}
impl Table {
    pub fn curve(&self) -> Curve<'_> {
        Curve::new(&self.x, &self.y).expect("decoded GT4 table must retain increasing finite knots")
    }
}

/// Defined output fields from 0x00361008, 0x00361220 and 0x003613d8.
/// This owns decoded tables; original pointers/descriptor flags are represented
/// by borrowed views instead of emulating the original process's addresses.
#[derive(Clone, Debug)]
pub struct TireTables {
    pub slip_mu_a: Table,
    pub slip_mu_b: Table,
    pub side: Table,
    pub corner_drag: Table,
    pub load: Table,
    pub negative_peak_at_164: f32,
    pub positive_peak_at_168: f32,
    pub positive_peak_mu_at_16c: f32,
    pub at_00: f32,
    pub at_04: f32,
    pub at_68: f32,
    pub terrain_at_08: [f32; 24],
}

impl TireTables {
    /// Decode the expanded runtime compound, preserving the original endpoint
    /// byte mutations and f32 operation order. Count/monotonicity checks are a
    /// safe Rust input boundary, not checks present in the original loaders.
    pub fn from_runtime(bytes: &mut [u8; 0xa8], input_scale: f32) -> Option<Self> {
        let negative = usize::from(bytes[9]);
        let positive = usize::from(bytes[10]);
        let side_count = usize::from(bytes[8]);
        let drag_count = usize::from(bytes[12]);
        let load_count = usize::from(bytes[11]);
        if negative == 0
            || positive == 0
            || negative > 8
            || positive > 8
            || negative + positive - 1 > 12
            || side_count == 0
            || side_count > 8
            || drag_count == 0
            || drag_count > 6
            || load_count == 0
            || load_count > 4
            || !input_scale.is_finite()
        {
            return None;
        }
        // 0x00361008: negative branch is written backwards. The common zero
        // slot is overwritten by the positive branch; descriptor starts +4.
        bytes[0x20] = 0;
        bytes[0x40] = bytes[0x48];
        bytes[0x30] = 0;
        bytes[0x28] = 0;
        bytes[0x38] = 0;
        let mut slip_x = vec![0.0; negative + positive];
        let mut mu_a = vec![0.0; negative + positive];
        let mut mu_b = vec![0.0; negative + positive];
        let mut negative_peak = 0.0;
        let mut largest = 0.0;
        for i in 0..negative {
            let j = negative - i;
            slip_x[j] = -f32::from(bytes[0x20 + i]) / 255.0;
            mu_a[j] = f32::from(bytes[0x30 + i]) / 200.0;
            if largest < mu_a[j] {
                negative_peak = slip_x[j];
                largest = mu_a[j];
            }
            mu_b[j] = f32::from(bytes[0x40 + i]) / 200.0;
        }
        let mut positive_peak = 0.0;
        let mut positive_peak_mu = 1.0;
        largest = 0.0;
        for i in 0..positive {
            let j = negative + i;
            slip_x[j] = f32::from(bytes[0x28 + i]) / 255.0;
            mu_a[j] = f32::from(bytes[0x38 + i]) / 200.0;
            if largest < mu_a[j] {
                positive_peak = slip_x[j];
                positive_peak_mu = mu_a[j];
                largest = mu_a[j];
            }
            mu_b[j] = f32::from(bytes[0x48 + i]) / 200.0;
        }
        let slip_x = slip_x[1..].to_vec();
        let mu_a = mu_a[1..].to_vec();
        let mu_b = mu_b[1..].to_vec();
        let slip_mu_a = Table {
            x: slip_x.clone(),
            y: mu_a,
        };
        let slip_mu_b = Table { x: slip_x, y: mu_b };

        // 0x00361220: side angle cap and corner-drag scale differ by one bit.
        let degrees = if bytes[2] == 0 { 90 } else { bytes[2] };
        let candidate = f32::from(degrees) * f32::from_bits(0x3c8efa34);
        let mut side_scale = f32::from_bits(0x3fc90fda);
        if candidate <= side_scale {
            side_scale = candidate;
        }
        bytes[0x10] = 0;
        bytes[0x18] = 0;
        let side = Table {
            x: (0..side_count)
                .map(|i| (f32::from(bytes[0x10 + i]) * side_scale) / 255.0)
                .collect(),
            y: (0..side_count)
                .map(|i| f32::from(bytes[0x18 + i]) / 200.0)
                .collect(),
        };
        bytes[0x50] = 0;
        bytes[0x56] = 0;
        let corner_drag = Table {
            x: (0..drag_count)
                .map(|i| (f32::from(bytes[0x50 + i]) * f32::from_bits(0x3fc90fd9)) / 255.0)
                .collect(),
            y: (0..drag_count)
                .map(|i| f32::from(bytes[0x56 + i]) / 200.0)
                .collect(),
        };
        // 0x003613d8: per-surface factors and load table.
        let nominal =
            f32::from(if bytes[0] == 0 { 100 } else { bytes[0] }) * f32::from_bits(0x3c23d70a);
        let mut terrain = [0.0; 24];
        for (i, out) in terrain.iter_mut().enumerate() {
            *out = if bytes[0x64 + i] == 0 {
                nominal
            } else {
                (f32::from(bytes[0x64 + i]) * f32::from_bits(0x3c23d70a)) * nominal
            };
        }
        let at_04 = f32::from(bytes[1]) * f32::from_bits(0x3c23d70a);
        let mut at_68 = input_scale / nominal;
        if at_68 > 1.0 {
            at_68 = 1.0;
        }
        bytes[0x5c] = 0;
        let load = Table {
            x: (0..load_count)
                .map(|i| (f32::from(bytes[0x5c + i]) * 20.0) * f32::from_bits(0x411ccccc))
                .collect(),
            y: (0..load_count)
                .map(|i| f32::from(bytes[0x60 + i]) / 200.0)
                .collect(),
        };
        for table in [&slip_mu_a, &slip_mu_b, &side, &corner_drag, &load] {
            Curve::new(&table.x, &table.y)?;
        }
        let at_00 = slip_mu_b.curve().sample(0.0) * f32::from_bits(0x411ccccc);
        Some(Self {
            slip_mu_a,
            slip_mu_b,
            side,
            corner_drag,
            load,
            negative_peak_at_164: negative_peak,
            positive_peak_at_168: positive_peak,
            positive_peak_mu_at_16c: positive_peak_mu,
            at_00,
            at_04,
            at_68,
            terrain_at_08: terrain,
        })
    }

    pub fn slip_pair(&self) -> PairedCurve<'_> {
        PairedCurve::new(&self.slip_mu_a.x, &self.slip_mu_a.y, &self.slip_mu_b.y)
            .expect("decoded GT4 slip tables share valid knots")
    }
}

/// Sparse wheel block used by the single-wheel body of 0x00352408. Fields retain
/// source offsets until the full game's type ownership has been reconstructed.
#[derive(Clone, Copy, Debug, Default)]
pub struct WheelForceState {
    pub angle_at_00: f32,
    pub offset_at_04: f32,
    pub correction_at_1c: f32,
    pub velocity_at_20: f32,
    pub velocity_at_28: f32,
    pub velocity_at_2c: f32,
    pub heading_at_30: f32,
    pub slip_at_34: f32,
    pub scale_at_4c: f32,
    pub limit_at_58: f32,
    pub curve_at_5c: f32,
    pub fade_at_60: f32,
    pub force_at_68: f32,
    pub force_at_6c: f32,
    pub car_force_slot: f32,
}

/// Source global/accessor values supplied explicitly instead of sharing game
/// process memory. `angle_mode` is the result of 0x003f7c80. The two mode bytes
/// come from the selected tire item's +0x1d9/+0x1da fields.
#[derive(Clone, Copy, Debug, Default)]
pub struct TireForceContext {
    pub angle_mode: i32,
    pub mode_at_1d9: u8,
    pub mode_at_1da: u8,
    pub velocity_gain_at_fcc: f32,
    pub wheel_kind: u8,
    pub side_mode: u8,
    pub car_angle_at_670: f32,
    pub car_angle_at_67c: f32,
    pub global_at_620764: f32,
    pub global_at_620768: f32,
}

/// Single-wheel body of 0x00352408. Lookup/count/selection and the final optional
/// per-car average are caller responsibilities. Returns the magnitude that the
/// source optionally accumulates, or None for its zero-fade early branch.
/// Uses recovered game trigonometry/angle helpers rather than host libm.
pub fn update_contact_force(
    wheel: &mut WheelForceState,
    tables: &TireTables,
    context: &TireForceContext,
) -> Option<f32> {
    use super::eefloat::{add, div, mul, sqrt};
    use super::math::{angle_difference, cos, sin};
    wheel.slip_at_34 = 0.0;
    if wheel.fade_at_60 == 0.0 {
        wheel.force_at_68 = 0.0;
        wheel.force_at_6c = 0.0;
        return None;
    }
    let saved_angle = wheel.angle_at_00;
    if context.angle_mode == 1 {
        wheel.angle_at_00 = super::eefloat::add(wheel.angle_at_00, wheel.correction_at_1c);
    } else if context.angle_mode == 2 {
        wheel.angle_at_00 = wheel.correction_at_1c;
    }
    let mut angle = wheel.angle_at_00;
    if wheel.velocity_at_28 < 0.0 {
        angle = super::eefloat::add(angle, f32::from_bits(0x40490fda));
    }
    wheel.slip_at_34 = angle_difference(angle, wheel.heading_at_30);
    let (magnitude, mut direction) = if context.mode_at_1d9 == 0 {
        fold_slip_angle(wheel.slip_at_34)
    } else {
        let cos_angle = cos(wheel.angle_at_00);
        let sin_angle = sin(wheel.angle_at_00);
        let mut value = super::eefloat::mul(
            super::eefloat::add(
                super::eefloat::mul(cos_angle, wheel.velocity_at_2c),
                super::eefloat::mul(sin_angle, wheel.velocity_at_28),
            ),
            context.velocity_gain_at_fcc,
        );
        let mut sign = 1.0;
        if value < 0.0 {
            value = -value;
            sign = -1.0;
        }
        (value, sign)
    };
    if ((context.mode_at_1d9 == 0 && context.mode_at_1da == 1)
        || (context.mode_at_1d9 != 0 && context.mode_at_1da == 0))
        && wheel.velocity_at_20 < 0.0
    {
        direction = -direction;
    }
    let curve = tables.side.curve().sample_linear_ee(magnitude);
    wheel.curve_at_5c = curve;
    let mut side = mul(direction, curve);
    let mut extra = 0.0;
    if context.wheel_kind == 2 {
        let mut delta = add(context.car_angle_at_670, -context.car_angle_at_67c);
        if context.side_mode == 1 {
            delta = add(delta, wheel.offset_at_04);
        } else {
            delta = add(delta, -wheel.offset_at_04);
        }
        delta = div(
            delta,
            mul(context.global_at_620764, f32::from_bits(0x3c8efa34)),
        );
        if delta > 1.0 {
            delta = 1.0;
        } else if delta < -1.0 {
            delta = -1.0;
        }
        extra = mul(delta, -context.global_at_620768);
    }
    if context.mode_at_1d9 == 0 {
        extra = mul(extra, wheel.fade_at_60);
        side = mul(side, wheel.fade_at_60);
    }
    if context.mode_at_1da == 1 {
        if side > wheel.limit_at_58 {
            side = wheel.limit_at_58;
        } else if side < -wheel.limit_at_58 {
            side = -wheel.limit_at_58;
        }
        wheel.force_at_68 = mul(side, wheel.scale_at_4c);
        wheel.car_force_slot = mul(extra, wheel.scale_at_4c);
        wheel.force_at_6c = 0.0;
    } else {
        let mut drag = tables.corner_drag.curve().sample_linear_ee(magnitude);
        if context.mode_at_1d9 == 0 {
            drag = mul(drag, wheel.fade_at_60);
        }
        let radius = sqrt(add(mul(side, side), mul(drag, drag)));
        if wheel.limit_at_58 < radius {
            let scale = div(wheel.limit_at_58, radius);
            drag = mul(drag, scale);
            side = mul(side, scale);
        }
        wheel.force_at_68 = mul(side, wheel.scale_at_4c);
        wheel.car_force_slot = mul(extra, wheel.scale_at_4c);
        wheel.force_at_6c = mul(drag, wheel.scale_at_4c);
    }
    wheel.angle_at_00 = saved_angle;
    Some(magnitude)
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadScaleState {
    pub camber_at_04: f32,
    pub normal_at_48: f32,
    pub scale_at_4c: f32,
    pub wear_at_50: f32,
    pub terrain_at_89: u8,
    pub flag_at_dd: u8,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LoadScaleContext {
    pub width_at_2d4: f32,
    pub axle_multiplier_at_fd0: f32,
    pub vehicle_mode: u8,
    pub wheel_side: u8,
    pub body_angle_at_670: f32,
    pub body_angle_at_67c: f32,
    pub tire_mode_at_1da: u8,
    /// Result of the original 0x00356ef0 wear/temperature stage.
    pub apply_wear_factor: bool,
    /// Result of 0x00467470 (exactly one for normal human control mode).
    pub control_grip_factor: f32,
}

/// Single-wheel numeric stage of 0x00357418, including 0x003573a8's division by
/// actual tire width. The load curve abscissa is not normalized static axle load.
/// Wear/temperature updates and original metadata selection remain with caller.
pub fn update_load_scale(
    wheel: &mut LoadScaleState,
    tables: &TireTables,
    camber_curve: Curve<'_>,
    context: &LoadScaleContext,
) {
    use super::eefloat::{add, div, mul};
    wheel.flag_at_dd = context.tire_mode_at_1da;
    let density = div(wheel.normal_at_48, context.width_at_2d4);
    let load_factor = tables.load.curve().sample_linear_ee(density);
    let mut normal = mul(wheel.normal_at_48, load_factor);
    if context.apply_wear_factor {
        normal = mul(normal, wheel.wear_at_50);
    }
    let roll = add(context.body_angle_at_670, -context.body_angle_at_67c);
    let mut angle = if context.vehicle_mode == 1 {
        0.0
    } else {
        let roll_angle = if context.wheel_side == 0 {
            -roll
        } else if context.wheel_side == 1 {
            roll
        } else {
            0.0
        };
        add(roll_angle, wheel.camber_at_04)
    };
    angle = camber_curve.sample_linear_ee(angle);
    let surface = tables.terrain_at_08[usize::from(wheel.terrain_at_89)];
    wheel.scale_at_4c = mul(
        mul(mul(normal, context.axle_multiplier_at_fd0), surface),
        angle,
    );
    wheel.scale_at_4c = mul(wheel.scale_at_4c, context.control_grip_factor);
}

/// Original 0x003575e0, called once per wheel by 0x00357690. Computes the
/// low-speed contact fade and heading together; zero fade returns zero heading.
pub fn contact_velocity_heading(velocity_at_28: f32, velocity_at_2c: f32) -> (f32, f32) {
    // This norm controls whether the source calls atan2 at all. Native per-
    // instruction rounding is required here: nearest rounding can cross the
    // 1 km/h branch for a neighboring f32 vector with nonzero lateral speed.
    use super::eefloat::{add, mul, sqrt};
    let speed = sqrt(add(
        mul(velocity_at_2c, velocity_at_2c),
        mul(velocity_at_28, velocity_at_28),
    ));
    let fade = if speed >= f32::from_bits(0x4031c71b) {
        1.0
    } else if speed <= f32::from_bits(0x3e8e38e3) {
        0.0
    } else {
        mul(
            add(speed, -f32::from_bits(0x3e8e38e3)),
            f32::from_bits(0x3ecccccf),
        )
    };
    let heading = if fade == 0.0 {
        0.0
    } else {
        super::math::atan2(-velocity_at_2c, velocity_at_28)
    };
    (heading, fade)
}
