//! Planar four-contact-patch vehicle dynamics, integrated in SI units. Parameters
//! represent a GT-style car, not a manufacturer-calibrated simulator.
//!
//! The CG/rear-axle distinction and low-speed kinematic transition follow the
//! standard single-track model; see CommonRoad's vehicleDynamics_ST reference.
use crate::{Env, MAX_SPEED, MAX_STEER_ANGLE, PHYSICS_DT, UNITS_PER_METRE, WHEELBASE};

pub const MASS_KG: f64 = 1450.0;
pub const YAW_INERTIA: f64 = 2300.0; // kg m²
pub const GRIP: f64 = 1.35;
pub const GRAVITY: f64 = 9.81;
pub const FRONT_TRACK_METRES: f64 = 1.72;
pub const MAX_STEER_RATE: f64 = 1.2; // road wheel radians / second
const STEER_ACCEL: f64 = 6.0;
const CG_HEIGHT: f64 = 0.48;
const ENGINE_POWER: f64 = 390_000.0; // W, representative wheel power
const MAX_DRIVE_ACCEL: f64 = 9.0;
const MAX_BRAKE_ACCEL: f64 = 14.0;

/// Ackermann road-wheel angles in the renderer's positive/right direction.
/// The inside front wheel turns farther; rear wheels remain aligned with body.
pub fn front_wheel_angles(steer: f64) -> (f64, f64) {
    if steer.abs() < 1e-8 {
        return (0.0, 0.0);
    }
    let wheelbase = WHEELBASE / UNITS_PER_METRE;
    let radius = wheelbase / steer.tan();
    (
        (wheelbase / (radius + FRONT_TRACK_METRES * 0.5)).atan(),
        (wheelbase / (radius - FRONT_TRACK_METRES * 0.5)).atan(),
    )
}

/// Sublinear load sensitivity: transferring load to an outside tire reduces
/// total axle grip, even though that individual tire can carry more force.
fn tyre_capacity(normal: f64) -> f64 {
    let nominal = MASS_KG * GRAVITY * 0.25;
    GRIP * nominal * (normal.max(0.0) / nominal).powf(0.9)
}

impl Env {
    pub(crate) fn integrate_vehicle(&mut self, steer_cmd: f64, pedal_cmd: f64) {
        let dt = PHYSICS_DT;
        let mut vx = self.vx / UNITS_PER_METRE;
        let mut vy = self.vy / UNITS_PER_METRE;
        let old_vx = vx;
        let old_vy = vy;
        let wheelbase = WHEELBASE / UNITS_PER_METRE;
        let lf = wheelbase * 0.5;
        let lr = wheelbase * 0.5;
        let old_yaw = self.yaw_rate;

        let speed_ratio = self.vx.abs() / MAX_SPEED;
        let target = steer_cmd * MAX_STEER_ANGLE / (1.0 + 0.45 * speed_ratio * speed_ratio);
        // A driver's steering target is filtered before a critically damped
        // rack servo. Never snap the rack velocity to zero when a target moves
        // across it: that bypassed the acceleration limit under exploration.
        self.steer_target += (target - self.steer_target) * dt / (0.12 + dt);
        let error = self.steer_target - self.steer_angle;
        let accel = (196.0 * error - 28.0 * self.steer_rate).clamp(-STEER_ACCEL, STEER_ACCEL);
        self.steer_rate = (self.steer_rate + accel * dt).clamp(-MAX_STEER_RATE, MAX_STEER_RATE);
        self.steer_angle += self.steer_rate * dt;
        self.steer_angle = self.steer_angle.clamp(-MAX_STEER_ANGLE, MAX_STEER_ANGLE);
        self.pedal += (pedal_cmd - self.pedal) * dt / (0.12 + dt);

        // Four contact patches, with longitudinal pitch transfer and a lagged
        // lateral roll transfer. Equal left/right loading is no longer assumed.
        let weight = MASS_KG * GRAVITY;
        let downforce = 0.75 * (vx * vx + vy * vy);
        let transfer = MASS_KG * self.longitudinal_accel * CG_HEIGHT / wheelbase;
        let normal_f = (weight * lr / wheelbase - transfer).clamp(weight * 0.1, weight * 0.9)
            + downforce * 0.45;
        let normal_r = (weight * lf / wheelbase + transfer).clamp(weight * 0.1, weight * 0.9)
            + downforce * 0.55;
        let roll_target = MASS_KG * self.lateral_accel * CG_HEIGHT / FRONT_TRACK_METRES;
        self.lateral_load_transfer += (roll_target - self.lateral_load_transfer) * dt / (0.15 + dt);
        let front_shift =
            (self.lateral_load_transfer * 0.55).clamp(-normal_f * 0.45, normal_f * 0.45);
        let rear_shift =
            (self.lateral_load_transfer * 0.45).clamp(-normal_r * 0.45, normal_r * 0.45);
        self.tyre_loads = [
            normal_f * 0.5 + front_shift,
            normal_f * 0.5 - front_shift,
            normal_r * 0.5 + rear_shift,
            normal_r * 0.5 - rear_shift,
        ];
        let drive =
            (MASS_KG * MAX_DRIVE_ACCEL).min(ENGINE_POWER / vx.max(1.0)) * self.pedal.max(0.0);
        let brake =
            self.pedal.min(0.0) * MASS_KG * MAX_BRAKE_ACCEL * if vx < -0.1 { -1.0 } else { 1.0 };
        let wheel_x = [lf, lf, -lr, -lr];
        let half_track = FRONT_TRACK_METRES * 0.5;
        let wheel_y = [-half_track, half_track, -half_track, half_track];
        let (left, right) = front_wheel_angles(self.steer_angle);
        let angles = [left, right, 0.0, 0.0];
        let mut fx = 0.0;
        let mut fy = 0.0;
        let mut moment = 0.0;
        for i in 0..4 {
            let normal = self.tyre_loads[i];
            let limit = tyre_capacity(normal);
            let front = i < 2;
            let longitudinal = if self.pedal >= 0.0 {
                if front {
                    0.0
                } else {
                    drive * 0.5
                }
            } else {
                brake * if front { 0.31 } else { 0.19 }
            }
            .clamp(-limit, limit);
            let lateral_limit = (limit * limit - longitudinal * longitudinal)
                .max(0.0)
                .sqrt();
            // Wheel velocity includes yaw and its own Ackermann angle.
            let wheel_vx = vx - self.yaw_rate * wheel_y[i];
            let wheel_vy = vy + self.yaw_rate * wheel_x[i];
            let alpha = wheel_vy.atan2(wheel_vx.abs().max(0.5))
                - angles[i] * if wheel_vx < 0.0 { -1.0 } else { 1.0 };
            let stiffness =
                (if front { 45_000.0 } else { 50_000.0 }) * (normal / (weight * 0.25)).powf(0.8);
            let desired_lateral = if lateral_limit > 1e-6 {
                -lateral_limit * (stiffness * alpha / lateral_limit).tanh()
            } else {
                0.0
            };
            // Tire carcass/contact-patch deformation takes distance to build up.
            // A 0.45 m relaxation length, integrated implicitly, remains stable
            // near zero speed and for our fixed 120 Hz physics steps.
            let response = dt * wheel_vx.abs().max(2.0) / 0.45;
            self.tyre_lateral[i] +=
                (desired_lateral - self.tyre_lateral[i]) * response / (1.0 + response);
            self.tyre_lateral[i] = self.tyre_lateral[i].clamp(-lateral_limit, lateral_limit);
            self.tyre_usage[i] = longitudinal.hypot(self.tyre_lateral[i]) / limit.max(1.0);
            let (sin_s, cos_s) = angles[i].sin_cos();
            let wheel_fx = longitudinal * cos_s - self.tyre_lateral[i] * sin_s;
            let wheel_fy = self.tyre_lateral[i] * cos_s + longitudinal * sin_s;
            fx += wheel_fx;
            fy += wheel_fy;
            moment += wheel_x[i] * wheel_fy - wheel_y[i] * wheel_fx;
        }
        let rolling = weight * 0.015 * vx.signum();
        let air = 0.5 * 1.225 * vx.hypot(vy);
        fx -= air * vx + rolling;
        fy -= air * vy;
        self.longitudinal_accel = fx / MASS_KG;
        self.lateral_accel = fy / MASS_KG;
        self.yaw_accel = moment / YAW_INERTIA;
        // Integrate forces, then rotate velocity into the new body frame.
        // Exact rotation avoids the energy injection of explicit Coriolis steps.
        let force_vx = vx + self.longitudinal_accel * dt;
        let force_vx = if vx >= 0.0 && force_vx < 0.0 && vy.abs() < 0.2 {
            0.0
        } else {
            force_vx
        };
        let force_vy = vy + fy / MASS_KG * dt;
        let dynamic_yaw = self.yaw_rate + self.yaw_accel * dt;
        let dynamic_delta = (old_yaw + dynamic_yaw) * 0.5 * dt;
        let dynamic_vx = force_vx * dynamic_delta.cos() + force_vy * dynamic_delta.sin();
        let dynamic_vy = -force_vx * dynamic_delta.sin() + force_vy * dynamic_delta.cos();

        // At walking speeds a rolling rear axle is a better-conditioned model
        // than slip angles divided by tiny speed. Blend smoothly into dynamics.
        let blend = ((force_vx.hypot(force_vy) - 2.0) / 6.0).clamp(0.0, 1.0);
        let blend = blend * blend * (3.0 - 2.0 * blend);
        let kinematic_yaw = force_vx / wheelbase * self.steer_angle.tan();
        vx = blend * dynamic_vx + (1.0 - blend) * force_vx;
        vy = blend * dynamic_vy + (1.0 - blend) * lr * kinematic_yaw;
        self.yaw_rate = blend * dynamic_yaw + (1.0 - blend) * kinematic_yaw;
        if vx.hypot(vy) < 0.01 {
            vx = 0.0;
            vy = 0.0;
            self.yaw_rate = 0.0;
        }
        let speed = vx.hypot(vy);
        if speed > MAX_SPEED / UNITS_PER_METRE {
            let scale = MAX_SPEED / UNITS_PER_METRE / speed;
            vx *= scale;
            vy *= scale;
        }
        let delta_heading = (old_yaw + self.yaw_rate) * 0.5 * dt;
        let next_heading = self.heading + delta_heading;
        let old_world_x = old_vx * self.heading.cos() - old_vy * self.heading.sin();
        let old_world_y = old_vx * self.heading.sin() + old_vy * self.heading.cos();
        let new_world_x = vx * next_heading.cos() - vy * next_heading.sin();
        let new_world_y = vx * next_heading.sin() + vy * next_heading.cos();
        self.x += 0.5 * (old_world_x + new_world_x) * dt * UNITS_PER_METRE;
        self.y += 0.5 * (old_world_y + new_world_y) * dt * UNITS_PER_METRE;
        self.heading = crate::wrap_angle(self.heading + delta_heading);
        self.vx = vx * UNITS_PER_METRE;
        self.vy = vy * UNITS_PER_METRE;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lateral_load_transfer_reduces_total_grip_and_tires_do_not_instantly_load() {
        let nominal = MASS_KG * GRAVITY * 0.25;
        assert!(
            tyre_capacity(nominal * 1.5) + tyre_capacity(nominal * 0.5)
                < 2.0 * tyre_capacity(nominal)
        );
        let mut env = Env::new(crate::Circuit::Monza.build(), 1);
        env.vx = 20.0 * UNITS_PER_METRE;
        env.steer_angle = 0.1;
        env.steer_target = 0.1;
        let command = 0.1 * (1.0 + 0.45 * (env.vx / MAX_SPEED).powi(2)) / MAX_STEER_ANGLE;
        env.integrate_vehicle(command, 0.0);
        let limit = tyre_capacity(env.tyre_loads[0]);
        let expected_steady = limit * (45_000.0 * front_wheel_angles(0.1).0 / limit).tanh();
        assert!(env.tyre_lateral[0] > 0.0);
        assert!(
            env.tyre_lateral[0] < expected_steady * 0.5,
            "contact patch must take time to build force"
        );
    }

    #[test]
    fn steering_actuator_never_snaps_its_angular_velocity() {
        let mut env = Env::new(crate::Circuit::Monza.build(), 1);
        let mut largest_accel: f64 = 0.0;
        let mut rng = crate::Rng::new(42);
        let mut noise = [0.0f32];
        for tick in 0..1800 {
            if tick % 6 == 0 {
                rng.normals_f32(&mut noise);
            }
            let command = (0.4 * noise[0] as f64).tanh();
            let previous_rate = env.steer_rate;
            env.integrate_vehicle(command, 0.0);
            largest_accel = largest_accel.max((env.steer_rate - previous_rate).abs() / PHYSICS_DT);
        }
        assert!(
            largest_accel <= STEER_ACCEL + 1e-6,
            "steering angular acceleration spikes to {largest_accel:.2} rad/s²"
        );
    }

    #[test]
    fn steady_turn_has_finite_grip_limited_yaw_and_expected_radius() {
        let mut env = Env::new(crate::Circuit::Monza.build(), 1);
        let speed = 15.0;
        for _ in 0..1200 {
            env.vx = speed * UNITS_PER_METRE;
            env.integrate_vehicle(0.1, 0.15);
        }
        let expected_yaw = speed / (WHEELBASE / UNITS_PER_METRE) * env.steer_angle.tan();
        println!(
            "steady turn speed={speed} steer={} yaw={} kinematic={} vy={}",
            env.steer_angle,
            env.yaw_rate,
            expected_yaw,
            env.vy / UNITS_PER_METRE
        );
        assert!((env.yaw_rate / expected_yaw - 1.0).abs() < 0.25);
        for _ in 0..120 {
            env.vx = 30.0 * UNITS_PER_METRE;
            let previous_vy = env.vy / UNITS_PER_METRE;
            let previous_yaw = env.yaw_rate;
            env.integrate_vehicle(1.0, -0.5);
            let lateral_accel = (env.vy / UNITS_PER_METRE - previous_vy) / PHYSICS_DT
                + previous_yaw * env.vx / UNITS_PER_METRE;
            assert!(
                lateral_accel.abs() <= 16.0,
                "lateral acceleration {lateral_accel}"
            );
            assert!(env.yaw_rate.is_finite() && env.vy.is_finite());
        }
    }
}
