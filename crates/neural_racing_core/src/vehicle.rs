//! Coordinate and control adapter for the recovered stock GT4 M3 model.
//! Native dynamics live in `gt4`; manual and learned controls share this path.
use crate::{
    gt4::{
        parameters::NativeParameters, runtime::RoadContact, steering, steering_frame::AnalogInput,
    },
    Env, UNITS_PER_METRE,
};

pub const MASS_KG: f64 = 1190.0;
pub const YAW_INERTIA: f64 = 2300.0;
/// Conservative course-speed/observation scale, not a tire-force coefficient.
pub const GRIP: f64 = 1.35;
pub const GRAVITY: f64 = 9.8;
pub const FRONT_TRACK_METRES: f64 = 1.608;
pub const REAR_TRACK_METRES: f64 = 1.603;
/// Observation normalization only; the native controller determines rack rate.
pub const MAX_STEER_RATE: f64 = 1.2;

/// Native analog-controller steering table, in road-wheel radians.
pub fn steering_limit(speed: f64) -> f64 {
    let p = NativeParameters::m3();
    f64::from(
        p.steering_table(0x58)
            .sample((speed.max(0.0) / UNITS_PER_METRE) as f32),
    )
}

/// Original front-wheel Ackermann map used by the visualizer.
pub fn front_wheel_angles(steer: f64) -> (f64, f64) {
    let p = NativeParameters::m3();
    let angle = |side| {
        f64::from(steering::wheel_angle(
            steer as f32,
            1,
            side,
            [p.f32(0x10c), p.f32(0x110)],
            false,
            0.0,
            p.f32(0x1008),
        ))
    };
    (angle(1), angle(0))
}
fn query_track_road(
    track: &crate::track::Track,
    origin: [f64; 3],
    height_origin: f32,
    x: f32,
    y: f32,
    z: f32,
) -> Option<RoadContact> {
    let (sin, cos) = origin[2].sin_cos();
    let wx = origin[0] / UNITS_PER_METRE + f64::from(x) * cos - f64::from(y) * sin;
    let wy = origin[1] / UNITS_PER_METRE + f64::from(x) * sin + f64::from(y) * cos;
    let hit = track.road_contact(wx as f32, wy as f32, z + height_origin)?;
    Some(RoadContact {
        height_metres: hit.height_metres - height_origin,
        normal: [
            (f64::from(hit.normal[0]) * cos + f64::from(hit.normal[1]) * sin) as f32,
            (-f64::from(hit.normal[0]) * sin + f64::from(hit.normal[1]) * cos) as f32,
            hit.normal[2],
        ],
        material_id: hit.material_id,
        flags: hit.flags,
        attribute14: hit.attribute14,
        attribute15: hit.attribute15,
        course_position: [wx as f32, -wy as f32],
    })
}

impl Env {
    fn planar_state(&self) -> [f64; 8] {
        [
            self.x,
            self.y,
            self.heading,
            self.vx,
            self.vy,
            self.yaw_rate,
            self.steer_angle,
            self.steer_rate,
        ]
    }

    /// Reinitializes native state after a race reset or an explicit public-state
    /// edit by a replay/test driver. A local origin retains precise track placement.
    pub(crate) fn reset_vehicle(&mut self) {
        self.vehicle_origin = [self.x, self.y, self.heading - std::f64::consts::FRAC_PI_2];
        // An explicit edit can replace the pose between decisions, leaving
        // `prev_arc` on the discarded position. Reacquire it so the road query
        // starts at the elevation actually under the car.
        self.prev_arc = self.track.nearest_on_centerline(self.position()).arc;
        self.vehicle_height_origin = self.track.arc_height_metres(self.prev_arc);
        let condition = match self.track.course_condition() {
            crate::course_catalog::CourseCondition::Dry => 0,
            crate::course_catalog::CourseCondition::Wet => 1,
            crate::course_catalog::CourseCondition::Dirt => 2,
            crate::course_catalog::CourseCondition::Snow => 3,
            crate::course_catalog::CourseCondition::Unknown => {
                self.off_track = true;
                return;
            }
        };
        self.native_vehicle
            .set_course_condition(condition)
            .expect("known original course condition");
        self.native_vehicle
            .reset_flat(0.0, 0.0, 0.0, (self.vx / UNITS_PER_METRE) as f32);
        let c = &mut self.native_vehicle.car;
        c.set_f32(0x644, (-self.vy / UNITS_PER_METRE) as f32);
        c.set_f32(0x648, (-self.vy / UNITS_PER_METRE) as f32);
        c.set_f32(0x64c, (self.vx / UNITS_PER_METRE) as f32);
        c.set_f32(0x680, self.yaw_rate as f32);
        c.set_f32(0x56c, self.steer_angle as f32);
        c.set_f32(0x588, self.steer_rate as f32);
        let origin = self.vehicle_origin;
        let height = self.vehicle_height_origin;
        let track = &self.track;
        if self
            .native_vehicle
            .refresh_queried_contacts(&mut |x, y, z| {
                query_track_road(track, origin, height, x, y, z)
            })
            .is_err()
        {
            self.off_track = true;
        }
        self.vehicle_snapshot = self.planar_state();
    }

    pub(crate) fn integrate_vehicle(&mut self, steer_cmd: f64, pedal_cmd: f64) {
        if self.off_track {
            return;
        }
        if self.planar_state() != self.vehicle_snapshot {
            self.reset_vehicle();
        }
        let input = AnalogInput {
            steering: (steer_cmd.clamp(-1.0, 1.0) * 4096.0).round() as i16,
            throttle: (pedal_cmd.clamp(0.0, 1.0) * 4096.0).round() as u16,
            brake: ((-pedal_cmd).clamp(0.0, 1.0) * 4096.0).round() as u16,
            ..Default::default()
        };
        let origin = self.vehicle_origin;
        let track = &self.track;
        let height = self.vehicle_height_origin;
        if self
            .native_vehicle
            .step_with_road(input, &mut |x, y, z| {
                query_track_road(track, origin, height, x, y, z)
            })
            .is_err()
        {
            self.off_track = true;
            return;
        }
        let c = &self.native_vehicle.car;
        let theta = self.vehicle_origin[2];
        let (sin, cos) = theta.sin_cos();
        let x = f64::from(c.f32(0x104)) * UNITS_PER_METRE;
        let y = f64::from(c.f32(0x108)) * UNITS_PER_METRE;
        self.x = self.vehicle_origin[0] + x * cos - y * sin;
        self.y = self.vehicle_origin[1] + x * sin + y * cos;
        self.heading =
            crate::wrap_angle(theta + f64::from(c.f32(0x674)) + std::f64::consts::FRAC_PI_2);
        self.vx = f64::from(c.f32(0x640)) * UNITS_PER_METRE;
        self.vy = -f64::from(c.f32(0x644)) * UNITS_PER_METRE;
        self.yaw_rate = f64::from(c.f32(0x680));
        self.steer_angle = f64::from(c.f32(0x56c));
        self.steer_rate = f64::from(c.f32(0x588));
        self.steer_target = steer_cmd * steering_limit(self.vx);
        self.pedal = f64::from(c.f32(0x58c) - c.f32(0x590));
        self.longitudinal_accel = f64::from(c.f32(0x654));
        self.lateral_accel = -f64::from(c.f32(0x658));
        self.yaw_accel = f64::from(c.f32(0x68c));
        let p = &self.native_vehicle.parameters;
        for i in 0..4 {
            let native_index = [1, 0, 3, 2][i];
            let w = c.wheel(native_index);
            self.tyre_loads[i] = f64::from(w.f32(0x48));
            self.tyre_lateral[i] = -f64::from(w.f32(0x68));
            // Diagnostic grip demand relative to the selected compound peak;
            // this ratio is never fed back into native force calculations.
            let tables = p.tire_for_wheel(native_index, usize::from(w.u8(0x89)));
            let peak = tables.positive_peak_mu_at_16c.abs().max(1e-6) * w.f32(0x4c);
            self.tyre_usage[i] =
                f64::from(w.f32(0x68).hypot(w.f32(0x70) + w.f32(0x74)) / peak.max(1e-6));
        }
        self.lateral_load_transfer = (self.tyre_loads[0] - self.tyre_loads[1] + self.tyre_loads[2]
            - self.tyre_loads[3])
            * 0.5;
        self.vehicle_snapshot = self.planar_state();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::WHEELBASE;

    #[test]
    fn elevated_road_query_transforms_positions_normals_and_wet_materials() {
        use crate::{road_mesh::RoadMesh, track::RecoveredStation, Circuit, Track, Vec2};
        let mut mesh = Vec::new();
        mesh.extend(4u32.to_le_bytes());
        mesh.extend(2u32.to_le_bytes());
        for [x, y] in [
            [90.0f32, 190.0],
            [110.0, 190.0],
            [110.0, 210.0],
            [90.0, 210.0],
        ] {
            let height = 30.0 + 0.1 * (x - 100.0) + 0.2 * (y - 200.0);
            for f in [x, height, -y] {
                mesh.extend(f.to_le_bytes());
            }
            mesh.extend([0, 0, 255, 127]);
        }
        for indices in [[0u16, 1, 2], [0, 2, 3]] {
            for i in indices {
                mesh.extend(i.to_le_bytes());
            }
            mesh.extend([7, 2, 0, 0, 0, 0]);
        }
        let stations = [95.0, 100.0, 105.0]
            .into_iter()
            .enumerate()
            .map(|(i, x)| RecoveredStation {
                arc: i as f64 * 5.0 * UNITS_PER_METRE,
                point: Vec2 {
                    x: x * UNITS_PER_METRE,
                    y: 200.0 * UNITS_PER_METRE,
                },
                height: (30.0 + 0.1 * (x - 100.0)) as f32,
                left: 5.0 * UNITS_PER_METRE,
                right: 5.0 * UNITS_PER_METRE,
                edges: None,
            })
            .collect();
        let track = Track::from_recovered(
            Circuit::Monza,
            false,
            stations,
            10.0 * UNITS_PER_METRE,
            RoadMesh::decode(&mesh).unwrap(),
        )
        .unwrap();
        let hit = query_track_road(
            &track,
            [
                100.0 * UNITS_PER_METRE,
                200.0 * UNITS_PER_METRE,
                std::f64::consts::FRAC_PI_2,
            ],
            30.0,
            2.0,
            3.0,
            1.0,
        )
        .unwrap();
        assert!((hit.height_metres - 0.1).abs() < 1e-5);
        let length = 1.05f32.sqrt();
        assert!((hit.normal[0] + 0.2 / length).abs() < 1e-6);
        assert!((hit.normal[1] - 0.1 / length).abs() < 1e-6);
        assert!((hit.normal[2] - 1.0 / length).abs() < 1e-6);
        assert_eq!(hit.course_position, [97.0, -202.0]);
        assert_eq!(
            crate::gt4::runtime::contact_material(hit.material_id, hit.flags),
            20
        );
        let mut env = Env::new(track, 10);
        assert!((env.vehicle_height_origin - 29.5).abs() < 1e-5);
        assert!(!env.off_track);
        assert_eq!(env.native_vehicle.car.wheel(0).u8(0x89), 20);
        let result = env.step(crate::Action {
            steer: 0.0,
            throttle: 0.0,
        });
        assert!(!result.done);
        assert!(env.native_vehicle.car.f32(0x10c).is_finite());
    }

    #[test]
    fn public_state_edits_reacquire_the_road_height() {
        use crate::{road_mesh::RoadMesh, track::RecoveredStation, Circuit, Track, Vec2};
        let mut mesh = Vec::new();
        mesh.extend(4u32.to_le_bytes());
        mesh.extend(2u32.to_le_bytes());
        for [x, y] in [
            [90.0f32, 190.0],
            [110.0, 190.0],
            [110.0, 210.0],
            [90.0, 210.0],
        ] {
            let height = 0.8 * (x - 90.0);
            for f in [x, height, -y] {
                mesh.extend(f.to_le_bytes());
            }
            mesh.extend([0, 0, 255, 127]);
        }
        for indices in [[0u16, 1, 2], [0, 2, 3]] {
            for i in indices {
                mesh.extend(i.to_le_bytes());
            }
            mesh.extend([7, 2, 0, 0, 0, 0]);
        }
        let stations = [90.0, 100.0, 110.0]
            .into_iter()
            .enumerate()
            .map(|(i, x)| RecoveredStation {
                arc: i as f64 * 10.0 * UNITS_PER_METRE,
                point: Vec2 {
                    x: x * UNITS_PER_METRE,
                    y: 200.0 * UNITS_PER_METRE,
                },
                height: (0.8 * (x - 90.0)) as f32,
                left: 5.0 * UNITS_PER_METRE,
                right: 5.0 * UNITS_PER_METRE,
                edges: None,
            })
            .collect();
        let track = Track::from_recovered(
            Circuit::Monza,
            false,
            stations,
            20.0 * UNITS_PER_METRE,
            RoadMesh::decode(&mesh).unwrap(),
        )
        .unwrap();
        let mut env = Env::new(track, 10);
        assert!(!env.off_track);
        // A replay driver places the car ten metres along an eight-metre rise
        // without reinitializing the environment.
        let arc = 10.0 * UNITS_PER_METRE;
        let point = env.track.arc_point(arc);
        env.x = point.x;
        env.y = point.y;
        env.heading = env.track.arc_tangent(arc).angle();
        env.vx = 20.0 * UNITS_PER_METRE;
        env.vy = 0.0;
        env.yaw_rate = 0.0;
        let result = env.step(crate::Action {
            steer: 0.0,
            throttle: 0.0,
        });
        assert!(!result.done && !env.off_track, "edited car lost the road");
        assert!(
            env.tyre_loads.iter().all(|load| *load > 0.0),
            "{:?}",
            env.tyre_loads
        );
    }

    #[test]
    fn held_keys_and_release_remain_stable_across_road_speeds() {
        for speed in [5.0, 10.0, 20.0, 40.0, 60.0, 80.0] {
            for pedal in [-1.0, 0.0, 1.0] {
                for direction in [-1.0, 1.0] {
                    let mut env = Env::new(crate::Circuit::Monza.build(), 1);
                    env.vx = speed * UNITS_PER_METRE;
                    // Exercise the real integrator on an open skidpad: walls
                    // must not end the maneuver before instability can develop.
                    for tick in 0..300 {
                        env.integrate_vehicle(if tick < 180 { direction } else { 0.0 }, pedal);
                        let rear_vy = env.vy / UNITS_PER_METRE
                            - WHEELBASE / UNITS_PER_METRE * 0.5 * env.yaw_rate;
                        let rear_slip = rear_vy.atan2((env.vx / UNITS_PER_METRE).abs().max(1.0));
                        // The recovered car permits a recoverable slide under full-lock
                        // coasting/braking; launch remains below the original 12-degree gate.
                        let limit = if pedal < 0.0 {
                            25.0
                        } else if pedal == 0.0 {
                            15.0
                        } else {
                            12.0
                        };
                        assert!(rear_slip.abs().to_degrees() < limit,
                            "speed={speed} pedal={pedal} turn={direction} tick={tick}: rear slip={} yaw={} vx={}",
                            rear_slip.to_degrees(), env.yaw_rate, env.vx / UNITS_PER_METRE);
                        assert!(
                            env.tyre_usage.iter().all(|u| u.is_finite() && *u >= 0.0),
                            "finite native tire-demand telemetry"
                        );
                    }
                    assert!(
                        env.yaw_rate.abs() < 0.05,
                        "release must recover: speed={speed} pedal={pedal} yaw={}",
                        env.yaw_rate
                    );
                }
            }
        }
    }

    #[test]
    fn keyboard_corrections_do_not_trigger_a_spin() {
        for speed in [5.0, 10.0, 20.0, 40.0, 60.0, 80.0] {
            let mut env = Env::new(crate::Circuit::Monza.build(), 1);
            env.vx = speed * UNITS_PER_METRE;
            for tick in 0..300 {
                let command = if tick >= 180 {
                    0.0
                } else if (tick / 15) % 2 == 0 {
                    1.0
                } else {
                    -1.0
                };
                env.integrate_vehicle(command, 1.0);
                let rear_vy =
                    env.vy / UNITS_PER_METRE - WHEELBASE / UNITS_PER_METRE * 0.5 * env.yaw_rate;
                let rear_slip = rear_vy.atan2((env.vx / UNITS_PER_METRE).abs().max(1.0));
                assert!(
                    rear_slip.abs().to_degrees() < 12.0,
                    "correction at speed={speed} tick={tick}: rear slip={}",
                    rear_slip.to_degrees()
                );
            }
            assert!(env.yaw_rate.abs() < 0.05);
        }
    }
}
