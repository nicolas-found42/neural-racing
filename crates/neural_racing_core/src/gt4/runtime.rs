//! Adapter of the recovered normal four-wheel model to queried road geometry.
//! Track geometry supplies the plane; car calculations retain native state,
//! axes, parameter values and call order. Distances here are metres.

use super::{
    aids, attitude, automatic, brake_heat,
    conductor::ForceConductor,
    contact_finish,
    eefloat::{add, div, mul},
    kinematics, math,
    native::NativeCar,
    normal::{self, NormalGeometry, NormalParameters, NormalWheel},
    parameters::NativeParameters,
    preparation, slip, steering, steering_frame,
    suspension::*,
};
use std::sync::Arc;

/// External road-query result in physics-native metres and Z-up axes.
#[derive(Clone, Copy, Debug)]
pub(crate) struct RoadContact {
    pub height_metres: f32,
    pub normal: [f32; 3],
    /// Original surface index; consumed by the recovered tire/rolling tables.
    pub material_id: u8,
    pub flags: u8,
    pub attribute14: f32,
    pub attribute15: f32,
    /// Original RNW horizontal X/Z, before the local vehicle-origin transform.
    pub course_position: [f32; 2],
}

/// Original 0x3f4e40: wet flags remap the original surface table index.
pub(crate) fn contact_material(raw: u8, flags: u8) -> u8 {
    if flags & 2 == 0 {
        return raw;
    }
    match raw {
        0 | 7 => 20,
        1 | 10 | 11 | 12 => 21,
        9 => 22,
        _ => raw,
    }
}

/// Original 0x369640, using the EE scalar operation order and SDK trig ports.
pub(crate) fn road_roughness(x: f32, z: f32, frequency: f32, shape: i8) -> f32 {
    let value = add(math::sin(mul(frequency, x)), math::cos(mul(frequency, z)));
    if shape == 1 {
        let squared = mul(value, value);
        if value < 0.0 {
            -squared
        } else {
            squared
        }
    } else {
        value
    }
}

fn flat_road(_: f32, _: f32, _: f32) -> Option<RoadContact> {
    Some(RoadContact {
        height_metres: 0.0,
        normal: [0.0, 0.0, 1.0],
        material_id: 0,
        flags: 0,
        attribute14: 1.0,
        attribute15: 1.0,
        course_position: [0.0, 0.0],
    })
}

#[derive(Clone)]
pub(crate) struct RuntimeVehicle {
    pub(crate) car: NativeCar,
    pub(crate) parameters: Arc<NativeParameters>,
    pub(crate) forces: ForceConductor,
    brake_heat: brake_heat::BrakeHeatParameters,
    geometry: [NormalGeometry; 4],
}

fn geometry(p: &NativeParameters, i: usize) -> NormalGeometry {
    let axle = usize::from(p.model_u8(0x14 + i));
    let shape = usize::from(p.model_u8(0x2c + i));
    let pair = p.model_u8(0x24 + i);
    NormalGeometry {
        axle,
        side: p.model_u8(0x1c + i),
        fore_aft: p.model_i8(0x3c + i),
        paired_wheel: (pair != 0x7f).then_some(usize::from(pair)),
        half_track_at_data_28: p.f32(0x28 + shape * 4),
        axle_distance_at_data_10: p.f32(0x10 + shape * 4),
        radius_at_data_2c4: p.f32(0x2c4 + axle * 4),
        roll_arm_at_data_ffc: p.f32(0xffc + axle * 4),
        base_alignment_at_data_fe4: p.f32(0xfe4 + axle * 4),
        travel_alignment_at_data_fec: p.f32(0xfec + axle * 4),
        ackermann_factors: [p.f32(0x10c), p.f32(0x110)],
    }
}

impl RuntimeVehicle {
    pub(crate) fn new(x: f32, y: f32, heading: f32, speed: f32) -> Self {
        Self::with_parameters(Arc::clone(NativeParameters::m3()), x, y, heading, speed)
    }

    fn with_parameters(p: Arc<NativeParameters>, x: f32, y: f32, heading: f32, speed: f32) -> Self {
        let geometry = std::array::from_fn(|i| geometry(&p, i));
        let mut result = Self {
            car: NativeCar::default(),
            brake_heat: brake_heat::BrakeHeatParameters::from_parameters(&p),
            parameters: p,
            forces: ForceConductor::default(),
            geometry,
        };
        result.reset_flat(x, y, heading, speed);
        result
    }

    pub(crate) fn set_course_condition(&mut self, condition: u8) -> Result<(), &'static str> {
        self.parameters = Arc::clone(NativeParameters::m3_for_condition(condition)?);
        self.geometry = std::array::from_fn(|i| geometry(&self.parameters, i));
        self.brake_heat = brake_heat::BrakeHeatParameters::from_parameters(&self.parameters);
        Ok(())
    }

    /// One original 60 Hz normal-car update. Both human and policy controls use
    /// this same native analog packet and the same automatic gear selector.
    pub(crate) fn step(&mut self, input: steering_frame::AnalogInput) -> Result<(), &'static str> {
        self.step_with_road(input, &mut flat_road)
    }

    pub(crate) fn step_with_road(
        &mut self,
        input: steering_frame::AnalogInput,
        road: &mut impl FnMut(f32, f32, f32) -> Option<RoadContact>,
    ) -> Result<(), &'static str> {
        self.query_road(road)?;
        self.save_history();
        preparation::prepare(&mut self.car, &self.parameters, false, false)?;
        steering_frame::input_analog(&mut self.car, &self.parameters, input)?;
        steering_frame::map_wheels(&mut self.car, &self.parameters)?;
        let slip_parameters = slip::SlipParameters::from_bytes(slip::ORIGINAL_SLIP_BYTES);
        for i in 0..4 {
            let w = self.car.wheel(i);
            let input = add(
                mul(math::cos(w.f32(0)), w.f32(0x28)),
                -mul(math::sin(w.f32(0)), w.f32(0x2c)),
            );
            let mut state = slip::WheelSlipState {
                input_at_20: w.f32(0x20),
                input_at_28: w.f32(0x28),
                ratio_at_38: w.f32(0x38),
                gain_at_3c: w.f32(0x3c),
                mode_at_44: w.i8(0x44),
                direction_at_45: w.i8(0x45),
            };
            let tire_slot = self.parameters.tire_slot(i, usize::from(w.u8(0x89)));
            slip::update_slip(
                input,
                &mut state,
                &slip_parameters,
                self.parameters.u8(0x410 + tire_slot * 0x1ec + 0x1d8) != 0,
                self.parameters.f32(0xfc8),
            );
            let mut w = self.car.wheel_mut(i);
            w.set_f32(0x38, state.ratio_at_38);
            w.set_f32(0x3c, state.gain_at_3c);
            w.set_i8(0x44, state.mode_at_44);
            w.set_i8(0x45, state.direction_at_45);
        }
        aids::update_standard(&mut self.car, &self.parameters)?;
        self.forces.step(
            &mut self.car,
            &self.parameters,
            self.parameters.angle_mode(),
            automatic::desired_gear,
        )?;
        brake_heat::update(&mut self.car, &self.parameters, self.brake_heat);
        if self.car.u8(0x710) != 2 {
            self.car.set_u32(0x70c, 0);
        }
        self.advance_pose_and_suspension_with_road(road)?;
        self.wrap_rotations();
        Ok(())
    }

    /// Original 0x343ed0 fields that feed the next numeric frame.
    fn save_history(&mut self) {
        let c = &mut self.car;
        for (to, from) in [
            (0x8bc, 0x66c),
            (0x8c0, 0x670),
            (0x8c4, 0x674),
            (0x8c8, 0x678),
            (0x8cc, 0x67c),
            (0x994, 0x700),
            (0x998, 0x704),
            (0x99c, 0x5a8),
        ] {
            c.set_u32(to, c.u32(from));
        }
        for i in 0..4 {
            for (to, from) in [
                (0x8d0, 0x94),
                (0x8e0, 0),
                (0x8f0, 4),
                (0x900, 8),
                (0x910, 0x24),
                (0x920, 0xa4),
                (0x980, 0xc0),
            ] {
                c.set_u32(to + i * 4, c.wheel(i).u32(from));
            }
            c.set_u32(0x930 + i * 4, c.u32(0x7cc + i * 0x34));
            c.set_u32(0x940 + i * 4, c.u32(0x7d0 + i * 0x34));
            c.set_u32(0x950 + i * 12, c.u32(0x7b8 + i * 0x34));
            c.set_u32(0x954 + i * 12, c.wheel(i).u32(0x8c));
            c.set_u32(0x958 + i * 12, c.u32(0x7c0 + i * 0x34));
        }
    }

    /// Numeric rotation/history portion of 0x344040.
    fn wrap_rotations(&mut self) {
        let c = &mut self.car;
        let tau = f32::from_bits(0x40c90fda);
        let heading = c.f32(0x674);
        let delta = if heading >= tau {
            -tau
        } else if heading <= -tau {
            tau
        } else {
            0.0
        };
        if delta != 0.0 {
            c.set_f32(0x674, add(heading, delta));
            c.set_f32(0x8c4, add(c.f32(0x8c4), delta));
        }
        for i in 0..4 {
            let axle = self.geometry[i].axle;
            let increment = if c.u8(0x5c0) != 0 {
                0.0
            } else {
                mul(
                    mul(c.wheel(i).f32(0x20), self.parameters.f32(0x2cc + axle * 4)),
                    c.f32(0x54c),
                )
            };
            let mut angle = add(c.wheel(i).f32(0x24), increment);
            let mut history = c.f32(0x910 + i * 4);
            while angle >= tau {
                angle = add(angle, -tau);
                history = add(history, -tau);
            }
            while angle <= -tau {
                angle = add(angle, tau);
                history = add(history, tau);
            }
            c.wheel_mut(i).set_f32(0x24, angle);
            c.set_f32(0x910 + i * 4, history);
        }
    }

    /// Original placement-tail + normal reset fields, with flat asphalt as the
    /// explicit external road input. Positive `speed` uses original rolling-start
    /// equations; zero speed retains the native free-engine starting state.
    pub(crate) fn reset_flat(&mut self, x: f32, y: f32, heading: f32, speed: f32) {
        self.car = NativeCar::default();
        self.forces = ForceConductor::default();
        let p = &self.parameters;
        let c = &mut self.car;
        c.set_u32(0x10, p.base_address());
        c.set_u32(4, 0x01c0_0000); // Explicit world-owner address for oracle export.
        c.set_f32(0x104, x);
        c.set_f32(0x108, y);
        c.set_f32(0x674, heading);
        c.set_f32(
            0x66c,
            math::atan2(add(p.f32(0x40), -p.f32(0x44)), p.f32(0x18)),
        );
        let basis = attitude::body_basis(c.f32(0x66c), 0.0, heading);
        for i in 0..3 {
            c.set_f32(0x110 + i * 4, basis.forward_at_110[i]);
            c.set_f32(0x11c + i * 4, basis.lateral_at_11c[i]);
            c.set_f32(0x128 + i * 4, basis.up_at_128[i]);
        }
        c.set_f32(0x10c, mul(basis.up_at_128[2], p.f32(0x3c)));
        c.set_f32(0x548, 1.0);
        c.set_u8(0x546, 0); // Native normal automatic-gear selector.
        c.set_u8(0x5b5, 1); // Disable original-course centering on custom courses.
        c.set_f32(0x54c, f32::from_bits(0x3c88_8888));
        c.set_f32(0x550, 60.0);
        c.set_f32(0x59c, -p.f32(0x54));
        c.set_f32(0x5a0, p.f32(0x54));
        c.set_u8(0x624, 1);
        c.set_u8(0x62a, 12);
        c.set_u8(0x708, 0x7f);
        c.set_u8(0x709, 0x7f);
        c.set_u8(0x6f0, 1); // Original correction constructor +0x14 bypass flag.
        c.set_f32(0x5d8, p.f32(0x48));
        c.set_f32(0x5dc, p.f32(0x4c));
        for i in 0..2 {
            if matches!(p.u8(0x244 + i), 3 | 4) {
                c.set_u8(0x628 + i, 1);
            }
        }
        for i in 0..4 {
            let g = self.geometry[i];
            let axle = g.axle;
            let load = add(
                div(p.f32(0x1040 + axle * 4), f32::from(p.model_u8(4 + axle))),
                p.f32(0x344 + axle * 0x58),
            );
            let mut wheel = c.wheel_mut(i);
            wheel.set_u8(0xb2, 0xff);
            wheel.set_f32(0xb8, 1.0);
            wheel.set_f32(0x48, load);
            wheel.set_f32(0x4c, load);
            wheel.set_f32(0x94, p.f32(0x318 + axle * 0x58));
            wheel.set_f32(0x3c, 1.0);
            wheel.set_f32(
                0,
                steering::wheel_angle(
                    0.0,
                    g.fore_aft,
                    g.side,
                    g.ackermann_factors,
                    p.u8(0x10d0) == 1,
                    p.f32(0x10cc),
                    p.f32(0x1008 + axle * 4),
                ),
            );
            wheel.set_f32(4, normal::alignment(i, g, 0.0, 0.0, p.f32(0x1004)));
            wheel.set_f32(
                8,
                normal::lateral_position(
                    g.side,
                    g.half_track_at_data_28,
                    g.roll_arm_at_data_ffc,
                    0.0,
                ),
            );
        }
        self.refresh_road_and_contacts();
        if speed != 0.0 {
            self.initialize_rolling_motion(speed);
        }
    }

    /// Normal FR branch of 0x00363868, after source placement/reset.
    fn initialize_rolling_motion(&mut self, speed: f32) {
        let p = &self.parameters;
        let c = &mut self.car;
        let speed = speed.min(p.f32(0xfd8));
        c.set_f32(0x6c8, speed);
        c.set_f32(0x6d0, speed);
        if c.f32(0x6cc) < speed {
            c.set_f32(0x6cc, speed);
        }
        for i in 0..3 {
            c.set_f32(0x648 + i * 4, mul(c.f32(0x110 + i * 4), speed));
        }
        let gear = automatic::gear_for_speed(speed, p);
        c.set_u8(0x624, gear);
        c.set_f32(0x61c, 1.0);
        let mut omega = mul(speed, p.f32(0x29c + usize::from(gear) * 4));
        let idle = mul(f32::from(p.u16(0x202)), f32::from_bits(0x3dd6_774e));
        let maximum = mul(f32::from(p.u16(0x200)), f32::from_bits(0x3dd6_774e));
        if omega < idle {
            omega = idle;
        } else if omega > maximum {
            omega = maximum;
        }
        c.set_f32(0x600, omega);
        c.set_i16(
            0x5fe,
            div(mul(omega, 60.0), f32::from_bits(0x40c9_0fda)) as i32 as i16,
        );
        c.set_u8(0x626, 1);
        c.set_f32(0x618, 1.0);
        c.set_f32(0x640, speed);
        for i in 0..4 {
            let mut w = c.wheel_mut(i);
            w.set_f32(0x20, speed);
            w.set_f32(0x28, speed);
        }
        for i in 0..2 {
            c.set_f32(0x630 + i * 4, mul(p.f32(0x2cc + i * 4), speed));
        }
    }

    /// Initial flat-road placement; frame updates call its stages separately in
    /// native order so suspension targets and contact masks retain their timing.
    pub(crate) fn refresh_road_and_contacts(&mut self) {
        self.refresh_wheel_geometry(false);
        self.refresh_road_basis();
        for i in 0..4 {
            self.car.wheel_mut(i).set_f32(0x8c, 0.0);
            self.car.wheel_mut(i).set_f32(0x90, 0.0);
            self.car.set_f32(0x7d4 + i * 0x34, 0.0);
            self.car.set_f32(0x7d8 + i * 0x34, 1.0);
            self.car.set_f32(0x7dc + i * 0x34, 0.0);
        }
        self.refresh_normal_targets(true);
        self.refresh_contact_velocities();
    }

    pub(crate) fn refresh_queried_contacts(
        &mut self,
        query: &mut impl FnMut(f32, f32, f32) -> Option<RoadContact>,
    ) -> Result<(), &'static str> {
        self.refresh_wheel_geometry(false);
        self.query_road(query)?;
        self.refresh_road_basis();
        self.refresh_normal_targets(true);
        self.refresh_contact_velocities();
        Ok(())
    }

    /// Original 0x3696c8 offset and 0x369760 position, in physics axes.
    fn wheel_query_position(&self, i: usize) -> [f32; 3] {
        let p = &self.parameters;
        let axle = self.geometry[i].axle;
        let base = 0x308 + axle * 0x58;
        let offset = add(
            add(
                -p.f32(base + 0x10),
                -add(p.f32(0x2c4 + axle * 4), -p.f32(0x2dc + axle * 4)),
            ),
            p.f32(base + 0xc),
        );
        std::array::from_fn(|j| {
            add(
                add(
                    self.car.f32(0x104 + j * 4),
                    self.car.wheel(i).f32(0xc + j * 4),
                ),
                mul(self.car.f32(0x128 + j * 4), offset),
            )
        })
    }

    /// Populate the original cached contact records. Missing or unsupported
    /// surfaces are reported rather than silently replaced with asphalt.
    pub(crate) fn query_road(
        &mut self,
        query: &mut impl FnMut(f32, f32, f32) -> Option<RoadContact>,
    ) -> Result<(), &'static str> {
        let positions = std::array::from_fn::<_, 4, _>(|i| self.wheel_query_position(i));
        let mut hits = [None; 4];
        for (i, hit) in hits.iter_mut().enumerate() {
            let [x, y, z] = positions[i];
            *hit = query(x, y, z);
            let Some(hit) = hit else {
                continue;
            };
            if contact_material(hit.material_id, hit.flags) >= 24
                || !hit.height_metres.is_finite()
                || !hit.normal.iter().all(|v| v.is_finite())
                || hit.normal[2] <= 0.0
            {
                return Err("unsupported road contact");
            }
        }
        if hits.iter().all(Option::is_none) {
            return Err("road query has no contact");
        }
        // Original 0x3f4ea0 classifies RAW surfaces; wet-remapped tire IDs
        // index the separate amplitude/frequency/shape tables in 0x369888.
        let classified = |hit: &RoadContact| {
            self.parameters
                .global_f32(0x622430 + u32::from(hit.material_id) * 4)
                .to_bits()
                != 0
        };
        let classifications = hits.map(|hit| hit.is_some_and(|hit| classified(&hit)));
        let all_classified = classifications.iter().all(|v| *v);
        for (i, hit) in hits.iter().enumerate() {
            // 0x3697a8 writes input X/Y/Z even when the subsequent query misses.
            self.car.set_f32(0x7b8 + i * 0x34, positions[i][0]);
            self.car.set_f32(0x7bc + i * 0x34, positions[i][2]);
            self.car.set_f32(0x7c0 + i * 0x34, -positions[i][1]);
            let Some(hit) = hit else {
                // Original 0x395760 only marks the cached miss. Original
                // 0x369888 clears bump+90, retaining height/material/normal.
                self.car.set_u8(0x7e8 + i * 0x34, 1);
                self.car.wheel_mut(i).set_f32(0x90, 0.0);
                if self.car.f32(0x7d8 + i * 0x34) < 0.5 {
                    self.car.set_f32(0x7d4 + i * 0x34, 0.0);
                    self.car.set_f32(0x7d8 + i * 0x34, 1.0);
                    self.car.set_f32(0x7dc + i * 0x34, 0.0);
                }
                continue;
            };
            self.car.set_u8(0x7e8 + i * 0x34, 0);
            let id = u32::from(contact_material(hit.material_id, hit.flags));
            let amplitude = self.parameters.global_f32(0x6209e4 + id * 4);
            let mut bump = 0.0;
            if amplitude != 0.0 {
                bump = road_roughness(
                    hit.course_position[0],
                    hit.course_position[1],
                    self.parameters.global_f32(0x620a44 + id * 4),
                    self.parameters.global_u8(0x620abc + id) as i8,
                );
                if classifications[i] && !all_classified {
                    bump = mul(bump, 0.5);
                }
                if self.parameters.global_u8(0x620aa4 + id) != 0 {
                    bump = mul(bump, self.car.wheel(i).f32(0x40));
                }
                bump = mul(bump, amplitude);
            }
            self.car.wheel_mut(i).set_f32(0x8c, hit.height_metres);
            self.car.wheel_mut(i).set_f32(0x90, bump);
            self.car.set_f32(0x7b8 + i * 0x34, hit.course_position[0]);
            self.car.set_f32(0x7c0 + i * 0x34, hit.course_position[1]);
            self.car
                .wheel_mut(i)
                .set_u8(0x89, contact_material(hit.material_id, hit.flags));
            self.car.set_f32(0x7cc + i * 0x34, hit.attribute14);
            self.car.set_f32(0x7d0 + i * 0x34, hit.attribute15);
            self.car.set_u8(0x7e9 + i * 0x34, hit.material_id);
            self.car.set_u8(0x7eb + i * 0x34, hit.flags);
            self.car.wheel_mut(i).set_u8(0x8b, hit.flags);
            // Original 0x344458 guards steep/degenerate contact normals.
            let normal = if hit.normal[2] < 0.5 {
                [0.0, 0.0, 1.0]
            } else {
                hit.normal
            };
            self.car.set_f32(0x7d4 + i * 0x34, normal[0]);
            self.car.set_f32(0x7d8 + i * 0x34, normal[2]);
            self.car.set_f32(0x7dc + i * 0x34, -normal[1]);
        }
        Ok(())
    }

    /// 0x3457b0 plus (unless z_only) 0x3458d0. The native wheel position
    /// uses fixed axle/track geometry; visual travel/alignment fields are separate.
    fn refresh_wheel_geometry(&mut self, z_only: bool) {
        let c = &mut self.car;
        let p = &self.parameters;
        for i in 0..4 {
            let g = self.geometry[i];
            for j in if z_only { 2..3 } else { 0..3 } {
                let half = mul(c.f32(0x11c + j * 4), g.half_track_at_data_28);
                let mut v = if g.side == 0 {
                    -half
                } else if g.side == 1 {
                    half
                } else {
                    0.0
                };
                let longitudinal = mul(
                    c.f32(0x110 + j * 4),
                    p.f32(0x10 + usize::from(p.model_u8(0x2c + i)) * 4),
                );
                if g.fore_aft > 0 {
                    v = add(v, longitudinal);
                } else if g.fore_aft < 0 {
                    v = add(v, -longitudinal);
                }
                c.wheel_mut(i).set_f32(0xc + j * 4, v);
            }
        }
    }

    /// Original four-wheel road basis at 0x35d350, after body/geometry update.
    fn refresh_road_basis(&mut self) {
        let c = &mut self.car;
        // Original model-zero branch of 0x35d350.
        let h: [f32; 4] = std::array::from_fn(|i| c.wheel(i).f32(0x8c));
        let p = &self.parameters;
        let front_rear = add(add(h[0], h[1]), -add(h[2], h[3]));
        let left = add(h[0], mul(add(h[2], -h[0]), p.f32(0x20)));
        let right = add(h[1], mul(add(h[3], -h[1]), p.f32(0x20)));
        let pitch = math::atan2(
            mul(front_rear, 0.5),
            mul(p.f32(0x18), math::cos(c.f32(0x66c))),
        );
        let roll = math::atan2(add(left, -right), mul(p.f32(0x1c), math::cos(c.f32(0x670))));
        let road = attitude::body_basis(pitch, roll, c.f32(0x674));
        c.set_f32(0x134, c.f32(0x104));
        c.set_f32(0x138, c.f32(0x108));
        c.set_f32(0x13c, mul(add(left, right), 0.5));
        c.set_f32(0x678, pitch);
        c.set_f32(0x67c, roll);
        for i in 0..3 {
            c.set_f32(0x140 + i * 4, road.forward_at_110[i]);
            c.set_f32(0x14c + i * 4, road.lateral_at_11c[i]);
            c.set_f32(0x158 + i * 4, road.up_at_128[i]);
        }
    }

    /// 0x36a490→0x36a080→0x36a1d8. Penetration reuses the earlier normal
    /// projections, as the original 0x36ba88 does, rather than recomputing them.
    fn refresh_normal_targets(&mut self, update_projections: bool) {
        let c = &mut self.car;
        let p = &self.parameters;
        if update_projections {
            let mut dot = 0.0;
            for j in 0..3 {
                dot = add(dot, mul(c.f32(0x128 + j * 4), c.f32(0x158 + j * 4)));
            }
            c.set_f32(0x6b8, dot);
            for i in 0..4 {
                let projection = add(
                    add(
                        mul(c.f32(0x7d4 + i * 0x34), c.f32(0x128)),
                        -mul(c.f32(0x7dc + i * 0x34), c.f32(0x12c)),
                    ),
                    mul(c.f32(0x7d8 + i * 0x34), c.f32(0x130)),
                );
                c.wheel_mut(i).set_f32(0xc8, projection);
            }
        }
        let mut mask = 0u8;
        for i in 0..4 {
            let g = self.geometry[i];
            let axle = g.axle;
            let base = 0x308 + axle * 0x58;
            let offset = add(
                add(
                    -p.f32(base + 0x10),
                    -add(p.f32(0x2c4 + axle * 4), -p.f32(0x2dc + axle * 4)),
                ),
                p.f32(base + 0xc),
            );
            let hub = add(
                add(c.f32(0x10c), c.wheel(i).f32(0x14)),
                mul(c.f32(0x130), offset),
            );
            let target = add(
                add(
                    add(
                        div(
                            mul(
                                add(hub, -add(c.wheel(i).f32(0x8c), c.wheel(i).f32(0x90))),
                                c.f32(0x160),
                            ),
                            c.wheel(i).f32(0xc8),
                        ),
                        -g.radius_at_data_2c4,
                    ),
                    p.f32(base + 0xc),
                ),
                -offset,
            );
            c.set_f32(0x514 + i * 4, target);
            if target < p.f32(base + 4) {
                mask |= 1 << i;
            }
        }
        c.set_u8(0x5b9, mask);
    }

    /// 0x35d920 and the normal four-wheel branch of 0x35d6b0, called at the
    /// end of the frame after the new road basis has been constructed.
    fn refresh_contact_velocities(&mut self) {
        let c = &mut self.car;
        let p = &self.parameters;
        let geometry = std::array::from_fn::<_, 4, _>(|i| {
            let g = self.geometry[i];
            kinematics::ContactGeometry {
                yaw_lateral_lever: -g.axle_distance_at_data_10 * f32::from(g.fore_aft.signum()),
                signed_half_track: g.half_track_at_data_28 * if g.side == 0 { -1.0 } else { 1.0 },
                height: p.f32(0x40 + g.axle * 4),
            }
        });
        let mut wheels = [kinematics::ContactVelocity::default(); 4];
        let (forward, lateral) = kinematics::update_contact_velocities(
            &mut wheels,
            &geometry,
            kinematics::ContactContext {
                world_velocity_at_648: std::array::from_fn(|i| c.f32(0x648 + i * 4)),
                road_forward_at_140: std::array::from_fn(|i| c.f32(0x140 + i * 4)),
                road_lateral_at_14c: std::array::from_fn(|i| c.f32(0x14c + i * 4)),
                yaw_rate_at_680: c.f32(0x680),
                roll_rate_at_688: c.f32(0x688),
                body_up_z_at_130: c.f32(0x130),
                roll_coupling_at_620784: p.global_u8(0x620784) != 0,
            },
        );
        c.set_f32(0x640, forward);
        c.set_f32(0x644, lateral);
        for (i, wheel) in wheels.into_iter().enumerate() {
            let mut w = c.wheel_mut(i);
            w.set_f32(0x28, wheel.forward_at_28);
            w.set_f32(0x2c, wheel.lateral_at_2c);
        }
    }

    /// Original free-pose advance followed by the normal four-wheel suspension
    /// and attitude passes. Cached road contacts are external-world inputs;
    /// no kinematic yaw, velocity damping or grip clamp is added.
    pub(crate) fn advance_pose_and_suspension(&mut self) {
        self.advance_pose_and_suspension_with_road(&mut flat_road)
            .expect("flat road is supported");
    }

    fn advance_pose_and_suspension_with_road(
        &mut self,
        road: &mut impl FnMut(f32, f32, f32) -> Option<RoadContact>,
    ) -> Result<(), &'static str> {
        let c = &mut self.car;
        let dt = c.f32(0x54c);
        for i in 0..3 {
            c.set_f32(
                0x104 + i * 4,
                add(c.f32(0x104 + i * 4), mul(c.f32(0x648 + i * 4), dt)),
            );
        }
        c.set_f32(0x674, add(c.f32(0x674), mul(c.f32(0x680), dt)));
        self.query_road(road)?;
        let c = &mut self.car;
        // 0x36a490 observes the basis and wheel geometry stored by the prior
        // frame; 0x36b0c8 constructs the new basis after normal-force integration.
        let basis = attitude::BodyBasis {
            forward_at_110: std::array::from_fn(|i| c.f32(0x110 + i * 4)),
            lateral_at_11c: std::array::from_fn(|i| c.f32(0x11c + i * 4)),
            up_at_128: std::array::from_fn(|i| c.f32(0x128 + i * 4)),
        };
        self.refresh_normal_targets(true);
        let p = &self.parameters;
        let c = &mut self.car;
        let mut wheels = std::array::from_fn::<_, 4, _>(|i| {
            let w = c.wheel(i);
            NormalWheel {
                suspension: SuspensionState {
                    normal_at_48: w.f32(0x48),
                    travel_at_94: w.f32(0x94),
                    velocity_at_98: w.f32(0x98),
                    spring_at_9c: w.f32(0x9c),
                    damper_at_a0: w.f32(0xa0),
                },
                alignment_at_04: w.f32(4),
                lateral_position_at_08: w.f32(8),
                body_lateral_force_at_d4: w.f32(0xd4),
                normal_velocity_at_d8: w.f32(0xd8),
                contact_transition_at_dc: w.u8(0xdc),
                position_input_at_514: c.f32(0x514 + i * 4),
                body_force_at_524: c.f32(0x524 + i * 4),
                contact_normal: [
                    c.f32(0x7d4 + i * 0x34),
                    -c.f32(0x7dc + i * 0x34),
                    c.f32(0x7d8 + i * 0x34),
                ],
            }
        });
        let springs = std::array::from_fn(|axle| SuspensionParameters {
            values: std::array::from_fn(|i| p.f32(0x308 + axle * 0x58 + i * 4)),
        });
        let damper = std::array::from_fn(|axis| DamperGlobals {
            mode_at_620dd9: p.global_u8(0x620dd9),
            blend_enabled_at_620cb0: p.global_u8(0x620cb0) != 0,
            negative_speed_at_620cb8: p.global_f32(0x620cb8 + axis as u32 * 4),
            negative_blend_at_620cc0: p.global_f32(0x620cc0 + axis as u32 * 4),
            positive_speed_at_620cc8: p.global_f32(0x620cc8 + axis as u32 * 4),
            positive_blend_at_620cd0: p.global_f32(0x620cd0 + axis as u32 * 4),
        });
        let parameters = NormalParameters {
            suspension: springs,
            damper,
            spring_rate: std::array::from_fn(|i| p.f32(0x31c + self.geometry[i].axle * 0x58)),
            context: SuspensionContext {
                dt_at_54c: dt,
                inverse_dt_at_550: c.f32(0x550),
                alternate_limit_at_5be: c.u16(0x5be) != 0,
                external_force_enabled_at_1076: p.u8(0x1076) != 0,
            },
            world_velocity_at_648: std::array::from_fn(|i| c.f32(0x648 + i * 4)),
            up_at_128: basis.up_at_128,
            pitch_rate_at_684: c.f32(0x684),
            roll_rate_at_688: c.f32(0x688),
            steer_at_56c: c.f32(0x56c),
            steer_alignment_at_data_1004: p.f32(0x1004),
            lateral_acceleration_at_658: c.f32(0x658),
            longitudinal_force_at_668: c.f32(0x668),
            lateral_transfer_at_data_34: p.f32(0x34),
            longitudinal_transfer_at_data_38: p.f32(0x38),
            aero_at_6ac: c.f32(0x6ac),
            aero_at_6b0: c.f32(0x6b0),
            front_wheel_count: p.model_u8(4),
            rear_wheel_count: p.model_u8(5),
            local_transfer_at_620e11: p.global_u8(0x620e11) != 0,
            height_at_data_1060: p.f32(0x1060),
        };
        normal::update_normals(&mut wheels, &self.geometry, &parameters, |_, _| {
            unreachable!("selected M3 has no external spring-force extension")
        });
        for (i, state) in wheels.iter().enumerate() {
            let mut w = c.wheel_mut(i);
            w.set_f32(4, state.alignment_at_04);
            w.set_f32(8, state.lateral_position_at_08);
            let s = state.suspension;
            for (at, value) in [
                (0x48, s.normal_at_48),
                (0x94, s.travel_at_94),
                (0x98, s.velocity_at_98),
                (0x9c, s.spring_at_9c),
                (0xa0, s.damper_at_a0),
                (0xd8, state.normal_velocity_at_d8),
            ] {
                w.set_f32(at, value);
            }
            w.set_u8(0xdc, state.contact_transition_at_dc);
            c.set_f32(0x524 + i * 4, state.body_force_at_524);
        }
        let mut body = attitude::AttitudeState {
            world_velocity_at_648: std::array::from_fn(|i| c.f32(0x648 + i * 4)),
            vertical_acceleration_at_65c: c.f32(0x65c),
            pitch_at_66c: c.f32(0x66c),
            roll_at_670: c.f32(0x670),
            heading_at_674: c.f32(0x674),
            pitch_rate_at_684: c.f32(0x684),
            roll_rate_at_688: c.f32(0x688),
            basis,
        };
        let body_wheels = std::array::from_fn::<_, 4, _>(|i| {
            let w = c.wheel(i);
            let g = self.geometry[i];
            attitude::AttitudeWheel {
                axle: g.axle,
                side: g.side,
                fore_aft: g.fore_aft,
                steer_at_00: w.f32(0),
                normal_at_48: w.f32(0x48),
                drive_at_70: w.f32(0x70),
                brake_at_74: w.f32(0x74),
                damper_at_a0: w.f32(0xa0),
                force_at_524: c.f32(0x524 + i * 4),
                radius_at_data_2c4: g.radius_at_data_2c4,
                axle_distance_at_data_10: g.axle_distance_at_data_10,
                reaction_coefficient_at_data_12c: p.f32(0x12c + g.axle * 4),
                static_force_at_data_344: p.f32(0x344 + g.axle * 0x58),
                contact_normal: [
                    c.f32(0x7d4 + i * 0x34),
                    -c.f32(0x7dc + i * 0x34),
                    c.f32(0x7d8 + i * 0x34),
                ],
            }
        });
        let parameters = attitude::AttitudeParameters {
            dt_at_54c: dt,
            vertical_mass_at_5dc: c.f32(0x5dc),
            pitch_coefficients_at_data_11c: [p.f32(0x11c), p.f32(0x120)],
            roll_coefficients_at_data_124: [p.f32(0x124), p.f32(0x128)],
            vertical_mode_at_620dec: p.global_u8(0x620dec),
            road_normal_at_158: std::array::from_fn(|i| c.f32(0x158 + i * 4)),
            aero_force_at_6ac: c.f32(0x6ac),
            aero_force_at_6b0: c.f32(0x6b0),
            special_type_at_data_1077: p.u8(0x1077),
            position_z_at_10c: c.f32(0x10c),
            road_z_at_13c: c.f32(0x13c),
            height_at_data_3c: p.f32(0x3c),
            special_force_at_data_1084: p.f32(0x1084),
        };
        attitude::update_attitude(&mut body, &body_wheels, parameters, false);
        for i in 0..3 {
            c.set_f32(0x648 + i * 4, body.world_velocity_at_648[i]);
            c.set_f32(0x110 + i * 4, body.basis.forward_at_110[i]);
            c.set_f32(0x11c + i * 4, body.basis.lateral_at_11c[i]);
            c.set_f32(0x128 + i * 4, body.basis.up_at_128[i]);
        }
        for (at, value) in [
            (0x65c, body.vertical_acceleration_at_65c),
            (0x66c, body.pitch_at_66c),
            (0x670, body.roll_at_670),
            (0x684, body.pitch_rate_at_684),
            (0x688, body.roll_rate_at_688),
        ] {
            c.set_f32(at, value);
        }
        self.correct_penetration();
        self.refresh_wheel_geometry(false);
        self.refresh_road_basis();
        self.refresh_contact_velocities();
        contact_finish::finish(&mut self.car, &self.parameters)
            .expect("selected normal M3 supports the original contact tail");
        Ok(())
    }

    /// Selected model-zero branch of 0x36ba88 with the cached road basis.
    fn correct_penetration(&mut self) {
        let old = self.car.u8(0x5b9);
        if old == 0 {
            return;
        }
        self.refresh_wheel_geometry(true);
        self.refresh_normal_targets(false);
        let low = self.car.u8(0x5b9);
        self.car.set_u8(0x5b9, low | old.wrapping_shl(4));
        if low & 15 != 0 {
            let p = &self.parameters;
            let penetration = std::array::from_fn::<_, 4, _>(|i| {
                add(
                    p.f32(0x30c + self.geometry[i].axle * 0x58),
                    -self.car.f32(0x514 + i * 4),
                )
                .max(0.0)
            });
            let [a, b, c, d] = penetration;
            let left = add(a, mul(add(c, -a), p.f32(0x20)));
            let right = add(b, mul(add(d, -b), p.f32(0x20)));
            let front = mul(add(a, b), 0.5);
            let rear = mul(add(c, d), 0.5);
            let mut lift = 0.0;
            if left != 0.0 || right != 0.0 {
                if left != right {
                    self.car.set_f32(
                        0x670,
                        add(
                            self.car.f32(0x670),
                            math::atan2(add(left, -right), p.f32(0x1c)),
                        ),
                    );
                }
                lift = mul(left.min(right), 0.5);
            }
            if front != 0.0 || rear != 0.0 {
                if front != rear {
                    self.car.set_f32(
                        0x66c,
                        add(
                            self.car.f32(0x66c),
                            math::atan2(add(front, -rear), p.f32(0x18)),
                        ),
                    );
                }
                lift = add(lift, mul(front.min(rear), 0.5));
            }
            self.car.set_f32(0x10c, add(self.car.f32(0x10c), lift));
        }
        let c = &mut self.car;
        let basis = attitude::body_basis(c.f32(0x66c), c.f32(0x670), c.f32(0x674));
        for i in 0..3 {
            c.set_f32(0x110 + i * 4, basis.forward_at_110[i]);
            c.set_f32(0x11c + i * 4, basis.lateral_at_11c[i]);
            c.set_f32(0x128 + i * 4, basis.up_at_128[i]);
        }
        let mut forward = 0.0;
        let mut lateral = 0.0;
        for i in 0..3 {
            forward = add(forward, mul(c.f32(0x140 + i * 4), c.f32(0x648 + i * 4)));
            lateral = add(lateral, mul(c.f32(0x14c + i * 4), c.f32(0x648 + i * 4)));
        }
        let floor = add(mul(forward, c.f32(0x148)), mul(lateral, c.f32(0x154))).min(0.0);
        if c.f32(0x650) < floor {
            c.set_f32(0x650, floor);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn original_prepared_wheel_query_positions_are_bit_exact() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_query_positions.json"
        ))
        .unwrap();
        let parameters = Arc::new(NativeParameters::fixture_parameters());
        for c in fixture["cases"].as_array().unwrap() {
            let mut v =
                RuntimeVehicle::with_parameters(Arc::clone(&parameters), 0.0, 0.0, 0.0, 0.0);
            let hex = c["input_car_hex"].as_str().unwrap();
            v.car = NativeCar::from_bytes(std::array::from_fn(|i| {
                u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap()
            }));
            let mut captured = Vec::new();
            v.query_road(&mut |x, y, z| {
                captured.extend([x.to_bits(), z.to_bits(), (-y).to_bits()]);
                flat_road(x, y, z)
            })
            .unwrap();
            let expected: Vec<u32> = c["expected_bits"]
                .as_array()
                .unwrap()
                .iter()
                .map(|b| u32::from_str_radix(b.as_str().unwrap(), 16).unwrap())
                .collect();
            assert_eq!(captured, expected, "query input {}", c["label"]);
        }
    }

    #[test]
    fn partial_contact_misses_match_original_cached_state_prefix() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_partial_contact.json"
        ))
        .unwrap();
        let parameters = Arc::new(NativeParameters::fixture_parameters());
        for case in fixture["cases"].as_array().unwrap() {
            let mut v =
                RuntimeVehicle::with_parameters(Arc::clone(&parameters), 0.0, 0.0, 0.0, 0.0);
            let hex = case["input_car_hex"].as_str().unwrap();
            v.car = NativeCar::from_bytes(std::array::from_fn(|i| {
                u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap()
            }));
            let mut i = 0;
            v.query_road(&mut |_, _, _| {
                let h = &case["hits"][i];
                i += 1;
                if h.is_null() {
                    return None;
                }
                Some(RoadContact {
                    height_metres: h["height_metres"].as_f64().unwrap() as f32,
                    normal: [0.0, 0.0, 1.0],
                    material_id: h["material_id"].as_u64().unwrap() as u8,
                    flags: h["flags"].as_u64().unwrap() as u8,
                    attribute14: 1.0,
                    attribute15: 1.0,
                    course_position: std::array::from_fn(|j| {
                        h["course_position"][j].as_f64().unwrap() as f32
                    }),
                })
            })
            .unwrap();
            for (off, bits) in fixture["layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(case["expected_bits"].as_array().unwrap())
            {
                assert_eq!(
                    v.car.u32(off.as_u64().unwrap() as usize),
                    u32::from_str_radix(bits.as_str().unwrap(), 16).unwrap(),
                    "missing mask {} offset {}",
                    case["variant"],
                    off
                );
            }
        }
    }

    #[test]
    fn missing_or_unknown_road_does_not_become_asphalt() {
        let mut v = RuntimeVehicle::new(0.0, 0.0, 0.0, 0.0);
        let before = v.car.bytes().to_vec();
        assert_eq!(
            v.query_road(&mut |_, _, _| None),
            Err("road query has no contact")
        );
        assert_eq!(v.car.bytes(), before.as_slice());
        assert_eq!(
            v.query_road(&mut |_, _, _| {
                let mut hit = flat_road(0.0, 0.0, 0.0).unwrap();
                hit.material_id = 24;
                Some(hit)
            }),
            Err("unsupported road contact")
        );
        assert_eq!(v.car.bytes(), before.as_slice());
    }

    #[test]
    fn surface_roughness_matches_original_contact_prefix() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_road_roughness.json"
        ))
        .unwrap();
        let hex = fixture["base_input_car_hex"].as_str().unwrap();
        let bytes =
            std::array::from_fn(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap());
        let p = Arc::new(NativeParameters::fixture_parameters());
        let ordered = |x: u32| {
            if x & 0x80000000 != 0 {
                0x80000000i64 - i64::from(x & 0x7fffffff)
            } else {
                0x80000000i64 + i64::from(x)
            }
        };
        for case in fixture["cases"].as_array().unwrap() {
            let mut v = RuntimeVehicle::with_parameters(Arc::clone(&p), 0.0, 0.0, 0.0, 0.0);
            v.car = NativeCar::from_bytes(bytes);
            let mut i = 0;
            v.query_road(&mut |_, _, _| {
                let h = &case["hits"][i];
                i += 1;
                Some(RoadContact {
                    height_metres: h["height_metres"].as_f64().unwrap() as f32,
                    normal: [0.0, 0.0, 1.0],
                    material_id: h["material_id"].as_u64().unwrap() as u8,
                    flags: h["flags"].as_u64().unwrap() as u8,
                    attribute14: 1.0,
                    attribute15: 1.0,
                    course_position: std::array::from_fn(|j| {
                        h["course_position"][j].as_f64().unwrap() as f32
                    }),
                })
            })
            .unwrap();
            for (j, (o, b)) in fixture["layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(case["expected_bits"].as_array().unwrap())
                .enumerate()
            {
                let actual = v.car.u32(o.as_u64().unwrap() as usize);
                let expected = u32::from_str_radix(b.as_str().unwrap(), 16).unwrap();
                if j % 3 == 2 {
                    assert_eq!(actual, expected);
                } else {
                    assert!(
                        (ordered(actual) - ordered(expected)).abs() <= 12,
                        "surface {} variant {} offset {}",
                        case["material"],
                        case["variant"],
                        o
                    );
                }
            }
        }
    }

    #[test]
    fn wet_surface_mapping_matches_original_instructions() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_wet_surface.json"
        ))
        .unwrap();
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 128);
        for case in fixture["cases"].as_array().unwrap() {
            assert_eq!(
                contact_material(
                    case["raw_material"].as_u64().unwrap() as u8,
                    case["flags"].as_u64().unwrap() as u8
                ),
                case["expected_material"].as_u64().unwrap() as u8
            );
        }
    }

    #[test]
    fn queried_road_basis_matches_original_nonflat_instructions() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_road_basis.json"
        ))
        .unwrap();
        let post: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_post_pipeline.json"
        ))
        .unwrap();
        assert_eq!(fixture["target_sha256"], post["target_sha256"]);
        assert_eq!(
            fixture["base_input_car_sha256"],
            post["cases"][0]["input_car_sha256"]
        );
        assert_eq!(fixture["parameter_inputs"], post["parameters"]);
        let hex = post["cases"][0]["input_car_hex"].as_str().unwrap();
        let bytes =
            std::array::from_fn(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap());
        let p = Arc::new(NativeParameters::fixture_parameters());
        for case in fixture["cases"].as_array().unwrap() {
            let mut vehicle = RuntimeVehicle::with_parameters(Arc::clone(&p), 0.0, 0.0, 0.0, 0.0);
            vehicle.car = NativeCar::from_bytes(bytes);
            vehicle
                .car
                .set_f32(0x66c, case["body_pitch"].as_f64().unwrap() as f32);
            vehicle
                .car
                .set_f32(0x670, case["body_roll"].as_f64().unwrap() as f32);
            let mut i = 0;
            vehicle
                .refresh_queried_contacts(&mut |_, _, _| {
                    let height_metres = case["heights"][i].as_f64().unwrap() as f32;
                    i += 1;
                    Some(RoadContact {
                        height_metres,
                        normal: [0.0, 0.0, 1.0],
                        material_id: 0,
                        flags: 0,
                        attribute14: 1.0,
                        attribute15: 1.0,
                        course_position: [0.0, 0.0],
                    })
                })
                .unwrap();
            for (offset, expected) in fixture["layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(case["expected_bits"].as_array().unwrap())
            {
                let offset =
                    usize::from_str_radix(offset.as_str().unwrap().trim_start_matches("0x"), 16)
                        .unwrap();
                assert_eq!(
                    vehicle.car.u32(offset),
                    u32::from_str_radix(expected.as_str().unwrap(), 16).unwrap(),
                    "road field {offset:x}"
                );
            }
        }
    }

    #[test]
    fn complete_free_motion_and_contact_pipeline_matches_original_instructions() {
        let fixture: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_post_pipeline.json"
        ))
        .unwrap();
        let inputs: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_native_parameter_inputs.json"
        ))
        .unwrap();
        assert_eq!(fixture["target_sha256"], inputs["original_target_sha256"]);
        for (name, identity) in fixture["parameters"].as_object().unwrap() {
            assert_eq!(identity["sha256"], inputs[name]["sha256"]);
        }
        let parameters = Arc::new(NativeParameters::fixture_parameters());
        let ordered = |bits: u32| {
            let signed = bits as i32;
            if signed < 0 {
                i64::from(i32::MIN) - i64::from(signed)
            } else {
                i64::from(signed)
            }
        };
        let cases = fixture["cases"].as_array().unwrap();
        assert_eq!(cases.len(), 21);
        assert_eq!(fixture["output_layout"].as_array().unwrap().len(), 232);
        for case in cases {
            let mut vehicle =
                RuntimeVehicle::with_parameters(Arc::clone(&parameters), 0.0, 0.0, 0.0, 0.0);
            let hex = case["input_car_hex"].as_str().unwrap();
            let bytes =
                std::array::from_fn(|i| u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap());
            vehicle.car = NativeCar::from_bytes(bytes);
            vehicle.advance_pose_and_suspension();
            vehicle.wrap_rotations();
            for (field, expected) in fixture["output_layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(case["expected_bits"].as_array().unwrap())
            {
                let offset = field["offset"].as_u64().unwrap() as usize;
                let expected = u32::from_str_radix(expected.as_str().unwrap(), 16).unwrap();
                let actual = if field["kind"] == "u8" {
                    u32::from(vehicle.car.u8(offset))
                } else {
                    vehicle.car.u32(offset)
                };
                if field["kind"] == "f32" {
                    assert!(f32::from_bits(actual).is_finite());
                    let ulp = (ordered(actual) - ordered(expected)).abs();
                    assert!(
                        ulp <= 12,
                        "frame{} {}: {ulp}ULP ({actual:08x}/{expected:08x})",
                        case["frame"],
                        field["label"]
                    );
                } else {
                    assert_eq!(
                        actual, expected,
                        "frame{} {}",
                        case["frame"], field["label"]
                    );
                }
            }
        }
    }

    #[test]
    fn flat_placement_matches_original_numeric_tail() {
        let evidence: serde_json::Value = serde_json::from_str(include_str!(
            "../../../../docs/reference/gt4/m3-placement-tail.json"
        ))
        .unwrap();
        let vehicle = RuntimeVehicle::new(0.0, 0.0, 0.0, 0.0);
        for (i, expected) in evidence["wheels"].as_array().unwrap().iter().enumerate() {
            for offset in [0, 4, 8, 0x48, 0x4c, 0x94] {
                let key = format!("0x{offset:x}");
                let bits =
                    u32::from_str_radix(expected[&key]["bits"].as_str().unwrap(), 16).unwrap();
                assert_eq!(
                    vehicle.car.wheel(i).u32(offset),
                    bits,
                    "wheel {i} offset {offset:x}"
                );
            }
        }
        assert_eq!(vehicle.car.f32(0x548), 1.0);
        assert_eq!(vehicle.car.u8(0x6f0), 1);
        assert_eq!(vehicle.car.u8(0x626), 0);
    }

    #[test]
    fn stationary_car_retains_stable_suspension_support() {
        let mut vehicle = RuntimeVehicle::new(0.0, 0.0, 0.0, 0.0);
        for frame in 0..600 {
            super::super::preparation::prepare(&mut vehicle.car, &vehicle.parameters, false, false)
                .unwrap();
            vehicle.advance_pose_and_suspension();
            let height = vehicle.car.f32(0x10c);
            assert!(
                (0.03..0.1).contains(&height),
                "frame {frame}: height {height}"
            );
            assert!(
                vehicle.car.f32(0x650).abs() < 0.1,
                "frame {frame}: vertical velocity {}",
                vehicle.car.f32(0x650)
            );
            for i in 0..4 {
                assert!((0.0..10000.0).contains(&vehicle.car.wheel(i).f32(0x48)));
            }
        }
    }
}
