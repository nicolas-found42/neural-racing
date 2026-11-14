//! Selected M3 standard FR force conductor, in the order of 0x3535b0.
//! Contact, suspension, curve-torque preparation and controller inputs must
//! already be present in the native car buffer. No per-step heap allocation.
use super::{
    chassis, control, drivetrain as drive, engine, native::NativeCar, parameters::NativeParameters,
    slip, tire,
};

#[derive(Clone)]
pub(crate) struct ForceConductor {
    control: control::StandardControlState,
    /// Runtime 0x623e52 optional statistic switch; integration defaults off.
    pub(crate) statistics_enabled: bool,
    /// Runtime 0x6244e8 unit mode; metric zero is an explicit integration input.
    pub(crate) economy_unit_mode: i32,
}
impl Default for ForceConductor {
    fn default() -> Self {
        let mut control = control::StandardControlState::default();
        control.engine.wheel_flags_at_46 = vec![0; 4];
        Self {
            control,
            statistics_enabled: false,
            economy_unit_mode: 0,
        }
    }
}
macro_rules! load_fields {($target:expr,$car:expr,$($field:ident:$offset:expr),* $(,)?)=>{$($target.$field=$car.f32($offset);)*};}
macro_rules! store_fields {($car:expr,$target:expr,$($field:ident:$offset:expr),* $(,)?)=>{$($car.set_f32($offset,$target.$field);)*};}
impl ForceConductor {
    fn load_control(&mut self, car: &NativeCar) {
        let c = &mut self.control;
        let s = &mut c.engine;
        load_fields!(s,car,dt_at_54c:0x54c,throttle_at_598:0x598,curve_torque_at_5ec:0x5ec,drag_at_5f0:0x5f0,throttle_at_5f4:0x5f4,angular_speed_at_600:0x600,throttle_at_604:0x604,coupling_input_at_610:0x610,coupling_input_at_614:0x614,clutch_ratio_at_61c:0x61c,torque_magnitude_at_620:0x620);
        s.rpm_at_5fe = car.i16(0x5fe);
        s.gear_at_624 = car.u8(0x624);
        s.limiter_at_625 = car.u8(0x625);
        s.clutch_at_626 = car.u8(0x626);
        s.flag_at_70a = car.u8(0x70a);
        for i in 0..2 {
            s.carrier_speed_at_630[i] = car.f32(0x630 + i * 4);
            s.drive_torque_at_638[i] = car.f32(0x638 + i * 4);
        }
        for i in 0..4 {
            s.wheel_flags_at_46[i] = car.wheel(i).u8(0x46);
        }
        load_fields!(c,car,brake_at_590:0x590,handbrake_at_594:0x594,clutch_input_at_618:0x618,forward_at_640:0x640,rpm_display_at_738:0x738,rpm_display_rate_at_73c:0x73c);
        c.input_kind_at_546 = car.u8(0x546);
        c.mode_at_56a = car.u8(0x56a);
        c.mode_at_56b = car.u8(0x56b);
        c.latch_at_62a = car.u8(0x62a);
        c.shift_request_at_774 = car.i8(0x774);
    }
    fn store_control(&self, car: &mut NativeCar) {
        let c = &self.control;
        let s = &c.engine;
        store_fields!(car,s,throttle_at_598:0x598,throttle_at_5f4:0x5f4,angular_speed_at_600:0x600,throttle_at_604:0x604,coupling_input_at_610:0x610,coupling_input_at_614:0x614,clutch_ratio_at_61c:0x61c,torque_magnitude_at_620:0x620);
        car.set_i16(0x5fe, s.rpm_at_5fe);
        car.set_u8(0x624, s.gear_at_624);
        car.set_u8(0x625, s.limiter_at_625);
        car.set_u8(0x626, s.clutch_at_626);
        car.set_u8(0x70a, s.flag_at_70a);
        for i in 0..2 {
            car.set_f32(0x630 + i * 4, s.carrier_speed_at_630[i]);
            car.set_f32(0x638 + i * 4, s.drive_torque_at_638[i]);
        }
        for i in 0..4 {
            car.wheel_mut(i).set_u8(0x46, s.wheel_flags_at_46[i]);
        }
        store_fields!(car,c,clutch_input_at_618:0x618,rpm_display_at_738:0x738,rpm_display_rate_at_73c:0x73c);
        car.set_u8(0x62a, c.latch_at_62a);
        car.set_i8(0x774, c.shift_request_at_774);
    }

    /// Executes the ordinary human-control mode0 path. The desired-gear query
    /// is called at the native controller boundary, after 0x3519b8, so it sees
    /// the current RPM and clutch stores. Its implementation must be the
    /// recovered controller, not a caller-invented shift heuristic.
    pub(crate) fn step<D: FnOnce(&mut NativeCar, &NativeParameters) -> i32>(
        &mut self,
        car: &mut NativeCar,
        p: &NativeParameters,
        angle_mode: i32,
        desired_gear: D,
    ) -> Result<(), &'static str> {
        if car.u8(0x56a) != 0 || car.u8(0x56b) != 0 {
            return Err("force conductor supports native human mode zero only");
        }
        self.load_control(car);
        let config = p.driveline_config();
        control::prepare_standard_control(&mut self.control, config, p.u16(0x260))?;
        let mut brakes = std::array::from_fn::<_, 4, _>(|i| {
            let w = car.wheel(i);
            control::BrakePreparationWheel {
                request_at_64: w.f32(0x64),
                normal_at_48: w.f32(0x48),
                multiplier_at_1010: p.f32(0x1010 + p.wheel_groups()[i] * 4),
                drive_sign_at_3c: config.wheel_drive_signs_at_3c[i],
                brake_at_78: w.f32(0x78),
                flags_at_46: w.u8(0x46),
            }
        });
        control::prepare_brakes(&mut brakes, car.f32(0x594), p.f32(0x1018));
        for i in 0..4 {
            car.wheel_mut(i).set_f32(0x78, brakes[i].brake_at_78);
            car.wheel_mut(i).set_u8(0x46, brakes[i].flags_at_46);
            self.control.engine.wheel_flags_at_46[i] = brakes[i].flags_at_46;
        }
        let mut longitudinal = drive::LongitudinalState::default();
        load_fields!(longitudinal,car,dt_at_54c:0x54c,inverse_dt_at_550:0x550,mass_at_5d8:0x5d8,velocity_at_640:0x640,increment_at_654:0x654,force_at_668:0x668,extra_force_at_6a8:0x6a8,extra_force_at_6b4:0x6b4);
        let rolling = std::array::from_fn::<_, 4, _>(|i| {
            let w = car.wheel(i);
            drive::RollingLoad {
                normal_at_48: w.f32(0x48),
                surface_coefficient: p.global_f32(0x620984 + u32::from(w.u8(0x89)) * 4),
            }
        });
        drive::prepare_longitudinal(&mut longitudinal, &rolling, p.rolling_curve());
        for i in 0..4 {
            let w = car.wheel(i);
            let tables = p.tire_for_wheel(i, usize::from(w.u8(0x89)));
            let (a, b) = tables.slip_pair().sample_ee(w.f32(0x38));
            let mut w = car.wheel_mut(i);
            w.set_f32(0x54, a);
            w.set_f32(0x58, b);
        }
        for i in 0..4 {
            let w = car.wheel(i);
            let slip = slip::WheelSlipState {
                ratio_at_38: w.f32(0x38),
                gain_at_3c: w.f32(0x3c),
                mode_at_44: w.i8(0x44),
                direction_at_45: w.i8(0x45),
                ..Default::default()
            };
            let split = tire::split_longitudinal_force(
                &slip,
                w.f32(0x54),
                w.f32(0x4c),
                p.global_u8(0x620d8e),
                p.global_u8(0x620d8f),
            );
            let mut w = car.wheel_mut(i);
            w.set_f32(0x70, split.at_70);
            w.set_f32(0x74, split.at_74);
            w.set_f32(0x7c, split.at_7c);
            w.set_f32(0x80, split.at_80);
        }
        let mut magnitude_sum = 0.0;
        let mut magnitude_count = 0;
        for i in 0..4 {
            let w = car.wheel(i);
            let slot = p.tire_slot(i, usize::from(w.u8(0x89)));
            let tables = p.tire(slot);
            let mut force = tire::WheelForceState {
                angle_at_00: w.f32(0),
                offset_at_04: w.f32(4),
                correction_at_1c: w.f32(0x1c),
                velocity_at_20: w.f32(0x20),
                velocity_at_28: w.f32(0x28),
                velocity_at_2c: w.f32(0x2c),
                heading_at_30: w.f32(0x30),
                slip_at_34: w.f32(0x34),
                scale_at_4c: w.f32(0x4c),
                limit_at_58: w.f32(0x58),
                curve_at_5c: w.f32(0x5c),
                fade_at_60: w.f32(0x60),
                force_at_68: w.f32(0x68),
                force_at_6c: w.f32(0x6c),
                car_force_slot: car.f32(0x534 + i * 4),
            };
            let context = tire::TireForceContext {
                angle_mode,
                mode_at_1d9: p.u8(0x410 + slot * 0x1ec + 0x1d9),
                mode_at_1da: p.u8(0x410 + slot * 0x1ec + 0x1da),
                velocity_gain_at_fcc: p.f32(0xfcc),
                wheel_kind: p.model_u8(0x44 + i),
                side_mode: p.model_u8(0x1c + i),
                car_angle_at_670: car.f32(0x670),
                car_angle_at_67c: car.f32(0x67c),
                global_at_620764: p.global_f32(0x620764),
                global_at_620768: p.global_f32(0x620768),
            };
            if let Some(m) = tire::update_contact_force(&mut force, tables, &context) {
                if p.model_i8(0x3c + i) > 0 {
                    magnitude_sum += m;
                    magnitude_count += 1;
                }
            }
            let mut w = car.wheel_mut(i);
            w.set_f32(0, force.angle_at_00);
            w.set_f32(0x34, force.slip_at_34);
            w.set_f32(0x5c, force.curve_at_5c);
            w.set_f32(0x68, force.force_at_68);
            w.set_f32(0x6c, force.force_at_6c);
            car.set_f32(0x534 + i * 4, force.car_force_slot);
        }
        if self.statistics_enabled {
            car.set_f32(
                0x570,
                if magnitude_count < 2 {
                    magnitude_sum
                } else {
                    magnitude_sum / magnitude_count as f32
                },
            );
        }
        let long_wheels = std::array::from_fn::<_, 4, _>(|i| {
            let w = car.wheel(i);
            drive::LongitudinalWheel {
                angle_at_00: w.f32(0),
                heading_at_30: w.f32(0x30),
                flag_at_46: w.u8(0x46),
                force_at_68: w.f32(0x68),
                force_at_6c: w.f32(0x6c),
                force_at_70: w.f32(0x70),
                force_at_74: w.f32(0x74),
                flag_at_dd: w.u8(0xdd),
                extra_mass_at_300: p.f32(0x300 + p.wheel_groups()[i] * 4),
            }
        });
        drive::update_longitudinal(&mut longitudinal, &long_wheels);
        store_fields!(car,longitudinal,velocity_at_640:0x640,increment_at_654:0x654,force_at_668:0x668,extra_force_at_6b4:0x6b4);
        let mut wheels = std::array::from_fn::<_, 4, _>(|i| {
            let w = car.wheel(i);
            drive::WheelTorqueState {
                speed_at_20: w.f32(0x20),
                force_at_7c: w.f32(0x7c),
                force_at_80: w.f32(0x80),
                brake_at_78: w.f32(0x78),
                torque_at_84: w.f32(0x84),
            }
        });
        for i in 0..4 {
            drive::update_reaction_torque(&mut wheels[i], p.f32(0x2c4 + p.wheel_groups()[i] * 4));
        }
        let steering_groups = std::array::from_fn::<_, 2, _>(|i| {
            let g = &p.carriers()[i];
            drive::SteeringTorqueGroup {
                first_wheel: g.first_wheel,
                paired_wheel: g.paired_wheel,
                kind_at_244: config.group_kinds_at_244[i],
                preload_at_248: p.f32(0x248 + i * 4),
            }
        });
        drive::adjust_steering_torque(
            car.f32(0x640),
            car.f32(0x56c),
            &mut wheels,
            &steering_groups,
        );
        let limit = engine::OutputLimit {
            enabled_at_db: p.u8(0x20f) != 0,
            power_at_dc: p.f32(0x210),
            softness_at_e0: p.f32(0x214),
        };
        drive::update_standard_rear_driveline(
            &mut self.control.engine,
            config,
            &wheels,
            p.wheel_groups(),
            p.carriers(),
            |s| native_na_torque(car, p, s, limit),
        )?;
        update_economy_sample(car, self.economy_unit_mode);
        let mut axles = std::array::from_fn::<_, 2, _>(|i| drive::AxleRotationState {
            carrier_speed_at_630: self.control.engine.carrier_speed_at_630[i],
            drive_torque_at_638: self.control.engine.drive_torque_at_638[i],
            lock_at_628: car.u8(0x628 + i),
        });
        drive::update_wheel_rotation(
            &mut wheels,
            &mut axles,
            p.rotation_configs(),
            car.f32(0x54c),
        );
        for i in 0..2 {
            self.control.engine.carrier_speed_at_630[i] = axles[i].carrier_speed_at_630;
            car.set_u8(0x628 + i, axles[i].lock_at_628);
        }
        self.control.forward_at_640 = car.f32(0x640);
        control::finish_standard_control(
            &mut self.control,
            config,
            p.u16(0x202),
            p.u16(0x200),
            &mut wheels,
            p.carriers(),
            |s| native_na_torque(car, p, s, limit),
        )?;
        self.store_control(car);
        for i in 0..4 {
            let mut w = car.wheel_mut(i);
            w.set_f32(0x20, wheels[i].speed_at_20);
            w.set_f32(0x84, wheels[i].torque_at_84);
        }
        let desired = desired_gear(car, p);
        self.control.latch_at_62a = car.u8(0x62a);
        control::apply_desired_gear(&mut self.control, desired, p.u8(0x32));
        control::update_rpm_display(&mut self.control);
        self.store_control(car);
        self.chassis(car, p, angle_mode);
        Ok(())
    }
    fn chassis(&self, car: &mut NativeCar, p: &NativeParameters, angle_mode: i32) {
        let mut normal = 0.0;
        for i in 0..4 {
            normal += car.wheel(i).f32(0x48);
        }
        let suspended = normal == 0.0;
        let mut wheels = std::array::from_fn::<_, 4, _>(|i| {
            let w = car.wheel(i);
            chassis::WheelChassisState {
                steer_at_00: w.f32(0),
                secondary_angle_at_1c: w.f32(0x1c),
                rim_speed_at_20: w.f32(0x20),
                forward_velocity_at_28: w.f32(0x28),
                lateral_velocity_at_2c: w.f32(0x2c),
                velocity_heading_at_30: w.f32(0x30),
                grip_at_4c: w.f32(0x4c),
                lateral_force_at_68: w.f32(0x68),
                corner_drag_at_6c: w.f32(0x6c),
                drive_force_at_70: w.f32(0x70),
                brake_force_at_74: w.f32(0x74),
                body_lateral_force_at_d4: w.f32(0xd4),
                force_direction_at_dd: w.u8(0xdd),
            }
        });
        let geometry = std::array::from_fn::<_, 4, _>(|i| {
            let side = p.model_u8(0x1c + i);
            let group = usize::from(p.model_u8(0x2c + i));
            let mut half_track = p.f32(0x28 + group * 4);
            if side == 0 {
                half_track = -half_track;
            }
            let mut axle = p.f32(0x10 + group * 4);
            if p.model_i8(0x3c + i) < 0 {
                axle = -axle;
            }
            let w = car.wheel(i);
            let tables = p.tire_for_wheel(i, usize::from(w.u8(0x89)));
            chassis::WheelGeometry {
                signed_half_track: half_track,
                signed_axle_distance: axle,
                additional_lateral_force: car.f32(0x534 + i * 4),
                normalization_grip: (tables.terrain_at_08[usize::from(w.u8(0x89))] * 1.0)
                    * tables.at_00,
            }
        });
        let mut state = chassis::ChassisState::default();
        load_fields!(state,car,lateral_acceleration_at_658:0x658,yaw_rate_at_680:0x680,yaw_acceleration_at_68c:0x68c,normalized_lateral_at_690:0x690,yaw_filter_at_740:0x740,yaw_filter_rate_at_744:0x744);
        chassis::aggregate_forces(
            &mut state,
            &mut wheels,
            &geometry,
            chassis::ChassisParameters {
                mass_at_5d8: car.f32(0x5d8),
                inverse_yaw_inertia_at_data_118: p.f32(0x118),
                dt_at_54c: car.f32(0x54c),
                angle_mode,
            },
            suspended,
        );
        store_fields!(car,state,lateral_acceleration_at_658:0x658,yaw_rate_at_680:0x680,yaw_acceleration_at_68c:0x68c,normalized_lateral_at_690:0x690,yaw_filter_at_740:0x740,yaw_filter_rate_at_744:0x744);
        for i in 0..4 {
            car.wheel_mut(i)
                .set_f32(0xd4, wheels[i].body_lateral_force_at_d4);
        }
        let mut velocity = chassis::VelocityState::default();
        load_fields!(velocity,car,forward_at_640:0x640,lateral_at_644:0x644,longitudinal_acceleration_at_654:0x654,lateral_acceleration_at_658:0x658,vertical_rate_at_668:0x668,yaw_rate_at_680:0x680,normalized_lateral_at_690:0x690);
        velocity.world_xy_at_648 = [car.f32(0x648), car.f32(0x64c)];
        let context = chassis::VelocityContext {
            stop_at_5be: car.i16(0x5be),
            held_at_5c0: car.u8(0x5c0) != 0,
            mode_at_56b: car.u8(0x56b),
            mode_at_56a: car.u8(0x56a),
            brake_at_590: car.f32(0x590),
            brake_at_594: car.f32(0x594),
            dt_at_54c: car.f32(0x54c),
            forward_basis_xy_at_110: [car.f32(0x110), car.f32(0x114)],
            lateral_basis_xy_at_11c: [car.f32(0x11c), car.f32(0x120)],
        };
        let fades = std::array::from_fn::<_, 4, _>(|i| car.wheel(i).f32(0x60));
        chassis::update_world_velocity(&mut velocity, context, &fades, suspended);
        store_fields!(car,velocity,forward_at_640:0x640,lateral_at_644:0x644,longitudinal_acceleration_at_654:0x654,lateral_acceleration_at_658:0x658,vertical_rate_at_668:0x668,yaw_rate_at_680:0x680,normalized_lateral_at_690:0x690);
        car.set_f32(0x648, velocity.world_xy_at_648[0]);
        car.set_f32(0x64c, velocity.world_xy_at_648[1]);
    }
}

/// NA branch 3554a0 followed by its actual 354b78 fuel stores. The admitted
/// standard mode0 makes 3683a8's special-gearbox predicate false.
fn native_na_torque(
    car: &mut NativeCar,
    p: &NativeParameters,
    state: &mut drive::RearDrivelineState,
    limit: engine::OutputLimit,
) -> f32 {
    use super::eefloat::{add, div, mul};
    let torque = engine::naturally_aspirated_torque(state, p.u16(0x200), limit);
    let counter = car.u8(0x5e8);
    car.set_f32(0x5e0, 0.0);
    if counter != 0 {
        car.set_u8(0x5e8, counter.wrapping_sub(1));
    }
    if torque > 0.0 {
        let rate = div(
            mul(
                torque,
                div(
                    mul(state.angular_speed_at_600, 60.0),
                    f32::from_bits(0x40c90fda),
                ),
            ),
            f32::from_bits(0x45db5612),
        );
        if counter != 0 {
            let assist = div(mul(rate, p.f32(0x10d4)), add(p.f32(0x10d4), 1.0));
            car.set_f32(0x5e0, assist);
            let fuel = add(car.f32(0x5e4), -mul(assist, state.dt_at_54c));
            car.set_f32(0x5e4, fuel);
            if fuel <= 0.0 {
                car.set_f32(0x5e4, 0.0);
                car.set_u8(0x5e8, 0);
            }
        }
        let consumed = mul(
            mul(div(p.f32(0x104c), f32::from_bits(0x4a229900)), rate),
            state.dt_at_54c,
        );
        car.set_f32(0x5c4, add(car.f32(0x5c4), consumed));
        if car.f32(0x5d0) > 0.0 && car.i16(0x5be) == 0 {
            let fuel = add(car.f32(0x5d4), -consumed);
            car.set_f32(0x5d4, if fuel < 0.0 { 0.0 } else { fuel });
        }
    }
    torque
}

/// 354ce8 with 472ae0/472a78. Unit mode is explicit runtime display config;
/// mode0 is the integration's metric binding, not a recovered factory setting.
fn update_economy_sample(car: &mut NativeCar, unit_mode: i32) {
    use super::eefloat::{add, div, mul};
    let counter = car.u8(0x5c1).wrapping_sub(1);
    car.set_u8(0x5c1, counter);
    if counter != 0 {
        return;
    }
    let mut distance = add(car.f32(0x6d4), -car.f32(0x5cc));
    let mut fuel = add(car.f32(0x5c4), -car.f32(0x5c8));
    if unit_mode == 1 {
        fuel = mul(fuel, f32::from_bits(0x3e874251));
    } else if unit_mode == 2 {
        fuel = mul(fuel, f32::from_bits(0x3e61413b));
    }
    let result = if unit_mode == 3 {
        if distance < f32::from_bits(0x3a83126e) {
            999
        } else {
            let rate = div(mul(fuel, 1_000_000.0), distance);
            if rate < 999.0 {
                rate as i32
            } else {
                999
            }
        }
    } else {
        if matches!(unit_mode, 1 | 2) {
            distance = mul(distance, f32::from_bits(0x3f1f136a));
        }
        if distance < f32::from_bits(0x3c23d70a) {
            0
        } else if fuel < f32::from_bits(0x358637bd) {
            999
        } else {
            let rate = div(mul(distance, f32::from_bits(0x3c23d70a)), fuel);
            if rate > 999.0 {
                999
            } else {
                rate as i32
            }
        }
    };
    car.set_u8(0x5c1, 30);
    car.set_i16(0x5c2, result as i16);
    car.set_f32(0x5cc, car.f32(0x6d4));
    car.set_f32(0x5c8, car.f32(0x5c4));
}

#[cfg(test)]
mod original_execution_tests {
    use super::*;
    use serde_json::Value;
    fn ordered(bits: u32) -> i64 {
        if bits & 0x80000000 != 0 {
            -(i64::from(bits & 0x7fffffff))
        } else {
            i64::from(bits)
        }
    }
    #[test]
    fn complete_force_conductor_matches_original_3535b0() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_conductor.json"
        ))
        .unwrap();
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 60);
        let params = &NativeParameters::fixture_parameters();
        for (index, c) in fixture["cases"].as_array().unwrap().iter().enumerate() {
            let hex = c["input_car_hex"].as_str().unwrap();
            let mut bytes = [0u8; super::super::native::CAR_BYTES];
            for (i, byte) in bytes.iter_mut().enumerate() {
                *byte = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap();
            }
            let mut car = NativeCar::from_bytes(bytes);
            ForceConductor::default()
                .step(
                    &mut car,
                    params,
                    c["angle_mode"].as_i64().unwrap_or(0) as i32,
                    super::super::automatic::desired_gear,
                )
                .unwrap();
            for (layout, bits) in fixture["output_layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(c["output_bits"].as_array().unwrap())
            {
                let actual = car.u32(layout["offset"].as_u64().unwrap() as usize);
                let expected = u32::from_str_radix(bits.as_str().unwrap(), 16).unwrap();
                if layout["kind"] == "bits" {
                    assert_eq!(actual, expected, "case {index} {}", layout["label"]);
                } else {
                    let ulp = (ordered(actual) - ordered(expected)).abs();
                    assert!(
                        ulp <= 12
                            || super::super::test_support::documented_yaw_roundoff(
                                c,
                                "force_conductor",
                                layout["offset"].as_u64().unwrap() as usize,
                                actual,
                                expected
                            ),
                        "case {index} {}: {ulp} ULP ({actual:08x}/{expected:08x})",
                        layout["label"]
                    );
                }
            }
        }
    }
}
