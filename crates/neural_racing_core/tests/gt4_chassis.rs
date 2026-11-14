//! Independent original-opcode Play references; no audit checkout is required.
use neural_racing_core::gt4;
use serde_json::Value;
fn f(v: &Value, key: &str) -> f32 {
    v[key].as_f64().unwrap() as f32
}
fn a<const N: usize>(v: &Value) -> [f32; N] {
    std::array::from_fn(|i| v[i].as_f64().unwrap() as f32)
}
fn bits(v: &Value) -> u32 {
    u32::from_str_radix(v.as_str().unwrap(), 16).unwrap()
}
fn ordered(b: u32) -> i64 {
    if b & 0x80000000 != 0 {
        -i64::from(b & 0x7fffffff)
    } else {
        i64::from(b)
    }
}
macro_rules! fields { ($ty:path,$v:expr, $($name:ident),+ $(,)?) => {{ use $ty as FixtureType; FixtureType { $($name:f($v,stringify!($name))),+ } }}; }
#[test]
fn recovered_chassis_math_matches_original_execution() {
    let document: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_chassis.json")).unwrap();
    assert_eq!(
        document["target_sha256"],
        "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
    );
    for c in document["cases"].as_array().unwrap() {
        let kind = c["kind"].as_str().unwrap();
        let values: Vec<f32> = match kind {
            "sin" => vec![gt4::math::sin(f(c, "input"))],
            "cos" => vec![gt4::math::cos(f(c, "input"))],
            "wrap_angle" => vec![gt4::math::wrap_angle(f(c, "input"))],
            "angle_difference" => vec![gt4::math::angle_difference(f(c, "input"), f(c, "other"))],
            "sin_cos" => {
                let (s, co) = gt4::math::sin_cos(f(c, "input"));
                vec![s, co]
            }
            "body" => {
                let mut s = gt4::body::BodyState {
                    angles: a(&c["angles"]),
                    position: a(&c["position"]),
                    velocity: a(&c["velocity"]),
                    angular_velocity: a(&c["omega"]),
                };
                let p = gt4::body::BodyParameters {
                    mass: f(c, "mass"),
                    inertia: a(&c["inertia"]),
                };
                gt4::body::integrate_unconstrained(
                    &mut s,
                    p,
                    a(&c["force"]),
                    a(&c["torque"]),
                    f(c, "dt"),
                );
                s.angles
                    .into_iter()
                    .chain(s.position)
                    .chain(s.velocity)
                    .chain(s.angular_velocity)
                    .collect()
            }
            "body_basis" => {
                let x: [f32; 3] = a(&c["angles"]);
                let b = gt4::attitude::body_basis(x[0], x[1], x[2]);
                b.forward_at_110
                    .into_iter()
                    .chain(b.lateral_at_11c)
                    .chain(b.up_at_128)
                    .collect()
            }
            "world_velocity" => {
                let s = &c["state"];
                let mut state = gt4::chassis::VelocityState {
                    forward_at_640: f(s, "forward_at_640"),
                    lateral_at_644: f(s, "lateral_at_644"),
                    world_xy_at_648: a(&s["world_xy_at_648"]),
                    longitudinal_acceleration_at_654: f(s, "longitudinal_acceleration_at_654"),
                    lateral_acceleration_at_658: f(s, "lateral_acceleration_at_658"),
                    vertical_rate_at_668: f(s, "vertical_rate_at_668"),
                    yaw_rate_at_680: f(s, "yaw_rate_at_680"),
                    normalized_lateral_at_690: f(s, "normalized_lateral_at_690"),
                };
                let x = &c["context"];
                let context = gt4::chassis::VelocityContext {
                    stop_at_5be: x["stop_at_5be"].as_i64().unwrap() as i16,
                    held_at_5c0: x["held_at_5c0"].as_bool().unwrap(),
                    mode_at_56b: x["mode_at_56b"].as_u64().unwrap() as u8,
                    mode_at_56a: x["mode_at_56a"].as_u64().unwrap() as u8,
                    brake_at_590: f(x, "brake_at_590"),
                    brake_at_594: f(x, "brake_at_594"),
                    dt_at_54c: f(x, "dt_at_54c"),
                    forward_basis_xy_at_110: a(&x["forward_basis_xy_at_110"]),
                    lateral_basis_xy_at_11c: a(&x["lateral_basis_xy_at_11c"]),
                };
                gt4::chassis::update_world_velocity(
                    &mut state,
                    context,
                    &a::<4>(&c["wheel_forces_at_60"]),
                    c["suspended"].as_bool().unwrap(),
                );
                vec![
                    state.forward_at_640,
                    state.lateral_at_644,
                    state.world_xy_at_648[0],
                    state.world_xy_at_648[1],
                    state.longitudinal_acceleration_at_654,
                    state.lateral_acceleration_at_658,
                    state.vertical_rate_at_668,
                    state.yaw_rate_at_680,
                    state.normalized_lateral_at_690,
                ]
            }
            "chassis" => {
                let mut state = fields!(
                    gt4::chassis::ChassisState,
                    &c["state"],
                    lateral_acceleration_at_658,
                    yaw_rate_at_680,
                    yaw_acceleration_at_68c,
                    normalized_lateral_at_690,
                    yaw_filter_at_740,
                    yaw_filter_rate_at_744
                );
                let p = &c["parameters"];
                let parameters = gt4::chassis::ChassisParameters {
                    mass_at_5d8: f(p, "mass_at_5d8"),
                    inverse_yaw_inertia_at_data_118: f(p, "inverse_yaw_inertia_at_data_118"),
                    dt_at_54c: f(p, "dt_at_54c"),
                    angle_mode: p["angle_mode"].as_i64().unwrap() as i32,
                };
                let mut wheels: Vec<_> = c["wheels"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|w| gt4::chassis::WheelChassisState {
                        steer_at_00: f(w, "steer_at_00"),
                        secondary_angle_at_1c: f(w, "secondary_angle_at_1c"),
                        rim_speed_at_20: f(w, "rim_speed_at_20"),
                        forward_velocity_at_28: f(w, "forward_velocity_at_28"),
                        lateral_velocity_at_2c: f(w, "lateral_velocity_at_2c"),
                        velocity_heading_at_30: f(w, "velocity_heading_at_30"),
                        grip_at_4c: f(w, "grip_at_4c"),
                        lateral_force_at_68: f(w, "lateral_force_at_68"),
                        corner_drag_at_6c: f(w, "corner_drag_at_6c"),
                        drive_force_at_70: f(w, "drive_force_at_70"),
                        brake_force_at_74: f(w, "brake_force_at_74"),
                        body_lateral_force_at_d4: f(w, "body_lateral_force_at_d4"),
                        force_direction_at_dd: w["force_direction_at_dd"].as_u64().unwrap() as u8,
                    })
                    .collect();
                let geometry: Vec<_> = c["geometry"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .map(|g| {
                        fields!(
                            gt4::chassis::WheelGeometry,
                            g,
                            signed_half_track,
                            signed_axle_distance,
                            additional_lateral_force,
                            normalization_grip
                        )
                    })
                    .collect();
                gt4::chassis::aggregate_forces(
                    &mut state,
                    &mut wheels,
                    &geometry,
                    parameters,
                    c["suspended"].as_bool().unwrap(),
                );
                [
                    state.lateral_acceleration_at_658,
                    state.yaw_rate_at_680,
                    state.yaw_acceleration_at_68c,
                    state.normalized_lateral_at_690,
                    state.yaw_filter_at_740,
                    state.yaw_filter_rate_at_744,
                ]
                .into_iter()
                .chain(wheels.iter().map(|w| w.body_lateral_force_at_d4))
                .collect()
            }
            "attitude" => {
                // The native fixture uses deliberately controlled coefficients, not car calibration.
                let angles: [f32; 3] = a(&c["angles"]);
                let mut s = gt4::attitude::AttitudeState {
                    world_velocity_at_648: [10., 1., -0.1],
                    pitch_at_66c: angles[0],
                    roll_at_670: angles[1],
                    heading_at_674: angles[2],
                    pitch_rate_at_684: 0.02,
                    roll_rate_at_688: -0.03,
                    ..Default::default()
                };
                let p = gt4::attitude::AttitudeParameters {
                    dt_at_54c: f(c, "dt"),
                    vertical_mass_at_5dc: 1450.,
                    pitch_coefficients_at_data_11c: [0.001, 0.0011],
                    roll_coefficients_at_data_124: [0.0012, 0.0013],
                    vertical_mode_at_620dec: c["mode"].as_u64().unwrap() as u8,
                    road_normal_at_158: [0., 0., 1.],
                    aero_force_at_6ac: 100.,
                    aero_force_at_6b0: 200.,
                    ..Default::default()
                };
                let wheels: Vec<_> = (0..4)
                    .map(|i| gt4::attitude::AttitudeWheel {
                        axle: i / 2,
                        side: (i % 2) as u8,
                        fore_aft: if i < 2 { 1 } else { -1 },
                        steer_at_00: if i < 2 { 0.1 } else { 0. },
                        normal_at_48: 3500. + i as f32 * 100.,
                        drive_at_70: 500.,
                        brake_at_74: -50.,
                        damper_at_a0: 30. + i as f32 * 10.,
                        force_at_524: 100. + i as f32 * 10.,
                        radius_at_data_2c4: if i < 2 { 0.33 } else { 0.35 },
                        axle_distance_at_data_10: if i < 2 { 1.3 } else { 1.4 },
                        reaction_coefficient_at_data_12c: if i < 2 { 0.05 } else { 0.06 },
                        static_force_at_data_344: if i < 2 { 3000. } else { 3100. },
                        contact_normal: [0., 0., 1.],
                    })
                    .collect();
                gt4::attitude::update_attitude(&mut s, &wheels, p, false);
                s.world_velocity_at_648
                    .into_iter()
                    .chain([
                        s.vertical_acceleration_at_65c,
                        s.pitch_at_66c,
                        s.roll_at_670,
                        s.heading_at_674,
                        s.pitch_rate_at_684,
                        s.roll_rate_at_688,
                    ])
                    .chain(s.basis.forward_at_110)
                    .chain(s.basis.lateral_at_11c)
                    .chain(s.basis.up_at_128)
                    .collect()
            }
            _ => panic!("unknown case {kind}"),
        };
        let expected: Vec<u32> = if let Some(xs) = c["output_bits"].as_array() {
            xs.iter().map(bits).collect()
        } else if kind == "sin_cos" {
            vec![bits(&c["sin_bits"]), bits(&c["cos_bits"])]
        } else {
            vec![bits(&c["bits"])]
        };
        assert_eq!(values.len(), expected.len());
        for (j, (v, b)) in values.into_iter().zip(expected).enumerate() {
            let ulp = (ordered(v.to_bits()) - ordered(b)).abs();
            let absolute = (v - f32::from_bits(b)).abs();
            assert!(
                ulp <= 12 || (kind == "chassis" && j == 2 && absolute <= 1e-6),
                "case{} {kind} component{j}: {ulp}ULP",
                c["index"]
            );
        }
    }
}
