//! Original whole 0x36a758 execution, including original suspension callees.
use neural_racing_core::gt4::{normal::*, suspension::*};
use serde_json::Value;
fn f(c: &Value, k: &str) -> f32 {
    c[k].as_f64().unwrap() as f32
}
fn ordered(b: u32) -> i64 {
    if b & 0x80000000 != 0 {
        -i64::from(b & 0x7fffffff)
    } else {
        i64::from(b)
    }
}
#[test]
fn four_wheel_normals_match_original_execution() {
    let d: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_normals.json")).unwrap();
    assert_eq!(
        d["target_sha256"],
        "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
    );
    for c in d["cases"].as_array().unwrap() {
        let airborne = c["airborne"].as_bool().unwrap();
        let mut wheels = std::array::from_fn(|i| NormalWheel {
            suspension: SuspensionState {
                normal_at_48: if i % 2 == 0 { 500. } else { 0. },
                travel_at_94: -0.05 + i as f32 * 0.005,
                velocity_at_98: 0.,
                spring_at_9c: 300.,
                damper_at_a0: -50.,
            },
            body_lateral_force_at_d4: 100. + i as f32 * 20.,
            position_input_at_514: if airborne {
                0.4
            } else {
                -0.08 + i as f32 * 0.01
            },
            contact_normal: [0., 0., 1.],
            ..Default::default()
        });
        let geometry = std::array::from_fn(|i| NormalGeometry {
            axle: i / 2,
            side: (i % 2) as u8,
            fore_aft: if i < 2 { 1 } else { -1 },
            paired_wheel: Some(i ^ 1),
            half_track_at_data_28: if i < 2 { 0.8 } else { 0.82 },
            axle_distance_at_data_10: if i < 2 { 1.3 } else { 1.4 },
            radius_at_data_2c4: if i < 2 { 0.33 } else { 0.35 },
            roll_arm_at_data_ffc: if i < 2 { 0.6 } else { 0.65 },
            base_alignment_at_data_fe4: if i < 2 { 0.01 } else { -0.01 },
            travel_alignment_at_data_fec: if i < 2 { 0.2 } else { 0.25 },
            ackermann_factors: [1.1, 0.9],
        });
        let spring = SuspensionParameters {
            values: std::array::from_fn(|i| c["parameters"][i].as_f64().unwrap() as f32),
        };
        let g = &c["globals"];
        let damper = DamperGlobals {
            mode_at_620dd9: g["mode_at_620dd9"].as_u64().unwrap() as u8,
            blend_enabled_at_620cb0: false,
            negative_speed_at_620cb8: f(g, "negative_speed_at_620cb8"),
            negative_blend_at_620cc0: f(g, "negative_blend_at_620cc0"),
            positive_speed_at_620cc8: f(g, "positive_speed_at_620cc8"),
            positive_blend_at_620cd0: f(g, "positive_blend_at_620cd0"),
        };
        let parameters = NormalParameters {
            suspension: [spring; 2],
            damper: [damper; 2],
            spring_rate: [10000.; 4],
            context: SuspensionContext {
                dt_at_54c: 1. / 60.,
                inverse_dt_at_550: 60.,
                alternate_limit_at_5be: false,
                external_force_enabled_at_1076: false,
            },
            world_velocity_at_648: [10., 2., -0.1],
            up_at_128: [0.02, -0.03, 0.999],
            pitch_rate_at_684: 0.1,
            roll_rate_at_688: -0.2,
            steer_at_56c: f(c, "steer"),
            steer_alignment_at_data_1004: 0.3,
            lateral_acceleration_at_658: 3.,
            longitudinal_force_at_668: 2000.,
            lateral_transfer_at_data_34: 0.12,
            longitudinal_transfer_at_data_38: 0.1,
            aero_at_6ac: 100.,
            aero_at_6b0: 200.,
            front_wheel_count: 2,
            rear_wheel_count: 2,
            local_transfer_at_620e11: c["local_transfer"].as_bool().unwrap(),
            height_at_data_1060: 0.3,
        };
        update_normals(&mut wheels, &geometry, &parameters, |_, _| {
            panic!("disabled external spring callback")
        });
        let mut actual = Vec::new();
        for w in wheels {
            actual.extend(
                [
                    w.alignment_at_04,
                    w.lateral_position_at_08,
                    w.suspension.normal_at_48,
                    w.suspension.travel_at_94,
                    w.suspension.velocity_at_98,
                    w.suspension.spring_at_9c,
                    w.suspension.damper_at_a0,
                    w.normal_velocity_at_d8,
                ]
                .map(f32::to_bits),
            );
            actual.push(u32::from(w.contact_transition_at_dc));
            actual.push(w.body_force_at_524.to_bits());
        }
        assert_eq!(actual.len(), c["output_bits"].as_array().unwrap().len());
        assert_eq!(actual.len(), 40);
        for (j, (a, b)) in actual
            .iter()
            .zip(c["output_bits"].as_array().unwrap())
            .enumerate()
        {
            let b = u32::from_str_radix(b.as_str().unwrap(), 16).unwrap();
            if j % 10 == 8 {
                assert_eq!(*a, b, "contact transition: {c}");
            } else {
                let ulp = (ordered(*a) - ordered(b)).abs();
                assert!(ulp <= 12, "component{j} {ulp}ULP: {c}");
            }
        }
    }
}
