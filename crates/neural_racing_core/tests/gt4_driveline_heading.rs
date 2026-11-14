//! Independent original-opcode Play EE fixtures; flags are exact and arithmetic <=12 ULP.
use neural_racing_core::gt4::{drivetrain::*, engine::*, tire};
use serde_json::Value;
fn f(c: &Value, k: &str) -> f32 {
    c[k].as_f64().unwrap() as f32
}
fn u(c: &Value, k: &str) -> u64 {
    c[k].as_u64().unwrap()
}
fn fa<const N: usize>(c: &Value, k: &str) -> [f32; N] {
    c[k].as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_f64().unwrap() as f32)
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn ordered(x: u32) -> i64 {
    if x & 0x80000000 != 0 {
        -(i64::from(x & 0x7fffffff))
    } else {
        i64::from(x)
    }
}
fn check(c: &Value, actual: &[f32], bits: &[Value]) {
    for (i, (a, b)) in actual.iter().zip(bits).enumerate() {
        let b = u32::from_str_radix(b.as_str().unwrap(), 16).unwrap();
        let distance = (ordered(a.to_bits()) - ordered(b)).abs();
        assert!(
            distance <= 12,
            "case {} component {i}: {distance} ULP",
            c["index"]
        );
    }
}
#[test]
fn recovered_driveline_and_contact_heading_match_original_execution() {
    let doc: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_driveline_heading.json")).unwrap();
    assert_eq!(doc["cases"].as_array().unwrap().len(), 241);
    for c in doc["cases"].as_array().unwrap() {
        let expected = c["output_bits"].as_array().unwrap();
        if c["kind"] == "velocity_heading" {
            let (heading, fade) =
                tire::contact_velocity_heading(f(c, "velocity_at_28"), f(c, "velocity_at_2c"));
            check(c, &[heading, fade], expected);
            continue;
        }
        let v = &c["state"];
        let mut state = RearDrivelineState {
            dt_at_54c: f(v, "dt_at_54c"),
            throttle_at_598: f(v, "throttle_at_598"),
            curve_torque_at_5ec: f(v, "curve_torque_at_5ec"),
            drag_at_5f0: f(v, "drag_at_5f0"),
            throttle_at_5f4: f(v, "throttle_at_5f4"),
            rpm_at_5fe: v["rpm_at_5fe"].as_i64().unwrap() as i16,
            angular_speed_at_600: f(v, "angular_speed_at_600"),
            throttle_at_604: f(v, "throttle_at_604"),
            coupling_input_at_610: f(v, "coupling_input_at_610"),
            coupling_input_at_614: f(v, "coupling_input_at_614"),
            clutch_ratio_at_61c: f(v, "clutch_ratio_at_61c"),
            torque_magnitude_at_620: f(v, "torque_magnitude_at_620"),
            gear_at_624: u(v, "gear_at_624") as u8,
            limiter_at_625: u(v, "limiter_at_625") as u8,
            clutch_at_626: u(v, "clutch_at_626") as u8,
            carrier_speed_at_630: fa(v, "carrier_speed_at_630"),
            drive_torque_at_638: fa(v, "drive_torque_at_638"),
            wheel_flags_at_46: v["wheel_flags_at_46"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_i64().unwrap() as _)
                .collect(),
            flag_at_70a: u(v, "flag_at_70a") as u8,
        };
        let v = &c["config"];
        let config = RearDrivelineConfig {
            drive_mode_at_234: u(v, "drive_mode_at_234") as u8,
            gearbox_mode_at_246: u(v, "gearbox_mode_at_246") as u8,
            transfer_at_235: v["transfer_at_235"].as_bool().unwrap(),
            driven_group_at_237: u(v, "driven_group_at_237") as usize,
            group_kinds_at_244: v["group_kinds_at_244"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_u64().unwrap() as u8)
                .collect::<Vec<_>>()
                .try_into()
                .unwrap(),
            wheel_drive_signs_at_3c: v["wheel_drive_signs_at_3c"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_i64().unwrap() as _)
                .collect(),
            gear_ratios_at_274: v["gear_ratios_at_274"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap() as f32)
                .collect(),
            clutch_torque_at_2e4: f(v, "clutch_torque_at_2e4"),
            shaft_inertias_at_2f0: fa(v, "shaft_inertias_at_2f0"),
            engine_inverse_inertia_at_2fc: f(v, "engine_inverse_inertia_at_2fc"),
            engine_inertia_at_240: f(v, "engine_inertia_at_240"),
            group_ratios_at_2c4: fa(v, "group_ratios_at_2c4"),
            group_ratios_at_2cc: fa(v, "group_ratios_at_2cc"),
        };
        let wheels: Vec<_> = c["wheels"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| WheelTorqueState {
                speed_at_20: f(v, "speed_at_20"),
                force_at_7c: f(v, "force_at_7c"),
                force_at_80: f(v, "force_at_80"),
                brake_at_78: f(v, "brake_at_78"),
                torque_at_84: f(v, "torque_at_84"),
            })
            .collect();
        let carriers: Vec<_> = c["carriers"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| CarrierWheelGroup {
                group: u(v, "group") as usize,
                first_wheel: u(v, "first_wheel") as usize,
                paired_wheel: v["paired_wheel"].as_u64().map(|x| x as usize),
            })
            .collect();
        let groups: Vec<_> = c["wheel_groups"]
            .as_array()
            .unwrap()
            .iter()
            .map(|x| x.as_u64().unwrap() as usize)
            .collect();
        let v = &c["output_limit"];
        let limit = OutputLimit {
            enabled_at_db: v["enabled_at_db"].as_bool().unwrap(),
            power_at_dc: f(v, "power_at_dc"),
            softness_at_e0: f(v, "softness_at_e0"),
        };
        update_standard_rear_driveline(&mut state, &config, &wheels, &groups, &carriers, |s| {
            naturally_aspirated_torque(s, u(c, "redline_at_200") as u16, limit)
        })
        .unwrap();
        check(
            c,
            &[
                state.angular_speed_at_600,
                state.torque_magnitude_at_620,
                state.carrier_speed_at_630[0],
                state.carrier_speed_at_630[1],
                state.drive_torque_at_638[0],
                state.drive_torque_at_638[1],
                state.throttle_at_5f4,
                state.throttle_at_604,
            ],
            &expected[..8],
        );
        let mut flags = vec![
            u32::from(state.gear_at_624)
                | (u32::from(state.limiter_at_625) << 8)
                | (u32::from(state.clutch_at_626) << 16),
            u32::from(state.flag_at_70a) << 16,
        ];
        flags.extend(state.wheel_flags_at_46.iter().map(|x| u32::from(*x) << 16));
        for (actual, b) in flags.iter().zip(&expected[8..]) {
            assert_eq!(
                *actual,
                u32::from_str_radix(b.as_str().unwrap(), 16).unwrap(),
                "case {} flags",
                c["index"]
            );
        }
    }
}
