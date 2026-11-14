//! Actual table-load through force pipeline, all14 original compound rows.
use neural_racing_core::gt4::{curve::Curve, slip::WheelSlipState, tire::*};
use serde_json::Value;
fn f(c: &Value, k: &str) -> f32 {
    c[k].as_f64().unwrap() as f32
}
fn u(c: &Value, k: &str) -> u8 {
    c[k].as_u64().unwrap() as u8
}
fn signed(c: &Value, k: &str) -> i8 {
    c[k].as_i64().unwrap() as i8
}
fn bytes<const N: usize>(c: &Value, k: &str) -> [u8; N] {
    c[k].as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap() as u8)
        .collect::<Vec<_>>()
        .try_into()
        .unwrap()
}
fn ordered(b: u32) -> i64 {
    if b & 0x80000000 != 0 {
        -(i64::from(b & 0x7fffffff))
    } else {
        i64::from(b)
    }
}
#[test]
fn coupled_tire_forces_match_original_guest_calls() {
    let doc: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_tire_forces.json")).unwrap();
    assert_eq!(doc["cases"].as_array().unwrap().len(), 126);
    for c in doc["cases"].as_array().unwrap() {
        let compound: [u8; 90] = bytes(c, "compound_bytes");
        let volume = bytes(c, "volume_bytes");
        let mut expanded = [0u8; 168];
        expand_compound_into(&mut expanded, &compound, &volume);
        let tables = TireTables::from_runtime(&mut expanded, 1.0).unwrap();
        let mut load = LoadScaleState {
            camber_at_04: f(c, "camber"),
            normal_at_48: f(c, "normal"),
            scale_at_4c: f(c, "scale"),
            wear_at_50: 0.8,
            terrain_at_89: u(c, "terrain"),
            flag_at_dd: 0,
        };
        update_load_scale(
            &mut load,
            &tables,
            Curve::new(&[-0.5, 0., 0.5], &[0.8, 1., 0.8]).unwrap(),
            &LoadScaleContext {
                width_at_2d4: f(c, "width"),
                axle_multiplier_at_fd0: f(c, "axle_multiplier"),
                vehicle_mode: 0,
                wheel_side: u(c, "side_mode"),
                body_angle_at_670: f(c, "roll"),
                body_angle_at_67c: 0.,
                tire_mode_at_1da: compound[89],
                apply_wear_factor: false,
                control_grip_factor: 1.,
            },
        );
        let (heading, fade) = contact_velocity_heading(f(c, "vx"), f(c, "vy"));
        let (mu_a, mu_b) = tables.slip_pair().sample(f(c, "ratio"));
        let slip = WheelSlipState {
            gain_at_3c: f(c, "gain"),
            mode_at_44: signed(c, "mode44"),
            direction_at_45: signed(c, "direction45"),
            ..Default::default()
        };
        let modes: [u8; 2] = bytes(c, "gain_modes");
        let split = split_longitudinal_force(&slip, mu_a, load.scale_at_4c, modes[0], modes[1]);
        let mut w = WheelForceState {
            angle_at_00: f(c, "angle"),
            offset_at_04: f(c, "camber"),
            correction_at_1c: 0.02,
            velocity_at_20: f(c, "vx"),
            velocity_at_28: f(c, "vx"),
            velocity_at_2c: f(c, "vy"),
            heading_at_30: heading,
            slip_at_34: 456.,
            scale_at_4c: load.scale_at_4c,
            limit_at_58: mu_b,
            curve_at_5c: 777.,
            fade_at_60: fade,
            force_at_68: 999.,
            force_at_6c: 111.,
            car_force_slot: 444.,
        };
        update_contact_force(
            &mut w,
            &tables,
            &TireForceContext {
                angle_mode: i32::from(u(c, "angle_mode")),
                mode_at_1d9: compound[88],
                mode_at_1da: compound[89],
                velocity_gain_at_fcc: f(c, "velocity_gain"),
                wheel_kind: u(c, "wheel_kind"),
                side_mode: u(c, "side_mode"),
                car_angle_at_670: f(c, "roll"),
                car_angle_at_67c: 0.,
                global_at_620764: 5.,
                global_at_620768: 0.1,
            },
        );
        let actual = [
            w.angle_at_00,
            w.heading_at_30,
            w.slip_at_34,
            load.scale_at_4c,
            mu_a,
            mu_b,
            w.curve_at_5c,
            w.fade_at_60,
            w.force_at_68,
            w.force_at_6c,
            split.at_70,
            split.at_74,
            split.at_7c,
            split.at_80,
            w.car_force_slot,
        ];
        let expected = c["output_bits"].as_array().unwrap();
        for (i, (a, b)) in actual.iter().zip(&expected[..15]).enumerate() {
            let b = u32::from_str_radix(b.as_str().unwrap(), 16).unwrap();
            let ulp = (ordered(a.to_bits()) - ordered(b)).abs();
            assert!(ulp <= 12, "case{} component{i}: {ulp}ULP", c["index"]);
        }
        assert_eq!(
            u32::from(load.flag_at_dd) << 8,
            u32::from_str_radix(expected[15].as_str().unwrap(), 16).unwrap(),
            "case{}flags",
            c["index"]
        );
    }
}
