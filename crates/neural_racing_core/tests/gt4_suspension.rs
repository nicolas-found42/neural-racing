//! Original-opcode Play reference, independent of the Rust implementations.
use neural_racing_core::gt4::suspension::*;
use serde_json::Value;
fn f(c: &Value, key: &str) -> f32 {
    c[key].as_f64().unwrap() as f32
}
fn ordered(x: u32) -> i64 {
    if x & 0x80000000 != 0 {
        -(i64::from(x & 0x7fffffff))
    } else {
        i64::from(x)
    }
}
#[test]
fn recovered_suspension_matches_original_execution() {
    let document: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_suspension.json")).unwrap();
    for c in document["cases"].as_array().unwrap() {
        let parameters = SuspensionParameters {
            values: c["parameters"]
                .as_array()
                .unwrap()
                .iter()
                .map(|x| x.as_f64().unwrap() as f32)
                .collect::<Vec<_>>()
                .try_into()
                .unwrap(),
        };
        let g = &c["globals"];
        let globals = DamperGlobals {
            mode_at_620dd9: g["mode_at_620dd9"].as_u64().unwrap() as u8,
            blend_enabled_at_620cb0: g["blend_enabled_at_620cb0"].as_bool().unwrap(),
            negative_speed_at_620cb8: f(g, "negative_speed_at_620cb8"),
            negative_blend_at_620cc0: f(g, "negative_blend_at_620cc0"),
            positive_speed_at_620cc8: f(g, "positive_speed_at_620cc8"),
            positive_blend_at_620cd0: f(g, "positive_blend_at_620cd0"),
        };
        let actual = if c["kind"] == "damper" {
            vec![damper_response(f(c, "velocity"), &parameters, &globals)]
        } else {
            let s = &c["state"];
            let mut state = SuspensionState {
                normal_at_48: f(s, "normal_at_48"),
                travel_at_94: f(s, "travel_at_94"),
                velocity_at_98: f(s, "velocity_at_98"),
                spring_at_9c: f(s, "spring_at_9c"),
                damper_at_a0: f(s, "damper_at_a0"),
            };
            let x = &c["context"];
            let context = SuspensionContext {
                dt_at_54c: f(x, "dt_at_54c"),
                inverse_dt_at_550: f(x, "inverse_dt_at_550"),
                alternate_limit_at_5be: x["alternate_limit_at_5be"].as_bool().unwrap(),
                external_force_enabled_at_1076: false,
            };
            update_suspension(
                f(c, "position_input"),
                f(c, "spring_rate"),
                &mut state,
                &parameters,
                &globals,
                &context,
                |_| panic!("disabled callback invoked"),
            );
            vec![
                state.normal_at_48,
                state.travel_at_94,
                state.velocity_at_98,
                state.spring_at_9c,
                state.damper_at_a0,
            ]
        };
        let expected = if let Some(a) = c["output_bits"].as_array() {
            a.clone()
        } else {
            vec![c["bits"].clone()]
        };
        assert_eq!(actual.len(), expected.len());
        for (j, (a, b)) in actual.iter().zip(expected).enumerate() {
            let b = u32::from_str_radix(b.as_str().unwrap(), 16).unwrap();
            let distance = (ordered(a.to_bits()) - ordered(b)).abs();
            assert!(
                distance <= 12,
                "case {} component{j}: {distance} ULP",
                c["index"]
            );
        }
    }
}
