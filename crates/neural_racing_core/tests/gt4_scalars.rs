//! Differential fixtures execute original game opcodes, not a second Rust formula.
use neural_racing_core::gt4;

use gt4::curve::{Curve, PairedCurve, RAW_620C08_X, RAW_620C30_Y, RAW_620C58_Y};
use gt4::engine::{
    corrected_curve_value, CorrectionState, EngineCorrection, ORIGINAL_CORRECTION_BYTES,
};
use gt4::slip::{update_slip, SlipParameters, WheelSlipState};
use serde_json::Value;

fn float(case: &Value, key: &str) -> f32 {
    case[key].as_f64().unwrap() as f32
}
fn bits(value: &Value) -> u32 {
    u32::from_str_radix(value.as_str().unwrap(), 16).unwrap()
}
fn ordered_bits(value: f32) -> i64 {
    let b = value.to_bits();
    if b & 0x80000000 != 0 {
        i64::from(!b)
    } else {
        i64::from(b | 0x80000000)
    }
}
fn compare(result: f32, case: &Value, suffix: &str) -> i64 {
    // Exact comparison to IEEE-mode original opcodes isolates port control flow
    // from the separate original-opcode Play! EE reference's rounding sensitivity.
    assert_eq!(
        result.to_bits(),
        bits(&case[format!("ieee_{suffix}")]),
        "IEEE-mode original opcode mismatch for {suffix}: {case}"
    );
    let ee = f32::from_bits(bits(&case[format!("ee_{suffix}")]));
    let error = (ordered_bits(result) - ordered_bits(ee)).abs();
    assert!(error <= 4, "Play EE drift {error} ULP for {suffix}: {case}");
    error
}
fn config(case: &Value) -> SlipParameters {
    let bytes: Vec<u8> = case["raw"]
        .as_array()
        .unwrap()
        .iter()
        .map(|v| v.as_u64().unwrap() as u8)
        .collect();
    SlipParameters::from_bytes(bytes.try_into().unwrap())
}

#[test]
fn recovered_scalar_ports_match_original_opcode_execution() {
    let fixture: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_scalars.json")).unwrap();
    assert_eq!(
        fixture["target_sha256"],
        "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
    );
    assert_eq!(
        fixture["authority"],
        "controlled-original-opcode-execution-on-upstream-Play-EE"
    );
    assert_eq!(
        fixture["emulator"]["commit"],
        "1f27232772e484bfb79364b751b84f41f4c87481"
    );
    let correction = EngineCorrection::from_tuning_bytes(ORIGINAL_CORRECTION_BYTES);
    let mut checked = 0;
    let mut worst_ee_ulp = 0;
    for case in fixture["cases"].as_array().unwrap() {
        let curve = || {
            Curve::new(
                &RAW_620C08_X,
                if case["descriptor"] == "0x620c90" {
                    &RAW_620C58_Y
                } else {
                    &RAW_620C30_Y
                },
            )
            .unwrap()
        };
        let result = match case["kind"].as_str().unwrap() {
            "curve" => Some(curve().sample(float(case, "x"))),
            "curve_linear" => Some(curve().sample_linear(float(case, "x"))),
            "curve_paired" => {
                let pair = PairedCurve::new(&RAW_620C08_X, &RAW_620C30_Y, &RAW_620C58_Y).unwrap();
                let (first, second) = pair.sample(float(case, "x"));
                worst_ee_ulp = worst_ee_ulp.max(compare(first, case, "first_bits"));
                worst_ee_ulp = worst_ee_ulp.max(compare(second, case, "second_bits"));
                assert_eq!(case["return_value"], 1);
                None
            }
            "angle_fold" => {
                let (value, direction) = gt4::tire::fold_slip_angle(float(case, "input"));
                worst_ee_ulp = worst_ee_ulp.max(compare(direction, case, "direction_bits"));
                Some(value)
            }
            "correction" => {
                if let Some(expected_params) = case["ieee_params_bits"].as_array() {
                    for (value, expected) in
                        correction.initialized_values().iter().zip(expected_params)
                    {
                        assert_eq!(
                            value.to_bits(),
                            bits(expected),
                            "initializer mismatch: {case}"
                        );
                    }
                }
                Some(correction.factor(&CorrectionState {
                    input_at_04: float(case, "x"),
                    input_at_08: float(case, "y"),
                    bypass: case["bypass"].as_bool().unwrap(),
                }))
            }
            "wrapper" => {
                let state = CorrectionState {
                    input_at_04: float(case, "x"),
                    input_at_08: float(case, "y"),
                    bypass: case["bypass"].as_bool().unwrap(),
                };
                let explicit = (case["state_kind"] != "null").then_some(&state);
                Some(corrected_curve_value(
                    curve(),
                    float(case, "query"),
                    explicit,
                    &correction,
                ))
            }
            "slip_gain" => Some(config(case).gain(
                float(case, "input"),
                case["branch"].as_u64().unwrap() as usize,
            )),
            "slip" => {
                let mut wheel = WheelSlipState {
                    input_at_20: float(case, "base"),
                    input_at_28: float(case, "helper_input"),
                    ..Default::default()
                };
                update_slip(
                    float(case, "input"),
                    &mut wheel,
                    &config(case),
                    case["alternate"].as_bool().unwrap(),
                    float(case, "alternate_gain"),
                );
                assert_eq!(
                    i64::from(wheel.mode_at_44),
                    case["mode"].as_i64().unwrap(),
                    "mode: {case}"
                );
                assert_eq!(
                    i64::from(wheel.direction_at_45),
                    case["direction"].as_i64().unwrap(),
                    "direction: {case}"
                );
                worst_ee_ulp = worst_ee_ulp.max(compare(wheel.ratio_at_38, case, "ratio_bits"));
                worst_ee_ulp = worst_ee_ulp.max(compare(wheel.gain_at_3c, case, "gain_bits"));
                None
            }
            kind => panic!("unrecognized original-opcode fixture {kind}"),
        };
        assert!(case["original_instructions"].as_u64().unwrap() > 0);
        assert_eq!(case["play_stopping_pc"], "01fff000");
        if let Some(result) = result {
            worst_ee_ulp = worst_ee_ulp.max(compare(result, case, "bits"));
        }
        checked += 1;
    }
    assert_eq!(checked, 280);
    assert_ne!(
        fixture["negative_control"]["original_bits"],
        fixture["negative_control"]["mutated_bits"]
    );
    println!(
        "{checked} original-opcode cases matched; maximum Play EE difference {worst_ee_ulp} ULP"
    );
}
