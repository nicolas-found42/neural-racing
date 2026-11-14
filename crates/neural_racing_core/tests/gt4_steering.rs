//! Controlled original-opcode execution fixtures, independent of the Rust port.
use neural_racing_core::gt4::steering::{
    approach, update_response, SteeringResponse, SteeringState, SteeringTable,
};
use serde_json::Value;

fn f(case: &Value, key: &str) -> f32 {
    case[key].as_f64().unwrap() as f32
}
fn bits(v: &Value) -> u32 {
    u32::from_str_radix(v.as_str().unwrap(), 16).unwrap()
}
fn ordered(v: f32) -> i64 {
    let b = v.to_bits();
    i64::from(if b & 0x8000_0000 != 0 {
        !b
    } else {
        b | 0x8000_0000
    })
}

#[test]
fn steering_ports_match_original_play_execution() {
    let fixture: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_steering.json")).unwrap();
    assert_eq!(
        fixture["target_sha256"],
        "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
    );
    assert_eq!(
        fixture["emulator"]["commit"],
        "1f27232772e484bfb79364b751b84f41f4c87481"
    );
    let mut checked = 0;
    let mut controls = 0;
    for case in fixture["cases"].as_array().unwrap() {
        let values = match case["kind"].as_str().unwrap() {
            "steering_response" => {
                let mut state = SteeringState {
                    angle: f(case, "current"),
                    rate: f(case, "rate"),
                };
                update_response(
                    &mut state,
                    f(case, "target"),
                    f(case, "heading"),
                    f(case, "speed"),
                    f(case, "dt"),
                    case["analog"].as_bool().unwrap(),
                    &SteeringResponse::original(case["group"].as_u64().unwrap() as usize),
                );
                vec![state.angle, state.rate]
            }
            "steering_approach" => {
                let mut value = f(case, "current");
                approach(f(case, "target"), f(case, "step"), &mut value);
                vec![value]
            }
            "steering_table" => {
                let bytes = |key: &str| -> [u8; 6] {
                    std::array::from_fn(|i| case[key][i].as_u64().unwrap() as u8)
                };
                let table = SteeringTable::from_bytes(bytes("xs"), bytes("ys"));
                table.speeds.into_iter().chain(table.angles).collect()
            }
            _ => unreachable!(),
        };
        if let Some(control) = case.get("ieee_control_bits") {
            controls += 1;
            for (actual, expected) in values.iter().zip(control.as_array().unwrap()) {
                assert_eq!(actual.to_bits(), bits(expected), "IEEE control: {case}");
            }
        }
        for (i, value) in values.iter().enumerate() {
            let expected = f32::from_bits(bits(&case["output_bits"][i]));
            let ulp = (ordered(*value) - ordered(expected)).abs();
            // Six small-rate cancellation cases are bit-exact in independent
            // original-opcode IEEE sensitivity execution. Their primary EE
            // drift is at most 5.960464477539063e-8 (32 ULP at that rate).
            let measured_rate_cancellation = case["kind"] == "steering_response"
                && i == 1
                && case.get("ieee_control_bits").is_some()
                && (*value - expected).abs() <= 1e-6;
            assert!(
                ulp <= 12 || measured_rate_cancellation,
                "original Play mismatch output {i}, {ulp} ULP: {case}"
            );
        }
        checked += 1;
    }
    assert_eq!(checked, 92);
    assert_eq!(controls, 6);
}
