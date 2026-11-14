//! Fixture-only gates for the independently derived two-frame yaw error proof.
use serde_json::Value;
use std::sync::OnceLock;

pub(crate) fn documented_yaw_roundoff(
    input: &Value,
    kind: &str,
    offset: usize,
    actual: u32,
    expected: u32,
) -> bool {
    if ![0x680, 0x68c, 0x740, 0x744].contains(&offset) {
        return false;
    }
    static PROOF: OnceLock<Value> = OnceLock::new();
    let proof = PROOF.get_or_init(|| {
        let proof: Value = serde_json::from_str(include_str!(
            "../../../../docs/reference/gt4/yaw-forward-error.json"
        ))
        .unwrap();
        let parameters: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_native_parameter_inputs.json"
        ))
        .unwrap();
        for (name, identity) in proof["parameter_inputs"].as_object().unwrap() {
            assert_eq!(*identity, parameters[name]["sha256"]);
        }
        assert_eq!(proof["target_sha256"], parameters["original_target_sha256"]);
        proof
    });
    let Some(case) = proof["cases"].as_array().unwrap().iter().find(|case| {
        case["kind"] == kind
            && case["angle_mode"].as_i64().unwrap() == input["angle_mode"].as_i64().unwrap_or(0)
            && case["captured_car_buffers"]["input-car"]["hex"] == input["input_car_hex"]
    }) else {
        return false;
    };
    let Some(bound) = case["output_offset_bounds"][format!("0x{offset:x}")].as_f64() else {
        return false;
    };
    let captured = case["captured_car_buffers"]["native-post"]["hex"]
        .as_str()
        .unwrap();
    let mut bytes = [0u8; 4];
    for (i, byte) in bytes.iter_mut().enumerate() {
        let index = (offset + i) * 2;
        *byte = u8::from_str_radix(&captured[index..index + 2], 16).unwrap();
    }
    assert_eq!(
        expected,
        u32::from_le_bytes(bytes),
        "proof target output changed"
    );
    (f64::from(f32::from_bits(actual)) - f64::from(f32::from_bits(expected))).abs() <= bound
}
