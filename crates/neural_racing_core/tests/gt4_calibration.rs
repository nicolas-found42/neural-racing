//! Original-opcode automatic gearing, tire units and contact-velocity fixtures.
use neural_racing_core::gt4::{calibration::*, kinematics::*};
use serde_json::Value;

fn f(value: &Value) -> f32 {
    value.as_f64().unwrap() as f32
}
fn ordered(bits: u32) -> i64 {
    if bits & 0x8000_0000 != 0 {
        -i64::from(bits & 0x7fff_ffff)
    } else {
        i64::from(bits)
    }
}

#[test]
fn recovered_calibration_and_contact_velocities_match_original() {
    let fixture: Value =
        serde_json::from_str(include_str!("fixtures/gt4_original_calibration.json")).unwrap();
    assert_eq!(
        fixture["target_sha256"],
        "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
    );
    let cases = fixture["cases"].as_array().unwrap();
    assert_eq!(cases.len(), 42);
    for (case_index, case) in cases.iter().enumerate() {
        let kind = case["kind"].as_str().unwrap();
        let mut actual = Vec::new();
        if kind == "kinematics" {
            let mut wheels = [ContactVelocity::default(); 4];
            let geometry = std::array::from_fn::<_, 4, _>(|i| ContactGeometry {
                yaw_lateral_lever: if i < 2 { -1.3 } else { 1.4 },
                signed_half_track: (if i < 2 { 0.8 } else { 0.82 })
                    * (if i % 2 == 0 { -1.0 } else { 1.0 }),
                height: if i < 2 { 0.5 } else { 0.6 },
            });
            let (forward, lateral) = update_contact_velocities(
                &mut wheels,
                &geometry,
                ContactContext {
                    world_velocity_at_648: [10.0, 2.0, 0.1],
                    road_forward_at_140: std::array::from_fn(|i| f(&case["forward"][i])),
                    road_lateral_at_14c: std::array::from_fn(|i| f(&case["lateral"][i])),
                    yaw_rate_at_680: f(&case["yaw"]),
                    roll_rate_at_688: 0.2,
                    body_up_z_at_130: 0.99,
                    roll_coupling_at_620784: case["roll_coupling"].as_bool().unwrap(),
                },
            );
            actual.extend([forward.to_bits(), lateral.to_bits()]);
            for wheel in wheels {
                actual.extend([wheel.forward_at_28.to_bits(), wheel.lateral_at_2c.to_bits()]);
            }
        } else {
            let dimensions = tire_dimensions(TireDimensionsBytes {
                width_at_1c8: case["width"].as_u64().unwrap() as u8,
                rim_at_1ca: case["rim"].as_u64().unwrap() as u8,
                sidewall_at_1cc: case["sidewall"].as_u64().unwrap() as u8,
                rim_adjustment_at_1e2: case["adjust"].as_i64().unwrap() as i8,
                width_percent_at_1e4: case["percent"].as_u64().unwrap() as u8,
            });
            if kind == "auto_gears" {
                let mut ratios = [-1; 12];
                build_auto_ratios(
                    AutoGearParameters {
                        gear_count_at_155: case["count"].as_u64().unwrap() as u8,
                        redline_at_bd: case["redline"].as_u64().unwrap() as u8,
                        final_ratio_at_150: case["final_ratio"].as_i64().unwrap() as i16,
                        selected_wheel_radius: dimensions[0],
                    },
                    case["speed"].as_i64().unwrap() as i32,
                    &mut ratios,
                );
                actual.extend(
                    ratios
                        .chunks_exact(2)
                        .map(|p| u32::from(p[0] as u16) | (u32::from(p[1] as u16) << 16)),
                );
            } else {
                assert_eq!(kind, "tire_dimensions");
                actual.extend(dimensions.map(f32::to_bits));
            }
        }
        let expected = case["output_bits"].as_array().unwrap();
        assert_eq!(actual.len(), expected.len());
        for (component, (actual, expected)) in actual.into_iter().zip(expected).enumerate() {
            let expected = u32::from_str_radix(expected.as_str().unwrap(), 16).unwrap();
            if kind == "auto_gears" {
                assert_eq!(
                    actual, expected,
                    "{kind} case {case_index}, packed pair {component}"
                );
            } else {
                let distance = (ordered(actual) - ordered(expected)).abs();
                assert!(distance <= 12, "{kind} case {case_index}, component {component}: {actual:08x} vs {expected:08x}, {distance} ULP");
            }
        }
    }
}
