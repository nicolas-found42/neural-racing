//! Original normal-car 0x357700 preparation, before input and force conductor.
use super::{
    eefloat::{add, div, mul},
    engine, math,
    native::NativeCar,
    parameters::NativeParameters,
    tire,
};

/// Supported source path: normal human mode0, car5be==0, selected M3 aero
/// extension flag1077==0 and wear/temperature controller disabled. Optional
/// statistics affect the later force conductor, not this native preparation.
pub(crate) fn prepare(
    car: &mut NativeCar,
    p: &NativeParameters,
    wear_enabled: bool,
    _statistics_enabled: bool,
) -> Result<(), &'static str> {
    if car.i16(0x5be) != 0
        || car.u8(0x56a) != 0
        || car.u8(0x56b) != 0
        || p.u8(0x1077) != 0
        || wear_enabled
    {
        return Err("unsupported native preparation control/wear/aero-extension state");
    }
    let timer = car.u8(0x6ff);
    if timer != 0 && timer != 255 {
        car.set_u8(0x6ff, timer.wrapping_sub(1));
    }
    car.set_f32(0x54c, mul(car.f32(0x548), f32::from_bits(0x3c888888)));
    car.set_f32(0x550, div(60.0, car.f32(0x548)));
    let fuel_mass = mul(car.f32(0x5d4), f32::from_bits(0x3f3d70a3));
    car.set_f32(0x5d8, add(p.f32(0x48), fuel_mass));
    car.set_f32(0x5dc, add(p.f32(0x4c), fuel_mass));
    aero(car, p);
    let correction = engine::CorrectionState {
        input_at_04: car.f32(0x6e0),
        input_at_08: car.f32(0x6e4),
        bypass: car.u8(0x6f0) != 0,
    };
    let settings = engine::EngineCorrection::from_tuning_bytes(engine::ORIGINAL_CORRECTION_BYTES);
    car.set_f32(
        0x5ec,
        engine::corrected_curve_value(p.curve(0x134), car.f32(0x600), Some(&correction), &settings),
    );
    let terminal = p.f32(0x134 + 8 + usize::from(p.u16(0x134)) * 4);
    car.set_f32(
        0x5f0,
        engine::drag_torque(
            car.f32(0x600),
            p.f32(0x204),
            p.global_u8(0x620e14) != 0,
            terminal,
        ),
    );
    let gravity = mul(mul(car.f32(0x148), p.f32(0x48)), f32::from_bits(0x411ccccc));
    car.set_f32(0x6c0, -add(add(car.f32(0x6b4), car.f32(0x6a8)), -gravity));
    let roll = car.f32(0x670);
    let base_roll = car.f32(0x67c);
    for i in 0..4 {
        let w = car.wheel(i);
        let slot = p.tire_slot(i, usize::from(w.u8(0x89)));
        let tables = p.tire(slot);
        let axle = p.wheel_groups()[i];
        let mut load = tire::LoadScaleState {
            camber_at_04: w.f32(4),
            normal_at_48: w.f32(0x48),
            scale_at_4c: w.f32(0x4c),
            wear_at_50: w.f32(0x50),
            terrain_at_89: w.u8(0x89),
            flag_at_dd: w.u8(0xdd),
        };
        tire::update_load_scale(
            &mut load,
            tables,
            p.curve(0x3b8 + axle * 12),
            &tire::LoadScaleContext {
                width_at_2d4: p.f32(0x2d4 + axle * 4),
                axle_multiplier_at_fd0: p.f32(0xfd0 + axle * 4),
                vehicle_mode: p.model_u8(0),
                wheel_side: p.model_u8(0x1c + i),
                body_angle_at_670: roll,
                body_angle_at_67c: base_roll,
                tire_mode_at_1da: p.u8(0x410 + slot * 0x1ec + 0x1da),
                apply_wear_factor: false,
                control_grip_factor: 1.0,
            },
        );
        let mut w = car.wheel_mut(i);
        w.set_f32(0x4c, load.scale_at_4c);
        w.set_u8(0xdd, load.flag_at_dd);
    }
    for i in 0..4 {
        let w = car.wheel(i);
        let (heading, fade) = tire::contact_velocity_heading(w.f32(0x28), w.f32(0x2c));
        let mut w = car.wheel_mut(i);
        w.set_f32(0x30, heading);
        w.set_f32(0x60, fade);
    }
    Ok(())
}

/// 0x3f3090; the selected initialized M3 has inactive 0x3f2db8 extension.
fn aero(car: &mut NativeCar, p: &NativeParameters) {
    let square = mul(car.f32(0x640), car.f32(0x640));
    let dt = car.f32(0x54c);
    let draft = if car.f32(0x6a0) == 0.0 {
        let next = add(car.f32(0x6a4), -mul(p.global_f32(0x622318), dt));
        if next < 0.0 {
            0.0
        } else {
            next
        }
    } else {
        let next = add(
            car.f32(0x6a4),
            mul(mul(car.f32(0x6a0), p.global_f32(0x622314)), dt),
        );
        if next > 1.0 {
            1.0
        } else {
            next
        }
    };
    car.set_f32(0x6a4, draft);
    let vertical = add(
        add(
            mul(car.f32(0x648), car.f32(0x128)),
            mul(car.f32(0x64c), car.f32(0x12c)),
        ),
        mul(car.f32(0x650), car.f32(0x130)),
    );
    let angle = math::atan2(vertical, car.f32(0x640)).abs();
    let remaining = add(1.0, -draft);
    let mut drag_scale = add(
        p.global_f32(0x62230c),
        mul(add(1.0, -p.global_f32(0x62230c)), remaining),
    );
    if car.u8(0x6c7) == 2 {
        drag_scale = 20.0;
    }
    let drag = mul(mul(p.f32(0x1068), drag_scale), square);
    car.set_f32(0x6a8, if car.f32(0x640) >= 0.0 { -drag } else { drag });
    let mut down_scale = add(
        p.global_f32(0x622310),
        mul(add(1.0, -p.global_f32(0x622310)), remaining),
    );
    let high = f32::from_bits(0x3e32b8c1);
    let low = f32::from_bits(0x3db2b8c1);
    if angle > high {
        down_scale = 0.0;
    } else if angle > low {
        down_scale = mul(down_scale, div(add(high, -angle), low));
    }
    for i in 0..2 {
        car.set_f32(
            0x6ac + i * 4,
            mul(mul(p.f32(0x106c + i * 4), down_scale), square),
        );
    }
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
    fn complete_frame_preparation_matches_original_357700() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_preparation.json"
        ))
        .unwrap();
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 36);
        let params = &NativeParameters::fixture_parameters();
        for (index, c) in fixture["cases"].as_array().unwrap().iter().enumerate() {
            let hex = c["input_car_hex"].as_str().unwrap();
            let mut bytes = [0u8; super::super::native::CAR_BYTES];
            for (i, byte) in bytes.iter_mut().enumerate() {
                *byte = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap();
            }
            let mut car = NativeCar::from_bytes(bytes);
            prepare(&mut car, params, false, false).unwrap();
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
                        ulp <= 12,
                        "case {index} {}: {ulp} ULP ({actual:08x}/{expected:08x})",
                        layout["label"]
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod composition_tests {
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
    fn prepared_force_pipeline_matches_original_composition() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_combined.json"
        ))
        .unwrap();
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 48);
        let params = &NativeParameters::fixture_parameters();
        for (index, c) in fixture["cases"].as_array().unwrap().iter().enumerate() {
            let hex = c["input_car_hex"].as_str().unwrap();
            let mut bytes = [0u8; super::super::native::CAR_BYTES];
            for (i, byte) in bytes.iter_mut().enumerate() {
                *byte = u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap();
            }
            let mut car = NativeCar::from_bytes(bytes);
            prepare(&mut car, params, false, false).unwrap();
            super::super::conductor::ForceConductor::default()
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
                                "prepared_conductor",
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
