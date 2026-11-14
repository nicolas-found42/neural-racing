//! Normal four-wheel manual-car automatic selection from 0x0036c160, with
//! thresholds from 0x0036bee0/0x0036bf28. The caller applies the returned request
//! through the separate original gear-change routine.

use super::{
    eefloat::{div, mul},
    native::NativeCar,
    parameters::NativeParameters,
};

/// Normal FR numeric branch of 0x0036bf68, used by the original rolling start.
pub(crate) fn gear_for_speed(speed: f32, p: &NativeParameters) -> u8 {
    if speed < 0.0 {
        return 0;
    }
    let maximum = p.u8(0x236);
    for gear in 1..maximum {
        let margin = if gear == 1 { 2000 } else { 500 };
        let rpm = i32::from(p.u16(0x262)) - margin;
        let threshold = div(
            mul(rpm as f32, f32::from_bits(0x3dd6_774e)),
            p.f32(0x29c + usize::from(gear) * 4),
        );
        if speed < threshold {
            return gear;
        }
    }
    maximum
}

/// Native `param_2=0` path. Training and the human use the same normal car mode;
/// the separate assisted/AI threshold modifier 0x003f1e30 is outside this path.
pub(crate) fn desired_gear(car: &mut NativeCar, p: &NativeParameters) -> i32 {
    assert_eq!(
        car.u8(0x56a),
        0,
        "automatic adapter requires normal manual car mode"
    );
    let gear = car.u8(0x624);
    if car.u16(0x5be) != 0 {
        return if gear == 1 { -1 } else { 1 };
    }
    assert_eq!(
        car.u8(0x546),
        0,
        "normal automatic selector requires native controller mode zero"
    );
    let maximum = p.u8(0x236);
    let remaining = car.i16(0x62e);
    if remaining != 0 {
        car.set_i16(0x62e, remaining.wrapping_sub(1));
    }
    let controller = p.global_u8(0x622118);
    let cooldown = car.i8(0x62a);
    if cooldown != 0 {
        let next = cooldown.wrapping_sub(1);
        car.set_i8(0x62a, next);
        if controller == 2 && next == 0 && car.i8(0x567) != 0 {
            car.set_u8(0x62a, 1);
        }
        return -1;
    }
    if car.i8(0x566) != 0 {
        return if gear != 0 { 0 } else { -1 };
    }
    let speed = car.f32(0x640);
    let threshold = f32::from_bits(0x3e8e_38e3);
    if speed > threshold && gear == 0 {
        return 1;
    }
    if car.f32(0x598) > 0.0 {
        if speed < threshold {
            return if gear == 1 { -1 } else { 1 };
        }
    } else if speed < -threshold && gear != 0 {
        return 0;
    }
    if gear != 0 && gear < maximum {
        let omega = mul(f32::from(p.u16(0x200)) + 500.0, f32::from_bits(0x3dd6_774e));
        if div(omega, p.f32(0x29c + usize::from(gear) * 4)) < speed {
            return i32::from(gear) + 1;
        }
    }
    match controller {
        1 if car.i8(0x567) != 0 => return -1,
        2 => {
            let pulse = car.i8(0x567);
            if pulse > 0 && gear < maximum {
                car.set_u8(0x62a, 1);
                car.set_u16(0x62e, 240);
                return i32::from(gear) + 1;
            }
            if pulse < 0 && gear > 1 {
                car.set_u8(0x62a, 1);
                car.set_u16(0x62e, 240);
                return i32::from(gear) - 1;
            }
            if car.i16(0x62e) != 0 {
                return -1;
            }
        }
        _ => {}
    }
    if car.i8(0x626) != 1 || gear == 0 || (gear == 1 && maximum == 1) {
        return -1;
    }
    let up = if gear < maximum {
        i32::from(p.u16(0x262))
    } else {
        20000
    };
    let mut down = -20000;
    if gear > 1 {
        down = i32::from(p.u16(0x260 + usize::from(gear) * 2));
        if car.f32(0x58c) < f32::from_bits(0x3d4c_cccc) {
            down += i32::from(p.global_u8(0x62211a)) * 100;
        }
        if car.f32(0x590) > f32::from_bits(0x3f4c_cccc) {
            down += i32::from(p.global_u8(0x62211b)) * 100;
        }
        if car.f32(0x58c) > 0.25 {
            down -= i32::from(p.global_u8(0x622119)) * 100;
        }
    }
    let rpm = i32::from(car.i16(0x5fe));
    if rpm < down {
        return i32::from(gear) - 1;
    }
    if rpm <= up {
        return -1;
    }
    // The selected FR car checks driven rear-wheel contact/slip before upshift.
    for i in 0..4 {
        if p.model_i8(0x3c + i) >= 0 {
            continue;
        }
        let wheel = car.wheel(i);
        if wheel.f32(0x48) == 0.0 {
            return -1;
        }
        let tire = p.tire_for_wheel(i, usize::from(wheel.u8(0x89)));
        if wheel.f32(0x38) < mul(tire.negative_peak_at_164, wheel.f32(0x3c)) {
            return -1;
        }
    }
    i32::from(gear) + 1
}

#[cfg(test)]
mod original_execution_tests {
    use super::*;
    use serde_json::Value;

    #[test]
    fn original_automatic_selector_matches_all_requests_and_cooldowns() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_automatic.json"
        ))
        .unwrap();
        assert_eq!(
            fixture["target_sha256"],
            "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
        );
        let cases = fixture["cases"].as_array().unwrap();
        assert_eq!(cases.len(), 102);
        let p = &NativeParameters::fixture_parameters();
        for (index, c) in cases.iter().enumerate() {
            let mut car = NativeCar::default();
            for (field, offset) in [("gear", 0x624), ("clutch", 0x626)] {
                car.set_u8(offset, c[field].as_u64().unwrap() as u8);
            }
            car.set_i16(0x5fe, c["rpm"].as_i64().unwrap() as i16);
            car.set_i8(0x62a, c["cooldown"].as_i64().unwrap() as i8);
            car.set_i16(0x62e, c["delay"].as_i64().unwrap() as i16);
            car.set_i8(0x567, c["pulse"].as_i64().unwrap() as i8);
            car.set_u8(0x566, u8::from(c["reverse"].as_bool().unwrap()));
            car.set_u16(0x5be, u16::from(c["held"].as_bool().unwrap()));
            for (field, offset) in [
                ("speed", 0x640),
                ("throttle", 0x598),
                ("throttle", 0x58c),
                ("brake", 0x590),
            ] {
                car.set_f32(offset, c[field].as_f64().unwrap() as f32);
            }
            for i in 0..4 {
                let mut wheel = car.wheel_mut(i);
                wheel.set_f32(
                    0x48,
                    if c["airborne"].as_bool().unwrap() && i == 3 {
                        0.0
                    } else {
                        3000.0
                    },
                );
                wheel.set_f32(0x38, c["slip"].as_f64().unwrap() as f32);
                wheel.set_f32(0x3c, 1.0);
            }
            let request = desired_gear(&mut car, p) as u32;
            let read = |v: &Value| u32::from_str_radix(v.as_str().unwrap(), 16).unwrap();
            assert_eq!(
                request,
                read(&c["expected_request_bits"]),
                "case {index} request"
            );
            assert_eq!(
                car.u32(0x62a),
                read(&c["output_bits"][1]),
                "case {index} cooldown"
            );
            assert_eq!(
                car.u32(0x62e),
                read(&c["output_bits"][2]),
                "case {index} delay"
            );
        }
    }
}
