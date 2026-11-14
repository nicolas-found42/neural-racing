//! Original standard-manual GT4 stability, traction and anti-lock control.
//! Selected four-wheel FR configuration; course/pit speed-limit policy belongs
//! to the caller, rather than a guessed replacement for the native conductor.
use super::{
    math,
    native::{NativeCar, WHEEL_BASE, WHEEL_STRIDE},
    parameters::NativeParameters,
    tire::Table,
};
const CFG: usize = 0x1098;
const DEG: f32 = f32::from_bits(0x3c8efa34);
const TENTH: f32 = f32::from_bits(0x3dcccccc);
fn w(i: usize, offset: usize) -> usize {
    WHEEL_BASE + i * WHEEL_STRIDE + offset
}

/// 0x3596c0, negative half of the original possibly nonmonotonic tire table.
pub fn inverse_negative(value: f32, table: &Table) -> f32 {
    if value <= table.y[0] {
        return table.x[0];
    }
    let mut i = 0;
    loop {
        if i >= table.x.len() - 1 {
            return table.x[i];
        }
        if table.x[i] >= 0.0 {
            return 0.0;
        }
        if value <= table.y[i + 1] {
            let fraction = (value - table.y[i]) / (table.y[i + 1] - table.y[i]);
            return table.x[i + 1] * fraction + table.x[i] * (1.0 - fraction);
        }
        i += 1;
    }
}
/// 0x359778, positive half of the same table.
pub fn inverse_positive(value: f32, table: &Table) -> f32 {
    let mut i = table.x.len() - 1;
    if value <= table.y[i] {
        return table.x[i];
    }
    loop {
        if i == 0 {
            return table.x[i];
        }
        if table.x[i] <= 0.0 {
            return 0.0;
        }
        if value <= table.y[i - 1] {
            let fraction = (value - table.y[i]) / (table.y[i - 1] - table.y[i]);
            return table.x[i - 1] * fraction + table.x[i] * (1.0 - fraction);
        }
        i -= 1;
    }
}
fn reset_traction_filter(car: &mut NativeCar) {
    car.set_f32(0x74c, 0.0);
    car.set_f32(0x748, car.f32(0x598));
}
/// Original 0x3f71e8, standard manual branch.
pub(crate) fn metrics(car: &mut NativeCar, p: &NativeParameters) {
    let mut under = 0.0;
    let mut over = 0.0;
    let forward = car.f32(0x640);
    if forward > 0.0 {
        let mut sum = 0.0;
        for i in 0..4 {
            sum += p.tire_for_wheel(i, usize::from(car.u8(w(i, 0x89)))).at_00;
        }
        let mut steer = car.f32(0x56c);
        let mut yaw = car.f32(0x680);
        let mut accel = car.f32(0x68c);
        if steer * yaw >= 0.0 {
            if steer < 0.0 {
                steer = -steer;
                yaw = -yaw;
                accel = -accel;
            }
            let expected = (forward * math::sin(steer)) / p.f32(0x18);
            if expected > 0.0 && car.f32(0x658).abs() / sum > f32::from_bits(0x3e4ccccc) {
                if accel * yaw > 0.0 {
                    yaw += accel * 0.5;
                }
                if yaw < expected {
                    under = 1.0 - yaw / expected;
                }
            }
        }
        if car.u8(0x55) != 0 && p.global_u8(0x620e0c) != 0 {
            under = (under * p.global_f32(0x6202a4)).min(1.0);
        }
        let mut largest = 0.0f32;
        for i in 0..4 {
            if p.model_i8(0x3c + i) < 0 {
                largest = largest.max(car.f32(w(i, 0x34)).abs());
            }
        }
        let lateral = car.f32(0x644);
        let blend = ((forward * forward + lateral * lateral).sqrt() * 0.018000001f32).min(1.0);
        let low = f32::from(p.global_u8(0x620db5)) * DEG;
        let high = f32::from(p.global_u8(0x620db6)) * DEG;
        let threshold = blend * (high - low) + low;
        // The native early exit retains the maximum rear slip angle itself;
        // it does not clear the metric below the speed-dependent threshold.
        over = largest;
        if largest >= threshold {
            let low = f32::from(p.global_u8(0x620db7)) * TENTH;
            let high = f32::from(p.global_u8(0x620db8)) * TENTH;
            over = ((blend * (high - low) + low) * (largest - threshold))
                / (f32::from_bits(0x3fc90fd9) - threshold);
            if car.u8(0x55) != 0 && p.global_u8(0x620e0d) != 0 {
                over *= p.global_f32(0x6202a8);
            }
            if over > 1.0 {
                over = 1.0;
            }
        }
    }
    let dt = car.f32(0x54c);
    for (flag, pos, rate, target) in [
        (0x620e0e, 0x574, 0x578, &mut under),
        (0x620e0f, 0x57c, 0x580, &mut over),
    ] {
        if p.global_u8(flag) != 0 {
            let velocity =
                car.f32(rate) + ((*target - car.f32(pos)) * 500.0 - car.f32(rate) * 20.0) * dt;
            car.set_f32(rate, velocity);
            *target = car.f32(pos) + velocity * dt;
        }
    }
    car.set_f32(0x574, under);
    car.set_f32(0x57c, over);
}
/// Original 0x3f9d00; standard rear drive and human-controller branch.
pub(crate) fn traction(car: &mut NativeCar, p: &NativeParameters) {
    let enabled = p.f32(CFG + 0x20) != 0.0;
    car.set_u8(0x6fd, u8::from(enabled));
    if !enabled {
        reset_traction_filter(car);
        return;
    }
    if car.f32(0x5ec) <= 0.0 {
        return;
    }
    if car.u8(0x626) == 0 {
        reset_traction_filter(car);
        return;
    }
    if car.f32(0x598) <= 0.0 {
        return;
    }
    let mut slip = 1.0;
    let mut peak = 1.0;
    let mut gain = 1.0;
    for i in 0..4 {
        if p.model_i8(0x3c + i) < 0 && car.f32(w(i, 0x48)) > 0.0 && car.f32(w(i, 0x38)) < slip {
            let tire = p.tire_for_wheel(i, usize::from(car.u8(w(i, 0x89))));
            gain = car.f32(w(i, 0x3c));
            peak = tire.negative_peak_at_164;
            if p.global_u8(0x620def) == 1 {
                let inverse = inverse_negative(car.f32(w(i, 0x5c)), &tire.slip_mu_b);
                if peak < inverse {
                    peak = inverse;
                }
            }
            slip = car.f32(w(i, 0x38));
        }
    }
    if p.global_u8(0x620def) == 0 {
        let metric = car.f32(0x690).abs();
        let factor = if metric < p.f32(CFG + 0x28) {
            1.0
        } else {
            (1.0 - (metric - p.f32(CFG + 0x28)) * p.f32(CFG + 0x24)).max(0.0)
        };
        peak *= factor;
    }
    peak *= p.f32(CFG + 0x2c);
    let speed = car.f32(0x640);
    if p.global_u8(0x620e1e) != 0 {
        let low = f32::from(p.global_u8(0x620e1f)) * f32::from_bits(0x4031c71b);
        if low < speed {
            let mut factor = f32::from(p.global_u8(0x620e21)) * 0.01 + 1.0;
            let high = f32::from(p.global_u8(0x620e20)) * f32::from_bits(0x4031c71b);
            if speed < high {
                factor *= (speed - low) / (high - low);
            }
            peak *= factor;
        }
    }
    let reduction = ((slip - peak) * p.f32(CFG + 0x20) * gain + 1.0).clamp(0.0, 1.0);
    if speed < f32::from_bits(0x41de38e2) {
        if reduction < 1.0 {
            car.set_u8(0x6fd, 2);
        }
        car.set_f32(0x598, car.f32(0x598) * reduction);
        reset_traction_filter(car);
        return;
    }
    let dt = f32::from_bits(0x3c888888);
    let mut rate = car.f32(0x74c)
        + ((car.f32(0x598) * reduction * f32::from_bits(0x3f8ccccc) - car.f32(0x748)) * 500.0
            - car.f32(0x74c) * 50.0)
            * dt;
    let mut value = car.f32(0x748) + rate * dt;
    if car.f32(0x598) < value {
        value = car.f32(0x598);
        rate = 0.0;
    } else if value < 0.0 {
        value = 0.0;
        rate = 0.0;
    }
    car.set_f32(0x74c, rate);
    car.set_f32(0x748, value);
    if value < car.f32(0x598) {
        car.set_u8(0x6fd, 2);
        car.set_f32(0x598, value);
    }
}
/// 0x3f94c0 ASC scratch-space updates; flags retain original byte layout.
pub(crate) fn stability(car: &mut NativeCar, p: &NativeParameters, s: &mut [f32; 16], allow: bool) {
    for i in 0..2 {
        s[2 + i] *= p.f32(0x102c + 4 * i);
    }
    if !allow {
        return;
    }
    let under = car.f32(0x574);
    let side = usize::from(car.f32(0x680) <= 0.0);
    let other = 1 - side;
    let mut active = 0;
    if car.f32(0x590) == 0.0 {
        let term = (car.f32(0x57c) - p.f32(CFG + 4)) * p.f32(CFG);
        if p.u8(CFG + 0x30) != 0 && term > 0.0 {
            active = 1;
            if p.u8(CFG + 0xc) != 0 {
                s[12 + other] += term;
            }
            if p.u8(CFG + 0xd) != 0 {
                s[14 + other] += term;
            }
        }
        if p.u8(CFG + 0x32) != 0 && (p.global_u8(0x6224a7) != 0 || car.f32(0x58c) == 0.0) {
            let term = (under - p.f32(CFG + 0x14)) * p.f32(CFG + 0x10);
            if term > 0.0 {
                if p.global_u8(0x6224a6) != 0 {
                    active |= 2;
                }
                if p.u8(CFG + 0x1c) != 0 {
                    s[12 + side] += term;
                }
                if p.u8(CFG + 0x1d) != 0 {
                    s[14 + side] += term;
                }
            }
        }
    } else {
        let factor = (1.0 - (car.f32(0x57c) - TENTH) * p.f32(CFG + 8)).max(0.0);
        if p.u8(CFG + 0x31) != 0 && factor < 1.0 {
            active = 1;
            if p.u8(CFG + 0xe) != 0 {
                s[4 + side] *= factor;
                s[8 + side] *= factor;
            }
            if p.u8(CFG + 0xf) != 0 {
                s[6 + side] *= factor;
                s[10 + side] *= factor;
            }
        }
        let factor = (1.0 - (under - TENTH) * p.f32(CFG + 0x18)).max(0.0);
        if p.u8(CFG + 0x33) != 0 && factor < 1.0 {
            if p.global_u8(0x6224a6) != 0 && p.global_u8(0x6224a8) == 0 {
                active |= 2;
            }
            if p.u8(CFG + 0x1e) != 0 {
                s[4 + other] *= factor;
                s[8 + other] *= factor;
            }
            if p.u8(CFG + 0x1f) != 0 {
                s[6 + other] *= factor;
                s[10 + other] *= factor;
            }
        }
    }
    if active & 2 != 0 && p.global_u8(0x620e10) != 0 {
        car.set_f32(
            0x598,
            car.f32(0x598) * (1.0 - under * p.global_f32(0x6202a0)).max(0.0),
        );
    }
    if active != 0 {
        car.set_u8(0x6fc, 2);
    } else if p.u16(CFG + 0x30) != 0
        || (p.global_u8(0x6224a6) != 0
            && (p.u8(CFG + 0x32) != 0 || (p.global_u8(0x6224a8) == 0 && p.u8(CFG + 0x33) != 0)))
    {
        car.set_u8(0x6fc, 1);
    }
}
/// Original selected four-wheel 0x3f9870, including dirt/snow brake blending.
pub(crate) fn stability_and_abs(car: &mut NativeCar, p: &NativeParameters) {
    let mut scratch = [1.0; 16];
    scratch[12..].fill(0.0);
    let mut peaks = [0.0; 4];
    for (i, peak) in peaks.iter_mut().enumerate() {
        let tire = p.tire_for_wheel(i, usize::from(car.u8(w(i, 0x89))));
        *peak = tire.positive_peak_at_168;
        let axle = usize::from(p.model_u8(0x14 + i));
        let percentage = p.global_u8(0x620ded + axle as u32);
        let slot = p.tire_slot(i, usize::from(car.u8(w(i, 0x89))));
        if percentage != 0 && p.u8(0x410 + slot * 0x1ec + 0x1d8) == 1 {
            let inverse = inverse_positive(car.f32(w(i, 0x5c)), &tire.slip_mu_b);
            if inverse < *peak {
                let fraction = f32::from(percentage) / 100.0;
                *peak = fraction * inverse + (1.0 - fraction) * *peak;
            }
        }
    }
    if p.course_kind() != 0 && p.global_u8(0x620dbd) != 0 {
        use super::eefloat::{add, div, mul};
        let threshold = mul(f32::from(p.global_u8(0x620dbd)), DEG);
        let angle = car.f32(0x56c).abs();
        let fraction = if threshold <= angle {
            1.0
        } else {
            div(angle, threshold)
        };
        for (i, factor) in scratch[..2].iter_mut().enumerate() {
            *factor = mul(
                *factor,
                add(
                    1.0,
                    -mul(
                        mul(fraction, f32::from(p.global_u8(0x620dbe + i as u32))),
                        0.01,
                    ),
                ),
            );
        }
    }
    stability(car, p, &mut scratch, car.f32(0x594) < TENTH);
    for (i, peak) in peaks.iter_mut().enumerate() {
        let axle = usize::from(p.model_u8(0x14 + i));
        let side = usize::from(p.model_u8(0x1c + i));
        let mut brake = car.f32(w(i, 0x64));
        if axle < 2 {
            brake *= scratch[axle];
            *peak *= scratch[2 + axle];
            if side < 2 {
                let slot = side + axle * 2;
                *peak *= scratch[4 + slot];
                brake *= scratch[8 + slot];
                brake += scratch[12 + slot];
                if brake > 1.0 {
                    brake = 1.0;
                }
            }
        }
        if brake != 0.0 {
            let factor = (car.f32(w(i, 0x4c))
                * p.f32(0x1034 + axle * 4)
                * (1.0
                    - (car.f32(w(i, 0x38)) - *peak)
                        * p.f32(0x1024 + axle * 4)
                        * car.f32(w(i, 0x3c))))
            .clamp(0.0, 1.0);
            brake *= factor;
            if brake == 0.0 {
                brake = f32::from_bits(0x3a83126e);
            }
        }
        car.set_f32(w(i, 0x64), brake);
    }
}
/// Manual standard-road subset of 0x3f79a0. Explicitly rejects native AI and
/// pit/course limiter contexts, which require their own original conductor.
pub(crate) fn update_standard(
    car: &mut NativeCar,
    p: &NativeParameters,
) -> Result<(), &'static str> {
    if car.u8(0x56a) != 0 || car.u8(0x56b) != 0 || car.u8(0x5e9) & 2 != 0 || car.u8(0x6c7) != 0 {
        return Err("unsupported native aid/controller context");
    }
    metrics(car, p);
    car.set_u8(0x6fd, 0);
    car.set_u8(0x6fc, 0);
    traction(car, p);
    stability_and_abs(car, p);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    fn ordered(bits: u32) -> i64 {
        i64::from(if bits & 0x8000_0000 == 0 {
            bits | 0x8000_0000
        } else {
            !bits
        })
    }

    #[test]
    fn original_course_condition_braking_matches_native_execution() {
        let f: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_course_condition_aids.json"
        ))
        .unwrap();
        let ordered = |x: u32| {
            if x & 0x80000000 != 0 {
                0x80000000i64 - i64::from(x & 0x7fffffff)
            } else {
                0x80000000i64 + i64::from(x)
            }
        };
        for c in f["cases"].as_array().unwrap() {
            let hex = c["input_car_hex"].as_str().unwrap();
            let mut car = NativeCar::from_bytes(std::array::from_fn(|i| {
                u8::from_str_radix(&hex[i * 2..i * 2 + 2], 16).unwrap()
            }));
            let p =
                NativeParameters::m3_for_condition(c["condition"].as_u64().unwrap() as u8).unwrap();
            stability_and_abs(&mut car, p);
            for (o, b) in f["layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(c["expected_bits"].as_array().unwrap())
            {
                let off = o.as_u64().unwrap() as usize;
                let expected = u32::from_str_radix(b.as_str().unwrap(), 16).unwrap();
                let actual = car.u32(off);
                if off == 0x6fc {
                    assert_eq!(actual, expected);
                } else {
                    assert!(
                        (ordered(actual) - ordered(expected)).abs() <= 12,
                        "condition {} offset {off:x}",
                        c["condition"]
                    );
                }
            }
        }
    }

    #[test]
    fn original_m3_aid_routines() {
        let fixture: Value =
            serde_json::from_str(include_str!("../../tests/fixtures/gt4_original_aids.json"))
                .unwrap();
        assert_eq!(
            fixture["target_sha256"],
            "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
        );
        let parameters = NativeParameters::fixture_parameters();
        let params = &parameters;
        let cases = fixture["cases"].as_array().unwrap();
        assert_eq!(cases.len(), 72);
        for (index, case) in cases.iter().enumerate() {
            let mut car = NativeCar::default();
            for word in case["input_nonzero_words"].as_array().unwrap() {
                car.set_u32(
                    word[0].as_u64().unwrap() as usize,
                    word[1].as_u64().unwrap() as u32,
                );
            }
            match case["kind"].as_str().unwrap() {
                "aid_metrics" => metrics(&mut car, params),
                "aid_traction" => traction(&mut car, params),
                "aid_braking" => stability_and_abs(&mut car, params),
                _ => panic!("unexpected aid fixture"),
            }
            for (field, expected) in case["output_layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(case["output_bits"].as_array().unwrap())
            {
                let offset = field["offset"].as_u64().unwrap() as usize;
                let expected = u32::from_str_radix(expected.as_str().unwrap(), 16).unwrap();
                let actual = car.u32(offset);
                if field["kind"] == "bits" {
                    assert_eq!(actual, expected, "case {index} field {offset:#x}");
                } else {
                    assert!(
                        (ordered(actual) - ordered(expected)).abs() <= 12,
                        "case {index} field {offset:#x}: {actual:08x} vs {expected:08x}"
                    );
                }
            }
        }
    }
}
