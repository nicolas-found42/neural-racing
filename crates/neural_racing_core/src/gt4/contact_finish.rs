//! Original35de20 tail after road/contact velocity projection.
use super::{
    eefloat::{add, mul},
    math,
    native::NativeCar,
    parameters::NativeParameters,
};
/// Original35dd00/343518 distance and fractional-meter accumulator.
fn distance(car: &mut NativeCar) {
    if car.f32(0x6c8) < 0.0 {
        return;
    }
    let step = mul(car.f32(0x6c8), car.f32(0x54c));
    let value = add(car.f32(0x6d8), step);
    let fraction = if value > 0.0 {
        let whole = value as i32;
        let mileage = car.u32(0x6dc).wrapping_add(whole as u32);
        if mileage > 1_599_999_999 {
            car.set_u32(0x6dc, 1_600_000_000);
            0.0
        } else {
            car.set_u32(0x6dc, mileage);
            add(value, -(whole as f32))
        }
    } else {
        0.0
    };
    car.set_f32(0x6d8, fraction);
    let value = add(car.f32(0x6d4), step);
    car.set_f32(
        0x6d4,
        if value >= 80_000_000.0 {
            add(value, -80_000_000.0)
        } else {
            value
        },
    );
    if car.u8(0x56a) == 1 {
        let value = add(car.f32(0x7b0), step);
        car.set_f32(
            0x7b0,
            if value >= 80_000_000.0 {
                add(value, -80_000_000.0)
            } else {
                value
            },
        );
    }
}
/// Original35dc08 speed display and lateral-acceleration smoothing.
fn filters(car: &mut NativeCar) {
    let old = car.f32(0x6d0);
    let target = car.f32(0x6c8);
    let step = mul(car.f32(0x54c), f32::from_bits(0x42de38e2));
    let next = if old < target {
        add(old, step).min(target)
    } else {
        add(old, -step).max(target)
    };
    car.set_f32(0x6d0, next);
    let dt = f32::from_bits(0x3c888888);
    let acceleration = add(
        mul(add(car.f32(0x658), -car.f32(0x664)), 200.0),
        -mul(car.f32(0x660), 20.0),
    );
    let rate = add(car.f32(0x660), mul(acceleration, dt));
    car.set_f32(0x660, rate);
    car.set_f32(0x664, add(car.f32(0x664), mul(rate, dt)));
}
/// Original35d990 free-wheel rolling constraint and carrier refresh.
fn free_wheels(car: &mut NativeCar, p: &NativeParameters) {
    let mut free = [false; 2];
    for i in 0..4 {
        if car.wheel(i).u8(0x46) == 0 {
            let wheel = car.wheel(i);
            let speed = add(
                mul(math::cos(wheel.f32(0)), wheel.f32(0x28)),
                -mul(math::sin(wheel.f32(0)), wheel.f32(0x2c)),
            );
            car.wheel_mut(i).set_f32(0x20, speed);
            free[usize::from(p.model_u8(0x2c + i))] = true;
        }
    }
    for (group, is_free) in free.into_iter().enumerate() {
        let axle = usize::from(p.model_u8(0x10 + group));
        if is_free || p.u8(0x244 + axle) == 0 {
            let first = usize::from(p.model_u8(0xc + group));
            let pair = p.model_u8(0x24 + first);
            let pair = if pair == 0x7f {
                first
            } else {
                usize::from(pair)
            };
            let radius_axle = if p.u8(0x235) != 0 {
                usize::from(p.u8(0x237))
            } else {
                axle
            };
            // Assembly35dba4 averages first, then35dbb8 multiplies inverse radius.
            let average = mul(
                add(car.wheel(first).f32(0x20), car.wheel(pair).f32(0x20)),
                0.5,
            );
            car.set_f32(
                0x630 + group * 4,
                mul(p.f32(0x2cc + radius_axle * 4), average),
            );
        }
    }
}
/// Call immediately after35d920/35d6b0 projection, before344040 rim-angle update.
/// The selected reset sets the original engine-correction bypass6f0 to one.
pub(crate) fn finish(car: &mut NativeCar, p: &NativeParameters) -> Result<(), &'static str> {
    if car.u8(0x6f0) == 0 || p.u8(0x234) != 0 || p.model_u8(2) != 2 {
        return Err("unsupported native contact-tail correction or drive configuration");
    }
    car.set_u32(0x6c8, car.u32(0x640));
    distance(car);
    if car.f32(0x6c8) < 0.0 {
        car.set_f32(0x6c8, -car.f32(0x6c8));
    }
    if car.u8(0x5b6) == 0 && car.f32(0x6cc) < car.f32(0x6c8) {
        car.set_u32(0x6cc, car.u32(0x6c8));
    }
    filters(car);
    free_wheels(car, p);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::super::kinematics::{self, ContactContext, ContactGeometry, ContactVelocity};
    use super::*;
    use serde_json::Value;
    fn ordered(v: u32) -> i64 {
        i64::from(if v & 0x80000000 == 0 {
            v | 0x80000000
        } else {
            !v
        })
    }
    #[test]
    fn original_full_contact_projection_and_finish() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_contact_finish.json"
        ))
        .unwrap();
        assert_eq!(
            fixture["target_sha256"],
            "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
        );
        let p = NativeParameters::fixture_parameters();
        let cases = fixture["cases"].as_array().unwrap();
        assert_eq!(cases.len(), 24);
        for (index, c) in cases.iter().enumerate() {
            let mut car = NativeCar::default();
            for v in c["input_nonzero_words"].as_array().unwrap() {
                car.set_u32(
                    v[0].as_u64().unwrap() as usize,
                    v[1].as_u64().unwrap() as u32,
                );
            }
            let geometry = std::array::from_fn::<_, 4, _>(|i| {
                let shape = usize::from(p.model_u8(0x2c + i));
                ContactGeometry {
                    yaw_lateral_lever: -p.f32(0x10 + shape * 4)
                        * f32::from(p.model_i8(0x3c + i).signum()),
                    signed_half_track: p.f32(0x28 + shape * 4)
                        * if p.model_u8(0x1c + i) == 0 { -1.0 } else { 1.0 },
                    height: p.f32(0x40 + shape * 4),
                }
            });
            let mut wheels = [ContactVelocity::default(); 4];
            let (forward, lateral) = kinematics::update_contact_velocities(
                &mut wheels,
                &geometry,
                ContactContext {
                    world_velocity_at_648: std::array::from_fn(|i| car.f32(0x648 + i * 4)),
                    road_forward_at_140: std::array::from_fn(|i| car.f32(0x140 + i * 4)),
                    road_lateral_at_14c: std::array::from_fn(|i| car.f32(0x14c + i * 4)),
                    yaw_rate_at_680: car.f32(0x680),
                    roll_rate_at_688: car.f32(0x688),
                    body_up_z_at_130: car.f32(0x130),
                    roll_coupling_at_620784: p.global_u8(0x620784) != 0,
                },
            );
            car.set_f32(0x640, forward);
            car.set_f32(0x644, lateral);
            for (i, w) in wheels.iter().enumerate() {
                car.wheel_mut(i).set_f32(0x28, w.forward_at_28);
                car.wheel_mut(i).set_f32(0x2c, w.lateral_at_2c);
            }
            finish(&mut car, &p).unwrap();
            for (field, expected) in c["output_layout"]
                .as_array()
                .unwrap()
                .iter()
                .zip(c["output_bits"].as_array().unwrap())
            {
                let offset = field["offset"].as_u64().unwrap() as usize;
                let expected = u32::from_str_radix(expected.as_str().unwrap(), 16).unwrap();
                let actual = car.u32(offset);
                if field["kind"] == "bits" {
                    assert_eq!(actual, expected, "case{index} field{offset:x}");
                } else {
                    assert!(
                        (ordered(actual) - ordered(expected)).abs() <= 12,
                        "case{index} field{offset:x}: {actual:08x} vs {expected:08x}"
                    );
                }
            }
        }
    }
}
