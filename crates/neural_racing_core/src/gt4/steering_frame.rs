//! Native normal four-wheel steering map, secondary torsion and compliance.
use super::{
    eefloat::{add, div, mul},
    math,
    native::{NativeCar, WHEEL_BASE, WHEEL_STRIDE},
    parameters::NativeParameters,
    steering,
};
fn w(i: usize, o: usize) -> usize {
    WHEEL_BASE + i * WHEEL_STRIDE + o
}
/// 0x348800 physical steering-angle normalization for the native output packet.
pub(crate) fn normalized(angle: f32) -> f32 {
    mul(angle, f32::from_bits(0x40747648)).clamp(-1.0, 1.0)
}
/// Original 0x3f7ea8 selected initialized mode-one branch, including 0x40cc58.
pub(crate) fn map_wheels(car: &mut NativeCar, p: &NativeParameters) -> Result<(), &'static str> {
    if p.angle_mode() != 1 {
        return Err("unsupported native secondary steering mode");
    }
    for i in 0..4 {
        let axle = usize::from(p.model_u8(0x14 + i));
        let input = car.f32(0x56c);
        let sign = p.model_i8(0x3c + i);
        let side = p.model_u8(0x1c + i);
        let index = if input < 0.0 && side < 2 {
            1 - side
        } else {
            side
        };
        let rear = sign < 0 && p.u8(0x10d0) == 1;
        let mut angle = if sign > 0 {
            input
        } else if rear {
            mul(-input, p.f32(0x10cc))
        } else {
            0.0
        };
        if (sign > 0 || rear) && index < 2 {
            angle = mul(angle, p.f32(0x10c + usize::from(index) * 4));
        }
        if side == 0 {
            angle = add(angle, -p.f32(0x1008 + axle * 4));
        } else if side == 1 {
            angle = add(angle, p.f32(0x1008 + axle * 4));
        }
        car.set_f32(w(i, 0), angle);
    }
    let dt = f32::from_bits(0x3c888888);
    let acceleration = add(
        mul(add(mul(car.f32(0x56c), 11.25), -car.f32(0x768)), 200.0),
        -mul(car.f32(0x76c), 20.0),
    );
    car.set_f32(0x770, acceleration);
    let rate = add(car.f32(0x76c), mul(acceleration, dt));
    car.set_f32(0x76c, rate);
    car.set_f32(0x768, add(car.f32(0x768), mul(rate, dt)));
    for i in 0..4 {
        let shape = u32::from(p.model_u8(0x2c + i));
        let old = car.f32(w(i, 0x1c));
        let mut limit = p.steering_global_f32(0x845bf8 + shape * 4);
        let target = if p.global_u8(0x620288 + u32::from(car.u8(w(i, 0x89)))) == 1 {
            0.0
        } else if car.f32(w(i, 0x60)) == 0.0 {
            add(
                old,
                -mul(add(car.f32(w(i, 0)), -car.f32(0x8e0 + i * 4)), 0.5),
            )
            .clamp(-limit, limit)
        } else if car.f32(w(i, 0x28)) >= 0.0 {
            mul(add(car.f32(w(i, 0x30)), -car.f32(w(i, 0))), 0.5)
        } else {
            math::angle_difference(
                add(car.f32(w(i, 0x30)), -f32::from_bits(0x40490fda)),
                car.f32(w(i, 0)),
            ) * 0.5
        };
        let mut acceleration = add(
            mul(
                p.steering_global_f32(0x845c00 + shape * 4),
                add(target, -old),
            ),
            -mul(
                p.steering_global_f32(0x845c08 + shape * 4),
                car.f32(w(i, 0x18)),
            ),
        );
        let mode = p.steering_global_u32(0x845bf4);
        let factor = if mode == 0 {
            1.0
        } else {
            div(
                mul(
                    car.f32(w(i, 0x4c)),
                    f32::from(p.model_u8(shape as usize + 8)),
                ),
                p.f32(0x1040 + shape as usize * 4),
            )
        };
        if mode == 2 || mode == 3 {
            acceleration = mul(acceleration, factor);
        }
        if mode == 1 || mode == 3 {
            limit = mul(limit, factor);
        }
        let mut rate = add(car.f32(w(i, 0x18)), mul(acceleration, car.f32(0x54c)));
        let mut angle = add(old, mul(rate, car.f32(0x54c)));
        if angle > limit {
            angle = limit;
            rate = 0.0;
        } else if angle < -limit {
            angle = -limit;
            rate = 0.0;
        }
        car.set_f32(w(i, 0x18), rate);
        car.set_f32(w(i, 0x1c), angle);
    }
    if p.global_u8(0x620af0) != 0 {
        for i in 0..4 {
            let shape = usize::from(p.model_u8(0x2c + i));
            let stiffness = add(
                p.f32(0x1050 + shape * 4),
                p.global_curve(0x620b24).sample_ee(car.f32(0x6ec)),
            );
            let mut force = mul(car.f32(w(i, 0x68)), p.f32(0x1058 + shape * 4));
            let deadband = p.global_curve(0x620b40).sample_ee(stiffness);
            if force >= 0.0 {
                force = add(force, -deadband);
                if force < 0.0 {
                    force = 0.0;
                }
            } else {
                force = add(force, deadband);
                if force > 0.0 {
                    force = 0.0;
                }
            }
            let mut target = 0.0;
            if force != 0.0 {
                target = mul(force, p.global_curve(0x620b54).sample_ee(stiffness));
                let limit = p.global_f32(0x620b04);
                if target > limit {
                    target = limit;
                } else if target < -limit {
                    target = -limit;
                }
                if shape == 1 {
                    target = -target;
                }
            }
            let old = car.f32(w(i, 0xcc));
            let rate = add(
                car.f32(w(i, 0xd0)),
                mul(
                    add(
                        mul(p.global_f32(0x620b08), add(target, -old)),
                        -mul(p.global_f32(0x620b0c), car.f32(w(i, 0xd0))),
                    ),
                    car.f32(0x54c),
                ),
            );
            car.set_f32(w(i, 0xd0), rate);
            let angle = add(old, mul(rate, car.f32(0x54c)));
            car.set_f32(w(i, 0xcc), angle);
            car.set_f32(w(i, 0), add(car.f32(w(i, 0)), angle));
        }
    }
    Ok(())
}

fn response_ee(
    state: &mut steering::SteeringState,
    target: f32,
    heading: f32,
    speed: f32,
    dt: f32,
    group: usize,
    p: &NativeParameters,
) {
    let bits = |a: usize| p.response_global_u32(a as u32);
    let f = |a| p.response_global_f32(a as u32);
    let offset = group * 4;
    let current = state.angle;
    let mut acceleration = add(
        mul(add(target, -current), f(0x843598 + offset)),
        -mul(state.rate, f(0x8435b0 + offset)),
    );
    let branch = if f(0x8435a8 + offset) != 0.0
        && ((current < target && current < heading) || (target < current && heading < current))
    {
        acceleration = add(
            acceleration,
            mul(add(heading, -current), f(0x8435a8 + offset)),
        );
        2
    } else if (current > 0.0 && target < current) || (current < 0.0 && current < target) {
        acceleration = add(acceleration, -mul(current, f(0x8435a0 + offset)));
        1
    } else {
        0
    };
    let slow = f([0x8435b8, 0x8435c8, 0x8435d8][branch] + group * 8 + 4);
    let fast = f([0x843600, 0x843610, 0x843620][branch] + group * 8 + 4);
    let mut limit = slow;
    if bits(0x8435e8 + offset) != 0 && f(0x8435f0 + offset) < speed {
        limit = fast;
        if speed < f(0x8435f8 + offset) {
            limit = add(
                slow,
                div(
                    mul(add(fast, -slow), add(speed, -f(0x8435f0 + offset))),
                    add(f(0x8435f8 + offset), -f(0x8435f0 + offset)),
                ),
            );
        }
    }
    let rate = add(state.rate, mul(acceleration, dt)).clamp(-limit, limit);
    let next = add(current, mul(rate, dt));
    if (current <= target && target <= next) || (target <= current && next <= target) {
        state.angle = target;
        state.rate = 0.0;
    } else {
        state.angle = next;
        state.rate = rate;
    }
}

/// Native type-two analog packet fields (0x349418), all channels in 12-bit units.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct AnalogInput {
    pub steering: i16,
    pub throttle: u16,
    pub brake: u16,
    pub reverse: u16,
    pub handbrake: i16,
    pub shift: u8,
}
fn approach(current: f32, target: f32, up: f32, down: f32) -> f32 {
    if current < target {
        add(current, up).min(target)
    } else if current > target {
        add(current, -down).max(target)
    } else {
        current
    }
}
/// 0x349f48 ->0x349418/0x348850 for the selected type-two manual packet.
/// Original track-centering must be explicitly disabled with native5b5 bit0;
/// its original-course lookup cannot be substituted with a different track.
pub(crate) fn input_analog(
    car: &mut NativeCar,
    p: &NativeParameters,
    input: AnalogInput,
) -> Result<(), &'static str> {
    if !(-4096..=4096).contains(&input.steering)
        || input.throttle > 4096
        || input.brake > 4096
        || input.reverse > 4096
        || !(0..=4096).contains(&input.handbrake)
    {
        return Err("native analog channel outside 12-bit range");
    }
    if car.u8(0x56a) != 0
        || car.u8(0x56b) != 0
        || car.u8(0x546) > 2
        || car.u8(0x5b5) & 1 == 0
        || car.u8(0x5e9) & 0x18 != 0
        || car.u8(0x568) != 0
        || car.u8(0x569) == 1
    {
        return Err("unsupported native analog input context");
    }
    car.set_u16(0x55c, car.u16(0x558));
    car.set_u16(0x55e, car.u16(0x55a));
    let mut heading = 0.0;
    let mut count = 0;
    for i in 0..4 {
        if p.model_i8(0x3c + i) > 0 {
            heading = add(heading, car.f32(w(i, 0x30)));
            count += 1;
        }
    }
    heading = div(heading, count as f32);
    let alternate = (0..4)
        .filter(|i| p.model_i8(0x3c + *i) > 0)
        .map(|i| p.global_u8(0x620288 + u32::from(car.u8(w(i, 0x89)))))
        .min()
        .unwrap_or(0)
        == 1;
    let cap = p
        .curve(if alternate { 0xd0 } else { 0x58 })
        .sample_linear_ee(car.f32(0x640).max(0.0));
    let upper = cap.max(heading).min(p.f32(0x54));
    let lower = (-cap).min(heading).max(-p.f32(0x54));
    car.set_f32(0x5a0, upper);
    car.set_f32(0x59c, lower);
    let axis = mul(f32::from(input.steering), f32::from_bits(0x39800000));
    let target = if axis < 0.0 {
        mul(-lower, axis)
    } else {
        mul(upper, axis)
    };
    let mut state = steering::SteeringState {
        angle: car.f32(0x56c),
        rate: car.f32(0x588),
    };
    response_ee(
        &mut state,
        target,
        heading,
        car.f32(0x640),
        car.f32(0x54c),
        usize::from(alternate),
        p,
    );
    car.set_f32(0x56c, state.angle);
    car.set_f32(0x588, state.rate);
    if input.steering != 0 {
        car.set_u8(0x569, 2);
    }
    car.set_f32(0x5a4, 0.0);
    if car.u8(0x569) == 1 || car.u8(0x569) == 2 {
        car.set_f32(0x560, normalized(state.angle));
    }
    let reverse = if car.i16(0x5be) != 0 || car.f32(0x640) > f32::from_bits(0x40b1c71b) {
        0
    } else {
        input.reverse
    };
    let (gas, reverse) = if reverse < input.throttle {
        (input.throttle, 0)
    } else {
        (reverse, reverse)
    };
    car.set_u8(0x566, u8::from(reverse != 0));
    let bias = p.global_f32(0x620268);
    let channel = |raw: u16| {
        let value = mul(f32::from(raw), f32::from_bits(0x39800000));
        if value != 0.0 {
            add(value, mul(bias, add(1.0, -value)))
        } else {
            value
        }
    };
    let target = channel(gas);
    let throttle = if p.global_u8(0x622121) != 0 && gas != 0 {
        approach(
            car.f32(0x58c),
            target,
            p.global_f32(0x620270),
            p.global_f32(0x620274),
        )
    } else {
        target
    };
    car.set_f32(0x58c, throttle);
    car.set_u8(0x568, 0);
    let target = channel(input.brake);
    let brake = if p.global_u8(0x622121) != 0 {
        approach(
            car.f32(0x590),
            target,
            p.global_f32(0x620278),
            p.global_f32(0x62027c),
        )
    } else {
        target
    };
    car.set_f32(0x590, brake);
    car.set_u8(0x567, if car.i16(0x5be) == 0 { input.shift } else { 0 });
    car.set_f32(0x594, channel(input.handbrake as u16));
    let clutch = if car.u8(0x5c0) != 0
        || car.i16(0x5be) != 0
        || (p.u8(0x237) == 1 && car.f32(0x594) > f32::from_bits(0x3f4ccccc))
    {
        0.0
    } else if car.u8(0x626) == 0 {
        if brake >= 0.25 {
            0.0
        } else {
            1.0
        }
    } else if i32::from(car.i16(0x5fe)) < i32::from(p.u16(0x202)) {
        0.0
    } else {
        1.0
    };
    car.set_f32(0x618, clutch);
    car.set_f32(0x598, throttle);
    for i in 0..4 {
        car.set_f32(w(i, 0x64), brake);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
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
    fn original_complete_steering_and_analog_input() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_steering_frame.json"
        ))
        .unwrap();
        assert_eq!(
            fixture["target_sha256"],
            "e0db8fec04eec28594c9d6946567d0bd7c6b1b50177427204e054fa34fcf7804"
        );
        let parameters = NativeParameters::fixture_parameters();
        let p = &parameters;
        let cases = fixture["cases"].as_array().unwrap();
        assert_eq!(cases.len(), 60);
        for (index, c) in cases.iter().enumerate() {
            let mut car = NativeCar::default();
            for v in c["input_nonzero_words"].as_array().unwrap() {
                car.set_u32(
                    v[0].as_u64().unwrap() as usize,
                    v[1].as_u64().unwrap() as u32,
                );
            }
            match c["kind"].as_str().unwrap() {
                "steering_frame" => map_wheels(&mut car, p).unwrap(),
                "analog_input" => {
                    let hex = c["packet_hex"].as_str().unwrap();
                    let packet: Vec<u8> = (0..hex.len())
                        .step_by(2)
                        .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
                        .collect();
                    let u = |o| u16::from_le_bytes(packet[o..o + 2].try_into().unwrap());
                    input_analog(
                        &mut car,
                        p,
                        AnalogInput {
                            steering: u(4) as i16,
                            throttle: u(8),
                            brake: u(10),
                            reverse: u(12),
                            handbrake: u(14) as i16,
                            shift: packet[2],
                        },
                    )
                    .unwrap();
                }
                _ => panic!("unexpected steering fixture"),
            }
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
                    assert_eq!(actual, expected, "case {index} field{offset:x}");
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
