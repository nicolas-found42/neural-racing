//! Original brake-temperature state update 0x3f38d8 and its 0x3f3808 initializer.
//! This preserves wheel+a4 stores. No brake-fade force multiplier is invented.
use super::{
    eefloat::{add, div, mul},
    native::NativeCar,
    parameters::NativeParameters,
};

#[derive(Clone, Copy, Debug)]
pub(crate) struct BrakeHeatParameters {
    pub at_845a00: f32,
    pub at_845a04: f32,
    pub at_845a08: f32,
    pub at_845a0c: f32,
}
impl BrakeHeatParameters {
    /// Raw settings are bytes620d84..620d87; zero first/second bytes mean one.
    pub(crate) fn from_bytes(bytes: [u8; 4]) -> Self {
        let n = f32::from(bytes[0].max(1));
        let m = f32::from(bytes[1].max(1));
        Self {
            at_845a00: div(1.0, mul(n, f32::from_bits(0x46191fff))),
            at_845a04: div(1.0, mul(mul(m, f32::from_bits(0x3dcccccc)), 100.0)),
            at_845a08: mul(f32::from(bytes[2]), f32::from_bits(0x3e8e38e3)),
            at_845a0c: mul(f32::from(bytes[3]), f32::from_bits(0x3c23d70a)),
        }
    }
    pub(crate) fn from_parameters(p: &NativeParameters) -> Self {
        Self::from_bytes(std::array::from_fn(|i| p.global_u8(0x620d84 + i as u32)))
    }
}

/// Original four-wheel numeric update after the force/pose pass. The original
/// bounded force/TCS/ASC paths have no identified consumer of wheel+a4; callers
/// must not infer a fade multiplier from this heat state.
pub(crate) fn update(car: &mut NativeCar, p: &NativeParameters, settings: BrakeHeatParameters) {
    let dt = car.f32(0x54c);
    let brake = car.f32(0x590);
    for i in 0..4 {
        let axle = p.wheel_groups()[i];
        let wheel = car.wheel(i);
        let speed = wheel.f32(0x20).abs();
        let mut generated = mul(
            mul(
                mul(
                    mul(p.f32(0x1010 + axle * 4), p.f32(0x2c4 + axle * 4)),
                    wheel.f32(0x64),
                ),
                speed,
            ),
            settings.at_845a00,
        );
        if brake == 0.0 {
            generated = 0.0;
        }
        let heat = wheel.f32(0xa4);
        let cooling = mul(
            mul(add(speed, settings.at_845a08), settings.at_845a04),
            add(heat, settings.at_845a0c),
        );
        let next = add(heat, mul(add(generated, -cooling), dt));
        car.wheel_mut(i).set_f32(
            0xa4,
            if next > 1.0 {
                1.0
            } else if next < 0.0 {
                0.0
            } else {
                next
            },
        );
    }
}

#[cfg(test)]
mod tests {
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
    fn original_initializer_and_heat_update_match_all150_cases() {
        let fixture: Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_brake_heat.json"
        ))
        .unwrap();
        assert_eq!(fixture["cases"].as_array().unwrap().len(), 150);
        let p = NativeParameters::fixture_parameters();
        for (index, c) in fixture["cases"].as_array().unwrap().iter().enumerate() {
            let raw = c["raw"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_u64().unwrap() as u8)
                .collect::<Vec<_>>()
                .try_into()
                .unwrap();
            let settings = BrakeHeatParameters::from_bytes(raw);
            let mut car = NativeCar::default();
            car.set_f32(0x54c, c["dt"].as_f64().unwrap() as f32);
            car.set_f32(0x590, c["brake"].as_f64().unwrap() as f32);
            for i in 0..4 {
                let mut w = car.wheel_mut(i);
                w.set_f32(0x20, c["speed"].as_f64().unwrap() as f32);
                w.set_f32(0xa4, c["heat"].as_f64().unwrap() as f32);
                w.set_f32(
                    0x64,
                    (c["request"].as_f64().unwrap() * (i + 1) as f64) as f32,
                );
            }
            update(&mut car, &p, settings);
            let outputs = [
                settings.at_845a00,
                settings.at_845a04,
                settings.at_845a08,
                settings.at_845a0c,
                car.wheel(0).f32(0xa4),
                car.wheel(1).f32(0xa4),
                car.wheel(2).f32(0xa4),
                car.wheel(3).f32(0xa4),
            ];
            for (field, (actual, expected)) in outputs
                .iter()
                .zip(c["output_bits"].as_array().unwrap())
                .enumerate()
            {
                let expected = u32::from_str_radix(expected.as_str().unwrap(), 16).unwrap();
                let ulp = (ordered(actual.to_bits()) - ordered(expected)).abs();
                assert!(ulp <= 12, "case{index} field{field}: {ulp}ULP");
            }
        }
    }
}
