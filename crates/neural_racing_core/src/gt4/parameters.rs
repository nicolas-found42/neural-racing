//! Checked decoding of original initialized car-data snapshots. Native pointers
//! are resolved against explicit captured regions, never dereferenced as host
//! addresses. Curves/configuration are cached once and shared between cars.

use std::collections::BTreeMap;

use super::{
    curve::Curve,
    drivetrain::{AxleRotationConfig, CarrierWheelGroup, RearDrivelineConfig},
    tire::{Table, TireTables},
};

pub(crate) const CAR_DATA_BYTES: usize = 0x1100;

#[derive(Clone)]
pub(crate) struct NativeParameters {
    data: [u8; CAR_DATA_BYTES],
    base: u32,
    globals: Vec<u8>,
    global_base: u32,
    steering_globals: [u8; 32],
    response_globals: [u8; 904],
    curves: BTreeMap<usize, Table>,
    rolling: Table,
    tires: [Option<TireTables>; 6],
    driveline: RearDrivelineConfig,
    rotations: [AxleRotationConfig; 2],
    carriers: [CarrierWheelGroup; 2],
    wheel_groups: [usize; 4],
    course_kind: u8,
}

fn word(bytes: &[u8], offset: usize) -> Result<u32, &'static str> {
    let value = bytes
        .get(offset..offset.checked_add(4).ok_or("offset overflow")?)
        .ok_or("snapshot offset outside captured region")?;
    Ok(u32::from_le_bytes(value.try_into().unwrap()))
}

fn captured<'a>(
    data: &'a [u8],
    base: u32,
    globals: &'a [u8],
    global_base: u32,
    address: u32,
    size: usize,
) -> Result<&'a [u8], &'static str> {
    for (region, region_base) in [(data, base), (globals, global_base)] {
        if let Some(offset) = address.checked_sub(region_base) {
            if let Some(value) = region.get(
                offset as usize
                    ..(offset as usize)
                        .checked_add(size)
                        .ok_or("captured pointer overflow")?,
            ) {
                return Ok(value);
            }
        }
    }
    Err("native pointer outside captured parameter regions")
}

fn table_at(
    data: &[u8],
    base: u32,
    globals: &[u8],
    global_base: u32,
    address: u32,
) -> Result<Table, &'static str> {
    let descriptor = captured(data, base, globals, global_base, address, 12)?;
    let count = word(descriptor, 0)? as usize;
    if !(1..=32).contains(&count) {
        return Err("unsupported native curve count");
    }
    let read_values = |pointer| -> Result<Vec<f32>, &'static str> {
        let bytes = captured(data, base, globals, global_base, pointer, count * 4)?;
        Ok(bytes
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes(b.try_into().unwrap()))
            .collect())
    };
    let x = read_values(word(descriptor, 4)?)?;
    let y = read_values(word(descriptor, 8)?)?;
    Curve::new(&x, &y).ok_or("invalid initialized native curve")?;
    Ok(Table { x, y })
}

impl NativeParameters {
    pub(crate) fn m3() -> &'static std::sync::Arc<Self> {
        static PARAMETERS: std::sync::OnceLock<std::sync::Arc<NativeParameters>> =
            std::sync::OnceLock::new();
        PARAMETERS.get_or_init(|| {
            std::sync::Arc::new(
                Self::from_snapshots(
                    *include_bytes!("data/m3_gtr_alms_01.bin"),
                    0x01d0_2000,
                    include_bytes!("data/global_parameters.bin").to_vec(),
                    0x0062_0000,
                )
                .expect("verified GT4 M3 initialized parameter snapshot"),
            )
        })
    }

    /// Original 0x34c998 classification: dry/wet=0, dirt=1, snow=2.
    pub(crate) fn course_kind(&self) -> u8 {
        self.course_kind
    }

    /// Separately initialized immutable snapshots; changing courses never edits
    /// another car's global or tire parameters. Wet material remapping is driven
    /// by the recovered triangle flags rather than a global grip multiplier.
    pub(crate) fn m3_for_condition(
        condition: u8,
    ) -> Result<&'static std::sync::Arc<Self>, &'static str> {
        static DIRT: std::sync::OnceLock<std::sync::Arc<NativeParameters>> =
            std::sync::OnceLock::new();
        static SNOW: std::sync::OnceLock<std::sync::Arc<NativeParameters>> =
            std::sync::OnceLock::new();
        let create = |snow: bool| {
            let globals = if snow {
                include_bytes!("data/global_parameters_snow.bin").as_slice()
            } else {
                include_bytes!("data/global_parameters.bin").as_slice()
            };
            let mut p = Self::from_snapshots(
                *include_bytes!("data/m3_gtr_alms_01_offroad.bin"),
                0x01d0_2000,
                globals.to_vec(),
                0x0062_0000,
            )
            .expect("original initialized off-road M3 snapshots");
            p.course_kind = if snow { 2 } else { 1 };
            std::sync::Arc::new(p)
        };
        match condition {
            0 | 1 => Ok(Self::m3()),
            2 => Ok(DIRT.get_or_init(|| create(false))),
            3 => Ok(SNOW.get_or_init(|| create(true))),
            _ => Err("unknown native course condition"),
        }
    }

    pub(crate) fn base_address(&self) -> u32 {
        self.base
    }
    /// Accepts fully initialized outputs, not database rows or uninitialized
    /// executable BSS. The selected adapter supports normal four-wheel FR cars.
    pub(crate) fn from_snapshots(
        data: [u8; CAR_DATA_BYTES],
        base: u32,
        globals: Vec<u8>,
        global_base: u32,
    ) -> Result<Self, &'static str> {
        base.checked_add(CAR_DATA_BYTES as u32)
            .ok_or("car-data address overflow")?;
        let gf = |address: u32| -> Result<u8, &'static str> {
            Ok(captured(&data, base, &globals, global_base, address, 1)?[0])
        };
        if data[0x30] != 0
            || gf(0x620479)? != 4
            || gf(0x62047a)? != 2
            || data[0x234] != 0
            || data[0x246] != 0
        {
            return Err("initialized car is outside the supported four-wheel FR configuration");
        }
        let f = |offset| -> Result<f32, &'static str> {
            let value = f32::from_bits(word(&data, offset)?);
            if !value.is_finite() {
                return Err("nonfinite native parameter");
            }
            Ok(value)
        };
        if f(0x48)? <= 0.0 || f(0x4c)? <= 0.0 {
            return Err("nonpositive initialized car mass");
        }
        let mut curves = BTreeMap::new();
        for offset in [0x58, 0x94, 0xd0, 0x134, 0x3b8, 0x3c4] {
            curves.insert(
                offset,
                table_at(&data, base, &globals, global_base, base + offset as u32)?,
            );
        }
        let rolling = table_at(&data, base, &globals, global_base, 0x620938)?;
        for address in [0x620b24, 0x620b40, 0x620b54] {
            curves.insert(
                address,
                table_at(&data, base, &globals, global_base, address as u32)?,
            );
        }
        let mut tires = std::array::from_fn(|_| None);
        for (slot, out) in tires.iter_mut().enumerate() {
            let offset = 0x410 + slot * 0x1ec;
            if word(&data, offset + 0xf8)? == 0 {
                continue;
            }
            let table = |at| {
                table_at(
                    &data,
                    base,
                    &globals,
                    global_base,
                    base + (offset + at) as u32,
                )
            };
            let mut terrain = [0.0; 24];
            for (i, value) in terrain.iter_mut().enumerate() {
                *value = f(offset + 8 + i * 4)?;
            }
            *out = Some(TireTables {
                slip_mu_a: table(0xf8)?,
                slip_mu_b: table(0x170)?,
                side: table(0x6c)?,
                corner_drag: table(0xbc)?,
                load: table(0x1ac)?,
                negative_peak_at_164: f(offset + 0x164)?,
                positive_peak_at_168: f(offset + 0x168)?,
                positive_peak_mu_at_16c: f(offset + 0x16c)?,
                at_00: f(offset)?,
                at_04: f(offset + 4)?,
                at_68: f(offset + 0x68)?,
                terrain_at_08: terrain,
            });
        }
        if data[0xf98..0xfc8]
            .iter()
            .any(|&slot| usize::from(slot) >= tires.len() || tires[usize::from(slot)].is_none())
        {
            return Err("invalid selected tire slot");
        }
        let mut wheel_groups = [0; 4];
        for (i, group) in wheel_groups.iter_mut().enumerate() {
            *group = usize::from(gf(0x62048c + i as u32)?);
        }
        let mut carriers = [CarrierWheelGroup {
            group: 0,
            first_wheel: 0,
            paired_wheel: None,
        }; 2];
        for (i, carrier) in carriers.iter_mut().enumerate() {
            let first = usize::from(gf(0x620484 + i as u32)?);
            if first >= 4 {
                return Err("invalid captured carrier first wheel");
            }
            let paired = gf(0x62049c + first as u32)?;
            let group = usize::from(gf(0x620488 + i as u32)?);
            if group > 1 || (paired != 0x7f && paired >= 4) {
                return Err("invalid captured carrier metadata");
            }
            *carrier = CarrierWheelGroup {
                group,
                first_wheel: first,
                paired_wheel: (paired != 0x7f).then_some(usize::from(paired)),
            };
        }
        let mut wheel_drive_signs = Vec::with_capacity(4);
        for i in 0..4 {
            wheel_drive_signs.push(gf(0x6204b4 + i)? as i8);
        }
        let count = usize::from(data[0x236]);
        if !(1..=9).contains(&count) || wheel_groups.iter().any(|&g| g > 1) {
            return Err("invalid captured gear/wheel count");
        }
        let driveline = RearDrivelineConfig {
            drive_mode_at_234: data[0x234],
            gearbox_mode_at_246: data[0x246],
            transfer_at_235: data[0x235] != 0,
            driven_group_at_237: usize::from(data[0x237]),
            group_kinds_at_244: [data[0x244], data[0x245]],
            wheel_drive_signs_at_3c: wheel_drive_signs,
            gear_ratios_at_274: (0..=count)
                .map(|i| f(0x274 + i * 4))
                .collect::<Result<_, _>>()?,
            clutch_torque_at_2e4: f(0x2e4)?,
            shaft_inertias_at_2f0: [f(0x2f0)?, f(0x2f4)?],
            engine_inverse_inertia_at_2fc: f(0x2fc)?,
            engine_inertia_at_240: f(0x240)?,
            group_ratios_at_2c4: [f(0x2c4)?, f(0x2c8)?],
            group_ratios_at_2cc: [f(0x2cc)?, f(0x2d0)?],
        };
        if driveline.transfer_at_235 || driveline.driven_group_at_237 != 1 {
            return Err("unsupported captured transfer configuration");
        }
        let mut rotations = [AxleRotationConfig {
            first_wheel: 0,
            paired_wheel: None,
            wheel_count_at_08: 2,
            kind_at_10: 0,
            preload_at_248: 0.0,
            drive_coefficient_at_1c: 0.0,
            coast_coefficient_at_258: 0.0,
            viscous_limit_at_14: 0.0,
            inverse_inertia_at_b4: 0.0,
            rotation_ratio_at_90: 0.0,
        }; 2];
        for (i, rotation) in rotations.iter_mut().enumerate() {
            let carrier = carriers[i];
            let group = carrier.group;
            *rotation = AxleRotationConfig {
                first_wheel: carrier.first_wheel,
                paired_wheel: carrier.paired_wheel,
                wheel_count_at_08: gf(0x620480 + i as u32)?,
                kind_at_10: data[0x244 + group],
                preload_at_248: f(0x248 + group * 4)?,
                drive_coefficient_at_1c: f(0x250 + group * 4)?,
                coast_coefficient_at_258: f(0x258 + group * 4)?,
                viscous_limit_at_14: f(0x248 + group * 4)?,
                inverse_inertia_at_b4: f(0x2e8 + group * 4)?,
                rotation_ratio_at_90: f(0x2c4 + group * 4)?,
            };
        }
        Ok(Self {
            data,
            base,
            globals,
            global_base,
            steering_globals: *include_bytes!("data/steering_globals.bin"),
            response_globals: *include_bytes!("data/steering_response_globals.bin"),
            curves,
            rolling,
            tires,
            driveline,
            rotations,
            carriers,
            wheel_groups,
            course_kind: 0,
        })
    }

    pub(crate) fn f32(&self, offset: usize) -> f32 {
        f32::from_bits(self.u32(offset))
    }
    pub(crate) fn u8(&self, offset: usize) -> u8 {
        self.data[offset]
    }
    pub(crate) fn i8(&self, offset: usize) -> i8 {
        self.u8(offset) as i8
    }
    pub(crate) fn u16(&self, offset: usize) -> u16 {
        u16::from_le_bytes(self.data[offset..offset + 2].try_into().unwrap())
    }
    pub(crate) fn i16(&self, offset: usize) -> i16 {
        self.u16(offset) as i16
    }
    pub(crate) fn u32(&self, offset: usize) -> u32 {
        word(&self.data, offset).unwrap()
    }
    pub(crate) fn curve(&self, offset: usize) -> Curve<'_> {
        self.curves[&offset].curve()
    }
    pub(crate) fn steering_table(&self, offset: usize) -> super::steering::SteeringTable {
        let table = &self.curves[&offset];
        super::steering::SteeringTable {
            speeds: table
                .x
                .as_slice()
                .try_into()
                .expect("six native steering knots"),
            angles: table
                .y
                .as_slice()
                .try_into()
                .expect("six native steering ordinates"),
        }
    }
    pub(crate) fn rolling_curve(&self) -> Curve<'_> {
        self.rolling.curve()
    }
    pub(crate) fn global_curve(&self, address: u32) -> Curve<'_> {
        self.curves[&(address as usize)].curve()
    }
    pub(crate) fn global_u8(&self, address: u32) -> u8 {
        self.globals[(address - self.global_base) as usize]
    }
    pub(crate) fn global_f32(&self, address: u32) -> f32 {
        f32::from_bits(word(&self.globals, (address - self.global_base) as usize).unwrap())
    }
    pub(crate) fn model_u8(&self, offset: usize) -> u8 {
        self.global_u8(0x620478 + offset as u32)
    }
    pub(crate) fn model_i8(&self, offset: usize) -> i8 {
        self.model_u8(offset) as i8
    }
    pub(crate) fn angle_mode(&self) -> i32 {
        self.steering_global_u32(0x845bf0) as i32
    }
    pub(crate) fn steering_global_f32(&self, address: u32) -> f32 {
        f32::from_bits(self.steering_global_u32(address))
    }
    pub(crate) fn steering_global_u32(&self, address: u32) -> u32 {
        let offset = (address - 0x845bf0) as usize;
        u32::from_le_bytes(
            self.steering_globals[offset..offset + 4]
                .try_into()
                .unwrap(),
        )
    }
    pub(crate) fn response_global_f32(&self, address: u32) -> f32 {
        f32::from_bits(self.response_global_u32(address))
    }
    pub(crate) fn response_global_u32(&self, address: u32) -> u32 {
        let offset = (address - 0x843538) as usize;
        u32::from_le_bytes(
            self.response_globals[offset..offset + 4]
                .try_into()
                .unwrap(),
        )
    }
    pub(crate) fn tire_slot(&self, wheel: usize, surface: usize) -> usize {
        assert!(wheel < 4 && surface < 24);
        usize::from(self.data[0xf98 + self.wheel_groups[wheel] * 24 + surface])
    }
    pub(crate) fn tire(&self, slot: usize) -> &TireTables {
        self.tires[slot]
            .as_ref()
            .expect("uninitialized selected tire slot")
    }
    pub(crate) fn tire_for_wheel(&self, wheel: usize, surface: usize) -> &TireTables {
        self.tire(self.tire_slot(wheel, surface))
    }
    pub(crate) fn driveline_config(&self) -> &RearDrivelineConfig {
        &self.driveline
    }
    pub(crate) fn rotation_configs(&self) -> &[AxleRotationConfig] {
        &self.rotations
    }
    pub(crate) fn carriers(&self) -> &[CarrierWheelGroup] {
        &self.carriers
    }
    pub(crate) fn wheel_groups(&self) -> &[usize] {
        &self.wheel_groups
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn original_course_condition_initialization_is_isolated_and_byte_exact() {
        let f: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_original_course_conditions.json"
        ))
        .unwrap();
        let decode = |s: &str| {
            s.as_bytes()
                .chunks_exact(2)
                .map(|b| u8::from_str_radix(std::str::from_utf8(b).unwrap(), 16).unwrap())
                .collect::<Vec<_>>()
        };
        for c in f["cases"].as_array().unwrap() {
            let condition = c["course_type"].as_u64().unwrap() as u8;
            let p = NativeParameters::m3_for_condition(condition).unwrap();
            assert_eq!(
                p.data.as_slice(),
                decode(c["expected_car_data_hex"].as_str().unwrap())
            );
            assert_eq!(
                p.globals,
                decode(c["expected_globals_hex"].as_str().unwrap())
            );
            assert_eq!(
                p.course_kind(),
                match condition {
                    2 => 1,
                    3 => 2,
                    _ => 0,
                }
            );
        }
        assert!(std::sync::Arc::ptr_eq(
            NativeParameters::m3(),
            NativeParameters::m3_for_condition(1).unwrap()
        ));
        assert!(!std::sync::Arc::ptr_eq(
            NativeParameters::m3(),
            NativeParameters::m3_for_condition(2).unwrap()
        ));
        assert!(!std::sync::Arc::ptr_eq(
            NativeParameters::m3_for_condition(2).unwrap(),
            NativeParameters::m3_for_condition(3).unwrap()
        ));
        assert!(NativeParameters::m3_for_condition(4).is_err());
    }

    #[test]
    fn verified_m3_snapshot_decodes_without_host_pointers() {
        let p = NativeParameters::m3();
        assert_eq!(p.f32(0x48), 1190.0);
        assert_eq!(p.f32(0x4c), 1020.0);
        assert_eq!(p.driveline_config().gear_ratios_at_274.len(), 7);
        assert_eq!(p.model_u8(1), 4);
        for wheel in 0..4 {
            for surface in 0..24 {
                assert!(!p.tire_for_wheel(wheel, surface).load.x.is_empty());
            }
        }
        assert_eq!(p.rolling.x.len(), 8);
    }

    #[test]
    fn unbacked_native_curve_pointers_are_rejected() {
        let mut bytes = *include_bytes!("data/m3_gtr_alms_01.bin");
        bytes[0x58 + 4..0x58 + 8].copy_from_slice(&0xdead_beefu32.to_le_bytes());
        assert!(NativeParameters::from_snapshots(
            bytes,
            0x01d0_2000,
            include_bytes!("data/global_parameters.bin").to_vec(),
            0x0062_0000
        )
        .is_err());
    }
}

#[cfg(test)]
impl NativeParameters {
    /// Exact initialized inputs frozen with the promoted original EE outputs.
    /// Historical numeric fixtures must not silently follow changing game defaults.
    pub(crate) fn fixture_parameters() -> Self {
        let document: serde_json::Value = serde_json::from_str(include_str!(
            "../../tests/fixtures/gt4_native_parameter_inputs.json"
        ))
        .unwrap();
        fn decode(value: &serde_json::Value) -> Vec<u8> {
            let hex = value["hex"].as_str().unwrap();
            (0..hex.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
                .collect()
        }
        let mut parameters = Self::from_snapshots(
            decode(&document["m3_gtr_alms_01.bin"]).try_into().unwrap(),
            document["car_data_base"].as_u64().unwrap() as u32,
            decode(&document["global_parameters.bin"]),
            document["global_base"].as_u64().unwrap() as u32,
        )
        .expect("promoted original-execution parameter inputs remain valid");
        parameters.steering_globals = decode(&document["steering_globals.bin"])
            .try_into()
            .expect("frozen 32-byte steering globals");
        parameters.response_globals = decode(&document["steering_response_globals.bin"])
            .try_into()
            .expect("frozen 904-byte response globals");
        parameters
    }
}
