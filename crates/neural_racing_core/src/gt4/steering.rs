//! Address-specific GT4 steering calibration and default digital response.
//!
//! The full controller routine 0x348850 also contains wheel-controller modes,
//! track assistance and optional countersteering. This module exposes its
//! ordinary four-wheel digital branch with those optional branches disabled.

use super::curve::Curve;

const DEGREES: f32 = f32::from_bits(0x3c8e_fa34);
const KMH: f32 = f32::from_bits(0x3e8e_38e3);

/// Six-point speed/angle curve produced by 0x360468.
#[derive(Clone, Debug)]
pub struct SteeringTable {
    pub speeds: [f32; 6],
    pub angles: [f32; 6],
}

impl SteeringTable {
    pub fn from_bytes(speeds: [u8; 6], angles: [u8; 6]) -> Self {
        Self {
            speeds: speeds.map(|v| f32::from(v) * KMH),
            angles: angles.map(|v| (f32::from(v) * f32::from_bits(0x3e4c_cccc)) * DEGREES),
        }
    }

    pub fn sample(&self, speed: f32) -> f32 {
        Curve::new(&self.speeds, &self.angles)
            .expect("GT4 steering speeds must increase")
            .sample_linear(speed)
    }
}

/// Raw 38-byte STEER-row conversion 0x4419e0 followed by 0x360468.
#[derive(Clone, Debug)]
pub struct SteeringCalibration {
    pub maximum_angle: f32,
    pub normal: SteeringTable,
    pub wheel_controller: SteeringTable,
    pub alternate: SteeringTable,
}

impl SteeringCalibration {
    pub fn from_specdb(row: &[u8; 38]) -> Self {
        fn six(row: &[u8], at: usize) -> [u8; 6] {
            row[at..at + 6].try_into().expect("fixed six-byte field")
        }
        Self {
            maximum_angle: f32::from(row[37]) * DEGREES,
            normal: SteeringTable::from_bytes(six(row, 1), six(row, 7)),
            wheel_controller: SteeringTable::from_bytes(six(row, 13), six(row, 19)),
            alternate: SteeringTable::from_bytes(six(row, 25), six(row, 31)),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SteeringState {
    /// car+0x56c
    pub angle: f32,
    /// car+0x588
    pub rate: f32,
}

/// Initialized globals from 0x347a00, indexed by surface/control group.
#[derive(Clone, Copy, Debug)]
pub struct SteeringResponse {
    pub stiffness: f32,
    pub return_stiffness: f32,
    pub heading_stiffness: f32,
    pub damping: f32,
    /// Digital and analog entries for normal, return and heading-following.
    pub limits: [[f32; 2]; 3],
    pub fast_limits: [[f32; 2]; 3],
    pub speed_start: f32,
    pub speed_end: f32,
    pub speed_blend: bool,
}

impl SteeringResponse {
    /// Original bytes 0x622120 and 0x62212c..0x622167, not invented tuning.
    pub fn original(group: usize) -> Self {
        assert!(group < 2);
        let degree_pair = |a: [u8; 2]| a.map(|v| f32::from(v) * DEGREES);
        if group == 0 {
            Self {
                stiffness: 100.0,
                return_stiffness: 25.0,
                heading_stiffness: 30.0,
                damping: 200.0 * f32::from_bits(0x3dcc_cccc),
                limits: [
                    degree_pair([30, 75]),
                    degree_pair([60, 60]),
                    degree_pair([60, 75]),
                ],
                fast_limits: [
                    degree_pair([10, 10]),
                    degree_pair([60, 60]),
                    degree_pair([60, 75]),
                ],
                speed_start: 50.0 * KMH,
                speed_end: 250.0 * KMH,
                speed_blend: true,
            }
        } else {
            Self {
                stiffness: 50.0,
                return_stiffness: 45.0,
                heading_stiffness: 37.0,
                damping: 100.0 * f32::from_bits(0x3dcc_cccc),
                limits: [
                    degree_pair([90, 90]),
                    degree_pair([120, 150]),
                    degree_pair([120, 150]),
                ],
                fast_limits: [
                    degree_pair([90, 90]),
                    degree_pair([120, 150]),
                    degree_pair([120, 150]),
                ],
                speed_start: 0.0,
                speed_end: 0.0,
                speed_blend: false,
            }
        }
    }
}

/// 0x348588: move a scalar toward its target by a bounded step.
pub fn approach(target: f32, step: f32, value: &mut f32) {
    let mut next = *value;
    if next < target {
        next += step;
        if target < next {
            next = target;
        }
    } else if target < next {
        next -= step;
        if next < target {
            next = target;
        }
    }
    *value = next;
}

/// 0x3485d0, with explicit original global parameters replacing process memory.
pub fn update_response(
    state: &mut SteeringState,
    target: f32,
    front_heading: f32,
    forward_speed: f32,
    dt: f32,
    analog: bool,
    config: &SteeringResponse,
) {
    let current = state.angle;
    let mut acceleration = (target - current) * config.stiffness - state.rate * config.damping;
    let heading_follow = config.heading_stiffness != 0.0
        && ((current < target && current < front_heading)
            || (target < current && front_heading < current));
    let branch;
    if heading_follow {
        branch = 2;
        acceleration += (front_heading - current) * config.heading_stiffness;
    } else if (0.0 < current && target < current) || (current < 0.0 && current < target) {
        branch = 1;
        acceleration -= current * config.return_stiffness;
    } else {
        branch = 0;
    }
    let index = usize::from(analog);
    let slow_limit = config.limits[branch][index];
    let fast_limit = config.fast_limits[branch][index];
    let mut limit = slow_limit;
    if config.speed_blend && config.speed_start < forward_speed {
        limit = fast_limit;
        if forward_speed < config.speed_end {
            limit = slow_limit
                + ((fast_limit - slow_limit) * (forward_speed - config.speed_start))
                    / (config.speed_end - config.speed_start);
        }
    }
    let mut rate = state.rate + acceleration * dt;
    if limit < rate {
        rate = limit;
    } else if rate < -limit {
        rate = -limit;
    }
    let next = current + rate * dt;
    if (current <= target && target <= next) || (target <= current && next <= target) {
        state.angle = target;
        state.rate = 0.0;
    } else {
        state.rate = rate;
        state.angle = next;
    }
}

/// Default four-wheel digital/analog-stick selection in 0x348850.
///
/// Caller supplies the actual front-wheel heading average and selected normal
/// or alternate speed table. Optional track assistance and the two non-default
/// countersteering switches are outside this entry point's supported scope.
pub fn update_digital(
    state: &mut SteeringState,
    raw_input: i16,
    analog_axis: bool,
    front_heading: f32,
    forward_speed: f32,
    dt: f32,
    calibration: &SteeringCalibration,
    alternate: bool,
) {
    assert!((-4096..=4096).contains(&raw_input));
    let table = if alternate {
        &calibration.alternate
    } else {
        &calibration.normal
    };
    let speed = if forward_speed < 0.0 {
        0.0
    } else {
        forward_speed
    };
    let cap = table.sample(speed);
    let mut upper = cap;
    let mut lower = -cap;
    if cap < front_heading {
        upper = front_heading;
    } else if front_heading < -cap {
        lower = front_heading;
    }
    if calibration.maximum_angle < upper {
        upper = calibration.maximum_angle;
    }
    if lower < -calibration.maximum_angle {
        lower = -calibration.maximum_angle;
    }
    let mut target = if raw_input < 0 {
        lower
    } else if 0 < raw_input {
        upper
    } else {
        0.0
    };
    if analog_axis {
        let normalized = f32::from(raw_input) * f32::from_bits(0x3980_0000);
        target = if normalized < 0.0 {
            -lower * normalized
        } else {
            upper * normalized
        };
    }
    update_response(
        state,
        target,
        front_heading,
        forward_speed,
        dt,
        analog_axis,
        &SteeringResponse::original(usize::from(alternate)),
    );
}

/// 0x3f7ca8: per-wheel steering geometry, including toe and optional rear steer.
pub fn wheel_angle(
    input: f32,
    longitudinal_sign: i8,
    side: u8,
    factors: [f32; 2],
    rear_enabled: bool,
    rear_gain: f32,
    toe: f32,
) -> f32 {
    let index = if input < 0.0 && side < 2 {
        1 - side
    } else {
        side
    };
    let mut angle = if 0 < longitudinal_sign {
        input
    } else if longitudinal_sign < 0 && rear_enabled {
        -input * rear_gain
    } else {
        0.0
    };
    if (0 < longitudinal_sign || (longitudinal_sign < 0 && rear_enabled)) && index < 2 {
        angle *= factors[usize::from(index)];
    }
    if side == 0 {
        angle -= toe;
    } else if side == 1 {
        angle += toe;
    }
    angle
}
