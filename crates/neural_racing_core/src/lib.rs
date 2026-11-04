//! Real circuits, seeded procedural training courses, and four-contact-patch vehicle dynamics.

mod circuit;
mod env;
pub mod rng;
mod track;
pub mod vehicle;

pub use circuit::{Circuit, CAR_LENGTH, CAR_WIDTH, UNITS_PER_METRE};
pub use env::{Action, Env, RewardComponents, StepInfo, StepOutcome};
pub use rng::Rng;
pub use track::{Bounds, Gate, Track};

pub const MAX_SPEED: f64 = 80.0 * UNITS_PER_METRE;
pub const WHEELBASE: f64 = 28.0;
pub const MAX_STEER_ANGLE: f64 = 0.55;
pub const MAX_LATERAL_ACCEL: f64 = vehicle::GRIP * vehicle::GRAVITY * UNITS_PER_METRE;

pub const PHYSICS_DT: f64 = 1.0 / 120.0;
pub const SUBSTEPS: usize = 6;
pub const DECISION_DT: f64 = PHYSICS_DT * SUBSTEPS as f64;

/// Decision budget for a default two-lap race.
pub const MAX_STEPS: i32 = 1200;
pub const DEFAULT_LAPS: i32 = 2;
/// Nominal half width. Every generated track varies the actual width along
/// the lap; this is the mean the variation is built around.
pub const BASE_HALF_WIDTH: f64 = 30.0;
pub const CHECKPOINT_COUNT: usize = 8;
pub const NUM_RAYS: usize = 11;
pub const RAY_RANGE: f64 = 70.0 * UNITS_PER_METRE;

pub const ARENA_WIDTH: f64 = 960.0;
pub const ARENA_HEIGHT: f64 = 820.0;
pub const ARENA_CENTER: Vec2 = Vec2 {
    x: ARENA_WIDTH * 0.5,
    y: ARENA_HEIGHT * 0.5,
};

pub const START_SPEED: f64 = 5.0 * UNITS_PER_METRE;

/// Distance from the wall, as a fraction of the local road half width, where
/// the wall penalty starts biting.
pub const WALL_WARN_FRACTION: f64 = 0.85;
pub const WALL_PENALTY: f64 = 0.35;
pub const OFF_TRACK_PENALTY: f64 = 30.0;
/// Ending an episode by exhausting its step budget. Must exceed the off-track
/// penalty so parking before a corner is never preferred to attempting it.
pub const TIMEOUT_PENALTY: f64 = 40.0;
/// Dense cost for carrying more speed than the braking-distance-aware profile
/// at the car's current arc allows. Above the profile the car cannot stop for
/// the next corner; below it the reward is silent, so full throttle on a
/// straight is never punished.
pub const SPEED_EXCESS_PENALTY: f64 = 0.25;
/// Per decision at standstill, fading to zero at the safe speed profile.
pub const SPEED_SHORTFALL_PENALTY: f64 = 0.10;
/// Progress reward per metre travelled along the track.
pub const PROGRESS_WEIGHT: f64 = 0.1;
/// Paid for every checkpoint gate, so partial progress is worth chasing
/// instead of parking the car.
pub const CHECKPOINT_BONUS: f64 = 2.0;
pub const LAP_BONUS: f64 = 20.0;
pub const FAST_LAP_BONUS: f64 = 15.0;
pub const TIME_COST: f64 = 0.01;
/// Representative braking deceleration used to build the speed profile.
pub const BRAKING_ACCEL: f64 = 8.8;
/// The profile never asks for less than this, so recovery stays possible.
pub const PROFILE_MIN_SPEED: f64 = 6.0;

/// Hand-set features the policy sees besides the ray distances: speed, slip,
/// yaw rate, steer, lateral offset, heading error sin/cos, speed fraction, then
/// road preview (offset and width ahead, curvature and corner speed), steering
/// rate, applied pedal, actuator history, accelerations, tire loads/usage,
/// five signed road headings, braking distance, stint progress, the speed
/// profile and its margin, per-wheel grip use and load, road-frame lateral
/// velocity, tyre forces, command deltas, lateral demand and grip reserve.
pub const STATE_FEATURES: usize = 56;
/// Arc distances the policy previews the road at.
pub const PREVIEW_NEAR: f64 = 12.0 * UNITS_PER_METRE;
pub const PREVIEW_FAR: f64 = 35.0 * UNITS_PER_METRE;
/// Curvature preview horizon: the heading change over this arc is one feature.
pub const PREVIEW_CURVE: f64 = 20.0 * UNITS_PER_METRE;
/// How far ahead the policy checks for a corner it must slow down for.
pub const PREVIEW_CORNER: f64 = 300.0 * UNITS_PER_METRE;
/// `STATE_FEATURES` state values followed by `NUM_RAYS` normalized ray distances.
pub const OBS_DIM: usize = STATE_FEATURES + NUM_RAYS;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }

    pub fn norm(self) -> f64 {
        self.x.hypot(self.y)
    }

    pub fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }

    pub fn rotated(self, angle: f64) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self {
            x: self.x * cos - self.y * sin,
            y: self.x * sin + self.y * cos,
        }
    }
}

impl std::ops::Add for Vec2 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl std::ops::Sub for Vec2 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl std::ops::Mul<f64> for Vec2 {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl std::ops::Neg for Vec2 {
    type Output = Self;

    fn neg(self) -> Self {
        Self {
            x: -self.x,
            y: -self.y,
        }
    }
}

/// Wraps `value` into `[-PI, PI)`.
pub fn wrap_angle(value: f64) -> f64 {
    (value + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Python-style modulo for a positive modulus.
pub(crate) fn py_mod(value: f64, modulus: f64) -> f64 {
    value.rem_euclid(modulus)
}
