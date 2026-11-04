//! Small deterministic PRNG shared by track generation, policy init and rollouts.

/// SplitMix64 stream with an optional spare normal sample.
pub struct Rng {
    state: u64,
    spare: Option<f64>,
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self {
            state: seed.wrapping_add(0x9e37_79b9_7f4a_7c15),
            spare: None,
        }
    }

    pub fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    pub fn next_u32(&mut self) -> u32 {
        (self.next_u64() >> 32) as u32
    }

    /// Uniform in `[0, 1)`.
    pub fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    pub fn range(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.next_f64()
    }

    pub fn unit(&mut self) -> Vec2 {
        let angle = self.range(0.0, std::f64::consts::TAU);
        Vec2 {
            x: angle.cos(),
            y: angle.sin(),
        }
    }

    /// Standard normal by Box-Muller, keeping the second sample.
    pub fn normal(&mut self) -> f64 {
        if let Some(spare) = self.spare.take() {
            return spare;
        }
        let u1 = 1.0 - self.next_f64();
        let u2 = self.next_f64();
        let radius = (-2.0 * u1.ln()).sqrt();
        let angle = std::f64::consts::TAU * u2;
        self.spare = Some(radius * angle.sin());
        radius * angle.cos()
    }

    pub fn normals_f32(&mut self, out: &mut [f32]) {
        for slot in out {
            *slot = self.normal() as f32;
        }
    }

    pub fn normals_f64(&mut self, out: &mut [f64]) {
        for slot in out {
            *slot = self.normal();
        }
    }
}

use crate::Vec2;
