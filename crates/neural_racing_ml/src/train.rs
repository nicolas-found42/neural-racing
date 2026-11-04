//! PPO with a shared-trunk critic, GAE and clipped surrogate updates.
//!
//! The actor is a Gaussian in pre-tanh space; the critic is the third output of
//! the same trunk. Advantages are normalized per batch, the proposal policy's
//! log probabilities are stored with every sample, and the update is clipped to
//! a trust region so a bad batch cannot destroy the current driving skill.

use crate::adam::Adam;
use crate::policy::{
    gaussian_log_prob, log_std_from_raw, log_std_raw_grad, Policy, Sample, ACTION_DIM,
};
use neural_racing_core::Rng;

pub const GAMMA: f64 = 0.999;
pub const GAE_LAMBDA: f64 = 0.95;
pub const LR: f64 = 1e-4;
pub const CLIP_RANGE: f64 = 0.2;
pub const VALUE_COEF: f64 = 0.5;
pub const ENTROPY_COEF: f64 = 0.02;
pub const EPOCHS: usize = 3;
pub const MINIBATCH: usize = 256;
pub const MAX_GRAD_NORM: f64 = 1.0;

/// One episode batch flattened to contiguous rows.
pub struct Batch {
    pub rows: usize,
    pub obs_dim: usize,
    pub act_dim: usize,
    pub obs: Vec<f32>,
    /// Pre-tanh action actually taken, `act_dim` per row.
    pub z: Vec<f32>,
    /// Log probability of `z` under the proposal policy.
    pub old_logp: Vec<f32>,
    /// State value predicted by the proposal policy.
    pub values: Vec<f32>,
    pub returns: Vec<f32>,
    pub advantages: Vec<f32>,
}

impl Batch {
    pub fn new(obs_dim: usize, act_dim: usize) -> Self {
        Self {
            rows: 0,
            obs_dim,
            act_dim,
            obs: Vec::new(),
            z: Vec::new(),
            old_logp: Vec::new(),
            values: Vec::new(),
            returns: Vec::new(),
            advantages: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.rows = 0;
        self.obs.clear();
        self.z.clear();
        self.old_logp.clear();
        self.values.clear();
        self.returns.clear();
        self.advantages.clear();
    }

    pub fn push_row(
        &mut self,
        obs: &[f32],
        z: &[f32],
        logp: f32,
        value: f32,
        ret: f32,
        advantage: f32,
    ) {
        debug_assert_eq!(obs.len(), self.obs_dim);
        debug_assert_eq!(z.len(), self.act_dim);
        self.obs.extend_from_slice(obs);
        self.z.extend_from_slice(z);
        self.old_logp.push(logp);
        self.values.push(value);
        self.returns.push(ret);
        self.advantages.push(advantage);
        self.rows += 1;
    }

    /// Pushes one complete trajectory. `terminal` means the last reward ended
    /// the episode; otherwise the last value bootstraps a truncated cut.
    pub fn push_trajectory(&mut self, trajectory: &Trajectory, terminal: bool) {
        let rows = trajectory.rewards.len();
        if rows == 0 {
            return;
        }
        let mut advantages = vec![0.0f64; rows];
        let mut gae = 0.0f64;
        for step in (0..rows).rev() {
            let next_value = if step + 1 == rows {
                if terminal {
                    0.0
                } else {
                    f64::from(trajectory.bootstrap)
                }
            } else {
                f64::from(trajectory.values[step + 1])
            };
            let alive = if step + 1 == rows && terminal {
                0.0
            } else {
                1.0
            };
            let delta = f64::from(trajectory.rewards[step]) + GAMMA * next_value * alive
                - f64::from(trajectory.values[step]);
            gae = delta + GAMMA * GAE_LAMBDA * alive * gae;
            advantages[step] = gae;
        }
        for step in 0..rows {
            let value = trajectory.values[step];
            self.push_row(
                &trajectory.obs[step * self.obs_dim..(step + 1) * self.obs_dim],
                &trajectory.z[step * self.act_dim..(step + 1) * self.act_dim],
                trajectory.logp[step],
                value,
                (f64::from(value) + advantages[step]) as f32,
                advantages[step] as f32,
            );
        }
    }
}

/// One unfinished episode. Ends either at a terminal outcome or at the update
/// boundary; the boundary is bootstrapped, never treated as terminal.
#[derive(Default)]
pub struct Trajectory {
    pub obs: Vec<f32>,
    pub z: Vec<f32>,
    pub logp: Vec<f32>,
    pub values: Vec<f32>,
    pub rewards: Vec<f32>,
    pub bootstrap: f32,
    pub rows: usize,
}

impl Trajectory {
    pub fn push(&mut self, obs: &[f32], sample: &Sample, reward: f32) {
        self.obs.extend_from_slice(obs);
        self.z.extend_from_slice(&sample.z);
        self.logp.push(sample.logp);
        self.values.push(sample.value);
        self.rewards.push(reward);
        self.rows += 1;
    }

    pub fn clear(&mut self) {
        self.obs.clear();
        self.z.clear();
        self.logp.clear();
        self.values.clear();
        self.rewards.clear();
        self.bootstrap = 0.0;
        self.rows = 0;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpdateStats {
    pub gradient_norm: f64,
    pub gradient_clip: f64,
    pub parameter_norm: f64,
    pub policy_kl: f64,
    pub update_scale: f64,
    pub entropy: f64,
    pub value_loss: f64,
    pub explained_variance: f64,
}

fn huber(delta: f64) -> f64 {
    if delta.abs() <= 1.0 {
        0.5 * delta * delta
    } else {
        delta.abs() - 0.5
    }
}

/// Runs `EPOCHS` clipped-surrogate passes over shuffled minibatches.
pub fn update(
    policy: &mut Policy,
    optimizer: &mut Adam,
    batch: &Batch,
) -> Result<UpdateStats, &'static str> {
    if batch.rows == 0 {
        return Err("empty batch");
    }
    let finite = batch
        .obs
        .iter()
        .chain(&batch.z)
        .chain(&batch.old_logp)
        .chain(&batch.values)
        .chain(&batch.returns)
        .chain(&batch.advantages)
        .all(|value| value.is_finite());
    if !finite {
        return Err("non-finite batch");
    }
    let rows = batch.rows;
    let out_dim = policy.outputs_per_row();

    // Normalize advantages so gradient scale does not depend on episode luck.
    let mean: f64 = batch.advantages.iter().map(|a| f64::from(*a)).sum::<f64>() / rows as f64;
    let var: f64 = batch
        .advantages
        .iter()
        .map(|a| {
            let d = f64::from(*a) - mean;
            d * d
        })
        .sum::<f64>()
        / rows as f64;
    let inv_std = 1.0 / (var + 1e-8).sqrt();
    let advantages: Vec<f32> = batch
        .advantages
        .iter()
        .map(|a| ((f64::from(*a) - mean) * inv_std) as f32)
        .collect();

    let before_params = policy.params().to_vec();
    let mut before_moments = Some(optimizer.snapshot());
    let mut rng = Rng::new(0x5eed_1234 ^ rows as u64);
    let mut indices: Vec<usize> = (0..rows).collect();
    let mut last_kl = 0.0f64;
    let mut last_entropy = 0.0f64;
    let mut last_value_loss = 0.0f64;
    let mut gradient_norm = 0.0f64;
    let mut update_scale = 1.0f64;
    let solved_mean: f64 = batch
        .values
        .iter()
        .zip(&batch.returns)
        .map(|(value, ret)| (f64::from(*value) - f64::from(*ret)).powi(2))
        .sum::<f64>()
        / rows as f64;
    let return_mean: f64 = batch.returns.iter().map(|v| f64::from(*v)).sum::<f64>() / rows as f64;
    let return_var: f64 = batch
        .returns
        .iter()
        .map(|v| {
            let d = f64::from(*v) - return_mean;
            d * d
        })
        .sum::<f64>()
        / rows as f64;
    let explained_variance = 1.0 - solved_mean / (return_var + 1e-8);

    'training: for _epoch in 0..EPOCHS {
        for index in (1..rows).rev() {
            let swap = rng.next_u64() as usize % (index + 1);
            indices.swap(index, swap);
        }
        for chunk in indices.chunks(MINIBATCH) {
            let count = chunk.len();
            let mut obs = Vec::with_capacity(count * policy.obs_dim);
            for &row in chunk {
                obs.extend_from_slice(&batch.obs[row * policy.obs_dim..(row + 1) * policy.obs_dim]);
            }
            let (outputs, cache) = policy.forward_cached(&obs, count);
            let mut d_outputs = vec![0f32; count * out_dim];
            let mut kl_sum = 0.0f64;
            let mut entropy_sum = 0.0f64;
            let mut value_loss_sum = 0.0f64;
            for (local, &row) in chunk.iter().enumerate() {
                let mean_slice = &outputs[local * out_dim..local * out_dim + ACTION_DIM];
                let log_std_slice =
                    &outputs[local * out_dim + ACTION_DIM..local * out_dim + 2 * ACTION_DIM];
                let value = outputs[local * out_dim + 2 * ACTION_DIM];
                let z = &batch.z[row * ACTION_DIM..(row + 1) * ACTION_DIM];
                let old = batch.old_logp[row];
                let advantage = f64::from(advantages[row]);
                let mut new_logp = 0.0f32;
                let mut clamped = [0f32; ACTION_DIM];
                for dim in 0..ACTION_DIM {
                    // The update must use exactly the distribution that sampled
                    // the action; the raw head is clamped, so clamp here too.
                    clamped[dim] = log_std_from_raw(log_std_slice[dim]);
                    new_logp += gaussian_log_prob(z[dim], mean_slice[dim], clamped[dim]);
                }
                let ratio = (f64::from(new_logp - old)).exp();
                let clipped = ratio.clamp(1.0 - CLIP_RANGE, 1.0 + CLIP_RANGE);
                let unclipped_loss = ratio * advantage;
                let clipped_loss = clipped * advantage;
                // Gradient flows only through the smaller branch.
                let surrogate_active = unclipped_loss <= clipped_loss;
                let d_ratio = if surrogate_active { -advantage } else { 0.0 };
                let d_logp = d_ratio * ratio;
                kl_sum += f64::from(old - new_logp);
                for dim in 0..ACTION_DIM {
                    let log_std = clamped[dim];
                    let sigma = log_std.exp();
                    let diff = z[dim] - mean_slice[dim];
                    let diff = f64::from(diff);
                    let sigma = f64::from(sigma);
                    let d_dmu = diff / (sigma * sigma);
                    let d_dls = diff * diff / (sigma * sigma) - 1.0;
                    let mean_slot = local * out_dim + dim;
                    let log_std_slot = local * out_dim + ACTION_DIM + dim;
                    d_outputs[mean_slot] = (d_logp * d_dmu) as f32;
                    let raw_grad = f64::from(log_std_raw_grad(log_std_slice[dim]));
                    d_outputs[log_std_slot] = ((d_logp * d_dls - ENTROPY_COEF) * raw_grad) as f32;
                    entropy_sum += f64::from(log_std);
                }
                let ret = f64::from(batch.returns[row]);
                let delta = f64::from(value) - ret;
                value_loss_sum += huber(delta);
                d_outputs[local * out_dim + 2 * ACTION_DIM] =
                    (VALUE_COEF * delta.clamp(-1.0, 1.0)) as f32;
            }
            last_kl = kl_sum / count as f64;
            last_entropy = entropy_sum / (count * ACTION_DIM) as f64;
            last_value_loss = value_loss_sum / count as f64;
            // Stop before the proposal leaves the trust region. The check is
            // per minibatch so one epoch cannot drift past the bound unnoticed.
            if last_kl.abs() > 0.02 {
                break 'training;
            }
            let mut grads = policy.backward(&cache, &d_outputs);
            let scale = 1.0 / count as f64;
            for layer in &mut grads {
                for value in layer.iter_mut() {
                    *value = (*value as f64 * scale) as f32;
                }
            }
            let finite = grads
                .iter()
                .flat_map(|layer| layer.iter())
                .all(|value| value.is_finite());
            if !finite {
                return Err("non-finite gradient");
            }
            if !policy
                .params()
                .iter()
                .flat_map(|layer| layer.iter())
                .all(|value| value.is_finite())
            {
                return Err("non-finite parameter");
            }
            gradient_norm = optimizer.step(policy.params_mut(), &grads);
            let after = policy.forward(&obs, count);
            let mut after_kl = 0.0f64;
            for (local, &row) in chunk.iter().enumerate() {
                let mut new_logp = 0.0f32;
                for dim in 0..ACTION_DIM {
                    let raw = after[local * out_dim + ACTION_DIM + dim];
                    new_logp += gaussian_log_prob(
                        batch.z[row * ACTION_DIM + dim],
                        after[local * out_dim + dim],
                        log_std_from_raw(raw),
                    );
                }
                after_kl += f64::from(batch.old_logp[row] - new_logp);
            }
            let after_kl = (after_kl / count as f64).abs();
            last_kl = last_kl.max(after_kl);
            if after_kl > 0.05 {
                // Hard trust-region guard: discard the whole update instead of
                // keeping a policy that already left the region mid-pass.
                for (layer, saved) in policy.params_mut().iter_mut().zip(&before_params) {
                    layer.copy_from_slice(saved);
                }
                if let Some(moments) = before_moments.take() {
                    optimizer.restore(moments);
                }
                update_scale = 0.0;
                break 'training;
            }
        }
    }
    let parameter_norm: f64 = policy
        .params()
        .iter()
        .flat_map(|layer| layer.iter())
        .map(|v| f64::from(*v) * f64::from(*v))
        .sum::<f64>()
        .sqrt();
    Ok(UpdateStats {
        gradient_norm,
        gradient_clip: optimizer.max_grad_norm(),
        parameter_norm,
        policy_kl: last_kl.abs(),
        update_scale,
        entropy: last_entropy,
        value_loss: last_value_loss,
        explained_variance,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::policy::Policy;

    #[test]
    fn sampled_log_probability_matches_the_stored_value() {
        let policy = Policy::new(5, 8, 2, 1);
        let obs = [0.2f32, -0.1, 0.4, 0.0, 0.9];
        let outputs = policy.forward(&obs, 1);
        let sample = policy.sample(&obs, &mut Rng::new(9));
        let mut logp = 0.0f32;
        for dim in 0..ACTION_DIM {
            let mean = sample.mean[dim];
            let log_std = sample.log_std[dim];
            logp += gaussian_log_prob(sample.z[dim], mean, log_std);
        }
        assert!((logp - sample.logp).abs() < 1e-5);
        let _ = outputs;
    }

    #[test]
    fn sampling_scores_the_bounded_distribution_it_actually_uses() {
        let outputs = [0.0, 0.0, -20.0, 20.0, 1.0];
        let sample = Policy::sample_with_eps(&outputs, [0.5, -0.5]);
        assert!(sample.log_std[0] >= crate::policy::LOG_STD_MIN);
        assert!(sample.log_std[0] <= crate::policy::LOG_STD_MIN + 1e-6);
        assert!(sample.log_std[1] <= crate::policy::LOG_STD_MAX);
        assert!(sample.log_std[1] >= crate::policy::LOG_STD_MAX - 1e-6);
        assert_eq!(sample.log_std[0], log_std_from_raw(-20.0));
        assert_eq!(sample.log_std[1], log_std_from_raw(20.0));
        let expected = gaussian_log_prob(sample.z[0], 0.0, sample.log_std[0])
            + gaussian_log_prob(sample.z[1], 0.0, sample.log_std[1]);
        assert!(
            (sample.logp - expected).abs() < 1e-6,
            "stored log-probability must use the bounded sigma"
        );
        assert!(crate::policy::log_std_raw_grad(-20.0) > 0.0);
        assert!(crate::policy::log_std_raw_grad(20.0) > 0.0);
    }

    #[test]
    fn update_raises_the_probability_of_advantageous_actions() {
        let mut policy = Policy::new(4, 16, 2, 21);
        let mut optimizer = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
        let good_obs: Vec<f32> = vec![0.1, -0.2, 0.3, 0.05];
        let bad_obs: Vec<f32> = vec![-0.4, 0.2, -0.1, 0.3];
        let mut rng = Rng::new(5);
        let good = policy.sample(&good_obs, &mut rng);
        let bad = policy.sample(&bad_obs, &mut rng);
        let mut batch = Batch::new(4, ACTION_DIM);
        for _ in 0..16 {
            batch.push_row(&good_obs, &good.z, good.logp, good.value, good.value, 1.0);
            batch.push_row(&bad_obs, &bad.z, bad.logp, bad.value, bad.value, -1.0);
        }
        update(&mut policy, &mut optimizer, &batch).unwrap();
        let logp = |policy: &Policy, obs: &[f32], z: &[f32; ACTION_DIM]| -> f32 {
            let out = policy.forward(obs, 1);
            let mut total = 0.0;
            for dim in 0..ACTION_DIM {
                total +=
                    gaussian_log_prob(z[dim], out[dim], log_std_from_raw(out[ACTION_DIM + dim]));
            }
            total
        };
        assert!(
            logp(&policy, &good_obs, &good.z) > good.logp,
            "advantageous action lost probability"
        );
        assert!(
            logp(&policy, &bad_obs, &bad.z) < bad.logp,
            "disadvantageous action gained probability"
        );
    }

    #[test]
    fn update_changes_parameters_and_reports_finite_statistics() {
        let mut policy = Policy::new(6, 16, 2, 11);
        let mut optimizer = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
        let mut batch = Batch::new(6, ACTION_DIM);
        let mut rng = Rng::new(4);
        for _ in 0..64 {
            let obs: Vec<f32> = (0..6).map(|_| rng.range(-1.0, 1.0) as f32).collect();
            let sample = policy.sample(&obs, &mut rng);
            batch.push_row(&obs, &sample.z, sample.logp, sample.value, 0.5, 1.0);
        }
        let before = policy.params().to_vec();
        let stats = update(&mut policy, &mut optimizer, &batch).unwrap();
        assert!(stats.policy_kl.is_finite());
        assert!(stats.gradient_norm.is_finite());
        assert!(stats.value_loss.is_finite());
        assert_ne!(policy.params(), before.as_slice());
    }
}
