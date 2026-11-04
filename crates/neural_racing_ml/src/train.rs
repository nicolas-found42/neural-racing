//! REINFORCE update with a KL-limited backtracking line search.

use crate::adam::Adam;
use crate::policy::Policy;

pub const GAMMA: f64 = 0.995;
pub const LR: f64 = 7e-4;
pub const SIGMA_START: f64 = 0.40;
pub const SIGMA_END: f64 = 0.12;
pub const SIGMA_DECAY: f64 = 3500.0;
pub const MAX_POLICY_KL: f64 = 0.01;
pub const MAX_BACKTRACKS: usize = 12;

/// One episode batch flattened to contiguous rows.
pub struct Batch {
    pub rows: usize,
    pub obs_dim: usize,
    pub act_dim: usize,
    pub obs: Vec<f32>,
    pub eps: Vec<f32>,
    pub returns: Vec<f32>,
}

impl Batch {
    pub fn new(obs_dim: usize, act_dim: usize) -> Self {
        Self {
            rows: 0,
            obs_dim,
            act_dim,
            obs: Vec::new(),
            eps: Vec::new(),
            returns: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.rows = 0;
        self.obs.clear();
        self.eps.clear();
        self.returns.clear();
    }

    pub fn push_row(&mut self, obs: &[f32], eps: &[f32], ret: f32) {
        debug_assert_eq!(obs.len(), self.obs_dim);
        debug_assert_eq!(eps.len(), self.act_dim);
        self.obs.extend_from_slice(obs);
        self.eps.extend_from_slice(eps);
        self.returns.push(ret);
        self.rows += 1;
    }

    /// Merges another batch (one worker's episode) into this one.
    pub fn extend(&mut self, other: &Batch) {
        debug_assert_eq!(other.obs_dim, self.obs_dim);
        debug_assert_eq!(other.act_dim, self.act_dim);
        self.obs.extend_from_slice(&other.obs);
        self.eps.extend_from_slice(&other.eps);
        self.returns.extend_from_slice(&other.returns);
        self.rows += other.rows;
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UpdateStats {
    pub gradient_norm: f64,
    pub gradient_clip: f64,
    pub parameter_norm: f64,
    pub policy_kl: f64,
    pub update_scale: f64,
}

pub fn sigma_at(episode: i64) -> f64 {
    let frac = ((episode as f64 - 1.0) / SIGMA_DECAY).min(1.0).max(0.0);
    SIGMA_START + (SIGMA_END - SIGMA_START) * frac
}

/// Reward-to-go, discounted by [`GAMMA`] and written back in place.
pub fn discounted_returns(rewards: &[f32]) -> Vec<f32> {
    let mut returns = vec![0f32; rewards.len()];
    let mut running = 0f64;
    for index in (0..rewards.len()).rev() {
        running = f64::from(rewards[index]) + GAMMA * running;
        returns[index] = running as f32;
    }
    returns
}

pub fn update(
    policy: &mut Policy,
    optimizer: &mut Adam,
    batch: &Batch,
    sigma: f64,
    policy_sigma: Option<f64>,
    max_policy_kl: Option<f64>,
) -> Result<UpdateStats, &'static str> {
    if batch.rows == 0 {
        return Err("empty training batch");
    }
    if batch.obs.len() != batch.rows * policy.obs_dim
        || batch.eps.len() != batch.rows * policy.act_dim
        || batch.returns.len() != batch.rows
    {
        return Err("training batch does not match the policy shape");
    }
    let max_policy_kl = max_policy_kl.unwrap_or(MAX_POLICY_KL);
    let (adv_mean, adv_std) = mean_std(&batch.returns);
    let scale = adv_std + 1e-8;
    let rows = batch.rows;
    let act_dim = policy.act_dim;
    let mut dz = vec![0f32; rows * act_dim];
    for row in 0..rows {
        let advantage = (batch.returns[row] - adv_mean) / scale;
        for action in 0..act_dim {
            dz[row * act_dim + action] =
                advantage * batch.eps[row * act_dim + action] / sigma as f32;
        }
    }

    let (previous_means, cache) = policy.forward_cached(&batch.obs, rows);
    let raw_grads = policy.backward(&cache, &dz);
    let rows_f = rows as f32;
    let grads: Vec<Vec<f32>> = raw_grads
        .iter()
        .map(|layer| layer.iter().map(|g| -g / rows_f).collect())
        .collect();
    let gradient_norm = l2_norm(&grads);
    if !gradient_norm.is_finite() {
        return Err("non-finite policy gradient");
    }
    let gradient_clip = (optimizer.max_grad_norm() / (gradient_norm + 1e-8)).min(1.0);

    let mut policy_kl = 0.0;
    let mut update_scale = 1.0;
    if let Some(policy_sigma) = policy_sigma {
        let old: Vec<Vec<f64>> = policy
            .params()
            .iter()
            .map(|layer| layer.iter().map(|v| f64::from(*v)).collect())
            .collect();
        let saved = optimizer.snapshot();
        optimizer.step(policy.params_mut(), &grads);
        let delta: Vec<Vec<f64>> = policy
            .params()
            .iter()
            .zip(&old)
            .map(|(layer, before)| {
                layer
                    .iter()
                    .zip(before)
                    .map(|(after, before)| f64::from(*after) - before)
                    .collect()
            })
            .collect();
        let mut accepted = false;
        for _ in 0..=MAX_BACKTRACKS {
            set_blended(policy, &old, &delta, update_scale);
            let next_means = policy.forward(&batch.obs, rows);
            policy_kl = mean_squared_kl(&previous_means, &next_means, act_dim, policy_sigma);
            if policy_kl.is_finite() && policy_kl <= max_policy_kl {
                accepted = true;
                break;
            }
            update_scale *= 0.5;
        }
        if !accepted {
            set_blended(policy, &old, &delta, 0.0);
            optimizer.restore(saved);
            update_scale = 0.0;
            policy_kl = 0.0;
        }
    } else {
        optimizer.step(policy.params_mut(), &grads);
    }

    let parameter_norm = l2_norm(policy.params());
    if !parameter_norm.is_finite() {
        return Err("non-finite policy parameters");
    }
    Ok(UpdateStats {
        gradient_norm,
        gradient_clip,
        parameter_norm,
        policy_kl,
        update_scale,
    })
}

fn set_blended(policy: &mut Policy, old: &[Vec<f64>], delta: &[Vec<f64>], scale: f64) {
    for ((layer, before), step) in policy.params_mut().iter_mut().zip(old).zip(delta) {
        for ((value, before), step) in layer.iter_mut().zip(before).zip(step) {
            *value = (before + step * scale) as f32;
        }
    }
}

fn mean_std(values: &[f32]) -> (f32, f32) {
    let n = values.len() as f32;
    let mean = values.iter().sum::<f32>() / n;
    let var = values
        .iter()
        .map(|value| (value - mean) * (value - mean))
        .sum::<f32>()
        / n;
    (mean, var.sqrt())
}

fn mean_squared_kl(previous: &[f32], next: &[f32], width: usize, sigma: f64) -> f64 {
    let rows = previous.len() / width;
    let mut total = 0f64;
    for row in 0..rows {
        let mut row_sum = 0f64;
        for column in 0..width {
            let index = row * width + column;
            let diff = f64::from(next[index]) - f64::from(previous[index]);
            row_sum += diff * diff;
        }
        total += row_sum;
    }
    let mean = total / rows as f64;
    mean / (2.0 * sigma * sigma)
}

fn l2_norm(layers: &[Vec<f32>]) -> f64 {
    layers
        .iter()
        .flat_map(|layer| layer.iter())
        .map(|value| f64::from(*value) * f64::from(*value))
        .sum::<f64>()
        .sqrt()
}
