//! Normalized residual MLP with an explicit forward/backward pass.
//!
//! The trunk is a linear input projection followed by residual blocks of
//! `x + leaky(dense(layer_norm(x)))`, closed by a layer norm. The output layer
//! exposes the policy means, a learned per-state log standard deviation and a
//! state value, so the actor, exploration and critic share one trunk while
//! keeping independent heads. No autograd dependency: the network is a flat
//! list of `f32` arrays, so a finite-difference gradient check costs nothing.

use neural_racing_core::Rng;

pub const LEAKY_SLOPE: f32 = 0.05;
/// Width and depth of the shipped racing brain. 349,749 parameters with the
/// 67-value observation and two actions.
pub const DEFAULT_WIDTH: usize = 256;
pub const DEFAULT_BLOCKS: usize = 5;
/// The policy is always steer and throttle plus brake.
pub const ACTION_DIM: usize = 2;
/// Exploration standard deviation is bounded so the policy can neither freeze
/// nor random-walk. These are log-space bounds (0.15 and 0.6).
pub const LOG_STD_MIN: f32 = -1.897_120_0;
pub const LOG_STD_MAX: f32 = -0.510_825_6;

const MAGIC: &[u8; 8] = b"NRPOLCY\0";
const VERSION: u32 = 2;
const NORM_EPSILON: f64 = 1e-5;

/// One sampled control decision and everything the estimator needs to train it.
#[derive(Clone, Copy, Debug)]
pub struct Sample {
    pub action: [f32; ACTION_DIM],
    /// Pre-tanh draw `mean + sigma * eps`. The estimator needs this unchanged.
    pub z: [f32; ACTION_DIM],
    pub mean: [f32; ACTION_DIM],
    pub log_std: [f32; ACTION_DIM],
    pub eps: [f32; ACTION_DIM],
    pub logp: f32,
    pub value: f32,
}

/// Log density of `z` under `Normal(mean, exp(log_std))`.
pub fn gaussian_log_prob(z: f32, mean: f32, log_std: f32) -> f32 {
    let inv = (-log_std).exp();
    let diff = (z - mean) * inv;
    -0.5 * diff * diff - log_std - 0.918_938_5
}

/// Per-layer buffers needed to differentiate one batch.
pub struct ForwardCache {
    pub rows: usize,
    /// Input to each dense layer, in forward order (`rows * in_dim`).
    pub dense_inputs: Vec<Vec<f32>>,
    /// Pre-activation of each dense layer (`rows * out_dim`).
    pub dense_pre: Vec<Vec<f32>>,
    /// Normalized input to each dense layer that follows a layer norm.
    pub norm_xhat: Vec<Vec<f32>>,
    /// Per-row inverse standard deviation for each layer norm.
    pub norm_inv_std: Vec<Vec<f32>>,
}

pub struct Policy {
    pub obs_dim: usize,
    pub width: usize,
    pub blocks: usize,
    pub act_dim: usize,
    /// Flattened layer parameters in the layout described by [`Policy::new`].
    params: Vec<Vec<f32>>,
}

fn dot(a: &[f32], b: &[f32]) -> f32 {
    let mut sums = [0.0f32; 8];
    let mut aa = a.chunks_exact(8);
    let mut bb = b.chunks_exact(8);
    for (x, w) in aa.by_ref().zip(bb.by_ref()) {
        for i in 0..8 {
            sums[i] += x[i] * w[i];
        }
    }
    let mut result: f32 = sums.into_iter().sum();
    for (x, w) in aa.remainder().iter().zip(bb.remainder()) {
        result += x * w;
    }
    result
}

fn dense_forward(
    input: &[f32],
    weight: &[f32],
    bias: &[f32],
    rows: usize,
    in_dim: usize,
    out_dim: usize,
) -> Vec<f32> {
    let mut pre = if rows > 1 {
        crate::matrix::product(input, weight, rows, out_dim, in_dim, false, true)
    } else if rows == 0 {
        Vec::new()
    } else {
        (0..out_dim)
            .map(|unit| dot(input, &weight[unit * in_dim..(unit + 1) * in_dim]))
            .collect()
    };
    for row in pre.chunks_exact_mut(out_dim) {
        for (value, bias) in row.iter_mut().zip(bias) {
            *value += bias;
        }
    }
    pre
}

/// `x = (h - mean) / sqrt(var + eps)`, `y = x * gamma + beta`.
fn norm_forward(
    input: &[f32],
    rows: usize,
    width: usize,
    gamma: &[f32],
    beta: &[f32],
    xhat: &mut Vec<f32>,
    inv_std: &mut Vec<f32>,
) -> Vec<f32> {
    xhat.clear();
    xhat.resize(rows * width, 0.0);
    inv_std.clear();
    inv_std.resize(rows, 0.0);
    let mut out = vec![0f32; rows * width];
    for row in 0..rows {
        let slice = &input[row * width..(row + 1) * width];
        let mean = slice.iter().map(|v| f64::from(*v)).sum::<f64>() / width as f64;
        let var = slice
            .iter()
            .map(|v| {
                let d = f64::from(*v) - mean;
                d * d
            })
            .sum::<f64>()
            / width as f64;
        let inv = 1.0 / (var + NORM_EPSILON).sqrt();
        inv_std[row] = inv as f32;
        for unit in 0..width {
            let x = (f64::from(slice[unit]) - mean) * inv;
            xhat[row * width + unit] = x as f32;
            out[row * width + unit] = (x * f64::from(gamma[unit]) + f64::from(beta[unit])) as f32;
        }
    }
    out
}

fn norm_backward(
    d_output: &[f32],
    xhat: &[f32],
    inv_std: &[f32],
    gamma: &[f32],
    rows: usize,
    width: usize,
    d_gamma: &mut [f32],
    d_beta: &mut [f32],
    d_input: &mut [f32],
) {
    for row in 0..rows {
        let base = row * width;
        let inv = f64::from(inv_std[row]);
        let mut sum_dy = 0f64;
        let mut sum_dy_xhat = 0f64;
        for unit in 0..width {
            let dy = f64::from(d_output[base + unit]) * f64::from(gamma[unit]);
            d_gamma[unit] +=
                (f64::from(d_output[base + unit]) * f64::from(xhat[base + unit])) as f32;
            d_beta[unit] += d_output[base + unit];
            sum_dy += dy;
            sum_dy_xhat += dy * f64::from(xhat[base + unit]);
        }
        let mean_dy = sum_dy / width as f64;
        let mean_dy_xhat = sum_dy_xhat / width as f64;
        for unit in 0..width {
            let dy = f64::from(d_output[base + unit]) * f64::from(gamma[unit]);
            d_input[base + unit] =
                (inv * (dy - mean_dy - f64::from(xhat[base + unit]) * mean_dy_xhat)) as f32;
        }
    }
}

fn leaky(value: f32) -> f32 {
    if value < 0.0 {
        value * LEAKY_SLOPE
    } else {
        value
    }
}

fn pair(items: &mut [Vec<f32>]) -> (&mut Vec<f32>, &mut Vec<f32>) {
    let (first, rest) = items.split_at_mut(1);
    (&mut first[0], &mut rest[0])
}

/// Bounded standard deviation parameterization. `raw` is unconstrained and
/// maps smoothly into `(LOG_STD_MIN, LOG_STD_MAX)`, so the sampling distribution
/// and the update always agree and the head can never leave the valid range.
pub fn log_std_from_raw(raw: f32) -> f32 {
    let x = f64::from(raw);
    let sigmoid = 1.0 / (1.0 + (-x).exp());
    ((f64::from(LOG_STD_MIN) + f64::from(LOG_STD_MAX - LOG_STD_MIN) * sigmoid) as f32)
        .clamp(LOG_STD_MIN, LOG_STD_MAX)
}

/// Derivative of [`log_std_from_raw`] with respect to `raw`.
pub fn log_std_raw_grad(raw: f32) -> f32 {
    let x = f64::from(raw);
    let sigmoid = 1.0 / (1.0 + (-x).exp());
    ((LOG_STD_MAX - LOG_STD_MIN) as f64 * sigmoid * (1.0 - sigmoid)) as f32
}

impl Policy {
    /// Parameter layout:
    /// `[W, b]` for the input projection, then for every residual block
    /// `[gamma, beta, W, b]`, then `[gamma, beta, W, b]` for the output.
    pub fn new(obs_dim: usize, width: usize, blocks: usize, seed: u64) -> Self {
        assert!(obs_dim > 0, "the policy needs an input");
        assert!(width >= 8, "the trunk needs at least eight units");
        assert!(blocks >= 1, "the trunk needs at least one residual block");
        let mut rng = Rng::new(seed ^ 0x9e37_79b9_7f4a_7c15);
        let out_dim = ACTION_DIM * 2 + 1;
        let mut params = Vec::with_capacity(6 + blocks * 4);
        let dense = |rng: &mut Rng, in_dim: usize, out_dim: usize, scale: f64| {
            let mut weight = Vec::with_capacity(in_dim * out_dim);
            for _ in 0..in_dim * out_dim {
                weight.push((rng.normal() * scale) as f32);
            }
            (weight, vec![0f32; out_dim])
        };
        let (weight, bias) = dense(&mut rng, obs_dim, width, (2.0 / obs_dim as f64).sqrt());
        params.push(weight);
        params.push(bias);
        for _ in 0..blocks {
            params.push(vec![1f32; width]);
            params.push(vec![0f32; width]);
            let (weight, bias) = dense(&mut rng, width, width, (1.0 / width as f64).sqrt());
            params.push(weight);
            params.push(bias);
        }
        params.push(vec![1f32; width]);
        params.push(vec![0f32; width]);
        let (weight, bias) = dense(&mut rng, width, out_dim, 0.01);
        params.push(weight);
        params.push(bias);
        // Fresh policy gently rolls forward: mean throttle 0.55, sigma 0.30,
        // value zero. The randomness above only shapes the residual correction.
        let bias = params.last_mut().expect("output bias");
        bias[1] = 0.55f64.atanh() as f32;
        // Logistic inverse for sigma 0.30 inside the bounded log-std range.
        let target = (0.30f64.ln() - f64::from(LOG_STD_MIN)) / f64::from(LOG_STD_MAX - LOG_STD_MIN);
        bias[2] = (target / (1.0 - target)).ln() as f32;
        bias[3] = bias[2];
        Self {
            obs_dim,
            width,
            blocks,
            act_dim: ACTION_DIM,
            params,
        }
    }

    pub fn params(&self) -> &[Vec<f32>] {
        &self.params
    }

    pub fn params_mut(&mut self) -> &mut [Vec<f32>] {
        &mut self.params
    }

    pub fn num_params(&self) -> usize {
        self.params.iter().map(Vec::len).sum()
    }

    fn dense_slots(&self, dense: usize) -> (usize, usize) {
        let index = if dense == 0 {
            0
        } else if dense == self.blocks + 1 {
            4 * self.blocks + 4
        } else {
            4 * dense
        };
        (index, index + 1)
    }

    fn norm_slots(&self, norm: usize) -> (usize, usize) {
        debug_assert!(norm <= self.blocks);
        if norm == self.blocks {
            (4 * self.blocks + 2, 4 * self.blocks + 3)
        } else {
            (4 * norm + 2, 4 * norm + 3)
        }
    }

    /// Raw outputs: `[mean; log_std; value]`, five values per row.
    pub fn outputs_per_row(&self) -> usize {
        self.act_dim * 2 + 1
    }

    pub fn forward(&self, obs: &[f32], rows: usize) -> Vec<f32> {
        self.forward_inner(obs, rows, None)
    }

    pub fn forward_cached(&self, obs: &[f32], rows: usize) -> (Vec<f32>, ForwardCache) {
        let mut cache = ForwardCache {
            rows,
            dense_inputs: Vec::with_capacity(self.blocks + 2),
            dense_pre: Vec::with_capacity(self.blocks + 2),
            norm_xhat: Vec::with_capacity(self.blocks + 1),
            norm_inv_std: Vec::with_capacity(self.blocks + 1),
        };
        let out = self.forward_inner(obs, rows, Some(&mut cache));
        (out, cache)
    }

    /// Gradient of `sum(output * d_output)` with respect to every parameter.
    pub fn backward(&self, cache: &ForwardCache, d_output: &[f32]) -> Vec<Vec<f32>> {
        let out_dim = self.outputs_per_row();
        assert_eq!(cache.rows * out_dim, d_output.len());
        assert_eq!(cache.dense_inputs.len(), self.blocks + 2);
        assert_eq!(cache.norm_xhat.len(), self.blocks + 1);
        let rows = cache.rows;
        let width = self.width;
        let mut grads: Vec<Vec<f32>> = self.params.iter().map(|p| vec![0f32; p.len()]).collect();
        let mut d = d_output.to_vec();

        // Output layer slots are contiguous norm then dense parameters.
        let out_base = 4 * self.blocks + 2;
        let out_dim_wide = out_dim;
        {
            let weight = &self.params[out_base + 2];
            let tail = &mut grads[out_base..out_base + 4];
            let (norm, dense_part) = tail.split_at_mut(2);
            let (d_gamma, d_beta) = pair(norm);
            let (d_weight, d_bias) = pair(dense_part);
            *d_weight = crate::matrix::product(
                &d,
                &cache.dense_inputs[self.blocks + 1],
                out_dim_wide,
                width,
                rows,
                true,
                false,
            );
            for row in d.chunks_exact(out_dim_wide) {
                for (slot, g) in d_bias.iter_mut().zip(row) {
                    *slot += g;
                }
            }
            let d_input =
                crate::matrix::product(&d, weight, rows, width, out_dim_wide, false, false);
            let mut d_hidden = vec![0f32; rows * width];
            norm_backward(
                &d_input,
                &cache.norm_xhat[self.blocks],
                &cache.norm_inv_std[self.blocks],
                &self.params[self.norm_slots(self.blocks).0],
                rows,
                width,
                d_gamma,
                d_beta,
                &mut d_hidden,
            );
            d = d_hidden;
        }

        // Residual blocks, last first. `y = x + leaky(pre)`.
        for block in (0..self.blocks).rev() {
            let base = 4 * block + 2;
            let weight = &self.params[base + 2];
            let mut d_pre = d.clone();
            for (slot, pre) in d_pre.iter_mut().zip(&cache.dense_pre[block + 1]) {
                if *pre < 0.0 {
                    *slot *= LEAKY_SLOPE;
                }
            }
            let tail = &mut grads[base..base + 4];
            let (norm, dense_part) = tail.split_at_mut(2);
            let (d_gamma, d_beta) = pair(norm);
            let (d_weight, d_bias) = pair(dense_part);
            *d_weight = crate::matrix::product(
                &d_pre,
                &cache.dense_inputs[block + 1],
                width,
                width,
                rows,
                true,
                false,
            );
            for row in d_pre.chunks_exact(width) {
                for (slot, g) in d_bias.iter_mut().zip(row) {
                    *slot += g;
                }
            }
            let d_norm = crate::matrix::product(&d_pre, weight, rows, width, width, false, false);
            let mut d_residual = vec![0f32; rows * width];
            norm_backward(
                &d_norm,
                &cache.norm_xhat[block],
                &cache.norm_inv_std[block],
                &self.params[base],
                rows,
                width,
                d_gamma,
                d_beta,
                &mut d_residual,
            );
            for (slot, skip) in d_residual.iter_mut().zip(&d) {
                *slot += skip;
            }
            d = d_residual;
        }

        // Input projection.
        let mut d_pre = d;
        for (slot, pre) in d_pre.iter_mut().zip(&cache.dense_pre[0]) {
            if *pre < 0.0 {
                *slot *= LEAKY_SLOPE;
            }
        }
        {
            let (d_weight, d_bias) = pair(&mut grads[0..2]);
            *d_weight = crate::matrix::product(
                &d_pre,
                &cache.dense_inputs[0],
                width,
                self.obs_dim,
                rows,
                true,
                false,
            );
            for row in d_pre.chunks_exact(width) {
                for (slot, g) in d_bias.iter_mut().zip(row) {
                    *slot += g;
                }
            }
        }
        grads
    }

    /// Splits raw outputs into a trainable sample. `eps` is the standard-normal
    /// draw in pre-tanh space; the estimator needs it unchanged.
    pub fn sample_from_outputs(outputs: &[f32], rng: &mut Rng) -> Sample {
        let mut eps = [0f32; ACTION_DIM];
        rng.normals_f32(&mut eps);
        Self::sample_with_eps(outputs, eps)
    }

    pub fn sample_with_eps(outputs: &[f32], eps: [f32; ACTION_DIM]) -> Sample {
        let mut mean = [0f32; ACTION_DIM];
        let mut log_std = [0f32; ACTION_DIM];
        let mut action = [0f32; ACTION_DIM];
        let mut z = [0f32; ACTION_DIM];
        let mut logp = 0f32;
        for dim in 0..ACTION_DIM {
            mean[dim] = outputs[dim];
            log_std[dim] = log_std_from_raw(outputs[ACTION_DIM + dim]);
            z[dim] =
                (f64::from(mean[dim]) + f64::from(log_std[dim].exp()) * f64::from(eps[dim])) as f32;
            action[dim] = f64::from(z[dim]).tanh().clamp(-1.0, 1.0) as f32;
            logp += gaussian_log_prob(z[dim], mean[dim], log_std[dim]);
        }
        Sample {
            action,
            z,
            mean,
            log_std,
            eps,
            logp,
            value: outputs[ACTION_DIM * 2],
        }
    }

    pub fn sample(&self, obs: &[f32], rng: &mut Rng) -> Sample {
        let outputs = self.forward(obs, 1);
        Self::sample_from_outputs(&outputs, rng)
    }

    /// Deterministic mean action: `tanh(mean)`, no exploration noise.
    pub fn mean_action(&self, obs: &[f32]) -> [f32; ACTION_DIM] {
        let outputs = self.forward(obs, 1);
        let mut action = [0f32; ACTION_DIM];
        for (dim, slot) in action.iter_mut().enumerate() {
            *slot = f64::from(outputs[dim]).tanh() as f32;
        }
        action
    }

    pub fn value(&self, obs: &[f32]) -> f32 {
        self.forward(obs, 1)[ACTION_DIM * 2]
    }

    /// Post-activation values of the input, every residual block and the final
    /// normalized trunk, for the UI brain view. The output head is excluded
    /// because means, log sigma and value do not share units.
    pub fn activations(&self, obs: &[f32]) -> Vec<Vec<f32>> {
        let mut layers = Vec::with_capacity(self.blocks + 3);
        let input = obs[..self.obs_dim].to_vec();
        layers.push(input.clone());
        let (w_slot, b_slot) = self.dense_slots(0);
        let mut h = vec![0f32; self.width];
        for unit in 0..self.width {
            let row = &self.params[w_slot][unit * self.obs_dim..(unit + 1) * self.obs_dim];
            h[unit] = leaky(dot(&input, row) + self.params[b_slot][unit]);
        }
        layers.push(h.clone());
        for block in 0..self.blocks {
            let (gamma_slot, beta_slot) = self.norm_slots(block);
            let normalized = self.normalize(
                &h,
                1,
                self.params[gamma_slot].as_slice(),
                self.params[beta_slot].as_slice(),
            );
            let (w_slot, b_slot) = self.dense_slots(block + 1);
            let mut next = h.clone();
            for unit in 0..self.width {
                let row = &self.params[w_slot][unit * self.width..(unit + 1) * self.width];
                next[unit] += leaky(dot(&normalized, row) + self.params[b_slot][unit]);
            }
            h = next;
            layers.push(h.clone());
        }
        let (gamma_slot, beta_slot) = self.norm_slots(self.blocks);
        layers.push(self.normalize(
            &h,
            1,
            self.params[gamma_slot].as_slice(),
            self.params[beta_slot].as_slice(),
        ));
        layers
    }

    fn normalize(&self, input: &[f32], rows: usize, gamma: &[f32], beta: &[f32]) -> Vec<f32> {
        let mut xhat = Vec::new();
        let mut inv_std = Vec::new();
        norm_forward(
            input,
            rows,
            self.width,
            gamma,
            beta,
            &mut xhat,
            &mut inv_std,
        )
    }

    fn forward_inner(
        &self,
        obs: &[f32],
        rows: usize,
        mut cache: Option<&mut ForwardCache>,
    ) -> Vec<f32> {
        assert_eq!(obs.len(), rows * self.obs_dim, "observation batch shape");
        let width = self.width;
        let out_dim = self.outputs_per_row();
        let mut h = obs.to_vec();

        let (w_slot, b_slot) = self.dense_slots(0);
        let pre = dense_forward(
            &h,
            &self.params[w_slot],
            &self.params[b_slot],
            rows,
            self.obs_dim,
            width,
        );
        let mut hidden = pre.clone();
        for value in &mut hidden {
            *value = leaky(*value);
        }
        if let Some(cache) = cache.as_deref_mut() {
            cache.dense_inputs.push(h.clone());
            cache.dense_pre.push(pre);
        }
        h = hidden;

        for block in 0..self.blocks {
            let (gamma_slot, beta_slot) = self.norm_slots(block);
            let mut xhat = Vec::new();
            let mut inv_std = Vec::new();
            let normalized = norm_forward(
                &h,
                rows,
                width,
                &self.params[gamma_slot],
                &self.params[beta_slot],
                &mut xhat,
                &mut inv_std,
            );
            let (w_slot, b_slot) = self.dense_slots(block + 1);
            let pre = dense_forward(
                &normalized,
                &self.params[w_slot],
                &self.params[b_slot],
                rows,
                width,
                width,
            );
            let mut next = h.clone();
            for (slot, value) in next.iter_mut().zip(&pre) {
                *slot += leaky(*value);
            }
            if let Some(cache) = cache.as_deref_mut() {
                cache.dense_inputs.push(normalized);
                cache.dense_pre.push(pre);
                cache.norm_xhat.push(xhat);
                cache.norm_inv_std.push(inv_std);
            }
            h = next;
        }

        let (gamma_slot, beta_slot) = self.norm_slots(self.blocks);
        let mut xhat = Vec::new();
        let mut inv_std = Vec::new();
        let normalized = norm_forward(
            &h,
            rows,
            width,
            &self.params[gamma_slot],
            &self.params[beta_slot],
            &mut xhat,
            &mut inv_std,
        );
        let (w_slot, b_slot) = self.dense_slots(self.blocks + 1);
        let output = dense_forward(
            &normalized,
            &self.params[w_slot],
            &self.params[b_slot],
            rows,
            width,
            out_dim,
        );
        if let Some(cache) = cache.as_deref_mut() {
            cache.dense_inputs.push(normalized);
            cache.dense_pre.push(output.clone());
            cache.norm_xhat.push(xhat);
            cache.norm_inv_std.push(inv_std);
        }
        output
    }

    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        let op = neural_racing_diagnostics::operation();
        neural_racing_diagnostics::emit!(
            "policy_save_started",
            op,
            neural_racing_diagnostics::json!({
            "parent":neural_racing_diagnostics::current_operation(), "parameters":self.num_params(), "obs_dim":self.obs_dim})
        );
        let started = std::time::Instant::now();
        let _context = neural_racing_diagnostics::context(op);
        let result = self.save_inner(path);
        neural_racing_diagnostics::emit!(
            "policy_save_finished",
            op,
            neural_racing_diagnostics::json!({
            "ok":result.is_ok(), "error_kind":result.as_ref().err().map(|e| format!("{:?}",e.kind())),
            "duration_ms":started.elapsed().as_millis() as u64})
        );
        result
    }

    fn save_inner(&self, path: &std::path::Path) -> std::io::Result<()> {
        let mut bytes = Vec::with_capacity(64 + self.num_params() * 4);
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&VERSION.to_le_bytes());
        bytes.extend_from_slice(&(self.obs_dim as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.act_dim as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.width as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.blocks as u32).to_le_bytes());
        for param in &self.params {
            bytes.extend_from_slice(&(param.len() as u32).to_le_bytes());
            for value in param {
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        }
        neural_racing_diagnostics::emit!(
            "policy_bytes",
            neural_racing_diagnostics::current_operation(),
            neural_racing_diagnostics::json!({"bytes":bytes.len(),"content_id":neural_racing_diagnostics::fingerprint(&bytes)})
        );
        std::fs::write(path, bytes)
    }

    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        let op = neural_racing_diagnostics::operation();
        neural_racing_diagnostics::emit!(
            "policy_load_started",
            op,
            neural_racing_diagnostics::json!({
            "parent":neural_racing_diagnostics::current_operation()})
        );
        let started = std::time::Instant::now();
        let _context = neural_racing_diagnostics::context(op);
        let result = Self::load_inner(path);
        // Only code-defined categories cross the diagnostic boundary; never raw I/O text.
        let reason = result
            .as_ref()
            .err()
            .map(|error| match error.to_string().as_str() {
                "not a policy file" => "bad_magic",
                "unsupported policy version: retrain from scratch" => "unsupported_version",
                "truncated policy file" => "truncated",
                "policy action dimension is not steer/throttle" => "action_dimension",
                "policy layer shape does not match its header" => "layer_shape",
                "trailing bytes after policy parameters" => "trailing_bytes",
                _ => "io_error",
            });
        neural_racing_diagnostics::emit!(
            "policy_load_finished",
            op,
            neural_racing_diagnostics::json!({
            "ok":result.is_ok(), "error_kind":result.as_ref().err().map(|e| format!("{:?}",e.kind())),
            "reason":reason,
            "obs_dim":result.as_ref().ok().map(|p|p.obs_dim),
            "duration_ms":started.elapsed().as_millis() as u64})
        );
        result
    }

    fn load_inner(path: &std::path::Path) -> std::io::Result<Self> {
        let bytes = std::fs::read(path)?;
        neural_racing_diagnostics::emit!(
            "policy_bytes",
            neural_racing_diagnostics::current_operation(),
            neural_racing_diagnostics::json!({"bytes":bytes.len(),"content_id":neural_racing_diagnostics::fingerprint(&bytes)})
        );
        let mut cursor = 0usize;
        let take = |cursor: &mut usize, count: usize| -> std::io::Result<&[u8]> {
            if *cursor + count > bytes.len() {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::UnexpectedEof,
                    "truncated policy file",
                ));
            }
            let slice = &bytes[*cursor..*cursor + count];
            *cursor += count;
            Ok(slice)
        };
        let u32_at = |slice: &[u8]| u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]);
        let invalid = |message: &str| {
            std::io::Error::new(std::io::ErrorKind::InvalidData, message.to_string())
        };

        if take(&mut cursor, 8)? != MAGIC {
            return Err(invalid("not a policy file"));
        }
        if u32_at(take(&mut cursor, 4)?) != VERSION {
            return Err(invalid("unsupported policy version: retrain from scratch"));
        }
        let obs_dim = u32_at(take(&mut cursor, 4)?) as usize;
        let act_dim = u32_at(take(&mut cursor, 4)?) as usize;
        let width = u32_at(take(&mut cursor, 4)?) as usize;
        let blocks = u32_at(take(&mut cursor, 4)?) as usize;
        if act_dim != ACTION_DIM {
            return Err(invalid("policy action dimension is not steer/throttle"));
        }
        let mut params = Vec::with_capacity(6 + blocks * 4);
        let read_vector = |cursor: &mut usize, expected: usize| -> std::io::Result<Vec<f32>> {
            let count = u32_at(take(cursor, 4)?) as usize;
            if count != expected {
                return Err(invalid("policy layer shape does not match its header"));
            }
            let raw = take(cursor, count * 4)?;
            Ok(raw
                .chunks_exact(4)
                .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                .collect())
        };
        // Input projection.
        params.push(read_vector(&mut cursor, obs_dim * width)?);
        params.push(read_vector(&mut cursor, width)?);
        // Residual blocks in file order: gamma, beta, weight, bias.
        for _ in 0..blocks {
            params.push(read_vector(&mut cursor, width)?);
            params.push(read_vector(&mut cursor, width)?);
            params.push(read_vector(&mut cursor, width * width)?);
            params.push(read_vector(&mut cursor, width)?);
        }
        // Final normalization, then the output head.
        params.push(read_vector(&mut cursor, width)?);
        params.push(read_vector(&mut cursor, width)?);
        params.push(read_vector(&mut cursor, width * (act_dim * 2 + 1))?);
        params.push(read_vector(&mut cursor, act_dim * 2 + 1)?);
        if cursor != bytes.len() {
            return Err(invalid("trailing bytes after policy parameters"));
        }
        Ok(Self {
            obs_dim,
            width,
            blocks,
            act_dim,
            params,
        })
    }
}

impl Clone for Policy {
    fn clone(&self) -> Self {
        Self {
            obs_dim: self.obs_dim,
            width: self.width,
            blocks: self.blocks,
            act_dim: self.act_dim,
            params: self.params.clone(),
        }
    }
}
