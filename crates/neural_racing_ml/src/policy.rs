//! Residual MLP with an explicit backward pass.

use neural_racing_core::Rng;

pub const LEAKY_SLOPE: f32 = 0.05;
/// Four residual blocks of equal width: 21k parameters, fast on one core.
pub const DEFAULT_HIDDEN_DIMS: [usize; 4] = [96, 96, 96, 96];

const MAGIC: &[u8; 8] = b"NRPOLCY\0";
const VERSION: u32 = 1;

/// Per-layer buffers needed to differentiate one batch.
pub struct ForwardCache {
    pub rows: usize,
    /// Layer input, `rows * in_dim`.
    pub inputs: Vec<Vec<f32>>,
    /// Pre-activation, `rows * out_dim`.
    pub pre: Vec<Vec<f32>>,
}

/// Residual MLP: `h = leaky(W h + b) + h` where widths match, plain affine
/// output layer last. Weights of one layer are stored `out_dim * in_dim` with
/// the input dimension contiguous, so a forward pass is a row of dot products.
pub struct Policy {
    pub obs_dim: usize,
    pub hidden_dims: Vec<usize>,
    pub act_dim: usize,
    dims: Vec<usize>,
    /// `[W, b]` per layer, in forward order.
    params: Vec<Vec<f32>>,
}

impl Policy {
    pub fn new(obs_dim: usize, hidden_dims: &[usize], act_dim: usize, seed: u64) -> Self {
        assert!(obs_dim > 0, "the policy needs an input");
        assert!(act_dim >= 2, "the output bias writes a throttle prior at index 1");
        let mut dims = Vec::with_capacity(hidden_dims.len() + 2);
        dims.push(obs_dim);
        dims.extend_from_slice(hidden_dims);
        dims.push(act_dim);

        let mut rng = Rng::new(seed ^ 0x9e37_79b9_7f4a_7c15);
        let n_layers = dims.len() - 1;
        let mut params = Vec::with_capacity(n_layers * 2);
        for layer in 0..n_layers {
            let in_d = dims[layer];
            let out_d = dims[layer + 1];
            let last = layer + 1 == n_layers;
            let mut weight = Vec::with_capacity(in_d * out_d);
            if last {
                // The final layer starts almost silent: the throttle prior sits
                // in the bias so every fresh policy rolls forward.
                for _ in 0..in_d * out_d {
                    weight.push(rng.range(-0.01, 0.01) as f32);
                }
            } else {
                let scale = if in_d == out_d {
                    (1.0 / in_d as f64).sqrt()
                } else {
                    (2.0 / in_d as f64).sqrt()
                };
                for _ in 0..in_d * out_d {
                    weight.push((rng.normal() * scale) as f32);
                }
            }
            let mut bias = vec![0f32; out_d];
            if last {
                bias[1] = (0.55f64).atanh() as f32;
            }
            params.push(weight);
            params.push(bias);
        }
        Self {
            obs_dim,
            hidden_dims: hidden_dims.to_vec(),
            act_dim,
            dims,
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

    /// Raw action means for `rows` observations (`rows * obs_dim` values).
    pub fn forward(&self, obs: &[f32], rows: usize) -> Vec<f32> {
        self.forward_inner(obs, rows, None)
    }

    /// Action means plus the buffers needed by [`Policy::backward`].
    pub fn forward_cached(&self, obs: &[f32], rows: usize) -> (Vec<f32>, ForwardCache) {
        let mut cache = ForwardCache {
            rows,
            inputs: Vec::new(),
            pre: Vec::new(),
        };
        let means = self.forward_inner(obs, rows, Some(&mut cache));
        (means, cache)
    }

    /// Gradient of `sum(output * d_output)` with respect to every parameter.
    pub fn backward(&self, cache: &ForwardCache, d_output: &[f32]) -> Vec<Vec<f32>> {
        assert_eq!(cache.rows * self.act_dim, d_output.len());
        assert_eq!(
            cache.inputs.len(),
            self.dims.len() - 1,
            "cache does not match this policy"
        );
        let rows = cache.rows;
        let mut grads: Vec<Vec<f32>> = self.params.iter().map(|p| vec![0f32; p.len()]).collect();
        let mut d_out = d_output.to_vec();

        for layer in (0..self.dims.len() - 1).rev() {
            let in_d = self.dims[layer];
            let out_d = self.dims[layer + 1];
            let hidden = layer + 2 < self.dims.len();
            let mut d_pre = d_out.clone();
            if hidden {
                // d/dx leaky_relu(x) = 1 or LEAKY_SLOPE.
                for (slot, pre) in d_pre.iter_mut().zip(&cache.pre[layer]) {
                    if *pre < 0.0 {
                        *slot *= LEAKY_SLOPE;
                    }
                }
            }

            let weight = &self.params[layer * 2];
            let (weight_grad, bias_grad) = grads
                .split_at_mut(layer * 2 + 1);
            let weight_grad = &mut weight_grad[layer * 2];
            let bias_grad = &mut bias_grad[0];
            for row in 0..rows {
                let input = &cache.inputs[layer][row * in_d..(row + 1) * in_d];
                for unit in 0..out_d {
                    let g = d_pre[row * out_d + unit];
                    if g == 0.0 {
                        continue;
                    }
                    bias_grad[unit] += g;
                    let unit_row = &mut weight_grad[unit * in_d..(unit + 1) * in_d];
                    for (slot, x) in unit_row.iter_mut().zip(input) {
                        *slot += g * x;
                    }
                }
            }

            let mut d_input = vec![0f32; rows * in_d];
            for row in 0..rows {
                let target = &mut d_input[row * in_d..(row + 1) * in_d];
                for unit in 0..out_d {
                    let g = d_pre[row * out_d + unit];
                    if g == 0.0 {
                        continue;
                    }
                    let unit_row = &weight[unit * in_d..(unit + 1) * in_d];
                    for (slot, w) in target.iter_mut().zip(unit_row) {
                        *slot += g * w;
                    }
                }
            }
            if hidden && in_d == out_d {
                // Residual connection passes the output gradient straight to the input.
                for (slot, g) in d_input.iter_mut().zip(&d_out) {
                    *slot += g;
                }
            }
            d_out = d_input;
        }
        grads
    }

    /// Samples one action: `tanh(mean + sigma * eps)`, clipped to `[-1, 1]`.
    /// `sigma = 0` gives the deterministic policy.
    pub fn act(&self, obs: &[f32], sigma: f64, rng: &mut Rng) -> (Vec<f32>, Vec<f32>) {
        let means = self.forward(obs, 1);
        let mut eps = vec![0f32; self.act_dim];
        rng.normals_f32(&mut eps);
        let action = means
            .iter()
            .zip(&eps)
            .map(|(mean, noise)| ((*mean as f64 + sigma * *noise as f64).tanh()).clamp(-1.0, 1.0) as f32)
            .collect();
        (action, eps)
    }

    /// Post-activation values of the input and every layer, for the UI brain view.
    pub fn activations(&self, obs: &[f32]) -> Vec<Vec<f32>> {
        let dims = &self.dims;
        let mut layers = Vec::with_capacity(dims.len());
        let mut h = obs[..dims[0]].to_vec();
        layers.push(h.clone());
        for layer in 0..dims.len() - 1 {
            let in_d = dims[layer];
            let out_d = dims[layer + 1];
            let hidden = layer + 2 < dims.len();
            let weight = &self.params[layer * 2];
            let bias = &self.params[layer * 2 + 1];
            let mut out = vec![0f32; out_d];
            for unit in 0..out_d {
                let row = &weight[unit * in_d..(unit + 1) * in_d];
                let mut acc = bias[unit];
                for (x, w) in h.iter().zip(row) {
                    acc += x * w;
                }
                if hidden && acc < 0.0 {
                    acc *= LEAKY_SLOPE;
                }
                if hidden && in_d == out_d {
                    acc += h[unit];
                }
                out[unit] = acc;
            }
            layers.push(out.clone());
            h = out;
        }
        layers
    }

    fn forward_inner(
        &self,
        obs: &[f32],
        rows: usize,
        mut cache: Option<&mut ForwardCache>,
    ) -> Vec<f32> {
        let dims = &self.dims;
        assert_eq!(obs.len(), rows * dims[0], "observation batch shape");
        let mut h = obs.to_vec();
        for layer in 0..dims.len() - 1 {
            let in_d = dims[layer];
            let out_d = dims[layer + 1];
            let hidden = layer + 2 < dims.len();
            let weight = &self.params[layer * 2];
            let bias = &self.params[layer * 2 + 1];
            let mut pre = vec![0f32; rows * out_d];
            for row in 0..rows {
                let input = &h[row * in_d..(row + 1) * in_d];
                for unit in 0..out_d {
                    let row_w = &weight[unit * in_d..(unit + 1) * in_d];
                    let mut acc = bias[unit];
                    for (x, w) in input.iter().zip(row_w) {
                        acc += x * w;
                    }
                    pre[row * out_d + unit] = acc;
                }
            }
            let mut out = pre.clone();
            if hidden {
                for (slot, value) in out.iter_mut().zip(&pre) {
                    if *value < 0.0 {
                        *slot = *value * LEAKY_SLOPE;
                    }
                }
                if in_d == out_d {
                    for (slot, skip) in out.iter_mut().zip(&h) {
                        *slot += skip;
                    }
                }
            }
            if let Some(cache) = cache.as_deref_mut() {
                cache.inputs.push(h.clone());
                cache.pre.push(pre);
            }
            h = out;
        }
        h
    }

    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        let mut bytes = Vec::with_capacity(64 + self.num_params() * 4);
        bytes.extend_from_slice(MAGIC);
        bytes.extend_from_slice(&VERSION.to_le_bytes());
        bytes.extend_from_slice(&(self.obs_dim as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.act_dim as u32).to_le_bytes());
        bytes.extend_from_slice(&(self.hidden_dims.len() as u32).to_le_bytes());
        for dim in &self.hidden_dims {
            bytes.extend_from_slice(&(*dim as u32).to_le_bytes());
        }
        for param in &self.params {
            bytes.extend_from_slice(&(param.len() as u32).to_le_bytes());
            for value in param {
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        }
        std::fs::write(path, bytes)
    }

    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        let bytes = std::fs::read(path)?;
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
        let invalid = |message: &str| std::io::Error::new(std::io::ErrorKind::InvalidData, message.to_string());

        if take(&mut cursor, 8)? != MAGIC {
            return Err(invalid("not a policy file"));
        }
        if u32_at(take(&mut cursor, 4)?) != VERSION {
            return Err(invalid("unsupported policy version"));
        }
        let obs_dim = u32_at(take(&mut cursor, 4)?) as usize;
        let act_dim = u32_at(take(&mut cursor, 4)?) as usize;
        let hidden_count = u32_at(take(&mut cursor, 4)?) as usize;
        let mut hidden_dims = Vec::with_capacity(hidden_count);
        for _ in 0..hidden_count {
            hidden_dims.push(u32_at(take(&mut cursor, 4)?) as usize);
        }
        let mut dims = Vec::with_capacity(hidden_count + 2);
        dims.push(obs_dim);
        dims.extend_from_slice(&hidden_dims);
        dims.push(act_dim);
        let mut params = Vec::with_capacity((dims.len() - 1) * 2);
        for layer in 0..dims.len() - 1 {
            let expected = dims[layer] * dims[layer + 1];
            let weight_count = u32_at(take(&mut cursor, 4)?) as usize;
            if weight_count != expected {
                return Err(invalid("policy layer shape does not match its header"));
            }
            let raw = take(&mut cursor, weight_count * 4)?;
            params.push(
                raw.chunks_exact(4)
                    .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                    .collect(),
            );
            let bias_count = u32_at(take(&mut cursor, 4)?) as usize;
            if bias_count != dims[layer + 1] {
                return Err(invalid("policy bias shape does not match its header"));
            }
            let raw = take(&mut cursor, bias_count * 4)?;
            params.push(
                raw.chunks_exact(4)
                    .map(|c| f32::from_le_bytes([c[0], c[1], c[2], c[3]]))
                    .collect(),
            );
        }
        Ok(Self {
            obs_dim,
            hidden_dims,
            act_dim,
            dims,
            params,
        })
    }
}

impl Clone for Policy {
    fn clone(&self) -> Self {
        Self {
            obs_dim: self.obs_dim,
            hidden_dims: self.hidden_dims.clone(),
            act_dim: self.act_dim,
            dims: self.dims.clone(),
            params: self.params.clone(),
        }
    }
}
