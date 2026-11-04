//! Adam with global norm clipping and a snapshot for KL rollback.

/// Saved moments so a rejected update can be undone exactly.
pub struct AdamMoments {
    m: Vec<Vec<f32>>,
    v: Vec<Vec<f32>>,
    t: i64,
}

pub struct Adam {
    lr: f64,
    beta1: f64,
    beta2: f64,
    eps: f64,
    max_grad_norm: f64,
    m: Vec<Vec<f32>>,
    v: Vec<Vec<f32>>,
    t: i64,
}

impl Adam {
    pub fn new(
        params: &[Vec<f32>],
        lr: f64,
        betas: (f64, f64),
        eps: f64,
        max_grad_norm: f64,
    ) -> Self {
        Self {
            lr,
            beta1: betas.0,
            beta2: betas.1,
            eps,
            max_grad_norm,
            m: params.iter().map(|p| vec![0f32; p.len()]).collect(),
            v: params.iter().map(|p| vec![0f32; p.len()]).collect(),
            t: 0,
        }
    }

    pub fn max_grad_norm(&self) -> f64 {
        self.max_grad_norm
    }

    pub fn snapshot(&self) -> AdamMoments {
        AdamMoments {
            m: self.m.clone(),
            v: self.v.clone(),
            t: self.t,
        }
    }

    pub fn restore(&mut self, saved: AdamMoments) {
        assert_eq!(saved.m.len(), self.m.len());
        self.m = saved.m;
        self.v = saved.v;
        self.t = saved.t;
    }

    /// Applies one step; returns the global gradient norm before clipping.
    pub fn step(&mut self, params: &mut [Vec<f32>], grads: &[Vec<f32>]) -> f64 {
        assert_eq!(params.len(), self.m.len());
        assert_eq!(grads.len(), self.m.len());
        let squares: f64 = grads
            .iter()
            .flat_map(|layer| layer.iter())
            .map(|value| f64::from(*value) * f64::from(*value))
            .sum();
        let total_norm = squares.sqrt();
        let clip = (self.max_grad_norm / (total_norm + 1e-8)).min(1.0);
        let step = (self.t + 1) as f64;
        let bias_correction1 = 1.0 - self.beta1.powf(step);
        let bias_correction2 = 1.0 - self.beta2.powf(step);
        let step_size = self.lr / bias_correction1;
        let correction = bias_correction2.sqrt();
        for index in 0..params.len() {
            let m = &mut self.m[index];
            let v = &mut self.v[index];
            for ((param, grad), (m, v)) in params[index]
                .iter_mut()
                .zip(&grads[index])
                .zip(m.iter_mut().zip(v.iter_mut()))
            {
                let g = f64::from(*grad) * clip;
                let exp_avg = f64::from(*m) * self.beta1 + g * (1.0 - self.beta1);
                let exp_avg_sq = f64::from(*v) * self.beta2 + g * g * (1.0 - self.beta2);
                let denom = exp_avg_sq.sqrt() / correction + self.eps;
                *param = (f64::from(*param) - step_size * exp_avg / denom) as f32;
                *m = exp_avg as f32;
                *v = exp_avg_sq as f32;
            }
        }
        self.t += 1;
        total_norm
    }
}
