//! Residual MLP driving policy with a hand-written forward/backward pass.
//!
//! No autograd dependency: the network is a few thousand flat `f32` arrays, so
//! a gradient check costs nothing and a build needs nothing but rustc.

mod adam;
mod policy;
mod train;

pub use adam::{Adam, AdamMoments};
pub use policy::{Policy, ForwardCache, DEFAULT_HIDDEN_DIMS, LEAKY_SLOPE};
pub use train::{
    discounted_returns, sigma_at, update, Batch, UpdateStats, GAMMA, LR, MAX_BACKTRACKS,
    MAX_POLICY_KL, SIGMA_DECAY, SIGMA_END, SIGMA_START,
};
