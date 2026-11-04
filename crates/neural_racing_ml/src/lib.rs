//! Normalized residual actor-critic for the racing brain.
//!
//! No autograd dependency: the network is a few flat `f32` arrays, so a
//! gradient check costs nothing and a build needs nothing but rustc.

mod adam;
mod matrix;
mod policy;
mod train;

pub use adam::{Adam, AdamMoments};
pub use policy::{
    gaussian_log_prob, log_std_from_raw, ForwardCache, Policy, Sample, ACTION_DIM, DEFAULT_BLOCKS,
    DEFAULT_WIDTH, LEAKY_SLOPE, LOG_STD_MAX, LOG_STD_MIN,
};
pub use train::{
    update, Batch, Trajectory, UpdateStats, CLIP_RANGE, ENTROPY_COEF, EPOCHS, GAE_LAMBDA, GAMMA,
    LR, MAX_GRAD_NORM, MINIBATCH, VALUE_COEF,
};
