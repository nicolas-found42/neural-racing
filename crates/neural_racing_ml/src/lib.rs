//! Normalized residual actor-critic for the racing brain.
//!
//! No autograd dependency: the network is a few flat `f32` arrays, so a
//! gradient check costs nothing.

mod adam;
mod matrix;
mod policy;
mod train;

pub use adam::{Adam, AdamMoments};
pub use policy::{
    gaussian_log_prob, log_std_from_raw, log_std_raw_grad, squashed_entropy, ForwardCache,
    InferenceWorkspace, Policy, Sample, ACTION_DIM, CONTROL_HOLD, DEFAULT_BLOCKS, DEFAULT_WIDTH,
    LEAKY_SLOPE, LOG_STD_MAX, LOG_STD_MIN,
};
pub use train::{
    update, Batch, Trajectory, UpdateStats, CLIP_RANGE, ENTROPY_COEF, EPOCHS, GAE_LAMBDA, GAMMA,
    LR, MAX_GRAD_NORM, MINIBATCH, VALUE_COEF,
};
