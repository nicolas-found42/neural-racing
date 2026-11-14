//! Recovered GT4 stock M3 normal-road vehicle path and its numeric helpers.
//!
//! `runtime` assembles the selected model for manual driving and learning on a
//! flat asphalt plane. Source addresses, branches and original-instruction
//! evidence are recorded in `PORT_LEDGER.md`. This is not the full GT4 game or
//! a claim of physical R5900 bit equality; external course/collision/peripheral
//! systems and unselected vehicle configurations remain outside this adapter.

pub mod curve;
pub mod engine;
pub mod slip;

pub mod tire;

pub mod attitude;
pub(crate) mod automatic;
pub mod body;
pub mod calibration;
pub mod chassis;
pub mod kinematics;
pub mod math;
pub mod normal;
pub(crate) mod parameters;
pub(crate) mod runtime;

pub mod drivetrain;

pub mod suspension;

pub mod steering;

pub(crate) mod eefloat;

pub mod control;

pub mod native;

pub(crate) mod conductor;

pub(crate) mod preparation;

pub(crate) mod aids;

pub(crate) mod steering_frame;

pub(crate) mod brake_heat;

#[cfg(test)]
pub(crate) mod test_support;

pub(crate) mod contact_finish;
