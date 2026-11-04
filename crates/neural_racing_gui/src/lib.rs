//! Neural Racing: watch or drive real circuits or procedural courses, and see the
//! trained policy's activations while it races.

pub mod app;
pub mod palette;
pub mod render;
pub mod sim;
pub mod widgets;

pub use app::{App, Mode};

pub mod learning;
