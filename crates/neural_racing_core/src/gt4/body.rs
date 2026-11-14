//! The unconstrained integration block within GT4 routine 0x003e2b28.
//! Checked API adapted from the user-supplied raw/patches/gt4-integration.
//! This generic body is separate from the normal four-wheel car chassis path.
//!
//! Sleep/contact/collision wrappers are not recovered here. Offset comments
//! record provenance; these Rust types do not describe the original binary ABI.
//! Angle indices intentionally carry no unverified Euler-axis names.

use super::math;
use std::{convert::Infallible, fmt};

/// Recovered gravity magnitude, preserving the supplied single-precision bits.
pub const GRAVITY: f32 = f32::from_bits(0x411c_f5c2);
/// Recovered angle period; not replaced by a host mathematical constant.
pub const ANGLE_PERIOD: f32 = f32::from_bits(0x40c9_0fda);
/// Host safety policy for the checked API: reject `abs(cos(angles[0])) <= 1e-4`.
/// This tolerance is not a recovered GT4 constant or an accuracy guarantee.
pub const MIN_ABS_EULER_COS: f32 = 1.0e-4;

// Keep the recovered bits and the four-drift / three-kick order, including
// negative stages. Algebraic simplification can change f32 rounding.
const ANGLE_WEIGHTS: [f32; 4] = [
    f32::from_bits(0x3f2c_f45b),
    f32::from_bits(0xbe33_d16c),
    f32::from_bits(0xbe33_d16c),
    f32::from_bits(0x3f2c_f45b),
];
const ANGULAR_VELOCITY_WEIGHTS: [f32; 3] = [
    f32::from_bits(0x3fac_f45b),
    f32::from_bits(0xbfd9_e8b6),
    f32::from_bits(0x3fac_f45b),
];

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BodyState {
    /// Original +0x04, +0x08, +0x0c. Recovered angle order, in radians.
    pub angles: [f32; 3],
    /// Original +0x10, +0x14, +0x18.
    pub position: [f32; 3],
    /// Original +0x1c, +0x20, +0x24.
    pub velocity: [f32; 3],
    /// Original +0x28, +0x2c, +0x30. Components used by the recovered equations.
    pub angular_velocity: [f32; 3],
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyParameters {
    /// Descriptor +0xb4. Must be finite and strictly positive in checked calls.
    pub mass: f32,
    /// Descriptor +0x24, +0x28, +0x2c. Finite, strictly positive principal inertias.
    pub inertia: [f32; 3],
}

/// Location of a numerical failure within one attempted step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegrationStage {
    /// 0: initial angle drift; 1..=3: angular kick followed by angle drift.
    Angular(usize),
    /// Single angle correction and translation using the incoming velocity.
    Translation,
}

/// Checked integration failures. Every error leaves the caller's state intact.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegrationError {
    /// A scalar or indexed vector component is NaN or infinite.
    NonFiniteInput {
        field: &'static str,
        component: Option<usize>,
    },
    /// Mass or a principal inertia is zero (including -0) or negative.
    NonPositiveParameter {
        field: &'static str,
        component: Option<usize>,
    },
    /// An orientation used by drift 0..=3 is too close to an Euler singularity.
    EulerSingularity { drift: usize },
    /// Finite inputs produced nonfinite intermediate or final state.
    NonFiniteResult { stage: IntegrationStage },
}

impl fmt::Display for IntegrationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonFiniteInput { field, component } => {
                write!(f, "nonfinite input: {field}")?;
                if let Some(index) = component {
                    write!(f, "[{index}]")?;
                }
                Ok(())
            }
            Self::NonPositiveParameter { field, component } => {
                write!(f, "parameter must be positive: {field}")?;
                if let Some(index) = component {
                    write!(f, "[{index}]")?;
                }
                Ok(())
            }
            Self::EulerSingularity { drift } => {
                write!(f, "Euler singularity at angle drift {drift}")
            }
            Self::NonFiniteResult { stage } => write!(f, "nonfinite result at {stage:?}"),
        }
    }
}

impl std::error::Error for IntegrationError {}

fn advance_angles(angles: &mut [f32; 3], angular_velocity: [f32; 3], dt: f32) {
    // Both sin/cos pairs and all rates use the incoming orientation.
    let (sin_first, cos_first) = math::sin_cos(angles[0]);
    let (sin_third, cos_third) = math::sin_cos(angles[2]);
    let [wx, wy, wz] = angular_velocity;
    let rate_second = (wx * sin_third - wy * cos_third) * (1.0 / cos_first);
    angles[1] += rate_second * dt;
    angles[0] += -(wx * cos_third + wy * sin_third) * dt;
    angles[2] += -(rate_second * sin_first + wz) * dt;
}

fn advance_angular_velocity(
    angular_velocity: &mut [f32; 3],
    inertia: [f32; 3],
    moment: [f32; 3],
    step: f32,
) {
    // All three right-hand sides must use the same pre-kick snapshot.
    let [wx, wy, wz] = *angular_velocity;
    let [ix, iy, iz] = inertia;
    angular_velocity[1] = wy + ((moment[1] + (iz - ix) * wz * wx) * step) / iy;
    angular_velocity[0] = wx + ((moment[0] + (iy - iz) * wy * wz) * step) / ix;
    angular_velocity[2] = wz + ((moment[2] + (ix - iy) * wx * wy) * step) / iz;
}

fn integrate_rotation<E>(
    state: &mut BodyState,
    inertia: [f32; 3],
    moment: [f32; 3],
    dt: f32,
    mut drift: impl FnMut(&mut BodyState, f32, usize) -> Result<(), E>,
) -> Result<(), E> {
    // The +0.0 operations are intentional, including their signed-zero effects.
    let moment = [moment[0] + 0.0, moment[1] + 0.0, moment[2] + 0.0];
    drift(state, dt * ANGLE_WEIGHTS[0], 0)?;
    for stage in 0..3 {
        advance_angular_velocity(
            &mut state.angular_velocity,
            inertia,
            moment,
            dt * ANGULAR_VELOCITY_WEIGHTS[stage],
        );
        drift(state, dt * ANGLE_WEIGHTS[stage + 1], stage + 1)?;
    }
    Ok(())
}

fn finish_step(state: &mut BodyState, mass: f32, force: [f32; 3], dt: f32) {
    // Two independent conditions, one correction each; never rem_euclid.
    for angle in &mut state.angles {
        if *angle < 0.0 {
            *angle += ANGLE_PERIOD;
        }
        if *angle >= ANGLE_PERIOD {
            *angle -= ANGLE_PERIOD;
        }
    }
    let force = [force[0] + 0.0, -mass * GRAVITY + force[1], force[2] + 0.0];
    let inverse_mass = 1.0 / mass;
    for (axis, force_component) in force.into_iter().enumerate() {
        state.position[axis] += state.velocity[axis] * dt;
        state.velocity[axis] += force_component * inverse_mass * dt;
    }
}

/// Execute the supplied recovered equations without validating inputs.
///
/// `force` and `moment` correspond to non-null +0x4c/+0x50 inputs. Force excludes
/// gravity, which is added on component 1. Keep vector frames consistent with the
/// recovered caller; this block does not transform inputs or infer axis names.
/// Translation uses the incoming velocity, then updates velocity. Angular motion
/// uses four angle drifts and three principal-inertia kicks.
///
/// The caller is responsible for positive mass/inertias, finite arithmetic and a
/// nonsingular Euler state at every drift. Violations may produce NaN/infinity.
/// Zero and negative finite `dt` are evaluated literally. Zero `dt` still runs
/// the equations and single angle correction; it is not a bitwise no-op.
/// Angles far outside one revolution need not end up in `[0, ANGLE_PERIOD)`.
///
/// See [`try_integrate_unconstrained`] for atomic validation and error reporting.
/// Uses the separately recovered GT4 math backend. Original-instruction fixture
/// checks bound numerical differences; hardware bitwise fidelity is unproven.
pub fn integrate_unconstrained(
    state: &mut BodyState,
    parameters: BodyParameters,
    force: [f32; 3],
    moment: [f32; 3],
    dt: f32,
) {
    let result: Result<(), Infallible> =
        integrate_rotation(state, parameters.inertia, moment, dt, |state, step, _| {
            advance_angles(&mut state.angles, state.angular_velocity, step);
            Ok(())
        });
    match result {
        Ok(()) => {}
        Err(never) => match never {},
    }
    finish_step(state, parameters.mass, force, dt);
}

/// Validate and integrate on a copy, committing only a finite successful result.
///
/// Checks every input component, positive mass/inertias, the cosine denominator
/// before each drift, and intermediate/final state finiteness. The Euler cutoff
/// is [`MIN_ABS_EULER_COS`], a host safety policy. Errors preserve all original
/// state bits, including NaN payloads and signed zeros. Successful calls use the
/// same equations as [`integrate_unconstrained`], including its single angle
/// correction and literal zero/negative timesteps.
///
/// Finiteness and sampled cosine checks do not guarantee accuracy, stability,
/// physical validity, a nonsingular final orientation, or absence of a crossing
/// between sampled orientations. No automatic timestep subdivision is performed.
///
/// ```
/// use neural_racing_core::gt4::body::{try_integrate_unconstrained, BodyParameters, BodyState};
/// let mut state = BodyState::default();
/// let body = BodyParameters { mass: 1200.0, inertia: [500.0, 1500.0, 1700.0] };
/// try_integrate_unconstrained(&mut state, body, [0.0; 3], [0.0; 3], 1.0 / 60.0)?;
/// assert!(state.velocity[1] < 0.0);
/// # Ok::<(), neural_racing_core::gt4::body::IntegrationError>(())
/// ```
pub fn try_integrate_unconstrained(
    state: &mut BodyState,
    parameters: BodyParameters,
    force: [f32; 3],
    moment: [f32; 3],
    dt: f32,
) -> Result<(), IntegrationError> {
    for (field, values) in state_vectors(state).into_iter().chain([
        ("inertia", parameters.inertia),
        ("force", force),
        ("moment", moment),
    ]) {
        for (component, value) in values.into_iter().enumerate() {
            finite_input(value, field, Some(component))?;
        }
    }
    finite_input(parameters.mass, "mass", None)?;
    finite_input(dt, "dt", None)?;
    if parameters.mass <= 0.0 {
        return Err(IntegrationError::NonPositiveParameter {
            field: "mass",
            component: None,
        });
    }
    for (component, inertia) in parameters.inertia.into_iter().enumerate() {
        if inertia <= 0.0 {
            return Err(IntegrationError::NonPositiveParameter {
                field: "inertia",
                component: Some(component),
            });
        }
    }

    let mut next = *state;
    integrate_rotation(
        &mut next,
        parameters.inertia,
        moment,
        dt,
        |next, step, drift| {
            let stage = IntegrationStage::Angular(drift);
            finite_result(next, stage)?;
            let (_, cosine) = math::sin_cos(next.angles[0]);
            if cosine.abs() <= MIN_ABS_EULER_COS {
                return Err(IntegrationError::EulerSingularity { drift });
            }
            advance_angles(&mut next.angles, next.angular_velocity, step);
            finite_result(next, stage)
        },
    )?;
    finish_step(&mut next, parameters.mass, force, dt);
    finite_result(&next, IntegrationStage::Translation)?;
    *state = next;
    Ok(())
}

fn state_vectors(state: &BodyState) -> [(&'static str, [f32; 3]); 4] {
    [
        ("angles", state.angles),
        ("position", state.position),
        ("velocity", state.velocity),
        ("angular_velocity", state.angular_velocity),
    ]
}

fn finite_input(
    value: f32,
    field: &'static str,
    component: Option<usize>,
) -> Result<(), IntegrationError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(IntegrationError::NonFiniteInput { field, component })
    }
}

fn finite_result(state: &BodyState, stage: IntegrationStage) -> Result<(), IntegrationError> {
    if state_vectors(state)
        .into_iter()
        .all(|(_, values)| values.into_iter().all(f32::is_finite))
    {
        Ok(())
    } else {
        Err(IntegrationError::NonFiniteResult { stage })
    }
}
