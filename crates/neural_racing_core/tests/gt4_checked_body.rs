use neural_racing_core::gt4::body::{
    try_integrate_unconstrained, BodyParameters, BodyState, IntegrationError, IntegrationStage,
    MIN_ABS_EULER_COS,
};

fn body() -> BodyParameters {
    BodyParameters {
        mass: 2.0,
        inertia: [1.0, 2.0, 3.0],
    }
}
fn bits(state: &BodyState) -> Vec<u32> {
    [
        state.angles,
        state.position,
        state.velocity,
        state.angular_velocity,
    ]
    .into_iter()
    .flatten()
    .map(f32::to_bits)
    .collect()
}
fn rejected(
    mut state: BodyState,
    parameters: BodyParameters,
    force: [f32; 3],
    moment: [f32; 3],
    dt: f32,
) -> IntegrationError {
    let before = bits(&state);
    let error = try_integrate_unconstrained(&mut state, parameters, force, moment, dt).unwrap_err();
    assert_eq!(bits(&state), before, "error={error}");
    error
}

#[test]
fn nonpositive_mass_and_each_inertia_are_rejected_atomically() {
    for value in [0.0, -0.0, -1.0, -f32::MIN_POSITIVE] {
        let error = rejected(
            BodyState::default(),
            BodyParameters {
                mass: value,
                ..body()
            },
            [0.0; 3],
            [0.0; 3],
            0.01,
        );
        assert_eq!(
            error,
            IntegrationError::NonPositiveParameter {
                field: "mass",
                component: None
            }
        );
        for axis in 0..3 {
            let mut parameters = body();
            parameters.inertia[axis] = value;
            let error = rejected(BodyState::default(), parameters, [0.0; 3], [0.0; 3], 0.01);
            assert_eq!(
                error,
                IntegrationError::NonPositiveParameter {
                    field: "inertia",
                    component: Some(axis)
                }
            );
        }
    }
}

#[test]
fn every_nonfinite_input_component_is_identified_without_mutation() {
    for value in [
        f32::from_bits(0x7fc0_1234),
        f32::INFINITY,
        f32::NEG_INFINITY,
    ] {
        for field in [
            "angles",
            "position",
            "velocity",
            "angular_velocity",
            "inertia",
            "force",
            "moment",
        ] {
            for axis in 0..3 {
                let mut state = BodyState {
                    position: [-0.0; 3],
                    ..BodyState::default()
                };
                let mut parameters = body();
                let mut force = [0.0; 3];
                let mut moment = [0.0; 3];
                match field {
                    "angles" => state.angles[axis] = value,
                    "position" => state.position[axis] = value,
                    "velocity" => state.velocity[axis] = value,
                    "angular_velocity" => state.angular_velocity[axis] = value,
                    "inertia" => parameters.inertia[axis] = value,
                    "force" => force[axis] = value,
                    "moment" => moment[axis] = value,
                    _ => unreachable!(),
                }
                assert_eq!(
                    rejected(state, parameters, force, moment, 0.01),
                    IntegrationError::NonFiniteInput {
                        field,
                        component: Some(axis)
                    }
                );
            }
        }
        assert_eq!(
            rejected(
                BodyState::default(),
                BodyParameters {
                    mass: value,
                    ..body()
                },
                [0.0; 3],
                [0.0; 3],
                0.01
            ),
            IntegrationError::NonFiniteInput {
                field: "mass",
                component: None
            }
        );
        assert_eq!(
            rejected(BodyState::default(), body(), [0.0; 3], [0.0; 3], value),
            IntegrationError::NonFiniteInput {
                field: "dt",
                component: None
            }
        );
    }
}

#[test]
fn singularities_at_each_of_four_drift_inputs_are_rejected() {
    let weights = [
        f32::from_bits(0x3f2c_f45b),
        f32::from_bits(0xbe33_d16c),
        f32::from_bits(0xbe33_d16c),
    ];
    let mut accumulated = 0.0;
    for (drift, weight) in weights.into_iter().chain([0.0]).enumerate() {
        let state = BodyState {
            angles: [std::f32::consts::FRAC_PI_2 + accumulated, 0.0, 0.0],
            angular_velocity: [1.0, 0.0, 0.0],
            ..BodyState::default()
        };
        let error = rejected(
            state,
            BodyParameters {
                inertia: [1.0; 3],
                ..body()
            },
            [0.0; 3],
            [0.0; 3],
            0.4,
        );
        assert_eq!(error, IntegrationError::EulerSingularity { drift });
        accumulated += 0.4 * weight;
    }
}

#[test]
fn cosine_cutoff_has_safe_and_rejected_sides() {
    for (distance, reject) in [
        (MIN_ABS_EULER_COS * 0.5, true),
        (MIN_ABS_EULER_COS * 2.0, false),
    ] {
        for sign in [-1.0, 1.0] {
            let mut state = BodyState {
                angles: [std::f32::consts::FRAC_PI_2 + sign * distance, 0.0, 0.0],
                ..BodyState::default()
            };
            let result = try_integrate_unconstrained(&mut state, body(), [0.0; 3], [0.0; 3], 0.0);
            assert_eq!(result.is_err(), reject);
        }
    }
}

#[test]
fn angular_drift_overflow_is_rejected_atomically() {
    let state = BodyState {
        angular_velocity: [f32::MAX, 0.0, 0.0],
        ..BodyState::default()
    };
    let error = rejected(
        state,
        BodyParameters {
            inertia: [1.0; 3],
            ..body()
        },
        [0.0; 3],
        [0.0; 3],
        f32::MAX,
    );
    assert_eq!(
        error,
        IntegrationError::NonFiniteResult {
            stage: IntegrationStage::Angular(0)
        }
    );
}

#[test]
fn angular_kick_overflow_is_rejected_atomically() {
    let error = rejected(BodyState::default(), body(), [0.0; 3], [f32::MAX; 3], 2.0);
    assert_eq!(
        error,
        IntegrationError::NonFiniteResult {
            stage: IntegrationStage::Angular(1)
        }
    );
}

#[test]
fn translational_overflow_is_rejected_atomically() {
    let state = BodyState {
        position: [f32::MAX; 3],
        velocity: [f32::MAX; 3],
        ..BodyState::default()
    };
    let error = rejected(state, body(), [0.0; 3], [0.0; 3], 2.0);
    assert_eq!(
        error,
        IntegrationError::NonFiniteResult {
            stage: IntegrationStage::Translation
        }
    );
}

#[test]
fn positive_subnormal_mass_and_inertia_do_not_bypass_overflow_checks() {
    let error = rejected(
        BodyState::default(),
        BodyParameters {
            mass: f32::from_bits(1),
            ..body()
        },
        [1.0; 3],
        [0.0; 3],
        0.1,
    );
    assert_eq!(
        error,
        IntegrationError::NonFiniteResult {
            stage: IntegrationStage::Translation
        }
    );
    let error = rejected(
        BodyState::default(),
        BodyParameters {
            inertia: [f32::from_bits(1); 3],
            ..body()
        },
        [0.0; 3],
        [1.0; 3],
        0.1,
    );
    assert_eq!(
        error,
        IntegrationError::NonFiniteResult {
            stage: IntegrationStage::Angular(1)
        }
    );
}

#[test]
fn errors_implement_standard_error_and_use_indexed_diagnostics() {
    fn standard_error(_: &dyn std::error::Error) {}
    let error = IntegrationError::NonFiniteInput {
        field: "force",
        component: Some(2),
    };
    standard_error(&error);
    assert_eq!(error.to_string(), "nonfinite input: force[2]");
    assert_eq!(
        IntegrationError::NonPositiveParameter {
            field: "mass",
            component: None
        }
        .to_string(),
        "parameter must be positive: mass"
    );
}
