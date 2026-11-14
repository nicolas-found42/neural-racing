use neural_racing_core::{Action, Circuit, Env, UNITS_PER_METRE as M};

#[test]
fn slowing_on_the_corner_approach_improves_feedback_before_a_crash() {
    let track = Circuit::Monza.build();
    let arc = (0..1000)
        .map(|i| i as f64 * M)
        .find(|arc| {
            let v = track.speed_profile(*arc) / M;
            let bend = neural_racing_core::wrap_angle(
                track.arc_tangent(*arc + 10.0 * M).angle() - track.arc_tangent(*arc).angle(),
            )
            .abs();
            (25.0..40.0).contains(&v) && bend < 0.01
        })
        .expect("straight approach to a braking zone");
    let limit = track.speed_profile(arc);
    let mut braking = Env::new(track, 1);
    braking.reset_for_section(arc, 1.2 * limit);
    let mut accelerating = braking.clone();
    let mut brake_feedback = 0.0;
    let mut gas_feedback = 0.0;
    for _ in 0..8 {
        let b = braking.step(Action {
            steer: 0.0,
            throttle: -1.0,
        });
        let g = accelerating.step(Action {
            steer: 0.0,
            throttle: 1.0,
        });
        assert!(
            !b.done && !g.done,
            "feedback must arrive before leaving the road"
        );
        brake_feedback += b.reward;
        gas_feedback += g.reward;
        assert!((b.reward - b.info.reward_components.total()).abs() < 1e-8);
    }
    assert!(braking.speed < accelerating.speed - 2.0 * M);
    assert!(
        brake_feedback > gas_feedback + 0.2,
        "early braking {brake_feedback} must beat overspeed throttle {gas_feedback}"
    );
}

#[test]
fn a_section_cannot_farm_a_checkpoint_or_lap_and_reset_restores_normal_racing() {
    let mut env = Env::new(Circuit::Monza.build(), 1);
    let gate = env.track.lap_length / 8.0;
    env.reset_for_section(gate - M, 5.0 * M);
    for _ in 0..8 {
        env.step(Action {
            steer: 0.0,
            throttle: 0.0,
        });
    }
    assert_eq!(env.next_checkpoint, 0);
    assert_eq!(env.credited_laps(), 0);
    assert!(!env.lap_done);
    env.reset_at_checkpoint(0);
    assert_eq!(env.steps, 0);
    assert_eq!(env.distance, 0.0);
}
