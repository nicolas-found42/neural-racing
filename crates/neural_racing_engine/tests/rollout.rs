use neural_racing_core::{Circuit, Env, Rng, OBS_DIM};
use neural_racing_engine::{rollout, rollout_at_checkpoint};
use neural_racing_ml::Policy;

#[test]
fn checkpoint_rollouts_sample_the_correct_start_and_keep_terminal_rows() {
    let policy = Policy::new(OBS_DIM, 16, 2, 42);
    for checkpoint in [0, 2, 4, 6] {
        let mut env = Env::new(Circuit::Monza.build(), 5);
        env.reset_at_checkpoint(checkpoint);
        let expected = policy.sample(&env.obs, &mut Rng::new(17));
        let result =
            rollout_at_checkpoint(&policy, env.track.clone(), 5, checkpoint, &mut Rng::new(17));
        assert_eq!(&result.batch.obs[..OBS_DIM], &env.obs);
        assert_eq!(&result.batch.z[..2], &expected.z);
        assert_eq!(result.batch.old_logp[0], expected.logp);
        assert_eq!(
            result.batch.rows,
            (result.report.steps as usize).div_ceil(neural_racing_ml::CONTROL_HOLD as usize)
        );
        assert!(result.report.steps > 0);
        assert!(
            result.report.off_track
                || result.report.lap_done
                || result.report.steps == env.max_steps()
        );
        assert!(result.batch.returns.iter().all(|v| v.is_finite()));
    }
}

#[test]
fn zero_checkpoint_keeps_the_normal_rollout_stream() {
    let policy = Policy::new(OBS_DIM, 16, 2, 9);
    let a = rollout(&policy, Circuit::Silverstone.build(), 1, &mut Rng::new(3));
    let b = rollout_at_checkpoint(
        &policy,
        Circuit::Silverstone.build(),
        1,
        0,
        &mut Rng::new(3),
    );
    assert_eq!(a.batch.obs, b.batch.obs);
    assert_eq!(a.batch.z, b.batch.z);
    assert_eq!(a.batch.returns, b.batch.returns);
    assert_eq!(a.report.distance, b.report.distance);
}
