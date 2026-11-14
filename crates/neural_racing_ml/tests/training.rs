use neural_racing_core::Rng;
use neural_racing_ml::{
    update, Adam, Batch, Policy, Sample, Trajectory, ACTION_DIM, GAMMA, LR, MAX_GRAD_NORM,
};

fn observations(rows: usize, width: usize) -> Vec<f32> {
    (0..rows * width)
        .map(|index| ((index as f32) * 0.37).sin())
        .collect()
}

#[test]
fn analytic_gradients_match_finite_differences() {
    let mut policy = Policy::new(7, 8, 2, 11);
    let rows = 3;
    let obs = observations(rows, 7);
    let dz: Vec<f32> = (0..rows * 5).map(|i| ((i as f32) * 1.13).cos()).collect();
    let (_, cache) = policy.forward_cached(&obs, rows);
    let grads = policy.backward(&cache, &dz);

    let loss = |policy: &Policy| -> f64 {
        policy
            .forward(&obs, rows)
            .iter()
            .zip(&dz)
            .map(|(z, d)| f64::from(*z) * f64::from(*d))
            .sum()
    };

    let mut checked = 0;
    let mut worst: f64 = 0.0;
    for (layer, layer_grad) in grads.iter().enumerate() {
        for &coordinate in &[0, layer_grad.len() / 3, layer_grad.len() - 1] {
            let original = policy.params()[layer][coordinate];
            let analytic = f64::from(layer_grad[coordinate]);
            let eps = 1e-3f32;
            policy.params_mut()[layer][coordinate] = original + eps;
            let high = loss(&policy);
            policy.params_mut()[layer][coordinate] = original - eps;
            let low = loss(&policy);
            policy.params_mut()[layer][coordinate] = original;
            let numeric = (high - low) / (2.0 * f64::from(eps));
            let error = (numeric - analytic).abs() / analytic.abs().max(1.0);
            worst = worst.max(error);
            checked += 1;
        }
    }
    assert!(checked >= 10);
    assert!(
        worst < 5e-3,
        "worst relative gradient error {worst:.3e} over {checked} coordinates"
    );
}

#[test]
fn normalization_keeps_each_trunk_layer_at_a_stable_scale() {
    let policy = Policy::new(4, 8, 2, 5);
    let obs = vec![0.25f32; 4];
    let activations = policy.activations(&obs);
    assert_eq!(activations.len(), 5);
    assert_eq!(activations[0].len(), 4);
    for layer in &activations[1..] {
        assert_eq!(layer.len(), 8);
        assert!(layer.iter().all(|value| value.is_finite()));
    }
}

#[test]
fn reusable_inference_matches_training_forward_and_individual_samples() {
    let policy = Policy::new(neural_racing_core::OBS_DIM, 256, 5, 42);
    let mut workspace = neural_racing_ml::InferenceWorkspace::default();
    // Reuse one workspace across growth, shrinkage, empty batches and new data.
    for (trial, rows) in [1, 4, 16, 0, 4, 1].into_iter().enumerate() {
        let obs: Vec<f32> = observations(rows, policy.obs_dim)
            .into_iter()
            .map(|v| v + trial as f32 * 0.13)
            .collect();
        let actual = policy.forward_into(&obs, rows, &mut workspace);
        let expected = policy.forward_cached(&obs, rows).0;
        assert_eq!(actual.len(), expected.len());
        for (&actual, expected) in actual.iter().zip(expected) {
            assert!((actual - expected).abs() < 1e-6);
        }
        for row in 0..rows {
            let mut batched_rng = Rng::new(row as u64 * 7919);
            let mut single_rng = Rng::new(row as u64 * 7919);
            let batched =
                Policy::sample_from_outputs(&actual[row * 5..(row + 1) * 5], &mut batched_rng);
            let single = policy.sample(
                &obs[row * policy.obs_dim..(row + 1) * policy.obs_dim],
                &mut single_rng,
            );
            assert_eq!(batched.eps, single.eps);
            for dim in 0..2 {
                assert!((batched.action[dim] - single.action[dim]).abs() < 1e-5);
            }
            assert!((batched.logp - single.logp).abs() < 1e-5);
            assert!((batched.value - single.value).abs() < 1e-5);
        }
    }
}

#[test]
fn sampled_actions_are_clipped_to_the_command_range() {
    let policy = Policy::new(4, 8, 2, 1);
    let mut rng = Rng::new(3);
    let obs = vec![0.5f32; 4];
    for _ in 0..200 {
        let sample = policy.sample(&obs, &mut rng);
        assert!(sample
            .action
            .iter()
            .all(|value| (-1.0..=1.0).contains(value)));
        assert!(sample.logp.is_finite());
        assert!(sample.log_std[0] < neural_racing_ml::LOG_STD_MAX);
        assert!(sample.log_std[0] > neural_racing_ml::LOG_STD_MIN);
    }
    let mean = policy.mean_action(&obs);
    assert!(mean.iter().all(|value| (-1.0..=1.0).contains(value)));
}

#[test]
fn empty_and_non_finite_batches_are_rejected_before_any_update() {
    let mut policy = Policy::new(4, 8, 2, 3);
    let mut adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
    let before = policy.params().to_vec();

    let batch = Batch::new(4, ACTION_DIM);
    let err = update(&mut policy, &mut adam, &batch).expect_err("empty batches fail");
    assert_eq!(err, "empty batch");
    assert_eq!(
        policy.params(),
        before.as_slice(),
        "rejected update changed params"
    );

    let mut nan = Batch::new(4, ACTION_DIM);
    nan.push_row(&[0.0; 4], &[0.0; 2], 0.0, 0.0, f32::NAN, 1.0);
    let err = update(&mut policy, &mut adam, &nan).expect_err("a NaN advantage must fail");
    assert_eq!(err, "non-finite batch");
    assert_eq!(policy.params(), before.as_slice());
}

#[test]
fn an_oversized_update_is_rolled_back_completely() {
    let mut policy = Policy::new(6, 16, 2, 9);
    let mut explosive = Adam::new(policy.params(), 1e6, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
    let before = policy.params().to_vec();
    let mut batch = Batch::new(6, ACTION_DIM);
    let mut rng = Rng::new(2);
    for _ in 0..16 {
        let obs = observations(1, 6);
        let sample = policy.sample(&obs, &mut rng);
        batch.push_row(
            &obs,
            &sample.z,
            sample.logp,
            sample.value,
            sample.value + 1.0,
            1.0,
        );
    }
    let stats = update(&mut policy, &mut explosive, &batch).expect("finite update");
    assert_eq!(
        stats.update_scale, 0.0,
        "an oversized step must be rejected"
    );
    assert_eq!(
        policy.params(),
        before.as_slice(),
        "rollback must restore params"
    );
    assert!(stats.policy_kl > 0.0);
}

#[test]
fn an_accepted_update_stays_inside_the_trust_region() {
    let mut policy = Policy::new(6, 16, 2, 4);
    let mut adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
    let before = policy.params().to_vec();
    let mut batch = Batch::new(6, ACTION_DIM);
    let mut rng = Rng::new(8);
    for row in 0..32 {
        let obs = observations(1, 6);
        let sample = policy.sample(&obs, &mut rng);
        batch.push_row(
            &obs,
            &sample.z,
            sample.logp,
            sample.value,
            (row as f32 * 0.1).sin(),
            (row as f32).cos(),
        );
    }
    let stats = update(&mut policy, &mut adam, &batch).expect("finite update");
    assert!(stats.policy_kl.is_finite());
    assert!(
        stats.policy_kl < 0.05,
        "trust region exceeded: KL {}",
        stats.policy_kl
    );
    assert!(stats.gradient_norm.is_finite() && stats.gradient_norm > 0.0);
    assert!(stats.parameter_norm.is_finite() && stats.parameter_norm > 0.0);
    assert!(
        stats.exploration_sigma >= f64::from(neural_racing_ml::LOG_STD_MIN).exp()
            && stats.exploration_sigma <= f64::from(neural_racing_ml::LOG_STD_MAX).exp(),
        "reported entropy must respect the bounded sigma: {}",
        stats.entropy
    );
    assert!(stats.gradient_clip > 0.0 && stats.gradient_clip <= 1.0);
    assert_ne!(
        policy.params(),
        before.as_slice(),
        "accepted update must move params"
    );
}

#[test]
fn truncated_trajectories_bootstrap_and_terminal_ones_do_not() {
    let value = 4.0f32;
    let mut truncated = Trajectory::default();
    let sample = Sample {
        action: [0.0; ACTION_DIM],
        z: [0.0; ACTION_DIM],
        mean: [0.0; ACTION_DIM],
        log_std: [0.0; ACTION_DIM],
        eps: [0.0; ACTION_DIM],
        logp: 0.0,
        value,
    };
    truncated.push(&[0.0; 6], &sample, 1.0);
    truncated.bootstrap = 2.0;

    let mut batch = Batch::new(6, ACTION_DIM);
    batch.push_trajectory(&truncated, false);
    let truncated_return = batch.returns[0];
    assert!(
        (truncated_return - (1.0 + GAMMA as f32 * 2.0)).abs() < 1e-5,
        "a cut sample must bootstrap: {truncated_return}"
    );

    batch.clear();
    batch.push_trajectory(&truncated, true);
    let terminal_return = batch.returns[0];
    assert!(
        (terminal_return - 1.0).abs() < 1e-5,
        "a terminal sample must not bootstrap: {terminal_return}"
    );
}

#[test]
fn policies_from_the_previous_vehicle_are_rejected() {
    let path = std::env::temp_dir().join(format!(
        "neural-policy-old-physics-{}.bin",
        std::process::id()
    ));
    let policy = Policy::new(neural_racing_core::OBS_DIM, 32, 2, 42);
    policy.save(&path).unwrap();
    let mut bytes = std::fs::read(&path).unwrap();
    for previous in [2u32, 3] {
        bytes[8..12].copy_from_slice(&previous.to_le_bytes());
        std::fs::write(&path, &bytes).unwrap();
        let error = match Policy::load(&path) {
            Ok(_) => panic!("old physics/cadence policy was accepted"),
            Err(error) => error,
        };
        assert_eq!(error.kind(), std::io::ErrorKind::InvalidData);
        assert!(error.to_string().contains("retrain from scratch"));
    }
    std::fs::remove_file(path).unwrap();
}

#[test]
fn the_policy_round_trips_through_disk() {
    let policy = Policy::new(neural_racing_core::OBS_DIM, 32, 2, 12);
    let directory = std::env::temp_dir().join(format!("nr-policy-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("policy.bin");
    policy.save(&path).unwrap();
    let loaded = Policy::load(&path).unwrap();
    assert_eq!(loaded.obs_dim, policy.obs_dim);
    assert_eq!(loaded.act_dim, policy.act_dim);
    assert_eq!(loaded.width, policy.width);
    assert_eq!(loaded.blocks, policy.blocks);
    assert_eq!(loaded.num_params(), policy.num_params());
    assert_eq!(loaded.params(), policy.params());
    let obs = observations(4, neural_racing_core::OBS_DIM);
    assert_eq!(loaded.forward(&obs, 4), policy.forward(&obs, 4));
    std::fs::remove_dir_all(&directory).unwrap();
}

#[test]
fn entropy_only_update_recovers_pedal_variation_from_saturation() {
    let mut policy = Policy::new(4, 16, 2, 42);
    for layer in policy.params_mut() {
        layer.fill(0.0);
    }
    policy.params_mut().last_mut().unwrap()[1] = 3.0;
    let obs = [0.0; 4];
    let before = policy.mean_action(&obs)[1];
    let mut rng = Rng::new(9);
    let mut batch = Batch::new(4, ACTION_DIM);
    for _ in 0..256 {
        let sample = policy.sample(&obs, &mut rng);
        batch.push_row(
            &obs,
            &sample.z,
            sample.logp,
            sample.value,
            sample.value,
            0.0,
        );
    }
    let mut adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
    update(&mut policy, &mut adam, &batch).unwrap();
    let after = policy.mean_action(&obs)[1];
    assert!(
        after < before,
        "entropy must pull a saturated pedal back into usable range: {before} -> {after}"
    );
}

#[test]
fn crash_credits_braking_decisions_three_seconds_earlier() {
    let mut trajectory = Trajectory::default();
    let sample = Sample {
        action: [0.0; 2],
        z: [0.0; 2],
        mean: [0.0; 2],
        log_std: [0.0; 2],
        eps: [0.0; 2],
        logp: 0.0,
        value: 0.0,
    };
    for step in 0..61 {
        trajectory.push(&[0.0; 4], &sample, if step == 60 { -30.0 } else { 0.0 });
    }
    let mut batch = Batch::new(4, 2);
    batch.push_trajectory(&trajectory, true);
    assert!(
        batch.advantages[0] < -5.0,
        "three-second braking credit too weak: {}",
        batch.advantages[0]
    );
}

#[test]
fn bounded_entropy_gradients_match_finite_differences_even_when_saturated() {
    for mean in [-8.0, -2.0, 0.0, 0.7, 3.0, 8.0] {
        for log_std in [-1.89, -1.0, -0.52] {
            let (h, dm, ds) = neural_racing_ml::squashed_entropy(mean, log_std);
            let eps = 1e-5;
            let numeric_m = (neural_racing_ml::squashed_entropy(mean + eps, log_std).0
                - neural_racing_ml::squashed_entropy(mean - eps, log_std).0)
                / (2.0 * eps);
            let numeric_s = (neural_racing_ml::squashed_entropy(mean, log_std + eps).0
                - neural_racing_ml::squashed_entropy(mean, log_std - eps).0)
                / (2.0 * eps);
            assert!(h.is_finite());
            assert!(
                (dm - numeric_m).abs() < 1e-7,
                "mean={mean} sigma={log_std}: {dm} != {numeric_m}"
            );
            assert!(
                (ds - numeric_s).abs() < 1e-7,
                "mean={mean} sigma={log_std}: {ds} != {numeric_s}"
            );
        }
    }
    let center = neural_racing_ml::squashed_entropy(0.0, -1.0).0;
    let saturated = neural_racing_ml::squashed_entropy(4.0, -1.0).0;
    assert!(center > saturated + 5.0);
}

#[test]
fn held_controls_discount_every_physics_decision_and_bootstrap_at_the_true_cut() {
    let sample = Sample {
        action: [0.0; 2],
        z: [0.0; 2],
        mean: [0.0; 2],
        log_std: [0.0; 2],
        eps: [0.0; 2],
        logp: 0.0,
        value: 1.0,
    };
    let reward = (0..4).map(|i| GAMMA.powi(i) * 2.0).sum::<f64>() as f32;
    let mut trajectory = Trajectory::default();
    trajectory.push_held(&[0.0; 4], &sample, reward, 4);
    trajectory.bootstrap = 5.0;
    let mut batch = Batch::new(4, 2);
    batch.push_trajectory(&trajectory, false);
    assert!((batch.returns[0] as f64 - (reward as f64 + GAMMA.powi(4) * 5.0)).abs() < 1e-5);
    batch.clear();
    batch.push_trajectory(&trajectory, true);
    assert_eq!(batch.returns[0], reward);
}
