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
        stats.entropy >= f64::from(neural_racing_ml::LOG_STD_MIN)
            && stats.entropy <= f64::from(neural_racing_ml::LOG_STD_MAX),
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
fn the_policy_round_trips_through_disk() {
    let policy = Policy::new(19, 32, 2, 12);
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
    let obs = observations(4, 19);
    assert_eq!(loaded.forward(&obs, 4), policy.forward(&obs, 4));
    std::fs::remove_dir_all(&directory).unwrap();
}
