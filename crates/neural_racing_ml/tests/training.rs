use neural_racing_core::Rng;
use neural_racing_ml::{
    discounted_returns, sigma_at, update, Adam, Batch, Policy, GAMMA, LR, SIGMA_END, SIGMA_START,
};

fn sample_obs(rows: usize, width: usize) -> Vec<f32> {
    (0..rows * width)
        .map(|index| ((index as f32) * 0.37).sin())
        .collect()
}

fn sample_eps(rows: usize, width: usize) -> Vec<f32> {
    (0..rows * width)
        .map(|index| ((index as f32) * 1.13).cos())
        .collect()
}

#[test]
fn analytic_gradients_match_finite_differences() {
    let mut policy = Policy::new(7, &[8, 8, 5], 2, 11);
    let rows = 3;
    let obs = sample_obs(rows, 7);
    let dz = sample_eps(rows, 2);
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
fn a_residual_policy_keeps_the_input_signal_scale() {
    let policy = Policy::new(4, &[8, 8, 8], 2, 5);
    let obs = vec![0.25f32; 4];
    let activations = policy.activations(&obs);
    assert_eq!(activations.len(), 5);
    assert_eq!(activations[0].len(), 4);
    for layer in &activations[1..4] {
        assert_eq!(layer.len(), 8);
        assert!(layer.iter().any(|value| value.abs() > 0.0));
    }
    assert_eq!(activations[4].len(), 2);
}

#[test]
fn scenes_are_clipped_to_the_action_range() {
    let policy = Policy::new(4, &[8, 8], 2, 1);
    let mut rng = Rng::new(3);
    let obs = vec![0.5f32; 4];
    for _ in 0..200 {
        let (action, eps) = policy.act(&obs, 50.0, &mut rng);
        assert_eq!(eps.len(), 2);
        assert!(action.iter().all(|value| (-1.0..=1.0).contains(value)));
    }
    let (action, _) = policy.act(&obs, 0.0, &mut rng);
    assert!(action.iter().all(|value| (-1.0..=1.0).contains(value)));
}

#[test]
fn mismatched_batches_are_rejected_before_any_update() {
    let mut policy = Policy::new(4, &[8, 8], 2, 3);
    let mut adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, 2.5);
    let before = policy.params().to_vec();

    let mut batch = Batch::new(4, 2);
    batch.push_row(&[0.0; 4], &[0.0; 2], 0.0);
    batch.push_row(&[0.0; 4], &[0.0; 2], 1.0);
    batch.returns.pop();
    let err = update(&mut policy, &mut adam, &batch, 0.4, None, None)
        .expect_err("two rows of data with one return must fail");
    assert_eq!(err, "training batch does not match the policy shape");
    assert_eq!(policy.params(), before.as_slice(), "rejected update changed params");

    let wrong_width = Batch::new(3, 2);
    let err = update(&mut policy, &mut adam, &wrong_width, 0.4, None, None)
        .expect_err("empty batches fail");
    assert_eq!(err, "empty training batch");
    assert_eq!(policy.params(), before.as_slice());
}

#[test]
fn kl_backtracking_rolls_back_an_oversized_step() {
    let mut policy = Policy::new(6, &[8, 8], 2, 9);
    let mut explosive = Adam::new(policy.params(), 1e6, (0.9, 0.999), 1e-8, 100.0);
    let before = policy.params().to_vec();
    let mut batch = Batch::new(6, 2);
    for row in 0..8 {
        batch.push_row(&sample_obs(1, 6), &sample_eps(1, 2), row as f32);
    }
    let sigma = 0.4;
    let stats = update(&mut policy, &mut explosive, &batch, sigma, Some(sigma), Some(1e-9))
        .expect("finite update");
    assert_eq!(stats.update_scale, 0.0, "an oversized step must be rejected");
    assert_eq!(stats.policy_kl, 0.0);
    assert_eq!(policy.params(), before.as_slice(), "rollback must restore params");
}

#[test]
fn an_accepted_update_moves_the_policy_and_reports_health() {
    let mut policy = Policy::new(6, &[8, 8], 2, 4);
    let mut adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, 2.5);
    let before = policy.params().to_vec();
    let mut batch = Batch::new(6, 2);
    for row in 0..8 {
        batch.push_row(&sample_obs(1, 6), &sample_eps(1, 2), (row as f32).sin());
    }
    let sigma = sigma_at(10);
    let stats = update(&mut policy, &mut adam, &batch, sigma, Some(sigma), None)
        .expect("finite update");
    assert_eq!(stats.update_scale, 1.0);
    assert!(stats.gradient_norm.is_finite() && stats.gradient_norm > 0.0);
    assert!(stats.parameter_norm.is_finite() && stats.parameter_norm > 0.0);
    assert!(stats.policy_kl >= 0.0 && stats.policy_kl <= neural_racing_ml::MAX_POLICY_KL);
    assert!(stats.gradient_clip > 0.0 && stats.gradient_clip <= 1.0);
    assert_ne!(policy.params(), before.as_slice(), "accepted update must move params");
}

#[test]
fn the_policy_round_trips_through_disk() {
    let policy = Policy::new(19, &[96, 96, 96, 96], 2, 12);
    let directory = std::env::temp_dir().join(format!("nr-policy-{}", std::process::id()));
    std::fs::create_dir_all(&directory).unwrap();
    let path = directory.join("policy.bin");
    policy.save(&path).unwrap();
    let loaded = Policy::load(&path).unwrap();
    assert_eq!(loaded.obs_dim, policy.obs_dim);
    assert_eq!(loaded.act_dim, policy.act_dim);
    assert_eq!(loaded.hidden_dims, policy.hidden_dims);
    assert_eq!(loaded.num_params(), policy.num_params());
    assert_eq!(loaded.params(), policy.params());
    let obs = sample_obs(4, 19);
    assert_eq!(loaded.forward(&obs, 4), policy.forward(&obs, 4));
    std::fs::remove_dir_all(&directory).unwrap();
}

#[test]
fn returns_are_discounted_rewards_to_go() {
    let rewards = [1.0f32, 2.0, 3.0];
    let returns = discounted_returns(&rewards);
    assert!((returns[2] - 3.0).abs() < 1e-6);
    assert!((returns[1] - (2.0 + GAMMA as f32 * 3.0)).abs() < 1e-5);
    assert!(
        (returns[0] - (1.0 + GAMMA as f32 * (2.0 + GAMMA as f32 * 3.0))).abs() < 1e-5
    );
}

#[test]
fn exploration_noise_decays_between_bounds() {
    assert!((sigma_at(1) - SIGMA_START).abs() < 1e-12);
    assert!(sigma_at(1_000_000) <= SIGMA_END + 1e-12);
    assert!(sigma_at(100) < sigma_at(1));
}
