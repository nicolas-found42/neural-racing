mod common;
use neural_racing_core::{
    Action, Env, StepOutcome, Track, CHECKPOINT_COUNT, DEFAULT_LAPS, FAST_LAP_BONUS, LAP_BONUS,
    MAX_STEPS,
};

/// Pure-pursuit driver that brakes for the curvature ahead: proves the
/// generated tracks and the lap/checkpoint rules are physically completable
/// without a learned policy.
fn drive(env: &mut Env, cruise: f64) -> StepOutcome {
    env.step(common::pursuit(env, cruise))
}

#[test]
fn a_pursuit_driver_completes_two_laps_on_random_tracks() {
    for seed in 0..40 {
        let mut env = Env::new(Track::random(seed), DEFAULT_LAPS);
        let mut crossings = Vec::new();
        let mut last = 0;
        let mut outcome = None;
        let mut reward_sum = 0.0;
        let mut lap_bonus_sum = 0.0;
        let mut component_sum = 0.0;
        for _ in 0..MAX_STEPS {
            let step = drive(&mut env, 240.0);
            reward_sum += step.reward;
            component_sum += step.info.reward_components.total();
            lap_bonus_sum += step.info.reward_components.lap;
            if step.info.checkpoints != last {
                crossings.push(step.info.checkpoints);
                last = step.info.checkpoints;
            }
            if step.done {
                outcome = Some(step);
                break;
            }
        }
        let step = outcome.unwrap_or_else(|| panic!("seed {seed}: race never finished"));
        assert!(!step.info.off_track, "seed {seed}: pursuit driver crashed");
        assert!(
            step.info.lap_done,
            "seed {seed}: only {} of {} gates in {} steps",
            step.info.checkpoints,
            env.total_gates(),
            step.info.steps
        );
        assert_eq!(step.info.laps_completed, DEFAULT_LAPS);
        assert_eq!(
            crossings,
            (1..=CHECKPOINT_COUNT as i32 * DEFAULT_LAPS).collect::<Vec<_>>(),
            "seed {seed}: gates were not crossed once each in order"
        );
        assert!(
            (step.info.distance - 2.0 * env.track.lap_length).abs() < 40.0,
            "seed {seed}: distance {} vs two laps {}",
            step.info.distance,
            2.0 * env.track.lap_length
        );

        assert!(
            (reward_sum - component_sum).abs() < 1e-9,
            "seed {seed}: reward {reward_sum} does not equal components {component_sum}"
        );
        assert!(
            lap_bonus_sum >= 2.0 * LAP_BONUS,
            "seed {seed}: two laps paid {lap_bonus_sum} in lap bonuses"
        );
        assert!(lap_bonus_sum <= 2.0 * LAP_BONUS + FAST_LAP_BONUS + 1e-9);
        assert!(step.info.steps < MAX_STEPS, "seed {seed}: ride is too slow");
    }
}

#[test]
fn a_finished_race_stays_terminal_and_pays_nothing_more() {
    let mut env = Env::new(Track::random(3), 1);
    let mut finished = None;
    for _ in 0..MAX_STEPS {
        let step = drive(&mut env, 240.0);
        if step.done && step.info.lap_done {
            finished = Some(step);
            break;
        }
    }
    let finished = finished.expect("one lap completed");
    for _ in 0..3 {
        let extra = env.step(Action {
            steer: 0.0,
            throttle: 0.0,
        });
        assert!(extra.done);
        assert_eq!(extra.reward, 0.0);
        assert_eq!(extra.info.steps, finished.info.steps);
        assert_eq!(extra.info.checkpoints, env.total_gates());
        assert_eq!(extra.info.reward_components.lap, 0.0);
    }
}

#[test]
fn a_crawl_times_out_without_finishing() {
    let mut env = Env::new(Track::random(11), DEFAULT_LAPS);
    let mut outcome = drive(&mut env, 34.0);
    while !outcome.done {
        outcome = drive(&mut env, 34.0);
    }
    assert_eq!(outcome.info.steps, MAX_STEPS);
    assert!(!outcome.info.lap_done);
    assert!(!outcome.info.off_track);
}

#[test]
fn leaving_the_road_ends_the_race_immediately() {
    let mut env = Env::new(Track::random(5), DEFAULT_LAPS);
    let mut crashed = false;
    for _ in 0..MAX_STEPS {
        let step = env.step(Action {
            steer: 1.0,
            throttle: 1.0,
        });
        if step.done {
            crashed = step.info.off_track;
            break;
        }
    }
    assert!(crashed, "full lock must leave the road");
    assert!(!env.lap_done);
}

#[test]
fn episode_budget_covers_two_slow_laps() {
    // Two laps at 115 px/s need ~50 s of simulated time, well under MAX_STEPS.
    let budget_seconds = MAX_STEPS as f64 * neural_racing_core::DECISION_DT;
    assert!(
        budget_seconds >= 55.0,
        "budget {budget_seconds}s is too tight"
    );
}
