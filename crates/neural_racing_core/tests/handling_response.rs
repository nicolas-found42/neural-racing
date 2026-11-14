//! Outcome checks for responsive GT-style handling at the shared Env::step seam.
//! These are broad gameplay acceptance bounds, not measurements of GT4 or BMW.
use neural_racing_core::{vehicle::GRAVITY, Action, Circuit, Env, DECISION_DT, UNITS_PER_METRE};

fn skidpad(speed_mps: f64) -> Env {
    let mut track = Circuit::Monza.build();
    // Keep real Env::step, sensors and forces. Only widen the terminal road
    // boundary: an ordinary circuit wall must not truncate a tire maneuver.
    track.half_width.fill(500.0 * UNITS_PER_METRE);
    let mut env = Env::new(track, 10);
    env.vx = speed_mps * UNITS_PER_METRE;
    env.vy = 0.0;
    env.yaw_rate = 0.0;
    env
}

#[test]
fn full_throttle_reaches_100_kmh_with_gt_class_urgency() {
    let mut env = skidpad(0.0);
    let target = 100.0 / 3.6;
    let mut elapsed = 0.0;
    while env.speed / UNITS_PER_METRE < target && elapsed < 6.0 {
        let result = env.step(Action {
            steer: 0.0,
            throttle: 1.0,
        });
        assert!(
            !result.done,
            "skidpad ended before acceleration could be measured"
        );
        elapsed += DECISION_DT;
    }
    println!(
        "0–100 km/h: {elapsed:.2}s, speed={:.2} m/s",
        env.speed / UNITS_PER_METRE
    );
    assert!(
        elapsed <= 4.0,
        "0–100 km/h takes {elapsed:.2}s; GT-style acceleration should take at most 4.0s"
    );
}

#[test]
fn road_speed_full_steering_can_access_a_racing_cars_cornering_grip() {
    for direction in [-1.0, 1.0] {
        let mut env = skidpad(30.0);
        let mut sum_lateral = 0.0;
        let mut samples = 0;
        for tick in 0..40 {
            let result = env.step(Action {
                steer: direction,
                throttle: 0.0,
            });
            assert!(
                !result.done,
                "skidpad ended before turn response could settle"
            );
            if tick >= 20 {
                sum_lateral += env.lateral_accel.abs();
                samples += 1;
            }
        }
        let lateral_g = sum_lateral / samples as f64 / GRAVITY;
        println!("30 m/s turn={direction}: sustained lateral={lateral_g:.3}g, yaw={:.3}rad/s, speed={:.2}m/s", env.yaw_rate, env.speed / UNITS_PER_METRE);
        assert!(
            lateral_g >= 0.95,
            "full road-speed steering reaches only {lateral_g:.3}g; it should allow at least 0.95g"
        );
    }
}

#[test]
fn modest_steering_keeps_acceleration_and_release_remains_controllable() {
    let mut straight = skidpad(10.0);
    let mut turning = skidpad(10.0);
    for _ in 0..20 {
        straight.step(Action {
            steer: 0.0,
            throttle: 1.0,
        });
        turning.step(Action {
            steer: 0.2,
            throttle: 1.0,
        });
    }
    let straight_gain = straight.speed / UNITS_PER_METRE - 10.0;
    let turning_gain = turning.speed / UNITS_PER_METRE - 10.0;
    println!("1s 10m/s acceleration: straight gain={straight_gain:.2}, steer=.2 gain={turning_gain:.2} m/s");
    assert!(
        turning_gain >= 0.8 * straight_gain,
        "a modest turn leaves only {:.1}% of straight acceleration",
        turning_gain / straight_gain * 100.0
    );
    for _ in 0..40 {
        turning.step(Action {
            steer: 0.0,
            throttle: 0.0,
        });
    }
    assert!(
        turning.yaw_rate.abs() < 0.05,
        "releasing steering leaves yaw={} rad/s",
        turning.yaw_rate
    );
}
