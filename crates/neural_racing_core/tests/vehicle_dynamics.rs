//! Physical behavior at the actual Env::step seam, in metres and seconds.
use neural_racing_core::{Action, Circuit, Env, DECISION_DT, UNITS_PER_METRE, WHEELBASE};

fn car(speed_mps: f64) -> Env {
    let mut env = Env::new(Circuit::Monza.build(), 1);
    env.vx = speed_mps * UNITS_PER_METRE;
    env.vy = 0.0;
    env.yaw_rate = 0.0;
    env
}

#[test]
fn keyboard_turn_from_start_does_not_spin_out() {
    for direction in [-1.0, 1.0] {
        let mut env = car(5.0);
        let mut worst_slip: f64 = 0.0;
        for tick in 0..20 {
            let result = env.step(Action {
                steer: direction,
                throttle: 1.0,
            });
            // CG sideslip includes ordinary rear-axle rolling in a tight turn.
            // Rear-axle slip measures the loss of grip that starts a spin.
            let rear_slip = ((env.vy - WHEELBASE * 0.5 * env.yaw_rate) / env.vx.abs().max(1.0))
                .atan()
                .to_degrees();
            worst_slip = worst_slip.max(rear_slip.abs());
            assert!(
                rear_slip.abs() < 12.0,
                "keyboard launch at {:.2}s: rear slip={rear_slip:.2}°, CG slip={:.2}°, speed={:.2}m/s, yaw={:.2}rad/s, steer={:.3}, grip={:?}",
                (tick + 1) as f64 * DECISION_DT, env.slip_angle_deg,
                env.speed / UNITS_PER_METRE, env.yaw_rate, env.steer_angle, env.tyre_usage
            );
            assert!(!result.done, "launch left the road at tick {tick}");
        }
        println!("direction={direction}: worst rear slip={worst_slip:.2}°");
    }
}

#[test]
fn steering_cannot_snap_between_opposite_locks() {
    let mut env = car(2.0);
    for i in 0..12 {
        let before = env.steer_angle;
        env.step(Action {
            steer: if i % 2 == 0 { 1.0 } else { -1.0 },
            throttle: 0.0,
        });
        let rate = (env.steer_angle - before).abs() / DECISION_DT;
        assert!(rate <= 1.5, "road-wheel steering snaps at {rate:.2} rad/s");
    }
}

#[test]
fn low_speed_turn_rolls_about_the_rear_axle() {
    let mut env = car(2.0);
    for _ in 0..12 {
        let rear_before = env.position()
            - neural_racing_core::Vec2 {
                x: WHEELBASE * 0.5,
                y: 0.0,
            }
            .rotated(env.heading);
        let heading_before = env.heading;
        env.step(Action {
            steer: 0.7,
            throttle: 0.05,
        });
        let rear_after = env.position()
            - neural_racing_core::Vec2 {
                x: WHEELBASE * 0.5,
                y: 0.0,
            }
            .rotated(env.heading);
        let heading_mid =
            heading_before + neural_racing_core::wrap_angle(env.heading - heading_before) * 0.5;
        let normal = neural_racing_core::Vec2 { x: 0.0, y: 1.0 }.rotated(heading_mid);
        let sideways = (rear_after - rear_before).dot(normal) / DECISION_DT / UNITS_PER_METRE;
        assert!(
            sideways.abs() < 0.15,
            "rear axle actually moves sideways at {sideways} m/s"
        );
    }
    let rear_lateral_mps = (env.vy - WHEELBASE * 0.5 * env.yaw_rate) / UNITS_PER_METRE;
    assert!(
        env.yaw_rate > 0.01,
        "front steering should turn a moving car"
    );
    assert!(
        rear_lateral_mps.abs() < 0.15,
        "rear wheels skid sideways at {rear_lateral_mps:.3} m/s during a slow turn"
    );
}

#[test]
fn acceleration_and_braking_are_in_road_car_ranges() {
    let mut accelerating = car(0.0);
    for _ in 0..10 {
        accelerating.step(Action {
            steer: 0.0,
            throttle: 1.0,
        });
    }
    let acceleration = accelerating.vx / UNITS_PER_METRE / (10.0 * DECISION_DT);
    assert!(
        (0.5..=11.0).contains(&acceleration),
        "average acceleration {acceleration:.1} m/s²"
    );
    let mut braking = car(20.0);
    for _ in 0..10 {
        braking.step(Action {
            steer: 0.0,
            throttle: -1.0,
        });
    }
    let deceleration = (20.0 - braking.vx / UNITS_PER_METRE) / (10.0 * DECISION_DT);
    assert!(
        (3.0..=18.0).contains(&deceleration),
        "average braking {deceleration:.1} m/s²"
    );
    for _ in 0..100 {
        braking.step(Action {
            steer: 0.0,
            throttle: -1.0,
        });
    }
    assert!(
        braking.vx.abs() < 0.01,
        "holding the brake must not automatically reverse the car"
    );
}

#[test]
fn stationary_steering_does_not_rotate_the_chassis() {
    let mut env = car(0.0);
    let heading = env.heading;
    let pos = env.position();
    for _ in 0..20 {
        env.step(Action {
            steer: 1.0,
            throttle: 0.0,
        });
    }
    assert!((env.heading - heading).abs() < 1e-8);
    assert!((env.position() - pos).norm() < 1e-8);
}

#[test]
fn oscillating_commands_cost_more_than_holding_a_settled_line() {
    let mut steady = car(15.0);
    let mut weaving = steady.clone();
    let mut smooth_cost = 0.0;
    let mut weave_cost = 0.0;
    for step in 0..12 {
        smooth_cost += steady
            .step(Action {
                steer: 0.0,
                throttle: 0.0,
            })
            .info
            .reward_components
            .smoothness;
        weave_cost += weaving
            .step(Action {
                steer: if step % 2 == 0 { 0.5 } else { -0.5 },
                throttle: 0.0,
            })
            .info
            .reward_components
            .smoothness;
    }
    assert!(
        weave_cost < smooth_cost - 0.5,
        "weaving={weave_cost}, steady={smooth_cost}"
    );
    assert_eq!(weaving.obs.len(), neural_racing_core::OBS_DIM);
    assert_eq!(weaving.obs[16], -0.5, "policy sees its previous command");
    assert!(weaving.obs.iter().all(|v| v.is_finite()));
}

#[test]
fn tire_forces_build_over_time_and_respect_each_contact_patch_limit() {
    let mut env = car(20.0);
    for _ in 0..20 {
        env.step(Action {
            steer: 0.15,
            throttle: 0.0,
        });
        assert!(env.tyre_loads.iter().all(|f| f.is_finite() && *f > 0.0));
        assert!(env.tyre_usage.iter().all(|u| (0.0..=1.000001).contains(u)));
    }
    assert!(
        env.tyre_loads[0] > env.tyre_loads[1],
        "outside front wheel gains load in a right turn"
    );
    assert!(env.tyre_lateral.iter().any(|f| f.abs() > 100.0));
}
