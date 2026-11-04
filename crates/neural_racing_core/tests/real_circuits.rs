mod common;
use neural_racing_core::{Action, Circuit, Env, CAR_LENGTH, CAR_WIDTH, UNITS_PER_METRE};

#[test]
fn real_layouts_have_physical_lengths_and_widths() {
    for circuit in Circuit::ALL {
        let track = circuit.build();
        let measured: f64 = (0..track.centerline.len())
            .map(|i| {
                (track.centerline[(i + 1) % track.centerline.len()] - track.centerline[i]).norm()
            })
            .sum();
        assert!((measured / UNITS_PER_METRE - circuit.length_metres()).abs() < 0.001);
        assert_eq!(track.circuit, Some(circuit));
        for i in 0..track.centerline.len() {
            assert!(
                ((track.outer[i] - track.inner[i]).norm() / UNITS_PER_METRE
                    - circuit.width_metres())
                .abs()
                    < 0.001
            );
        }
        let ratio = CAR_WIDTH / (track.half_width[0] * 2.0);
        assert!((0.13..0.18).contains(&ratio), "{circuit:?}: {ratio}");
        assert!((CAR_LENGTH / UNITS_PER_METRE - 5.020).abs() < 1e-9);
        assert!((CAR_WIDTH / UNITS_PER_METRE - 2.040).abs() < 1e-9);
        println!(
            "{}: {:.3} km, {:.1} m road, car {:.1}% of road width, {} samples",
            circuit.name(),
            measured / UNITS_PER_METRE / 1000.0,
            circuit.width_metres(),
            ratio * 100.0,
            track.centerline.len()
        );
    }
}

#[test]
fn five_full_laps_are_drivable_on_each_circuit() {
    for circuit in Circuit::ALL {
        let mut env = Env::new(circuit.build(), 5);
        let budget = env.max_steps();
        let mut finished = false;
        for _ in 0..budget {
            let hit = env.track.nearest_on_centerline(env.position());
            let action = common::pursuit(&env, 32.0 * UNITS_PER_METRE);
            let result = env.step(action);
            if result.done {
                assert!(
                    result.info.lap_done,
                    "{circuit:?}: off={} gates={} steps={} arc={} speed={} offset={} steer={} yaw={}",
                    result.info.off_track, result.info.checkpoints, env.steps, hit.arc, env.speed / UNITS_PER_METRE, env.lateral_offset / UNITS_PER_METRE, env.steer_angle, env.yaw_rate
                );
                assert_eq!(result.info.checkpoints, 40);
                assert!((env.distance - 5.0 * env.track.lap_length).abs() < 80.0);
                finished = true;
                println!(
                    "{}: five laps in {:.1}s / {:.1}s budget",
                    circuit.name(),
                    env.steps as f64 * 0.05,
                    budget as f64 * 0.05
                );
                break;
            }
        }
        assert!(finished);
    }
}

#[test]
fn layout_matches_geographic_reference_without_reflection_or_axis_stretch() {
    for (circuit, expected_length, expected_width) in [
        (Circuit::Monza, 5793.0, 12.0),
        (Circuit::Silverstone, 5891.0, 15.0),
        (Circuit::RedBullRing, 4326.0, 12.5),
    ] {
        let track = circuit.build();
        assert!((track.lap_length / UNITS_PER_METRE - expected_length).abs() < 0.001);
        assert!((track.half_width[0] * 2.0 / UNITS_PER_METRE - expected_width).abs() < 0.001);
        let coords = circuit.coordinates();
        let (lon, lat) = coords[0];
        let to_radians = std::f64::consts::PI / 180.0;
        let mut raw: Vec<_> = coords
            .iter()
            .map(|&(x, y)| neural_racing_core::Vec2 {
                x: (x - lon) * to_radians * 6_371_000.0 * (lat * to_radians).cos(),
                y: -(y - lat) * to_radians * 6_371_000.0,
            })
            .collect();
        if (raw[0] - raw[raw.len() - 1]).norm() < 0.01 {
            raw.pop();
        }
        let raw_length: f64 = (0..raw.len())
            .map(|i| (raw[(i + 1) % raw.len()] - raw[i]).norm())
            .sum();
        // Independent raw-polyline calibration. Smoothing changes length a
        // little; all original landmarks must still be within 8 metres.
        let scale = expected_length / raw_length * UNITS_PER_METRE;
        let mut worst: f64 = 0.0;
        for point in raw {
            let point = point * scale;
            let hit = track.nearest_on_centerline(point);
            let error = (point - track.arc_point(hit.arc)).norm() / UNITS_PER_METRE;
            worst = worst.max(error);
        }
        assert!(worst < 8.0, "{circuit:?}: source landmark drift {worst} m");
        println!(
            "{}: worst raw-map landmark displacement {:.2} m",
            circuit.name(),
            worst
        );
    }
}

#[test]
fn timeout_budget_and_cached_maps_are_independent() {
    let mut first = Circuit::Monza.build();
    first.centerline[0].x += 1000.0;
    let second = Circuit::Monza.build();
    assert!((first.centerline[0].x - second.centerline[0].x - 1000.0).abs() < 1e-9);
    let mut env = Env::new(second, 2);
    assert!(env.max_steps() > 1200);
    env.steps = env.max_steps() - 1;
    let step = env.step(Action {
        steer: 0.0,
        throttle: 0.0,
    });
    assert!(step.done && !step.info.off_track && !step.info.lap_done);
    let position = env.position();
    let after = env.step(Action {
        steer: 1.0,
        throttle: 1.0,
    });
    assert!(after.done);
    assert_eq!(after.reward, 0.0);
    assert_eq!(env.position(), position);
    assert_eq!(env.steps, env.max_steps());
}

#[test]
fn practice_starts_still_require_one_complete_lap() {
    for checkpoint in [0, 2, 4, 6] {
        let mut env = Env::new(Circuit::Monza.build(), 1);
        env.reset_at_checkpoint(checkpoint);
        for _ in 0..env.max_steps() {
            let step = env.step(common::pursuit(&env, 20.0 * UNITS_PER_METRE));
            if step.done {
                break;
            }
        }
        assert!(
            env.lap_done,
            "practice start {checkpoint}: off={} distance={}",
            env.off_track, env.distance
        );
        assert_eq!(env.next_checkpoint, 8);
        assert!((env.distance - env.track.lap_length).abs() < 20.0);
    }
}
