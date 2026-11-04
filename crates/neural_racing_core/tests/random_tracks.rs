use neural_racing_core::{
    Env, Track, Vec2, ARENA_HEIGHT, ARENA_WIDTH, CHECKPOINT_COUNT, DEFAULT_LAPS, MAX_SPEED,
    NUM_RAYS, OBS_DIM, RAY_RANGE, STATE_FEATURES,
};

const SEEDS: std::ops::Range<u64> = 0..160;

fn dist_to_centerline(track: &Track, point: Vec2) -> f64 {
    track.nearest_on_centerline(point).lateral.abs()
}

#[test]
fn every_seed_builds_a_drivable_loop() {
    for seed in SEEDS {
        let track = Track::random(seed);
        assert!(
            (700.0..=1700.0).contains(&track.lap_length),
            "seed {seed}: lap length {} outside the generator budget",
            track.lap_length
        );
        assert_eq!(track.centerline.len(), track.outer.len());
        assert_eq!(track.gates().len(), CHECKPOINT_COUNT);
        assert!(
            dist_to_centerline(&track, track.start) < 1.0,
            "seed {seed}: start is not on the centerline"
        );
        assert!(
            (track.start_heading - track.arc_tangent(0.0).angle()).abs() < 1e-9,
            "seed {seed}: start heading does not follow the track"
        );

        // Walls follow the local width, which varies around the lap.
        let (narrow, wide) = track.width_range();
        assert!(
            narrow > 15.0 && wide < 46.0,
            "seed {seed}: half widths {narrow}..{wide} are outside the sane road range"
        );
        for (index, point) in track.outer.iter().enumerate() {
            let offset = track.nearest_on_centerline(*point).lateral.abs();
            let width = track.half_width[index];
            assert!(
                (offset - width).abs() < 2.5,
                "seed {seed}: outer[{index}] is {offset} from a {width} wide road"
            );
        }
        for (index, point) in track.inner.iter().enumerate() {
            let offset = track.nearest_on_centerline(*point).lateral.abs();
            let width = track.half_width[index];
            assert!(
                (offset - width).abs() < 2.5,
                "seed {seed}: inner[{index}] is {offset} from a {width} wide road"
            );
        }
        assert!(
            track
                .gates()
                .iter()
                .all(|gate| gate.half_width >= narrow && gate.half_width <= wide),
            "seed {seed}: a gate ignores the local road width"
        );

        // The track fits inside the drawable arena.
        assert!(
            track.bounds.min_x >= 0.0
                && track.bounds.min_y >= 0.0
                && track.bounds.max_x <= ARENA_WIDTH
                && track.bounds.max_y <= ARENA_HEIGHT,
            "seed {seed}: track {:?} leaves the arena",
            track.bounds
        );

        // Checkpoints are evenly spaced around one lap and reachable.
        for (index, gate) in track.gates().iter().enumerate() {
            let arc = track.nearest_on_centerline(gate.center).arc;
            let expected = (index + 1) as f64 * track.lap_length / CHECKPOINT_COUNT as f64;
            let wrap = |value: f64| {
                (value + track.lap_length / 2.0).rem_euclid(track.lap_length)
                    - track.lap_length / 2.0
            };
            assert!(
                wrap(arc - expected).abs() < 2.0,
                "seed {seed}: gate {index} at arc {arc} is not at {expected}"
            );
            assert!(dist_to_centerline(&track, gate.center) < 1.0);
        }
    }
}

#[test]
fn tracks_stay_small_and_widths_vary() {
    let mut varying = 0;
    for seed in SEEDS {
        let track = Track::random(seed);
        let span_x = track.bounds.max_x - track.bounds.min_x;
        let span_y = track.bounds.max_y - track.bounds.min_y;
        assert!(
            span_x < ARENA_WIDTH * 0.78 && span_y < ARENA_HEIGHT * 0.80,
            "seed {seed}: track spans {span_x:.0}x{span_y:.0}, which fills the arena"
        );
        assert!(
            span_x > 200.0 && span_y > 200.0,
            "seed {seed}: track spans only {span_x:.0}x{span_y:.0}"
        );
        let (narrow, wide) = track.width_range();
        assert!(
            narrow > 15.0,
            "seed {seed}: narrowest half width {narrow} is not a road"
        );
        if wide - narrow > 0.25 * wide {
            varying += 1;
        }
    }
    assert!(
        varying > SEEDS.end / 2,
        "only {varying} of {} tracks vary their width",
        SEEDS.end
    );
}

#[test]
fn checkpoint_crossings_respect_the_local_width() {
    let track = Track::random(5);
    let gate = track.gates()[0];
    let normal = Vec2 {
        x: -gate.tangent.y,
        y: gate.tangent.x,
    };
    let offset = |distance: f64| {
        let start = gate.center + normal * distance - gate.tangent * 3.0;
        let end = gate.center + normal * distance + gate.tangent * 3.0;
        track.checkpoint_crossing(0, start, end)
    };
    assert_eq!(offset(0.0), 1, "crossing the middle of a gate counts");
    assert_eq!(offset(gate.half_width * 0.9), 1, "the road is drivable");
    assert_eq!(offset(gate.half_width * 1.1), 0, "outside is not a crossing");
}

#[test]
fn distinct_seeds_produce_distinct_tracks() {
    let a = Track::random(1);
    let b = Track::random(2);
    let max_drift = a
        .centerline
        .iter()
        .zip(&b.centerline)
        .map(|(lhs, rhs)| (*lhs - *rhs).norm())
        .fold(0.0, f64::max);
    assert!(max_drift > 20.0, "two seeds drew the same loop");
}

#[test]
fn rays_always_report_a_wall_inside_the_sensor_range() {
    for seed in 0..40 {
        let track = Track::random(seed);
        let mut env = Env::new(track, DEFAULT_LAPS);
        for _ in 0..120 {
            env.step(neural_racing_core::Action {
                steer: 0.0,
                throttle: 0.4,
            });
            assert_eq!(env.rays.len(), NUM_RAYS);
            for ray in env.rays {
                assert!(
                    ray > 0.0 && ray <= RAY_RANGE,
                    "seed {seed}: ray distance {ray} outside (0, {RAY_RANGE}]"
                );
            }
            assert!(env.obs.iter().all(|value| value.is_finite()));
            assert_eq!(env.obs.len(), OBS_DIM);
        }
    }
}

#[test]
fn observations_are_normalized_and_heading_aware() {
    let env = Env::new(Track::random(7), DEFAULT_LAPS);
    assert_eq!(env.obs.len(), OBS_DIM);
    let speed = env.obs[0];
    assert!((speed - env.vx as f32 / MAX_SPEED as f32).abs() < 1e-6);
    // Facing along the start tangent: cos(heading error) is 1.
    assert!((env.obs[6] - 1.0).abs() < 1e-5);
    assert!(env.obs[5].abs() < 1e-5);
    assert!((env.obs[4] as f64).abs() < 1.0);
    for ray in &env.obs[STATE_FEATURES..] {
        assert!((0.0..=1.0).contains(ray));
    }
}

#[test]
fn env_is_deterministic_for_a_seed() {
    let run = |seed: u64| {
        let mut env = Env::new(Track::random(seed), DEFAULT_LAPS);
        let mut trace = Vec::new();
        for step in 0..200 {
            let steer = ((step as f64) * 0.05).sin() * 0.4;
            let outcome = env.step(neural_racing_core::Action {
                steer,
                throttle: 0.6,
            });
            trace.push((env.x, env.y, env.heading, outcome.reward));
        }
        trace
    };
    assert_eq!(run(42), run(42));
    assert_ne!(run(42), run(43));
}
