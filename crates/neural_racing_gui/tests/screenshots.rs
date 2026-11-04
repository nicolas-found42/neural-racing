//! Headless render of the real UI: drives the same `App::render` path eframe
//! uses, rasterises each mode with wgpu, and writes PNGs. No window opens.

use std::path::PathBuf;

use egui::{Key, Vec2};
use egui_kittest::Harness;
use neural_racing_gui::{App, Mode};

const SIZE: Vec2 = Vec2::new(1264.0, 900.0);
/// Screenshots live in the workspace root so the README can embed them.
const OUT_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../docs/screenshots");

fn harness() -> Harness<'static, App> {
    let mut app = App::new(None, PathBuf::new());
    app.configure(None, Some(Mode::Watch), None);
    Harness::builder()
        .with_size(SIZE)
        .with_pixels_per_point(1.0)
        .with_step_dt(1.0 / 60.0)
        .with_accessibility_check(false)
        .with_context_setup(App::install_style)
        .build_ui_state(|ui, app: &mut App| app.render(ui), app)
}

fn save(harness: &mut Harness<'static, App>, name: &str) {
    let image = harness.render().expect("wgpu renders the frame");
    assert_eq!(image.width(), SIZE.x as u32);
    assert_eq!(image.height(), SIZE.y as u32);
    std::fs::create_dir_all(OUT_DIR).expect("screenshot directory");
    let path = PathBuf::from(OUT_DIR).join(format!("{name}.png"));
    image.save(&path).expect("screenshot writes");
    let bytes = std::fs::read(&path).expect("screenshot is readable");
    assert!(bytes.len() > 20_000, "{name}.png looks blank");
}

#[test]
fn watch_mode_uses_the_current_live_model() {
    let mut harness = harness();
    harness
        .state_mut()
        .configure(None, Some(Mode::Train), Some(1000.0));
    for _ in 0..40 {
        harness.run_steps(1);
        std::thread::sleep(std::time::Duration::from_millis(20));
    }
    assert!(harness
        .state()
        .training_stats()
        .is_some_and(|s| s.updates > 0));
    harness
        .state_mut()
        .configure(None, Some(Mode::Watch), Some(1.0));
    harness.run_steps(8);
    assert!(harness.state().race().env.steps > 0);
    assert!(harness.state().race().env.last_action.throttle.abs() > 0.0);
    save(&mut harness, "watch");
}

#[test]
fn arrow_keys_drive_the_car_in_manual_mode() {
    // Find a start straight enough to hold the throttle on; the screenshot
    // should show the car driving, not the off-track banner.
    let mut seed_choice = None;
    for seed in 1..400u64 {
        let mut probe = harness();
        probe
            .state_mut()
            .configure(Some(seed), Some(Mode::Manual), Some(1.0));
        probe.run_steps(2);
        let before = probe.state().race().env.position();
        probe.key_down(Key::ArrowUp);
        probe.run_steps(70);
        probe.key_up(Key::ArrowUp);
        let race = probe.state().race();
        if !race.is_over() && (race.env.position() - before).norm() > 40.0 {
            seed_choice = Some(seed);
            break;
        }
    }
    let seed = seed_choice.expect("no seed left the car on the road under throttle");
    let mut harness = harness();
    harness
        .state_mut()
        .configure(Some(seed), Some(Mode::Manual), Some(1.0));
    harness.run_steps(2);
    harness.key_down(Key::ArrowUp);
    harness.run_steps(70);
    harness.key_up(Key::ArrowUp);
    let race = harness.state().race();
    assert!(!race.is_over(), "the chosen manual run left the road");
    assert!(race.env.steps > 15, "the manual race did not advance");
    save(&mut harness, "manual");
}

/// Train mode is driven by real trainer threads, so the generation counters in
/// `train.png` depend on how many generations landed before the capture; the
/// committed image is one such frame, not a byte-stable golden file.
#[test]
fn live_training_streams_generations_into_the_dashboard() {
    let mut harness = harness();
    harness
        .state_mut()
        .configure(None, Some(Mode::Train), Some(1000.0));
    for _ in 0..100 {
        harness.run_steps(5);
        std::thread::sleep(std::time::Duration::from_millis(25));
    }
    let stats = harness
        .state()
        .training_stats()
        .expect("a generation arrived");
    assert!(stats.episodes > 0, "training reported no episodes");
    assert!(stats.updates > 0);
    assert_eq!(stats.cars.len(), 4);
    assert!(stats
        .cars
        .iter()
        .all(|c| c.track.circuit == Some(neural_racing_core::Circuit::Monza)));
    save(&mut harness, "train");
}

#[test]
fn real_circuits_render_at_car_scale_and_overview() {
    use neural_racing_core::Circuit;
    for circuit in Circuit::ALL {
        let mut harness = harness();
        harness.state_mut().select_circuit(Some(circuit));
        harness
            .state_mut()
            .configure(None, Some(Mode::Manual), Some(1.0));
        harness.run_steps(2);
        harness.key_press(Key::Space);
        harness.run_steps(2);
        save(&mut harness, &format!("{}-follow", circuit.slug()));
        harness.key_press(Key::V);
        harness.run_steps(2);
        save(&mut harness, &format!("{}-overview", circuit.slug()));
        assert_eq!(harness.state().race().env.track.circuit, Some(circuit));
        harness.key_press(Key::R);
        harness.run_steps(2);
        assert_eq!(harness.state().race().env.track.circuit, Some(circuit));
        let old = harness.state().race().env.position();
        harness.key_press(Key::Space);
        harness.key_down(Key::ArrowUp);
        harness.run_steps(50);
        harness.key_up(Key::ArrowUp);
        assert!((harness.state().race().env.position() - old).norm() > 30.0);
        assert!(!harness.state().race().is_over());
        harness.key_press(Key::N);
        harness.run_steps(2);
        assert_ne!(harness.state().race().env.track.circuit, Some(circuit));
    }
}

#[test]
fn front_wheel_steering_is_visible_in_manual_mode() {
    let mut harness = harness();
    harness
        .state_mut()
        .configure(None, Some(Mode::Manual), Some(1.0));
    harness.key_down(Key::ArrowRight);
    harness.run_steps(25);
    harness.key_up(Key::ArrowRight);
    assert!(harness.state().race().env.steer_angle > 0.05);
    harness.key_press(Key::Space);
    harness.run_steps(2);
    save(&mut harness, "front-wheel-steering");
}

#[test]
fn circuit_controls_fit_the_minimum_window() {
    use neural_racing_core::Circuit;
    let mut app = App::new(None, PathBuf::new());
    app.select_circuit(Some(Circuit::Silverstone));
    app.configure(None, Some(Mode::Manual), Some(1.0));
    let mut harness = Harness::builder()
        .with_size(Vec2::new(1000.0, 720.0))
        .with_pixels_per_point(1.0)
        .with_accessibility_check(false)
        .with_context_setup(App::install_style)
        .build_ui_state(|ui, app: &mut App| app.render(ui), app);
    harness.run_steps(2);
    harness.key_press(Key::Space);
    harness.run_steps(2);
    let image = harness.render().unwrap();
    assert_eq!((image.width(), image.height()), (1000, 720));
    image
        .save(PathBuf::from(OUT_DIR).join("silverstone-compact.png"))
        .unwrap();
    harness.key_press(Key::V);
    harness.run_steps(2);
    let image = harness.render().unwrap();
    image
        .save(PathBuf::from(OUT_DIR).join("silverstone-compact-overview.png"))
        .unwrap();
    assert_eq!(
        harness.state().race().env.track.circuit,
        Some(Circuit::Silverstone)
    );
}

#[test]
fn cars_stay_live_at_every_speed_and_pause_freezes_learning() {
    use std::time::{Duration, Instant};
    let app = App::new(None, PathBuf::new());
    let mut harness = Harness::builder()
        .with_size(SIZE)
        .with_pixels_per_point(1.0)
        .with_accessibility_check(false)
        .with_context_setup(App::install_style)
        .build_ui_state(|ui, app: &mut App| app.render(ui), app);
    for speed in [0.25, 1.0, 16.0, 1000.0] {
        harness.state_mut().configure(None, None, Some(speed));
        let before = harness.state().training_stats().map_or(0, |s| s.decisions);
        let deadline = Instant::now() + Duration::from_secs(10);
        while Instant::now() < deadline {
            harness.run_steps(1);
            std::thread::sleep(Duration::from_millis(16));
            if harness
                .state()
                .training_stats()
                .is_some_and(|s| s.decisions > before + 4)
            {
                break;
            }
        }
        let snapshot = harness.state().training_stats().expect("live snapshot");
        assert!(snapshot.decisions > before + 4, "cars stopped at {speed}x");
        assert_eq!(
            harness.state().race().env.position(),
            snapshot.cars[0].position()
        );
        save(&mut harness, &format!("learning-{speed}x"));
    }
    harness.key_press(Key::Space);
    for _ in 0..15 {
        harness.run_steps(1);
        std::thread::sleep(Duration::from_millis(20));
    }
    let paused = harness.state().training_stats().unwrap();
    assert!(paused.paused);
    let decisions = paused.decisions;
    let position = harness.state().race().env.position();
    for _ in 0..10 {
        harness.run_steps(1);
        std::thread::sleep(Duration::from_millis(20));
    }
    assert_eq!(
        harness.state().training_stats().unwrap().decisions,
        decisions
    );
    assert_eq!(harness.state().race().env.position(), position);
    harness.key_press(Key::Space);
    for _ in 0..15 {
        harness.run_steps(1);
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(harness.state().training_stats().unwrap().decisions > decisions);
}

/// Explicit performance acceptance run on the user's hardware, outside the
/// ordinary regression suite. Renders live at every iteration for 30s/circuit.
#[test]
#[ignore = "90-second sustained live-render benchmark; run with --release --ignored"]
fn sustained_live_render_at_1000x() {
    benchmark_live_render(1000.0, "1000x");
}

#[test]
#[ignore = "90-second uncapped capacity measurement; run with --release --ignored"]
fn sustained_live_render_at_max_speed() {
    benchmark_live_render(f32::INFINITY, "max");
}

fn benchmark_live_render(speed: f32, label: &str) {
    use neural_racing_core::{Circuit, DECISION_DT};
    use std::time::{Duration, Instant};
    for circuit in Circuit::ALL {
        let mut app = App::new(None, PathBuf::new());
        app.select_circuit(Some(circuit));
        app.configure(None, Some(Mode::Train), Some(speed));
        let mut harness = Harness::builder()
            .with_size(SIZE)
            .with_pixels_per_point(1.0)
            .with_accessibility_check(false)
            .with_context_setup(App::install_style)
            .build_ui_state(|ui, app: &mut App| app.render(ui), app);
        harness.render().unwrap(); // warm GPU initialization before timing
        harness.run_steps(1); // consume the current worker state after GPU initialization
        let start = Instant::now();
        let initial = harness.state().training_stats().map_or(0, |s| s.decisions);
        let mut frame_times = Vec::new();
        let mut moving_frames = 0;
        let mut last_decisions = initial;
        while start.elapsed() < Duration::from_secs(30) {
            let frame = Instant::now();
            harness.run_steps(1);
            harness.render().unwrap();
            let snapshot = harness.state().training_stats().unwrap();
            assert!(snapshot.error.is_none());
            assert_eq!(
                harness.state().race().env.position(),
                snapshot.cars[0].position()
            );
            assert!(snapshot
                .cars
                .iter()
                .all(|c| c.track.circuit == Some(circuit)));
            if snapshot.decisions != last_decisions {
                moving_frames += 1;
            }
            last_decisions = snapshot.decisions;
            frame_times.push(frame.elapsed().as_secs_f64());
            if let Some(rest) = Duration::from_secs_f64(1.0 / 60.0).checked_sub(frame.elapsed()) {
                std::thread::sleep(rest);
            }
        }
        let elapsed = start.elapsed().as_secs_f64();
        let snapshot = harness.state().training_stats().unwrap();
        let achieved = (snapshot.decisions - initial) as f64 * DECISION_DT / 4.0 / elapsed;
        let stats =
            &snapshot.stats[neural_racing_engine::live::LiveTrainer::circuit_index(Some(circuit))];
        frame_times.sort_by(f64::total_cmp);
        let p95 = frame_times[frame_times.len() * 95 / 100];
        println!("{} requested={label} achieved={achieved:.1}x live_frames_per_sec={:.1} render_p95_ms={:.2} attempts={} laps={} updates={} recent_mean={:.3}",
            circuit.slug(), moving_frames as f64 / elapsed, p95 * 1000.0,
            stats.attempts, stats.laps, snapshot.updates,
            stats.history.iter().map(|a| a.progress).sum::<f64>() / stats.history.len().max(1) as f64);
        assert!(snapshot.updates > 20 && stats.attempts > 20);
        // A wall-time performance probe is not a convergence deadline: 30 s of
        // fresh learning does not have to complete a lap on every circuit.
        assert!(
            moving_frames as f64 / elapsed > 25.0,
            "cars' live state must keep refreshing"
        );
        assert!(p95 < 0.050, "rendering should stay responsive");
        assert!(
            achieved > 150.0,
            "the live trainer must keep a useful multiple of real time"
        );
        save(
            &mut harness,
            &format!("learning-{}-{label}", circuit.slug()),
        );
    }
}
