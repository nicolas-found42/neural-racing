//! Local GPU diagnostics of the production App path, without modifying README captures.
use egui::{Key, Vec2};
use egui_kittest::Harness;
use neural_racing_core::Circuit;
use neural_racing_gui::{ground_tiles, App, Mode};
use std::{path::PathBuf, time::Instant};

/// The tiles are rasterised on a background worker; wait for the visible set
/// with a deadline and the last observed state, never a fixed sleep.
fn wait_for_tiles(harness: &mut Harness<'_, App>, label: &str) {
    let deadline = Instant::now() + std::time::Duration::from_secs(240);
    loop {
        harness.run_steps(1);
        let pending = ground_tiles::pending(&harness.ctx);
        if pending == Some(0) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "{label}: ground tiles still pending {pending:?}"
        );
        std::thread::sleep(std::time::Duration::from_millis(5));
    }
}

fn frame_times(harness: &mut Harness<'_, App>, frames: usize) -> (f64, f64, f64) {
    let mut times = Vec::with_capacity(frames);
    for _ in 0..frames {
        let start = Instant::now();
        harness.run_steps(1);
        let _pixels = harness.render().unwrap();
        times.push(start.elapsed().as_secs_f64() * 1000.0);
    }
    times.sort_by(f64::total_cmp);
    (
        times[frames / 2],
        times[frames * 95 / 100],
        times[frames - 1],
    )
}

#[test]
#[ignore = "writes local follow/overview captures and measures warmed GPU readbacks"]
fn imported_course_follow_camera_and_warmed_frame_cost() {
    let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../.audit/track-visual-fix");
    std::fs::create_dir_all(&directory).unwrap();
    let mut reports = Vec::new();
    for id in ["gt4-suzuka", "gt4-nurburgring"] {
        let mut app = App::new(None, PathBuf::new());
        app.select_circuit(Some(Circuit::lookup(id).unwrap()));
        app.configure(None, Some(Mode::Manual), Some(1.0));
        let mut harness = Harness::builder()
            .with_size(Vec2::new(1264.0, 900.0))
            .with_pixels_per_point(1.0)
            .with_accessibility_check(false)
            .with_context_setup(App::install_style)
            .build_ui_state(|ui, app: &mut App| app.render(ui), app);
        harness.run_steps(1);
        harness.key_press(Key::Space); // Keep the source spawn fixed during timing.
        wait_for_tiles(&mut harness, id);
        harness.run_steps(1);
        harness
            .render()
            .unwrap()
            .save(directory.join(format!("{id}-app-follow.png")))
            .unwrap();
        let (p50, p95, max) = frame_times(&mut harness, 120);

        // Overview uses the coarsest tile level; it must converge too.
        harness.key_press(Key::V);
        wait_for_tiles(&mut harness, &format!("{id} overview"));
        harness.run_steps(1);
        harness
            .render()
            .unwrap()
            .save(directory.join(format!("{id}-app-overview.png")))
            .unwrap();
        let (overview_p50, overview_p95, overview_max) = frame_times(&mut harness, 60);
        harness.key_press(Key::V);
        wait_for_tiles(&mut harness, &format!("{id} follow"));

        // Movement invalidates tiles; measure the cache-miss path too.
        harness.key_press(Key::Space); // resume
        harness.key_down(Key::ArrowUp);
        let (moving_p50, moving_p95, moving_max) = frame_times(&mut harness, 120);
        harness.key_up(Key::ArrowUp);
        harness.key_press(Key::Space); // pause again
        wait_for_tiles(&mut harness, &format!("{id} moving settle"));

        reports.push(format!(
            "{{\"course\":\"{id}\",\"frames\":120,\"follow_p50_ms\":{p50},\"follow_p95_ms\":{p95},\"follow_max_ms\":{max},\"overview_p50_ms\":{overview_p50},\"overview_p95_ms\":{overview_p95},\"overview_max_ms\":{overview_max},\"moving_p50_ms\":{moving_p50},\"moving_p95_ms\":{moving_p95},\"moving_max_ms\":{moving_max}}}"
        ));
    }
    std::fs::write(directory.join("app-follow-frame-cost.json"), format!("{{\"measurement\":\"App UI plus wgpu render and synchronous RGBA readback; 1264x900; paused spawn (follow), overview, and driving with cache misses\",\"courses\":[{}]}}\n", reports.join(","))).unwrap();
}
