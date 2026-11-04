//! Headless render of the real UI: drives the same `App::render` path eframe
//! uses, rasterises each mode with wgpu, and writes PNGs. No window opens.

use std::path::PathBuf;

use egui::{Key, Vec2};
use egui_kittest::Harness;
use neural_racing_gui::{app::find_policy, App, Mode};

const SIZE: Vec2 = Vec2::new(1264.0, 900.0);
/// Screenshots live in the workspace root so the README can embed them.
const OUT_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../docs/screenshots");

fn harness() -> Harness<'static, App> {
    let (policy, path) = find_policy(None);
    assert!(policy.is_some(), "models/policy.bin is missing: train first");
    let app = App::new(policy, path);
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
fn ai_keeps_driving_on_a_random_track() {
    let mut harness = harness();
    // Mode first, then the seed: switching mode resets the race, so a seed set
    // before the switch would be replaced by a fresh random track.
    harness
        .state_mut()
        .configure(None, Some(Mode::Watch), Some(2.0));
    harness.state_mut().seed_with(7_654_321);
    harness.run_steps(150);
    let race = harness.state().race();
    assert!(!race.is_over(), "the trained AI crashed before the screenshot");
    assert!(race.env.steps > 60, "the race did not advance");
    assert!(race.env.speed > 150.0, "the AI stopped driving");
    save(&mut harness, "watch");
}

#[test]
fn arrow_keys_drive_the_car_in_manual_mode() {
    // Find a start straight enough to hold the throttle on; the screenshot
    // should show the car driving, not the off-track banner.
    let mut seed_choice = None;
    for seed in 1..400u64 {
        let mut probe = harness();
        probe.state_mut().configure(Some(seed), Some(Mode::Manual), Some(1.0));
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

#[test]
fn live_training_streams_generations_into_the_dashboard() {
    let mut harness = harness();
    harness
        .state_mut()
        .configure(Some(20_260), Some(Mode::Train), Some(4.0));
    for _ in 0..40 {
        harness.run_steps(5);
        std::thread::sleep(std::time::Duration::from_millis(25));
    }
    let stats = harness.state().training_stats().expect("a generation arrived");
    assert!(stats.episodes > 0, "training reported no episodes");
    save(&mut harness, "train");
}
