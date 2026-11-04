//! Neural Racing: watch or drive a car on seeded random tracks, and see the
//! trained policy's activations while it races.

use std::path::PathBuf;

use neural_racing_gui::{app::find_policy, App, Mode};

fn main() -> eframe::Result<()> {
    let mut policy: Option<PathBuf> = None;
    let mut seed: Option<u64> = None;
    let mut mode: Option<Mode> = None;
    let mut speed: Option<f32> = None;
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        let mut value = || iter.next().unwrap_or_else(|| panic!("{flag} needs a value"));
        match flag.as_str() {
            "--policy" => policy = Some(PathBuf::from(value())),
            "--seed" => seed = Some(value().parse().expect("integer")),
            "--speed" => speed = Some(value().parse().expect("speed multiplier")),
            "--mode" => {
                mode = Some(match value().as_str() {
                    "watch" | "ai" => Mode::Watch,
                    "manual" | "drive" => Mode::Manual,
                    "train" => Mode::Train,
                    other => panic!("unknown mode {other}"),
                })
            }
            "--help" | "-h" => {
                println!(
                    "neural-racing [--policy PATH] [--seed N] [--mode watch|manual|train] [--speed X]"
                );
                return Ok(());
            }
            other => panic!("unknown flag {other}"),
        }
    }

    let (loaded, path) = find_policy(policy);
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1264.0, 900.0])
            .with_min_inner_size([1000.0, 720.0])
            .with_title("Neural Racing — random tracks, learnt driver"),
        ..Default::default()
    };
    eframe::run_native(
        "Neural Racing",
        options,
        Box::new(move |cc| {
            App::install_style(&cc.egui_ctx);
            let mut application = App::new(loaded, path);
            application.configure(seed, mode, speed);
            Ok(Box::new(application))
        }),
    )
}
