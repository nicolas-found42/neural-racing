//! Neural Racing: watch or drive real circuits or procedural courses, and see the
//! trained policy's activations while it races.

use std::path::PathBuf;

use neural_racing_gui::{app::find_policy, App, Mode};

fn main() -> eframe::Result<()> {
    let _diagnostics = neural_racing_diagnostics::init("gui");
    let mut policy: Option<PathBuf> = None;
    let mut seed: Option<u64> = None;
    let mut mode: Option<Mode> = None;
    let mut circuit = None;
    let mut speed: Option<f32> = None;
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        let mut value = || {
            iter.next()
                .unwrap_or_else(|| panic!("{flag} needs a value"))
        };
        match flag.as_str() {
            "--track" => circuit = Some(value()),
            "--policy" => policy = Some(PathBuf::from(value())),
            "--seed" => seed = Some(value().parse().expect("integer")),
            "--speed" => {
                let text = value();
                speed = Some(if text == "max" {
                    f32::INFINITY
                } else {
                    text.parse().expect("speed multiplier or max")
                });
            }
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
                    "neural-racing [--policy PATH] [--seed N] [--mode watch|manual|train] [--speed X|max] [--track monza|silverstone|red-bull-ring|random]"
                );
                return Ok(());
            }
            other => panic!("unknown flag {other}"),
        }
    }

    // Learning always starts fresh. Disk policies are only an explicit legacy
    // watch-mode option; the live session has no persistence path.
    let (loaded, path) = if mode == Some(Mode::Watch) {
        find_policy(policy)
    } else {
        (None, PathBuf::new())
    };
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1264.0, 900.0])
            .with_min_inner_size([1000.0, 720.0])
            .with_title("Neural Racing — watch the model learn"),
        ..Default::default()
    };
    let result = eframe::run_native(
        "Neural Racing",
        options,
        Box::new(move |cc| {
            App::install_style(&cc.egui_ctx);
            let mut application = App::new(loaded, path);
            application.configure(seed, mode, speed);
            if let Some(name) = circuit.as_deref() {
                let selected = neural_racing_core::Circuit::ALL
                    .into_iter()
                    .find(|c| c.slug() == name);
                assert!(
                    selected.is_some() || name == "random",
                    "unknown track {name}"
                );
                application.select_circuit(selected);
            }
            Ok(Box::new(application))
        }),
    );
    neural_racing_diagnostics::emit!(
        "gui_exited",
        0,
        neural_racing_diagnostics::json!({"ok":result.is_ok()})
    );
    result
}
