//! Races a saved policy over fresh random tracks and reports pass/fail.
//! `cargo run --release -p neural_racing_engine --bin verify -- --model models/policy.bin`

use std::path::PathBuf;

use neural_racing_engine::evaluate;
use neural_racing_ml::Policy;

struct Args {
    model: PathBuf,
    tracks: usize,
    laps: i32,
    seed: u64,
    quiet: bool,
}

fn main() -> std::process::ExitCode {
    let _diagnostics = neural_racing_diagnostics::init("verify");
    let mut args = Args {
        model: PathBuf::from("models/policy.bin"),
        tracks: 50,
        laps: neural_racing_core::DEFAULT_LAPS,
        seed: 9_000_000,
        quiet: false,
    };
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        let mut value = || {
            iter.next()
                .unwrap_or_else(|| panic!("{flag} needs a value"))
        };
        match flag.as_str() {
            "--model" => args.model = PathBuf::from(value()),
            "--tracks" => args.tracks = value().parse().expect("integer"),
            "--laps" => args.laps = value().parse().expect("integer"),
            "--seed" => args.seed = value().parse().expect("integer"),
            "--quiet" => args.quiet = true,
            "--help" | "-h" => {
                println!("verify [--model PATH] [--tracks N] [--laps N] [--seed N] [--quiet]");
                return std::process::ExitCode::SUCCESS;
            }
            other => panic!("unknown flag {other}"),
        }
    }

    neural_racing_diagnostics::emit!(
        "verify_config",
        0,
        neural_racing_diagnostics::json!({"seed":args.seed,"tracks":args.tracks,"laps":args.laps})
    );
    let policy = Policy::load(&args.model)
        .unwrap_or_else(|error| panic!("load {}: {error}", args.model.display()));
    if policy.obs_dim != neural_racing_core::OBS_DIM {
        eprintln!("This policy uses the old observation/physics format. Train a fresh model for the current vehicle dynamics.");
        neural_racing_diagnostics::emit!(
            "policy_incompatible",
            0,
            neural_racing_diagnostics::json!({"actual_obs_dim":policy.obs_dim,"expected_obs_dim":neural_racing_core::OBS_DIM})
        );
        return std::process::ExitCode::from(2);
    }
    println!(
        "{} ({} parameters) on {} fresh tracks, {} laps each",
        args.model.display(),
        policy.num_params(),
        args.tracks,
        args.laps
    );

    let report = evaluate(&policy, args.seed, args.tracks, args.laps);
    if !args.quiet {
        for row in &report.rows {
            println!(
                "  seed {:9} | {} | laps {}/{} | steps {:5} | dist {:6.0} | speed {:5.0} | min wall {:4.0}",
                row.seed,
                if row.finished {
                    "finished"
                } else if row.off_track {
                    "crashed "
                } else {
                    "timeout "
                },
                row.laps_completed,
                args.laps,
                row.steps,
                row.distance,
                row.mean_speed,
                row.min_wall,
            );
        }
    }
    println!(
        "finished {}/{} ({:.1}%) | crashed {} | timed out {} | mean distance {:.0} | mean speed {:.0}",
        report.finished,
        args.tracks,
        report.success_rate() * 100.0,
        report.crashed,
        report.timed_out,
        report.mean_distance,
        report.mean_speed,
    );

    if report.finished < args.tracks {
        neural_racing_diagnostics::emit!(
            "verify_result",
            0,
            neural_racing_diagnostics::json!({"passed":false,"finished":report.finished,"tracks":args.tracks})
        );
        return std::process::ExitCode::FAILURE;
    }
    println!(
        "PASS: every one of {} unseen random tracks completed {} laps without leaving the road",
        args.tracks, args.laps
    );
    neural_racing_diagnostics::emit!(
        "verify_result",
        0,
        neural_racing_diagnostics::json!({"passed":true,"tracks":args.tracks})
    );
    std::process::ExitCode::SUCCESS
}
