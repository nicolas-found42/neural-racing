//! Headless training loop: `cargo run --release -p neural_racing_engine --bin train`.

use std::io::Write;
use std::path::PathBuf;

use neural_racing_engine::{evaluate, score, EvalReport, RaceConfig, Trainer};
use neural_racing_ml::Policy;

struct Args {
    generations: i64,
    workers: usize,
    episodes_per_worker: usize,
    laps: i32,
    seed: u64,
    out: PathBuf,
    eval_every: i64,
    eval_tracks: usize,
    resume: Option<PathBuf>,
    log: Option<PathBuf>,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            generations: 400,
            workers: std::thread::available_parallelism()
                .map(|n| n.get().saturating_sub(1).max(1))
                .unwrap_or(4),
            episodes_per_worker: 2,
            laps: neural_racing_core::DEFAULT_LAPS,
            seed: 0,
            out: PathBuf::from("models/policy.bin"),
            eval_every: 10,
            eval_tracks: 24,
            resume: None,
            log: None,
        }
    }
}

fn parse_args() -> Args {
    let mut args = Args::default();
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        let mut value = || {
            iter.next()
                .unwrap_or_else(|| panic!("{flag} needs a value"))
        };
        match flag.as_str() {
            "--generations" => args.generations = value().parse().expect("integer"),
            "--workers" => args.workers = value().parse().expect("integer"),
            "--episodes-per-worker" => args.episodes_per_worker = value().parse().expect("integer"),
            "--laps" => args.laps = value().parse().expect("integer"),
            "--seed" => args.seed = value().parse().expect("integer"),
            "--out" => args.out = PathBuf::from(value()),
            "--eval-every" => args.eval_every = value().parse().expect("integer"),
            "--eval-tracks" => args.eval_tracks = value().parse().expect("integer"),
            "--resume" => args.resume = Some(PathBuf::from(value())),
            "--log" => args.log = Some(PathBuf::from(value())),
            "--help" | "-h" => {
                println!(
                    "train [--generations N] [--workers N] [--episodes-per-worker N] [--laps N]\n\
                     \x20     [--seed N] [--out PATH] [--eval-every N] [--eval-tracks N]\n\
                     \x20     [--resume PATH] [--log PATH]"
                );
                std::process::exit(0);
            }
            other => panic!("unknown flag {other}"),
        }
    }
    args
}

fn print_eval(report: &EvalReport, tracks: usize) {
    println!(
        "    eval {}/{} tracks finished ({:.0}%) | crashes {} | timeouts {} | mean dist {:.0} | mean speed {:.0} | min wall {:.0}",
        report.finished,
        tracks,
        report.success_rate() * 100.0,
        report.crashed,
        report.timed_out,
        report.mean_distance,
        report.mean_speed,
        report.min_wall,
    );
}

fn main() {
    let args = parse_args();
    let mut config = RaceConfig::new(args.seed, args.workers);
    config.laps = args.laps;
    config.episodes_per_worker = args.episodes_per_worker;

    let mut trainer = match &args.resume {
        Some(path) => {
            let policy = Policy::load(path)
                .unwrap_or_else(|error| panic!("load {}: {error}", path.display()));
            assert_eq!(
                policy.obs_dim,
                neural_racing_core::OBS_DIM,
                "old observation/physics format: start fresh for the current vehicle dynamics"
            );
            println!(
                "resumed {} ({} parameters, {} layers)",
                path.display(),
                policy.num_params(),
                policy.blocks + 1
            );
            Trainer::with_policy(config, policy)
        }
        None => Trainer::new(config),
    };

    if let Some(parent) = args.out.parent() {
        std::fs::create_dir_all(parent).expect("create model directory");
    }
    let mut log = args.log.as_ref().map(|path| {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent).expect("create log directory");
        }
        std::fs::File::create(path).expect("open log")
    });
    if let Some(log) = log.as_mut() {
        writeln!(
            log,
            "{{\"event\":\"start\",\"seed\":{},\"workers\":{},\"laps\":{},\"episodes\":{}}}",
            args.seed, args.workers, args.laps, args.episodes_per_worker
        )
        .ok();
    }

    println!(
        "training {} generations | {} workers x {} episodes | {} laps | normalized {}x{} actor-critic",
        args.generations,
        args.workers,
        args.episodes_per_worker,
        args.laps,
        neural_racing_ml::DEFAULT_BLOCKS,
        neural_racing_ml::DEFAULT_WIDTH
    );

    let mut best: Option<((usize, f64, f64), i64)> = None;
    let best_path = args.out.with_file_name("best.bin");
    let eval_base: u64 = 7_000_000 + args.seed;

    for _ in 0..args.generations {
        let stats = trainer.train_generation();
        println!(
            "gen {:4} | eps {:3} | finished {:2} crash {:2} timeout {:2} | cp {:4.1} | dist {:6.0} | rew {:7.2} | kl {:.4} scale {:.2} | ent {:5.2} | gnorm {:5.2} | {:5.2}s",
            stats.generation,
            stats.episodes,
            stats.finished,
            stats.crashes,
            stats.timeouts,
            stats.mean_checkpoints,
            stats.best_distance,
            stats.mean_reward,
            stats.update.policy_kl,
            stats.update.update_scale,
            stats.entropy,
            stats.update.gradient_norm,
            stats.elapsed_seconds,
        );
        if let Some(log) = log.as_mut() {
            writeln!(
                log,
                "{{\"event\":\"generation\",\"generation\":{},\"episodes\":{},\"finished\":{},\"crashes\":{},\"timeouts\":{},\"mean_checkpoints\":{:.4},\"best_distance\":{:.4},\"mean_reward\":{:.4},\"mean_speed\":{:.4},\"entropy\":{:.5},\"policy_kl\":{:.8},\"update_scale\":{:.4},\"gradient_norm\":{:.6},\"rows\":{},\"seconds\":{:.3}}}",
                stats.generation,
                stats.episodes,
                stats.finished,
                stats.crashes,
                stats.timeouts,
                stats.mean_checkpoints,
                stats.best_distance,
                stats.mean_reward,
                stats.mean_speed,
                stats.entropy,
                stats.update.policy_kl,
                stats.update.update_scale,
                stats.update.gradient_norm,
                stats.rows,
                stats.elapsed_seconds,
            )
            .ok();
        }

        let due = args.eval_every > 0
            && (stats.generation % args.eval_every == 0 || stats.generation == args.generations);
        if due {
            let report = evaluate(&trainer.policy, eval_base, args.eval_tracks, args.laps);
            print_eval(&report, args.eval_tracks);
            trainer
                .policy
                .save(&args.out)
                .expect("save policy checkpoint");
            let current = score(&report);
            if best.is_none() || current > best.map(|(value, _)| value).unwrap() {
                best = Some((current, stats.generation));
                trainer.policy.save(&best_path).expect("save best policy");
                println!(
                    "    saved best policy at generation {} -> {}",
                    stats.generation,
                    best_path.display()
                );
            }
            if report.finished == args.eval_tracks {
                println!(
                    "    all {} evaluation tracks finished at generation {}",
                    args.eval_tracks, stats.generation
                );
            }
            if let Some(log) = log.as_mut() {
                writeln!(
                    log,
                    "{{\"event\":\"eval\",\"generation\":{},\"tracks\":{},\"finished\":{},\"crashed\":{},\"timed_out\":{},\"mean_distance\":{:.4},\"mean_speed\":{:.4},\"min_wall\":{:.4}}}",
                    stats.generation,
                    args.eval_tracks,
                    report.finished,
                    report.crashed,
                    report.timed_out,
                    report.mean_distance,
                    report.mean_speed,
                    report.min_wall,
                )
                .ok();
            }
        }
    }

    trainer.policy.save(&args.out).expect("save final policy");
    println!(
        "saved {} ({} parameters) — best generation {}",
        args.out.display(),
        trainer.policy.num_params(),
        best.map(|(_, generation)| generation).unwrap_or(0)
    );
}
