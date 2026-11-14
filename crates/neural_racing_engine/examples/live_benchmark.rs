//! Bounded live-learning benchmark. Arguments: seconds seed circuit workers minimum_speed.
//! A nonzero speed floor requires at least 60 seconds and fails below that floor.
//! No checkpoint is loaded or saved.
use neural_racing_core::{Circuit, DECISION_DT};
use neural_racing_engine::{
    deterministic_episode,
    live::{LiveTrainer, CAR_COUNT},
};
use std::time::{Duration, Instant};
fn main() {
    let _diagnostics = neural_racing_diagnostics::init("live_benchmark");
    let seconds: u64 = std::env::args()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap_or(15);
    assert!(seconds > 0, "benchmark duration must be positive");
    let seed: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(42);
    let circuit = std::env::args()
        .nth(3)
        .and_then(|name| Circuit::ALL.into_iter().find(|c| c.slug() == name))
        .unwrap_or(Circuit::Monza);
    let mut live = LiveTrainer::new(circuit.build(), seed);
    let workers: usize = std::env::args()
        .nth(4)
        .map(|s| s.parse().expect("worker count"))
        .unwrap_or(1);
    let minimum_speed: f64 = std::env::args()
        .nth(5)
        .map(|s| s.parse().expect("minimum speed"))
        .unwrap_or(0.0);
    assert!(minimum_speed.is_finite() && minimum_speed >= 0.0);
    assert!(
        minimum_speed == 0.0 || seconds >= 60,
        "speed verification requires at least 60 seconds"
    );
    assert!(
        minimum_speed == 0.0 || !cfg!(debug_assertions),
        "speed verification requires a release build"
    );
    live.set_simulation_workers(workers)
        .expect("simulation workers");
    println!(
        "configuration cars={CAR_COUNT} simulation_workers={} rays={} obs_dim={} parameters={}",
        live.simulation_workers(),
        neural_racing_core::NUM_RAYS,
        neural_racing_core::OBS_DIM,
        live.policy.num_params()
    );
    let before = deterministic_episode(&live.policy, circuit.build(), 1);
    let start = Instant::now();
    let mut next_report = 10;
    while start.elapsed() < Duration::from_secs(seconds) {
        live.step().unwrap();
        if start.elapsed().as_secs() >= next_report {
            let stats = live.current_stats();
            let best = stats.history.iter().map(|a| a.progress).fold(0.0, f64::max);
            let crashes = stats
                .history
                .iter()
                .filter(|a| a.reason == neural_racing_engine::live::EndReason::Crash)
                .count();
            let stalls = stats
                .history
                .iter()
                .filter(|a| a.reason == neural_racing_engine::live::EndReason::Stalled)
                .count();
            let timeouts = stats
                .history
                .iter()
                .filter(|a| a.reason == neural_racing_engine::live::EndReason::Timeout)
                .count();
            println!("progress seconds={} updates={} attempts={} laps={} recent_best={best:.3} crashes={crashes} stalls={stalls} timeouts={timeouts}", next_report, live.updates, stats.attempts, stats.laps);
            next_report += 10;
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    let after = deterministic_episode(&live.policy, circuit.build(), 1);
    let stats = live.current_stats();
    let speed = live.decisions as f64 * DECISION_DT / CAR_COUNT as f64 / elapsed;
    println!(
        "{}",
        neural_racing_diagnostics::json!({"event":"benchmark_finished","circuit":circuit.slug(),
        "seed":seed,"wall_seconds":elapsed,"cars":CAR_COUNT,"simulation_workers":live.simulation_workers(),
        "decisions":live.decisions,"updates":live.updates,"attempts":stats.attempts,"completed_laps":stats.laps,
        "decision_dt":DECISION_DT,"per_car_speed":speed,"minimum_speed":minimum_speed,
        "speed_gate_requested":minimum_speed>0.0,
        "speed_gate_passed":if minimum_speed>0.0 {Some(speed>=minimum_speed && live.updates>0)} else {None},
        "rays":neural_racing_core::NUM_RAYS,"obs_dim":neural_racing_core::OBS_DIM,
        "parameters":live.policy.num_params(),"profile":if cfg!(debug_assertions) {"debug"} else {"release"}})
    );
    println!("circuit={} seed={seed} seconds={elapsed:.2} decisions={} updates={} attempts={} completed_laps={} per_car_speed={:.1}x initial_progress={:.4} learned_progress={:.4} recent_mean={:.4}",
        circuit.slug(), live.decisions, live.updates, stats.attempts, stats.laps,
        live.decisions as f64 * DECISION_DT / CAR_COUNT as f64 / elapsed,
        before.distance / live.cars[0].track.lap_length,
        after.distance / live.cars[0].track.lap_length,
        stats.history.iter().map(|a| a.progress).sum::<f64>() / stats.history.len().max(1) as f64);
    assert!(speed >= minimum_speed && live.updates > 0,
        "speed gate failed: achieved {speed:.3}x per car with {} updates, required {minimum_speed:.3}x", live.updates);
}
