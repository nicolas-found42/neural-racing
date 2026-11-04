//! Bounded, in-memory live-learning benchmark. No checkpoint is loaded or saved.
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
    let seed: u64 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(42);
    let circuit = std::env::args()
        .nth(3)
        .and_then(|name| Circuit::ALL.into_iter().find(|c| c.slug() == name))
        .unwrap_or(Circuit::Monza);
    let mut live = LiveTrainer::new(circuit.build(), seed);
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
    println!("circuit={} seed={seed} seconds={elapsed:.2} decisions={} updates={} attempts={} completed_laps={} per_car_speed={:.1}x initial_progress={:.4} learned_progress={:.4} recent_mean={:.4}",
        circuit.slug(), live.decisions, live.updates, stats.attempts, stats.laps,
        live.decisions as f64 * DECISION_DT / CAR_COUNT as f64 / elapsed,
        before.distance / live.cars[0].track.lap_length,
        after.distance / live.cars[0].track.lap_length,
        stats.history.iter().map(|a| a.progress).sum::<f64>() / stats.history.len().max(1) as f64);
}
