//! Safe synthetic incidents, using real production paths. No existing weights modified.
use neural_racing_core::Circuit;
use neural_racing_diagnostics::{emit, json};
use neural_racing_engine::live::LiveTrainer;
fn main() {
    let _diagnostics = neural_racing_diagnostics::init("diagnostics_probe");
    let mode = std::env::args().nth(1).unwrap_or_else(|| "live".into());
    match mode.as_str() {
        "live" => {
            let mut live = LiveTrainer::new(Circuit::Monza.build(), 42);
            while live.updates < 3 {
                live.step().unwrap();
            }
            live.select_track(Circuit::Silverstone.build());
            // A non-finite policy must stop before poisoning physics state.
            let saved = live.policy.clone();
            live.policy.params_mut()[0][0] = f32::NAN;
            let decisions = live.decisions;
            assert_eq!(live.step(), Err("non-finite inference"));
            assert_eq!(live.decisions, decisions);
            live.policy = saved;
            live.step().unwrap();
            assert!(live.decisions > decisions);
            emit!(
                "probe_recovered",
                live.diagnostic_id,
                json!({"decisions":live.decisions,"injected":"non-finite policy"})
            );
            live.diagnostic_progress();
        }
        "panic" => panic!("secret-canary-DO-NOT-RETAIN"),
        "flood" => {
            for n in 0..100_000 {
                emit!("probe_flood", 1, json!({"n":n}));
            }
        }
        "wait" => {
            emit!("probe_waiting", 1, json!({}));
            assert!(neural_racing_diagnostics::flush(
                std::time::Duration::from_secs(2)
            ));
            std::thread::sleep(std::time::Duration::from_secs(30));
        }
        _ => panic!("unsupported probe mode"),
    }
}
