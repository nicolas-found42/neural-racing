//! This executable has one test so the process-wide recorder has one owner.
use neural_racing_core::Circuit;
use neural_racing_diagnostics::Value;
use neural_racing_gui::learning::{Learning, Snapshot};
use std::time::{Duration, Instant};
fn wait(worker: &Learning, predicate: impl Fn(&Snapshot) -> bool) -> Snapshot {
    let start = Instant::now();
    loop {
        if let Some(s) = worker.take() {
            if predicate(&s) {
                return s;
            }
        }
        assert!(start.elapsed() < Duration::from_secs(15));
        std::thread::sleep(Duration::from_millis(5));
    }
}
#[test]
fn retained_commands_reconstruct_pause_switch_reset_and_shutdown() {
    let dir = std::env::temp_dir().join(format!(
        "neural-racing-worker-evidence-{}",
        std::process::id()
    ));
    std::env::set_var("NEURAL_RACING_DIAGNOSTICS_DIR", &dir);
    let guard = neural_racing_diagnostics::init("worker_test");
    let mut worker = Learning::start(Circuit::Monza.build(), 42, f64::INFINITY);
    wait(&worker, |s| s.updates >= 1);
    worker.running(false);
    let paused = wait(&worker, |s| s.paused);
    worker.track(Circuit::Silverstone.build());
    let switched = wait(&worker, |s| s.revision == 1);
    assert_eq!(switched.decisions, paused.decisions);
    worker.reset(123);
    let fresh = wait(&worker, |s| s.revision == 2);
    assert_eq!(fresh.decisions, 0);
    assert_ne!(fresh.diagnostic_id, paused.diagnostic_id);
    worker.running(true);
    wait(&worker, |s| s.revision == 2 && s.decisions > 0);
    drop(worker);
    drop(guard);
    let mut events = Vec::<Value>::new();
    for file in std::fs::read_dir(&dir).unwrap().flatten() {
        for line in std::fs::read_to_string(file.path()).unwrap().lines() {
            events.push(line.parse().unwrap());
        }
    }
    for request in events.iter().filter(|e| e["event"] == "command_requested") {
        let id = &request["operation"];
        let received = events
            .iter()
            .find(|e| e["operation"] == *id && e["event"] == "command_received")
            .unwrap();
        let applied = events
            .iter()
            .find(|e| e["operation"] == *id && e["event"] == "command_applied")
            .unwrap();
        assert!(request["seq"].as_u64() < received["seq"].as_u64());
        assert!(received["seq"].as_u64() < applied["seq"].as_u64());
    }
    assert!(events.iter().any(|e| e["event"] == "worker_session_reset"));
    assert!(events.iter().any(|e| e["event"] == "process_end"));
    assert!(events
        .iter()
        .all(|e| e["dropped_total"].as_u64().unwrap_or(0) == 0));
    let script =
        std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../scripts/diagnostics.py");
    let output = std::process::Command::new("python3")
        .arg(script)
        .arg("--dir")
        .arg(&dir)
        .arg("--summary")
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(String::from_utf8_lossy(&output.stdout).contains("\"clean_end_observed\": true"));
    // Optional evidence destination for a manual audit; normal tests remove their files.
    if let Some(destination) = std::env::var_os("NEURAL_RACING_TEST_EVIDENCE") {
        std::fs::create_dir_all(&destination).unwrap();
        for entry in std::fs::read_dir(&dir).unwrap().flatten() {
            std::fs::copy(
                entry.path(),
                std::path::Path::new(&destination).join(entry.file_name()),
            )
            .unwrap();
        }
    }
    std::fs::remove_dir_all(dir).unwrap();
}
