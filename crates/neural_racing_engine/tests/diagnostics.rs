use neural_racing_diagnostics::Value;
use std::{fs, path::Path, process::Command};
fn read(dir: &Path) -> Vec<Value> {
    fs::read_dir(dir)
        .unwrap()
        .flatten()
        .flat_map(|e| {
            fs::read_to_string(e.path())
                .unwrap()
                .lines()
                .map(|l| l.parse::<Value>().unwrap())
                .collect::<Vec<_>>()
        })
        .collect()
}
#[test]
fn cli_failures_survive_process_exit_without_retaining_paths_or_payloads() {
    let root =
        std::env::temp_dir().join(format!("neural-racing-cli-evidence-{}", std::process::id()));
    fs::create_dir_all(&root).unwrap();
    let evidence = root.join("evidence");
    let secret_path = root.join("secret-canary-DO-NOT-RETAIN.bin");
    let invoke = |path: &Path| {
        Command::new(env!("CARGO_BIN_EXE_verify"))
            .args(["--tracks", "1", "--quiet", "--model"])
            .arg(path)
            .env("NEURAL_RACING_DIAGNOSTICS_DIR", &evidence)
            .output()
            .unwrap()
    };
    assert!(!invoke(&secret_path).status.success());
    fs::write(&secret_path, b"truncated").unwrap();
    assert!(!invoke(&secret_path).status.success());
    let policy = neural_racing_ml::Policy::new(neural_racing_core::OBS_DIM, 8, 1, 42);
    policy.save(&secret_path).unwrap();
    let evaluated = invoke(&secret_path);
    assert_eq!(evaluated.status.code(), Some(1));
    let events = read(&evidence);
    assert!(events
        .iter()
        .any(|e| e["event"] == "policy_load_finished" && e["data"]["error_kind"] == "NotFound"));
    assert!(events
        .iter()
        .any(|e| e["event"] == "policy_load_finished" && e["data"]["error_kind"] == "InvalidData"));
    assert!(events.iter().any(|e| e["event"] == "evaluation_finished"));
    assert_eq!(
        events
            .iter()
            .filter(|e| e["event"] == "process_end")
            .count(),
        3
    );
    assert_eq!(events.iter().filter(|e| e["event"] == "panic").count(), 2);
    for event in &events {
        assert!(!event.to_string().contains("secret-canary"));
    }
    // An unavailable telemetry sink must not change verification's exit status.
    let blocked = root.join("blocked");
    fs::write(&blocked, b"not a directory").unwrap();
    let result = Command::new(env!("CARGO_BIN_EXE_verify"))
        .args(["--tracks", "1", "--quiet", "--model"])
        .arg(&secret_path)
        .env("NEURAL_RACING_DIAGNOSTICS_DIR", blocked)
        .output()
        .unwrap();
    assert_eq!(result.status.code(), evaluated.status.code());
    let help_dir = root.join("help-evidence");
    let help = Command::new(env!("CARGO_BIN_EXE_train"))
        .arg("--help")
        .env("NEURAL_RACING_DIAGNOSTICS_DIR", &help_dir)
        .output()
        .unwrap();
    assert!(help.status.success());
    assert!(read(&help_dir).iter().any(|e| e["event"] == "process_end"));
    fs::remove_dir_all(root).unwrap();
}
