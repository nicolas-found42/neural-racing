use neural_racing_core::{Circuit, UNITS_PER_METRE};
use neural_racing_diagnostics::Value;
use neural_racing_engine::live::LiveTrainer;
use std::{fs, path::Path, process::Command};

#[test]
fn retained_attempt_identifies_native_contact_loss() {
    let temporary =
        std::env::temp_dir().join(format!("neural-racing-contact-{}", std::process::id()));
    let preserve = std::env::var_os("NEURAL_RACING_CONTACT_TEST_DIR");
    let directory = preserve.as_ref().map(Path::new).unwrap_or(&temporary);
    std::env::set_var("NEURAL_RACING_DIAGNOSTICS_DIR", directory);
    let recorder = neural_racing_diagnostics::init("contact_test");
    let track = Circuit::lookup("gt4-suzuka").unwrap().build();
    let mut live = LiveTrainer::with_curriculum(track, 42, false);
    live.cars[0].x += 100_000.0 * UNITS_PER_METRE;
    live.step().unwrap();
    assert!(live.cars[0].off_track);
    let session = live.diagnostic_id;
    drop(live);
    drop(recorder);

    let output = Command::new("python3")
        .arg(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../scripts/diagnostics.py"
        ))
        .arg("--dir")
        .arg(directory)
        .args([
            "--operation",
            &session.to_string(),
            "--event",
            "live_attempt_ended",
        ])
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let row: Value = std::str::from_utf8(&output.stdout)
        .unwrap()
        .trim()
        .parse()
        .unwrap();
    assert_eq!(row["operation"], session);
    assert_eq!(row["data"]["car"], 0);
    assert_eq!(row["data"]["reason"], "Crash");
    assert_eq!(
        row["data"]["state"]["departure_reason"],
        "road query has no contact"
    );
    if preserve.is_none() {
        fs::remove_dir_all(directory).unwrap();
    }
}
