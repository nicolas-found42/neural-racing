use egui::{Context, LayerId, Pos2, RawInput, Rect, Vec2};
use neural_racing_core::Circuit;
use neural_racing_diagnostics::Value;
use neural_racing_gui::render::{draw_track, View};
use std::time::{Duration, Instant};

#[test]
fn retained_ground_evidence_distinguishes_fallback_and_worker_readiness() {
    let dir = std::env::temp_dir().join(format!("neural-racing-ground-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    let dir = std::env::var_os("NEURAL_RACING_GROUND_TEST_DIR").map_or(dir, Into::into);
    std::env::set_var("NEURAL_RACING_DIAGNOSTICS_DIR", &dir);
    let guard = neural_racing_diagnostics::init("ground_test");
    let ctx = Context::default();
    let canvas = Rect::from_min_size(Pos2::ZERO, Vec2::new(480.0, 320.0));
    let draw = |track: &neural_racing_core::Track| {
        let mut output = ctx.run_pass(
            RawInput {
                screen_rect: Some(canvas),
                ..Default::default()
            },
            |ctx| {
                let mut painter = ctx.layer_painter(LayerId::background());
                painter.set_clip_rect(canvas);
                draw_track(&painter, &View::follow(canvas, track.start), track);
            },
        );
        output.textures_delta.clear();
        output
    };
    draw(&Circuit::Monza.build());
    let track = Circuit::lookup("gt4-suzuka").unwrap().build();
    let deadline = Instant::now() + Duration::from_secs(120);
    loop {
        let frame = draw(&track);
        if frame.shapes.iter().any(|shape| matches!(&shape.shape,
            egui::Shape::Mesh(mesh) if matches!(mesh.texture_id, egui::TextureId::Managed(id) if id != 0))) {
            if neural_racing_gui::ground_tiles::pending(&ctx) == Some(0) {
                break;
            }
        }
        assert!(
            Instant::now() < deadline,
            "tile cache {:?}",
            neural_racing_gui::ground_tiles::stats(&ctx)
        );
        std::thread::sleep(Duration::from_millis(5));
    }
    drop(ctx);
    drop(guard);
    let result = std::process::Command::new("python3")
        .arg(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../scripts/diagnostics.py"
        ))
        .arg("--dir")
        .arg(&dir)
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let rows: Vec<Value> = String::from_utf8(result.stdout)
        .unwrap()
        .lines()
        .map(|row| row.parse().unwrap())
        .collect();
    let find = |name: &str, op: &Value| {
        rows.iter()
            .find(|row| row["event"] == name && row["operation"] == *op)
            .unwrap()
    };
    let request = |course: &str| {
        rows.iter()
            .find(|row| {
                row["event"] == "ground_course_requested" && row["data"]["circuit"] == course
            })
            .unwrap()
    };
    let monza = &request("monza")["operation"];
    let suzuka = request("gt4-suzuka");
    assert_eq!(&suzuka["data"]["previous_course"], monza);
    assert_eq!(
        find("ground_course_prepared", monza)["data"]["status"],
        "no_source_visual"
    );
    let op = &suzuka["operation"];
    let received = find("ground_course_received", op);
    let ready = find("ground_visible_ready", op);
    assert_eq!(received["data"]["worker"], ready["data"]["worker"]);
    assert_eq!(received["data"]["has_raster"], true);
    assert_eq!(ready["data"]["pending"], 0);
    assert!(ready["data"]["accepted"].as_u64().unwrap() > 0);
    assert!(rows.iter().any(|row| row["event"] == "process_end"));
    if std::env::var_os("NEURAL_RACING_GROUND_TEST_DIR").is_none() {
        std::fs::remove_dir_all(dir).unwrap();
    }
}
