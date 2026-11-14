mod common;
use neural_racing_core::{course_catalog, Action, Circuit, Env, UNITS_PER_METRE as M};
use serde_json::{json, Value};
use std::panic::{catch_unwind, AssertUnwindSafe};

fn input_receipt() -> Option<Value> {
    std::env::var("GT4_COURSE_INPUT_RECEIPT").ok().map(|path| {
        let value: Value =
            serde_json::from_slice(&std::fs::read(path).expect("input receipt exists")).unwrap();
        assert_eq!(
            value["catalog_source"].as_str().unwrap(),
            include_str!("../src/circuit_data/gt4_catalog.rs"),
            "compiled catalog differs from frozen receipt"
        );
        assert_eq!(
            value["files_source"].as_str().unwrap(),
            include_str!("../src/circuit_data/gt4_files.rs"),
            "compiled asset binding differs from frozen receipt"
        );
        value
    })
}

fn state(env: &Env) -> Value {
    let hit = env.track.nearest_on_centerline(env.position());
    let height = env.track.arc_height_metres(hit.arc);
    let contact = env
        .track
        .road_contact((env.x / M) as f32, (env.y / M) as f32, height + 1.0);
    json!({"steps":env.steps,"position_m":[env.x/M,env.y/M],"route_height_m":height,
        "arc_m":hit.arc/M,"distance_m":env.distance/M,"speed_mps":env.speed/M,
        "offset_m":env.lateral_offset/M,"half_width_m":env.road_half_width/M,
        "surface":contact.map(|c|c.material_id),"mesh_height_m":contact.map(|c|c.height_metres),
        "normal_loads":env.tyre_loads,"off_track":env.off_track,"finished":env.lap_done})
}
fn finite(env: &Env) -> bool {
    env.obs.iter().all(|v| v.is_finite())
        && env.tyre_loads.iter().all(|v| v.is_finite())
        && [env.x, env.y, env.speed, env.distance, env.heading]
            .iter()
            .all(|v| v.is_finite())
}
fn run(circuit: Circuit, traverse: bool) -> Result<Value, String> {
    let track = circuit.try_build()?;
    let Circuit::Gt4(index) = circuit else {
        unreachable!()
    };
    let metadata = &course_catalog::courses()[usize::from(index)];
    let lengths: Value =
        serde_json::from_str(include_str!("fixtures/gt4_course_lengths.json")).unwrap();
    let expected = lengths["courses"][metadata.id]["length_metres"]
        .as_f64()
        .unwrap();
    if (track.lap_length / M - expected).abs() > 0.02 {
        return Err(format!(
            "native distance mismatch: measured={} expected={expected}",
            track.lap_length / M
        ));
    }
    let mut env = Env::new(track, 1);
    if !finite(&env) {
        return Err(format!("nonfinite spawn {}", state(&env)));
    }
    let spawn = state(&env);
    // Check the actual native wheel normal-load telemetry, rather than assuming
    // a body-center road hit establishes all four wheel contacts.
    if env.tyre_loads.iter().any(|v| *v <= 0.0) {
        return Err(format!("unloaded native wheel at spawn {spawn}"));
    }
    for _ in 0..20 {
        let desired = common::pursuit(&env, 12.0 * M);
        let outcome = env.step(Action {
            steer: desired.steer,
            throttle: 0.35,
        });
        if !finite(&env) || (outcome.done && !env.lap_done) {
            return Err(format!("one-second moderate launch failed {}", state(&env)));
        }
        if env.lap_done {
            break;
        }
    }
    let launch = state(&env);
    if circuit.is_practice() && (env.lap_done || env.credited_laps() != 0 || env.total_gates() != 0)
    {
        return Err(format!("practice area acquired race goals {}", state(&env)));
    }
    if traverse && !env.lap_done && !circuit.is_practice() {
        // This independent driver is a feasibility probe, never training data or
        // a claim about learned-policy or human competence.
        while env.steps < env.max_steps() {
            let action = common::pursuit(&env, 12.0 * M);
            let outcome = env.step(action);
            if !finite(&env) {
                return Err(format!("nonfinite pursuit {}", state(&env)));
            }
            if outcome.done {
                break;
            }
        }
        if !env.lap_done {
            return Err(format!("independent pursuit failed {}", state(&env)));
        }
    }
    Ok(
        json!({"spawn":spawn,"launch":launch,"final":state(&env),"length_metres":expected,"manual_practice":circuit.is_practice()}),
    )
}
fn collect(traverse: bool) -> (Vec<Value>, Vec<String>) {
    let mut records = Vec::new();
    let mut failures = Vec::new();
    for &circuit in Circuit::all()
        .iter()
        .filter(|c| matches!(c, Circuit::Gt4(_)))
    {
        let result =
            catch_unwind(AssertUnwindSafe(|| run(circuit, traverse))).unwrap_or_else(|p| {
                Err(format!(
                    "panic: {}",
                    p.downcast_ref::<String>()
                        .map(String::as_str)
                        .or_else(|| p.downcast_ref::<&str>().copied())
                        .unwrap_or("unknown panic")
                ))
            });
        match result {
            Ok(data) => {
                eprintln!("{} PASS", circuit.slug());
                records.push(json!({"course":circuit.slug(),"ok":true,"data":data}));
            }
            Err(error) => {
                eprintln!("{} FAIL {error}", circuit.slug());
                failures.push(format!("{}: {error}", circuit.slug()));
                records.push(json!({"course":circuit.slug(),"ok":false,"error":error}));
            }
        }
    }
    (records, failures)
}
#[test]
fn every_available_gt4_course_has_source_distance_and_a_finite_launch() {
    let (_, failures) = collect(false);
    assert!(
        failures.is_empty(),
        "{} course failures:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
#[test]
#[ignore = "release-mode all-course independent-driver feasibility audit"]
fn independent_driver_audits_race_routes_and_manual_practice_areas() {
    let receipt = input_receipt();
    let (records, failures) = collect(true);
    let path = std::env::var("GT4_COURSE_REPORT").unwrap_or_else(|_| {
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../.audit/gt4-tracks/all-course-driving.json")
            .to_string_lossy()
            .into_owned()
    });
    let path = std::path::Path::new(&path);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).unwrap();
    }
    std::fs::write(path,serde_json::to_vec_pretty(&json!({"driver":"race routes: independent common::pursuit at12m/s; areas: one-second manual launch; no training","debug_assertions":cfg!(debug_assertions),"architecture":std::env::consts::ARCH,"input_receipt":receipt,"courses":records,"failures":failures.len()})).unwrap()).unwrap();
    assert!(
        failures.is_empty(),
        "{} course failures; report {}",
        failures.len(),
        path.display()
    );
}

fn run_keyboard(id: &str, cruise: f64) -> Result<Value, String> {
    catch_unwind(AssertUnwindSafe(|| -> Result<Value, String> {
        let circuit = Circuit::lookup(id).ok_or("representative course unavailable")?;
        let mut env = Env::new(circuit.try_build()?, 1);
        let mut action = Action {
            steer: 0.0,
            throttle: 0.0,
        };
        let mut steering_remainder: f64 = 0.0;
        let trace_dir=std::env::var("GT4_KEYBOARD_TRACE_DIR").ok();
        let mut trace=std::collections::VecDeque::new();
        let mut held_desired=[0.0;2];
        for tick in 0..env.max_steps() {
            let instant=if trace_dir.is_some() || tick%4==0 {Some(common::pursuit(&env,cruise*M))} else {None};
            let before=trace_dir.as_ref().map(|_| json!({"state":state(&env),"heading_error_rad":env.heading_error,"yaw_rate_rad_s":env.yaw_rate,"cg_slip_deg":env.slip_angle_deg,"body_velocity_mps":[env.vx/M,env.vy/M],"rear_slip_estimate_deg":(env.vy-env.yaw_rate*neural_racing_core::WHEELBASE*0.5).atan2(env.vx).to_degrees(),"rear_slip_estimate_contract":"kinematic bicycle estimate from public body velocity/yaw and display wheelbase; NOT native wheel slip","tyre_usage":env.tyre_usage}));
            let remainder_before=steering_remainder;
            if tick % 4 == 0 {
                let desired = instant.as_ref().unwrap();
                held_desired=[desired.steer,desired.throttle];
                steering_remainder += desired.steer;
                action.steer = if steering_remainder.abs() > 0.5 {
                    steering_remainder.signum()
                } else {
                    0.0
                };
                steering_remainder -= action.steer;
                action.throttle = if desired.throttle.abs() > 0.15 {
                    desired.throttle.signum()
                } else {
                    0.0
                };
            }
            let outcome = env.step(action);
            if trace_dir.is_some() {
                let desired=instant.as_ref().unwrap();
                trace.push_back(json!({"tick":tick,"seconds":env.steps as f64*neural_racing_core::DECISION_DT,"controller_update":tick%4==0,"desired_current":[desired.steer,desired.throttle],"desired_at_last_update":held_desired,"binary_command":[action.steer,action.throttle],"steering_remainder_before":remainder_before,"steering_remainder_after":steering_remainder,"before":before,"after":{"state":state(&env),"heading_error_rad":env.heading_error,"yaw_rate_rad_s":env.yaw_rate,"cg_slip_deg":env.slip_angle_deg,"body_velocity_mps":[env.vx/M,env.vy/M],"rear_slip_estimate_deg":(env.vy-env.yaw_rate*neural_racing_core::WHEELBASE*0.5).atan2(env.vx).to_degrees()}}));
                if trace.len()>40 {trace.pop_front();}
            }
            if !finite(&env) {
                return Err(format!("nonfinite keyboard drive {}", state(&env)));
            }
            if outcome.done {
                break;
            }
        }
        if let Some(dir)=trace_dir {
            let path=std::path::Path::new(&dir).join(format!("{id}-{cruise}mps.json"));
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            let condition=match circuit {Circuit::Gt4(i)=>format!("{:?}",course_catalog::courses()[usize::from(i)].condition),_=>"legacy".into()};
            std::fs::write(path,serde_json::to_vec_pretty(&json!({"course":id,"source_course_condition":condition,"target_mps":cruise,"decision_dt":neural_racing_core::DECISION_DT,"command_period_seconds":0.2,"last_decisions":trace})).unwrap()).unwrap();
        }
        if !env.lap_done {
            return Err(format!("5Hz keyboard driver failed {}", state(&env)));
        }
        Ok(state(&env))
    }))
    .unwrap_or_else(|_| Err("keyboard driver panicked".into()))
}

#[test]
#[ignore = "release-mode 5Hz keyboard-command feasibility on representative imported routes"]
fn representative_imported_routes_accept_keyboard_commands_at_human_intervals() {
    let receipt = input_receipt();
    let mut rows = Vec::new();
    let mut failures = Vec::new();
    for id in [
        "gt4-highspeedring",
        "gt4-tsukuba_wet",
        "gt4-suzuka",
        "gt4-yosemite_dirt_short",
    ] {
        let result = run_keyboard(id, 15.0);
        match result {
            Ok(data) => {
                eprintln!("{id} keyboard PASS");
                rows.push(json!({"course":id,"ok":true,"data":data}));
            }
            Err(error) => {
                eprintln!("{id} keyboard FAIL {error}");
                failures.push(format!("{id}: {error}"));
                rows.push(json!({"course":id,"ok":false,"error":error}));
            }
        }
    }
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../.audit/gt4-tracks/keyboard-course-driving.json");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path,serde_json::to_vec_pretty(&json!({"driver":"exact legacy5Hz binary-command test driver;15m/s target; no training/human competence claim","input_receipt":receipt,"courses":rows,"failures":failures.len()})).unwrap()).unwrap();
    assert!(failures.is_empty(), "{}", failures.join("\n"));
}

#[test]
#[ignore = "diagnostic sweep; retains the lower-speed keyboard results next to the 15 m/s checkpoint"]
fn keyboard_speed_sweep_retains_lower_speed_results() {
    let mut rows = Vec::new();
    for id in [
        "gt4-highspeedring",
        "gt4-tsukuba_wet",
        "gt4-suzuka",
        "gt4-yosemite_dirt_short",
    ] {
        for cruise in [8.0, 10.0, 12.0, 15.0] {
            let (ok, data, error) = match run_keyboard(id, cruise) {
                Ok(data) => (true, Some(data), None),
                Err(error) => (false, None, Some(error)),
            };
            rows.push(json!({
                "course": id,
                "target_mps": cruise,
                "ok": ok,
                "data": data,
                "error": error,
            }));
        }
    }
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../.audit/track-visual-fix/keyboard-speed-sweep.json");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(
        path,
        serde_json::to_vec_pretty(&json!({
            "driver": "same 5Hz binary-command test driver at 8/10/12/15 m/s; no physics or guide changes",
            "courses": rows,
        }))
        .unwrap(),
    )
    .unwrap();
}

#[test]
fn all_original_driving_resources_remain_available() {
    let available: Vec<_> = course_catalog::courses()
        .iter()
        .filter(|c| c.available)
        .collect();
    assert_eq!(course_catalog::courses().len(), 110);
    assert_eq!(
        available.len(),
        95,
        "do not make failures disappear by disabling original driving resources"
    );
    assert_eq!(
        available
            .iter()
            .filter(|c| c.topology == course_catalog::CourseTopology::Area)
            .count(),
        6
    );
}

#[test]
#[ignore = "separate12m/s terrain-aware5Hzkeyboard feasibility; retains15m/s failure checkpoint"]
fn dirt_route_keyboard_feasibility_at_twelve_metres_per_second() {
    let receipt = input_receipt();
    let result = run_keyboard("gt4-yosemite_dirt_short", 12.0);
    let (ok, data, error) = match result {
        Ok(data) => (true, Some(data), None),
        Err(error) => (false, None, Some(error)),
    };
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../.audit/gt4-tracks/keyboard-dirt-12mps.json");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(path,serde_json::to_vec_pretty(&json!({"course":"gt4-yosemite_dirt_short","target_mps":12.0,"driver":"same exact5Hz binary controller; separate slower target; no physics changes","baseline_15mps":"retained in checkpoint95-round1-final-keyboard-course-driving.json","input_receipt":receipt,"ok":ok,"data":data,"error":error})).unwrap()).unwrap();
    assert!(ok, "12m/s keyboard drive failed: {error:?}");
}
