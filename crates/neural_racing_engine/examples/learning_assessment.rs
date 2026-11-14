//! Uncapped live-algorithm assessment with retained policies and noise-free audits.
//! Usage: learning_assessment SECONDS SEED CIRCUIT OUTPUT_DIRECTORY
use neural_racing_core::{
    Action, Circuit, Env, CHECKPOINT_COUNT, DECISION_DT, NUM_RAYS, OBS_DIM, UNITS_PER_METRE,
};
use neural_racing_diagnostics::json;
use neural_racing_engine::curriculum::{section_exit, Curriculum};
use neural_racing_engine::live::{LiveTrainer, CAR_COUNT, STINT_LAPS};
use neural_racing_ml::Policy;
use neural_racing_ml::CONTROL_HOLD;
use std::{
    path::PathBuf,
    time::{Duration, Instant},
};

fn audit(policy: &Policy, circuit: Circuit, checkpoint: usize) -> neural_racing_diagnostics::Value {
    let mut env = Env::new(circuit.build(), STINT_LAPS);
    env.reset_at_checkpoint(checkpoint);
    let mut reward = 0.0;
    let mut action = [0.0; 2];
    let mut brake_steps = 0;
    loop {
        if env.steps % CONTROL_HOLD as i32 == 0 {
            action = policy.mean_action(&env.obs);
        }
        brake_steps += u64::from(action[1] < -0.1);
        let outcome = env.step(Action {
            steer: action[0] as f64,
            throttle: action[1] as f64,
        });
        reward += outcome.reward;
        if outcome.done {
            break;
        }
    }
    json!({"circuit":circuit.slug(),"start_checkpoint":checkpoint,
        "lap_progress":env.distance / env.track.lap_length,"completed_laps":env.credited_laps(),
        "five_lap_goal":env.lap_done,"crashed":env.off_track,"steps":env.steps,
        "sim_seconds":env.steps as f64 * DECISION_DT,"reward":reward,
        "brake_fraction":brake_steps as f64 / env.steps.max(1) as f64,
        "final_speed_mps":env.speed / UNITS_PER_METRE,
        "final_heading_error":env.heading_error,"final_lateral_offset_metres":env.lateral_offset / UNITS_PER_METRE})
}

fn corner_audit(policy: &Policy, circuit: Circuit) -> neural_racing_diagnostics::Value {
    let track = circuit.build();
    let curriculum = Curriculum::new(&track);
    let mut results = Vec::new();
    for level in [0, 2, 4] {
        for lesson in curriculum.validation_lessons(&track, level) {
            let mut env = Env::new(track.clone(), 5);
            env.reset_for_section(lesson.start_arc, lesson.entry_speed);
            let mut action = [0.0; 2];
            let mut success = false;
            let mut brakes = 0;
            while env.steps < 1800 {
                if env.steps % CONTROL_HOLD as i32 == 0 {
                    action = policy.mean_action(&env.obs);
                }
                brakes += u64::from(action[1] < -0.1);
                let outcome = env.step(Action {
                    steer: action[0] as f64,
                    throttle: action[1] as f64,
                });
                if outcome.done {
                    break;
                }
                if section_exit(&env, lesson) {
                    success = true;
                    break;
                }
            }
            results.push(
                json!({"corner":lesson.corner,"level":level,"success":success,
                "progress":(env.distance / lesson.distance).clamp(0.0,1.0),"crashed":env.off_track,
                "final_speed_mps":env.speed / UNITS_PER_METRE,
                "brake_fraction":brakes as f64 / env.steps.max(1) as f64}),
            );
        }
    }
    json!({"circuit":circuit.slug(),"results":results})
}

fn main() {
    let _diagnostics = neural_racing_diagnostics::init("learning_assessment");
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 5, "SECONDS SEED CIRCUIT OUTPUT_DIRECTORY");
    let seconds: u64 = args[1].parse().expect("integer seconds");
    assert!(seconds > 0);
    let seed: u64 = args[2].parse().expect("integer seed");
    let circuit = Circuit::ALL
        .into_iter()
        .find(|c| c.slug() == args[3])
        .expect("circuit");
    let directory = PathBuf::from(&args[4]);
    std::fs::create_dir_all(&directory).expect("output directory");
    let mut live = LiveTrainer::new(circuit.build(), seed);
    println!(
        "{}",
        json!({"event":"assessment_config","seconds":seconds,"seed":seed,
        "circuit":circuit.slug(),"pacing":"uncapped","cars":CAR_COUNT,"rays":NUM_RAYS,
        "obs_dim":OBS_DIM,"parameters":live.policy.num_params(),"laps":STINT_LAPS,
        "decision_dt":DECISION_DT,"profile":if cfg!(debug_assertions) {"debug"} else {"release"}})
    );
    println!(
        "{}",
        json!({"event":"baseline_corner_audit","audit":corner_audit(&live.policy,circuit)})
    );
    live.policy
        .save(&directory.join("initial.bin"))
        .expect("initial policy");
    for c in Circuit::ALL {
        println!(
            "{}",
            json!({"event":"baseline_audit","audit":audit(&live.policy,c,0)})
        );
    }
    let start = Instant::now();
    let mut next_report = 10;
    let mut next_audit = 60;
    let mut audit_seconds = 0.0;
    while start.elapsed() < Duration::from_secs(seconds) {
        live.step().expect("live training step");
        let elapsed = start.elapsed().as_secs_f64();
        if elapsed >= next_report as f64 {
            let stats = live.current_stats();
            println!(
                "{}",
                json!({"event":"training_progress","wall_seconds":elapsed,
                "updates":live.updates,"attempts":stats.attempts,"decisions":live.decisions,
                "completed_laps":stats.laps,"five_lap_goals":stats.completed_stints,
                "best_streak":stats.best_streak,"restores":live.restores,
                "section_attempts":stats.section_attempts,"section_successes":stats.section_successes,
                "curriculum_levels":live.curriculum.as_ref().map(Curriculum::levels),
                "pedal_counts":live.pedal_counts,"saturated_controls":live.saturated_controls,
                "recent_mean_goal_progress":stats.history.iter().map(|a|a.progress).sum::<f64>() / stats.history.len().max(1) as f64,
                "per_car_speed":live.decisions as f64 * DECISION_DT / CAR_COUNT as f64 / elapsed})
            );
            next_report += 10;
        }
        if elapsed >= next_audit as f64 && elapsed < seconds as f64 {
            let audit_start = Instant::now();
            live.policy
                .save(&directory.join(format!("at-{next_audit}s.bin")))
                .expect("audit policy");
            println!(
                "{}",
                json!({"event":"checkpoint_audit","wall_seconds":elapsed,
                "updates":live.updates,"audit":audit(&live.policy,circuit,0)})
            );
            audit_seconds += audit_start.elapsed().as_secs_f64();
            next_audit += 60;
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    live.diagnostic_progress();
    live.policy
        .save(&directory.join("final.bin"))
        .expect("final policy");
    let stats = live.current_stats();
    let update = live.last_update;
    println!(
        "{}",
        json!({"event":"training_finished","wall_seconds":elapsed,
        "checkpoint_audit_seconds":audit_seconds,"updates":live.updates,"decisions":live.decisions,
        "attempts":stats.attempts,"completed_laps":stats.laps,"five_lap_goals":stats.completed_stints,
        "best_streak":stats.best_streak,"best_lap_seconds":stats.best_lap,"restores":live.restores,
        "section_attempts":stats.section_attempts,"section_successes":stats.section_successes,
        "curriculum_levels":live.curriculum.as_ref().map(Curriculum::levels),
        "pedal_counts":live.pedal_counts,"saturated_controls":live.saturated_controls,
        "per_car_speed":live.decisions as f64 * DECISION_DT / CAR_COUNT as f64 / elapsed,
        "simulated_hours_per_car":live.decisions as f64 * DECISION_DT / CAR_COUNT as f64 / 3600.0,
        "last_policy_kl":update.map(|u|u.policy_kl),"last_update_scale":update.map(|u|u.update_scale),
        "last_explained_variance":update.map(|u|u.explained_variance),
        "last_exploration_sigma":update.map(|u|u.exploration_sigma)})
    );
    // Reload the retained checkpoint so the final assessment also verifies its contents.
    let saved = Policy::load(&directory.join("final.bin")).expect("reload final policy");
    assert_eq!(saved.params(), live.policy.params());
    assert_eq!(saved.obs_dim, OBS_DIM);
    println!(
        "{}",
        json!({"event":"final_corner_audit","audit":corner_audit(&saved,circuit)})
    );
    for c in Circuit::ALL {
        println!(
            "{}",
            json!({"event":"final_audit","audit":audit(&saved,c,0)})
        );
    }
    for car in 1..CAR_COUNT {
        println!(
            "{}",
            json!({"event":"final_practice_start_audit",
            "audit":audit(&saved,circuit,car * CHECKPOINT_COUNT / CAR_COUNT)})
        );
    }
}
