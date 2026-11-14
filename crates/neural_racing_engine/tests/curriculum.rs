#[path = "../../neural_racing_core/tests/common/mod.rs"]
mod common;
use neural_racing_core::{Circuit, Env, UNITS_PER_METRE};
use neural_racing_engine::curriculum::{section_exit, Curriculum};

#[test]
fn generated_corner_lessons_are_physically_feasible_with_the_shared_car() {
    for circuit in Circuit::ALL {
        let track = circuit.build();
        let curriculum = Curriculum::new(&track);
        for level in 0..5 {
            for lesson in curriculum.validation_lessons(&track, level) {
                let mut env = Env::new(track.clone(), 5);
                env.reset_for_section(lesson.start_arc, lesson.entry_speed);
                let mut success = false;
                for _ in 0..1800 {
                    // Independent test-only driver. These actions never enter training.
                    let action = common::pursuit(&env, 15.0 * UNITS_PER_METRE);
                    let outcome = env.step(action);
                    if outcome.done {
                        break;
                    }
                    if section_exit(&env, lesson) {
                        success = true;
                        break;
                    }
                }
                assert!(success, "{} corner {} level {} start {:.1}m speed {:.1}m/s progress {:.2} offset {:.2}m heading {:.2}", circuit.slug(),lesson.corner,level,lesson.start_arc / UNITS_PER_METRE,lesson.entry_speed / UNITS_PER_METRE,env.distance / lesson.distance, env.lateral_offset / UNITS_PER_METRE,env.heading_error);
                assert_eq!(env.credited_laps(), 0);
                assert_eq!(env.next_checkpoint, 0);
            }
        }
    }
}
