//! Successful corner exits unlock longer approaches, then full laps and stints.
//! Starts vary; targets describe real road sections, never an expert action.
use neural_racing_core::{wrap_angle, Env, Rng, Track, UNITS_PER_METRE as M};

const LEVELS: u8 = 5;
const WINDOW: u32 = 20;

#[derive(Clone, Copy, Debug)]
pub struct Lesson {
    pub corner: usize,
    pub level: u8,
    pub start_arc: f64,
    pub distance: f64,
    pub entry_speed: f64,
}

#[derive(Clone)]
struct Corner {
    apex: f64,
    entry: f64,
    exit: f64,
    level: u8,
    attempts: u32,
    successes: u32,
}

pub struct Curriculum {
    corners: Vec<Corner>,
    next: usize,
    starts: u64,
    review_next: usize,
    pub section_attempts: u64,
    pub section_successes: u64,
    lap_attempts: u32,
    lap_successes: u32,
    stint_ready: bool,
    closed_route: bool,
}

impl Curriculum {
    pub fn new(track: &Track) -> Self {
        let samples = (track.lap_length / (5.0 * M)).ceil() as usize;
        let spacing = track.lap_length / samples as f64;
        let curvature = |arc: f64| {
            wrap_angle(
                track.arc_tangent(arc + 10.0 * M).angle()
                    - track.arc_tangent(arc - 10.0 * M).angle(),
            )
            .abs()
        };
        let mut candidates: Vec<_> = (0..samples)
            .map(|i| (i as f64 * spacing, curvature(i as f64 * spacing)))
            .filter(|(_, bend)| *bend > 0.08)
            .collect();
        candidates.sort_by(|a, b| b.1.total_cmp(&a.1));
        let mut corners: Vec<Corner> = Vec::new();
        for (apex, bend) in candidates {
            if corners.iter().any(|c| {
                let d = (c.apex - apex).abs();
                (if track.closed {
                    d.min(track.lap_length - d)
                } else {
                    d
                }) < 120.0 * M
            }) {
                continue;
            }
            let mut entry = apex;
            let mut exit = apex;
            for _ in 0..16 {
                if curvature(entry - spacing) < bend * 0.2 {
                    break;
                }
                if !track.closed && entry - spacing < 0.0 {
                    break;
                }
                entry -= spacing;
            }
            for _ in 0..16 {
                if curvature(exit + spacing) < bend * 0.2 {
                    break;
                }
                if !track.closed && exit + spacing > track.lap_length {
                    break;
                }
                exit += spacing;
            }
            corners.push(Corner {
                apex,
                entry,
                exit: if track.closed {
                    exit + 30.0 * M
                } else {
                    (exit + 30.0 * M).min(track.lap_length)
                },
                level: 0,
                attempts: 0,
                successes: 0,
            });
            if corners.len() == 16 {
                break;
            }
        }
        if !track.closed {
            corners.retain(|c| c.exit - c.apex > 5.0 * M);
        }
        corners.sort_by(|a, b| a.apex.total_cmp(&b.apex));
        Self {
            corners,
            next: 0,
            starts: 0,
            review_next: 0,
            section_attempts: 0,
            section_successes: 0,
            lap_attempts: 0,
            lap_successes: 0,
            stint_ready: false,
            closed_route: track.closed,
        }
    }

    pub fn mastered(&self) -> usize {
        self.corners.iter().filter(|c| c.level == LEVELS).count()
    }
    pub fn corner_count(&self) -> usize {
        self.corners.len()
    }
    pub fn levels(&self) -> Vec<u8> {
        self.corners.iter().map(|c| c.level).collect()
    }
    pub fn stint_ready(&self) -> bool {
        self.stint_ready
    }

    /// Fixed starts outside the training jitter range, at each chosen difficulty.
    /// These evaluations never update mastery or enter the PPO batch.
    pub fn validation_lessons(&self, track: &Track, level: u8) -> Vec<Lesson> {
        assert!(level < LEVELS);
        self.corners
            .iter()
            .enumerate()
            .map(|(corner, c)| {
                let start = match level {
                    0 => c.apex - 10.0 * M,
                    1 => c.entry - 25.0 * M,
                    2 => c.entry - 80.0 * M,
                    3 => c.entry - 160.0 * M,
                    _ => c.entry - 250.0 * M,
                } - 15.0 * M;
                let start = if track.closed { start } else { start.max(0.0) };
                Lesson {
                    corner,
                    level,
                    start_arc: if track.closed {
                        start.rem_euclid(track.lap_length)
                    } else {
                        start
                    },
                    distance: (c.exit - start).min(track.lap_length * 0.4),
                    entry_speed: (track.speed_profile(start)
                        * [0.4, 0.55, 0.7, 0.85, 1.0][level as usize]
                        * 1.05)
                        .clamp(5.0 * M, 65.0 * M),
                }
            })
            .collect()
    }

    pub fn start(&mut self, env: &mut Env, rng: &mut Rng) -> Option<Lesson> {
        if self.mastered() == self.corners.len() {
            env.laps = if self.closed_route && self.stint_ready {
                super::live::STINT_LAPS
            } else {
                1
            };
            // Full laps always include a normal-start attempt, plus varied gates.
            let checkpoint = if !env.track.closed || self.next % 4 == 0 {
                0
            } else {
                self.next % 8
            };
            self.next += 1;
            env.reset_at_checkpoint(checkpoint);
            return None;
        }
        // One in four starts reviews a mastered corner, preventing the shared
        // network from forgetting easy turns while concentrating on hard ones.
        self.starts += 1;
        let review = self.starts % 4 == 0 && self.mastered() > 0;
        let corner = if review {
            let corner = (self.review_next..self.review_next + self.corners.len())
                .find(|i| self.corners[i % self.corners.len()].level == LEVELS)
                .unwrap()
                % self.corners.len();
            self.review_next = corner + 1;
            corner
        } else {
            let corner = (self.next..self.next + self.corners.len())
                .find(|i| self.corners[i % self.corners.len()].level < LEVELS)
                .unwrap()
                % self.corners.len();
            self.next = corner + 1;
            corner
        };
        let c = &self.corners[corner];
        let level = c.level.min(LEVELS - 1);
        let start = match level {
            0 => c.apex - 10.0 * M,
            1 => c.entry - 25.0 * M,
            2 => c.entry - 80.0 * M,
            3 => c.entry - 160.0 * M,
            _ => c.entry - 250.0 * M,
        } - rng.range(0.0, 10.0 * M);
        let start = if env.track.closed {
            start
        } else {
            start.max(0.0)
        };
        let speed_fraction = [0.4, 0.55, 0.7, 0.85, 1.0][level as usize];
        let speed = (track_speed(env, start) * speed_fraction * rng.range(0.9, 1.0))
            .clamp(5.0 * M, 65.0 * M);
        let lesson = Lesson {
            corner,
            level,
            start_arc: if env.track.closed {
                start.rem_euclid(env.track.lap_length)
            } else {
                start
            },
            distance: (c.exit - start).min(env.track.lap_length * 0.4),
            entry_speed: speed,
        };
        env.laps = if self.closed_route {
            super::live::STINT_LAPS
        } else {
            1
        };
        env.reset_for_section(lesson.start_arc, speed);
        Some(lesson)
    }

    /// A sampled exit alone cannot promote a corner: at least 16/20 successes
    /// at each difficulty, including its approach, are required.
    pub fn record(&mut self, lesson: Option<Lesson>, success: bool) -> bool {
        if let Some(lesson) = lesson {
            self.section_attempts += 1;
            self.section_successes += u64::from(success);
            let c = &mut self.corners[lesson.corner];
            let reviewing = c.level == LEVELS && lesson.level == LEVELS - 1;
            if c.level != lesson.level && !reviewing {
                return false;
            } // another car promoted it
            c.attempts += 1;
            c.successes += u32::from(success);
            if c.attempts == WINDOW {
                let promote = !reviewing && c.successes >= 16;
                let regress = (reviewing && c.successes < 12)
                    || (!reviewing && c.level > 0 && c.successes < 4);
                c.attempts = 0;
                c.successes = 0;
                if promote {
                    c.level = (c.level + 1).min(LEVELS);
                    return true;
                }
                if regress {
                    c.level = c.level.saturating_sub(1);
                    return true;
                }
            }
        } else if self.closed_route && !self.stint_ready {
            self.lap_attempts += 1;
            self.lap_successes += u32::from(success);
            if self.lap_attempts == WINDOW {
                self.stint_ready = self.lap_successes >= 16;
                self.lap_attempts = 0;
                self.lap_successes = 0;
                return self.stint_ready;
            }
        }
        false
    }
}

fn track_speed(env: &Env, arc: f64) -> f64 {
    env.track.speed_profile(arc)
}

pub fn section_exit(env: &Env, lesson: Lesson) -> bool {
    !env.off_track
        && env.distance >= lesson.distance
        && env.speed >= 3.0 * M
        && env.speed <= 1.1 * env.speed_profile
        && env.lateral_offset.abs() <= 0.75 * env.road_half_width
        && env.heading_error.abs() < 0.6
}

#[cfg(test)]
mod tests {
    use super::*;
    use neural_racing_core::{Action, Circuit};

    #[test]
    fn reviews_detect_forgotten_corners_and_return_them_to_practice() {
        let track = Circuit::Monza.build();
        let mut c = Curriculum::new(&track);
        c.corners[0].level = LEVELS;
        c.starts = 3;
        let mut env = Env::new(track, 5);
        let lesson = c.start(&mut env, &mut Rng::new(42)).unwrap();
        assert_eq!(lesson.corner, 0);
        assert_eq!(lesson.level, LEVELS - 1);
        for _ in 0..WINDOW {
            c.record(Some(lesson), false);
        }
        assert_eq!(c.levels()[0], LEVELS - 1);
        assert_eq!(c.mastered(), 0);
        assert_eq!(env.credited_laps(), 0);
    }

    #[test]
    fn every_real_track_has_varied_finite_corner_starts_and_no_fake_laps() {
        for circuit in Circuit::ALL {
            let track = circuit.build();
            let mut curriculum = Curriculum::new(&track);
            assert!(curriculum.corner_count() >= 3, "{}", circuit.slug());
            let mut env = Env::new(track, 5);
            let mut rng = Rng::new(42);
            let first = curriculum.start(&mut env, &mut rng).unwrap();
            let second = curriculum.start(&mut env, &mut rng).unwrap();
            assert_ne!(first.corner, second.corner);
            assert!(env.obs.iter().all(|v| v.is_finite()));
            assert!(second.entry_speed >= 5.0 * M);
            assert!(second.distance > 30.0 * M);
            assert_eq!(env.credited_laps(), 0);
            let outcome = env.step(Action {
                steer: 0.0,
                throttle: 0.0,
            });
            assert_eq!(outcome.info.reward_components.checkpoint, 0.0);
            assert!(!env.lap_done);
        }
    }

    #[test]
    fn failures_do_not_advance_and_successes_unlock_longer_faster_approaches_then_laps() {
        let track = neural_racing_core::Circuit::Monza.build();
        let mut c = Curriculum::new(&track);
        let mut env = Env::new(track, 5);
        let mut rng = Rng::new(7);
        let initial = c.start(&mut env, &mut rng).unwrap();
        for _ in 0..40 {
            assert!(!c.record(Some(initial), false));
        }
        assert_eq!(c.levels()[initial.corner], 0);
        for corner in 0..c.corner_count() {
            for level in 0..LEVELS {
                let lesson = Lesson {
                    corner,
                    level,
                    ..initial
                };
                for attempt in 0..20 {
                    assert_eq!(c.record(Some(lesson), true), attempt == 19);
                }
            }
        }
        assert_eq!(c.mastered(), c.corner_count());
        assert!(c.start(&mut env, &mut rng).is_none());
        assert_eq!(env.laps, 1);
        for _ in 0..20 {
            c.record(None, true);
        }
        assert!(c.start(&mut env, &mut rng).is_none());
        assert_eq!(env.laps, 5);
    }
    #[test]
    fn open_route_mastery_never_promotes_to_five_lap_stints() {
        let track = Circuit::lookup("gt4-testcourse_100m").unwrap().build();
        assert!(!track.closed);
        let mut curriculum = Curriculum::new(&track);
        for corner in &mut curriculum.corners {
            corner.level = LEVELS;
        }
        for _ in 0..100 {
            curriculum.record(None, true);
        }
        assert!(!curriculum.stint_ready());
        let mut env = Env::new(track, 5);
        assert!(curriculum.start(&mut env, &mut Rng::new(7)).is_none());
        assert_eq!(env.laps, 1);
        assert_eq!(env.credited_laps(), 0);
    }
}
