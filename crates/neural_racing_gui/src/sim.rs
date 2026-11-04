//! One race in progress: the environment plus the visual leftovers.

use neural_racing_core::{Action, Env, Rng, Track, Vec2, DECISION_DT};
use neural_racing_ml::Policy;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    Finished { seconds: f64 },
    Crashed { seconds: f64 },
    TimedOut { seconds: f64 },
}

impl Outcome {
    pub fn label(self) -> &'static str {
        match self {
            Outcome::Finished { .. } => "FINISHED",
            Outcome::Crashed { .. } => "OFF TRACK",
            Outcome::TimedOut { .. } => "TIME UP",
        }
    }

    pub fn seconds(self) -> f64 {
        match self {
            Outcome::Finished { seconds }
            | Outcome::Crashed { seconds }
            | Outcome::TimedOut { seconds } => seconds,
        }
    }
}

pub struct RaceView {
    pub env: Env,
    pub track_seed: u64,
    pub trail: Vec<Vec2>,
    pub skids: Vec<(Vec2, f64)>,
    pub outcome: Option<Outcome>,
    pub steps_at_end: i32,
    pub laps_done: i32,
    pub lap_start_step: i32,
    pub lap_times: Vec<f64>,
    pub min_wall: f64,
    rng: Rng,
}

impl RaceView {
    pub fn new(track_seed: u64, laps: i32) -> Self {
        Self {
            env: Env::new(Track::random(track_seed), laps),
            track_seed,
            trail: Vec::with_capacity(400),
            skids: Vec::with_capacity(400),
            outcome: None,
            steps_at_end: 0,
            laps_done: 0,
            lap_start_step: 0,
            lap_times: Vec::new(),
            min_wall: f64::INFINITY,
            rng: Rng::new(track_seed ^ 0x51ed_270b),
        }
    }

    pub fn reset(&mut self, track_seed: u64, laps: i32) {
        self.env = Env::new(Track::random(track_seed), laps);
        self.track_seed = track_seed;
        self.trail.clear();
        self.skids.clear();
        self.outcome = None;
        self.steps_at_end = 0;
        self.laps_done = 0;
        self.lap_start_step = 0;
        self.lap_times.clear();
        self.min_wall = f64::INFINITY;
        self.rng = Rng::new(track_seed ^ 0x51ed_270b);
    }

    pub fn is_over(&self) -> bool {
        self.outcome.is_some()
    }

    pub fn elapsed(&self) -> f64 {
        f64::from(self.env.steps) * DECISION_DT
    }

    /// One decision. Returns `true` once the race has just ended.
    pub fn advance(&mut self, policy: Option<&Policy>, manual: Option<Action>) -> bool {
        if self.outcome.is_some() {
            return false;
        }
        let action = match manual {
            Some(action) => action,
            None => match policy {
                Some(policy) => {
                    let (action, _) = policy.act(&self.env.obs, 0.0, &mut self.rng);
                    Action {
                        steer: f64::from(action[0]),
                        throttle: f64::from(action[1]),
                    }
                }
                None => Action {
                    steer: 0.0,
                    throttle: 0.0,
                },
            },
        };
        let outcome = self.env.step(action);
        self.min_wall = self.min_wall.min(self.env.min_ray);
        let position = self.env.position();
        if self.trail.last().is_none_or(|last| (*last - position).norm() > 2.0) {
            self.trail.push(position);
            if self.trail.len() > 420 {
                self.trail.remove(0);
            }
        }
        if self.env.slip_angle_deg.abs() > 6.5 {
            let cos = self.env.heading.cos();
            let sin = self.env.heading.sin();
            let skid = Vec2 {
                x: self.env.x - 11.0 * cos,
                y: self.env.y - 11.0 * sin,
            };
            self.skids.push((skid, self.env.heading));
            if self.skids.len() > 420 {
                self.skids.remove(0);
            }
        }
        if outcome.info.laps_completed > self.laps_done {
            self.lap_times.push(
                f64::from(outcome.info.steps - self.lap_start_step) * DECISION_DT,
            );
            self.laps_done = outcome.info.laps_completed;
            self.lap_start_step = outcome.info.steps;
        }
        if outcome.done {
            let seconds = self.elapsed();
            self.steps_at_end = outcome.info.steps;
            self.outcome = Some(if outcome.info.lap_done {
                Outcome::Finished { seconds }
            } else if outcome.info.off_track {
                Outcome::Crashed { seconds }
            } else {
                Outcome::TimedOut { seconds }
            });
            return true;
        }
        false
    }

    pub fn gates_done(&self) -> i32 {
        self.env.next_checkpoint
    }

    pub fn gates_total(&self) -> i32 {
        self.env.total_gates()
    }

    pub fn best_lap(&self) -> Option<f64> {
        self.lap_times.iter().copied().reduce(f64::min)
    }
}
