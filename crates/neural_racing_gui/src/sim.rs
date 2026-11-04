//! One race in progress: the environment plus the visual leftovers.

use neural_racing_core::{Action, Circuit, Env, Rng, Track, Vec2, DECISION_DT};
use neural_racing_ml::Policy;
use std::{borrow::Cow, time::Instant};

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

#[derive(Clone, Copy)]
struct Pose {
    x: f64,
    y: f64,
    heading: f64,
    steer: f64,
}
impl Pose {
    fn of(env: &Env) -> Self {
        Self {
            x: env.x,
            y: env.y,
            heading: env.heading,
            steer: env.steer_angle,
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
    previous_pose: Option<Pose>,
    pose_received: Instant,
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
            previous_pose: None,
            pose_received: Instant::now(),
        }
    }

    pub fn reset(&mut self, track_seed: u64, laps: i32) {
        self.env = Env::new(Track::random(track_seed), laps);
        self.track_seed = track_seed;
        self.trail.clear();
        self.previous_pose = None;
        self.skids.clear();
        self.outcome = None;
        self.steps_at_end = 0;
        self.laps_done = 0;
        self.lap_start_step = 0;
        self.lap_times.clear();
        self.min_wall = f64::INFINITY;
        self.rng = Rng::new(track_seed ^ 0x51ed_270b);
    }

    pub fn on_circuit(circuit: Circuit, laps: i32) -> Self {
        let mut race = Self::new(424_242, laps);
        race.env = Env::new(circuit.build(), laps);
        race
    }

    /// Adopt an actual training car. Trails contain received live positions;
    /// this method never runs a second demonstration simulation.
    pub fn receive(&mut self, env: Env) {
        if env.steps < self.env.steps || env.track.circuit != self.env.track.circuit {
            self.trail.clear();
            self.skids.clear();
            self.lap_times.clear();
        }
        if env.steps != self.env.steps {
            self.previous_pose = if env.steps > self.env.steps
                && !self.env.off_track
                && !self.env.lap_done
                && env.track.circuit == self.env.track.circuit
            {
                Some(Pose::of(&self.env))
            } else {
                None
            };
            self.pose_received = Instant::now();
        }
        self.env = env;
        self.laps_done = self.env.laps_completed;
        self.outcome = if self.env.lap_done {
            Some(Outcome::Finished {
                seconds: self.elapsed(),
            })
        } else if self.env.off_track {
            Some(Outcome::Crashed {
                seconds: self.elapsed(),
            })
        } else {
            None
        };
        self.trail.push(self.env.position());
        if self.trail.len() > 180 {
            self.trail.remove(0);
        }
    }

    /// Smooth the 20 Hz control states at viewing speeds. High-speed training
    /// uses the actual latest pose, avoiding interpolation across whole corners.
    pub fn drawing_env(&self, speed: f32, paused: bool) -> Cow<'_, Env> {
        if paused || speed > 4.0 || self.is_over() {
            return Cow::Borrowed(&self.env);
        }
        let Some(before) = self.previous_pose else {
            return Cow::Borrowed(&self.env);
        };
        let t = (self.pose_received.elapsed().as_secs_f64() * speed as f64 / DECISION_DT)
            .clamp(0.0, 1.0);
        let mut env = self.env.clone();
        env.x = before.x + (env.x - before.x) * t;
        env.y = before.y + (env.y - before.y) * t;
        env.heading =
            before.heading + neural_racing_core::wrap_angle(env.heading - before.heading) * t;
        env.steer_angle = before.steer + (env.steer_angle - before.steer) * t;
        Cow::Owned(env)
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
                    let action = policy.mean_action(&self.env.obs);
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
        self.previous_pose = Some(Pose::of(&self.env));
        self.pose_received = Instant::now();
        let outcome = self.env.step(action);
        self.min_wall = self.min_wall.min(self.env.min_ray);
        for &point in &self.env.path {
            if self
                .trail
                .last()
                .is_none_or(|last| (*last - point).norm() > 0.5)
            {
                self.trail.push(point);
                if self.trail.len() > 720 {
                    self.trail.remove(0);
                }
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
            self.lap_times
                .push(f64::from(outcome.info.steps - self.lap_start_step) * DECISION_DT);
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

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn viewing_speed_interpolates_pose_without_changing_simulation() {
        let mut race = RaceView::on_circuit(Circuit::Monza, 1);
        race.previous_pose = Some(Pose {
            x: race.env.x - 10.0,
            y: race.env.y,
            heading: 3.13,
            steer: -0.1,
        });
        race.env.heading = -3.13;
        race.env.steer_angle = 0.1;
        race.pose_received = Instant::now() - std::time::Duration::from_secs_f64(DECISION_DT * 0.5);
        let actual = race.env.position();
        let drawing = race.drawing_env(1.0, false);
        assert!(drawing.x > actual.x - 10.0 && drawing.x < actual.x);
        assert!(
            drawing.heading.abs() > 3.0,
            "interpolate across the short angular wrap"
        );
        assert!(drawing.steer_angle.abs() < 0.05);
        assert_eq!(race.env.position(), actual);
        assert_eq!(race.drawing_env(1000.0, false).position(), actual);
        assert_eq!(race.drawing_env(1.0, true).position(), actual);
    }
}
