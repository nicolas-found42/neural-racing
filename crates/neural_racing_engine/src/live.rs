//! Four actual training cars, advanced in fixed physics decisions. No display,
//! wall clock, threads or persistence: scheduling belongs to the caller.

use std::collections::VecDeque;

use neural_racing_core::{Action, Circuit, Env, Rng, Track, DECISION_DT, OBS_DIM, UNITS_PER_METRE};
use neural_racing_ml::{
    update, Adam, Batch, Policy, Trajectory, UpdateStats, DEFAULT_BLOCKS, DEFAULT_WIDTH, LR,
};

pub const CAR_COUNT: usize = 4;
/// Numeric pacing range; +infinity requests uncapped live training.
pub const MAX_REQUESTED_SPEED: f64 = 2000.0;
pub fn normalize_speed(speed: f64) -> f64 {
    if speed == f64::INFINITY {
        speed
    } else if speed.is_finite() {
        speed.clamp(0.25, MAX_REQUESTED_SPEED)
    } else {
        1.0
    }
}
pub const STINT_LAPS: i32 = 5;
/// Bound optimizer work and trajectory memory, even for a full real-world lap.
pub const UPDATE_ROUNDS: usize = 4096;
/// Minimum rows per optimizer step. Short attempts (a crash after a few metres)
/// would otherwise shrink an update to a few dozen rows and turn the policy
/// step into noise exactly when the critic needs the most data.
pub const MIN_UPDATE_ROWS: usize = 1024;
pub const MAX_HISTORY: usize = 160;
const STALL_STEPS: i32 = 200; // ten simulated seconds without five metres of progress
/// A deterministic audit runs every this many policy updates.
const SNAPSHOT_UPDATES: u64 = 200;
/// Decisions per audit; enough to separate a launch from a stall.
const SNAPSHOT_STEPS: usize = 1200;
/// Restore the best snapshot when the mean policy falls below this fraction of it.
const SNAPSHOT_TRIGGER: f64 = 0.4;
/// Audits below this progress are not worth remembering (a fresh policy).
const SNAPSHOT_MIN_PROGRESS: f64 = 0.02;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EndReason {
    Goal,
    Crash,
    Stalled,
    Timeout,
}

#[derive(Clone, Debug)]
pub struct Attempt {
    pub number: u64,
    pub progress: f64,
    pub seconds: f64,
    pub reason: EndReason,
}

#[derive(Clone, Default)]
pub struct CircuitStats {
    pub attempts: u64,
    pub laps: u64,
    pub completed_stints: u64,
    pub best_streak: i32,
    pub best_lap: Option<f64>,
    pub history: VecDeque<Attempt>,
}

pub struct LiveTrainer {
    pub policy: Policy,
    pub cars: [Env; CAR_COUNT],
    pub stats: [CircuitStats; 4],
    pub updates: u64,
    pub decisions: u64,
    pub episodes: u64,
    pub last_update: Option<UpdateStats>,
    pub last_end: [Option<EndReason>; CAR_COUNT],
    /// Number of times a collapsed mean policy was replaced by the best snapshot.
    pub restores: u64,
    track: Track,
    best_progress: f64,
    best_params: Option<Vec<Vec<f32>>>,
    optimizer: Adam,
    rngs: [Rng; CAR_COUNT],
    trajectories: [Trajectory; CAR_COUNT],
    batch: Batch,
    rounds: usize,
    completed: [bool; CAR_COUNT],
    progress_mark: [f64; CAR_COUNT],
    progress_step: [i32; CAR_COUNT],
    lap_steps: [i32; CAR_COUNT],
    counted_laps: [i32; CAR_COUNT],
}

impl LiveTrainer {
    pub fn new(track: Track, seed: u64) -> Self {
        let policy = Policy::new(OBS_DIM, DEFAULT_WIDTH, DEFAULT_BLOCKS, seed);
        let optimizer = Adam::new(
            policy.params(),
            LR,
            (0.9, 0.999),
            1e-8,
            crate::MAX_GRAD_NORM,
        );
        Self {
            policy,
            optimizer,
            cars: Self::practice_cars(&track),
            track: track.clone(),
            best_progress: 0.0,
            best_params: None,
            restores: 0,
            stats: std::array::from_fn(|_| CircuitStats::default()),
            rngs: std::array::from_fn(|i| Rng::new(seed.wrapping_add(i as u64 * 7919))),
            trajectories: std::array::from_fn(|_| Trajectory::default()),
            batch: Batch::new(OBS_DIM, 2),
            rounds: 0,
            completed: [false; CAR_COUNT],
            updates: 0,
            decisions: 0,
            episodes: 0,
            last_update: None,
            last_end: [None; CAR_COUNT],
            progress_mark: [0.0; CAR_COUNT],
            progress_step: [0; CAR_COUNT],
            lap_steps: [0; CAR_COUNT],
            counted_laps: [0; CAR_COUNT],
        }
    }

    fn practice_cars(track: &Track) -> [Env; CAR_COUNT] {
        std::array::from_fn(|i| {
            let mut car = Env::new(track.clone(), STINT_LAPS);
            car.reset_at_checkpoint(i * neural_racing_core::CHECKPOINT_COUNT / CAR_COUNT);
            car
        })
    }

    pub fn circuit_index(circuit: Option<Circuit>) -> usize {
        match circuit {
            Some(Circuit::Monza) => 0,
            Some(Circuit::Silverstone) => 1,
            Some(Circuit::RedBullRing) => 2,
            None => 3,
        }
    }
    pub fn current_stats(&self) -> &CircuitStats {
        &self.stats[Self::circuit_index(self.cars[0].track.circuit)]
    }
    /// Keep the model and Adam state, but discard unfinished on-policy samples.
    pub fn select_track(&mut self, track: Track) {
        self.track = track.clone();
        self.best_progress = 0.0;
        self.best_params = None;
        self.cars = Self::practice_cars(&track);
        self.trajectories = std::array::from_fn(|_| Trajectory::default());
        self.batch.clear();
        self.rounds = 0;
        self.completed = [false; CAR_COUNT];
        self.last_end = [None; CAR_COUNT];
        self.progress_mark = [0.0; CAR_COUNT];
        self.progress_step = [0; CAR_COUNT];
        self.lap_steps = [0; CAR_COUNT];
        self.counted_laps = [0; CAR_COUNT];
    }

    /// One decision for each car. All four use the same policy version. The
    /// attempt-based update boundary and sample cap are independent of wall-clock speed and frames.

    pub fn step(&mut self) -> Result<(), &'static str> {
        for i in 0..CAR_COUNT {
            if self.last_end[i].take().is_some() {
                self.cars[i].reset();
                self.progress_mark[i] = 0.0;
                self.progress_step[i] = 0;
                self.lap_steps[i] = 0;
                self.counted_laps[i] = 0;
            }
            // A reset changes the observation, so sample after it.
            let sample = {
                let obs = &self.cars[i].obs;
                self.policy.sample(obs, &mut self.rngs[i])
            };
            let action = Action {
                steer: f64::from(sample.action[0]),
                throttle: f64::from(sample.action[1]),
            };
            let observation = self.cars[i].obs;
            let car = &mut self.cars[i];
            let outcome = car.step(action);
            self.trajectories[i].push(&observation, &sample, outcome.reward as f32);
            if car.distance > self.progress_mark[i] + 5.0 * UNITS_PER_METRE {
                self.progress_mark[i] = car.distance;
                self.progress_step[i] = car.steps;
            }
            let stalled = car.steps - self.progress_step[i] >= STALL_STEPS;
            if stalled && !outcome.done {
                // Combined with the dense low-speed cost, this makes parking
                // worse than crashing into the wall it is avoiding, so a
                // cautious local optimum cannot win.
                *self.trajectories[i].rewards.last_mut().unwrap() -= 40.0;
            }
            let stats = &mut self.stats[Self::circuit_index(car.track.circuit)];
            let credited_laps = car.credited_laps();
            if credited_laps > self.counted_laps[i] {
                let lap_seconds = (car.steps - self.lap_steps[i]) as f64 * DECISION_DT;
                stats.laps += (credited_laps - self.counted_laps[i]) as u64;
                stats.best_streak = stats.best_streak.max(credited_laps);
                stats.best_lap = Some(
                    stats
                        .best_lap
                        .map_or(lap_seconds, |old| old.min(lap_seconds)),
                );
                self.lap_steps[i] = car.steps;
                self.counted_laps[i] = credited_laps;
            }
            if outcome.done || stalled {
                let reason = if car.lap_done {
                    EndReason::Goal
                } else if car.off_track {
                    EndReason::Crash
                } else if stalled {
                    EndReason::Stalled
                } else {
                    EndReason::Timeout
                };
                stats.attempts += 1;
                self.episodes += 1;
                let seconds = car.steps as f64 * DECISION_DT;
                if reason == EndReason::Goal {
                    stats.completed_stints += 1;
                }
                stats.history.push_back(Attempt {
                    number: stats.attempts,
                    progress: (car.distance / (car.track.lap_length * STINT_LAPS as f64))
                        .clamp(0.0, 1.0),
                    seconds,
                    reason,
                });
                if stats.history.len() > MAX_HISTORY {
                    stats.history.pop_front();
                }
                self.trajectories[i].bootstrap = 0.0;
                self.batch.push_trajectory(&self.trajectories[i], true);
                self.trajectories[i].clear();
                self.last_end[i] = Some(reason);
                self.completed[i] = true;
            }
        }
        self.decisions += CAR_COUNT as u64;
        self.rounds += 1;
        let all_done = self.completed.iter().all(|done| *done);
        let ceiling = self.rounds == UPDATE_ROUNDS;
        if all_done || ceiling {
            // Keep complete attempts so corner outcomes credit the earlier
            // braking and steering decisions. The round ceiling bounds memory
            // and update latency when a car survives for several minutes.
            self.push_round_trajectories();
            if all_done {
                self.completed = [false; CAR_COUNT];
            }
            // Accumulate several attempt boundaries into one update unless the
            // round ceiling forces it: a degenerate policy dies in a handful of
            // steps, and one update per death would see almost no data.
            if ceiling || self.batch.rows >= MIN_UPDATE_ROWS {
                self.last_update =
                    Some(update(&mut self.policy, &mut self.optimizer, &self.batch)?);
                self.batch.clear();
                self.rounds = 0;
                self.updates += 1;
                if self.updates % SNAPSHOT_UPDATES == 0 {
                    self.audit_policy();
                }
            }
        }
        Ok(())
    }

    /// Flush every live trajectory to the batch at a round boundary.
    ///
    /// An ended attempt is pushed and cleared in the step that ends it, so any
    /// trajectory still holding rows here is mid-attempt, including one from a
    /// car that finished an earlier attempt and already restarted. Those are
    /// truncated, not terminal: they get the current value as bootstrap so the
    /// round ceiling does not bias returns towards zero.
    fn push_round_trajectories(&mut self) {
        for i in 0..CAR_COUNT {
            if self.trajectories[i].rows == 0 {
                continue;
            }
            self.trajectories[i].bootstrap = self.policy.value(&self.cars[i].obs);
            self.batch.push_trajectory(&self.trajectories[i], false);
            self.trajectories[i].clear();
        }
    }

    /// Progress of the noise-free policy over a bounded run. Exploration keeps
    /// the four training cars alive even when the mean action has collapsed to
    /// a stall, so learning quality is judged on the policy actually shipped.
    fn audit_progress(&self) -> (f64, bool) {
        let mut env = Env::new(self.track.clone(), 1);
        let mut crashed = false;
        for _ in 0..SNAPSHOT_STEPS {
            let action = self.policy.mean_action(&env.obs);
            let outcome = env.step(Action {
                steer: f64::from(action[0]),
                throttle: f64::from(action[1]),
            });
            if outcome.done {
                crashed = outcome.info.off_track;
                break;
            }
        }
        let stopped = !crashed && env.speed / UNITS_PER_METRE < 1.0;
        (env.distance.max(0.0) / self.track.lap_length, stopped)
    }

    fn audit_policy(&mut self) {
        let (progress, stopped) = self.audit_progress();
        if progress > self.best_progress && !stopped {
            self.best_progress = progress;
            self.best_params = Some(self.policy.params().to_vec());
            return;
        }
        if self.best_progress < SNAPSHOT_MIN_PROGRESS
            || (!stopped && progress >= self.best_progress * SNAPSHOT_TRIGGER)
        {
            return;
        }
        let Some(best) = self.best_params.clone() else {
            return;
        };
        for (layer, saved) in self.policy.params_mut().iter_mut().zip(&best) {
            layer.copy_from_slice(saved);
        }
        self.optimizer = Adam::new(
            self.policy.params(),
            LR,
            (0.9, 0.999),
            1e-8,
            crate::MAX_GRAD_NORM,
        );
        self.cars = Self::practice_cars(&self.track);
        self.trajectories = std::array::from_fn(|_| Trajectory::default());
        self.batch.clear();
        self.rounds = 0;
        self.completed = [false; CAR_COUNT];
        self.progress_mark = [0.0; CAR_COUNT];
        self.progress_step = [0; CAR_COUNT];
        self.lap_steps = [0; CAR_COUNT];
        self.counted_laps = [0; CAR_COUNT];
        self.last_end = [None; CAR_COUNT];
        self.restores += 1;
    }
}

/// Wall-time credit is capped to 20ms, preventing catch-up bursts after a slow
/// update, pause, dragged window or sleep. Physics dt never changes.
#[derive(Default)]
pub struct Pacer {
    credit: f64,
}
impl Pacer {
    pub fn reset(&mut self) {
        self.credit = 0.0;
    }
    pub fn accrue(&mut self, elapsed: f64, speed: f64) {
        let speed = if speed.is_finite() {
            speed.clamp(0.25, MAX_REQUESTED_SPEED)
        } else {
            1.0
        };
        self.credit = (self.credit + elapsed.clamp(0.0, 0.02) * speed / DECISION_DT)
            .min((0.02 * speed / DECISION_DT).max(1.0));
    }
    pub fn ready(&self) -> bool {
        self.credit >= 1.0
    }
    pub fn take(&mut self) -> bool {
        if self.credit >= 1.0 {
            self.credit -= 1.0;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn five_lap_goal_counts_each_lap_without_restarting_early() {
        let mut live = LiveTrainer::new(Circuit::Monza.build(), 42);
        assert_eq!(live.policy.num_params(), 350_725);
        for gate in 0..40 {
            let car = &mut live.cars[0];
            let g = car.track.gates()[gate % neural_racing_core::CHECKPOINT_COUNT];
            let tangent = car
                .track
                .arc_tangent((gate % 8 + 1) as f64 * car.track.lap_length / 8.0);
            let position = g.center - tangent * (0.05 * UNITS_PER_METRE);
            car.x = position.x;
            car.y = position.y;
            car.heading = tangent.angle();
            car.vx = 5.0 * UNITS_PER_METRE;
            car.vy = 0.0;
            car.yaw_rate = 0.0;
            car.steer_angle = 0.0;
            car.steer_rate = 0.0;
            car.steer_target = 0.0;
            live.step().unwrap();
            assert_eq!(
                live.last_end[0],
                if gate == 39 {
                    Some(EndReason::Goal)
                } else {
                    None
                }
            );
            assert_eq!(live.counted_laps[0], ((gate + 1) / 8) as i32);
        }
        assert_eq!(live.current_stats().completed_stints, 1);
        assert_eq!(live.current_stats().best_streak, 5);
        assert_eq!(live.current_stats().laps, 5);
        assert!(
            live.current_stats().best_lap.unwrap() < 1.0,
            "best lap must not become total stint time"
        );
        live.step().unwrap();
        assert_eq!(live.cars[0].steps, 1);
        assert_eq!(live.cars[0].laps_completed, 0);
    }

    #[test]
    fn live_samples_update_model_and_circuit_switch_retains_learning() {
        let mut live = LiveTrainer::new(Circuit::Monza.build(), 42);
        let initial = live.policy.params().to_vec();
        for _ in 0..UPDATE_ROUNDS * 2 {
            if live.updates >= 2 {
                break;
            }
            live.step().unwrap();
        }
        assert!(live.decisions >= CAR_COUNT as u64);
        assert_eq!(live.updates, 2);
        assert_ne!(live.policy.params(), initial);
        assert!(live.episodes > 0);
        assert_eq!(live.current_stats().attempts, live.episodes);
        let trained = live.policy.params().to_vec();
        let attempts = live.episodes;
        live.select_track(Circuit::Silverstone.build());
        assert_eq!(live.policy.params(), trained);
        assert_eq!(live.episodes, attempts);
        assert_eq!(live.current_stats().attempts, 0);
        assert_eq!(live.stats[0].attempts, attempts);
        assert!(live.cars.iter().all(|c| c.steps == 0
            && c.laps == STINT_LAPS
            && c.track.circuit == Some(Circuit::Silverstone)));
        assert_eq!(live.batch.rows, 0);
        assert_eq!(live.rounds, 0);
        assert_eq!(live.track.circuit, Some(Circuit::Silverstone));
        assert_eq!(live.best_progress, 0.0);
        assert!(live.best_params.is_none());
        let fresh = LiveTrainer::new(Circuit::Silverstone.build(), 77);
        assert_ne!(fresh.policy.params(), trained);
        assert_eq!(fresh.episodes, 0);
        assert_eq!(fresh.updates, 0);
    }

    #[test]
    fn stalled_and_timed_out_cars_restart_without_ending_training() {
        let mut live = LiveTrainer::new(Circuit::Monza.build(), 7);
        live.progress_step = [-STALL_STEPS; CAR_COUNT];
        live.step().unwrap();
        assert_eq!(live.episodes, 4);
        assert!(live.last_end.iter().all(|r| *r == Some(EndReason::Stalled)));
        live.step().unwrap();
        assert!(live.cars.iter().all(|c| c.steps == 1));
        for car in &mut live.cars {
            car.steps = car.max_steps() - 1;
        }
        live.progress_step = [i32::MAX / 2; CAR_COUNT];
        live.step().unwrap();
        assert!(live.last_end.iter().all(|r| *r == Some(EndReason::Timeout)));
        live.step().unwrap();
        assert!(live.cars.iter().all(|c| c.steps == 1));
    }

    #[test]
    fn a_collapsed_mean_policy_restores_the_best_snapshot() {
        let mut live = LiveTrainer::new(Track::oval(), 5);
        let good = live.policy.params().to_vec();
        live.best_progress = 0.2;
        live.best_params = Some(good.clone());
        for layer in live.policy.params_mut() {
            layer.fill(0.0);
        }
        // Hard braking: the mean policy stands still, which is the collapse the
        // audit must catch (a zero action merely coasts from the start speed).
        if let Some(bias) = live.policy.params_mut().last_mut() {
            bias[1] = -5.0;
        }
        let (progress, stopped) = live.audit_progress();
        assert!(progress < 0.2 * SNAPSHOT_TRIGGER);
        assert!(stopped, "a parked mean policy is detected as a stall");
        live.audit_policy();
        assert_eq!(live.restores, 1);
        assert_eq!(live.policy.params(), good.as_slice());
        assert_eq!(live.best_progress, 0.2, "a restore is not a new best");
    }

    #[test]
    fn round_boundary_bootstraps_a_restarted_car_instead_of_truncating_it() {
        let mut live = LiveTrainer::new(Track::oval(), 11);
        // A car that finished an earlier attempt keeps `completed` set until the
        // round boundary, but its new attempt is already under way.
        live.completed[0] = true;
        let obs = live.cars[0].obs;
        let sample = live.policy.sample(&obs, &mut live.rngs[0]);
        live.trajectories[0].push(&obs, &sample, 1.0);
        let bootstrap = live.policy.value(&obs);
        live.push_round_trajectories();
        assert_eq!(live.batch.rows, 1);
        let expected = 1.0 + neural_racing_ml::GAMMA as f32 * bootstrap - sample.value;
        assert!(
            (live.batch.advantages[0] - expected).abs() < 1e-5,
            "mid-attempt row must bootstrap {bootstrap}, got advantage {} vs {expected}",
            live.batch.advantages[0]
        );
    }

    #[test]
    fn speed_changes_scheduling_not_physics_or_learning() {
        let track = Circuit::RedBullRing.build();
        let mut slow = LiveTrainer::new(track.clone(), 9);
        let mut fast = LiveTrainer::new(track, 9);
        for _ in 0..256 {
            slow.step().unwrap();
            fast.step().unwrap();
        }
        assert_eq!(slow.policy.params(), fast.policy.params());
        for (a, b) in slow.cars.iter().zip(&fast.cars) {
            assert_eq!(a.position(), b.position());
            assert_eq!(a.steps, b.steps);
        }
        for speed in [0.25, 1.0, 16.0, 1000.0, MAX_REQUESTED_SPEED] {
            let mut pacer = Pacer::default();
            let mut ticks = 0;
            for _ in 0..1000 {
                pacer.accrue(0.001, speed);
                while pacer.take() {
                    ticks += 1;
                }
            }
            assert!(
                (ticks as f64 - speed / DECISION_DT).abs() <= 1.0,
                "{speed}: {ticks}"
            );
            pacer.accrue(100.0, speed);
            pacer.reset();
            assert!(!pacer.take());
        }
    }
}
