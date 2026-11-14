//! Four actual training cars, advanced in fixed physics decisions. No display,
//! scheduling threads: scheduling belongs to the caller; diagnostics are best effort.
use crate::curriculum::{section_exit, Curriculum, Lesson};
use neural_racing_diagnostics::{emit, json};
use rayon::prelude::*;
use std::collections::VecDeque;
use std::time::Instant;

use neural_racing_core::{Action, Circuit, Env, Rng, Track, DECISION_DT, OBS_DIM, UNITS_PER_METRE};
use neural_racing_ml::{
    update, Adam, Batch, InferenceWorkspace, Policy, Sample, Trajectory, UpdateStats,
    DEFAULT_BLOCKS, DEFAULT_WIDTH, GAMMA, LR,
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
/// Audit ceiling covers a beginner-paced real lap; the environment also bounds it.
const SNAPSHOT_STEPS: usize = 18000;
/// One on-policy draw held for 200 ms, including all intermediate rewards.
pub use neural_racing_ml::CONTROL_HOLD;
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
    Section,
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
    pub completed_routes: u64,
    pub best_route: Option<f64>,
    pub section_attempts: u64,
    pub section_successes: u64,
    pub best_streak: i32,
    pub best_lap: Option<f64>,
    pub history: VecDeque<Attempt>,
}

struct PendingControl {
    obs: [f32; OBS_DIM],
    sample: Sample,
    reward: f64,
    steps: u32,
}
impl PendingControl {
    fn finish(self, trajectory: &mut Trajectory) {
        trajectory.push_held(&self.obs, &self.sample, self.reward as f32, self.steps);
    }
}

pub struct LiveTrainer {
    pub diagnostic_id: u64,
    diagnostic_epoch: u64,
    diagnostic_counts: [u64; 5],
    diagnostic_rollbacks: u64,
    diagnostic_last_progress: Instant,
    diagnostic_history: [VecDeque<[f64; 6]>; CAR_COUNT],
    pub policy: Policy,
    pub cars: [Env; CAR_COUNT],
    pub stats: Vec<CircuitStats>,
    pub updates: u64,
    pub decisions: u64,
    /// Applied pedal bins [braking, coasting, throttle], per environment decision.
    pub pedal_counts: [u64; 3],
    pub saturated_controls: u64,
    pub episodes: u64,
    pub last_update: Option<UpdateStats>,
    pub last_end: [Option<EndReason>; CAR_COUNT],
    /// Number of times a collapsed mean policy was replaced by the best snapshot.
    pub restores: u64,
    pub curriculum: Option<Curriculum>,
    lessons: [Option<Lesson>; CAR_COUNT],
    curriculum_rng: Rng,
    pending: [Option<PendingControl>; CAR_COUNT],
    track: Track,
    best_progress: f64,
    best_params: Option<Vec<Vec<f32>>>,
    optimizer: Adam,
    inference: InferenceWorkspace,
    simulation_pool: Option<rayon::ThreadPool>,
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
        Self::with_curriculum(track, seed, true)
    }

    pub fn with_curriculum(track: Track, seed: u64, enabled: bool) -> Self {
        let mut curriculum = enabled.then(|| Curriculum::new(&track));
        let mut curriculum_rng = Rng::new(seed ^ 0xc077_3a5e);
        let mut cars = Self::practice_cars(&track);
        let lessons = std::array::from_fn(|i| {
            curriculum
                .as_mut()
                .and_then(|c| c.start(&mut cars[i], &mut curriculum_rng))
        });
        let policy = Policy::new(OBS_DIM, DEFAULT_WIDTH, DEFAULT_BLOCKS, seed);
        let optimizer = Adam::new(
            policy.params(),
            LR,
            (0.9, 0.999),
            1e-8,
            crate::MAX_GRAD_NORM,
        );
        let diagnostic_id = neural_racing_diagnostics::operation();
        emit!(
            "live_started",
            diagnostic_id,
            json!({"parent":neural_racing_diagnostics::current_operation(),
            "seed":seed,"track":crate::diagnostics::track(&track),"cars":CAR_COUNT,"laps":STINT_LAPS,
            "decision_dt":DECISION_DT,"min_update_rows":MIN_UPDATE_ROWS,"max_update_rounds":UPDATE_ROUNDS,
            "lr":LR,"obs_dim":OBS_DIM,"parameters":policy.num_params(),
            "curriculum":enabled,"control_hold":CONTROL_HOLD,"gae_lambda":neural_racing_ml::GAE_LAMBDA})
        );
        Self {
            diagnostic_id,
            diagnostic_epoch: 0,
            diagnostic_counts: [0; 5],
            diagnostic_rollbacks: 0,
            diagnostic_last_progress: Instant::now(),
            diagnostic_history: std::array::from_fn(|_| VecDeque::with_capacity(8)),
            policy,
            optimizer,
            inference: InferenceWorkspace::default(),
            simulation_pool: None,
            cars,
            curriculum,
            curriculum_rng,
            lessons,
            pending: std::array::from_fn(|_| None),
            track: track.clone(),
            best_progress: 0.0,
            best_params: None,
            restores: 0,
            stats: vec![CircuitStats::default(); Circuit::stats_count()],
            rngs: std::array::from_fn(|i| Rng::new(seed.wrapping_add(i as u64 * 7919))),
            trajectories: std::array::from_fn(|_| Trajectory::default()),
            batch: Batch::new(OBS_DIM, 2),
            rounds: 0,
            completed: [false; CAR_COUNT],
            updates: 0,
            decisions: 0,
            pedal_counts: [0; 3],
            saturated_controls: 0,
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
        circuit.map_or(Circuit::stats_count() - 1, Circuit::stats_index)
    }
    pub fn current_stats(&self) -> &CircuitStats {
        &self.stats[Self::circuit_index(self.cars[0].track.circuit)]
    }

    /// Changes execution only: weights, optimizer, RNGs and trajectories stay put.
    /// The fixed four-car learner never creates more workers than independent cars.
    pub fn set_simulation_workers(
        &mut self,
        workers: usize,
    ) -> Result<(), rayon::ThreadPoolBuildError> {
        self.simulation_pool = if workers <= 1 {
            None
        } else {
            Some(
                rayon::ThreadPoolBuilder::new()
                    .num_threads(workers.min(CAR_COUNT))
                    .thread_name(|i| format!("racing-physics-{i}"))
                    .build()?,
            )
        };
        emit!(
            "live_execution_changed",
            self.diagnostic_id,
            json!({"simulation_workers":self.simulation_workers(),"updates":self.updates,
                "decisions":self.decisions,"inference_batch_rows":CAR_COUNT})
        );
        Ok(())
    }

    pub fn simulation_workers(&self) -> usize {
        self.simulation_pool
            .as_ref()
            .map_or(1, |pool| pool.current_num_threads())
    }
    /// Keep the model and Adam state, but discard unfinished on-policy samples.
    pub fn select_track(&mut self, track: Track) {
        self.diagnostic_epoch += 1;
        emit!(
            "live_track_changed",
            self.diagnostic_id,
            json!({"epoch":self.diagnostic_epoch,
            "track":crate::diagnostics::track(&track),"updates":self.updates,"decisions":self.decisions,
            "discarded_rows":self.batch.rows + self.trajectories.iter().map(|t|t.rows).sum::<usize>()})
        );
        for history in &mut self.diagnostic_history {
            history.clear();
        }
        self.track = track.clone();
        self.best_progress = 0.0;
        self.best_params = None;
        self.cars = Self::practice_cars(&track);
        if self.curriculum.is_some() {
            self.curriculum = Some(Curriculum::new(&track));
        }
        self.reset_practice();
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
        let _context = neural_racing_diagnostics::context(self.diagnostic_id);
        let result = self.step_inner();
        if let Err(reason) = result {
            emit!(
                "live_error",
                self.diagnostic_id,
                json!({"reason":reason,"epoch":self.diagnostic_epoch,
                "updates":self.updates,"decisions":self.decisions,"batch_rows":self.batch.rows,
                "cars":self.cars.iter().map(crate::diagnostics::state).collect::<Vec<_>>(),
                "recent_decisions":self.diagnostic_history})
            );
        }
        if self.diagnostic_last_progress.elapsed().as_secs() >= 10 {
            self.diagnostic_progress();
            self.diagnostic_last_progress = Instant::now();
        }
        result
    }

    pub fn diagnostic_progress(&self) {
        emit!(
            "live_progress",
            self.diagnostic_id,
            json!({"epoch":self.diagnostic_epoch,"updates":self.updates,
            "decisions":self.decisions,"attempts":self.episodes,"end_counts":self.diagnostic_counts,
            "pedal_counts":self.pedal_counts,"saturated_controls":self.saturated_controls,
            "rollbacks":self.diagnostic_rollbacks,"restores":self.restores,
            "curriculum_levels":self.curriculum.as_ref().map(Curriculum::levels),
            "section_attempts":self.current_stats().section_attempts,
            "section_successes":self.current_stats().section_successes,
            "cars":self.cars.iter().map(crate::diagnostics::state).collect::<Vec<_>>()})
        );
    }

    fn step_inner(&mut self) -> Result<(), &'static str> {
        let mut observations = [0.0; CAR_COUNT * OBS_DIM];
        for i in 0..CAR_COUNT {
            if self.last_end[i].take().is_some() {
                self.pending[i] = None;
                self.lessons[i] = if let Some(c) = &mut self.curriculum {
                    c.start(&mut self.cars[i], &mut self.curriculum_rng)
                } else {
                    self.cars[i].reset();
                    None
                };
                self.diagnostic_history[i].clear();
                self.progress_mark[i] = 0.0;
                self.progress_step[i] = 0;
                self.lap_steps[i] = 0;
                self.counted_laps[i] = 0;
            }
            // Reset before gathering the proposal's observations.
            observations[i * OBS_DIM..(i + 1) * OBS_DIM].copy_from_slice(&self.cars[i].obs);
        }
        let outputs = self
            .policy
            .forward_into(&observations, CAR_COUNT, &mut self.inference);
        if !outputs.iter().all(|v| v.is_finite()) {
            return Err("non-finite inference");
        }
        for i in 0..CAR_COUNT {
            if self.pending[i].is_none() {
                let width = self.policy.outputs_per_row();
                let sample = Policy::sample_from_outputs(
                    &outputs[i * width..(i + 1) * width],
                    &mut self.rngs[i],
                );
                if !sample.action.iter().all(|v| v.is_finite()) || !sample.value.is_finite() {
                    return Err("non-finite inference");
                }
                self.pending[i] = Some(PendingControl {
                    obs: self.cars[i].obs,
                    sample,
                    reward: 0.0,
                    steps: 0,
                });
            }
        }
        let actions: [_; CAR_COUNT] = std::array::from_fn(|i| {
            let sample = &self.pending[i].as_ref().unwrap().sample;
            Action {
                steer: f64::from(sample.action[0]),
                throttle: f64::from(sample.action[1]),
            }
        });
        let mut outcomes = [None; CAR_COUNT];
        if let Some(pool) = &self.simulation_pool {
            let session = self.diagnostic_id;
            pool.install(|| {
                self.cars
                    .par_iter_mut()
                    .zip(&actions)
                    .zip(&mut outcomes)
                    .for_each(|((car, action), outcome)| {
                        let _context = neural_racing_diagnostics::context(session);
                        *outcome = Some(car.step(*action));
                    });
            });
        } else {
            for ((car, action), outcome) in self.cars.iter_mut().zip(&actions).zip(&mut outcomes) {
                *outcome = Some(car.step(*action));
            }
        }
        for i in 0..CAR_COUNT {
            let action = actions[i];
            self.pedal_counts[if action.throttle < -0.1 {
                0
            } else if action.throttle > 0.1 {
                2
            } else {
                1
            }] += 1;
            self.saturated_controls +=
                u64::from(action.steer.abs() > 0.98) + u64::from(action.throttle.abs() > 0.98);
            let car = &mut self.cars[i];
            let outcome = outcomes[i].unwrap();
            if !outcome.reward.is_finite() || !car.obs.iter().all(|v| v.is_finite()) {
                return Err("non-finite environment");
            }
            // Eight sparse decision samples, in simulation time, only for terminal/anomaly context.
            if neural_racing_diagnostics::enabled() && (car.steps % 20 == 1 || outcome.done) {
                let history = &mut self.diagnostic_history[i];
                if history.len() == 8 {
                    history.pop_front();
                }
                history.push_back([
                    car.steps as f64,
                    car.speed,
                    car.distance,
                    car.lateral_offset,
                    action.steer,
                    action.throttle,
                ]);
            }
            let mut reward = outcome.reward;
            if car.distance > self.progress_mark[i] + 5.0 * UNITS_PER_METRE {
                self.progress_mark[i] = car.distance;
                self.progress_step[i] = car.steps;
            }
            let stalled = car.steps - self.progress_step[i] >= STALL_STEPS;
            if stalled && !outcome.done {
                // Combined with the dense low-speed cost, this makes parking
                // worse than crashing into the wall it is avoiding, so a
                // cautious local optimum cannot win.
                reward -= 40.0;
            }
            let section_success =
                !outcome.done && self.lessons[i].is_some_and(|lesson| section_exit(car, lesson));
            let section_timeout = self.lessons[i].is_some_and(|lesson| {
                car.steps as f64 * DECISION_DT
                    > (lesson.distance / (5.0 * UNITS_PER_METRE) + 10.0).min(90.0)
            });
            if section_success {
                reward += 10.0;
            }
            if section_timeout && !outcome.done && !stalled {
                reward -= 40.0;
            }
            let pending = self.pending[i].as_mut().unwrap();
            pending.reward += GAMMA.powi(pending.steps as i32) * reward;
            pending.steps += 1;
            let ended = outcome.done || stalled || section_success || section_timeout;
            if pending.steps == CONTROL_HOLD || ended {
                self.pending[i]
                    .take()
                    .unwrap()
                    .finish(&mut self.trajectories[i]);
            }
            let stats = &mut self.stats[Self::circuit_index(car.track.circuit)];
            let credited_laps = car.credited_laps();
            if car.track.closed
                && !car.track.free_roam
                && self.lessons[i].is_none()
                && credited_laps > self.counted_laps[i]
            {
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
            if ended {
                let reason = if section_success {
                    EndReason::Section
                } else if car.lap_done {
                    EndReason::Goal
                } else if car.off_track {
                    EndReason::Crash
                } else if stalled {
                    EndReason::Stalled
                } else {
                    EndReason::Timeout
                };
                if self.lessons[i].is_some() {
                    stats.section_attempts += 1;
                    stats.section_successes += u64::from(section_success);
                }
                if let Some(c) = &mut self.curriculum {
                    if c.record(
                        self.lessons[i],
                        section_success || reason == EndReason::Goal,
                    ) {
                        emit!(
                            "curriculum_changed",
                            self.diagnostic_id,
                            json!({"epoch":self.diagnostic_epoch,"update":self.updates,
                            "levels":c.levels(),"stint_ready":c.stint_ready()})
                        );
                    }
                }
                stats.attempts += 1;
                self.episodes += 1;
                self.diagnostic_counts[reason as usize] += 1;
                if self.episodes <= 4 || self.episodes % 1000 == 0 || reason == EndReason::Goal {
                    emit!(
                        "live_attempt_ended",
                        self.diagnostic_id,
                        json!({"epoch":self.diagnostic_epoch,
                        "attempt":self.episodes,"circuit_attempt":stats.attempts,"car":i,
                        "reason":format!("{reason:?}"),"update":self.updates,"state":crate::diagnostics::state(car),
                        "recent_decisions":self.diagnostic_history[i],
                        "lesson":self.lessons[i].map(|l|json!({"corner":l.corner,"level":l.level,"distance":l.distance,"entry_speed":l.entry_speed}))})
                    );
                }
                let seconds = car.steps as f64 * DECISION_DT;
                if reason == EndReason::Goal && self.lessons[i].is_none() && !car.track.free_roam {
                    if car.track.closed && car.laps == STINT_LAPS {
                        stats.completed_stints += 1;
                    } else if !car.track.closed {
                        stats.completed_routes += 1;
                        stats.best_route =
                            Some(stats.best_route.map_or(seconds, |old| old.min(seconds)));
                    }
                }
                stats.history.push_back(Attempt {
                    number: stats.attempts,
                    progress: (car.distance
                        / self.lessons[i]
                            .map_or(car.track.lap_length * car.laps as f64, |l| l.distance))
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
                let update_started = Instant::now();
                let rows = self.batch.rows;
                self.last_update =
                    Some(update(&mut self.policy, &mut self.optimizer, &self.batch)?);
                let stats = self.last_update.unwrap();
                if stats.update_scale == 0.0 {
                    self.diagnostic_rollbacks += 1;
                }
                if self.updates < 3
                    || (self.updates + 1) % 100 == 0
                    || update_started.elapsed().as_secs() >= 1
                {
                    emit!(
                        "live_update",
                        self.diagnostic_id,
                        json!({"epoch":self.diagnostic_epoch,"update":self.updates + 1,
                        "rows":rows,"ceiling":ceiling,"kl":stats.policy_kl,"scale":stats.update_scale,
                        "gradient_norm":stats.gradient_norm,"explained_variance":stats.explained_variance,
                        "duration_ms":update_started.elapsed().as_millis() as u64,
                        "action_entropy":stats.entropy,"latent_sigma":stats.exploration_sigma})
                    );
                }
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

    fn reset_practice(&mut self) {
        self.pending = std::array::from_fn(|_| None);
        self.lessons = std::array::from_fn(|i| {
            self.curriculum
                .as_mut()
                .and_then(|c| c.start(&mut self.cars[i], &mut self.curriculum_rng))
        });
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
            if let Some(pending) = self.pending[i].take() {
                pending.finish(&mut self.trajectories[i]);
            }
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
        let mut action = [0.0; 2];
        for step in 0..SNAPSHOT_STEPS {
            if step % CONTROL_HOLD as usize == 0 {
                action = self.policy.mean_action(&env.obs);
            }
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
        emit!(
            "policy_audit",
            self.diagnostic_id,
            json!({"epoch":self.diagnostic_epoch,"update":self.updates,
            "progress":progress,"stopped":stopped,"best_progress":self.best_progress})
        );
        // Normal-start progress cannot rank corner-practice policies: restoring
        // a fast straight-line specialist here would erase newly learned turns.
        if self
            .curriculum
            .as_ref()
            .is_some_and(|c| c.mastered() < c.corner_count())
        {
            return;
        }
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
        self.reset_practice();
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
        self.diagnostic_epoch += 1;
        for history in &mut self.diagnostic_history {
            history.clear();
        }
        emit!(
            "policy_restored",
            self.diagnostic_id,
            json!({"epoch":self.diagnostic_epoch,"update":self.updates,
            "observed_progress":progress,"restored_progress":self.best_progress,"stopped":stopped,"restores":self.restores})
        );
    }
}

/// Wall-time credit is capped to 20ms per check, preventing catch-up bursts
/// after a slow update, pause, dragged window or sleep. Physics dt never changes.
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
        // Trim an idle backlog before banking this check's share: one check is
        // clamped to 20 ms, so the bank stays below its cap plus that share and
        // no pause can hand back more than one frame of catch-up per check.
        // Trimming after the addition instead would destroy the sub-decision
        // remainder at every boundary, making the delivered rate depend on
        // scheduling jitter rather than on elapsed wall time.
        let cap = (0.02 * speed / DECISION_DT).max(1.0);
        self.credit = self.credit.min(cap) + elapsed.clamp(0.0, 0.02) * speed / DECISION_DT;
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
    fn held_draws_keep_one_likelihood_and_include_partial_terminal_controls() {
        let mut live = LiveTrainer::with_curriculum(Track::oval(), 42, false);
        let mut expected_reward = 0.0;
        let obs = live.cars[0].obs;
        let sample = live.policy.sample(&obs, &mut Rng::new(42));
        for held in 0..CONTROL_HOLD {
            live.step().unwrap();
            expected_reward += GAMMA.powi(held as i32) * live.cars[0].last_reward;
            assert!((live.cars[0].last_action.throttle - sample.action[1] as f64).abs() < 1e-6);
            assert!((live.cars[0].last_action.steer - sample.action[0] as f64).abs() < 1e-6);
        }
        assert_eq!(live.trajectories[0].rows, 1);
        assert_eq!(live.trajectories[0].steps, [CONTROL_HOLD]);
        assert!((live.trajectories[0].logp[0] - sample.logp).abs() < 1e-6);
        assert!((live.trajectories[0].rewards[0] as f64 - expected_reward).abs() < 1e-6);
        live.step().unwrap();
        assert!(live.pending[0].is_some());
        // A cut after one step uses gamma^1, never gamma^4 or a fabricated terminal.
        let bootstrap = live.policy.value(&live.cars[0].obs);
        let held = live.pending[0].as_ref().unwrap();
        let expected_return = held.reward + GAMMA * bootstrap as f64;
        live.push_round_trajectories();
        assert!(live.pending.iter().all(Option::is_none));
        assert!((live.batch.returns[1] as f64 - expected_return).abs() < 1e-5);
        // A terminal in the middle of the next hold is flushed immediately.
        live.step().unwrap();
        live.cars[0].steps = live.cars[0].max_steps() - 1;
        live.progress_step[0] = live.cars[0].steps;
        live.step().unwrap();
        assert_eq!(live.last_end[0], Some(EndReason::Timeout));
        assert!(live.pending[0].is_none());
        assert_eq!(live.trajectories[0].rows, 0);
        assert!(live.batch.returns.iter().all(|r| r.is_finite()));
    }

    #[test]
    fn parallel_physics_matches_serial_through_updates_restarts_and_track_switch() {
        let mut serial = LiveTrainer::with_curriculum(Circuit::Monza.build(), 42, false);
        let mut parallel = LiveTrainer::with_curriculum(Circuit::Monza.build(), 42, false);
        parallel.set_simulation_workers(CAR_COUNT).unwrap();
        for _ in 0..UPDATE_ROUNDS * 2 {
            let previous_update = serial.updates;
            serial.step().unwrap();
            parallel.step().unwrap();
            assert_eq!(serial.updates, parallel.updates);
            assert_eq!(serial.decisions, parallel.decisions);
            assert_eq!(serial.episodes, parallel.episodes);
            assert_eq!(serial.last_end, parallel.last_end);
            for (a, b) in serial.cars.iter().zip(&parallel.cars) {
                assert_eq!(a.obs, b.obs);
                assert_eq!(a.position(), b.position());
                assert_eq!(a.last_reward, b.last_reward);
            }
            if serial.updates != previous_update {
                assert_eq!(serial.policy.params(), parallel.policy.params());
            }
            if serial.updates >= 2 {
                break;
            }
        }
        assert_eq!(serial.updates, 2);
        assert!(serial.episodes > 0);
        let weights = parallel.policy.params().to_vec();
        for live in [&mut serial, &mut parallel] {
            live.select_track(Circuit::Silverstone.build());
        }
        assert_eq!(parallel.simulation_workers(), CAR_COUNT);
        assert_eq!(parallel.policy.params(), weights);
        serial.step().unwrap();
        parallel.step().unwrap();
        for (a, b) in serial.cars.iter().zip(&parallel.cars) {
            assert_eq!(a.obs, b.obs);
        }
    }

    #[test]
    fn batched_decisions_use_reset_observations_and_each_cars_rng() {
        let mut live = LiveTrainer::with_curriculum(Circuit::Monza.build(), 42, false);
        live.last_end[1] = Some(EndReason::Crash);
        live.cars[1].obs.fill(100.0);
        let expected: [_; CAR_COUNT] = std::array::from_fn(|i| {
            let mut car = live.cars[i].clone();
            if i == 1 {
                car.reset();
            }
            let sample = live
                .policy
                .sample(&car.obs, &mut Rng::new(42 + i as u64 * 7919));
            (car.obs, sample)
        });
        live.step().unwrap();
        assert_eq!(live.updates, 0);
        assert_eq!(live.decisions, CAR_COUNT as u64);
        for (i, (observation, sample)) in expected.into_iter().enumerate() {
            assert_eq!(live.pending[i].as_ref().unwrap().obs, observation);
            assert_eq!(live.cars[i].steps, 1);
            assert!((live.cars[i].last_action.steer - sample.action[0] as f64).abs() < 1e-5);
            assert!((live.cars[i].last_action.throttle - sample.action[1] as f64).abs() < 1e-5);
            assert!((live.pending[i].as_ref().unwrap().sample.logp - sample.logp).abs() < 1e-5);
            assert!((live.pending[i].as_ref().unwrap().sample.value - sample.value).abs() < 1e-5);
        }
    }

    #[test]
    fn five_lap_goal_counts_each_lap_without_restarting_early() {
        let mut live = LiveTrainer::with_curriculum(Circuit::Monza.build(), 42, false);
        assert_eq!(live.policy.num_params(), 359_173);
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
        let mut live = LiveTrainer::with_curriculum(Circuit::Monza.build(), 42, false);
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
        let fresh = LiveTrainer::with_curriculum(Circuit::Silverstone.build(), 77, false);
        assert_ne!(fresh.policy.params(), trained);
        assert_eq!(fresh.episodes, 0);
        assert_eq!(fresh.updates, 0);
    }

    #[test]
    fn stalled_and_timed_out_cars_restart_without_ending_training() {
        let _diagnostics = (std::env::var("NEURAL_RACING_CAPTURE_TEST").as_deref()
            == Ok("endings"))
        .then(|| neural_racing_diagnostics::init("attempt_endings_test"));
        let mut live = LiveTrainer::with_curriculum(Circuit::Monza.build(), 7, false);
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
        live.diagnostic_progress();
    }

    #[test]
    fn a_collapsed_mean_policy_restores_the_best_snapshot() {
        let _diagnostics = (std::env::var("NEURAL_RACING_CAPTURE_TEST").as_deref()
            == Ok("restore"))
        .then(|| neural_racing_diagnostics::init("policy_restore_test"));
        let mut live = LiveTrainer::with_curriculum(Track::oval(), 5, false);
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
        live.step().unwrap();
        live.diagnostic_progress();
    }

    #[test]
    fn round_boundary_bootstraps_a_restarted_car_instead_of_truncating_it() {
        let mut live = LiveTrainer::with_curriculum(Track::oval(), 11, false);
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
        let mut slow = LiveTrainer::with_curriculum(track.clone(), 9, false);
        let mut fast = LiveTrainer::with_curriculum(track, 9, false);
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
    #[test]
    fn actual_open_route_goal_counts_one_route_and_no_laps_or_stints() {
        let track = Circuit::lookup("gt4-testcourse_100m").unwrap().build();
        let mut live = LiveTrainer::with_curriculum(track, 42, false);
        for gate in 0..neural_racing_core::CHECKPOINT_COUNT {
            let car = &mut live.cars[0];
            let g = car.track.gates()[gate];
            let tangent = g.tangent;
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
            if gate + 1 < neural_racing_core::CHECKPOINT_COUNT {
                assert_eq!(live.last_end[0], None);
            }
        }
        assert_eq!(live.last_end[0], Some(EndReason::Goal));
        assert_eq!(live.current_stats().completed_routes, 1);
        assert_eq!(live.current_stats().laps, 0);
        assert_eq!(live.current_stats().completed_stints, 0);
        assert_eq!(live.current_stats().best_streak, 0);
        assert!(live.current_stats().best_lap.is_none());
        assert!(live.current_stats().best_route.is_some());
        assert_eq!(live.cars[0].credited_laps(), 0);
        live.step().unwrap();
        assert_eq!(live.current_stats().completed_routes, 1);
    }

    #[test]
    fn a_successful_open_route_section_is_not_a_completed_route() {
        let track = Circuit::lookup("gt4-testcourse_100m").unwrap().build();
        let mut live = LiveTrainer::with_curriculum(track, 42, false);
        live.cars[0].reset_for_section(0.0, 5.0 * UNITS_PER_METRE);
        live.lessons[0] = Some(Lesson {
            corner: 0,
            level: 0,
            start_arc: 0.0,
            distance: 0.05 * UNITS_PER_METRE,
            entry_speed: 5.0 * UNITS_PER_METRE,
        });
        live.step().unwrap();
        assert_eq!(live.last_end[0], Some(EndReason::Section));
        assert_eq!(live.current_stats().section_successes, 1);
        assert_eq!(live.current_stats().completed_routes, 0);
        assert_eq!(live.current_stats().completed_stints, 0);
        assert_eq!(live.current_stats().laps, 0);
    }

    #[test]
    fn irregular_check_cadence_delivers_the_requested_rate_without_banking_catch_up() {
        // A live worker alternates one-millisecond sleeps with short work
        // slices, so consecutive checks are never evenly spaced.
        for speed in [0.25, 1.0] {
            let mut pacer = Pacer::default();
            let mut ticks = 0u64;
            let mut wall = 0.0;
            for i in 0..50_000 {
                let elapsed = if i % 7 == 6 { 0.0032 } else { 0.0011 };
                wall += elapsed;
                pacer.accrue(elapsed, speed);
                while pacer.take() {
                    ticks += 1;
                }
            }
            let wanted = speed / DECISION_DT * wall;
            // Only the last, still unspent sub-decision may remain; scheduling
            // jitter must never destroy already banked credit.
            assert!(
                ticks as f64 <= wanted + 1.0,
                "{speed}x banked a catch-up burst: {ticks} > {wanted:.1}"
            );
            assert!(
                wanted - (ticks as f64) < 1.0,
                "{speed}x lost banked credit: {ticks} decisions for {wanted:.1} of elapsed wall time"
            );
        }
        // A long stall is still clamped to one 20 ms frame of simulated time.
        let mut pacer = Pacer::default();
        pacer.accrue(0.019, 1.0);
        assert!((pacer.credit - 0.38).abs() < 1e-9);
        pacer.accrue(5.0, 1.0);
        assert!((pacer.credit - 0.78).abs() < 1e-9);
        assert!(!pacer.take(), "a stall must not bank a catch-up burst");
        let mut banked = Pacer::default();
        for _ in 0..3 {
            banked.accrue(0.019, 1.0);
        }
        banked.accrue(5.0, 1.0);
        assert!(banked.take());
        assert!(!banked.take(), "a stall must not bank a second decision");
    }
}
