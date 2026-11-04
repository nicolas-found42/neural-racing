//! Parallel rollouts, the generation loop, and evaluation on fresh tracks.

use std::time::Instant;

use neural_racing_core::{
    Action, Env, Rng, Track, CHECKPOINT_COUNT, DEFAULT_LAPS, MAX_STEPS, OBS_DIM,
};
use neural_racing_ml::{
    update, Adam, Batch, Policy, Sample, Trajectory, UpdateStats, DEFAULT_BLOCKS, DEFAULT_WIDTH,
    LR, MAX_GRAD_NORM,
};

#[derive(Clone, Debug)]
pub struct RaceConfig {
    pub laps: i32,
    pub workers: usize,
    pub episodes_per_worker: usize,
    pub seed: u64,
    /// First track seed; each episode draws its own.
    pub track_seed_base: u64,
}

impl RaceConfig {
    pub fn new(seed: u64, workers: usize) -> Self {
        Self {
            laps: DEFAULT_LAPS,
            workers: workers.max(1),
            episodes_per_worker: 2,
            seed,
            track_seed_base: seed.wrapping_mul(1_000_003).wrapping_add(7),
        }
    }

    pub fn track_seed(&self, generation: i64, worker: usize, episode: usize) -> u64 {
        self.track_seed_base
            .wrapping_add((generation as u64).wrapping_mul(1_000_000))
            .wrapping_add((worker as u64) * 97)
            .wrapping_add(episode as u64 * 7_919)
    }
}

#[derive(Clone, Debug)]
pub struct EpisodeReport {
    pub track_seed: u64,
    pub reward: f64,
    pub distance: f64,
    pub checkpoints: i32,
    pub laps_completed: i32,
    pub lap_done: bool,
    pub off_track: bool,
    pub steps: i32,
    pub mean_speed: f64,
    pub min_wall: f64,
    pub lap_steps: Vec<i32>,
}

impl EpisodeReport {
    pub fn finished(&self) -> bool {
        self.lap_done && !self.off_track
    }
}

pub struct Rollout {
    pub report: EpisodeReport,
    pub batch: Batch,
}

fn decide(policy: &Policy, obs: &[f32], rng: &mut Rng, explore: bool) -> Sample {
    if explore {
        policy.sample(obs, rng)
    } else {
        let action = policy.mean_action(obs);
        Sample {
            action,
            z: [0.0; 2],
            mean: [0.0; 2],
            log_std: [0.0; 2],
            eps: [0.0; 2],
            logp: 0.0,
            value: policy.value(obs),
        }
    }
}

/// Runs one race on `track`, sampling from the policy and recording the rows
/// the estimator consumes. Finite differences are not involved anywhere.
pub fn rollout(policy: &Policy, track: Track, laps: i32, rng: &mut Rng) -> Rollout {
    rollout_inner(policy, track, 0, laps, rng, true)
}

fn rollout_inner(
    policy: &Policy,
    track: Track,
    track_seed: u64,
    laps: i32,
    rng: &mut Rng,
    explore: bool,
) -> Rollout {
    let mut env = Env::new(track, laps);
    let mut trajectory = Trajectory::default();
    let mut rewards: Vec<f32> = Vec::with_capacity(MAX_STEPS as usize);
    let mut min_wall = f64::INFINITY;
    let mut speed_sum = 0.0;
    let mut steps = 0i32;
    let mut lap_steps = Vec::new();
    let mut last_laps = 0;
    let mut last_lap_step = 0;
    let mut info;

    loop {
        let sample = decide(policy, &env.obs, rng, explore);
        trajectory.push(&env.obs, &sample, 0.0);
        let outcome = env.step(Action {
            steer: f64::from(sample.action[0]),
            throttle: f64::from(sample.action[1]),
        });
        if let Some(slot) = trajectory.rewards.last_mut() {
            *slot = outcome.reward as f32;
        }
        rewards.push(outcome.reward as f32);
        min_wall = min_wall.min(env.min_ray);
        speed_sum += env.speed;
        steps += 1;
        info = outcome.info;
        if info.laps_completed > last_laps {
            lap_steps.push(info.steps - last_lap_step);
            last_laps = info.laps_completed;
            last_lap_step = info.steps;
        }
        if outcome.done {
            break;
        }
    }

    let mut batch = Batch::new(policy.obs_dim, policy.act_dim);
    batch.push_trajectory(&trajectory, true);

    Rollout {
        report: EpisodeReport {
            track_seed,
            reward: f64::from(rewards.iter().sum::<f32>()),
            distance: info.distance,
            checkpoints: info.checkpoints,
            laps_completed: info.laps_completed,
            lap_done: info.lap_done,
            off_track: info.off_track,
            steps: info.steps,
            mean_speed: speed_sum / f64::from(steps.max(1)),
            min_wall,
            lap_steps,
        },
        batch,
    }
}

/// One deterministic race using the mean action; used for evaluation and the UI.
pub fn deterministic_episode(policy: &Policy, track: Track, laps: i32) -> EpisodeReport {
    let mut rng = Rng::new(0);
    rollout_inner(policy, track, 0, laps, &mut rng, false).report
}

#[derive(Clone, Debug)]
pub struct EvalRow {
    pub seed: u64,
    pub finished: bool,
    pub laps_completed: i32,
    pub off_track: bool,
    pub steps: i32,
    pub distance: f64,
    pub mean_speed: f64,
    pub min_wall: f64,
}

#[derive(Clone, Debug)]
pub struct EvalReport {
    pub rows: Vec<EvalRow>,
    pub finished: usize,
    pub crashed: usize,
    pub timed_out: usize,
    pub mean_distance: f64,
    pub mean_speed: f64,
    pub min_wall: f64,
}

impl EvalReport {
    pub fn success_rate(&self) -> f64 {
        if self.rows.is_empty() {
            0.0
        } else {
            self.finished as f64 / self.rows.len() as f64
        }
    }
}

/// Races the deterministic policy over `count` unseen random tracks.
pub fn evaluate(policy: &Policy, base_seed: u64, count: usize, laps: i32) -> EvalReport {
    let mut rows = Vec::with_capacity(count);
    for index in 0..count {
        let seed = base_seed.wrapping_add(index as u64);
        let report = deterministic_episode(policy, Track::random(seed), laps);
        rows.push(EvalRow {
            seed,
            finished: report.finished(),
            laps_completed: report.laps_completed,
            off_track: report.off_track,
            steps: report.steps,
            distance: report.distance,
            mean_speed: report.mean_speed,
            min_wall: report.min_wall,
        });
    }
    let count = rows.len() as f64;
    let report = EvalReport {
        finished: rows.iter().filter(|row| row.finished).count(),
        crashed: rows.iter().filter(|row| row.off_track).count(),
        timed_out: rows
            .iter()
            .filter(|row| !row.finished && !row.off_track)
            .count(),
        mean_distance: rows.iter().map(|row| row.distance).sum::<f64>() / count,
        mean_speed: rows.iter().map(|row| row.mean_speed).sum::<f64>() / count,
        min_wall: rows
            .iter()
            .map(|row| row.min_wall)
            .fold(f64::INFINITY, f64::min),
        rows,
    };
    report
}

#[derive(Clone, Debug)]
pub struct GenerationStats {
    pub generation: i64,
    pub episodes: usize,
    pub finished: usize,
    pub crashes: usize,
    pub timeouts: usize,
    pub mean_checkpoints: f64,
    pub best_distance: f64,
    pub mean_reward: f64,
    pub mean_speed: f64,
    pub entropy: f64,
    pub update: UpdateStats,
    pub rows: usize,
    pub elapsed_seconds: f64,
}

pub struct Trainer {
    pub config: RaceConfig,
    pub policy: Policy,
    pub adam: Adam,
    pub generation: i64,
    pub episodes_seen: i64,
}

impl Trainer {
    pub fn new(config: RaceConfig) -> Self {
        let policy = Policy::new(OBS_DIM, DEFAULT_WIDTH, DEFAULT_BLOCKS, config.seed);
        let adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
        Self {
            config,
            policy,
            adam,
            generation: 0,
            episodes_seen: 0,
        }
    }

    pub fn with_policy(config: RaceConfig, policy: Policy) -> Self {
        let adam = Adam::new(policy.params(), LR, (0.9, 0.999), 1e-8, MAX_GRAD_NORM);
        Self {
            config,
            policy,
            adam,
            generation: 0,
            episodes_seen: 0,
        }
    }

    /// Collects `workers * episodes_per_worker` episodes in parallel, then
    /// applies one policy-gradient update.
    pub fn train_generation(&mut self) -> GenerationStats {
        let started = Instant::now();
        let generation = self.generation + 1;

        let policy = &self.policy;
        let config = &self.config;

        let results: Vec<Vec<Rollout>> = std::thread::scope(|scope| {
            let handles: Vec<_> = (0..config.workers)
                .map(|worker| {
                    let policy = &*policy;
                    let config = config;
                    scope.spawn(move || {
                        let mut rollouts = Vec::with_capacity(config.episodes_per_worker);
                        for episode in 0..config.episodes_per_worker {
                            let track_seed = config.track_seed(generation, worker, episode);
                            let mut rng = Rng::new(
                                config
                                    .seed
                                    .wrapping_add((worker as u64) << 32)
                                    .wrapping_add((episode as u64) << 16)
                                    .wrapping_add(generation as u64),
                            );
                            // Every third episode comes from the hard tail of the
                            // distribution (narrow road, sharp corners), where
                            // the policy is weakest; the rest are ordinary tracks.
                            let track = match (generation as usize + worker + episode) % 3 {
                                0 => Track::random_hard(track_seed),
                                1 => Track::random_narrow(track_seed),
                                _ => Track::random(track_seed),
                            };
                            rollouts.push(rollout_inner(
                                policy,
                                track,
                                track_seed,
                                config.laps,
                                &mut rng,
                                true,
                            ));
                        }
                        rollouts
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|handle| handle.join().expect("rollout worker panicked"))
                .collect()
        });

        let mut batch = Batch::new(OBS_DIM, 2);
        let mut reports = Vec::with_capacity(config.workers * config.episodes_per_worker);
        for mut rollouts in results {
            for item in rollouts.drain(..) {
                for row in 0..item.batch.rows {
                    let width = item.batch.obs_dim;
                    let dim = item.batch.act_dim;
                    batch.push_row(
                        &item.batch.obs[row * width..(row + 1) * width],
                        &item.batch.z[row * dim..(row + 1) * dim],
                        item.batch.old_logp[row],
                        item.batch.values[row],
                        item.batch.returns[row],
                        item.batch.advantages[row],
                    );
                }
                reports.push(item.report);
            }
        }

        let rows = batch.rows;
        let stats = update(&mut self.policy, &mut self.adam, &batch)
            .unwrap_or_else(|error| panic!("generation {generation} update failed: {error}"));

        self.generation = generation;
        self.episodes_seen += reports.len() as i64;

        let episodes = reports.len() as f64;
        let mean_checkpoints = reports
            .iter()
            .map(|r| f64::from(r.checkpoints))
            .sum::<f64>()
            / episodes;
        let mean_speed = reports.iter().map(|r| r.mean_speed).sum::<f64>() / episodes;
        let result = GenerationStats {
            generation,
            episodes: reports.len(),
            finished: reports.iter().filter(|r| r.finished()).count(),
            crashes: reports.iter().filter(|r| r.off_track).count(),
            timeouts: reports
                .iter()
                .filter(|r| !r.finished() && !r.off_track)
                .count(),
            mean_checkpoints,
            best_distance: reports.iter().map(|r| r.distance).fold(0.0, f64::max),
            mean_reward: reports.iter().map(|r| r.reward).sum::<f64>() / episodes,
            mean_speed,
            entropy: stats.entropy,
            update: stats,
            rows,
            elapsed_seconds: started.elapsed().as_secs_f64(),
        };
        result
    }
}

/// Score used to keep the best checkpoint: finishes first, then progress.
pub fn score(report: &EvalReport) -> (usize, f64, f64) {
    (report.finished, report.mean_distance, report.mean_speed)
}

/// Number of gates in one lap, exposed for UI progress bars.
pub fn gates_per_lap() -> i32 {
    CHECKPOINT_COUNT as i32
}

/// Incremental, in-memory learning used by the live desktop session.
pub mod live;
