//! Bounded live worker: four cars, one shared model, latest-state publication.
//! Rendering never waits for physics or an optimizer update.
use std::collections::VecDeque;
use std::sync::{mpsc, Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use neural_racing_core::{Env, Track, Vec2, DECISION_DT};

use neural_racing_engine::live::{
    normalize_speed, CircuitStats, EndReason, LiveTrainer, Pacer, CAR_COUNT,
};
use neural_racing_ml::{Policy, UpdateStats};

pub struct Snapshot {
    pub cars: [Env; CAR_COUNT],
    pub trails: [Vec<Vec2>; CAR_COUNT],
    pub stats: [CircuitStats; 4],
    pub policy: Arc<Policy>,
    pub ends: [Option<EndReason>; CAR_COUNT],
    pub updates: u64,
    pub episodes: u64,
    pub decisions: u64,
    pub last_update: Option<UpdateStats>,
    pub achieved_speed: f64,
    pub paused: bool,
    pub revision: u64,
    pub error: Option<String>,
}

enum Command {
    Running(bool),
    Speed(f64),
    Track(Track, u64),
    Reset(u64, u64),
    Stop,
}

pub struct Learning {
    sender: mpsc::Sender<Command>,
    latest: Arc<Mutex<Option<Snapshot>>>,
    worker: Option<JoinHandle<()>>,
    pub revision: u64,
}

impl Learning {
    pub fn start(track: Track, seed: u64, speed: f64) -> Self {
        let (sender, receiver) = mpsc::channel::<Command>();
        let latest = Arc::new(Mutex::new(None));
        let published = latest.clone();
        let worker = std::thread::Builder::new()
            .name("live-learning".into())
            .spawn(move || {
                let mut trainer = LiveTrainer::new(track, seed);
                let mut trails: [VecDeque<Vec2>; CAR_COUNT] =
                    std::array::from_fn(|_| VecDeque::new());
                let mut policy = Arc::new(trainer.policy.clone());
                let mut policy_update = 0;
                let mut running = true;
                let mut requested = normalize_speed(speed);
                let mut revision = 0;
                let mut pacer = Pacer::default();
                let mut previous = Instant::now();
                let mut last_publish = previous - Duration::from_secs(1);
                let mut rate_start = previous;
                let mut rate_decisions = 0;
                let mut achieved = 0.0;
                let mut error = None;
                loop {
                    let mut changed = false;
                    while let Ok(command) = receiver.try_recv() {
                        match command {
                            Command::Stop => {
                                return;
                            }
                            Command::Running(value) => {
                                running = value;
                                pacer.reset();
                                achieved = 0.0;
                            }
                            Command::Speed(value) => {
                                requested = normalize_speed(value);
                                pacer.reset();
                            }
                            Command::Track(track, version) => {
                                trainer.select_track(track);
                                revision = version;
                                pacer.reset();
                                for trail in &mut trails {
                                    trail.clear();
                                }
                            }
                            Command::Reset(seed, version) => {
                                trainer = LiveTrainer::new(trainer.cars[0].track.clone(), seed);
                                revision = version;
                                policy_update = u64::MAX;
                                error = None;
                                pacer.reset();
                                for trail in &mut trails {
                                    trail.clear();
                                }
                            }
                        }
                        changed = true;
                    }
                    if changed {
                        previous = Instant::now();
                        rate_start = previous;
                        rate_decisions = trainer.decisions;
                        achieved = 0.0;
                    }
                    let now = Instant::now();
                    if running && error.is_none() && requested.is_finite() {
                        pacer.accrue((now - previous).as_secs_f64(), requested);
                    }
                    previous = now;
                    // Check controls and publish between short work slices. An update
                    // has a fixed sample ceiling; no unbounded rollout blocks the worker.
                    let slice = Instant::now();
                    while running && error.is_none() && (requested.is_infinite() || pacer.take()) {
                        if let Err(message) = trainer.step() {
                            error = Some(message.to_string());
                        }
                        for (car, trail) in trainer.cars.iter().zip(&mut trails) {
                            if car.steps == 1 {
                                trail.clear();
                            }
                            for &position in &car.path {
                                if trail
                                    .back()
                                    .is_none_or(|last| (*last - position).norm() > 0.5)
                                {
                                    trail.push_back(position);
                                    if trail.len() > 720 {
                                        trail.pop_front();
                                    }
                                }
                            }
                        }
                        if slice.elapsed() >= Duration::from_millis(3) {
                            break;
                        }
                    }
                    if rate_start.elapsed() >= Duration::from_millis(500) {
                        achieved = (trainer.decisions - rate_decisions) as f64 * DECISION_DT
                            / CAR_COUNT as f64
                            / rate_start.elapsed().as_secs_f64();
                        rate_start = Instant::now();
                        rate_decisions = trainer.decisions;
                    }
                    if changed || last_publish.elapsed() >= Duration::from_millis(16) {
                        if policy_update != trainer.updates {
                            policy = Arc::new(trainer.policy.clone());
                            policy_update = trainer.updates;
                        }
                        let snapshot = Snapshot {
                            cars: trainer.cars.clone(),
                            trails: std::array::from_fn(|i| trails[i].iter().copied().collect()),
                            stats: trainer.stats.clone(),
                            policy: policy.clone(),
                            ends: trainer.last_end,
                            updates: trainer.updates,
                            episodes: trainer.episodes,
                            decisions: trainer.decisions,
                            last_update: trainer.last_update,
                            achieved_speed: achieved,
                            paused: !running,
                            revision,
                            error: error.clone(),
                        };
                        // Replace an old frame, never queue stale frames behind the UI.
                        *published.lock().unwrap() = Some(snapshot);
                        last_publish = Instant::now();
                    }
                    if running && error.is_none() && (requested.is_infinite() || pacer.ready()) {
                        // No mandatory idle time while there is useful work.
                        // The short slices still check controls and publish frames.
                        std::thread::yield_now();
                    } else {
                        std::thread::sleep(Duration::from_millis(if running { 1 } else { 5 }));
                    }
                }
            })
            .expect("start live learning thread");
        Self {
            sender,
            latest,
            worker: Some(worker),
            revision: 0,
        }
    }

    pub fn take(&self) -> Option<Snapshot> {
        self.latest.lock().unwrap().take()
    }
    pub fn running(&self, value: bool) {
        let _ = self.sender.send(Command::Running(value));
    }
    pub fn speed(&self, value: f64) {
        let _ = self.sender.send(Command::Speed(value));
    }
    pub fn track(&mut self, track: Track) {
        self.revision += 1;
        let _ = self.sender.send(Command::Track(track, self.revision));
    }
    pub fn reset(&mut self, seed: u64) {
        self.revision += 1;
        let _ = self.sender.send(Command::Reset(seed, self.revision));
    }
}
impl Drop for Learning {
    fn drop(&mut self) {
        let _ = self.sender.send(Command::Stop);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use neural_racing_core::Circuit;

    fn wait_for(worker: &Learning, predicate: impl Fn(&Snapshot) -> bool) -> Snapshot {
        let start = Instant::now();
        loop {
            if let Some(snapshot) = worker.take() {
                if predicate(&snapshot) {
                    return snapshot;
                }
            }
            assert!(
                start.elapsed() < Duration::from_secs(10),
                "live worker did not publish expected state"
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    #[test]
    fn pause_resume_speed_switch_and_reset_operate_on_one_live_session() {
        let mut worker = Learning::start(Circuit::Monza.build(), 42, 1.0);
        let started = wait_for(&worker, |s| s.decisions >= 4);
        assert!(started
            .cars
            .iter()
            .all(|c| c.track.circuit == Some(Circuit::Monza)));
        worker.running(false);
        let paused = wait_for(&worker, |s| s.paused);
        std::thread::sleep(Duration::from_millis(100));
        let still = wait_for(&worker, |s| s.paused);
        assert_eq!(still.decisions, paused.decisions);
        assert_eq!(still.policy.params(), paused.policy.params());
        assert_eq!(still.cars[0].position(), paused.cars[0].position());
        worker.speed(f64::INFINITY);
        worker.running(true);
        let trained = wait_for(&worker, |s| s.updates >= 1);
        assert!(trained.decisions > paused.decisions);
        assert_ne!(trained.policy.params(), paused.policy.params());
        worker.running(false);
        let before_switch = wait_for(&worker, |s| s.paused);
        worker.track(Circuit::Silverstone.build());
        let switched = wait_for(&worker, |s| s.revision == 1);
        assert_eq!(switched.policy.params(), before_switch.policy.params());
        assert_eq!(switched.episodes, before_switch.episodes);
        assert!(switched
            .cars
            .iter()
            .all(|c| c.steps == 0 && c.track.circuit == Some(Circuit::Silverstone)));
        worker.reset(1234);
        let fresh = wait_for(&worker, |s| s.revision == 2);
        assert_eq!(fresh.updates, 0);
        assert_eq!(fresh.episodes, 0);
        assert_eq!(fresh.decisions, 0);
        assert_ne!(fresh.policy.params(), switched.policy.params());
        assert!(fresh
            .stats
            .iter()
            .all(|s| s.attempts == 0 && s.history.is_empty()));
        worker.running(true);
        let resumed = wait_for(&worker, |s| s.revision == 2 && s.decisions > 0);
        assert!(!resumed.paused);
    }
}
