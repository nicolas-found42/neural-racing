//! The application: modes, controls, dashboard and live training.

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Arc;

use eframe::App as EframeApp;
use egui::{Align2, Context, FontId, Key, RichText, Sense, Stroke, Ui, Vec2};

use neural_racing_core::{
    Action, Rng, CHECKPOINT_COUNT, DECISION_DT, DEFAULT_LAPS, MAX_SPEED, MAX_STEER_ANGLE,
    RAY_RANGE,
};
use neural_racing_engine::{GenerationStats, RaceConfig, Trainer};
use neural_racing_ml::{Policy, DEFAULT_HIDDEN_DIMS};

use crate::palette as colors;
use crate::render::{self, View};
use crate::sim::{Outcome, RaceView};
use crate::widgets;

const DECISIONS_PER_SECOND: f32 = 1.0 / DECISION_DT as f32;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Watch,
    Manual,
    Train,
}

enum TrainMsg {
    Stats(Box<GenerationStats>),
    Snapshot(Box<Policy>, i64),
    Failed(String),
}

struct TrainingSession {
    receiver: Receiver<TrainMsg>,
    running: Arc<AtomicBool>,
    latest: Option<GenerationStats>,
    snapshot: Option<Policy>,
    snapshot_generation: i64,
    hero: RaceView,
    hero_policy: Option<Policy>,
    failure: Option<String>,
    started_at: std::time::Instant,
    episodes: i64,
    generations: i64,
}

enum UiAction {
    TogglePause,
    Restart,
    NewTrack,
    SetSpeed(f32),
    ToggleRays,
    ToggleTrail,
    SetMode(Mode),
    ToggleTraining,
    UsePolicy,
    ReloadPolicy,
}

pub struct App {
    mode: Mode,
    policy: Option<Policy>,
    policy_path: PathBuf,
    race: RaceView,
    speed: f32,
    paused: bool,
    show_rays: bool,
    show_trail: bool,
    seed_source: Rng,
    restart_timer: f32,
    accumulator: f32,
    best_lap: Option<f64>,
    finished: i32,
    crashed: i32,
    training: Option<TrainingSession>,
    status: String,
}

impl App {
    /// Paints every panel and the race canvas. eframe and the headless
    /// screenshot harness both drive this one path.
    pub fn render(&mut self, ui: &mut Ui) {
        let ctx = ui.ctx().clone();
        self.frame(&ctx, ui);
    }


    /// Dark theme and compact spacing, applied to whatever context runs the app.
    pub fn install_style(context: &egui::Context) {
        context.set_theme(egui::Theme::Dark);
        context.set_visuals(egui::Visuals::dark());
        context.all_styles_mut(|style| {
            style.spacing.item_spacing = Vec2::new(8.0, 6.0);
            style.spacing.button_padding = Vec2::new(10.0, 5.0);
        });
    }

    pub fn new(policy: Option<Policy>, policy_path: PathBuf) -> Self {
        Self {
            mode: Mode::Watch,
            policy,
            policy_path,
            race: RaceView::new(424_242, DEFAULT_LAPS),
            speed: 2.0,
            paused: false,
            show_rays: true,
            show_trail: true,
            seed_source: Rng::new(0x5eed_cafe),
            restart_timer: 0.0,
            accumulator: 0.0,
            best_lap: None,
            finished: 0,
            crashed: 0,
            training: None,
            status: String::new(),
        }
    }

    /// Starts on a caller-chosen track instead of the default seed.
    pub fn seed_with(&mut self, seed: u64) {
        self.race.reset(seed, DEFAULT_LAPS);
    }

    /// Applies launch flags before the first frame.
    pub fn configure(&mut self, seed: Option<u64>, mode: Option<Mode>, speed: Option<f32>) {
        if let Some(seed) = seed {
            self.seed_with(seed);
        }
        if let Some(speed) = speed {
            self.speed = speed;
        }
        if let Some(mode) = mode {
            self.apply(UiAction::SetMode(mode));
        }
    }

    fn next_seed(&mut self) -> u64 {
        self.seed_source.next_u64() >> 8
    }

    /// The race currently on screen, whichever mode is active.
    pub fn race(&self) -> &RaceView {
        self.active_race()
    }

    /// Latest generation the trainer has reported, if training has started.
    pub fn training_stats(&self) -> Option<&GenerationStats> {
        self.training
            .as_ref()
            .and_then(|session| session.latest.as_ref())
    }

    fn active_race(&self) -> &RaceView {
        if self.mode == Mode::Train {
            if let Some(session) = self.training.as_ref() {
                return &session.hero;
            }
        }
        &self.race
    }

    fn restart(&mut self, new_track: bool) {
        let seed = if new_track {
            self.next_seed()
        } else {
            self.race.track_seed
        };
        self.race.reset(seed, DEFAULT_LAPS);
        self.restart_timer = 0.0;
        self.accumulator = 0.0;
    }

    fn record_outcome(&mut self) {
        let Some(outcome) = self.race.outcome else {
            return;
        };
        if let Some(lap) = self.race.best_lap() {
            self.best_lap = Some(self.best_lap.map_or(lap, |best| best.min(lap)));
        }
        match outcome {
            Outcome::Finished { .. } => self.finished += 1,
            _ => self.crashed += 1,
        }
        self.status = match outcome {
            Outcome::Finished { seconds } => format!("finished two laps in {seconds:.1}s"),
            Outcome::Crashed { seconds } => format!("left the road after {seconds:.1}s"),
            Outcome::TimedOut { seconds } => format!("ran out of time after {seconds:.1}s"),
        };
    }

    fn apply(&mut self, action: UiAction) {
        match action {
            UiAction::TogglePause => self.paused = !self.paused,
            UiAction::Restart => self.restart(false),
            UiAction::NewTrack => self.restart(true),
            UiAction::SetSpeed(speed) => self.speed = speed,
            UiAction::ToggleRays => self.show_rays = !self.show_rays,
            UiAction::ToggleTrail => self.show_trail = !self.show_trail,
            UiAction::SetMode(mode) => {
                if mode == Mode::Train {
                    if self.training.is_none() {
                        self.start_training();
                    } else if let Some(session) = self.training.as_mut() {
                        session.running.store(true, Ordering::Relaxed);
                    }
                } else if let Some(session) = self.training.as_mut() {
                    session.running.store(false, Ordering::Relaxed);
                }
                self.mode = mode;
            }
            UiAction::ToggleTraining => {
                if let Some(session) = self.training.as_mut() {
                    let running = session.running.load(Ordering::Relaxed);
                    session.running.store(!running, Ordering::Relaxed);
                }
            }
            UiAction::UsePolicy => {
                let snapshot = self
                    .training
                    .as_ref()
                    .and_then(|session| session.snapshot.clone());
                if let Some(policy) = snapshot {
                    let generation = self
                        .training
                        .as_ref()
                        .map(|session| session.snapshot_generation)
                        .unwrap_or(0);
                    self.status = format!("adopted the policy from generation {generation}");
                    self.policy = Some(policy);
                    self.paused = false;
                    self.mode = Mode::Watch;
                    if let Some(session) = self.training.as_mut() {
                        session.running.store(false, Ordering::Relaxed);
                    }
                    self.restart(true);
                }
            }
            UiAction::ReloadPolicy => self.reload_policy(),
        }
    }

    fn start_training(&mut self) {
        let workers = std::thread::available_parallelism()
            .map(|cores| cores.get().saturating_sub(2).max(1))
            .unwrap_or(4);
        let mut config = RaceConfig::new(self.next_seed(), workers);
        config.laps = DEFAULT_LAPS;
        config.episodes_per_worker = 2;
        let (sender, receiver) = mpsc::channel();
        let running = Arc::new(AtomicBool::new(true));
        let flag = running.clone();
        let hero = RaceView::new(self.next_seed(), DEFAULT_LAPS);
        spawn_training(config, sender, flag);
        self.training = Some(TrainingSession {
            receiver,
            running,
            latest: None,
            snapshot: None,
            snapshot_generation: 0,
            hero,
            hero_policy: None,
            failure: None,
            started_at: std::time::Instant::now(),
            episodes: 0,
            generations: 0,
        });
    }

    fn poll_training(&mut self, dt: f32) {
        let mut messages = Vec::new();
        {
            let Some(session) = self.training.as_ref() else {
                return;
            };
            while let Ok(message) = session.receiver.try_recv() {
                messages.push(message);
            }
        }
        let mut reset_hero = false;
        if let Some(session) = self.training.as_mut() {
            for message in messages {
                match message {
                    TrainMsg::Stats(stats) => {
                        session.episodes = stats.episodes as i64;
                        session.generations = stats.generation;
                        session.latest = Some(*stats);
                    }
                    TrainMsg::Snapshot(policy, generation) => {
                        session.snapshot = Some(*policy);
                        session.snapshot_generation = generation;
                        reset_hero = true;
                    }
                    TrainMsg::Failed(error) => session.failure = Some(error),
                }
            }
        }
        if reset_hero {
            let seed = self.next_seed();
            if let Some(session) = self.training.as_mut() {
                session.hero_policy = session.snapshot.clone();
                session.hero.reset(seed, DEFAULT_LAPS);
            }
        }
        if self.paused {
            return;
        }
        let steps = (dt * DECISIONS_PER_SECOND * 8.0).floor().min(48.0) as usize;
        let mut ended = false;
        if let Some(session) = self.training.as_mut() {
            session.hero_policy = session.snapshot.clone();
            for _ in 0..steps {
                if session.hero.advance(session.hero_policy.as_ref(), None)
                    || session.hero.is_over()
                {
                    ended = true;
                    break;
                }
            }
        }
        if ended {
            let seed = self.next_seed();
            if let Some(session) = self.training.as_mut() {
                session.hero.reset(seed, DEFAULT_LAPS);
            }
        }
    }

    fn handle_shortcuts(&mut self, ctx: &Context) {
        let mut actions = Vec::new();
        let pressed =
            |key: Key| ctx.input_mut(|input| input.consume_key(egui::Modifiers::NONE, key));
        if pressed(Key::Space) {
            actions.push(UiAction::TogglePause);
        }
        if pressed(Key::R) {
            actions.push(UiAction::Restart);
        }
        if pressed(Key::N) {
            actions.push(UiAction::NewTrack);
        }
        if pressed(Key::T) {
            actions.push(UiAction::ToggleRays);
        }
        if pressed(Key::P) {
            actions.push(UiAction::ToggleTrail);
        }
        if pressed(Key::Tab) {
            actions.push(UiAction::SetMode(match self.mode {
                Mode::Watch => Mode::Manual,
                Mode::Manual => Mode::Train,
                Mode::Train => Mode::Watch,
            }));
        }
        for (key, speed) in [
            (Key::Num1, 1.0),
            (Key::Num2, 2.0),
            (Key::Num3, 4.0),
            (Key::Num4, 8.0),
        ] {
            if pressed(key) {
                actions.push(UiAction::SetSpeed(speed));
            }
        }
        for action in actions {
            self.apply(action);
        }
    }

    fn manual_action(ctx: &Context) -> Action {
        let mut steer = 0.0;
        let mut throttle = 0.0;
        ctx.input(|input| {
            if input.key_down(Key::ArrowLeft) || input.key_down(Key::A) {
                steer -= 1.0;
            }
            if input.key_down(Key::ArrowRight) || input.key_down(Key::D) {
                steer += 1.0;
            }
            if input.key_down(Key::ArrowUp) || input.key_down(Key::W) {
                throttle = 1.0;
            }
            if input.key_down(Key::ArrowDown) || input.key_down(Key::S) {
                throttle = -1.0;
            }
        });
        Action { steer, throttle }
    }

    fn advance_race(&mut self, ctx: &Context, dt: f32) {
        if self.mode == Mode::Train || self.paused {
            return;
        }
        if self.race.is_over() {
            self.restart_timer += dt;
            if self.restart_timer > 1.4 {
                self.restart(true);
            }
            return;
        }
        let manual = if self.mode == Mode::Manual {
            Some(Self::manual_action(ctx))
        } else {
            None
        };
        let rate = if self.mode == Mode::Manual {
            DECISIONS_PER_SECOND
        } else {
            DECISIONS_PER_SECOND * self.speed
        };
        self.accumulator += dt * rate;
        let mut budget = self.accumulator.floor().min(60.0);
        self.accumulator -= budget;
        while budget > 0.0 {
            let ended = self.race.advance(self.policy.as_ref(), manual);
            budget -= 1.0;
            if ended {
                self.record_outcome();
                break;
            }
        }
    }

    fn reload_policy(&mut self) {
        match Policy::load(&self.policy_path) {
            Ok(policy) => {
                self.status = format!(
                    "loaded {} ({} parameters)",
                    self.policy_path.display(),
                    policy.num_params()
                );
                self.policy = Some(policy);
                self.restart(true);
            }
            Err(error) => {
                self.status = format!("could not load {}: {error}", self.policy_path.display());
            }
        }
    }

    fn header(&self, ui: &mut Ui, actions: &mut Vec<UiAction>) {
        ui.horizontal(|ui| {
            ui.label(
                RichText::new("NEURAL RACING")
                    .size(18.0)
                    .strong()
                    .color(colors::TEXT),
            );
            ui.label(
                RichText::new("random tracks · learnt driver")
                    .size(12.0)
                    .color(colors::MUTED),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                for (mode, label) in [
                    (Mode::Watch, "Watch AI"),
                    (Mode::Manual, "Manual"),
                    (Mode::Train, "Train"),
                ] {
                    let selected = self.mode == mode;
                    if ui
                        .add(
                            egui::Button::new(
                                RichText::new(label)
                                    .size(13.0)
                                    .color(if selected { colors::DARK } else { colors::TEXT }),
                            )
                            .fill(if selected { colors::ACCENT } else { colors::PANEL })
                            .stroke(Stroke::new(1.0, colors::PANEL_EDGE))
                            .corner_radius(14),
                        )
                        .clicked()
                    {
                        actions.push(UiAction::SetMode(mode));
                    }
                }
            });
        });
    }

    fn race_section(&self, ui: &mut Ui, race: &RaceView) {
        ui.horizontal(|ui| {
            let (label, color) = match race.outcome {
                None => ("RACING", colors::ACCENT),
                Some(Outcome::Finished { .. }) => ("FINISHED", colors::GOOD),
                Some(Outcome::Crashed { .. }) => ("OFF TRACK", colors::BAD),
                Some(Outcome::TimedOut { .. }) => ("TIME UP", colors::WARN),
            };
            widgets::pill(ui, label, color);
            if let Some(outcome) = race.outcome {
                ui.label(
                    RichText::new(format!("{:.1}s", outcome.seconds()))
                        .size(14.0)
                        .monospace()
                        .color(colors::TEXT),
                );
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                ui.label(
                    RichText::new(format!("track #{:06X}", race.track_seed & 0xFF_FFFF))
                        .size(11.0)
                        .monospace()
                        .color(colors::MUTED),
                );
            });
        });

        widgets::stat_row(
            ui,
            "Lap",
            &format!("{} / {}", race.laps_done.min(DEFAULT_LAPS), DEFAULT_LAPS),
            colors::TEXT,
        );
        widgets::stat_row(
            ui,
            "Race time",
            &format!("{:.2}s", race.elapsed()),
            colors::TEXT,
        );
        widgets::stat_row(
            ui,
            "Best lap",
            &race
                .best_lap()
                .map(|lap| format!("{lap:.2}s"))
                .unwrap_or_else(|| "—".to_string()),
            colors::GOOD,
        );

        let gates = race.gates_total().max(1);
        ui.add(
            egui::ProgressBar::new(race.gates_done() as f32 / gates as f32)
                .desired_width(ui.available_width())
                .fill(colors::ACCENT.gamma_multiply(0.8))
                .text(
                    RichText::new(format!("checkpoints {} / {}", race.gates_done(), gates))
                        .size(11.0),
                ),
        );
    }

    fn control_section(&self, ui: &mut Ui, actions: &mut Vec<UiAction>) {
        ui.horizontal(|ui| {
            let label = if self.paused { "Resume" } else { "Pause" };
            if ui
                .add(
                    egui::Button::new(RichText::new(label).size(14.0))
                        .fill(colors::ACCENT.gamma_multiply(0.25))
                        .min_size(Vec2::new(112.0, 32.0)),
                )
                .clicked()
            {
                actions.push(UiAction::TogglePause);
            }
            if ui
                .add(
                    egui::Button::new(RichText::new("Restart").size(14.0))
                        .min_size(Vec2::new(104.0, 32.0)),
                )
                .clicked()
            {
                actions.push(UiAction::Restart);
            }
            if ui
                .add(
                    egui::Button::new(RichText::new("New track").size(14.0))
                        .min_size(Vec2::new(112.0, 32.0)),
                )
                .clicked()
            {
                actions.push(UiAction::NewTrack);
            }
        });
        ui.horizontal(|ui| {
            ui.label(RichText::new("Time").size(12.0).color(colors::MUTED));
            for speed in [1.0f32, 2.0, 4.0, 8.0] {
                let selected = (self.speed - speed).abs() < f32::EPSILON;
                if ui
                    .add(
                        egui::Button::new(
                            RichText::new(format!("{speed:.0}×"))
                                .size(12.0)
                                .color(if selected { colors::DARK } else { colors::TEXT }),
                        )
                        .fill(if selected { colors::ACCENT } else { colors::PANEL })
                        .min_size(Vec2::new(38.0, 24.0)),
                    )
                    .clicked()
                {
                    actions.push(UiAction::SetSpeed(speed));
                }
            }
            if ui
                .selectable_label(self.show_rays, RichText::new("Rays").size(12.0))
                .clicked()
            {
                actions.push(UiAction::ToggleRays);
            }
            if ui
                .selectable_label(self.show_trail, RichText::new("Trail").size(12.0))
                .clicked()
            {
                actions.push(UiAction::ToggleTrail);
            }
        });
    }

    fn training_section(&self, ui: &mut Ui, actions: &mut Vec<UiAction>) {
        let Some(session) = self.training.as_ref() else {
            ui.label(RichText::new("Training is not running.").size(12.0));
            return;
        };
        let running = session.running.load(Ordering::Relaxed);
        ui.horizontal(|ui| {
            if ui
                .add(
                    egui::Button::new(
                        RichText::new(if running {
                            "Pause training"
                        } else {
                            "Resume training"
                        })
                        .size(13.0),
                    )
                    .fill(colors::ACCENT.gamma_multiply(0.22))
                    .min_size(Vec2::new(152.0, 30.0)),
                )
                .clicked()
            {
                actions.push(UiAction::ToggleTraining);
            }
            if ui
                .add_enabled(
                    session.snapshot.is_some(),
                    egui::Button::new(RichText::new("Use this policy").size(13.0))
                        .min_size(Vec2::new(134.0, 30.0)),
                )
                .clicked()
            {
                actions.push(UiAction::UsePolicy);
            }
        });

        if let Some(error) = &session.failure {
            ui.label(RichText::new(error).size(12.0).color(colors::BAD));
        }

        widgets::stat_row(
            ui,
            "Generation",
            &format!("{}", session.generations),
            colors::TEXT,
        );
        widgets::stat_row(
            ui,
            "Episodes",
            &format!("{}", session.episodes),
            colors::TEXT,
        );
        if let Some(stats) = &session.latest {
            let ratio = stats.finished as f64 / stats.episodes.max(1) as f64;
            widgets::stat_row(
                ui,
                "Last generation",
                &format!("{} / {} finished", stats.finished, stats.episodes),
                if ratio > 0.5 { colors::GOOD } else { colors::WARN },
            );
            widgets::gauge(
                ui,
                "Mean checkpoints",
                stats.mean_checkpoints,
                0.0,
                f64::from(DEFAULT_LAPS * CHECKPOINT_COUNT as i32),
                colors::ACCENT,
                &format!("{:.1}", stats.mean_checkpoints),
            );
            widgets::gauge(
                ui,
                "Mean speed",
                stats.mean_speed,
                0.0,
                MAX_SPEED,
                colors::GOOD,
                &format!("{:.0} px/s", stats.mean_speed),
            );
            widgets::stat_row(
                ui,
                "KL · update scale",
                &format!(
                    "{:.4} · {:.2}",
                    stats.update.policy_kl, stats.update.update_scale
                ),
                colors::MUTED,
            );
            widgets::stat_row(
                ui,
                "Exploration σ",
                &format!("{:.2}", stats.sigma),
                colors::MUTED,
            );
        }
        ui.add_space(4.0);
        ui.label(
            RichText::new(format!(
                "{} generations in {:.0}s · snapshot every 4",
                session.generations,
                session.started_at.elapsed().as_secs_f32()
            ))
            .size(11.0)
            .color(colors::MUTED),
        );
    }

    fn telemetry_section(&self, ui: &mut Ui, race: &RaceView) {
        let env = &race.env;
        widgets::gauge(
            ui,
            "Speed",
            env.speed,
            0.0,
            MAX_SPEED,
            colors::ACCENT,
            &format!("{:.0} px/s", env.speed),
        );
        widgets::centered_gauge(
            ui,
            "Steering",
            env.steer_angle / MAX_STEER_ANGLE,
            colors::GOOD,
            &format!("{:+.2}", env.steer_angle / MAX_STEER_ANGLE),
        );
        widgets::centered_gauge(
            ui,
            "Throttle",
            env.last_action.throttle,
            colors::WARN,
            &format!("{:+.2}", env.last_action.throttle),
        );
        widgets::gauge(
            ui,
            "Wall clearance",
            env.min_ray,
            0.0,
            RAY_RANGE,
            if env.min_ray < 0.8 * env.road_half_width {
                colors::BAD
            } else {
                colors::GOOD
            },
            &format!("{:.0} px", env.min_ray),
        );
        widgets::gauge(
            ui,
            "Road width",
            env.road_half_width * 2.0,
            30.0,
            90.0,
            colors::ACCENT,
            &format!("{:.0} px", env.road_half_width * 2.0),
        );
        widgets::centered_gauge(
            ui,
            "Heading error",
            env.heading_error / std::f64::consts::PI,
            colors::ACCENT,
            &format!("{:+.0}°", env.heading_error.to_degrees()),
        );
    }

    fn brain_section(&self, ui: &mut Ui, race: &RaceView, actions: &mut Vec<UiAction>) {
        let policy = match self.mode {
            Mode::Train => self
                .training
                .as_ref()
                .and_then(|session| session.hero_policy.as_ref()),
            _ => self.policy.as_ref(),
        };
        let Some(policy) = policy else {
            ui.label(
                RichText::new(
                    "No trained policy loaded. Train one, or drop a policy file at models/policy.bin.",
                )
                .size(12.0)
                .color(colors::MUTED),
            );
            if ui
                .add(
                    egui::Button::new(RichText::new("Start training").size(13.0))
                        .fill(colors::ACCENT.gamma_multiply(0.25)),
                )
                .clicked()
            {
                actions.push(UiAction::SetMode(Mode::Train));
            }
            return;
        };

        let activations = policy.activations(&race.env.obs);
        widgets::brain(ui, &activations);
        let outputs = activations.last().cloned().unwrap_or_default();
        let steer = outputs.first().copied().unwrap_or(0.0).tanh();
        let throttle = outputs.get(1).copied().unwrap_or(0.0).tanh();
        ui.label(
            RichText::new(format!(
                "output · steer {steer:+.2} · throttle {throttle:+.2}"
            ))
            .size(11.0)
            .monospace()
            .color(colors::MUTED),
        );
    }

    fn session_section(&self, ui: &mut Ui, actions: &mut Vec<UiAction>) {
        widgets::stat_row(
            ui,
            "Races finished",
            &format!("{}", self.finished),
            colors::GOOD,
        );
        widgets::stat_row(ui, "Races crashed", &format!("{}", self.crashed), colors::BAD);
        widgets::stat_row(
            ui,
            "Session best lap",
            &self
                .best_lap
                .map(|lap| format!("{lap:.2}s"))
                .unwrap_or_else(|| "—".to_string()),
            colors::GOOD,
        );
        ui.add_space(6.0);
        ui.label(
            RichText::new(format!("policy: {}", self.policy_path.display()))
                .size(10.0)
                .monospace()
                .color(colors::MUTED),
        );
        if ui
            .add(
                egui::Button::new(RichText::new("Reload policy from disk").size(12.0))
                    .fill(colors::PANEL),
            )
            .clicked()
        {
            actions.push(UiAction::ReloadPolicy);
        }
        if !self.status.is_empty() {
            ui.add_space(4.0);
            ui.label(RichText::new(&self.status).size(11.0).color(colors::MUTED));
        }
        ui.add_space(10.0);
    }

    fn dashboard(&mut self, ui: &mut Ui) {
        let mut actions = Vec::new();
        egui::ScrollArea::vertical().show(ui, |ui| {
            widgets::section(ui, "Race");
            self.race_section(ui, self.active_race());

            widgets::section(ui, "Control");
            self.control_section(ui, &mut actions);

            if self.mode == Mode::Train {
                widgets::section(ui, "Training");
                self.training_section(ui, &mut actions);
            }

            widgets::section(ui, "Telemetry");
            self.telemetry_section(ui, self.active_race());

            widgets::section(ui, "Brain");
            self.brain_section(ui, self.active_race(), &mut actions);

            widgets::section(ui, "Session");
            self.session_section(ui, &mut actions);
        });
        for action in actions {
            self.apply(action);
        }
    }

    fn canvas(&self, ui: &mut Ui) {
        let size = ui.available_size();
        let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
        let painter = ui.painter_at(rect);
        let inner = rect.shrink(2.0);
        render::draw_background(&painter, inner);
        let race = self.active_race();
        let view = View::fit(inner.shrink(6.0), race.env.track.bounds, 34.0);

        render::draw_track(&painter, &view, &race.env.track);
        if self.show_trail {
            render::draw_trail(&painter, &view, &race.trail);
        }
        render::draw_skids(&painter, &view, &race.skids);
        if self.show_rays {
            render::draw_rays(&painter, &view, &race.env);
        }
        let crashed = matches!(race.outcome, Some(Outcome::Crashed { .. }));
        render::draw_car(&painter, &view, &race.env, crashed);

        painter.text(
            inner.left_top() + Vec2::new(12.0, 10.0),
            Align2::LEFT_TOP,
            format!(
                "LAP {} / {}   {:.1}s",
                (race.laps_done + 1).min(DEFAULT_LAPS),
                DEFAULT_LAPS,
                race.elapsed()
            ),
            FontId::monospace(13.0),
            colors::TEXT,
        );
        painter.text(
            inner.right_top() + Vec2::new(-12.0, 10.0),
            Align2::RIGHT_TOP,
            format!(
                "road {:.0}-{:.0}px · {} samples",
                race.env.track.width_range().0 * 2.0,
                race.env.track.width_range().1 * 2.0,
                race.env.track.centerline.len()
            ),
            FontId::monospace(10.0),
            colors::MUTED,
        );
        if self.mode == Mode::Manual && !race.is_over() {
            painter.text(
                inner.left_bottom() + Vec2::new(12.0, -12.0),
                Align2::LEFT_BOTTOM,
                "arrow keys / WASD to drive",
                FontId::proportional(12.0),
                colors::MUTED,
            );
        }

        if let Some(outcome) = race.outcome {
            let color = match outcome {
                Outcome::Finished { .. } => colors::GOOD,
                Outcome::Crashed { .. } => colors::BAD,
                Outcome::TimedOut { .. } => colors::WARN,
            };
            let banner = egui::Rect::from_center_size(inner.center(), Vec2::new(330.0, 92.0));
            painter.rect_filled(banner, 14, colors::DARK.gamma_multiply(0.92));
            painter.rect_stroke(
                banner,
                14,
                Stroke::new(1.5, color),
                egui::StrokeKind::Inside,
            );
            painter.text(
                egui::Pos2::new(banner.center().x, banner.center().y - 16.0),
                Align2::CENTER_CENTER,
                outcome.label(),
                FontId::proportional(24.0),
                color,
            );
            painter.text(
                egui::Pos2::new(banner.center().x, banner.center().y + 18.0),
                Align2::CENTER_CENTER,
                if race.lap_times.is_empty() {
                    format!("{:.1}s", outcome.seconds())
                } else {
                    format!(
                        "{:.1}s  ·  best lap {:.1}s",
                        outcome.seconds(),
                        race.best_lap().unwrap_or(0.0)
                    )
                },
                FontId::monospace(13.0),
                colors::MUTED,
            );
        }
    }
}

fn spawn_training(config: RaceConfig, sender: Sender<TrainMsg>, running: Arc<AtomicBool>) {
    std::thread::spawn(move || {
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut trainer = Trainer::new(config);
            while running.load(Ordering::Relaxed) {
                let stats = trainer.train_generation();
                let generation = stats.generation;
                if sender.send(TrainMsg::Stats(Box::new(stats))).is_err() {
                    return;
                }
                if generation % 4 == 0
                    && sender
                        .send(TrainMsg::Snapshot(
                            Box::new(trainer.policy.clone()),
                            generation,
                        ))
                        .is_err()
                {
                    return;
                }
            }
        }));
        if result.is_err() {
            let _ = sender.send(TrainMsg::Failed(
                "training thread stopped after an error; press Train again".to_string(),
            ));
        }
    });
}

impl Drop for App {
    fn drop(&mut self) {
        if let Some(session) = &self.training {
            session.running.store(false, Ordering::Relaxed);
        }
    }
}

impl EframeApp for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.render(ui);
    }
}

impl App {
    fn frame(&mut self, ctx: &Context, ui: &mut Ui) {
        let dt = ctx.input(|input| input.stable_dt).min(0.1);
        self.handle_shortcuts(ctx);
        self.advance_race(&ctx, dt);
        self.poll_training(dt);

        let panel_frame = egui::Frame::NONE
            .fill(colors::PANEL)
            .inner_margin(egui::Margin::symmetric(14, 8));

        let mut actions = Vec::new();
        egui::Panel::top("header")
            .frame(panel_frame)
            .show(ui, |ui| self.header(ui, &mut actions));
        for action in actions {
            self.apply(action);
        }

        egui::Panel::bottom("status")
            .frame(
                egui::Frame::NONE
                    .fill(colors::PANEL)
                    .inner_margin(egui::Margin::symmetric(14, 6)),
            )
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    ui.label(
                        RichText::new(
                            "tab mode · space pause · R restart · N new track · T rays · P trail · 1-4 speed",
                        )
                        .size(11.0)
                        .color(colors::MUTED),
                    );
                    ui.with_layout(
                        egui::Layout::right_to_left(egui::Align::Center),
                        |ui| {
                            let mode = match self.mode {
                                Mode::Watch => "watching the trained AI",
                                Mode::Manual => "manual drive",
                                Mode::Train => "training live",
                            };
                            ui.label(RichText::new(mode).size(11.0).color(colors::ACCENT));
                        },
                    );
                });
            });

        egui::Panel::right("dashboard")
            .exact_size(384.0)
            .resizable(false)
            .frame(
                egui::Frame::NONE
                    .fill(colors::PANEL)
                    .inner_margin(egui::Margin::symmetric(14, 6)),
            )
            .show(ui, |ui| self.dashboard(ui));

        egui::CentralPanel::default_margins()
            .frame(
                egui::Frame::NONE
                    .fill(colors::DARK)
                    .inner_margin(egui::Margin::same(10)),
            )
            .show(ui, |ui| self.canvas(ui));

        ctx.request_repaint();
    }
}

/// Locates a saved policy: explicit path, then the repo's `models` directory.
pub fn find_policy(explicit: Option<PathBuf>) -> (Option<Policy>, PathBuf) {
    let mut candidates: Vec<PathBuf> = Vec::new();
    if let Some(path) = explicit {
        candidates.push(path);
    }
    let repo_models = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models");
    candidates.push(repo_models.join("policy.bin"));
    candidates.push(repo_models.join("best.bin"));
    candidates.push(PathBuf::from("models/policy.bin"));
    for candidate in &candidates {
        if candidate.exists() {
            if let Ok(policy) = Policy::load(candidate) {
                if policy.obs_dim == neural_racing_core::OBS_DIM
                    && policy.hidden_dims.len() == DEFAULT_HIDDEN_DIMS.len()
                {
                    return (Some(policy), candidate.clone());
                }
            }
        }
    }
    (None, candidates[0].clone())
}
