//! The application: modes, controls, dashboard and live training.

use neural_racing_diagnostics::{emit, json};
use std::path::PathBuf;

use eframe::App as EframeApp;
use egui::{Align2, Context, FontId, Key, RichText, Sense, Stroke, Ui, Vec2};

use crate::learning::{Learning, Snapshot};
use neural_racing_core::{
    Action, Circuit, Rng, DECISION_DT, DEFAULT_LAPS, MAX_SPEED, MAX_STEER_ANGLE, RAY_RANGE,
    UNITS_PER_METRE,
};
use neural_racing_engine::live::{
    normalize_speed, LiveTrainer, CAR_COUNT, MAX_REQUESTED_SPEED, STINT_LAPS,
};
use neural_racing_ml::{Policy, DEFAULT_BLOCKS, DEFAULT_WIDTH};

use crate::palette as colors;
use crate::render::{self, View};
use crate::sim::{Outcome, RaceView};
use crate::widgets;

const DECISIONS_PER_SECOND: f32 = 1.0 / DECISION_DT as f32;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Watch,
    Manual,
    Train,
}

struct TrainingSession {
    worker: Learning,
    latest: Option<Snapshot>,
    cars: [RaceView; CAR_COUNT],
}

enum UiAction {
    TogglePause,
    Restart,
    NewTrack,
    SetSpeed(f32),
    ToggleRays,
    ToggleTrail,
    SetMode(Mode),
    Follow(usize),
    SelectCircuit(Option<Circuit>),
    ToggleOverview,
}

pub struct App {
    mode: Mode,
    circuit: Option<Circuit>,
    course_filter: String,
    car_sprite: Option<render::CarSprite>,
    overview: bool,
    followed: usize,
    policy: Option<Policy>,
    race: RaceView,
    speed: f32,
    maximum_speed: bool,
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
    diagnostic_last_frame: std::time::Instant,
    diagnostic_last_report: std::time::Instant,
    diagnostic_max_gap_ms: u64,
    diagnostic_frames: u64,
    diagnostic_stale_frames: u64,
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

    pub fn new(policy: Option<Policy>, _policy_path: PathBuf) -> Self {
        Self {
            mode: Mode::Train,
            circuit: Some(Circuit::Monza),
            course_filter: String::new(),
            car_sprite: None,
            overview: false,
            followed: 0,
            policy,
            race: RaceView::on_circuit(Circuit::Monza, DEFAULT_LAPS),
            speed: 1.0,
            maximum_speed: false,
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
            diagnostic_last_frame: std::time::Instant::now(),
            diagnostic_last_report: std::time::Instant::now(),
            diagnostic_max_gap_ms: 0,
            diagnostic_frames: 0,
            diagnostic_stale_frames: 0,
        }
    }

    /// Selects a seeded procedural course instead of the default real circuit.
    pub fn seed_with(&mut self, seed: u64) {
        self.circuit = None;
        self.race.reset(seed, DEFAULT_LAPS);
        self.sync_training_track();
    }

    /// Applies launch flags before the first frame.
    pub fn configure(&mut self, seed: Option<u64>, mode: Option<Mode>, speed: Option<f32>) {
        if let Some(seed) = seed {
            self.seed_with(seed);
        }
        if let Some(speed) = speed {
            self.apply(UiAction::SetSpeed(speed));
        }
        if let Some(mode) = mode {
            self.apply(UiAction::SetMode(mode));
        }
        emit!(
            "ui_config",
            self.race.diagnostic_id,
            json!({"seed":seed,"mode":format!("{:?}",self.mode),
            "speed":self.speed,"uncapped":self.maximum_speed,"track":neural_racing_engine::diagnostics::track(&self.race.env.track)})
        );
    }

    fn practice_selected(&self) -> bool {
        self.circuit.is_some_and(Circuit::is_practice)
    }

    fn enter_practice(&mut self) {
        if self.practice_selected() {
            self.mode = Mode::Manual;
            if let Some(session) = &self.training {
                session.worker.running(false);
            }
        }
    }

    pub fn select_circuit(&mut self, circuit: Option<Circuit>) {
        self.circuit = circuit;
        self.status.clear();
        self.enter_practice();
        self.best_lap = None;
        self.restart(false);
        self.sync_training_track();
    }

    fn next_seed(&mut self) -> u64 {
        self.seed_source.next_u64() >> 8
    }

    /// The race currently on screen, whichever mode is active.
    pub fn race(&self) -> &RaceView {
        self.active_race()
    }

    /// Latest live training state received from the worker.
    pub fn training_stats(&self) -> Option<&Snapshot> {
        self.training
            .as_ref()
            .and_then(|session| session.latest.as_ref())
    }

    fn active_race(&self) -> &RaceView {
        if self.mode == Mode::Train {
            if let Some(session) = self.training.as_ref() {
                return &session.cars[self.followed];
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
        if let Some(circuit) = self.circuit {
            if new_track {
                let courses = Circuit::all();
                let index = courses.iter().position(|c| *c == circuit).unwrap_or(0);
                self.circuit = Some(courses[(index + 1) % courses.len()]);
                self.best_lap = None;
            }
            self.race = RaceView::on_circuit(self.circuit.unwrap(), DEFAULT_LAPS);
        } else {
            self.race.reset(seed, DEFAULT_LAPS);
        }
        emit!(
            "race_restarted",
            self.race.diagnostic_id,
            json!({"mode":format!("{:?}",self.mode),"seed":seed,
            "track":neural_racing_engine::diagnostics::track(&self.race.env.track),"new_track":new_track,
            "parent":neural_racing_diagnostics::current_operation()})
        );
        self.restart_timer = 0.0;
        self.accumulator = 0.0;
    }

    fn sync_training_track(&mut self) {
        self.enter_practice();
        if self.practice_selected() {
            return;
        }
        if let Some(session) = self.training.as_mut() {
            session.worker.track(self.race.env.track.clone());
            session.latest = None;
            for car in &mut session.cars {
                car.receive(neural_racing_core::Env::new(
                    self.race.env.track.clone(),
                    STINT_LAPS,
                ));
            }
        }
    }

    fn record_outcome(&mut self) {
        let Some(outcome) = self.race.outcome else {
            return;
        };
        emit!(
            "race_ended",
            self.race.diagnostic_id,
            json!({"mode":format!("{:?}",self.mode),"seed":self.race.track_seed,
            "reason":outcome.label(),"state":neural_racing_engine::diagnostics::state(&self.race.env)})
        );
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
        let operation = neural_racing_diagnostics::operation();
        let _context = neural_racing_diagnostics::context(operation);
        let action_name = match &action {
            UiAction::TogglePause => "pause",
            UiAction::Restart => "restart",
            UiAction::NewTrack => "next_track",
            UiAction::SetSpeed(_) => "speed",
            UiAction::SetMode(_) => "mode",
            UiAction::SelectCircuit(_) => "track",
            UiAction::ToggleOverview => "overview",
            UiAction::Follow(_) => "follow",
            UiAction::ToggleRays => "rays",
            UiAction::ToggleTrail => "trail",
        };
        emit!(
            "ui_control_requested",
            operation,
            json!({"action":action_name,"mode":format!("{:?}",self.mode),
            "race":self.race.diagnostic_id,
            "requested_circuit":if let UiAction::SelectCircuit(circuit) = &action {
                Some(circuit.map(|c|c.slug()).unwrap_or("procedural"))
            } else { None },
            "requested_mode":if let UiAction::SetMode(mode) = &action {
                Some(format!("{mode:?}"))
            } else { None }})
        );
        match action {
            UiAction::SelectCircuit(circuit) => self.select_circuit(circuit),
            UiAction::ToggleOverview => self.overview = !self.overview,
            UiAction::TogglePause => {
                self.paused = !self.paused;
                if let Some(session) = &self.training {
                    session
                        .worker
                        .running(self.mode == Mode::Train && !self.paused);
                }
            }
            UiAction::Restart => {
                if self.mode == Mode::Train {
                    let seed = self.next_seed();
                    if let Some(session) = &mut self.training {
                        session.worker.reset(seed);
                        session.latest = None;
                        for car in &mut session.cars {
                            car.env.reset();
                            car.trail.clear();
                            car.outcome = None;
                        }
                    }
                } else {
                    self.restart(false);
                }
            }
            UiAction::NewTrack => {
                self.restart(true);
                self.sync_training_track();
            }
            UiAction::SetSpeed(speed) => {
                let requested = normalize_speed(speed as f64);
                self.maximum_speed = requested.is_infinite();
                self.speed = if self.maximum_speed {
                    MAX_REQUESTED_SPEED as f32
                } else {
                    requested as f32
                };
                if let Some(session) = &self.training {
                    session.worker.speed(if self.maximum_speed {
                        f64::INFINITY
                    } else {
                        self.speed as f64
                    });
                }
            }
            UiAction::Follow(index) => self.followed = index.min(CAR_COUNT - 1),
            UiAction::ToggleRays => self.show_rays = !self.show_rays,
            UiAction::ToggleTrail => self.show_trail = !self.show_trail,
            UiAction::SetMode(mode) => {
                if self.practice_selected() && mode != Mode::Manual {
                    self.status = "Practice areas support manual driving. Select a race route to learn or watch.".into();
                    return;
                }
                if mode == Mode::Watch {
                    if let Some(snapshot) = self.training_stats() {
                        self.policy = Some((*snapshot.policy).clone());
                        self.race.env.reset();
                        self.race.outcome = None;
                        self.race.trail.clear();
                        self.race.laps_done = 0;
                        self.race.lap_times.clear();
                        self.race.lap_start_step = 0;
                    }
                }
                self.mode = mode;
                if mode == Mode::Train && self.training.is_none() {
                    self.start_training();
                }
                if let Some(session) = &self.training {
                    session.worker.running(mode == Mode::Train && !self.paused);
                }
            }
        }
        emit!(
            "ui_control",
            operation,
            json!({"action":action_name,"mode":format!("{:?}",self.mode),
            "paused":self.paused,"speed":self.speed,"uncapped":self.maximum_speed,
            "race":self.race.diagnostic_id,"overview":self.overview,"followed":self.followed,
            "rays":self.show_rays,"trail":self.show_trail,"circuit":self.circuit.map(|c|c.slug()).unwrap_or("procedural")})
        );
    }

    fn start_training(&mut self) {
        if self.practice_selected() {
            return;
        }
        let track = self.race.env.track.clone();
        let seed = self.next_seed();
        let worker = Learning::start(
            track.clone(),
            seed,
            if self.maximum_speed {
                f64::INFINITY
            } else {
                self.speed as f64
            },
        );
        worker.running(!self.paused && self.mode == Mode::Train);
        self.training = Some(TrainingSession {
            worker,
            latest: None,
            cars: std::array::from_fn(|_| {
                let mut car = RaceView::on_circuit(Circuit::Monza, STINT_LAPS);
                car.receive(neural_racing_core::Env::new(track.clone(), STINT_LAPS));
                car
            }),
        });
    }

    fn poll_training(&mut self) {
        if self.mode == Mode::Train && self.training.is_none() {
            self.start_training();
        }
        if let Some(session) = &mut self.training {
            if let Some(snapshot) = session.worker.take() {
                if snapshot.revision != session.worker.revision {
                    self.diagnostic_stale_frames += 1;
                    return;
                }
                // Acknowledge pause with the worker's exact stopped state.
                if self.paused && !snapshot.paused {
                    return;
                }
                for (i, car) in session.cars.iter_mut().enumerate() {
                    car.receive(snapshot.cars[i].clone());
                    car.trail = snapshot.trails[i].clone();
                }
                session.latest = Some(snapshot);
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
        if pressed(Key::V) {
            actions.push(UiAction::ToggleOverview);
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
                self.restart(false);
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
        self.accumulator = (self.accumulator + dt * rate).min((rate * 0.05).max(1.0));
        let mut budget = self.accumulator.floor();
        self.accumulator -= budget;
        let started = std::time::Instant::now();
        while budget > 0.0 {
            let ended = self.race.advance(self.policy.as_ref(), manual);
            budget -= 1.0;
            if ended {
                self.record_outcome();
                break;
            }
            if started.elapsed() >= std::time::Duration::from_millis(4) {
                break;
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
                RichText::new("watch a driver learn")
                    .size(12.0)
                    .color(colors::MUTED),
            );
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                for (mode, label) in [
                    (Mode::Watch, "Watch model"),
                    (Mode::Manual, "Manual"),
                    (Mode::Train, "Learn"),
                ] {
                    let selected = self.mode == mode;
                    if ui
                        .add_enabled(
                            !self.practice_selected() || mode == Mode::Manual,
                            egui::Button::new(RichText::new(label).size(13.0).color(if selected {
                                colors::DARK
                            } else {
                                colors::TEXT
                            }))
                            .fill(if selected {
                                colors::ACCENT
                            } else {
                                colors::PANEL
                            })
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
                None if self.practice_selected() => ("PRACTICE", colors::ACCENT),
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
                    RichText::new(
                        race.env
                            .track
                            .circuit
                            .map(|c| c.name().to_string())
                            .unwrap_or_else(|| {
                                format!("track #{:06X}", race.track_seed & 0xFF_FFFF)
                            }),
                    )
                    .size(11.0)
                    .monospace()
                    .color(colors::MUTED),
                );
            });
        });

        if self.practice_selected() {
            ui.label("Practice area · manual driving");
            ui.label("No laps, checkpoint rewards or race finish.");
            widgets::stat_row(
                ui,
                "Practice time",
                &format!("{:.2}s", race.elapsed()),
                colors::TEXT,
            );
            return;
        }
        if race.env.track.closed {
            widgets::stat_row(
                ui,
                "Lap",
                &format!("{} / {}", race.laps_done.min(race.env.laps), race.env.laps),
                colors::TEXT,
            );
        } else {
            let progress = if race.env.lap_done {
                1.0
            } else {
                (race.env.distance / race.env.track.lap_length).clamp(0.0, 1.0)
            };
            widgets::stat_row(
                ui,
                "Route progress",
                &format!("{:.1}%", progress * 100.0),
                colors::TEXT,
            );
        }
        widgets::stat_row(
            ui,
            "Race time",
            &format!("{:.2}s", race.elapsed()),
            colors::TEXT,
        );
        if race.env.track.closed {
            widgets::stat_row(
                ui,
                "Best lap",
                &race
                    .best_lap()
                    .map(|lap| format!("{lap:.2}s"))
                    .unwrap_or_else(|| "—".to_string()),
                colors::GOOD,
            );
        }

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

    fn control_section(&mut self, ui: &mut Ui, actions: &mut Vec<UiAction>) {
        egui::ComboBox::from_label("Circuit")
            .selected_text(
                self.circuit
                    .map(|c| c.name())
                    .unwrap_or("Procedural course"),
            )
            .show_ui(ui, |ui| {
                ui.text_edit_singleline(&mut self.course_filter);
                let filter = self.course_filter.to_lowercase();
                egui::ScrollArea::vertical()
                    .max_height(280.0)
                    .show(ui, |ui| {
                        for &circuit in Circuit::all() {
                            if !filter.is_empty()
                                && !circuit.name().to_lowercase().contains(&filter)
                                && !circuit.slug().to_lowercase().contains(&filter)
                            {
                                continue;
                            }
                            if ui
                                .selectable_label(
                                    self.circuit == Some(circuit),
                                    match circuit {
                                        Circuit::Gt4(_) => {
                                            format!(
                                                "{} · {}{}",
                                                circuit.name(),
                                                circuit.slug(),
                                                if circuit.is_practice() {
                                                    " · Manual practice"
                                                } else {
                                                    ""
                                                }
                                            )
                                        }
                                        _ => circuit.name().to_owned(),
                                    },
                                )
                                .clicked()
                            {
                                actions.push(UiAction::SelectCircuit(Some(circuit)));
                            }
                        }
                        for course in neural_racing_core::course_catalog::courses()
                            .iter()
                            .filter(|c| !c.available)
                        {
                            if !filter.is_empty()
                                && !course.name.to_lowercase().contains(&filter)
                                && !course.id.to_lowercase().contains(&filter)
                            {
                                continue;
                            }
                            ui.add_enabled(
                                false,
                                egui::Label::new(format!(
                                    "{} · {:?}",
                                    course.name, course.topology
                                )),
                            );
                        }
                    });
                if self.mode != Mode::Train
                    && ui
                        .selectable_label(self.circuit.is_none(), "Procedural course")
                        .clicked()
                {
                    actions.push(UiAction::SelectCircuit(None));
                }
            });
        if let Some(Circuit::Gt4(index)) = self.circuit {
            use neural_racing_core::course_catalog::{CourseCondition, CourseTopology};
            let course = &neural_racing_core::course_catalog::courses()[usize::from(index)];
            let condition = match course.condition {
                CourseCondition::Dry => "Dry",
                CourseCondition::Wet => "Wet",
                CourseCondition::Dirt => "Dirt",
                CourseCondition::Snow => "Snow",
                CourseCondition::Unknown => "Condition unknown",
            };
            let route = match course.topology {
                CourseTopology::Closed => "Circuit",
                CourseTopology::Open => "Open route",
                CourseTopology::Area => "Manual practice",
                _ => "Route unknown",
            };
            ui.label(format!("GT4 · {condition} · {route}"));
        }
        if self.practice_selected() {
            ui.label("Manual practice; learning and watching resume on race routes.");
        }
        ui.horizontal(|ui| {
            if ui
                .button(if self.paused { "Resume" } else { "Pause" })
                .clicked()
            {
                actions.push(UiAction::TogglePause);
            }
            if ui
                .button(if self.mode == Mode::Train {
                    "Start over"
                } else {
                    "Restart"
                })
                .clicked()
            {
                actions.push(UiAction::Restart);
            }
            if ui.button("Next circuit").clicked() {
                actions.push(UiAction::NewTrack);
            }
        });
        ui.add_enabled_ui(self.mode != Mode::Manual, |ui| {
            let mut speed = self.speed;
            if ui
                .add(
                    egui::Slider::new(&mut speed, 0.25..=MAX_REQUESTED_SPEED as f32)
                        .logarithmic(true)
                        .text("× speed")
                        .max_decimals(2),
                )
                .changed()
            {
                actions.push(UiAction::SetSpeed(speed));
            }
            ui.horizontal(|ui| {
                if ui.small_button("1× real time").clicked() {
                    actions.push(UiAction::SetSpeed(1.0));
                }
                if self.mode == Mode::Train && ui.selectable_label(self.maximum_speed, "Max")
                    .on_hover_text("Run training as fast as available compute permits. Cars remain live; achieved speed is measured.")
                    .clicked() {
                    actions.push(UiAction::SetSpeed(if self.maximum_speed { self.speed } else { f32::INFINITY }));
                }
                let achieved = self.training_stats().map_or(0.0, |s| s.achieved_speed);
                if self.mode == Mode::Train {
                    ui.label(
                        RichText::new(format!("{achieved:.1}× achieved")).color(colors::ACCENT),
                    );
                }
            });
        });
        if self.mode == Mode::Train {
            ui.horizontal(|ui| {
                ui.label("Follow car");
                for i in 0..CAR_COUNT {
                    if ui
                        .selectable_label(self.followed == i, format!("{}", i + 1))
                        .clicked()
                    {
                        actions.push(UiAction::Follow(i));
                    }
                }
            });
        }
        ui.horizontal(|ui| {
            if ui.selectable_label(self.overview, "Overview (V)").clicked() {
                actions.push(UiAction::ToggleOverview);
            }
            if ui.selectable_label(self.show_rays, "Rays").clicked() {
                actions.push(UiAction::ToggleRays);
            }
            if ui.selectable_label(self.show_trail, "Trail").clicked() {
                actions.push(UiAction::ToggleTrail);
            }
        });
    }

    fn training_section(&self, ui: &mut Ui) {
        let Some(snapshot) = self.training_stats() else {
            ui.label("Loading learning session…");
            return;
        };
        if let Some(error) = &snapshot.error {
            ui.colored_label(colors::BAD, error);
        }
        let stats = &snapshot.stats[LiveTrainer::circuit_index(self.circuit)];
        if let Some((mastered, total, stints)) = snapshot.curriculum {
            let stage = if mastered < total {
                format!("Corners {mastered}/{total}")
            } else if !snapshot.closed_route {
                "Full routes".into()
            } else if stints {
                "Five-lap stints".into()
            } else {
                "Full laps".into()
            };
            widgets::stat_row(ui, "Practice stage", &stage, colors::ACCENT);
            widgets::stat_row(
                ui,
                "Safe section exits",
                &format!("{} / {}", stats.section_successes, stats.section_attempts),
                colors::GOOD,
            );
        }
        widgets::stat_row(
            ui,
            "Attempts · this circuit",
            &stats.attempts.to_string(),
            colors::TEXT,
        );
        if snapshot.closed_route {
            widgets::stat_row(ui, "Completed laps", &stats.laps.to_string(), colors::GOOD);
            widgets::stat_row(
                ui,
                "Five-lap goals",
                &stats.completed_stints.to_string(),
                colors::GOOD,
            );
            widgets::stat_row(
                ui,
                "Best consecutive laps",
                &format!("{} / {STINT_LAPS}", stats.best_streak),
                colors::GOOD,
            );
            widgets::stat_row(
                ui,
                "Best lap",
                &stats.best_lap.map_or("—".into(), |v| format!("{v:.2}s")),
                colors::GOOD,
            );
        } else {
            widgets::stat_row(
                ui,
                "Completed routes",
                &stats.completed_routes.to_string(),
                colors::GOOD,
            );
            widgets::stat_row(
                ui,
                "Best route",
                &stats.best_route.map_or("—".into(), |v| format!("{v:.2}s")),
                colors::GOOD,
            );
        }
        widgets::stat_row(
            ui,
            "Model updates · all circuits",
            &snapshot.updates.to_string(),
            colors::MUTED,
        );
        if let Some(update) = snapshot.last_update {
            widgets::stat_row(
                ui,
                "Policy KL · last update",
                &format!("{:.4}", update.policy_kl),
                if update.policy_kl < 0.02 {
                    colors::GOOD
                } else {
                    colors::WARN
                },
            );
            widgets::stat_row(
                ui,
                "Latent exploration σ",
                &format!("{:.2}", update.exploration_sigma),
                colors::ACCENT,
            );
            widgets::stat_row(
                ui,
                "Value loss",
                &format!("{:.3}", update.value_loss),
                colors::MUTED,
            );
            widgets::stat_row(
                ui,
                "Explained variance",
                &format!("{:+.2}", update.explained_variance),
                if update.explained_variance > 0.3 {
                    colors::GOOD
                } else {
                    colors::WARN
                },
            );
            widgets::stat_row(
                ui,
                "Control entropy",
                &format!("{:.2}", update.entropy),
                colors::MUTED,
            );
        }
        ui.label(
            RichText::new("Progress toward current practice goal")
                .size(12.0)
                .color(colors::MUTED),
        );
        let (rect, _) =
            ui.allocate_exact_size(Vec2::new(ui.available_width(), 92.0), Sense::hover());
        let p = ui.painter_at(rect);
        p.rect_filled(rect, 4, colors::DARK);
        let plot = rect.shrink2(Vec2::new(12.0, 14.0));
        for fraction in [0.0, 0.5, 1.0] {
            let y = plot.bottom() - fraction * plot.height();
            p.line_segment(
                [egui::pos2(plot.left(), y), egui::pos2(plot.right(), y)],
                Stroke::new(0.5, colors::PANEL_EDGE),
            );
        }
        let count = stats.history.len();
        let points: Vec<_> = stats
            .history
            .iter()
            .enumerate()
            .map(|(i, a)| {
                egui::pos2(
                    plot.left() + i as f32 / count.saturating_sub(1).max(1) as f32 * plot.width(),
                    plot.bottom() - a.progress as f32 * plot.height(),
                )
            })
            .collect();
        if points.len() > 1 {
            p.add(egui::Shape::line(points, Stroke::new(1.5, colors::ACCENT)));
        }
        p.text(
            rect.left_top() + Vec2::new(5.0, 2.0),
            Align2::LEFT_TOP,
            if snapshot.closed_route {
                "100% = current section, lap or stint"
            } else {
                "100% = current section or full route"
            },
            FontId::proportional(10.0),
            colors::MUTED,
        );
        if count == 0 {
            p.text(
                rect.center(),
                Align2::CENTER_CENTER,
                "First attempts in progress",
                FontId::proportional(12.0),
                colors::MUTED,
            );
        }
        ui.label(
            RichText::new(if snapshot.closed_route {
                "Corner practice, then full laps and five-lap stints"
            } else {
                "Corner practice, then full routes"
            })
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
            &format!("{:.0} km/h", env.speed / UNITS_PER_METRE * 3.6),
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
            "Throttle / brake",
            env.pedal,
            colors::WARN,
            &format!("{:+.2}", env.pedal),
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
            &format!("{:.1} m", env.min_ray / UNITS_PER_METRE),
        );
        widgets::gauge(
            ui,
            "Road width",
            env.road_width,
            30.0,
            16.0 * UNITS_PER_METRE,
            colors::ACCENT,
            &format!("{:.1} m", env.road_width / UNITS_PER_METRE),
        );
        widgets::centered_gauge(
            ui,
            "Heading error",
            env.heading_error / std::f64::consts::PI,
            colors::ACCENT,
            &format!("{:+.0}°", env.heading_error.to_degrees()),
        );
        let limit = env.speed_profile.max(1.0);
        widgets::gauge(
            ui,
            "Speed vs braking profile",
            env.speed / limit,
            0.0,
            1.5,
            if env.speed > limit {
                colors::BAD
            } else {
                colors::GOOD
            },
            &format!("{:.2}× profile", env.speed / limit),
        );
        widgets::stat_row(
            ui,
            "Command jitter · steer / pedal",
            &format!("{:.3} / {:.3}", env.steer_delta, env.throttle_delta),
            colors::MUTED,
        );
    }

    fn brain_section(&self, ui: &mut Ui, race: &RaceView, actions: &mut Vec<UiAction>) {
        let policy = match self.mode {
            Mode::Train => self.training.as_ref().and_then(|session| {
                session
                    .latest
                    .as_ref()
                    .map(|snapshot| snapshot.policy.as_ref())
            }),
            _ => self.policy.as_ref(),
        };
        if self.practice_selected() {
            ui.label("Select a race route to learn or watch the model. Your learning session is preserved.");
            return;
        }
        let Some(policy) = policy else {
            ui.label(
                RichText::new("Start learning to create a new driving model.")
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
        // The activation stack ends at the normalized trunk; the mean action and
        // critic value come from the output head, which it deliberately omits.
        let action = policy.mean_action(&race.env.obs);
        let (steer, throttle) = (action[0], action[1]);
        let value = policy.value(&race.env.obs);
        ui.label(
            RichText::new(format!(
                "output · steer {steer:+.2} · throttle {throttle:+.2} · value {value:+.2}"
            ))
            .size(11.0)
            .monospace()
            .color(colors::MUTED),
        );
    }

    fn session_section(&self, ui: &mut Ui, _actions: &mut Vec<UiAction>) {
        if self.practice_selected() {
            ui.label("Manual practice has no race results or lap times.");
            if !self.status.is_empty() {
                ui.label(&self.status);
            }
            return;
        }
        widgets::stat_row(
            ui,
            "Races finished",
            &format!("{}", self.finished),
            colors::GOOD,
        );
        widgets::stat_row(
            ui,
            "Races crashed",
            &format!("{}", self.crashed),
            colors::BAD,
        );
        if self.race.env.track.closed {
            widgets::stat_row(
                ui,
                "Session best lap",
                &self
                    .best_lap
                    .map(|lap| format!("{lap:.2}s"))
                    .unwrap_or_else(|| "—".to_string()),
                colors::GOOD,
            );
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
            widgets::section(
                ui,
                if self.practice_selected() {
                    "Practice"
                } else {
                    "Race"
                },
            );
            self.race_section(ui, self.active_race());

            widgets::section(ui, "Control");
            self.control_section(ui, &mut actions);

            if self.mode == Mode::Train {
                widgets::section(ui, "Training");
                self.training_section(ui);
            }

            widgets::section(ui, "Telemetry");
            self.telemetry_section(ui, self.active_race());

            widgets::section(ui, "Brain");
            self.brain_section(ui, self.active_race(), &mut actions);

            if self.mode != Mode::Train {
                widgets::section(ui, "Session");
                self.session_section(ui, &mut actions);
            }
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
        let drawing_speed = if self.mode == Mode::Manual {
            1.0
        } else {
            self.speed
        };
        let drawing_env = race.drawing_env(drawing_speed, self.paused);
        let view = if race.env.track.circuit.is_some() && !self.overview {
            View::follow(inner.shrink(6.0), drawing_env.position())
        } else {
            View::fit(inner.shrink(6.0), race.env.track.bounds, 34.0)
        };

        render::draw_track(&painter, &view, &race.env.track);
        if self.show_trail {
            render::draw_trail(&painter, &view, &race.trail);
        }
        render::draw_skids(&painter, &view, &race.skids);
        if self.show_rays {
            render::draw_rays(&painter, &view, &drawing_env);
        }
        if self.mode == Mode::Train {
            if let Some(session) = &self.training {
                for (i, car) in session.cars.iter().enumerate() {
                    if i == self.followed {
                        continue;
                    }
                    let mut ghost = painter.clone();
                    ghost.set_opacity(0.40);
                    render::draw_car_with_sprite(
                        &ghost,
                        &view,
                        &car.drawing_env(drawing_speed, self.paused),
                        car.env.off_track,
                        self.car_sprite.as_ref(),
                    );
                    painter.text(
                        view.pos(car.env.position()) + Vec2::new(0.0, -15.0),
                        Align2::CENTER_BOTTOM,
                        format!("{}", i + 1),
                        FontId::monospace(11.0),
                        colors::MUTED,
                    );
                }
            }
        }
        let crashed = matches!(race.outcome, Some(Outcome::Crashed { .. }));
        render::draw_car_with_sprite(
            &painter,
            &view,
            &drawing_env,
            crashed,
            self.car_sprite.as_ref(),
        );
        if self.mode == Mode::Train && self.overview {
            let pos = view.pos(race.env.position());
            painter.circle_stroke(pos, 5.0, Stroke::new(1.0, colors::CAR));
            painter.text(
                pos + Vec2::new(0.0, -8.0),
                Align2::CENTER_BOTTOM,
                format!("{}", self.followed + 1),
                FontId::monospace(11.0),
                colors::TEXT,
            );
        }

        if race.env.track.circuit.is_some() && !self.overview {
            let map_rect = egui::Rect::from_min_size(
                inner.right_bottom() - Vec2::new(222.0, 198.0),
                Vec2::new(210.0, 180.0),
            );
            render::draw_minimap(&painter, map_rect, &race.env);
            if self.mode == Mode::Train {
                if let Some(session) = &self.training {
                    let map_view = View::fit(map_rect.shrink(16.0), race.env.track.bounds, 20.0);
                    for (i, car) in session.cars.iter().enumerate() {
                        let pos = map_view.pos(car.env.position());
                        painter.circle_filled(
                            pos,
                            3.0,
                            if i == self.followed {
                                colors::CAR
                            } else {
                                colors::ACCENT
                            },
                        );
                    }
                }
            }
        }
        painter.text(
            inner.left_top() + Vec2::new(12.0, 10.0),
            Align2::LEFT_TOP,
            if self.practice_selected() {
                format!("PRACTICE AREA   {:.1}s", race.elapsed())
            } else if !race.env.track.closed {
                format!(
                    "ROUTE {:.1}%   {:.1}s",
                    if race.env.lap_done {
                        100.0
                    } else {
                        (race.env.distance / race.env.track.lap_length * 100.0).clamp(0.0, 100.0)
                    },
                    race.elapsed()
                )
            } else {
                format!(
                    "{} {} / {}   {:.1}s",
                    "LAP",
                    (race.laps_done + 1).min(race.env.laps),
                    race.env.laps,
                    race.elapsed()
                )
            },
            FontId::monospace(13.0),
            colors::TEXT,
        );
        painter.text(
            inner.right_top() + Vec2::new(-12.0, 10.0),
            Align2::RIGHT_TOP,
            format!(
                "{:.3} km · road {:.1} m",
                race.env.track.lap_length / UNITS_PER_METRE / 1000.0,
                race.env.road_width / UNITS_PER_METRE
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

        if let Some(outcome) = race.outcome.filter(|_| self.mode != Mode::Train) {
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

impl EframeApp for App {
    fn ui(&mut self, ui: &mut Ui, _frame: &mut eframe::Frame) {
        self.render(ui);
    }
}

impl App {
    fn frame(&mut self, ctx: &Context, ui: &mut Ui) {
        if self.car_sprite.is_none() {
            self.car_sprite = Some(
                render::CarSprite::original_m3(ctx)
                    .expect("validated existing GT4 M3 collection sprite"),
            );
        }

        let now = std::time::Instant::now();
        self.diagnostic_max_gap_ms = self
            .diagnostic_max_gap_ms
            .max(now.duration_since(self.diagnostic_last_frame).as_millis() as u64);
        self.diagnostic_last_frame = now;
        self.diagnostic_frames += 1;
        let dt = ctx.input(|input| input.stable_dt).min(0.1);
        self.handle_shortcuts(ctx);
        self.advance_race(&ctx, dt);
        self.poll_training();

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
                            "tab mode · space pause · R start over · N next track · V camera · T rays · P trail · 1-4 speed",
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

        // Ten-second monotonic windows; not every frame. A frozen renderer cannot report until it resumes.
        if self.diagnostic_last_report.elapsed().as_secs() >= 10 {
            emit!(
                "ui_health",
                0,
                json!({"frames":self.diagnostic_frames,"max_frame_gap_ms":self.diagnostic_max_gap_ms,
                "stale_revision_frames":self.diagnostic_stale_frames,"mode":format!("{:?}",self.mode),
                "session":self.training_stats().map(|s|s.diagnostic_id),
                "snapshot_age_ms":self.training_stats().map(|s|s.published_at.elapsed().as_millis() as u64),
                "worker_finished":self.training.as_ref().map(|s|s.worker.is_finished())})
            );
            self.diagnostic_frames = 0;
            self.diagnostic_max_gap_ms = 0;
            self.diagnostic_last_report = now;
        }
        ctx.request_repaint();
    }
}

/// Locates a saved policy: explicit path, then the repo's `models` directory.
pub fn find_policy(explicit: Option<PathBuf>) -> (Option<Policy>, PathBuf) {
    let lookup = neural_racing_diagnostics::operation();
    let _context = neural_racing_diagnostics::context(lookup);
    let explicit_supplied = explicit.is_some();
    let mut candidates: Vec<PathBuf> = Vec::new();
    if let Some(path) = explicit {
        candidates.push(path);
    }
    let repo_models = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../models");
    candidates.push(repo_models.join("policy.bin"));
    candidates.push(repo_models.join("best.bin"));
    candidates.push(PathBuf::from("models/policy.bin"));
    emit!(
        "watch_policy_lookup",
        lookup,
        json!({"explicit":explicit_supplied,"candidates":candidates.len()})
    );
    for (index, candidate) in candidates.iter().enumerate() {
        let attempt = neural_racing_diagnostics::operation();
        let _context = neural_racing_diagnostics::context(attempt);
        // try_exists retains permission errors without changing the previous fallback.
        let exists = candidate.try_exists();
        emit!(
            "watch_policy_candidate",
            attempt,
            json!({"parent":lookup,"candidate_index":index,"exists":exists.as_ref().ok(),
            "error_kind":exists.as_ref().err().map(|error|format!("{:?}",error.kind()))})
        );
        if exists.unwrap_or(false) {
            if let Ok(policy) = Policy::load(candidate) {
                if policy.obs_dim == neural_racing_core::OBS_DIM
                    && policy.width == DEFAULT_WIDTH
                    && policy.blocks == DEFAULT_BLOCKS
                {
                    emit!(
                        "watch_policy_selected",
                        attempt,
                        json!({"candidate_index":index,"obs_dim":policy.obs_dim})
                    );
                    return (Some(policy), candidate.clone());
                }
                emit!(
                    "watch_policy_incompatible",
                    attempt,
                    json!({"candidate_index":index,"obs_dim":policy.obs_dim,"width":policy.width,"blocks":policy.blocks})
                );
            }
        }
    }
    emit!(
        "watch_policy_unavailable",
        lookup,
        json!({"candidates":candidates.len()})
    );
    (None, candidates[0].clone())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    #[test]
    fn held_keyboard_turn_does_not_spin_and_manual_ignores_training_speed() {
        for key in [Key::ArrowLeft, Key::ArrowRight, Key::A, Key::D] {
            let mut app = App::new(None, PathBuf::new());
            app.configure(None, Some(Mode::Manual), Some(1000.0));
            let ctx = Context::default();
            for tick in 0..20 {
                let mut input = egui::RawInput::default();
                if tick == 0 {
                    for key in [key, Key::ArrowUp] {
                        input.events.push(egui::Event::Key {
                            key,
                            physical_key: None,
                            pressed: true,
                            repeat: false,
                            modifiers: egui::Modifiers::NONE,
                        });
                    }
                }
                let mut output = ctx.run_ui(input, |ui| {
                    app.advance_race(ui.ctx(), neural_racing_core::DECISION_DT as f32)
                });
                output.textures_delta.clear();
                let env = &app.race.env;
                let rear_slip = ((env.vy - neural_racing_core::WHEELBASE * 0.5 * env.yaw_rate)
                    / env.vx.abs().max(1.0))
                .atan()
                .to_degrees();
                assert!(
                    rear_slip.abs() < 12.0,
                    "{key:?} tick={tick}: rear slip={rear_slip}"
                );
                assert!(!app.race.is_over(), "{key:?} tick={tick}: left road");
            }
            assert_eq!(app.race.env.steps, 20, "manual stays at real time");
        }
    }

    fn wait_for(app: &mut App, predicate: impl Fn(&Snapshot) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            app.poll_training();
            if app.training_stats().is_some_and(&predicate) {
                return;
            }
            assert!(
                Instant::now() < deadline,
                "app did not receive expected session state; latest (decisions, updates, revision, paused, error) = {:?}",
                app.training_stats().map(|s|(s.decisions,s.updates,s.revision,s.paused,&s.error))
            );
            std::thread::sleep(Duration::from_millis(10));
        }
    }

    #[test]
    fn speed_slider_and_max_mode_do_not_silently_clamp_to_1000() {
        let mut app = App::new(None, PathBuf::new());
        app.configure(None, None, Some(1500.0));
        assert_eq!(app.speed, 1500.0);
        assert!(!app.maximum_speed);
        app.configure(None, None, Some(f32::INFINITY));
        assert!(app.maximum_speed);
        assert_eq!(app.speed, MAX_REQUESTED_SPEED as f32);
        app.configure(None, None, Some(0.25));
        assert!(!app.maximum_speed);
        assert_eq!(app.speed, 0.25);
        app.configure(None, None, Some(f32::NAN));
        assert_eq!(app.speed, 1.0);
    }

    #[test]
    fn controls_target_learning_and_manual_does_not_discard_the_session() {
        let mut app = App::new(None, PathBuf::new());
        assert!(app.mode == Mode::Train && app.speed == 1.0 && app.policy.is_none());
        app.configure(None, None, Some(1000.0));
        wait_for(&mut app, |s| s.updates >= 2);
        app.apply(UiAction::TogglePause);
        wait_for(&mut app, |s| s.paused);
        let weights = app.training_stats().unwrap().policy.params().to_vec();
        let decisions = app.training_stats().unwrap().decisions;
        let old_position = app.race().env.position();
        app.apply(UiAction::SetMode(Mode::Manual));
        app.apply(UiAction::Restart);
        app.apply(UiAction::SetMode(Mode::Train));
        assert_eq!(app.race().env.position(), old_position);
        assert_eq!(app.training_stats().unwrap().decisions, decisions);
        app.apply(UiAction::SelectCircuit(Some(Circuit::Silverstone)));
        wait_for(&mut app, |s| {
            s.paused && s.cars[0].track.circuit == Some(Circuit::Silverstone)
        });
        assert_eq!(app.training_stats().unwrap().policy.params(), weights);
        app.apply(UiAction::Follow(3));
        assert_eq!(
            app.race().env.position(),
            app.training_stats().unwrap().cars[3].position()
        );
        app.apply(UiAction::Restart);
        wait_for(&mut app, |s| s.paused && s.decisions == 0);
        assert_eq!(app.training_stats().unwrap().episodes, 0);
        assert_ne!(app.training_stats().unwrap().policy.params(), weights);
        app.apply(UiAction::TogglePause);
        wait_for(&mut app, |s| s.decisions > 0);
        app.apply(UiAction::SetMode(Mode::Manual));
        wait_for(&mut app, |s| s.paused);
        let stopped = app.training_stats().unwrap().decisions;
        std::thread::sleep(Duration::from_millis(50));
        app.poll_training();
        assert_eq!(app.training_stats().unwrap().decisions, stopped);
        app.apply(UiAction::SetMode(Mode::Train));
        wait_for(&mut app, |s| s.decisions > stopped);
    }
    #[test]
    fn practice_area_preserves_the_paused_learner_and_rejects_race_modes() {
        let area = *Circuit::all()
            .iter()
            .find(|c| c.is_practice())
            .expect("published practice geometry");
        let mut app = App::new(None, PathBuf::new());
        app.paused = true;
        wait_for(&mut app, |s| s.paused);
        let weights = app.training_stats().unwrap().policy.params().to_vec();
        let decisions = app.training_stats().unwrap().decisions;
        let training_track = app.training_stats().unwrap().cars[0].track.circuit;
        app.select_circuit(Some(area));
        assert_eq!(app.mode, Mode::Manual);
        assert!(app.paused);
        assert!(app.race.env.track.free_roam);
        for mode in [Mode::Train, Mode::Watch] {
            app.apply(UiAction::SetMode(mode));
            assert_eq!(app.mode, Mode::Manual);
        }
        wait_for(&mut app, |s| s.paused);
        let snapshot = app.training_stats().unwrap();
        assert_eq!(snapshot.decisions, decisions);
        assert_eq!(snapshot.policy.params(), weights);
        assert_eq!(snapshot.cars[0].track.circuit, training_track);
    }
    #[test]
    fn open_route_training_hud_uses_route_goals_without_lap_or_stint_claims() {
        let mut app = App::new(None, PathBuf::new());
        app.paused = true;
        app.select_circuit(Some(Circuit::lookup("gt4-testcourse_100m").unwrap()));
        wait_for(&mut app, |s| s.paused && !s.closed_route);
        let ctx = egui::Context::default();
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| app.training_section(ui));
        output.textures_delta.clear();
        let text = output
            .shapes
            .iter()
            .filter_map(|s| match &s.shape {
                egui::Shape::Text(t) => Some(t.galley.job.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(text.contains("Completed routes"), "{text}");
        assert!(text.contains("Full routes"), "{text}");
        assert!(!text.contains("Five-lap"), "{text}");
        assert!(!text.contains("Completed laps"), "{text}");
    }
    #[test]
    fn road_width_hud_uses_both_sides_not_twice_the_nearest_edge() {
        let mut app = App::new(None, PathBuf::new());
        app.race.env.road_width = 20.0 * UNITS_PER_METRE;
        app.race.env.road_half_width = 3.0 * UNITS_PER_METRE;
        app.race.env.min_ray = 7.0 * UNITS_PER_METRE;
        let ctx = egui::Context::default();
        let mut output = ctx.run_ui(egui::RawInput::default(), |ui| {
            app.telemetry_section(ui, &app.race)
        });
        output.textures_delta.clear();
        let text = output
            .shapes
            .iter()
            .filter_map(|s| match &s.shape {
                egui::Shape::Text(t) => Some(t.galley.job.text.as_str()),
                _ => None,
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(text.contains("20.0 m"), "{text}");
        assert!(!text.contains("6.0 m"), "{text}");
    }
}
