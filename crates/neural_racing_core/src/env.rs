//! Four-contact-patch car stepping around a [`Track`], one decision at a time.

use crate::track::Track;
use crate::{
    py_mod, wrap_angle, Vec2, BASE_HALF_WIDTH, CHECKPOINT_BONUS, CHECKPOINT_COUNT, DECISION_DT,
    FAST_LAP_BONUS, LAP_BONUS, MAX_LATERAL_ACCEL, MAX_SPEED, MAX_STEER_ANGLE, MAX_STEPS, NUM_RAYS,
    OBS_DIM, OFF_TRACK_PENALTY, PREVIEW_CURVE, PREVIEW_FAR, PREVIEW_NEAR, PROFILE_MIN_SPEED,
    PROGRESS_WEIGHT, RAY_RANGE, SPEED_EXCESS_PENALTY, SPEED_SHORTFALL_PENALTY, START_SPEED,
    STATE_FEATURES, SUBSTEPS, TIMEOUT_PENALTY, TIME_COST, WALL_PENALTY, WALL_WARN_FRACTION,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Action {
    pub steer: f64,
    pub throttle: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct RewardComponents {
    pub progress: f64,
    pub slip: f64,
    pub time: f64,
    pub wall: f64,
    pub off_track: f64,
    pub excess_speed: f64,
    /// Payback for staying below the safe speed profile: leaving grip unused.
    pub speed_shortfall: f64,
    pub checkpoint: f64,
    pub lap: f64,
    pub smoothness: f64,
    pub stability: f64,
    pub endurance: f64,
    pub timeout: f64,
}

impl RewardComponents {
    pub fn total(&self) -> f64 {
        self.progress
            + self.slip
            + self.time
            + self.wall
            + self.off_track
            + self.excess_speed
            + self.speed_shortfall
            + self.checkpoint
            + self.lap
            + self.smoothness
            + self.stability
            + self.endurance
            + self.timeout
    }
}

#[derive(Clone, Copy, Debug)]
pub struct StepInfo {
    pub distance: f64,
    pub off_track: bool,
    pub lap_done: bool,
    pub steps: i32,
    pub checkpoints: i32,
    pub laps_completed: i32,
    pub min_ray: f64,
    pub slip_deg: f64,
    pub reward_components: RewardComponents,
}

#[derive(Clone, Copy, Debug)]
pub struct StepOutcome {
    pub reward: f64,
    pub done: bool,
    pub info: StepInfo,
}

#[derive(Clone)]
pub struct Env {
    pub track: Track,
    pub laps: i32,
    pub x: f64,
    pub y: f64,
    pub heading: f64,
    pub vx: f64,
    pub vy: f64,
    pub yaw_rate: f64,
    pub steer_angle: f64,
    pub steer_rate: f64,
    pub pedal: f64,
    pub steer_target: f64,
    pub lateral_accel: f64,
    pub yaw_accel: f64,
    pub lateral_load_transfer: f64,
    /// Per-wheel transient lateral force, normal load, and combined grip usage.
    /// Order: front left, front right, rear left, rear right.
    pub tyre_lateral: [f64; 4],
    pub tyre_loads: [f64; 4],
    pub tyre_usage: [f64; 4],
    /// Body longitudinal acceleration in m/s², used for axle load transfer.
    pub longitudinal_accel: f64,
    pub slip_angle_deg: f64,
    pub lateral_offset: f64,
    /// Half the road width under the car, refreshed every decision.
    pub road_half_width: f64,
    pub speed: f64,
    /// Braking-distance-aware limit at the car's current arc.
    pub speed_profile: f64,
    /// Lateral velocity in the road frame.
    pub lateral_velocity: f64,
    /// Command change on the most recent decision, for jitter telemetry.
    pub steer_delta: f64,
    pub throttle_delta: f64,
    pub heading_error: f64,
    pub steps: i32,
    pub next_checkpoint: i32,
    pub laps_completed: i32,
    pub distance: f64,
    pub min_ray: f64,
    pub off_track: bool,
    pub lap_done: bool,
    pub last_reward: f64,
    /// Steering and throttle commanded on the most recent decision.
    pub last_action: Action,
    /// Command from the decision before `last_action`.
    prev_action: Action,
    pub rays: [f64; NUM_RAYS],
    /// Actual integration positions from the most recent control interval.
    pub path: [Vec2; SUBSTEPS],
    /// Observation of the current state: state features then normalized rays.
    pub obs: [f32; OBS_DIM],
    prev_arc: f64,
    start_gate: usize,
    paid_checkpoint: i32,
    ray_angles: [f64; NUM_RAYS],
}

impl Env {
    /// A fresh race: `laps` laps on `track`.
    pub fn new(track: Track, laps: i32) -> Self {
        assert!(laps >= 1, "a race needs at least one lap");
        let mut ray_angles = [0.0; NUM_RAYS];
        for (index, angle) in ray_angles.iter_mut().enumerate() {
            let t = index as f64 / (NUM_RAYS - 1) as f64;
            *angle = -0.6 * std::f64::consts::PI + 1.2 * std::f64::consts::PI * t;
        }
        let mut env = Self {
            track,
            laps,
            x: 0.0,
            y: 0.0,
            heading: 0.0,
            vx: 0.0,
            vy: 0.0,
            yaw_rate: 0.0,
            steer_angle: 0.0,
            steer_rate: 0.0,
            pedal: 0.0,
            steer_target: 0.0,
            lateral_accel: 0.0,
            yaw_accel: 0.0,
            lateral_load_transfer: 0.0,
            tyre_lateral: [0.0; 4],
            tyre_loads: [0.0; 4],
            tyre_usage: [0.0; 4],
            longitudinal_accel: 0.0,
            slip_angle_deg: 0.0,
            lateral_offset: 0.0,
            road_half_width: 0.0,
            speed: 0.0,
            speed_profile: START_SPEED,
            lateral_velocity: 0.0,
            steer_delta: 0.0,
            throttle_delta: 0.0,
            heading_error: 0.0,
            steps: 0,
            next_checkpoint: 0,
            laps_completed: 0,
            distance: 0.0,
            min_ray: RAY_RANGE,
            off_track: false,
            lap_done: false,
            last_reward: 0.0,
            last_action: Action {
                steer: 0.0,
                throttle: 0.0,
            },
            prev_action: Action {
                steer: 0.0,
                throttle: 0.0,
            },
            rays: [RAY_RANGE; NUM_RAYS],
            path: [Vec2 { x: 0.0, y: 0.0 }; SUBSTEPS],
            obs: [0.0; OBS_DIM],
            prev_arc: 0.0,
            start_gate: 0,
            paid_checkpoint: 0,
            ray_angles,
        };
        env.reset();
        env
    }

    pub fn total_gates(&self) -> i32 {
        CHECKPOINT_COUNT as i32 * self.laps
    }

    /// Real circuits allow the configured laps at a beginner average 10 m/s,
    /// plus 90 seconds for braking and recovery. Procedural training retains its original budget.
    pub fn max_steps(&self) -> i32 {
        if self.track.circuit.is_some() {
            ((self.track.lap_length * self.laps as f64 / (10.0 * crate::UNITS_PER_METRE) + 90.0)
                / crate::DECISION_DT)
                .ceil() as i32
        } else {
            MAX_STEPS * self.laps.max(crate::DEFAULT_LAPS) / crate::DEFAULT_LAPS
        }
    }

    /// Unique completed laps, unaffected by reversing over a paid gate.
    pub fn credited_laps(&self) -> i32 {
        self.paid_checkpoint / CHECKPOINT_COUNT as i32
    }

    pub fn position(&self) -> Vec2 {
        Vec2 {
            x: self.x,
            y: self.y,
        }
    }

    /// Start a full-lap practice attempt at an existing checkpoint. The gate
    /// sequence rotates with the start, so completing it still requires one
    /// entire circuit, never a shortcut or a partial lap.
    pub fn reset_at_checkpoint(&mut self, checkpoint: usize) {
        self.start_gate = checkpoint % CHECKPOINT_COUNT;
        self.reset();
    }

    pub fn reset(&mut self) {
        let arc = self.track.lap_length * self.start_gate as f64 / CHECKPOINT_COUNT as f64;
        let start = self.track.arc_point(arc);
        self.x = start.x;
        self.y = start.y;
        self.heading = self.track.arc_tangent(arc).angle();
        self.vx = START_SPEED;
        self.vy = 0.0;
        self.yaw_rate = 0.0;
        self.steer_angle = 0.0;
        self.steer_rate = 0.0;
        self.pedal = 0.0;
        self.steer_target = 0.0;
        self.lateral_accel = 0.0;
        self.yaw_accel = 0.0;
        self.lateral_load_transfer = 0.0;
        self.tyre_lateral = [0.0; 4];
        self.tyre_loads = [crate::vehicle::MASS_KG * crate::vehicle::GRAVITY / 4.0; 4];
        self.tyre_usage = [0.0; 4];
        self.paid_checkpoint = 0;
        self.longitudinal_accel = 0.0;
        self.slip_angle_deg = 0.0;
        self.lateral_offset = 0.0;
        self.road_half_width = 0.0;
        self.speed = START_SPEED;
        self.speed_profile = START_SPEED;
        self.lateral_velocity = 0.0;
        self.steer_delta = 0.0;
        self.throttle_delta = 0.0;
        self.heading_error = 0.0;
        self.steps = 0;
        self.next_checkpoint = 0;
        self.laps_completed = 0;
        self.distance = 0.0;
        self.min_ray = RAY_RANGE;
        self.off_track = false;
        self.lap_done = false;
        self.last_reward = 0.0;
        self.last_action = Action {
            steer: 0.0,
            throttle: 0.0,
        };
        self.prev_action = Action {
            steer: 0.0,
            throttle: 0.0,
        };
        self.rays = [RAY_RANGE; NUM_RAYS];
        self.path = [self.position(); SUBSTEPS];
        let hit = self.sense();
        self.prev_arc = hit.arc;
    }

    /// Advances one decision. Calling `step` after the race is over returns a
    /// zero-reward, `done` outcome instead of advancing state.
    pub fn step(&mut self, action: Action) -> StepOutcome {
        if self.lap_done || self.off_track || self.steps >= self.max_steps() {
            return self.terminal_outcome();
        }
        let previous = self.position();
        let steer_cmd = action.steer.clamp(-1.0, 1.0);
        let throttle_cmd = action.throttle.clamp(-1.0, 1.0);
        let previous_action = self.last_action;
        let before_previous_action = self.prev_action;
        let previous_lateral_accel = self.lateral_accel;
        self.prev_action = previous_action;
        self.last_action = Action {
            steer: steer_cmd,
            throttle: throttle_cmd,
        };
        let steer_step = steer_cmd - previous_action.steer;
        let throttle_step = throttle_cmd - previous_action.throttle;
        let previous_steer_step = previous_action.steer - before_previous_action.steer;
        let previous_throttle_step = previous_action.throttle - before_previous_action.throttle;
        self.steer_delta = steer_step;
        self.throttle_delta = throttle_step;

        for i in 0..SUBSTEPS {
            self.integrate_vehicle(steer_cmd, throttle_cmd);
            self.path[i] = self.position();
        }

        self.speed = self.vx.hypot(self.vy);
        self.slip_angle_deg = self.vy.atan2(self.vx.abs().max(1.0)).to_degrees();
        self.steps += 1;

        let pos = self.position();
        let nearest = self.sense();
        let cos_h = self.heading.cos();
        let sin_h = self.heading.sin();
        let world_vx = self.vx * cos_h - self.vy * sin_h;
        let world_vy = self.vx * sin_h + self.vy * cos_h;

        let forward = world_vx * nearest.tangent.x + world_vy * nearest.tangent.y;
        let mut components = RewardComponents {
            // Distance advanced along the track, in metres, scaled to keep the
            // return small enough to regress with a Huber value loss.
            progress: forward * DECISION_DT / crate::UNITS_PER_METRE * PROGRESS_WEIGHT,
            slip: -0.02 * (self.vy.abs() / crate::UNITS_PER_METRE) * DECISION_DT,
            // A race rewards forward motion. Parking before a corner must not
            // beat attempting it; retain room for very slow hairpin recovery.
            time: -TIME_COST
                - 0.05 * (1.0 - forward / (3.0 * crate::UNITS_PER_METRE)).clamp(0.0, 1.0),
            // First and second command differences: a steady steering ramp is
            // free, while high-frequency corrections are expensive.
            smoothness: -0.10 * steer_step.powi(2)
                - 0.03 * throttle_step.powi(2)
                - 0.05 * (steer_step - previous_steer_step).powi(2)
                - 0.02 * (throttle_step - previous_throttle_step).powi(2)
                - 0.002
                    * (((self.lateral_accel - previous_lateral_accel).abs() / DECISION_DT - 8.0)
                        .max(0.0)
                        / 20.0)
                        .min(2.0)
                        .powi(2),
            stability: -0.02
                * ((self.slip_angle_deg.to_radians().abs() - 0.06).max(0.0) / 0.15)
                    .min(2.0)
                    .powi(2),
            ..RewardComponents::default()
        };
        let mut reward = components.total();

        let warn_dist = WALL_WARN_FRACTION * nearest.half_width;
        if self.min_ray < warn_dist {
            let closeness = (warn_dist - self.min_ray) / warn_dist;
            components.wall = -WALL_PENALTY * closeness * closeness;
            reward += components.wall;
        }

        // Above the braking-distance-aware profile the car cannot stop for the
        // next corner. Below it this term is silent, so a straight-line throttle
        // application is never punished.
        let limit = self
            .speed_profile
            .max(PROFILE_MIN_SPEED * crate::UNITS_PER_METRE);
        let excess = (self.speed / limit - 1.0).clamp(0.0, 2.0);
        if excess > 0.0 {
            components.excess_speed = -SPEED_EXCESS_PENALTY * excess;
            reward += components.excess_speed;
        }
        // Below the profile the car is leaving grip unused. With only the cap
        // above, a policy that has never survived the first corner learns that
        // crawling minimises crash frequency and parks the throttle; paying for
        // the deficit keeps acceleration on the gradient.
        let deficit = (1.0 - self.speed / limit).clamp(0.0, 1.0);
        if deficit > 0.0 {
            components.speed_shortfall = -SPEED_SHORTFALL_PENALTY * deficit;
            reward += components.speed_shortfall;
        }

        let half_lap = self.track.lap_length / 2.0;
        let delta = py_mod(
            nearest.arc - self.prev_arc + half_lap,
            self.track.lap_length,
        ) - half_lap;
        if delta.abs() < 200.0 {
            self.distance += delta;
        }
        self.prev_arc = nearest.arc;

        let gate_count = CHECKPOINT_COUNT as i32;
        let done = if nearest.lateral.abs() > nearest.half_width {
            self.off_track = true;
            components.off_track = -OFF_TRACK_PENALTY;
            reward += components.off_track;
            true
        } else {
            if self.next_checkpoint > 0
                && self.track.checkpoint_crossing(
                    ((self.next_checkpoint - 1 + self.start_gate as i32) % gate_count) as usize,
                    previous,
                    pos,
                ) == -1
            {
                self.next_checkpoint -= 1;
                self.laps_completed = self.next_checkpoint / gate_count;
            }
            if self.track.checkpoint_crossing(
                ((self.next_checkpoint + self.start_gate as i32) % gate_count) as usize,
                previous,
                pos,
            ) == 1
            {
                self.next_checkpoint += 1;
                self.laps_completed = self.next_checkpoint / gate_count;
                if self.next_checkpoint > self.paid_checkpoint {
                    self.paid_checkpoint = self.next_checkpoint;
                    components.checkpoint += CHECKPOINT_BONUS;
                    reward += CHECKPOINT_BONUS;
                    if self.next_checkpoint % gate_count == 0 {
                        components.lap += LAP_BONUS;
                        // Later consecutive laps are worth more; a full five-lap
                        // stint earns a separate finish bonus. Never repay a gate.
                        components.endurance += 3.0 * (self.laps_completed - 1) as f64;
                        if self.laps_completed == self.laps && self.laps >= 5 {
                            components.endurance += 30.0;
                        }
                        reward += LAP_BONUS + components.endurance;
                    }
                }
            }
            if self.next_checkpoint == self.total_gates() {
                self.lap_done = true;
                let pace = (1.0 - self.steps as f64 / self.max_steps() as f64).max(0.0);
                components.lap += FAST_LAP_BONUS * pace;
                reward += FAST_LAP_BONUS * pace;
                true
            } else if self.steps >= self.max_steps() {
                // Running out of time must never beat crashing: otherwise
                // parking in front of a corner is the return-maximising move.
                components.timeout = -TIMEOUT_PENALTY;
                reward += components.timeout;
                true
            } else {
                false
            }
        };

        self.obs[25] = self.laps_completed as f32 / self.laps as f32;
        self.last_reward = reward;
        StepOutcome {
            reward,
            done,
            info: StepInfo {
                distance: self.distance.max(0.0),
                off_track: self.off_track,
                lap_done: self.lap_done,
                steps: self.steps,
                checkpoints: self.next_checkpoint,
                laps_completed: self.laps_completed,
                min_ray: self.min_ray,
                slip_deg: self.slip_angle_deg,
                reward_components: components,
            },
        }
    }

    fn terminal_outcome(&self) -> StepOutcome {
        StepOutcome {
            reward: 0.0,
            done: true,
            info: StepInfo {
                distance: self.distance.max(0.0),
                off_track: self.off_track,
                lap_done: self.lap_done,
                steps: self.steps,
                checkpoints: self.next_checkpoint,
                laps_completed: self.laps_completed,
                min_ray: self.min_ray,
                slip_deg: self.slip_angle_deg,
                reward_components: RewardComponents::default(),
            },
        }
    }

    /// Refreshes rays, centerline error and the observation vector.
    fn sense(&mut self) -> crate::track::CenterlineHit {
        let pos = self.position();
        let hit = self.track.nearest_on_centerline(pos);
        let mut angles = [0.0f64; NUM_RAYS];
        for (slot, relative) in angles.iter_mut().zip(&self.ray_angles) {
            *slot = self.heading + relative;
        }
        self.track.cast_rays_into(pos, &angles, &mut self.rays);
        self.min_ray = self.rays.iter().copied().fold(RAY_RANGE, f64::min);
        self.lateral_offset = hit.lateral;
        self.road_half_width = hit.half_width;
        self.heading_error = wrap_angle(self.heading - hit.tangent.angle());
        self.speed_profile = self.track.speed_profile(hit.arc);
        let (sin_h, cos_h) = self.heading.sin_cos();
        let world_vx = self.vx * cos_h - self.vy * sin_h;
        let world_vy = self.vx * sin_h + self.vy * cos_h;
        self.lateral_velocity = world_vx * -hit.tangent.y + world_vy * hit.tangent.x;

        self.obs[0] = (self.vx / MAX_SPEED) as f32;
        self.obs[1] = (self.vy / 150.0).clamp(-1.0, 1.0) as f32;
        self.obs[2] = (self.yaw_rate / 4.0).clamp(-1.0, 1.0) as f32;
        self.obs[3] = (self.steer_angle / MAX_STEER_ANGLE) as f32;
        self.obs[4] = (hit.lateral / hit.half_width).clamp(-1.0, 1.0) as f32;
        self.obs[5] = self.heading_error.sin() as f32;
        self.obs[6] = self.heading_error.cos() as f32;
        self.obs[7] = (self.speed / MAX_SPEED) as f32;
        // Road preview: where the centerline and the walls go next.
        let preview = |arc: f64| -> (Vec2, f64) {
            let point = self.track.arc_point(arc);
            let tangent = self.track.arc_tangent(arc);
            let normal = Vec2 {
                x: -tangent.y,
                y: tangent.x,
            };
            let offset = (pos - point).dot(normal) / self.track.arc_half_width(arc);
            (normal, offset)
        };
        let (_, near_offset) = preview(hit.arc + PREVIEW_NEAR);
        let (_, far_offset) = preview(hit.arc + PREVIEW_FAR);
        let turn = wrap_angle(
            self.track.arc_tangent(hit.arc + PREVIEW_CURVE).angle() - hit.tangent.angle(),
        );
        self.obs[8] = near_offset.clamp(-2.0, 2.0) as f32;
        self.obs[9] =
            (self.track.arc_half_width(hit.arc + PREVIEW_NEAR) / BASE_HALF_WIDTH).min(2.0) as f32;
        self.obs[10] = (turn / std::f64::consts::PI).clamp(-1.0, 1.0) as f32;
        self.obs[11] =
            (self.track.arc_half_width(hit.arc + PREVIEW_FAR) / BASE_HALF_WIDTH).min(2.0) as f32;
        // How tight the next 60 m is: 1 is a straight, 0 is the tightest bend.
        let corner = self
            .track
            .tightest_corner_ahead(hit.arc, 60.0 * crate::UNITS_PER_METRE);
        self.obs[12] = if corner.is_finite() {
            (corner / (200.0 * crate::UNITS_PER_METRE)).min(1.0) as f32
        } else {
            1.0
        };
        self.obs[13] = (self.steer_rate / crate::vehicle::MAX_STEER_RATE).clamp(-1.0, 1.0) as f32;
        self.obs[14] = self.pedal as f32;
        self.obs[15] = (self.steer_target / MAX_STEER_ANGLE) as f32;
        self.obs[16] = self.last_action.steer as f32;
        self.obs[17] = self.last_action.throttle as f32;
        self.obs[18] = (self.longitudinal_accel / 15.0).clamp(-2.0, 2.0) as f32;
        self.obs[19] = (self.lateral_accel / 15.0).clamp(-2.0, 2.0) as f32;
        self.obs[20] = (self.yaw_accel / 10.0).clamp(-2.0, 2.0) as f32;
        self.obs[21] = (self.slip_angle_deg.to_radians() / 0.35).clamp(-2.0, 2.0) as f32;
        self.obs[22] = ((self.tyre_usage[0] + self.tyre_usage[1]) * 0.5) as f32;
        self.obs[23] = ((self.tyre_usage[2] + self.tyre_usage[3]) * 0.5) as f32;
        let weight = crate::vehicle::MASS_KG * crate::vehicle::GRAVITY;
        self.obs[24] = (self.lateral_load_transfer / weight) as f32;
        self.obs[25] = self.laps_completed as f32 / self.laps as f32;
        for (i, metres) in [10.0, 25.0, 50.0, 100.0, 200.0].into_iter().enumerate() {
            self.obs[26 + i] = (wrap_angle(
                self.track
                    .arc_tangent(hit.arc + metres * crate::UNITS_PER_METRE)
                    .angle()
                    - self.heading,
            ) / std::f64::consts::PI) as f32;
        }
        let speed_mps = self.speed / crate::UNITS_PER_METRE;
        self.obs[31] = (speed_mps.powi(2) / (2.0 * 8.0 * 300.0)).min(2.0) as f32;
        self.obs[32] = ((self.tyre_loads[0] + self.tyre_loads[1]) / weight) as f32;
        self.obs[33] = ((self.tyre_loads[2] + self.tyre_loads[3]) / weight) as f32;
        self.obs[34] = far_offset.clamp(-2.0, 2.0) as f32;
        let road_yaw = turn / PREVIEW_CURVE * self.vx;
        self.obs[35] = ((self.yaw_rate - road_yaw) / 2.0).clamp(-2.0, 2.0) as f32;
        // Braking-distance-aware profile: what the car may do now, and at two
        // previews. Positive margin means the car is carrying too much speed.
        let profile = self.speed_profile;
        self.obs[36] = (profile / MAX_SPEED) as f32;
        self.obs[37] = ((self.speed - profile) / MAX_SPEED).clamp(-1.0, 1.0) as f32;
        self.obs[38] = (self
            .track
            .speed_profile(hit.arc + 100.0 * crate::UNITS_PER_METRE)
            / MAX_SPEED) as f32;
        self.obs[39] = (self
            .track
            .speed_profile(hit.arc + 300.0 * crate::UNITS_PER_METRE)
            / MAX_SPEED) as f32;
        let nominal = weight * 0.25;
        for wheel in 0..4 {
            self.obs[40 + wheel] = self.tyre_usage[wheel] as f32;
            self.obs[44 + wheel] = (self.tyre_loads[wheel] / nominal - 1.0).clamp(-1.0, 1.0) as f32;
        }
        self.obs[48] =
            (self.lateral_velocity / (30.0 * crate::UNITS_PER_METRE)).clamp(-2.0, 2.0) as f32;
        self.obs[49] = (((self.tyre_lateral[0] + self.tyre_lateral[1]) / weight) / 1.5)
            .clamp(-1.0, 1.0) as f32;
        self.obs[50] = (((self.tyre_lateral[2] + self.tyre_lateral[3]) / weight) / 1.5)
            .clamp(-1.0, 1.0) as f32;
        self.obs[51] = (self.steer_delta / 2.0).clamp(-1.0, 1.0) as f32;
        self.obs[52] = (self.throttle_delta / 2.0).clamp(-1.0, 1.0) as f32;
        self.obs[53] = (self.steps as f64 / self.max_steps().max(1) as f64) as f32;
        self.obs[54] =
            ((self.yaw_rate * self.speed / MAX_LATERAL_ACCEL) / 2.0).clamp(-1.0, 1.0) as f32;
        self.obs[55] =
            (1.0 - self.tyre_usage.iter().copied().fold(0.0f64, f64::max)).clamp(0.0, 1.0) as f32;
        // Square-root compression: on a road 40 units wide, a 320-unit sensor
        // range would quantize the wall into the bottom 12% of the input.
        for (slot, ray) in self.obs[STATE_FEATURES..].iter_mut().zip(&self.rays) {
            *slot = (*ray / RAY_RANGE).sqrt() as f32;
        }
        hit
    }
}

#[cfg(test)]
mod endurance_tests {
    use super::*;
    fn cross(env: &mut Env, gate: usize, forward: bool) -> StepOutcome {
        let g = env.track.gates()[gate];
        let tangent = env
            .track
            .arc_tangent((gate + 1) as f64 * env.track.lap_length / CHECKPOINT_COUNT as f64);
        let direction = if forward { 1.0 } else { -1.0 };
        let position = g.center - tangent * (direction * 0.05 * crate::UNITS_PER_METRE);
        env.x = position.x;
        env.y = position.y;
        env.heading = tangent.angle() + if forward { 0.0 } else { std::f64::consts::PI };
        env.vx = 5.0 * crate::UNITS_PER_METRE;
        env.vy = 0.0;
        env.yaw_rate = 0.0;
        env.step(Action {
            steer: 0.0,
            throttle: 0.0,
        })
    }
    #[test]
    fn five_lap_goal_increases_incentive_and_cannot_farm_a_gate() {
        let mut env = Env::new(crate::Circuit::Monza.build(), 5);
        let first = cross(&mut env, 0, true);
        assert!(first.info.reward_components.checkpoint > 0.0);
        cross(&mut env, 0, false);
        let replay = cross(&mut env, 0, true);
        assert_eq!(replay.info.reward_components.checkpoint, 0.0);
        let mut bonuses = Vec::new();
        for gate in 1..40 {
            let result = cross(&mut env, gate % CHECKPOINT_COUNT, true);
            assert_eq!(
                result.done,
                gate == 39,
                "must continue after laps one through four"
            );
            if gate % CHECKPOINT_COUNT == 7 {
                bonuses.push(
                    result.info.reward_components.lap + result.info.reward_components.endurance,
                );
            }
            assert!((result.reward - result.info.reward_components.total()).abs() < 1e-9);
        }
        assert_eq!(env.credited_laps(), 5);
        assert!(bonuses.windows(2).all(|b| b[1] > b[0]));
        assert!(env.lap_done);
        assert_eq!(
            env.step(Action {
                steer: 0.0,
                throttle: 0.0
            })
            .reward,
            0.0
        );
    }
}
