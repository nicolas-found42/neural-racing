//! Bicycle-model car stepping around a [`Track`], one decision at a time.

use crate::track::Track;
use crate::{
    py_mod, wrap_angle, Vec2, AERO_DRAG, BRAKE_DECEL, CHECKPOINT_COUNT, CORNER_STIFF_F,
    CHECKPOINT_BONUS, CORNER_STIFF_R, DECISION_DT, ENGINE_ACCEL, FAST_LAP_BONUS, LAP_BONUS,
    BASE_HALF_WIDTH, MAX_LATERAL_ACCEL, MAX_SPEED, MAX_STEER_ANGLE, MAX_STEPS, NUM_RAYS, OBS_DIM,
    STATE_FEATURES,
    OFF_TRACK_PENALTY, PHYSICS_DT, PREVIEW_CORNER, PREVIEW_CURVE, PREVIEW_FAR, PREVIEW_NEAR, RAY_RANGE,
    SPEED_EXCESS_PENALTY,
    REVERSE_ACCEL, ROLLING_DRAG, START_SPEED,
    STEER_SERVO_RATE, SUBSTEPS, TIME_COST, WALL_PENALTY, WALL_WARN_FRACTION, WHEELBASE,
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
    pub checkpoint: f64,
    pub lap: f64,
}

impl RewardComponents {
    pub fn total(&self) -> f64 {
        self.progress
            + self.slip
            + self.time
            + self.wall
            + self.off_track
            + self.excess_speed
            + self.checkpoint
            + self.lap
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
    pub slip_angle_deg: f64,
    pub lateral_offset: f64,
    /// Half the road width under the car, refreshed every decision.
    pub road_half_width: f64,
    pub speed: f64,
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
    pub rays: [f64; NUM_RAYS],
    /// Observation of the current state: state features then normalized rays.
    pub obs: [f32; OBS_DIM],
    prev_arc: f64,
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
            slip_angle_deg: 0.0,
            lateral_offset: 0.0,
            road_half_width: 0.0,
            speed: 0.0,
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
            rays: [RAY_RANGE; NUM_RAYS],
            obs: [0.0; OBS_DIM],
            prev_arc: 0.0,
            ray_angles,
        };
        env.reset();
        env
    }

    pub fn total_gates(&self) -> i32 {
        CHECKPOINT_COUNT as i32 * self.laps
    }

    pub fn position(&self) -> Vec2 {
        Vec2 {
            x: self.x,
            y: self.y,
        }
    }

    pub fn reset(&mut self) {
        self.x = self.track.start.x;
        self.y = self.track.start.y;
        self.heading = self.track.start_heading;
        self.vx = START_SPEED;
        self.vy = 0.0;
        self.yaw_rate = 0.0;
        self.steer_angle = 0.0;
        self.slip_angle_deg = 0.0;
        self.lateral_offset = 0.0;
        self.road_half_width = 0.0;
        self.speed = START_SPEED;
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
        self.rays = [RAY_RANGE; NUM_RAYS];
        let hit = self.sense();
        self.prev_arc = hit.arc;
    }

    /// Advances one decision. Calling `step` after the race is over returns a
    /// zero-reward, `done` outcome instead of advancing state.
    pub fn step(&mut self, action: Action) -> StepOutcome {
        if self.lap_done {
            return self.terminal_outcome();
        }
        let previous = self.position();
        let steer_cmd = action.steer.clamp(-1.0, 1.0);
        let throttle_cmd = action.throttle.clamp(-1.0, 1.0);
        self.last_action = Action {
            steer: steer_cmd,
            throttle: throttle_cmd,
        };

        for _ in 0..SUBSTEPS {
            let speed_ratio = self.vx.abs() / MAX_SPEED;
            let speed_factor = 1.0 / (1.0 + 0.45 * speed_ratio * speed_ratio);
            let target_steer = steer_cmd * MAX_STEER_ANGLE * speed_factor;
            let steer_diff = target_steer - self.steer_angle;
            let max_delta = STEER_SERVO_RATE * PHYSICS_DT;
            self.steer_angle += steer_diff.clamp(-max_delta, max_delta);

            let accel_x = if throttle_cmd >= 0.0 {
                throttle_cmd * ENGINE_ACCEL
            } else if self.vx > 5.0 {
                throttle_cmd * BRAKE_DECEL
            } else {
                throttle_cmd * REVERSE_ACCEL
            };
            let drag_x = AERO_DRAG * self.vx * self.vx.abs() + ROLLING_DRAG * self.vx;
            self.vx += (accel_x - drag_x) * PHYSICS_DT;
            self.vx = self.vx.clamp(-0.25 * MAX_SPEED, MAX_SPEED);

            if self.vx.abs() > 25.0 {
                let lf = WHEELBASE * 0.5;
                let lr = WHEELBASE * 0.5;
                let steer_sign = if self.vx > 0.0 { 1.0 } else { -1.0 };
                let alpha_f = (self.vy + lf * self.yaw_rate).atan2(self.vx.abs())
                    - self.steer_angle * steer_sign;
                let alpha_r = (self.vy - lr * self.yaw_rate).atan2(self.vx.abs());
                let mut fy_f = -CORNER_STIFF_F * alpha_f * self.vx.abs();
                let mut fy_r = -CORNER_STIFF_R * alpha_r * self.vx.abs();
                fy_f = fy_f.clamp(-MAX_LATERAL_ACCEL, MAX_LATERAL_ACCEL);
                fy_r = fy_r.clamp(-MAX_LATERAL_ACCEL, MAX_LATERAL_ACCEL);

                self.vy += ((fy_f + fy_r) - self.vx * self.yaw_rate) * PHYSICS_DT;
                self.vy *= 0.94;
                let yaw_accel = (lf * fy_f - lr * fy_r) / (0.5 * WHEELBASE);
                self.yaw_rate += yaw_accel * PHYSICS_DT;
                self.yaw_rate *= 0.90;
            } else {
                self.vy *= 0.8;
                self.yaw_rate = (self.vx / WHEELBASE) * self.steer_angle.tan();
            }

            // Heading is wrapped before body velocity is rotated into the world.
            self.heading =
                py_mod(self.heading + self.yaw_rate * PHYSICS_DT + std::f64::consts::PI,
                       std::f64::consts::TAU)
                    - std::f64::consts::PI;
            let cos_h = self.heading.cos();
            let sin_h = self.heading.sin();
            let world_vx = self.vx * cos_h - self.vy * sin_h;
            let world_vy = self.vx * sin_h + self.vy * cos_h;
            self.x += world_vx * PHYSICS_DT;
            self.y += world_vy * PHYSICS_DT;
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

        let mut components = RewardComponents {
            progress: (world_vx * nearest.tangent.x + world_vy * nearest.tangent.y) * DECISION_DT
                / 60.0,
            slip: -0.0004 * self.vy.abs() * DECISION_DT,
            time: -TIME_COST,
            ..RewardComponents::default()
        };
        let mut reward = components.total();

        let warn_dist = WALL_WARN_FRACTION * nearest.half_width;
        if self.min_ray < warn_dist {
            let closeness = (warn_dist - self.min_ray) / warn_dist;
            components.wall = -WALL_PENALTY * closeness * closeness;
            reward += components.wall;
        }

        // Carrying more speed than the next corner allows costs every step, not
        // just when the wall arrives.
        let corner = self.track.tightest_corner_ahead(nearest.arc, PREVIEW_CORNER);
        if corner.is_finite() && self.speed > START_SPEED * 0.6 {
            let limit = (MAX_LATERAL_ACCEL * corner).sqrt();
            let excess = (self.speed / limit - 1.0).clamp(0.0, 2.0);
            if excess > 0.0 {
                components.excess_speed = -SPEED_EXCESS_PENALTY * excess;
                reward += components.excess_speed;
            }
        }

        let half_lap = self.track.lap_length / 2.0;
        let delta = py_mod(nearest.arc - self.prev_arc + half_lap, self.track.lap_length) - half_lap;
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
                    ((self.next_checkpoint - 1) % gate_count) as usize,
                    previous,
                    pos,
                ) == -1
            {
                self.next_checkpoint -= 1;
                self.laps_completed = self.next_checkpoint / gate_count;
            }
            if self
                .track
                .checkpoint_crossing((self.next_checkpoint % gate_count) as usize, previous, pos)
                == 1
            {
                self.next_checkpoint += 1;
                components.checkpoint += CHECKPOINT_BONUS;
                reward += CHECKPOINT_BONUS;
                if self.next_checkpoint % gate_count == 0 {
                    components.lap += LAP_BONUS;
                    reward += LAP_BONUS;
                    self.laps_completed = self.next_checkpoint / gate_count;
                }
            }
            if self.next_checkpoint == self.total_gates() {
                self.lap_done = true;
                let pace = (1.0 - self.steps as f64 / MAX_STEPS as f64).max(0.0);
                components.lap += FAST_LAP_BONUS * pace;
                reward += FAST_LAP_BONUS * pace;
                true
            } else {
                self.steps >= MAX_STEPS
            }
        };

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
                lap_done: true,
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
        preview(hit.arc + PREVIEW_FAR);
        let turn = wrap_angle(
            self.track.arc_tangent(hit.arc + PREVIEW_CURVE).angle() - hit.tangent.angle(),
        );
        self.obs[8] = near_offset.clamp(-2.0, 2.0) as f32;
        self.obs[9] = (self.track.arc_half_width(hit.arc + PREVIEW_NEAR) / BASE_HALF_WIDTH)
            .min(2.0) as f32;
        self.obs[10] = (turn / std::f64::consts::PI).clamp(-1.0, 1.0) as f32;
        self.obs[11] = (self.track.arc_half_width(hit.arc + PREVIEW_FAR) / BASE_HALF_WIDTH)
            .min(2.0) as f32;
        // Speed the tightest upcoming corner can be taken at, relative to the
        // current speed: above one means the car is carrying too much speed.
        let corner = self.track.tightest_corner_ahead(hit.arc, PREVIEW_CORNER);
        let demand = if corner.is_finite() {
            (MAX_LATERAL_ACCEL * corner).sqrt() / self.speed.max(START_SPEED * 0.6)
        } else {
            0.0
        };
        self.obs[12] = (demand / 3.0).min(1.0) as f32;
        // Square-root compression: on a road 40 units wide, a 320-unit sensor
        // range would quantize the wall into the bottom 12% of the input.
        for (slot, ray) in self.obs[STATE_FEATURES..].iter_mut().zip(&self.rays) {
            *slot = (*ray / RAY_RANGE).sqrt() as f32;
        }
        hit
    }
}
