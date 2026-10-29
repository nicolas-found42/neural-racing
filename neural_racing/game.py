"""Top-down car environment: stadium track, dynamic bicycle car model, raycasts, and beginner dashboard."""

import math
from collections import deque

import numpy as np
import pygame

TRACK_WIDTH = 960
DASH_WIDTH = 560
WIDTH = TRACK_WIDTH + DASH_WIDTH
HEIGHT = 820

# Dynamic Bicycle Car Physics Constants
MAX_SPEED = 420.0
ENGINE_ACCEL = 310.0
BRAKE_DECEL = 440.0
REVERSE_ACCEL = 160.0
AERO_DRAG = 0.0012
ROLLING_DRAG = 0.45

WHEELBASE = 28.0
MAX_STEER_ANGLE = 0.50  # ~28.6 degrees max front wheel angle
STEER_SERVO_RATE = 4.8  # rad/s steering actuator speed
CORNER_STIFF_F = 5.2  # Front tire lateral grip stiffness
CORNER_STIFF_R = 5.8  # Rear tire lateral grip stiffness
MAX_LATERAL_ACCEL = 620.0  # Tire friction circle limit (triggers drift/skids)

PHYSICS_DT = 1.0 / 60.0
SUBSTEPS = 3
DECISION_DT = PHYSICS_DT * SUBSTEPS
MAX_STEPS = 650
HALF_WIDTH = 80.0
NUM_RAYS = 11
RAY_RANGE = 400.0
OBS_DIM = 8 + NUM_RAYS

START = np.array([300.0, 540.0])
START_HEADING = 0.0
START_SPEED = 0.3 * MAX_SPEED

WALL_WARN_DIST = 80.0
WALL_PENALTY = 0.3
OFF_TRACK_PENALTY = 10.0
LAP_BONUS = 12.0
TIME_COST = 0.002

# Palette
COLOR_BG = (18, 20, 26)
COLOR_GRASS_STRIPE = (22, 25, 32)
COLOR_TRACK = (52, 56, 66)
COLOR_CENTERLINE = (85, 92, 108)
COLOR_EDGE = (210, 215, 225)
COLOR_KERB_RED = (210, 65, 60)
COLOR_CAR = (235, 75, 60)
COLOR_CAR_CABIN = (35, 42, 55)
COLOR_LINE = (225, 230, 240)
COLOR_DASH_BG = (26, 29, 38)
COLOR_CARD_BG = (34, 38, 50)
COLOR_CARD_BORDER = (58, 65, 84)
COLOR_TEXT_MAIN = (238, 242, 250)
COLOR_TEXT_MUTED = (160, 170, 190)
COLOR_GOOD = (64, 214, 128)
COLOR_WARN = (245, 190, 65)
COLOR_BAD = (240, 80, 75)
COLOR_ACCENT = (85, 170, 255)

_SCENE_FONTS: dict[str, pygame.font.Font] = {}


def _stadium_centerline(samples=180):
    """Closed stadium loop: two straights joined by semicircular ends."""
    r = 150.0
    n_arc = samples // 4
    pts = []
    pts.append(
        np.stack(
            [
                np.linspace(280.0, 680.0, samples // 2, endpoint=False),
                np.full(samples // 2, 540.0),
            ],
            axis=1,
        )
    )
    ang = np.linspace(np.pi / 2, -np.pi / 2, n_arc, endpoint=False)
    pts.append(np.stack([680.0 + r * np.cos(ang), 390.0 + r * np.sin(ang)], axis=1))
    pts.append(
        np.stack(
            [
                np.linspace(680.0, 280.0, samples // 2, endpoint=False),
                np.full(samples // 2, 240.0),
            ],
            axis=1,
        )
    )
    ang = np.linspace(-np.pi / 2, -3 * np.pi / 2, n_arc, endpoint=False)
    pts.append(np.stack([280.0 + r * np.cos(ang), 390.0 + r * np.sin(ang)], axis=1))
    return np.concatenate(pts, axis=0)


def _offset_polyline(poly, dist):
    """Offset a closed polyline along its per-vertex normals."""
    nxt = np.roll(poly, -1, axis=0)
    prv = np.roll(poly, 1, axis=0)
    tang = nxt - prv
    tang /= np.linalg.norm(tang, axis=1, keepdims=True)
    normal = np.stack([-tang[:, 1], tang[:, 0]], axis=1)
    return poly + normal * dist


def _segments(poly):
    return poly, np.roll(poly, -1, axis=0)


class Track:
    def __init__(self, samples=180):
        self.centerline = _stadium_centerline(samples)
        self.outer = _offset_polyline(self.centerline, HALF_WIDTH)
        self.inner = _offset_polyline(self.centerline, -HALF_WIDTH)
        self.center_seg_a, self.center_seg_b = _segments(self.centerline)
        self.seg_a = np.concatenate(
            [_segments(self.outer)[0], _segments(self.inner)[0]]
        )
        self.seg_b = np.concatenate(
            [_segments(self.outer)[1], _segments(self.inner)[1]]
        )
        self.seg_vec = self.seg_b - self.seg_a
        self.seg_len = np.linalg.norm(self.seg_vec, axis=1)
        self.seg_len_sq = np.maximum(self.seg_len**2, 1e-9)
        diff = self.center_seg_b - self.center_seg_a
        self.center_len = np.linalg.norm(diff, axis=1)
        self.center_tangent = diff / np.maximum(self.center_len[:, None], 1e-9)
        self.center_cum = np.concatenate([[0.0], np.cumsum(self.center_len)])[:-1]
        self.LAP_LENGTH = float(self.center_len.sum())

    def nearest_on_centerline(self, point):
        """Return (segment index, signed lateral offset, arc length)."""
        ap = point - self.center_seg_a
        denom = np.maximum(self.center_len**2, 1e-9)
        t = np.clip(
            (ap * (self.center_seg_b - self.center_seg_a)).sum(axis=1) / denom, 0.0, 1.0
        )
        closest = self.center_seg_a + t[:, None] * (
            self.center_seg_b - self.center_seg_a
        )
        d2 = ((point - closest) ** 2).sum(axis=1)
        i = int(np.argmin(d2))
        tangent = self.center_tangent[i]
        normal = np.array([-tangent[1], tangent[0]])
        lateral = float(np.dot(point - closest[i], normal))
        arc = float(self.center_cum[i] + t[i] * self.center_len[i])
        return i, lateral, arc

    def tangent(self, point):
        i, _, _ = self.nearest_on_centerline(point)
        return self.center_tangent[i]

    def cast_rays(self, origin, angles):
        """Distances to the boundary polylines along each angle, capped at RAY_RANGE."""
        d = np.stack([np.cos(angles), np.sin(angles)], axis=1)
        ox, oy = origin
        denom = (
            d[:, None, 0] * self.seg_vec[None, :, 1]
            - d[:, None, 1] * self.seg_vec[None, :, 0]
        )
        diff = self.seg_a[None, :, :] - np.array([ox, oy])
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (
                diff[:, :, 0] * self.seg_vec[None, :, 1]
                - diff[:, :, 1] * self.seg_vec[None, :, 0]
            ) / denom
            u = (diff[:, :, 0] * d[:, None, 1] - diff[:, :, 1] * d[:, None, 0]) / denom
        hit = (np.abs(denom) > 1e-9) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
        t = np.where(hit, t, np.inf)
        return np.minimum(t.min(axis=1), RAY_RANGE)


class Env:
    def __init__(self, seed=None):
        self.track = Track()
        self.rng = np.random.default_rng(seed)
        self.ray_angles = np.linspace(-0.6 * np.pi, 0.6 * np.pi, NUM_RAYS)
        self.skid_marks = deque(maxlen=260)
        self.trail = deque(maxlen=160)
        self.reset()

    def reset(self):
        self.x, self.y = float(START[0]), float(START[1])
        self.heading = float(START_HEADING)
        self.vx = float(START_SPEED)  # Forward velocity in car body frame
        self.vy = 0.0  # Lateral (sideways drift) velocity in car body frame
        self.yaw_rate = 0.0  # Angular velocity (rad/s)
        self.steer_angle = 0.0  # Actual front wheel angle (rad)
        self.slip_angle_deg = 0.0  # Current drift angle in degrees
        self.last_action = np.zeros(2, dtype=np.float32)
        self.last_reward = 0.0
        self.last_rays = np.full(NUM_RAYS, RAY_RANGE, dtype=np.float32)
        self.lateral_offset = 0.0
        self.speed = self.vx

        self.steps = 0
        self.distance = 0.0
        self.prev_arc = self.track.nearest_on_centerline(np.array([self.x, self.y]))[2]
        self.min_ray = RAY_RANGE
        self.off_track = False
        self.lap_done = False
        self.skid_marks.clear()
        self.trail.clear()
        return self._observation()

    def _observation(self):
        pos = np.array([self.x, self.y])
        rays = self.track.cast_rays(pos, self.heading + self.ray_angles)
        self.last_rays = rays
        self.min_ray = float(rays.min())

        seg_idx, lateral, _ = self.track.nearest_on_centerline(pos)
        self.lateral_offset = lateral
        tangent = self.track.center_tangent[seg_idx]
        track_angle = math.atan2(tangent[1], tangent[0])
        heading_err = (self.heading - track_angle + math.pi) % (2 * math.pi) - math.pi

        state_features = np.array(
            [
                self.vx / MAX_SPEED,
                np.clip(self.vy / 150.0, -1.0, 1.0),
                np.clip(self.yaw_rate / 4.0, -1.0, 1.0),
                self.steer_angle / MAX_STEER_ANGLE,
                np.clip(lateral / HALF_WIDTH, -1.5, 1.5),
                math.sin(heading_err),
                math.cos(heading_err),
                (self.x - TRACK_WIDTH / 2) / (TRACK_WIDTH / 2),
            ],
            dtype=np.float32,
        )

        return np.concatenate([state_features, (rays / RAY_RANGE).astype(np.float32)])

    def step(self, action):
        steer_cmd = float(np.clip(action[0], -1.0, 1.0))
        throttle_cmd = float(np.clip(action[1], -1.0, 1.0))
        self.last_action = np.array([steer_cmd, throttle_cmd], dtype=np.float32)

        for _ in range(SUBSTEPS):
            # 1. Steering actuator servo with speed-sensitive stability assist
            speed_factor = 1.0 / (1.0 + 0.45 * (abs(self.vx) / MAX_SPEED) ** 2)
            target_steer = steer_cmd * MAX_STEER_ANGLE * speed_factor
            steer_diff = target_steer - self.steer_angle
            max_delta = STEER_SERVO_RATE * PHYSICS_DT
            self.steer_angle += float(np.clip(steer_diff, -max_delta, max_delta))

            # 2. Longitudinal forces (throttle, braking, aerodynamic + rolling drag)
            if throttle_cmd >= 0.0:
                accel_x = throttle_cmd * ENGINE_ACCEL
            else:
                if self.vx > 5.0:
                    accel_x = throttle_cmd * BRAKE_DECEL
                else:
                    accel_x = throttle_cmd * REVERSE_ACCEL

            drag_x = AERO_DRAG * self.vx * abs(self.vx) + ROLLING_DRAG * self.vx
            self.vx += (accel_x - drag_x) * PHYSICS_DT
            self.vx = float(np.clip(self.vx, -0.25 * MAX_SPEED, MAX_SPEED))

            # 3. Dynamic Bicycle Lateral & Yaw Physics
            if abs(self.vx) > 25.0:
                Lf = WHEELBASE * 0.5
                Lr = WHEELBASE * 0.5
                # Front and rear tire slip angles
                alpha_f = math.atan2(
                    self.vy + Lf * self.yaw_rate, abs(self.vx)
                ) - self.steer_angle * np.sign(self.vx)
                alpha_r = math.atan2(self.vy - Lr * self.yaw_rate, abs(self.vx))

                # Tire cornering forces saturated by a friction circle
                fy_f = -CORNER_STIFF_F * alpha_f * abs(self.vx)
                fy_r = -CORNER_STIFF_R * alpha_r * abs(self.vx)
                fy_f = float(np.clip(fy_f, -MAX_LATERAL_ACCEL, MAX_LATERAL_ACCEL))
                fy_r = float(np.clip(fy_r, -MAX_LATERAL_ACCEL, MAX_LATERAL_ACCEL))

                # Update lateral slip velocity and yaw rate
                self.vy += ((fy_f + fy_r) - self.vx * self.yaw_rate) * PHYSICS_DT
                self.vy *= 0.94  # Tire lateral damping
                yaw_accel = (Lf * fy_f - Lr * fy_r) / (0.5 * WHEELBASE)
                self.yaw_rate += yaw_accel * PHYSICS_DT
                self.yaw_rate *= 0.90
            else:
                # Low-speed kinematic fallback
                self.vy *= 0.8
                self.yaw_rate = (self.vx / WHEELBASE) * math.tan(self.steer_angle)

            self.heading = (self.heading + self.yaw_rate * PHYSICS_DT + math.pi) % (
                2 * math.pi
            ) - math.pi

            # Convert car-frame velocity (vx, vy) to world coordinates
            cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
            world_vx = self.vx * cos_h - self.vy * sin_h
            world_vy = self.vx * sin_h + self.vy * cos_h
            self.x += world_vx * PHYSICS_DT
            self.y += world_vy * PHYSICS_DT

        self.speed = float(math.hypot(self.vx, self.vy))
        self.slip_angle_deg = float(
            math.degrees(math.atan2(self.vy, max(abs(self.vx), 1.0)))
        )
        self.trail.append((int(self.x), int(self.y)))

        # Record rear tire skid marks when drifting hard or braking hard
        if abs(self.slip_angle_deg) > 6.5 or (throttle_cmd < -0.65 and self.vx > 140.0):
            cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
            for side in (-6.5, 6.5):
                rx = self.x - 11.0 * cos_h - side * sin_h
                ry = self.y - 11.0 * sin_h + side * cos_h
                self.skid_marks.append((int(rx), int(ry)))

        self.steps += 1
        pos = np.array([self.x, self.y])
        tangent = self.track.tangent(pos)
        cos_h, sin_h = math.cos(self.heading), math.sin(self.heading)
        world_vel = np.array(
            [self.vx * cos_h - self.vy * sin_h, self.vx * sin_h + self.vy * cos_h]
        )

        # Reward forward progress along track, penalize excessive sideways sliding
        reward = float(np.dot(world_vel, tangent)) * DECISION_DT / 100.0
        reward -= 0.0004 * abs(self.vy) * DECISION_DT
        reward -= TIME_COST

        obs = self._observation()
        if self.min_ray < WALL_WARN_DIST:
            reward -= (
                WALL_PENALTY * ((WALL_WARN_DIST - self.min_ray) / WALL_WARN_DIST) ** 2
            )

        _, lateral, arc = self.track.nearest_on_centerline(pos)
        delta = (
            arc - self.prev_arc + self.track.LAP_LENGTH / 2
        ) % self.track.LAP_LENGTH - self.track.LAP_LENGTH / 2
        if abs(delta) < 200.0:
            self.distance += delta
        self.prev_arc = arc

        if self.distance >= self.track.LAP_LENGTH:
            self.lap_done = True
            reward += LAP_BONUS
            done = True
        elif abs(lateral) > HALF_WIDTH:
            self.off_track = True
            reward -= OFF_TRACK_PENALTY
            done = True
        else:
            done = self.steps >= MAX_STEPS

        self.last_reward = float(reward)
        info = {
            "distance": max(0.0, self.distance),
            "off_track": self.off_track,
            "lap_done": self.lap_done,
            "steps": self.steps,
            "min_ray": self.min_ray,
            "slip_deg": self.slip_angle_deg,
        }
        return obs, reward, done, info


def _transform_pts(local_pts, center, angle):
    cos_a, sin_a = math.cos(angle), math.sin(angle)
    rot = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
    pts = np.asarray(local_pts, dtype=float) @ rot.T + np.asarray(center, dtype=float)
    return [(float(p[0]), float(p[1])) for p in pts]


def _draw_car_model(screen, env):
    """Draw a detailed top-down car with articulated front wheels, cabin, and brake lights."""
    center = (env.x, env.y)
    h = env.heading

    # 1. Draw 4 Wheels (Front wheels rotate with env.steer_angle)
    wheel_box = [[-4.5, -2.2], [4.5, -2.2], [4.5, 2.2], [-4.5, 2.2]]
    wheel_offsets = [
        (10.5, -8.5, h + env.steer_angle),  # Front-Left
        (10.5, 8.5, h + env.steer_angle),  # Front-Right
        (-10.5, -8.5, h),  # Rear-Left
        (-10.5, 8.5, h),  # Rear-Right
    ]
    cos_h, sin_h = math.cos(h), math.sin(h)
    for lx, ly, w_ang in wheel_offsets:
        wx = env.x + lx * cos_h - ly * sin_h
        wy = env.y + lx * sin_h + ly * cos_h
        pygame.draw.polygon(
            screen, (20, 22, 25), _transform_pts(wheel_box, (wx, wy), w_ang)
        )

    # 2. Sculpted Car Chassis
    chassis = [
        [-15, -7.5],
        [-12, -8.5],
        [10, -8.5],
        [16, -6.0],
        [16, 6.0],
        [10, 8.5],
        [-12, 8.5],
        [-15, 7.5],
    ]
    pygame.draw.polygon(screen, COLOR_CAR, _transform_pts(chassis, center, h))
    pygame.draw.polygon(screen, (255, 140, 125), _transform_pts(chassis, center, h), 1)

    # 3. Cabin & Windshield
    cabin = [[-7, -6.0], [6, -5.5], [3, 5.5], [-7, 6.0]]
    pygame.draw.polygon(screen, COLOR_CAR_CABIN, _transform_pts(cabin, center, h))
    windshield = [[2, -5.2], [8, -4.5], [8, 4.5], [2, 5.2]]
    pygame.draw.polygon(screen, (95, 135, 170), _transform_pts(windshield, center, h))

    # 4. Headlights & Brake Lights
    for ly in (-5.0, 5.0):
        hx = env.x + 15.5 * cos_h - ly * sin_h
        hy = env.y + 15.5 * sin_h + ly * cos_h
        pygame.draw.circle(screen, (255, 245, 190), (int(hx), int(hy)), 2)

        bx = env.x - 14.8 * cos_h - ly * sin_h
        by = env.y - 14.8 * sin_h + ly * cos_h
        braking = env.last_action[1] < -0.15
        b_color = (255, 40, 40) if braking else (120, 25, 25)
        pygame.draw.circle(screen, b_color, (int(bx), int(by)), 3 if braking else 2)


def _draw_neural_net_panel(screen, rect, policy, fonts):
    """Draw a beginner-friendly live visual diagram of the 6-layer deep neural network."""
    pygame.draw.rect(screen, COLOR_CARD_BG, rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, rect, 1, border_radius=8)

    title = fonts["bold"].render(
        "INSIDE THE AI BRAIN (6-Layer Deep Residual Network)", True, COLOR_ACCENT
    )
    screen.blit(title, (rect.x + 12, rect.y + 8))
    sub = fonts["small"].render(
        "Each circle is a 'Neuron'. Bright Green = Active (+), Red = Inhibited (-). Lines show strong pathways.",
        True,
        COLOR_TEXT_MUTED,
    )
    screen.blit(sub, (rect.x + 12, rect.y + 28))

    acts = getattr(policy, "last_layer_activations", [])
    if not acts:
        return

    # Display up to 8 representative nodes per hidden layer so a beginner can read it clearly
    max_nodes_per_layer = [8, 9, 9, 9, 9, 9, 8, 2]
    num_cols = len(acts)
    left_pad = rect.x + 58
    right_pad = rect.right - 68
    top_pad = rect.y + 62
    bot_pad = rect.bottom - 26

    col_xs = np.linspace(left_pad, right_pad, num_cols)
    node_positions = []
    sampled_values = []
    sampled_indices = []

    for col_idx, layer_vec in enumerate(acts):
        n_total = len(layer_vec)
        n_show = min(
            n_total, max_nodes_per_layer[min(col_idx, len(max_nodes_per_layer) - 1)]
        )
        if n_total <= n_show:
            idxs = np.arange(n_total)
        else:
            idxs = np.linspace(0, n_total - 1, n_show, dtype=int)
        vals = layer_vec[idxs]
        ys = (
            np.linspace(top_pad, bot_pad, n_show)
            if n_show > 1
            else [0.5 * (top_pad + bot_pad)]
        )
        if col_idx == num_cols - 1:
            ys = [
                top_pad + 0.30 * (bot_pad - top_pad),
                top_pad + 0.70 * (bot_pad - top_pad),
            ]
        pts = [(int(col_xs[col_idx]), int(y)) for y in ys]
        node_positions.append(pts)
        sampled_values.append(vals)
        sampled_indices.append(idxs)

    # 1. Draw active connections between layers
    for col_idx in range(num_cols - 1):
        W = policy.params.get(f"W{col_idx + 1}")
        if W is None:
            continue
        pts_a = node_positions[col_idx]
        pts_b = node_positions[col_idx + 1]
        idxs_a = sampled_indices[col_idx]
        idxs_b = sampled_indices[col_idx + 1]
        vals_a = sampled_values[col_idx]

        for i_a, (xa, ya) in enumerate(pts_a):
            act_a = float(vals_a[i_a])
            if abs(act_a) < 0.08:
                continue
            for i_b, (xb, yb) in enumerate(pts_b):
                w_val = float(W[idxs_a[i_a], idxs_b[i_b]])
                signal = act_a * w_val
                strength = min(1.0, abs(signal) * 1.4)
                if strength < 0.14:
                    continue
                if signal >= 0:
                    col = (
                        int(40 + 50 * strength),
                        int(90 + 150 * strength),
                        int(70 + 70 * strength),
                    )
                else:
                    col = (
                        int(110 + 135 * strength),
                        int(45 + 40 * strength),
                        int(50 + 40 * strength),
                    )
                pygame.draw.line(
                    screen, col, (xa, ya), (xb, yb), 1 if strength < 0.55 else 2
                )

    # 2. Draw Neuron Nodes
    layer_titles = ["Sensors", "L1", "L2", "L3", "L4", "L5", "L6", "Actions"]
    for col_idx, pts in enumerate(node_positions):
        vals = sampled_values[col_idx]
        lbl = fonts["small"].render(
            layer_titles[min(col_idx, len(layer_titles) - 1)], True, COLOR_TEXT_MUTED
        )
        screen.blit(lbl, (pts[0][0] - lbl.get_width() // 2, rect.y + 44))

        for i_n, (nx, ny) in enumerate(pts):
            v = float(np.tanh(vals[i_n] * 0.6))
            mag = min(1.0, abs(v))
            if v >= 0:
                fill = (
                    int(35 + 30 * (1 - mag)),
                    int(65 + 185 * mag),
                    int(55 + 85 * mag),
                )
            else:
                fill = (
                    int(70 + 180 * mag),
                    int(40 + 35 * (1 - mag)),
                    int(45 + 35 * (1 - mag)),
                )
            radius = 7 if col_idx in (0, num_cols - 1) else 5
            pygame.draw.circle(screen, fill, (nx, ny), radius)
            pygame.draw.circle(screen, (210, 220, 235), (nx, ny), radius, 1)

    # Input & Output beginner labels
    in_labels = [
        "Speed",
        "Drift",
        "Spin",
        "Steer",
        "Wall L",
        "Front",
        "Wall R",
        "Far R",
    ]
    for i, (nx, ny) in enumerate(node_positions[0]):
        if i < len(in_labels):
            t = fonts["small"].render(in_labels[i], True, COLOR_TEXT_MUTED)
            screen.blit(t, (nx - t.get_width() - 9, ny - 6))

    out_labels = ["Steer", "Pedal"]
    for i, (nx, ny) in enumerate(node_positions[-1]):
        if i < len(out_labels):
            t = fonts["bold"].render(out_labels[i], True, COLOR_TEXT_MAIN)
            screen.blit(t, (nx + 11, ny - 7))


def _draw_bar_gauge(
    screen, x, y, w, h, value, min_val, max_val, color, center_zero=False
):
    pygame.draw.rect(screen, (22, 25, 34), (x, y, w, h), border_radius=4)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, (x, y, w, h), 1, border_radius=4)
    norm = np.clip((value - min_val) / max(max_val - min_val, 1e-6), 0.0, 1.0)
    if center_zero:
        mid = x + w // 2
        pygame.draw.line(screen, (120, 130, 150), (mid, y), (mid, y + h), 1)
        px = int(x + norm * w)
        left = min(mid, px)
        width = max(2, abs(px - mid))
        pygame.draw.rect(screen, color, (left, y + 2, width, h - 4), border_radius=3)
    else:
        fill_w = int(norm * (w - 4))
        if fill_w > 0:
            pygame.draw.rect(
                screen, color, (x + 2, y + 2, fill_w, h - 4), border_radius=3
            )


def _plain_english_summary(env):
    """Generate a single clear sentence explaining what the car is doing right now."""
    if env.off_track:
        return "CRASHED! The car ran off the road boundary and is resetting."
    if env.lap_done:
        return "LAP COMPLETE! The AI successfully drove all the way around the stadium!"

    steer, pedal = float(env.last_action[0]), float(env.last_action[1])
    steer_txt = (
        "steering right"
        if steer > 0.2
        else ("steering left" if steer < -0.2 else "holding the wheel straight")
    )
    pedal_txt = (
        "pressing the gas"
        if pedal > 0.15
        else ("hitting the brakes" if pedal < -0.15 else "coasting")
    )
    if abs(env.slip_angle_deg) > 7.5:
        drift_txt = f" (tires drifting {abs(env.slip_angle_deg):.0f} deg!)"
    elif env.min_ray < 65.0:
        drift_txt = " (careful: very close to the wall!)"
    else:
        drift_txt = " safely on the track."
    return f"Currently {pedal_txt} & {steer_txt}{drift_txt}"


def draw_scene(screen, env, font=None, hud_lines=None, policy=None, stats=None):
    if not _SCENE_FONTS:
        _SCENE_FONTS.update(
            {
                "title": pygame.font.SysFont("arial", 18, bold=True),
                "bold": pygame.font.SysFont("arial", 14, bold=True),
                "body": pygame.font.SysFont("arial", 13),
                "small": pygame.font.SysFont("arial", 11),
            }
        )
    fonts = _SCENE_FONTS

    track = env.track
    screen.fill(COLOR_BG)

    # 1. Draw Track Surface, Centerline, and Start/Finish Checkerboard
    pygame.draw.polygon(screen, COLOR_TRACK, [(p[0], p[1]) for p in track.outer])
    pygame.draw.polygon(screen, COLOR_BG, [(p[0], p[1]) for p in track.inner])

    # Dashed centerline
    for i in range(0, len(track.centerline), 4):
        p1 = track.centerline[i]
        p2 = track.centerline[(i + 1) % len(track.centerline)]
        pygame.draw.line(screen, COLOR_CENTERLINE, (p1[0], p1[1]), (p2[0], p2[1]), 2)

    pygame.draw.lines(screen, COLOR_EDGE, True, [(p[0], p[1]) for p in track.outer], 3)
    pygame.draw.lines(screen, COLOR_EDGE, True, [(p[0], p[1]) for p in track.inner], 3)

    # Checkerboard Start/Finish Line
    for idx, y_start in enumerate(range(460, 620, 16)):
        col = (240, 240, 245) if idx % 2 == 0 else (40, 42, 48)
        pygame.draw.rect(screen, col, (295, y_start, 10, 16))

    # 2. Draw Skid Marks & Driving Trail
    for sx, sy in env.skid_marks:
        pygame.draw.circle(screen, (32, 34, 40), (sx, sy), 2)
    if len(env.trail) > 2:
        pygame.draw.lines(screen, (70, 95, 130), False, list(env.trail), 1)

    # 3. Draw Color-Coded Laser Distance Sensors (Green=Safe, Yellow=Medium, Red=Danger)
    origin = np.array([env.x, env.y])
    hits = env.last_rays
    for ang, dist in zip(env.heading + env.ray_angles, hits):
        end = origin + dist * np.array([math.cos(ang), math.sin(ang)])
        ratio = dist / RAY_RANGE
        if ratio < 0.22:
            ray_col = COLOR_BAD
        elif ratio < 0.50:
            ray_col = COLOR_WARN
        else:
            ray_col = COLOR_GOOD
        pygame.draw.line(screen, ray_col, origin, end, 1)
        pygame.draw.circle(screen, ray_col, (int(end[0]), int(end[1])), 3)

    # 4. Draw Upgraded Car Model
    _draw_car_model(screen, env)

    # 5. Live Plain-English Banner at Bottom of Track View
    banner_rect = pygame.Rect(16, HEIGHT - 76, TRACK_WIDTH - 32, 60)
    pygame.draw.rect(screen, COLOR_CARD_BG, banner_rect, border_radius=8)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, banner_rect, 1, border_radius=8)
    status_title = fonts["bold"].render(
        "LIVE AI DRIVING NARRATOR (Plain-English Summary):", True, COLOR_ACCENT
    )
    screen.blit(status_title, (banner_rect.x + 14, banner_rect.y + 8))
    status_text = fonts["title"].render(
        _plain_english_summary(env), True, COLOR_TEXT_MAIN
    )
    screen.blit(status_text, (banner_rect.x + 14, banner_rect.y + 30))

    # 6. Right-Side Beginner Dashboard Panel
    dash_rect = pygame.Rect(TRACK_WIDTH, 0, DASH_WIDTH, HEIGHT)
    pygame.draw.rect(screen, COLOR_DASH_BG, dash_rect)
    pygame.draw.line(
        screen, COLOR_CARD_BORDER, (TRACK_WIDTH, 0), (TRACK_WIDTH, HEIGHT), 2
    )

    dx = TRACK_WIDTH + 14
    dw = DASH_WIDTH - 28

    # Card A: Plain-English Controls & Telemetry (What the Car Sees & Does)
    card1 = pygame.Rect(dx, 12, dw, 228)
    pygame.draw.rect(screen, COLOR_CARD_BG, card1, border_radius=8)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, card1, 1, border_radius=8)
    screen.blit(
        fonts["bold"].render(
            "1. CAR CONTROLS & SENSORS (What is happening right now)",
            True,
            COLOR_ACCENT,
        ),
        (card1.x + 12, card1.y + 8),
    )

    steer_val = float(env.last_action[0])
    pedal_val = float(env.last_action[1])
    lap_pct = min(100.0, 100.0 * max(0.0, env.distance) / env.track.LAP_LENGTH)

    rows = [
        (
            "Forward Speed",
            f"{max(0.0, env.vx):3.0f} / {MAX_SPEED:.0f} px/s",
            env.vx,
            0.0,
            MAX_SPEED,
            COLOR_ACCENT,
            False,
        ),
        (
            "Steering Wheel",
            f"{'LEFT' if steer_val < -0.08 else ('RIGHT' if steer_val > 0.08 else 'CENTER')} ({steer_val:+.2f})",
            steer_val,
            -1.0,
            1.0,
            COLOR_WARN,
            True,
        ),
        (
            "Gas / Brake Pedal",
            f"{'GAS' if pedal_val >= 0 else 'BRAKE'} ({pedal_val:+.2f})",
            pedal_val,
            -1.0,
            1.0,
            COLOR_GOOD if pedal_val >= 0 else COLOR_BAD,
            True,
        ),
        (
            "Tire Drift / Slip",
            f"{abs(env.slip_angle_deg):4.1f} deg {'(SLIDING!)' if abs(env.slip_angle_deg) > 6.5 else '(Gripping)'}",
            abs(env.slip_angle_deg),
            0.0,
            25.0,
            COLOR_BAD if abs(env.slip_angle_deg) > 6.5 else COLOR_GOOD,
            False,
        ),
        (
            "Closest Wall Dist",
            f"{env.min_ray:3.0f} px {'(DANGER!)' if env.min_ray < 75 else '(Safe)'}",
            env.min_ray,
            0.0,
            RAY_RANGE,
            COLOR_BAD if env.min_ray < 75 else COLOR_GOOD,
            False,
        ),
        (
            "Lap Completion",
            f"{lap_pct:5.1f}% around track",
            lap_pct,
            0.0,
            100.0,
            COLOR_GOOD,
            False,
        ),
    ]

    for idx, (label, val_txt, val, vmin, vmax, col, centered) in enumerate(rows):
        ry = card1.y + 36 + idx * 31
        screen.blit(
            fonts["body"].render(label, True, COLOR_TEXT_MAIN), (card1.x + 12, ry)
        )
        _draw_bar_gauge(
            screen,
            card1.x + 145,
            ry + 2,
            195,
            14,
            val,
            vmin,
            vmax,
            col,
            center_zero=centered,
        )
        screen.blit(
            fonts["small"].render(val_txt, True, COLOR_TEXT_MUTED),
            (card1.x + 350, ry + 1),
        )

    # Card B: Visual Neural Network Diagram
    card2 = pygame.Rect(dx, 250, dw, 355)
    if policy is not None:
        _draw_neural_net_panel(screen, card2, policy, fonts)

    # Card C: Training Progress & History Chart
    card3 = pygame.Rect(dx, 615, dw, 193)
    pygame.draw.rect(screen, COLOR_CARD_BG, card3, border_radius=8)
    pygame.draw.rect(screen, COLOR_CARD_BORDER, card3, 1, border_radius=8)
    screen.blit(
        fonts["bold"].render(
            "2. TRAINING & PERFORMANCE SCOREBOARD", True, COLOR_ACCENT
        ),
        (card3.x + 12, card3.y + 8),
    )

    if stats:
        ep = stats.get("episode", 1)
        total_eps = stats.get("total_episodes", ep)
        sigma = stats.get("sigma", 0.0)
        best_dist = stats.get("best_distance", env.distance)
        laps_completed = stats.get("laps", 0)
        history_dist = stats.get("history_distance", [])

        info_lines = [
            f"Attempt (Episode): #{ep} of {total_eps}"
            if total_eps
            else f"Run #{ep} (Watch Mode)",
            f"Best Distance Reached: {best_dist:,.0f} / {env.track.LAP_LENGTH:,.0f} px ({100 * best_dist / env.track.LAP_LENGTH:.0f}%)",
            f"Total Full Laps Finished: {laps_completed}",
            f"Exploration Curiosity (Noise): {sigma * 100:.0f}% (High early to explore, low later for precision)",
        ]
        for i, line in enumerate(info_lines):
            screen.blit(
                fonts["body"].render(line, True, COLOR_TEXT_MAIN),
                (card3.x + 12, card3.y + 32 + i * 20),
            )

        # Mini sparkline chart of recent episode distances
        chart_rect = pygame.Rect(card3.x + 12, card3.y + 116, dw - 24, 65)
        pygame.draw.rect(screen, (22, 25, 34), chart_rect, border_radius=4)
        pygame.draw.rect(screen, COLOR_CARD_BORDER, chart_rect, 1, border_radius=4)
        screen.blit(
            fonts["small"].render(
                "Learning Curve (Distance Driven per Attempt - Goal is Top Line):",
                True,
                COLOR_TEXT_MUTED,
            ),
            (chart_rect.x + 8, chart_rect.y + 4),
        )

        if len(history_dist) >= 2:
            recent = history_dist[-60:]
            xs = np.linspace(chart_rect.x + 8, chart_rect.right - 8, len(recent))
            ys = [
                chart_rect.bottom
                - 6
                - (min(d, env.track.LAP_LENGTH) / env.track.LAP_LENGTH)
                * (chart_rect.height - 24)
                for d in recent
            ]
            pts = list(zip(xs.astype(int), np.array(ys, dtype=int)))
            pygame.draw.lines(screen, COLOR_GOOD, False, pts, 2)
    elif hud_lines:
        for row, text in enumerate(hud_lines):
            surf = fonts["body"].render(text, True, COLOR_TEXT_MAIN)
            screen.blit(surf, (card3.x + 12, card3.y + 34 + row * 22))


def build_screen(caption="Deep Neural Racing & Live Brain Dashboard"):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(caption)
    return screen
