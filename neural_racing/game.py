"""Top-down car environment: stadium track, car physics, raycasts, reward."""

import math

import numpy as np
import pygame

WIDTH, HEIGHT = 1000, 700
MAX_SPEED = 420.0
ACCEL = 260.0
BRAKE = 400.0
DRAG = 0.6
TURN_RATE = 3.2
MIN_TURN_SPEED = 120.0
PHYSICS_DT = 1.0 / 60.0
SUBSTEPS = 3
DECISION_DT = PHYSICS_DT * SUBSTEPS
MAX_STEPS = 600
HALF_WIDTH = 80.0
NUM_RAYS = 8
RAY_RANGE = 400.0

START = np.array([300.0, 500.0])
START_HEADING = 0.0
START_SPEED = 0.3 * MAX_SPEED

WALL_WARN_DIST = 80.0
WALL_PENALTY = 0.3
OFF_TRACK_PENALTY = 10.0
LAP_BONUS = 10.0
TIME_COST = 0.002

COLOR_BG = (24, 26, 30)
COLOR_TRACK = (58, 62, 70)
COLOR_EDGE = (150, 156, 168)
COLOR_CAR = (232, 84, 66)
COLOR_LINE = (200, 205, 215)


def _stadium_centerline(samples=180):
    """Closed stadium loop: two straights joined by semicircular ends."""
    r = 150.0
    n_arc = samples // 4
    pts = []
    pts.append(np.stack([np.linspace(300.0, 700.0, samples // 2, endpoint=False),
                         np.full(samples // 2, 500.0)], axis=1))
    ang = np.linspace(np.pi / 2, -np.pi / 2, n_arc, endpoint=False)
    pts.append(np.stack([700.0 + r * np.cos(ang), 350.0 + r * np.sin(ang)], axis=1))
    pts.append(np.stack([np.linspace(700.0, 300.0, samples // 2, endpoint=False),
                         np.full(samples // 2, 200.0)], axis=1))
    ang = np.linspace(-np.pi / 2, -3 * np.pi / 2, n_arc, endpoint=False)
    pts.append(np.stack([300.0 + r * np.cos(ang), 350.0 + r * np.sin(ang)], axis=1))
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
        self.seg_a = np.concatenate([_segments(self.outer)[0], _segments(self.inner)[0]])
        self.seg_b = np.concatenate([_segments(self.outer)[1], _segments(self.inner)[1]])
        self.seg_vec = self.seg_b - self.seg_a
        self.seg_len = np.linalg.norm(self.seg_vec, axis=1)
        self.seg_len_sq = np.maximum(self.seg_len ** 2, 1e-9)
        diff = self.center_seg_b - self.center_seg_a
        self.center_len = np.linalg.norm(diff, axis=1)
        self.center_tangent = diff / np.maximum(self.center_len[:, None], 1e-9)
        self.center_cum = np.concatenate([[0.0], np.cumsum(self.center_len)])[:-1]
        self.LAP_LENGTH = float(self.center_len.sum())

    def nearest_on_centerline(self, point):
        """Return (segment index, signed lateral offset, arc length)."""
        ap = point - self.center_seg_a
        denom = np.maximum(self.center_len ** 2, 1e-9)
        t = np.clip((ap * (self.center_seg_b - self.center_seg_a)).sum(axis=1) / denom, 0.0, 1.0)
        closest = self.center_seg_a + t[:, None] * (self.center_seg_b - self.center_seg_a)
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
        # Ray/segment intersection via the 2D cross product, vectorized over segments.
        denom = d[:, None, 0] * self.seg_vec[None, :, 1] - d[:, None, 1] * self.seg_vec[None, :, 0]
        diff = self.seg_a[None, :, :] - np.array([ox, oy])
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (diff[:, :, 0] * self.seg_vec[None, :, 1] - diff[:, :, 1] * self.seg_vec[None, :, 0]) / denom
            u = (diff[:, :, 0] * d[:, None, 1] - diff[:, :, 1] * d[:, None, 0]) / denom
        hit = (np.abs(denom) > 1e-9) & (t >= 0.0) & (u >= 0.0) & (u <= 1.0)
        t = np.where(hit, t, np.inf)
        return np.minimum(t.min(axis=1), RAY_RANGE)


class Env:
    def __init__(self, seed=None):
        self.track = Track()
        self.rng = np.random.default_rng(seed)
        self.ray_angles = np.linspace(-np.pi / 2, np.pi / 2, NUM_RAYS)
        self.reset()

    def reset(self):
        self.x, self.y = START
        self.heading = START_HEADING
        self.speed = START_SPEED
        self.steps = 0
        self.distance = 0.0
        self.prev_arc = self.track.nearest_on_centerline(np.array([self.x, self.y]))[2]
        self.min_ray = RAY_RANGE
        self.off_track = False
        self.lap_done = False
        return self._observation()

    def _observation(self):
        rays = self.track.cast_rays(np.array([self.x, self.y]), self.heading + self.ray_angles)
        self.min_ray = float(rays.min())
        return np.concatenate([
            [(self.x - WIDTH / 2) / (WIDTH / 2), (self.y - HEIGHT / 2) / (HEIGHT / 2),
             self.speed / MAX_SPEED, math.sin(self.heading), math.cos(self.heading)],
            rays / RAY_RANGE,
        ]).astype(np.float32)

    def step(self, action):
        steer = float(np.clip(action[0], -1.0, 1.0))
        throttle = float(np.clip(action[1], -1.0, 1.0))
        for _ in range(SUBSTEPS):
            if throttle >= 0:
                self.speed += throttle * ACCEL * PHYSICS_DT
            else:
                self.speed += throttle * BRAKE * PHYSICS_DT
            self.speed -= self.speed * DRAG * PHYSICS_DT
            self.speed = float(np.clip(self.speed, -0.3 * MAX_SPEED, MAX_SPEED))
            grip = min(1.0, abs(self.speed) / MIN_TURN_SPEED)
            self.heading += steer * TURN_RATE * grip * PHYSICS_DT
            self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
            self.x += self.speed * math.cos(self.heading) * PHYSICS_DT
            self.y += self.speed * math.sin(self.heading) * PHYSICS_DT

        self.steps += 1
        pos = np.array([self.x, self.y])
        tangent = self.track.tangent(pos)
        velocity = self.speed * np.array([math.cos(self.heading), math.sin(self.heading)])
        reward = float(np.dot(velocity, tangent)) * DECISION_DT / 100.0
        reward -= TIME_COST

        obs = self._observation()
        if self.min_ray < WALL_WARN_DIST:
            reward -= WALL_PENALTY * ((WALL_WARN_DIST - self.min_ray) / WALL_WARN_DIST) ** 2

        _, lateral, arc = self.track.nearest_on_centerline(pos)
        delta = (arc - self.prev_arc + self.track.LAP_LENGTH / 2) % self.track.LAP_LENGTH - self.track.LAP_LENGTH / 2
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

        info = {"distance": max(0.0, self.distance), "off_track": self.off_track,
                "lap_done": self.lap_done, "steps": self.steps, "min_ray": self.min_ray}
        return obs, reward, done, info


def draw_scene(screen, env, font=None, hud_lines=None):
    track = env.track
    screen.fill(COLOR_BG)
    pygame.draw.polygon(screen, COLOR_TRACK, [(p[0], p[1]) for p in track.outer])
    pygame.draw.polygon(screen, COLOR_BG, [(p[0], p[1]) for p in track.inner])
    pygame.draw.lines(screen, COLOR_EDGE, True, [(p[0], p[1]) for p in track.outer], 2)
    pygame.draw.lines(screen, COLOR_EDGE, True, [(p[0], p[1]) for p in track.inner], 2)
    pygame.draw.line(screen, COLOR_LINE, (300, 420), (300, 580), 3)

    if env.min_ray < RAY_RANGE:
        origin = np.array([env.x, env.y])
        hits = track.cast_rays(origin, env.heading + env.ray_angles)
        for ang, dist in zip(env.heading + env.ray_angles, hits):
            end = origin + dist * np.array([math.cos(ang), math.sin(ang)])
            shade = int(80 + 175 * (1.0 - dist / RAY_RANGE))
            pygame.draw.line(screen, (30, shade, shade // 2), origin, end, 1)

    corners = np.array([[-14, -8], [14, -8], [14, 8], [-14, 8]], dtype=float)
    rot = np.array([[math.cos(env.heading), -math.sin(env.heading)],
                    [math.sin(env.heading), math.cos(env.heading)]])
    pts = corners @ rot.T + np.array([env.x, env.y])
    pygame.draw.polygon(screen, COLOR_CAR, [(p[0], p[1]) for p in pts])
    pygame.draw.circle(screen, COLOR_LINE, (int(env.x), int(env.y)), 2)

    if hud_lines and font is not None:
        for row, text in enumerate(hud_lines):
            surf = font.render(text, True, COLOR_LINE)
            screen.blit(surf, (12, 10 + row * 18))


def build_screen(caption="Neural Racing"):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption(caption)
    return screen
