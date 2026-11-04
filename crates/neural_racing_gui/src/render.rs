//! Track, car and sensor painting for the race view.

use egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2 as EVec2};
use neural_racing_core::{Env, Track, Vec2, CAR_LENGTH, CAR_WIDTH, UNITS_PER_METRE};

use crate::palette as colors;

pub struct View {
    origin: Pos2,
    scale: f32,
    /// World position drawn at `origin`.
    world_min: Vec2,
}

impl View {
    /// Frames `bounds` plus `margin` world units, so every track fills the
    /// canvas whatever its size.
    pub fn fit(rect: Rect, bounds: neural_racing_core::Bounds, margin: f32) -> Self {
        let width = (bounds.max_x - bounds.min_x) as f32 + 2.0 * margin;
        let height = (bounds.max_y - bounds.min_y) as f32 + 2.0 * margin;
        let scale = (rect.width() / width).min(rect.height() / height);
        let center = Vec2 {
            x: (bounds.min_x + bounds.max_x) * 0.5,
            y: (bounds.min_y + bounds.max_y) * 0.5,
        };
        Self {
            origin: rect.center(),
            scale,
            world_min: center,
        }
    }

    /// Fixed physical field of view: car and road always share one scale.
    pub fn follow(rect: Rect, position: Vec2) -> Self {
        Self {
            origin: rect.center(),
            scale: rect.width().min(rect.height()) / (100.0 * UNITS_PER_METRE as f32),
            world_min: position,
        }
    }

    pub fn pos(&self, point: Vec2) -> Pos2 {
        Pos2::new(
            self.origin.x + (point.x - self.world_min.x) as f32 * self.scale,
            self.origin.y + (point.y - self.world_min.y) as f32 * self.scale,
        )
    }
}

fn closed(points: &[Vec2], view: &View) -> Vec<Pos2> {
    let mut screen: Vec<Pos2> = points.iter().map(|p| view.pos(*p)).collect();
    if let Some(first) = screen.first().copied() {
        screen.push(first);
    }
    screen
}

pub fn draw_background(painter: &Painter, rect: Rect) {
    painter.rect_filled(rect, egui::CornerRadius::same(10), colors::GRASS);
    let stripe = 44.0;
    let mut y = rect.top() - rect.width();
    let mut index = 0;
    while y < rect.bottom() {
        if index % 2 == 0 {
            painter.rect_filled(
                Rect::from_min_size(Pos2::new(rect.left(), y), EVec2::new(rect.width(), stripe)),
                egui::CornerRadius::ZERO,
                colors::GRASS_STRIPE,
            );
        }
        y += stripe;
        index += 1;
    }
}

pub fn draw_track(painter: &Painter, view: &View, track: &Track) {
    // The road is a ring built from one quad per centerline segment, which
    // keeps variable widths, concave infields and hairpins all exact.
    let n = track.centerline.len();
    let mut mesh = egui::Mesh::default();
    for i in 0..n {
        let j = (i + 1) % n;
        let quad = [
            view.pos(track.outer[i]),
            view.pos(track.outer[j]),
            view.pos(track.inner[j]),
            view.pos(track.inner[i]),
        ];
        let base = mesh.vertices.len() as u32;
        for point in quad {
            mesh.colored_vertex(point, colors::ROAD);
        }
        mesh.indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
    painter.add(Shape::mesh(mesh));

    // The procedural learning course has a guide; real circuits have none.
    if track.circuit.is_none() {
        for shape in Shape::dashed_line(
            &closed(&track.centerline, view),
            Stroke::new(1.4, colors::CENTERLINE),
            12.0,
            16.0,
        ) {
            painter.add(shape);
        }
    }

    // Kerbs are outside the measured racing surface, so they cannot steal
    // width from the asphalt. Their dimensions use the same scale as the car.
    for (edge, index_offset) in [(&track.outer, 0usize), (&track.inner, 1usize)] {
        let mut kerb = edge.clone();
        for (i, point) in kerb.iter_mut().enumerate() {
            let outward = *point - track.centerline[i];
            *point = *point + outward * (0.45 * UNITS_PER_METRE / outward.norm());
        }
        let path = closed(&kerb, view);
        for (index, pair) in path.windows(2).enumerate() {
            let previous = track.centerline[(index + n - 1) % n];
            let here = track.centerline[index];
            let next = track.centerline[(index + 1) % n];
            let turn =
                neural_racing_core::wrap_angle((next - here).angle() - (here - previous).angle())
                    .abs();
            if track.circuit.is_some() && turn < 0.002 {
                continue;
            }
            let color = if (index + index_offset) % 2 == 0 {
                colors::KERB_RED
            } else {
                colors::KERB_LIGHT
            };
            painter.line_segment(
                [pair[0], pair[1]],
                Stroke::new(0.9 * UNITS_PER_METRE as f32 * view.scale, color),
            );
        }
        painter.add(Shape::line(
            closed(edge, view),
            Stroke::new(0.12 * UNITS_PER_METRE as f32 * view.scale, colors::WALL),
        ));
    }

    // Checkpoints, drawn across the local road width.
    for (index, gate) in track.gates().iter().enumerate() {
        let normal = Vec2 {
            x: -gate.tangent.y,
            y: gate.tangent.x,
        };
        let a = gate.center + normal * gate.half_width;
        let b = gate.center - normal * gate.half_width;
        painter.add(Shape::line(
            vec![view.pos(a), view.pos(b)],
            Stroke::new(1.8, colors::GATE),
        ));
        if view.scale > 0.15 {
            painter.text(
                view.pos(gate.center + normal * (gate.half_width * 0.70)),
                Align2::CENTER_CENTER,
                format!("{}", index + 1),
                FontId::proportional(11.0),
                colors::GATE_TEXT,
            );
        }
    }

    draw_start_line(painter, view, track);
}

fn draw_start_line(painter: &Painter, view: &View, track: &Track) {
    let tangent = Vec2 {
        x: track.start_heading.cos(),
        y: track.start_heading.sin(),
    };
    let normal = Vec2 {
        x: -tangent.y,
        y: tangent.x,
    };
    let half_width = track.half_width[0];
    let squares = 10;
    let depth = half_width * 0.30;
    for column in 0..squares {
        let from = -half_width + 2.0 * half_width * column as f64 / squares as f64;
        let to = -half_width + 2.0 * half_width * (column + 1) as f64 / squares as f64;
        let corners = [
            track.start + normal * from,
            track.start + normal * to,
            track.start + normal * to + tangent * depth,
            track.start + normal * from + tangent * depth,
        ];
        let fill = if column % 2 == 0 {
            colors::WALL
        } else {
            colors::DARK
        };
        painter.add(Shape::convex_polygon(
            corners.iter().map(|p| view.pos(*p)).collect(),
            fill,
            Stroke::NONE,
        ));
    }
}

pub fn draw_trail(painter: &Painter, view: &View, trail: &[Vec2]) {
    if trail.len() < 2 {
        return;
    }
    let points: Vec<Pos2> = trail.iter().map(|p| view.pos(*p)).collect();
    painter.add(Shape::line(points, Stroke::new(2.0, colors::TRAIL)));
}

pub fn draw_rays(painter: &Painter, view: &View, env: &Env) {
    let origin = view.pos(env.position());
    let count = env.rays.len() as f32;
    for (index, distance) in env.rays.iter().enumerate() {
        let spread = -0.6 * std::f64::consts::PI
            + 1.2 * std::f64::consts::PI * index as f64 / (count as f64 - 1.0);
        let angle = env.heading + spread;
        let tip = Vec2 {
            x: env.x + angle.cos() * distance,
            y: env.y + angle.sin() * distance,
        };
        let closeness = 1.0 - (*distance / neural_racing_core::RAY_RANGE).clamp(0.0, 1.0) as f32;
        let color = Color32::from_rgba_unmultiplied(
            (255.0 * closeness + 40.0) as u8,
            (230.0 - 120.0 * closeness) as u8,
            90,
            70 + (120.0 * closeness) as u8,
        );
        painter.add(Shape::line(
            vec![origin, view.pos(tip)],
            Stroke::new(1.0, color),
        ));
        painter.circle_filled(view.pos(tip), 2.0, color);
    }
}

/// Wheel patches use their own steering angle about the axle, independently
/// of the chassis heading. The right/left angles satisfy Ackermann geometry.
fn wheel_corners(env: &Env, front: bool, side: f64) -> [Vec2; 4] {
    let (left, right) = neural_racing_core::vehicle::front_wheel_angles(env.steer_angle);
    let steering = if front {
        if side < 0.0 {
            left
        } else {
            right
        }
    } else {
        0.0
    };
    let axle = if front { 0.5 } else { -0.5 } * neural_racing_core::WHEELBASE;
    let lateral = side
        * neural_racing_core::vehicle::FRONT_TRACK_METRES
        * 0.5
        * neural_racing_core::UNITS_PER_METRE;
    let center = env.position()
        + Vec2 {
            x: axle,
            y: lateral,
        }
        .rotated(env.heading);
    let half_length = 0.34 * neural_racing_core::UNITS_PER_METRE;
    let half_width = 0.15 * neural_racing_core::UNITS_PER_METRE;
    [
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ]
    .map(|(x, y)| center + Vec2 { x, y }.rotated(env.heading + steering))
}

pub fn draw_car(painter: &Painter, view: &View, env: &Env, crashed: bool) {
    let cos = env.heading.cos();
    let sin = env.heading.sin();
    let local = |forward: f64, right: f64| Vec2 {
        x: env.x + forward * cos - right * sin,
        y: env.y + forward * sin + right * cos,
    };
    let body_color = if crashed { colors::BAD } else { colors::CAR };
    let half_l = CAR_LENGTH * 0.5;
    let half_w = CAR_WIDTH * 0.5;
    let body = [
        local(half_l, half_w * 0.85),
        local(half_l, -half_w * 0.85),
        local(half_l * 0.6, -half_w),
        local(-half_l, -half_w),
        local(-half_l, half_w),
        local(half_l * 0.6, half_w),
    ];
    painter.add(Shape::convex_polygon(
        body.iter().map(|p| view.pos(*p)).collect(),
        body_color,
        Stroke::NONE,
    ));
    let cabin = [
        local(half_l * 0.35, half_w * 0.73),
        local(half_l * 0.35, -half_w * 0.73),
        local(-half_l * 0.5, -half_w * 0.66),
        local(-half_l * 0.5, half_w * 0.66),
    ];
    painter.add(Shape::convex_polygon(
        cabin.iter().map(|p| view.pos(*p)).collect(),
        colors::CABIN,
        Stroke::NONE,
    ));
    // Closed-wheel GT car: tires and rear wing stay inside its physical envelope.
    for front in [false, true] {
        for side in [-1.0, 1.0] {
            let tire = wheel_corners(env, front, side);
            painter.add(Shape::convex_polygon(
                tire.iter().map(|p| view.pos(*p)).collect(),
                colors::TIRE,
                Stroke::new(0.65 * view.scale, colors::KERB_LIGHT),
            ));
        }
    }
    painter.line_segment(
        [
            view.pos(local(-half_l * 0.86, -half_w)),
            view.pos(local(-half_l * 0.86, half_w)),
        ],
        Stroke::new(2.0 * view.scale, colors::TIRE),
    );
    for r in [-0.65, 0.65] {
        painter.line_segment(
            [
                view.pos(local(half_l * 0.9, r * half_w - 1.5)),
                view.pos(local(half_l * 0.9, r * half_w + 1.5)),
            ],
            Stroke::new(1.5 * view.scale, colors::KERB_LIGHT),
        );
    }
}

/// Overview locator is a map symbol, not an enlarged car sprite.
pub fn draw_minimap(painter: &Painter, rect: Rect, env: &Env) {
    painter.rect_filled(rect, 8, colors::DARK);
    let view = View::fit(rect.shrink(16.0), env.track.bounds, 20.0);
    painter.add(Shape::line(
        closed(&env.track.centerline, &view),
        Stroke::new(2.0, colors::MUTED),
    ));
    painter.circle_filled(view.pos(env.track.start), 3.0, colors::KERB_LIGHT);
    painter.circle_filled(view.pos(env.position()), 4.0, colors::ACCENT);
    painter.text(
        rect.left_bottom() + EVec2::new(8.0, -4.0),
        Align2::LEFT_BOTTOM,
        "CIRCUIT MAP · V overview",
        FontId::monospace(9.0),
        colors::MUTED,
    );
}

pub fn draw_skids(painter: &Painter, view: &View, skids: &[(Vec2, f64)]) {
    for (point, heading) in skids {
        let cos = heading.cos();
        let sin = heading.sin();
        let left = Vec2 {
            x: point.x - 8.0 * cos,
            y: point.y - 8.0 * sin,
        };
        let right = Vec2 {
            x: point.x - 13.0 * cos,
            y: point.y - 13.0 * sin,
        };
        painter.add(Shape::line(
            vec![view.pos(left), view.pos(right)],
            Stroke::new(3.0, colors::SKID),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use neural_racing_core::Circuit;

    #[test]
    fn fit_centers_and_contains_every_circuit_at_both_viewport_sizes() {
        for size in [EVec2::new(820.0, 780.0), EVec2::new(560.0, 600.0)] {
            let rect = Rect::from_min_size(Pos2::new(17.0, 63.0), size);
            for circuit in Circuit::ALL {
                let track = circuit.build();
                let view = View::fit(rect, track.bounds, 34.0);
                let center = Vec2 {
                    x: (track.bounds.min_x + track.bounds.max_x) * 0.5,
                    y: (track.bounds.min_y + track.bounds.max_y) * 0.5,
                };
                assert!(view.pos(center).distance(rect.center()) < 0.001);
                for point in track.outer.iter().chain(&track.inner) {
                    assert!(rect.contains(view.pos(*point)));
                }
                let follow = View::follow(rect, track.start);
                assert_eq!(follow.pos(track.start), rect.center());
                let px_car = follow
                    .pos(
                        track.start
                            + Vec2 {
                                x: CAR_WIDTH,
                                y: 0.0,
                            },
                    )
                    .x
                    - follow.pos(track.start).x;
                let px_road = follow
                    .pos(
                        track.start
                            + Vec2 {
                                x: circuit.width_metres() * UNITS_PER_METRE,
                                y: 0.0,
                            },
                    )
                    .x
                    - follow.pos(track.start).x;
                assert!((px_car / px_road - (2.040 / circuit.width_metres()) as f32).abs() < 1e-5);
            }
        }
    }
}

#[cfg(test)]
mod wheel_tests {
    use super::*;
    #[test]
    fn front_wheel_meshes_steer_while_rear_meshes_follow_the_body() {
        let mut env = neural_racing_core::Env::new(neural_racing_core::Circuit::Monza.build(), 1);
        env.steer_angle = 0.4;
        let (left, right) = neural_racing_core::vehicle::front_wheel_angles(env.steer_angle);
        assert!(right > left && left > 0.0);
        for (side, angle) in [(-1.0, left), (1.0, right)] {
            let front = wheel_corners(&env, true, side);
            let rear = wheel_corners(&env, false, side);
            assert!(
                neural_racing_core::wrap_angle((front[1] - front[0]).angle() - env.heading - angle)
                    .abs()
                    < 1e-8
            );
            assert!(
                neural_racing_core::wrap_angle((rear[1] - rear[0]).angle() - env.heading).abs()
                    < 1e-8
            );
        }
    }
}
