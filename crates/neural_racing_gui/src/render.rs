//! Track, car and sensor painting for the race view.

use egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2 as EVec2};
use neural_racing_core::{Env, Track, Vec2};

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
        let scale = (rect.width() / width).min(rect.height() / height).max(0.05);
        let center = Vec2 {
            x: (bounds.min_x + bounds.max_x) * 0.5,
            y: (bounds.min_y + bounds.max_y) * 0.5,
        };
        let origin = Pos2::new(
            rect.center().x - width * scale * 0.5,
            rect.center().y - height * scale * 0.5,
        );
        Self {
            origin,
            scale,
            world_min: Vec2 {
                x: center.x - width as f64 * 0.5 / scale as f64,
                y: center.y - height as f64 * 0.5 / scale as f64,
            },
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

    // Dashed centerline.
    for shape in Shape::dashed_line(
        &closed(&track.centerline, view),
        Stroke::new(1.4, colors::CENTERLINE),
        12.0,
        16.0,
    ) {
        painter.add(shape);
    }

    // Kerbs and walls follow both edges, alternating red and white.
    for (edge, index_offset) in [(&track.outer, 0usize), (&track.inner, 1usize)] {
        let path = closed(edge, view);
        painter.add(Shape::line(path.clone(), Stroke::new(6.0, colors::KERB)));
        for (index, pair) in path.windows(2).enumerate() {
            let color = if (index + index_offset) % 2 == 0 {
                colors::KERB_RED
            } else {
                colors::KERB_LIGHT
            };
            painter.add(Shape::line(vec![pair[0], pair[1]], Stroke::new(5.0, color)));
        }
        painter.add(Shape::line(path, Stroke::new(1.6, colors::WALL)));
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
        painter.text(
            view.pos(gate.center + normal * (gate.half_width * 0.70)),
            Align2::CENTER_CENTER,
            format!("{}", index + 1),
            FontId::proportional(11.0),
            colors::GATE_TEXT,
        );
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
        let spread = -0.6 * std::f64::consts::PI + 1.2 * std::f64::consts::PI
            * index as f64
            / (count as f64 - 1.0);
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

pub fn draw_car(painter: &Painter, view: &View, env: &Env, crashed: bool) {
    let cos = env.heading.cos();
    let sin = env.heading.sin();
    let local = |forward: f64, right: f64| Vec2 {
        x: env.x + forward * cos - right * sin,
        y: env.y + forward * sin + right * cos,
    };
    let body_color = if crashed { colors::BAD } else { colors::CAR };
    let body = [local(17.0, 9.0), local(17.0, -9.0), local(-17.0, -9.0), local(-17.0, 9.0)];
    painter.add(Shape::convex_polygon(
        body.iter().map(|p| view.pos(*p)).collect(),
        body_color,
        Stroke::new(1.0, colors::CAR_EDGE),
    ));
    let cabin = [local(5.0, 6.0), local(5.0, -6.0), local(-7.0, -6.0), local(-7.0, 6.0)];
    painter.add(Shape::convex_polygon(
        cabin.iter().map(|p| view.pos(*p)).collect(),
        colors::CABIN,
        Stroke::NONE,
    ));
    for (forward, right) in [(10.0, 10.0), (10.0, -10.0), (-10.0, 10.0), (-10.0, -10.0)] {
        painter.circle_filled(view.pos(local(forward, right)), 3.2, colors::TIRE);
    }
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
