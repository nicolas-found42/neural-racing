//! Track, car and sensor painting for the race view.

use egui::{Align2, Color32, FontId, Painter, Pos2, Rect, Shape, Stroke, Vec2 as EVec2};
use neural_racing_core::{Env, Track, Vec2, CAR_LENGTH, CAR_WIDTH, UNITS_PER_METRE};

use crate::palette as colors;

/// Existing GT4 M3 GTR collection frames, preserving their saved camera scale.
/// Transparent padding is cropped without moving the native model origin.
pub struct CarSprite {
    frames: Vec<SpriteFrame>,
}
struct SpriteFrame {
    texture: egui::TextureHandle,
    uv: Rect,
    local_corners: [Vec2; 4],
}
impl CarSprite {
    pub fn original_m3(ctx: &egui::Context) -> Result<Self, String> {
        let sources: [&[u8]; 13] = [
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer-060.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer-050.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer-040.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer-030.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer-020.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer-010.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer+010.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer+020.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer+030.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer+040.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer+050.png"),
            include_bytes!("../assets/gt4/viewer-bakes/GT4_BMW_0003-source-steer+060.png"),
        ];
        // Saved orthographic frame/model bounds, corroborated by source alpha
        // projections. The body origin is not the image/bounds midpoint.
        let pixels_per_metre = 512.0 / 4.769063949584961;
        let centre_forward = (-2.3995256423950195 + 2.2205052375793457) * 0.5;
        let origin = [256.0, 256.0 + centre_forward * pixels_per_metre];
        let mut frames = Vec::with_capacity(13);
        for bytes in sources {
            let image = image::load_from_memory(bytes)
                .map_err(|e| e.to_string())?
                .into_rgba8();
            let size = [image.width() as usize, image.height() as usize];
            if size != [512, 512] {
                return Err("existing M3 pose frame differs from saved orthographic camera".into());
            }
            frames.push(
                SpriteFrame::from_rgba(ctx, size, image.as_raw(), origin, pixels_per_metre)
                    .map_err(str::to_owned)?,
            );
        }
        Ok(Self { frames })
    }
    fn pose_index(&self, steer: f64) -> usize {
        ((-steer.to_degrees() / 10.0).round() + 6.0).clamp(0.0, 12.0) as usize
    }
    fn frame(&self, steer: f64) -> &SpriteFrame {
        &self.frames[self.pose_index(steer)]
    }
}
impl SpriteFrame {
    fn from_rgba(
        ctx: &egui::Context,
        size: [usize; 2],
        rgba: &[u8],
        origin_pixel: [f64; 2],
        pixels_per_metre: f64,
    ) -> Result<Self, &'static str> {
        if !pixels_per_metre.is_finite()
            || pixels_per_metre <= 0.0
            || !origin_pixel.iter().all(|v| v.is_finite())
        {
            return Err("invalid sprite physical calibration");
        }
        let pixels = size[0]
            .checked_mul(size[1])
            .ok_or("sprite dimensions overflow")?;
        if size.contains(&0) || pixels.checked_mul(4) != Some(rgba.len()) {
            return Err("invalid RGBA sprite dimensions");
        }
        let mut bounds = [size[0], size[1], 0, 0];
        for (i, pixel) in rgba.chunks_exact(4).enumerate() {
            if pixel[3] > 0 {
                let x = i % size[0];
                let y = i / size[0];
                bounds[0] = bounds[0].min(x);
                bounds[1] = bounds[1].min(y);
                bounds[2] = bounds[2].max(x + 1);
                bounds[3] = bounds[3].max(y + 1);
            }
        }
        if bounds[2] == 0 {
            return Err("sprite contains no visible model");
        }
        let uv = Rect::from_min_max(
            Pos2::new(
                bounds[0] as f32 / size[0] as f32,
                bounds[1] as f32 / size[1] as f32,
            ),
            Pos2::new(
                bounds[2] as f32 / size[0] as f32,
                bounds[3] as f32 / size[1] as f32,
            ),
        );
        let texture = ctx.load_texture(
            "original-m3-top-down",
            egui::ColorImage::from_rgba_unmultiplied(size, rgba),
            egui::TextureOptions::LINEAR,
        );
        let local_corners = [
            (bounds[0], bounds[1]),
            (bounds[2], bounds[1]),
            (bounds[2], bounds[3]),
            (bounds[0], bounds[3]),
        ]
        .map(|(x, y)| {
            let x = (x as f64 - origin_pixel[0]) / pixels_per_metre;
            let y = (y as f64 - origin_pixel[1]) / pixels_per_metre;
            let (forward, right) = (-y, x); // Saved existing sprites face image-up.
            Vec2 {
                x: forward * UNITS_PER_METRE,
                y: right * UNITS_PER_METRE,
            }
        });
        Ok(Self {
            texture,
            uv,
            local_corners,
        })
    }
}

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

    /// Screen points per metre, the same scale for the car and the ground.
    pub fn points_per_metre(&self) -> f64 {
        f64::from(self.scale) * UNITS_PER_METRE
    }

    /// World-space corners of a screen rectangle, for culling and tile requests.
    pub fn world_bounds(&self, rect: Rect) -> (Vec2, Vec2) {
        let scale = f64::from(self.scale).max(f64::MIN_POSITIVE);
        let world = |point: Pos2| Vec2 {
            x: self.world_min.x + f64::from(point.x - self.origin.x) / scale,
            y: self.world_min.y + f64::from(point.y - self.origin.y) / scale,
        };
        (world(rect.min), world(rect.max))
    }
}

fn road_path(points: &[Vec2], view: &View, closed: bool) -> Vec<Pos2> {
    let mut screen: Vec<Pos2> = points.iter().map(|p| view.pos(*p)).collect();
    if closed {
        if let Some(first) = screen.first().copied() {
            screen.push(first);
        }
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

fn practice_ground(track: &Track, view: &View) -> Option<egui::Mesh> {
    let triangles = track.practice_ground_triangles();
    if triangles.is_empty() {
        return None;
    }
    let mut mesh = egui::Mesh::default();
    for triangle in triangles {
        let base = mesh.vertices.len() as u32;
        for point in triangle {
            mesh.colored_vertex(view.pos(*point), colors::ROAD);
        }
        mesh.indices.extend([base, base + 1, base + 2]);
    }
    Some(mesh)
}

/// Neutral original ground for practice areas while source tiles load.
pub(crate) fn draw_practice_ground(painter: &Painter, view: &View, track: &Track) -> bool {
    match practice_ground(track, view) {
        Some(mesh) => {
            painter.add(Shape::mesh(mesh));
            true
        }
        None => false,
    }
}

pub(crate) fn draw_neutral_source_ground(painter: &Painter, view: &View, track: &Track) -> bool {
    let Some(geometry) = track.road_render_geometry() else {
        return false;
    };
    let mut mesh = egui::Mesh::default();
    for item in &geometry.order {
        match *item {
            neural_racing_core::RoadRenderItem::Triangle(index) => {
                let triangle = &geometry.triangles[index];
                if !Rect::from_points(&triangle.vertices.map(|v| view.pos(v.position)))
                    .intersects(painter.clip_rect())
                {
                    continue;
                }
                // Numeric source materials are retained in core; neutral shading
                // here avoids assigning unsupported names to those identifiers.
                let color = if triangle.racing_surface {
                    colors::ROAD
                } else {
                    Color32::from_rgb(64, 68, 65)
                };
                let base = mesh.vertices.len() as u32;
                for vertex in triangle.vertices {
                    mesh.colored_vertex(view.pos(vertex.position), color);
                }
                mesh.indices.extend([base, base + 1, base + 2]);
            }
            neural_racing_core::RoadRenderItem::Boundary(index) => {
                if !mesh.indices.is_empty() {
                    painter.add(Shape::mesh(std::mem::take(&mut mesh)));
                }
                let boundary = &geometry.boundaries[index];
                painter.line_segment(
                    boundary.vertices.map(|v| view.pos(v.position)),
                    Stroke::new(
                        (0.08 * UNITS_PER_METRE as f32 * view.scale).max(0.4),
                        Color32::from_rgb(142, 145, 141),
                    ),
                );
            }
        }
    }
    if !mesh.indices.is_empty() {
        painter.add(Shape::mesh(mesh));
    }
    true
}

pub fn draw_track(painter: &Painter, view: &View, track: &Track) {
    if crate::ground_tiles::draw(painter, view, track) {
        return;
    }
    if track.free_roam {
        // Actual source ground, not the short navigation hint or a synthetic boundary.
        draw_practice_ground(painter, view, track);
        return;
    }
    if draw_neutral_source_ground(painter, view, track) {
        return;
    }
    // Build one quad per actual segment; open routes never connect their endpoints. This
    // keeps variable widths, concave infields and hairpins all exact.
    let n = track.centerline.len();
    let mut mesh = egui::Mesh::default();
    for i in 0..if track.closed { n } else { n.saturating_sub(1) } {
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
            &road_path(&track.centerline, view, track.closed),
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
        let path = road_path(&kerb, view, track.closed);
        for (index, pair) in path.windows(2).enumerate() {
            let previous = track.centerline[if track.closed {
                (index + n - 1) % n
            } else {
                index.saturating_sub(1)
            }];
            let here = track.centerline[index];
            let next = track.centerline[if track.closed {
                (index + 1) % n
            } else {
                (index + 1).min(n - 1)
            }];
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
            road_path(edge, view, track.closed),
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
    // Decorations follow both actual edges, not a symmetric half-width proxy.
    let right = track.inner[0];
    let left = track.outer[0];
    let width = (left - right).norm();
    let squares = (width / (0.4 * UNITS_PER_METRE)).ceil().clamp(2.0, 256.0) as usize;
    let depth = 0.4 * UNITS_PER_METRE;
    for column in 0..squares {
        let from = right + (left - right) * (column as f64 / squares as f64);
        let to = right + (left - right) * ((column + 1) as f64 / squares as f64);
        let corners = [from, to, to + tangent * depth, from + tangent * depth];
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
    let track = if front {
        neural_racing_core::vehicle::FRONT_TRACK_METRES
    } else {
        neural_racing_core::vehicle::REAR_TRACK_METRES
    };
    let lateral = side * track * 0.5 * neural_racing_core::UNITS_PER_METRE;
    let center = env.position()
        + Vec2 {
            x: axle,
            y: lateral,
        }
        .rotated(env.heading);
    let half_length = if front { 0.3231 } else { 0.3366 } * neural_racing_core::UNITS_PER_METRE;
    let half_width = 0.135 * neural_racing_core::UNITS_PER_METRE;
    [
        (-half_length, -half_width),
        (half_length, -half_width),
        (half_length, half_width),
        (-half_length, half_width),
    ]
    .map(|(x, y)| center + Vec2 { x, y }.rotated(env.heading + steering))
}

fn sprite_corners(env: &Env, sprite: &SpriteFrame) -> [Vec2; 4] {
    sprite
        .local_corners
        .map(|point| env.position() + point.rotated(env.heading))
}

pub fn draw_car_with_sprite(
    painter: &Painter,
    view: &View,
    env: &Env,
    crashed: bool,
    sprite: Option<&CarSprite>,
) {
    let Some(sprite) = sprite else {
        draw_car(painter, view, env, crashed);
        return;
    };
    let sprite = sprite.frame(env.steer_angle);
    let mut mesh = egui::Mesh::with_texture(sprite.texture.id());
    // The supplied collection frames already contain original wheels/tires.
    let canonical = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    for (position, (u, v)) in sprite_corners(env, sprite).into_iter().zip(canonical) {
        mesh.vertices.push(egui::epaint::Vertex {
            pos: view.pos(position),
            uv: Pos2::new(
                sprite.uv.min.x + sprite.uv.width() * u,
                sprite.uv.min.y + sprite.uv.height() * v,
            ),
            color: if crashed { colors::BAD } else { Color32::WHITE },
        });
    }
    mesh.indices.extend([0, 1, 2, 0, 2, 3]);
    painter.add(Shape::mesh(mesh));
}

pub fn draw_car(painter: &Painter, view: &View, env: &Env, crashed: bool) {
    let cos = env.heading.cos();
    let sin = env.heading.sin();
    let local = |forward: f64, right: f64| Vec2 {
        x: env.x + forward * cos - right * sin,
        y: env.y + forward * sin + right * cos,
    };
    let body_color = if crashed { colors::BAD } else { colors::CAR };
    let front = 2.220 * UNITS_PER_METRE;
    let rear = CAR_LENGTH - front;
    let half_w = CAR_WIDTH * 0.5;
    let body = [
        local(front, half_w * 0.85),
        local(front, -half_w * 0.85),
        local(front * 0.6, -half_w),
        local(-rear, -half_w),
        local(-rear, half_w),
        local(front * 0.6, half_w),
    ];
    painter.add(Shape::convex_polygon(
        body.iter().map(|p| view.pos(*p)).collect(),
        body_color,
        Stroke::NONE,
    ));
    let cabin = [
        local(front * 0.35, half_w * 0.73),
        local(front * 0.35, -half_w * 0.73),
        local(-rear * 0.5, -half_w * 0.66),
        local(-rear * 0.5, half_w * 0.66),
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
            view.pos(local(-rear * 0.86, -half_w)),
            view.pos(local(-rear * 0.86, half_w)),
        ],
        Stroke::new(2.0 * view.scale, colors::TIRE),
    );
    for r in [-0.65, 0.65] {
        painter.line_segment(
            [
                view.pos(local(front * 0.9, r * half_w - 1.5)),
                view.pos(local(front * 0.9, r * half_w + 1.5)),
            ],
            Stroke::new(1.5 * view.scale, colors::KERB_LIGHT),
        );
    }
}

/// Overview locator is a map symbol, not an enlarged car sprite.
pub fn draw_minimap(painter: &Painter, rect: Rect, env: &Env) {
    painter.rect_filled(rect, 8, colors::DARK);
    let view = View::fit(rect.shrink(16.0), env.track.bounds, 20.0);
    if env.track.free_roam {
        draw_practice_ground(painter, &view, &env.track);
    } else {
        painter.add(Shape::line(
            road_path(&env.track.centerline, &view, env.track.closed),
            Stroke::new(2.0, colors::MUTED),
        ));
    }
    painter.circle_filled(view.pos(env.track.start), 3.0, colors::KERB_LIGHT);
    painter.circle_filled(view.pos(env.position()), 4.0, colors::ACCENT);
    painter.text(
        rect.left_bottom() + EVec2::new(8.0, -4.0),
        Align2::LEFT_BOTTOM,
        if env.track.free_roam {
            "PRACTICE MAP · V overview"
        } else {
            "ROUTE MAP · V overview"
        },
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
                assert!(
                    (px_car / px_road
                        - (CAR_WIDTH / UNITS_PER_METRE / circuit.width_metres()) as f32)
                        .abs()
                        < 1e-5
                );
            }
        }
    }
}

#[cfg(test)]
mod wheel_tests {
    use super::*;
    use neural_racing_core::Circuit;
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
    #[test]
    fn open_routes_do_not_paint_an_end_to_start_road_segment() {
        let ctx = egui::Context::default();
        let mut track = Circuit::Monza.build();
        track.closed = false;
        let mut output = ctx.run_ui(egui::RawInput::default(), |root| {
            egui::CentralPanel::default().show(root, |ui| {
                let rect = ui.available_rect_before_wrap();
                let view = View::fit(rect, track.bounds, 0.0);
                draw_track(ui.painter(), &view, &track);
            });
        });
        output.textures_delta.clear();
        let road = output
            .shapes
            .iter()
            .find_map(|s| match &s.shape {
                Shape::Mesh(m) if m.vertices.first().is_some_and(|v| v.color == colors::ROAD) => {
                    Some(m)
                }
                _ => None,
            })
            .expect("road mesh painted");
        assert_eq!(road.indices.len(), (track.centerline.len() - 1) * 6);
        let first = road.vertices[0].pos;
        let last = road.vertices[road.vertices.len() - 3].pos;
        assert!(
            road.indices.chunks_exact(3).all(|triangle| {
                let points: Vec<_> = triangle
                    .iter()
                    .map(|i| road.vertices[*i as usize].pos)
                    .collect();
                !(points.contains(&first) && points.contains(&last))
            }),
            "painted an invented closing triangle"
        );
    }

    #[test]
    fn practice_areas_paint_no_checkpoint_or_finish_shapes() {
        let track = Circuit::all()
            .iter()
            .find(|c| c.is_practice())
            .unwrap()
            .build();
        assert!(!track.practice_ground_triangles().is_empty());
        let canvas = Rect::from_min_size(Pos2::ZERO, EVec2::new(800.0, 600.0));
        let view = View::fit(canvas, track.bounds, 0.0);
        let ctx = egui::Context::default();
        let outputs = run_source_frames(
            &ctx,
            canvas,
            |painter| draw_track(painter, &view, &track),
            "practice",
        );
        let output = outputs.last().unwrap();
        assert!(!output.shapes.is_empty());
        assert!(
            output
                .shapes
                .iter()
                .all(|s| matches!(s.shape, Shape::Mesh(_))),
            "practice source ground must have no checkpoint, text or finish overlays"
        );
        if track.source_visual().unwrap().is_none() {
            let Shape::Mesh(ground) = &output.shapes[0].shape else {
                unreachable!()
            };
            assert_eq!(
                ground.indices.len(),
                track.practice_ground_triangles().len() * 3
            );
        }
    }
    #[test]
    fn m3_body_envelope_uses_verified_physical_dimensions() {
        assert!((CAR_LENGTH / UNITS_PER_METRE - 4.619).abs() < 0.001);
        assert!((CAR_WIDTH / UNITS_PER_METRE - 1.879).abs() < 0.001);
    }

    #[test]
    fn start_paint_stays_between_asymmetric_edges_and_has_physical_depth() {
        let track = Circuit::lookup("gt4-highspeedring").unwrap().build();
        let view = View {
            origin: Pos2::ZERO,
            scale: 1.0,
            world_min: track.start,
        };
        let ctx = egui::Context::default();
        let mut output = ctx.run_pass(egui::RawInput::default(), |ctx| {
            draw_start_line(
                &ctx.layer_painter(egui::LayerId::background()),
                &view,
                &track,
            );
        });
        output.textures_delta.clear();
        let tangent = Vec2 {
            x: track.start_heading.cos(),
            y: track.start_heading.sin(),
        };
        let normal = Vec2 {
            x: -tangent.y,
            y: tangent.x,
        };
        let points: Vec<_> = output
            .shapes
            .iter()
            .filter_map(|s| match &s.shape {
                Shape::Path(p) => Some(&p.points),
                _ => None,
            })
            .flatten()
            .map(|p| Vec2 {
                x: f64::from(p.x),
                y: f64::from(p.y),
            })
            .collect();
        assert!(!points.is_empty());
        let min = |axis: Vec2| {
            points
                .iter()
                .map(|p| p.dot(axis))
                .fold(f64::INFINITY, f64::min)
        };
        let max = |axis: Vec2| {
            points
                .iter()
                .map(|p| p.dot(axis))
                .fold(f64::NEG_INFINITY, f64::max)
        };
        assert!(
            (max(tangent) - min(tangent) - 0.4 * UNITS_PER_METRE).abs() < 0.01,
            "start stripe depth_m={}",
            (max(tangent) - min(tangent)) / UNITS_PER_METRE
        );
        assert!(
            (min(normal) - (track.inner[0] - track.start).dot(normal)).abs() < 0.01,
            "stripe missed right edge"
        );
        assert!(
            (max(normal) - (track.outer[0] - track.start).dot(normal)).abs() < 0.01,
            "stripe missed left edge"
        );
    }
    #[test]
    fn sprite_geometry_crops_alpha_and_anchors_the_native_origin() {
        let ctx = egui::Context::default();
        let poses = CarSprite::original_m3(&ctx).unwrap();
        assert_eq!(poses.frames.len(), 13);
        assert_eq!(poses.pose_index(0.0), 6);
        assert_eq!(
            poses.pose_index(20f64.to_radians()),
            4,
            "simulation right steer uses collection negative pose"
        );
        assert_eq!(poses.pose_index((-20f64).to_radians()), 8);
        let sprite = poses.frame(0.0);
        assert!((sprite.uv.min.x - 153.0 / 512.0).abs() < 1e-6);
        assert!((sprite.uv.max.x - 359.0 / 512.0).abs() < 1e-6);
        let mut env = Env::new(Circuit::Monza.build(), 1);
        env.x = 0.0;
        env.y = 0.0;
        env.heading = 0.0;
        let corners = sprite_corners(&env, sprite);
        let mpp = 4.769063949584961 / 512.0;
        let origin_y = 256.0 + (-2.3995256423950195 + 2.2205052375793457) * 0.5 / mpp;
        assert!((corners[0].x / UNITS_PER_METRE - (origin_y - 7.0) * mpp).abs() < 1e-9);
        assert!((corners[2].x / UNITS_PER_METRE - (origin_y - 504.0) * mpp).abs() < 1e-9);
        assert!((corners[0].y / UNITS_PER_METRE - (153.0 - 256.0) * mpp).abs() < 1e-9);
        assert!(
            ((corners[1] - corners[0]).norm() / UNITS_PER_METRE - 206.0 * mpp).abs() < 1e-9,
            "saved camera scale, not alpha fit"
        );
        env.heading = std::f64::consts::FRAC_PI_2;
        let rotated = sprite_corners(&env, sprite);
        for (a, b) in corners.iter().zip(rotated) {
            assert!((b.y - a.x).abs() < 1e-9);
            assert!((b.x + a.y).abs() < 1e-9);
        }
    }

    fn run_source_frames(
        ctx: &egui::Context,
        canvas: Rect,
        mut draw: impl FnMut(&egui::Painter),
        label: &str,
    ) -> Vec<egui::FullOutput> {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(240);
        let mut outputs = Vec::new();
        loop {
            let output = ctx.run_pass(
                egui::RawInput {
                    screen_rect: Some(canvas),
                    ..Default::default()
                },
                |ctx| {
                    let mut painter = ctx.layer_painter(egui::LayerId::background());
                    painter.set_clip_rect(canvas);
                    draw(&mut painter);
                },
            );
            let pending = crate::ground_tiles::pending(ctx);
            let mut output = output;
            output.textures_delta.clear();
            outputs.push(output);
            if pending == Some(0) {
                return outputs;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "{label}: source ground tiles still pending {pending:?}"
            );
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    fn tile_rects(outputs: &[egui::FullOutput]) -> Vec<Rect> {
        outputs
            .last()
            .map(|output| {
                output
                    .shapes
                    .iter()
                    .filter_map(|clipped| match &clipped.shape {
                        Shape::Mesh(mesh) if mesh.texture_id != egui::TextureId::default() => {
                            let mut rect = Rect::NOTHING;
                            for vertex in &mesh.vertices {
                                rect.extend_with(vertex.pos);
                            }
                            Some(rect)
                        }
                        _ => None,
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    #[test]
    fn imported_courses_draw_depth_composited_ground_tiles_over_the_course() {
        for (id, follow) in [("gt4-suzuka", true), ("gt4-nurburgring", false)] {
            let track = Circuit::lookup(id).unwrap().build();
            let canvas = Rect::from_min_size(Pos2::ZERO, EVec2::new(1200.0, 800.0));
            let view = if follow {
                View::follow(canvas, track.start)
            } else {
                View::fit(canvas.shrink(20.0), track.bounds, 20.0)
            };
            let ctx = egui::Context::default();
            let outputs = run_source_frames(
                &ctx,
                canvas,
                |painter| draw_track(painter, &view, &track),
                id,
            );
            let output = outputs.last().unwrap();
            let tile_rects = tile_rects(&outputs);
            assert!(
                !tile_rects.is_empty(),
                "{id}: no textured ground tiles drawn"
            );
            for shape in &output.shapes {
                if let Shape::Text(text) = &shape.shape {
                    assert!(
                        !text.galley.text().contains("Source visual unavailable"),
                        "{id}: source ground failed: {}",
                        text.galley.text()
                    );
                }
            }
            // The visible course must be covered by exact tiles, not by the
            // neutral fallback or the old mean-height face stream.
            let bounds = Rect::from_points(&[
                view.pos(Vec2 {
                    x: track.bounds.min_x,
                    y: track.bounds.min_y,
                }),
                view.pos(Vec2 {
                    x: track.bounds.max_x,
                    y: track.bounds.max_y,
                }),
            ])
            .intersect(canvas);
            if follow {
                let here = view.pos(track.start);
                for dx in -3..=3 {
                    for dy in -3..=3 {
                        let point = Pos2::new(here.x + dx as f32 * 60.0, here.y + dy as f32 * 60.0);
                        if canvas.contains(point) {
                            assert!(
                                tile_rects.iter().any(|rect| rect.contains(point)),
                                "{id}: no ground tile at {point:?}"
                            );
                        }
                    }
                }
            } else {
                for i in 0..=20 {
                    for j in 0..=20 {
                        let point = Pos2::new(
                            bounds.left() + bounds.width() * i as f32 / 20.0,
                            bounds.top() + bounds.height() * j as f32 / 20.0,
                        );
                        assert!(
                            tile_rects.iter().any(|rect| rect.contains(point)),
                            "{id}: overview hole at {point:?}"
                        );
                    }
                }
            }
        }
    }

    /// Zoom, viewport size and course switches all change the wanted tile
    /// level and generation. The cache must converge each time and stay within
    /// its documented bounds instead of keeping every level ever requested.
    #[test]
    fn tile_cache_converges_across_zoom_viewport_and_course_switch() {
        let ctx = egui::Context::default();
        let large = Rect::from_min_size(Pos2::ZERO, EVec2::new(1600.0, 1000.0));
        let small = Rect::from_min_size(Pos2::ZERO, EVec2::new(400.0, 300.0));
        let suzuka = Circuit::lookup("gt4-suzuka").unwrap().build();
        let nurburgring = Circuit::lookup("gt4-nurburgring").unwrap().build();
        let mut views = vec![
            (
                "suzuka overview",
                &suzuka,
                View::fit(large.shrink(20.0), suzuka.bounds, 20.0),
                large,
            ),
            (
                "suzuka follow",
                &suzuka,
                View::follow(large, suzuka.start),
                large,
            ),
            (
                "nurburgring overview",
                &nurburgring,
                View::fit(large.shrink(20.0), nurburgring.bounds, 20.0),
                large,
            ),
            (
                "nurburgring small follow",
                &nurburgring,
                View::follow(small, nurburgring.start),
                small,
            ),
        ];
        // Rapid level changes while requests are still in flight must not
        // strand keys in the request set: the worker skips a level the camera
        // has left, and only the drawn level may stay requested.
        let pass = |track: &Track, view: View, canvas: Rect| {
            let mut output = ctx.run_pass(
                egui::RawInput {
                    screen_rect: Some(canvas),
                    ..Default::default()
                },
                |ctx| {
                    let mut painter = ctx.layer_painter(egui::LayerId::background());
                    painter.set_clip_rect(canvas);
                    draw_track(&mut painter, &view, track);
                },
            );
            output.textures_delta.clear();
        };
        for _ in 0..4 {
            pass(
                &nurburgring,
                View::fit(large.shrink(20.0), nurburgring.bounds, 20.0),
                large,
            );
            pass(&nurburgring, View::follow(large, nurburgring.start), large);
        }
        let requests = crate::ground_tiles::stats(&ctx).map(|(_, requests)| requests);
        assert!(
            requests.is_some_and(|requests| requests <= crate::ground_tiles::REQUEST_LIMIT),
            "stale-level requests accumulated: {requests:?}"
        );
        run_source_frames(
            &ctx,
            large,
            |painter| {
                draw_track(
                    painter,
                    &View::follow(large, nurburgring.start),
                    &nurburgring,
                )
            },
            "after rapid zoom",
        );

        for (label, track, view, canvas) in views.drain(..) {
            let outputs = run_source_frames(
                &ctx,
                canvas,
                |painter| draw_track(painter, &view, track),
                label,
            );
            let output = outputs.last().unwrap();
            for shape in &output.shapes {
                if let Shape::Text(text) = &shape.shape {
                    assert!(
                        !text.galley.text().contains("Source visual unavailable"),
                        "{label}: {}",
                        text.galley.text()
                    );
                }
            }
            let (tiles, requests) =
                crate::ground_tiles::stats(&ctx).expect("tile cache created by draw_track");
            assert_eq!(requests, 0, "{label}: requests still pending");
            assert!(tiles > 0, "{label}: no resident tiles");
            assert!(
                tiles <= crate::ground_tiles::TILE_LIMIT,
                "{label}: {tiles} tiles exceed the documented bound"
            );
            assert!(
                tile_rects(&outputs).len() <= tiles.max(1),
                "{label}: drew unknown tiles"
            );
        }
    }

    #[test]
    #[ignore = "renders source course textures to the local audit directory"]
    fn source_course_render_diagnostics() {
        for (id, follow) in [
            ("gt4-suzuka", 0u8),
            ("gt4-nurburgring", 0),
            ("gt4-suzuka", 1),
            ("gt4-suzuka", 2),
            ("gt4-newyork", 1),
            ("gt4-gymkhana_free", 1),
        ] {
            let track = Circuit::lookup(id).unwrap().build();
            let mut harness = egui_kittest::Harness::builder()
                .with_size(EVec2::new(1400.0, 900.0))
                .with_pixels_per_point(1.0)
                .with_accessibility_check(false)
                .build_ui(move |ui| {
                    ui.painter().rect_filled(ui.max_rect(), 0, colors::GRASS);
                    let view = match follow {
                        0 => View::fit(ui.max_rect(), track.bounds, 15.0 * UNITS_PER_METRE as f32),
                        1 => View::follow(ui.max_rect(), track.start),
                        _ => View {
                            origin: ui.max_rect().center(),
                            // 30 screen points per metre: 3.3x the follow view.
                            scale: 30.0 / UNITS_PER_METRE as f32,
                            world_min: track.start,
                        },
                    };
                    draw_track(ui.painter(), &view, &track);
                });
            harness.run_steps(1);
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(240);
            while crate::ground_tiles::pending(&harness.ctx) != Some(0) {
                assert!(
                    std::time::Instant::now() < deadline,
                    "{id}: tiles still pending {:?}",
                    crate::ground_tiles::pending(&harness.ctx)
                );
                std::thread::sleep(std::time::Duration::from_millis(10));
                harness.run_steps(1);
            }
            harness.run_steps(1);
            let suffix = match follow {
                0 => "-overview",
                1 => "-follow",
                _ => "-zoom",
            };
            let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join(format!(
                "../../.audit/track-visual-fix/{id}{suffix}-source-render.png"
            ));
            std::fs::create_dir_all(path.parent().unwrap()).unwrap();
            harness.render().unwrap().save(path).unwrap();
        }
    }
    #[test]
    #[ignore = "writes a diagnostic to local audit only; preserves documentation screenshots"]
    fn native_m3_sprite_render_diagnostic() {
        struct State {
            env: Env,
            sprite: Option<CarSprite>,
        }
        let state = State {
            env: Env::new(Circuit::Monza.build(), 1),
            sprite: None,
        };
        let mut harness = egui_kittest::Harness::builder()
            .with_size(EVec2::new(960.0, 600.0))
            .with_pixels_per_point(1.0)
            .with_accessibility_check(false)
            .build_ui_state(
                |ui, state: &mut State| {
                    if state.sprite.is_none() {
                        state.sprite = Some(CarSprite::original_m3(ui.ctx()).unwrap());
                    }
                    ui.painter().rect_filled(ui.max_rect(), 0, colors::GRASS);
                    state.env.x = 0.0;
                    state.env.y = 0.0;
                    for (origin, angle, steer) in [
                        (Pos2::new(240.0, 210.0), 0.0, 0.0),
                        (Pos2::new(700.0, 210.0), std::f64::consts::FRAC_PI_2, 0.4),
                    ] {
                        let view = View {
                            origin,
                            scale: 45.0 / UNITS_PER_METRE as f32,
                            world_min: Vec2 { x: 0.0, y: 0.0 },
                        };
                        state.env.heading = angle;
                        state.env.steer_angle = steer;
                        draw_car_with_sprite(
                            ui.painter(),
                            &view,
                            &state.env,
                            false,
                            state.sprite.as_ref(),
                        );
                        ui.painter().circle_filled(origin, 2.0, colors::BAD);
                    }
                    ui.painter().line_segment(
                        [Pos2::new(100.0, 470.0), Pos2::new(145.0, 470.0)],
                        Stroke::new(2.0, Color32::WHITE),
                    );
                    ui.painter().text(
                        Pos2::new(100.0, 490.0),
                        Align2::LEFT_TOP,
                        "1 metre | native origin red | existing collection, 107.36 source pixels/metre",
                        FontId::monospace(15.0),
                        Color32::WHITE,
                    );
                },
                state,
            );
        harness.run_steps(1);
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../.audit/gt4-sprite/rendered-m3-dimensions.png");
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        harness.render().unwrap().save(path).unwrap();
    }
    #[test]
    fn existing_collection_steering_uses_actual_rack_and_has_no_duplicate_wheels() {
        let ctx = egui::Context::default();
        let sprite = CarSprite::original_m3(&ctx).unwrap();
        let mut env = Env::new(Circuit::Monza.build(), 1);
        env.steer_angle = 20f64.to_radians();
        env.steer_target = -40f64.to_radians();
        let expected = sprite.frames[4].texture.id();
        let view = View::follow(
            Rect::from_min_size(Pos2::ZERO, EVec2::new(800.0, 600.0)),
            env.position(),
        );
        let mut output = ctx.run_pass(egui::RawInput::default(), |ctx| {
            draw_car_with_sprite(
                &ctx.layer_painter(egui::LayerId::background()),
                &view,
                &env,
                false,
                Some(&sprite),
            )
        });
        output.textures_delta.clear();
        assert_eq!(
            output.shapes.len(),
            1,
            "wheels already baked; no extra tire patches"
        );
        let Shape::Mesh(mesh) = &output.shapes[0].shape else {
            panic!("existing sprite not drawn")
        };
        assert_eq!(
            mesh.texture_id, expected,
            "pose follows actual rack rather than requested input"
        );
    }
}
