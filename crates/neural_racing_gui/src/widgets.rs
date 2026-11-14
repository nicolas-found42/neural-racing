//! Dashboard widgets: gauges, telemetry bars and the brain view.

use egui::{Align2, Color32, FontId, Pos2, Rect, Response, RichText, Sense, Stroke, Ui, Vec2};

use crate::palette as colors;

pub fn section(ui: &mut Ui, title: &str) {
    ui.add_space(10.0);
    ui.label(
        RichText::new(title.to_uppercase())
            .size(10.0)
            .strong()
            .color(colors::MUTED),
    );
    ui.add_space(2.0);
}

pub fn stat_row(ui: &mut Ui, label: &str, value: &str, color: Color32) {
    ui.horizontal(|ui| {
        ui.label(RichText::new(label).size(13.0).color(colors::MUTED));
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.label(RichText::new(value).size(14.0).strong().color(color));
        });
    });
}

/// Label, readout and a fill bar between `min` and `max`.
pub fn gauge(
    ui: &mut Ui,
    label: &str,
    value: f64,
    min: f64,
    max: f64,
    color: Color32,
    readout: &str,
) {
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, 26.0), Sense::hover());
    let painter = ui.painter();
    painter.text(
        rect.left_top(),
        Align2::LEFT_TOP,
        label,
        FontId::proportional(12.0),
        colors::MUTED,
    );
    painter.text(
        rect.right_top(),
        Align2::RIGHT_TOP,
        readout,
        FontId::monospace(12.0),
        color,
    );
    let bar = Rect::from_min_size(
        Pos2::new(rect.left(), rect.top() + 16.0),
        Vec2::new(rect.width(), 6.0),
    );
    painter.rect_filled(bar, 3, colors::DARK);
    let fraction = ((value - min) / (max - min)).clamp(0.0, 1.0) as f32;
    if fraction > 0.0 {
        painter.rect_filled(
            Rect::from_min_size(bar.min, Vec2::new(bar.width() * fraction, bar.height())),
            3,
            color,
        );
    }
}

/// Bar that grows left (negative) or right (positive) from the centre.
pub fn centered_gauge(ui: &mut Ui, label: &str, value: f64, color: Color32, readout: &str) {
    let width = ui.available_width();
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, 26.0), Sense::hover());
    let painter = ui.painter();
    painter.text(
        rect.left_top(),
        Align2::LEFT_TOP,
        label,
        FontId::proportional(12.0),
        colors::MUTED,
    );
    painter.text(
        rect.right_top(),
        Align2::RIGHT_TOP,
        readout,
        FontId::monospace(12.0),
        color,
    );
    let bar = Rect::from_min_size(
        Pos2::new(rect.left(), rect.top() + 16.0),
        Vec2::new(rect.width(), 6.0),
    );
    painter.rect_filled(bar, 3, colors::DARK);
    let clamped = value.clamp(-1.0, 1.0) as f32;
    let centre = bar.center().x;
    let half = bar.width() * 0.5;
    let fill = Rect::from_min_max(
        Pos2::new(centre, bar.top()),
        Pos2::new(centre + half * clamped, bar.bottom()),
    );
    if clamped.abs() > 0.01 {
        painter.rect_filled(fill, 3, color);
    }
    painter.rect_filled(
        Rect::from_min_size(
            Pos2::new(centre - 0.5, bar.top()),
            Vec2::new(1.0, bar.height()),
        ),
        0,
        colors::PANEL_EDGE,
    );
}

fn activation_color(value: f32) -> Color32 {
    let magnitude = (value.abs() / 1.2).clamp(0.0, 1.0);
    if value >= 0.0 {
        colors::GOOD.gamma_multiply(0.15 + 0.85 * magnitude)
    } else {
        colors::BAD.gamma_multiply(0.15 + 0.85 * magnitude)
    }
}

/// Vertical strips of every layer's activations: the network as a signal map.
pub fn brain(ui: &mut Ui, activations: &[Vec<f32>]) -> Response {
    let width = ui.available_width();
    let height = 132.0;
    let (rect, response) = ui.allocate_exact_size(Vec2::new(width, height + 18.0), Sense::hover());
    let painter = ui.painter();
    let plot = Rect::from_min_size(rect.min, Vec2::new(rect.width(), height));
    painter.rect_filled(plot, 6, colors::DARK);

    if activations.is_empty() {
        return response;
    }
    let gap = 8.0;
    let columns = activations.len() as f32;
    let column_width = ((plot.width() - gap * (columns - 1.0)) / columns).max(3.0);
    for (index, layer) in activations.iter().enumerate() {
        let x = plot.left() + index as f32 * (column_width + gap);
        let count = layer.len().max(1);
        let cell = plot.height() / count as f32;
        let peak = layer.iter().fold(0f32, |acc, value| acc.max(value.abs()));
        for (unit, value) in layer.iter().enumerate() {
            let top = plot.top() + unit as f32 * cell;
            let bar = Rect::from_min_size(
                Pos2::new(x, top + 0.3),
                Vec2::new(column_width, (cell - 0.6).max(0.8)),
            );
            painter.rect_filled(bar, 0, activation_color(*value));
            if peak > 0.0 && (*value / peak).abs() > 0.97 {
                painter.rect_stroke(
                    bar,
                    0,
                    Stroke::new(0.8, colors::TEXT),
                    egui::StrokeKind::Inside,
                );
            }
        }
        painter.text(
            Pos2::new(x + column_width * 0.5, plot.bottom() + 4.0),
            Align2::CENTER_TOP,
            if index == 0 {
                format!("{} in", layer.len())
            } else if index == activations.len() - 1 {
                "act".to_string()
            } else {
                format!("{}", layer.len())
            },
            FontId::monospace(9.0),
            colors::MUTED,
        );
    }
    response
}

pub fn pill(ui: &mut Ui, text: &str, color: Color32) {
    let font = FontId::proportional(12.0);
    let galley = ui.painter().layout_no_wrap(text.to_string(), font, color);
    let size = Vec2::new(galley.size().x + 16.0, 22.0);
    let (rect, _) = ui.allocate_exact_size(size, Sense::hover());
    ui.painter()
        .rect_filled(rect, 11, color.gamma_multiply(0.22));
    ui.painter().rect_stroke(
        rect,
        11,
        Stroke::new(1.0, color.gamma_multiply(0.7)),
        egui::StrokeKind::Inside,
    );
    ui.painter().galley(
        Pos2::new(
            rect.center().x - galley.size().x * 0.5,
            rect.center().y - galley.size().y * 0.5,
        ),
        galley,
        color,
    );
}
