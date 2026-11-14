//! Geometry for top-down painting, retaining original source faces and deck heights.
use crate::Vec2;
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct RoadRenderVertex {
    pub position: Vec2,
    pub height_metres: f32,
}

#[derive(Clone, Copy, Debug)]
pub struct RoadRenderTriangle {
    pub vertices: [RoadRenderVertex; 3],
    pub material_id: u8,
    pub flags: u8,
    pub source_triangle: u32,
    /// Clipped to the course corridor, rather than adjoining source ground.
    pub racing_surface: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct RoadRenderBoundary {
    pub vertices: [RoadRenderVertex; 2],
    pub flags: u16,
    pub source_segment: u16,
}

#[derive(Clone, Copy, Debug)]
pub enum RoadRenderItem {
    Triangle(usize),
    Boundary(usize),
}

#[derive(Clone, Debug, Default)]
pub struct RoadRenderGeometry {
    pub triangles: Vec<RoadRenderTriangle>,
    pub boundaries: Vec<RoadRenderBoundary>,
    /// Faces and their boundaries share one ordering, so lower-deck lines
    /// cannot be painted indiscriminately over upper-deck faces.
    pub order: Vec<RoadRenderItem>,
}

struct CorridorMask {
    points: [Vec2; 3],
    heights: [f32; 2],
}

impl RoadRenderGeometry {
    pub(crate) fn build(
        mesh: &crate::road_mesh::RoadMesh,
        outer: &[Vec2],
        inner: &[Vec2],
        heights: &[f32],
        closed: bool,
    ) -> Self {
        let scale = crate::UNITS_PER_METRE;
        let cell_size = 25.0 * scale;
        let cell = |p: Vec2| {
            (
                (p.x / cell_size).floor() as i32,
                (p.y / cell_size).floor() as i32,
            )
        };
        let bounds = |p: &[Vec2]| {
            p.iter().fold(
                (
                    f64::INFINITY,
                    f64::INFINITY,
                    f64::NEG_INFINITY,
                    f64::NEG_INFINITY,
                ),
                |b, p| (b.0.min(p.x), b.1.min(p.y), b.2.max(p.x), b.3.max(p.y)),
            )
        };
        let mut masks = Vec::new();
        let mut grid: HashMap<(i32, i32), Vec<usize>> = HashMap::new();
        for i in 0..if closed { outer.len() } else { outer.len() - 1 } {
            let j = (i + 1) % outer.len();
            let mut range = [heights[i].min(heights[j]), heights[i].max(heights[j])];
            for (point, reference) in [
                (outer[i], heights[i]),
                (inner[i], heights[i]),
                (outer[j], heights[j]),
                (inner[j], heights[j]),
            ] {
                if let Some(hit) = mesh.contact(
                    (point.x / scale) as f32,
                    (point.y / scale) as f32,
                    reference,
                ) {
                    range[0] = range[0].min(hit.height_metres);
                    range[1] = range[1].max(hit.height_metres);
                }
            }
            for points in [
                [outer[i], outer[j], inner[j]],
                [outer[i], inner[j], inner[i]],
            ] {
                let index = masks.len();
                let b = bounds(&points);
                let (x0, y0) = cell(Vec2 { x: b.0, y: b.1 });
                let (x1, y1) = cell(Vec2 { x: b.2, y: b.3 });
                for x in x0..=x1 {
                    for y in y0..=y1 {
                        grid.entry((x, y)).or_default().push(index);
                    }
                }
                masks.push(CorridorMask {
                    points,
                    heights: range,
                });
            }
        }
        let mut result = Self::default();
        let mut ordering = Vec::new();
        let mut boundary_heights: HashMap<u16, f32> = HashMap::new();
        for (source, points, material, flags, span) in mesh.source_faces() {
            let vertices = points.map(|p| RoadRenderVertex {
                position: Vec2 {
                    x: f64::from(p[0]) * scale,
                    y: f64::from(p[1]) * scale,
                },
                height_metres: p[2],
            });
            let sort_height = points.iter().map(|p| p[2]).sum::<f32>() / 3.0;
            for edge in mesh.source_triangle_boundaries(span) {
                boundary_heights
                    .entry(edge.source_index)
                    .and_modify(|h| *h = h.max(sort_height))
                    .or_insert(sort_height);
            }
            let add = |result: &mut Self,
                       ordering: &mut Vec<(f32, u8, RoadRenderItem)>,
                       vertices,
                       racing| {
                let index = result.triangles.len();
                result.triangles.push(RoadRenderTriangle {
                    vertices,
                    material_id: material,
                    flags,
                    source_triangle: source,
                    racing_surface: racing,
                });
                ordering.push((
                    sort_height,
                    if racing { 1 } else { 0 },
                    RoadRenderItem::Triangle(index),
                ));
            };
            add(&mut result, &mut ordering, vertices, false);
            let p = vertices.map(|v| v.position);
            let b = bounds(&p);
            let (x0, y0) = cell(Vec2 { x: b.0, y: b.1 });
            let (x1, y1) = cell(Vec2 { x: b.2, y: b.3 });
            let mut candidates = Vec::new();
            for x in x0..=x1 {
                for y in y0..=y1 {
                    if let Some(ids) = grid.get(&(x, y)) {
                        candidates.extend_from_slice(ids);
                    }
                }
            }
            candidates.sort_unstable();
            candidates.dedup();
            let lo = points.iter().map(|p| p[2]).fold(f32::INFINITY, f32::min);
            let hi = points
                .iter()
                .map(|p| p[2])
                .fold(f32::NEG_INFINITY, f32::max);
            for index in candidates {
                let mask = &masks[index];
                if lo > mask.heights[1] + 0.5 || hi < mask.heights[0] - 0.5 {
                    continue;
                }
                let polygon = clip_face(vertices, mask.points);
                for i in 1..polygon.len().saturating_sub(1) {
                    let triangle = [polygon[0], polygon[i], polygon[i + 1]];
                    let a = triangle[1].position - triangle[0].position;
                    let b = triangle[2].position - triangle[0].position;
                    if a.x * b.y - a.y * b.x > 1e-10 {
                        add(&mut result, &mut ordering, triangle, true);
                    }
                }
            }
        }
        for edge in mesh.source_boundary_segments() {
            let Some(&sort_height) = boundary_heights.get(&edge.source_index) else {
                continue;
            };
            let vertices = [edge.start, edge.end].map(|p| RoadRenderVertex {
                position: Vec2 {
                    x: f64::from(p[0]) * scale,
                    y: f64::from(p[1]) * scale,
                },
                height_metres: p[2],
            });
            let index = result.boundaries.len();
            result.boundaries.push(RoadRenderBoundary {
                vertices,
                flags: edge.flags,
                source_segment: edge.source_index,
            });
            ordering.push((sort_height, 2, RoadRenderItem::Boundary(index)));
        }
        ordering.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        result.order = ordering.into_iter().map(|(_, _, item)| item).collect();
        result
    }
}

/// Clip an original convex source face against a corridor triangle. Every new
/// vertex remains on that source face; no inverted ribbon triangle is painted.
pub(crate) fn clip_face(source: [RoadRenderVertex; 3], mask: [Vec2; 3]) -> Vec<RoadRenderVertex> {
    let cross = |a: Vec2, b: Vec2| a.x * b.y - a.y * b.x;
    let area = cross(mask[1] - mask[0], mask[2] - mask[0]);
    if area.abs() < 1e-12 {
        return Vec::new();
    }
    let sign = area.signum();
    let mut polygon = source.to_vec();
    for i in 0..3 {
        if polygon.is_empty() {
            break;
        }
        let a = mask[i];
        let edge = mask[(i + 1) % 3] - a;
        let side = |p: Vec2| cross(edge, p - a) * sign;
        let mut output = Vec::with_capacity(polygon.len() + 1);
        let mut previous = *polygon.last().unwrap();
        let mut before = side(previous.position);
        for current in polygon {
            let after = side(current.position);
            if (before >= 0.0) != (after >= 0.0) {
                let t = (before / (before - after)).clamp(0.0, 1.0);
                output.push(RoadRenderVertex {
                    position: previous.position + (current.position - previous.position) * t,
                    height_metres: previous.height_metres
                        + (current.height_metres - previous.height_metres) * t as f32,
                });
            }
            if after >= 0.0 {
                output.push(current);
            }
            previous = current;
            before = after;
        }
        polygon = output;
    }
    polygon
}

#[cfg(test)]
mod tests {
    use super::*;
    fn v(x: f64, y: f64) -> RoadRenderVertex {
        RoadRenderVertex {
            position: Vec2 { x, y },
            height_metres: (x + 2.0 * y) as f32,
        }
    }
    #[test]
    fn clipping_preserves_source_plane_and_orientation_for_both_mask_windings() {
        let source = [v(0.0, 0.0), v(10.0, 0.0), v(0.0, 10.0)];
        let mask = [
            Vec2 { x: 2.0, y: 2.0 },
            Vec2 { x: 8.0, y: 2.0 },
            Vec2 { x: 2.0, y: 8.0 },
        ];
        for mask in [mask, [mask[0], mask[2], mask[1]]] {
            let p = clip_face(source, mask);
            assert_eq!(p.len(), 3);
            for v in &p {
                assert!(
                    (f64::from(v.height_metres) - v.position.x - 2.0 * v.position.y).abs() < 1e-5
                );
                assert!(v.position.x >= 2.0 && v.position.y >= 2.0);
                assert!(v.position.x + v.position.y <= 10.0);
            }
            let a = p[1].position - p[0].position;
            let b = p[2].position - p[0].position;
            assert!(a.x * b.y - a.y * b.x > 0.0);
        }
    }
    #[test]
    fn a_corridor_cannot_paint_a_missing_source_road_or_fill_its_hole() {
        let source = [v(0.0, 0.0), v(1.0, 0.0), v(0.0, 1.0)];
        let mask = [
            Vec2 { x: 2.0, y: 2.0 },
            Vec2 { x: 8.0, y: 2.0 },
            Vec2 { x: 2.0, y: 8.0 },
        ];
        assert!(clip_face(source, mask).is_empty());
    }
}
