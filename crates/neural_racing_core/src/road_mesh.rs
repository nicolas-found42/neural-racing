//! Original RNW4 road triangles in physics coordinates (metres, Z up).
use crate::road_boundary::{BoundarySegment, RoadBoundaries};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub struct RoadContact {
    pub height_metres: f32,
    pub normal: [f32; 3],
    pub material_id: u8,
    pub flags: u8,
    pub attribute14: f32,
    pub attribute15: f32,
}
impl RoadContact {
    pub fn flat() -> Self {
        Self {
            height_metres: 0.0,
            normal: [0.0, 0.0, 1.0],
            material_id: 0,
            flags: 0,
            attribute14: 1.0,
            attribute15: 1.0,
        }
    }
}

#[derive(Clone)]
struct Vertex {
    point: [f32; 3],
    attributes: [u8; 2],
}
#[derive(Clone)]
struct Triangle {
    vertices: [u16; 3],
    material: u8,
    flags: u8,
    boundary_span: u32,
}
#[derive(Clone)]
pub(crate) struct RoadMesh {
    vertices: Vec<Vertex>,
    triangles: Vec<Triangle>,
    cells: HashMap<(i32, i32), Vec<usize>>,
    broad: Vec<usize>,
    boundaries: Option<RoadBoundaries>,
}
const CELL: f32 = 25.0;
fn cell(x: f32) -> i32 {
    (x / CELL).floor() as i32
}
impl RoadMesh {
    pub(crate) fn source_faces(
        &self,
    ) -> impl Iterator<Item = (u32, [[f32; 3]; 3], u8, u8, u32)> + '_ {
        self.triangles
            .iter()
            .enumerate()
            .filter_map(|(index, triangle)| {
                let points = triangle.vertices.map(|i| self.vertices[i as usize].point);
                let a = points[1].map(f64::from);
                let b = points[2].map(f64::from);
                let p = points[0].map(f64::from);
                let winding = (a[0] - p[0]) * (b[1] - p[1]) - (a[1] - p[1]) * (b[0] - p[0]);
                (winding > 1e-10).then_some((
                    index as u32,
                    points,
                    triangle.material,
                    triangle.flags,
                    triangle.boundary_span,
                ))
            })
    }

    pub(crate) fn source_boundary_segments(&self) -> Vec<BoundarySegment> {
        self.boundaries
            .as_ref()
            .map_or_else(Vec::new, RoadBoundaries::referenced_segments)
    }

    pub(crate) fn source_triangle_boundaries(&self, span: u32) -> Vec<BoundarySegment> {
        self.boundaries
            .as_ref()
            .map_or_else(Vec::new, |b| b.candidates(span).collect())
    }

    pub(crate) fn upward_triangles(&self) -> impl Iterator<Item = [[f32; 3]; 3]> + '_ {
        self.triangles.iter().filter_map(|triangle| {
            let p = triangle.vertices.map(|i| self.vertices[i as usize].point);
            let winding = (f64::from(p[1][0]) - f64::from(p[0][0]))
                * (f64::from(p[2][1]) - f64::from(p[0][1]))
                - (f64::from(p[1][1]) - f64::from(p[0][1]))
                    * (f64::from(p[2][0]) - f64::from(p[0][0]));
            (winding > 1e-10).then_some(p)
        })
    }

    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < 8 {
            return Err("truncated GT4 road mesh".into());
        }
        let u32_at = |i| u32::from_le_bytes(bytes[i..i + 4].try_into().unwrap()) as usize;
        let nv = u32_at(0);
        let nt = u32_at(4);
        if nv == 0 || nv > 65535 || nt == 0 || nt > 65535 || bytes.len() != 8 + nv * 16 + nt * 12 {
            return Err("invalid GT4 road mesh extents".into());
        }
        let mut vertices = Vec::with_capacity(nv);
        for b in bytes[8..8 + nv * 16].chunks_exact(16) {
            let f = |i| f32::from_le_bytes(b[i..i + 4].try_into().unwrap());
            // RNW [X,Yup,Z] == car [X,Z,-Y]. Do not fit to a published length.
            let point = [f(0), -f(8), f(4)];
            if !point.iter().all(|v| v.is_finite()) {
                return Err("non-finite GT4 vertex".into());
            }
            vertices.push(Vertex {
                point,
                attributes: [b[14], b[15]],
            });
        }
        let mut triangles = Vec::with_capacity(nt);
        let mut cells = HashMap::<_, Vec<_>>::new();
        let mut broad = Vec::new();
        for b in bytes[8 + nv * 16..].chunks_exact(12) {
            let u = |i| u16::from_le_bytes([b[i], b[i + 1]]);
            let indices = [u(0), u(2), u(4)];
            if indices.iter().any(|i| *i as usize >= nv) {
                return Err("GT4 triangle index out of range".into());
            }
            let index = triangles.len();
            triangles.push(Triangle {
                vertices: indices,
                material: b[6] & 31,
                flags: b[7],
                boundary_span: u32::from_le_bytes(b[8..12].try_into().unwrap()),
            });
            let p = indices.map(|i| vertices[i as usize].point);
            let min_x = p.iter().map(|v| v[0]).fold(f32::INFINITY, f32::min);
            let max_x = p.iter().map(|v| v[0]).fold(f32::NEG_INFINITY, f32::max);
            let min_y = p.iter().map(|v| v[1]).fold(f32::INFINITY, f32::min);
            let max_y = p.iter().map(|v| v[1]).fold(f32::NEG_INFINITY, f32::max);
            let (x0, x1, y0, y1) = (cell(min_x), cell(max_x), cell(min_y), cell(max_y));
            if i64::from(x1 - x0 + 1) * i64::from(y1 - y0 + 1) > 10000 {
                broad.push(index);
                continue;
            }
            for x in x0..=x1 {
                for y in y0..=y1 {
                    cells.entry((x, y)).or_default().push(index);
                }
            }
        }
        Ok(Self {
            vertices,
            triangles,
            cells,
            broad,
            boundaries: None,
        })
    }

    pub(crate) fn attach_boundaries(&mut self, bytes: &[u8]) -> Result<(), String> {
        let boundaries = RoadBoundaries::decode(bytes)?;
        if self
            .triangles
            .iter()
            .any(|t| !boundaries.valid_span(t.boundary_span))
        {
            return Err("GT4 triangle boundary span is out of range".into());
        }
        self.boundaries = Some(boundaries);
        Ok(())
    }

    pub(crate) fn boundary_candidates(&self, x: f32, y: f32, z: f32) -> Vec<BoundarySegment> {
        let Some((_, triangle)) = self.contact_with_triangle(x, y, z) else {
            return Vec::new();
        };
        self.boundaries.as_ref().map_or_else(Vec::new, |b| {
            b.candidates(self.triangles[triangle].boundary_span)
                .collect()
        })
    }

    pub(crate) fn contact(&self, x: f32, y: f32, z: f32) -> Option<RoadContact> {
        self.contact_with_triangle(x, y, z)
            .map(|(contact, _)| contact)
    }

    fn contact_with_triangle(&self, x: f32, y: f32, z: f32) -> Option<(RoadContact, usize)> {
        if ![x, y, z].iter().all(|v| v.is_finite()) {
            return None;
        }
        let mut best = None;
        // Original per-frame 0x3957f0 clears the query hint, starts the
        // downward ray five metres above the prepared wheel position, and
        // takes the highest front face below that point (not the nearest deck).
        let ray_start = f64::from(z + 5.0);
        let mut best_height = f64::NEG_INFINITY;
        let local = self
            .cells
            .get(&(cell(x), cell(y)))
            .map_or(&[][..], Vec::as_slice);
        for &index in local.iter().chain(&self.broad) {
            let t = &self.triangles[index];
            let v = t.vertices.map(|i| &self.vertices[i as usize]);
            let p = v.map(|v| v.point.map(f64::from));
            let a = [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]];
            let b = [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]];
            let denominator = a[0] * b[1] - a[1] * b[0];
            // Original downward road rays reject backfaces; an underside is
            // not another drivable deck. The axis transform preserves winding.
            if denominator <= 1e-10 {
                continue;
            }
            let dx = f64::from(x) - p[0][0];
            let dy = f64::from(y) - p[0][1];
            let u = (dx * b[1] - dy * b[0]) / denominator;
            let w = (a[0] * dy - a[1] * dx) / denominator;
            if u < -1e-6 || w < -1e-6 || u + w > 1.0 + 1e-6 {
                continue;
            }
            let height = p[0][2] + u * a[2] + w * b[2];
            if height > ray_start || height <= best_height {
                continue;
            }
            let normal = [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                denominator,
            ];
            let length = normal.iter().map(|n| n * n).sum::<f64>().sqrt();
            let weights = [1.0 - u - w, u, w];
            let interpolate = |slot, mask, scale| {
                v.iter()
                    .zip(weights)
                    .map(|(v, w)| f64::from(v.attributes[slot] & mask) * w / scale)
                    .sum::<f64>()
                    .clamp(0.0, 1.0) as f32
            };
            best = Some((
                RoadContact {
                    height_metres: height as f32,
                    normal: normal.map(|n| (n / length) as f32),
                    material_id: t.material,
                    flags: t.flags,
                    attribute14: interpolate(0, 255, 255.0),
                    attribute15: interpolate(1, 127, 127.0),
                },
                index,
            ));
            best_height = height;
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn road_with_deck_offset(offset: f32) -> RoadMesh {
        let mut bytes = Vec::new();
        bytes.extend(6u32.to_le_bytes());
        bytes.extend(2u32.to_le_bytes());
        // Two stacked, sloped triangles: file Y is height and file -Z is car Y.
        for [x, y, z] in [
            [0.0f32, 0.0, 0.0],
            [10.0, 10.0, 0.0],
            [0.0, 0.0, -10.0],
            [0.0, offset, 0.0],
            [10.0, offset + 10.0, 0.0],
            [0.0, offset, -10.0],
        ] {
            for f in [x, y, z] {
                bytes.extend(f.to_le_bytes());
            }
            bytes.extend([0, 0, 255, 127]);
        }
        for (i, material) in [(0u16, 0u8), (3, 2)] {
            for v in [i, i + 1, i + 2] {
                bytes.extend(v.to_le_bytes());
            }
            bytes.extend([material, 7, 0, 0, 0, 0]);
        }
        RoadMesh::decode(&bytes).unwrap()
    }
    #[test]
    fn underside_winding_is_rejected_instead_of_flipped_into_a_road() {
        let mut bytes = Vec::new();
        bytes.extend(3u32.to_le_bytes());
        bytes.extend(1u32.to_le_bytes());
        for [x, y, z] in [[0.0f32, 0.0, 0.0], [10.0, 0.0, 0.0], [0.0, 0.0, -10.0]] {
            for f in [x, y, z] {
                bytes.extend(f.to_le_bytes());
            }
            bytes.extend([0, 0, 255, 127]);
        }
        for i in [0u16, 2, 1] {
            bytes.extend(i.to_le_bytes());
        }
        bytes.extend([0, 0, 0, 0, 0, 0]);
        assert!(RoadMesh::decode(&bytes)
            .unwrap()
            .contact(2.0, 2.0, 1.0)
            .is_none());
    }

    #[test]
    fn original_axes_slope_material_and_reference_height_select_the_right_deck() {
        let mesh = road_with_deck_offset(20.0);
        let low = mesh.contact(2.0, 2.0, 3.0).unwrap();
        let high = mesh.contact(2.0, 2.0, 23.0).unwrap();
        assert!((low.height_metres - 2.0).abs() < 1e-6);
        assert!((high.height_metres - 22.0).abs() < 1e-6);
        assert_eq!((low.material_id, high.material_id), (0, 2));
        assert!((low.normal[0] + std::f32::consts::FRAC_1_SQRT_2).abs() < 1e-6);
        assert!(low.normal[2] > 0.7);
        assert_eq!(low.flags, 7);
        assert_eq!(low.attribute14, 1.0);
        assert_eq!(low.attribute15, 1.0);
        assert!(mesh.contact(100.0, 100.0, 0.0).is_none());
    }

    #[test]
    fn native_downward_ray_chooses_highest_reachable_face() {
        let mesh = road_with_deck_offset(4.0);
        // The lower deck is exactly at the reference height. Native +5 m
        // placement still hits the upper deck first.
        let hit = mesh.contact(2.0, 2.0, 2.0).unwrap();
        assert_eq!(hit.height_metres, 6.0);
        assert_eq!(hit.material_id, 2);
        let low = mesh.contact(2.0, 2.0, -1.0).unwrap();
        assert_eq!(low.height_metres, 2.0);
        assert!(mesh.contact(2.0, 2.0, -4.0).is_none());
        // Original bounds reach the bottom of the mesh; no invented maximum
        // downward contact distance rejects a reachable original face.
        assert_eq!(mesh.contact(2.0, 2.0, 100.0).unwrap().height_metres, 6.0);
    }

    #[test]
    fn boundary_candidates_follow_the_selected_source_triangle_and_deck() {
        let mut mesh = road_with_deck_offset(20.0);
        assert!(mesh.boundary_candidates(2.0, 2.0, 3.0).is_empty());
        mesh.triangles[0].boundary_span = 1 << 24;
        mesh.triangles[1].boundary_span = (1 << 24) | 1;
        let mut bytes = Vec::new();
        bytes.extend(6u32.to_le_bytes());
        bytes.extend(2u32.to_le_bytes());
        for (height, flags) in [(0.0f32, 7u16), (20.0, 17)] {
            for (point, counter) in [
                ([0.0, height, 0.0], 3i16),
                ([10.0, height + 10.0, 0.0], 0),
                ([0.0, height, -10.0], -3),
            ] {
                for value in point {
                    bytes.extend(value.to_le_bytes());
                }
                bytes.extend(counter.to_le_bytes());
                bytes.extend(flags.to_le_bytes());
            }
        }
        for index in [0u16, 3] {
            bytes.extend(index.to_le_bytes());
        }
        mesh.attach_boundaries(&bytes).unwrap();
        let lower = mesh.boundary_candidates(2.0, 2.0, 3.0);
        let upper = mesh.boundary_candidates(2.0, 2.0, 23.0);
        assert_eq!(lower.len(), 1);
        assert_eq!(upper.len(), 1);
        assert_eq!((lower[0].source_index, lower[0].flags), (0, 7));
        assert_eq!((upper[0].source_index, upper[0].flags), (3, 17));
        assert_eq!(lower[0].start[2], 0.0);
        assert_eq!(upper[0].start[2], 20.0);
        assert!(mesh.boundary_candidates(100.0, 100.0, 0.0).is_empty());
    }

    #[test]
    fn original_course_backfaces_match_native_downward_queries() {
        // Two original triangles per fixture, with only vertex indices remapped.
        // The closer, negative-winding face must not replace the native hit.
        // Source hashes and original EE query results are in docs/reference/gt4/fixtures.
        for (bytes, query, expected_height, material) in [
            (
                include_bytes!("circuit_data/gt4_fixtures/montecarlo-native-backface.mesh")
                    .as_slice(),
                [243.526316f32, 106.316683, 45.501434],
                45.66817093,
                7,
            ),
            (
                include_bytes!("circuit_data/gt4_fixtures/suzuka-native-backface.mesh").as_slice(),
                [-3.054007f32, 322.020589, 14.023250],
                14.02353764,
                2,
            ),
        ] {
            let hit = RoadMesh::decode(bytes)
                .unwrap()
                .contact(query[0], query[1], query[2])
                .expect("original positive-winding road hit");
            assert!((hit.height_metres - expected_height).abs() < 5e-5);
            assert_eq!(hit.material_id, material);
            assert!(hit.normal[2] > 0.0);
        }
    }
}
