//! Original RNW boundary records. These are distinct from an authored route guide.
use std::sync::Arc;

#[derive(Clone, Copy, Debug)]
pub(crate) struct BoundarySegment {
    pub source_index: u16,
    pub start: [f32; 3],
    pub end: [f32; 3],
    pub flags: u16,
}

impl BoundarySegment {
    /// Point-ray portion of original 0x45d550. This is not the full swept
    /// eight-point car/corner response. Boundary height is intentionally absent:
    /// the original callback is 2D; the owning road triangle supplies deck scope.
    pub(crate) fn point_crossing_fraction(&self, from: [f64; 2], to: [f64; 2]) -> Option<f64> {
        if self.flags & 2 != 0 || !from.iter().chain(&to).all(|v| v.is_finite()) {
            return None;
        }
        let a = [f64::from(self.start[0]), f64::from(self.start[1])];
        let edge = [f64::from(self.end[0]) - a[0], f64::from(self.end[1]) - a[1]];
        let side = |p: [f64; 2]| edge[0] * (p[1] - a[1]) - edge[1] * (p[0] - a[0]);
        let before = side(from);
        let after = side(to);
        if before <= 0.0 || after >= 0.0 {
            return None;
        }
        let fraction = before / (before - after);
        let hit = [
            from[0] + (to[0] - from[0]) * fraction,
            from[1] + (to[1] - from[1]) * fraction,
        ];
        let length_sq = edge[0] * edge[0] + edge[1] * edge[1];
        let along = ((hit[0] - a[0]) * edge[0] + (hit[1] - a[1]) * edge[1]) / length_sq;
        (0.0..=1.0).contains(&along).then_some(fraction)
    }
}

#[derive(Clone)]
pub(crate) struct RoadBoundaries {
    vertices: Arc<Vec<Vertex>>,
    indices: Arc<Vec<u16>>,
}

#[derive(Clone)]
struct Vertex {
    point: [f32; 3],
    counter: i16,
    flags: u16,
}

impl RoadBoundaries {
    pub(crate) fn decode(bytes: &[u8]) -> Result<Self, String> {
        if bytes.len() < 8 {
            return Err("truncated GT4 boundary header".into());
        }
        let u = |offset| u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap()) as usize;
        let nv = u(0);
        let ni = u(4);
        if nv > 65535 || ni > 16_777_215 || bytes.len() != 8 + nv * 16 + ni * 2 {
            return Err("invalid GT4 boundary extents".into());
        }
        let mut vertices = Vec::with_capacity(nv);
        for record in bytes[8..8 + nv * 16].chunks_exact(16) {
            let f = |offset| f32::from_le_bytes(record[offset..offset + 4].try_into().unwrap());
            let point = [f(0), -f(8), f(4)];
            if !point.iter().all(|value| value.is_finite()) {
                return Err("non-finite GT4 boundary vertex".into());
            }
            vertices.push(Vertex {
                point,
                counter: i16::from_le_bytes(record[12..14].try_into().unwrap()),
                flags: u16::from_le_bytes(record[14..16].try_into().unwrap()),
            });
        }
        let indices: Vec<_> = bytes[8 + nv * 16..]
            .chunks_exact(2)
            .map(|record| u16::from_le_bytes(record.try_into().unwrap()))
            .collect();
        let result = Self {
            vertices: Arc::new(vertices),
            indices: Arc::new(indices),
        };
        if result
            .indices
            .iter()
            .any(|&index| result.segment(index).is_none())
        {
            return Err("invalid GT4 boundary segment index or closure".into());
        }
        Ok(result)
    }

    pub(crate) fn valid_span(&self, packed: u32) -> bool {
        let start = (packed & 0x00ff_ffff) as usize;
        let count = (packed >> 24) as usize;
        count == 0
            || start
                .checked_add(count)
                .is_some_and(|end| end <= self.indices.len())
    }

    pub(crate) fn candidates(&self, packed: u32) -> impl Iterator<Item = BoundarySegment> + '_ {
        let start = (packed & 0x00ff_ffff) as usize;
        let count = (packed >> 24) as usize;
        let indices = self.indices.get(start..start + count).unwrap_or(&[]);
        indices.iter().filter_map(|&index| self.segment(index))
    }

    pub(crate) fn referenced_segments(&self) -> Vec<BoundarySegment> {
        let mut indices = self.indices.as_ref().clone();
        indices.sort_unstable();
        indices.dedup();
        indices
            .into_iter()
            .filter_map(|index| self.segment(index))
            .collect()
    }

    fn segment(&self, index: u16) -> Option<BoundarySegment> {
        let vertex = self.vertices.get(usize::from(index))?;
        // Original 0x3e6a58: an ending record closes its own chain, never
        // the next unrelated chain. Per-triangle indices name segment starts.
        let next = i32::from(index) + 1 + i32::from(vertex.counter.min(0));
        let next = self.vertices.get(usize::try_from(next).ok()?)?;
        Some(BoundarySegment {
            source_index: index,
            start: vertex.point,
            end: next.point,
            flags: vertex.flags,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend(6u32.to_le_bytes());
        bytes.extend(4u32.to_le_bytes());
        for (point, counter, flags) in [
            ([0.0f32, 2.0, 0.0], 3i16, 7u16),
            ([2.0, 2.0, 0.0], 0, 8),
            ([1.0, 2.0, -2.0], -3, 9),
            ([0.0, 12.0, 0.0], 3, 17),
            ([2.0, 12.0, 0.0], 0, 18),
            ([1.0, 12.0, -2.0], -3, 19),
        ] {
            for value in point {
                bytes.extend(value.to_le_bytes());
            }
            bytes.extend(counter.to_le_bytes());
            bytes.extend(flags.to_le_bytes());
        }
        for index in [0u16, 2, 3, 5] {
            bytes.extend(index.to_le_bytes());
        }
        bytes
    }
    #[test]
    fn triangle_spans_preserve_chain_closure_height_and_start_flags() {
        let b = RoadBoundaries::decode(&fixture()).unwrap();
        let lower: Vec<_> = b.candidates(2 << 24).collect();
        let upper: Vec<_> = b.candidates((2 << 24) | 2).collect();
        assert_eq!(lower.len(), 2);
        assert_eq!(upper.len(), 2);
        assert_eq!(lower[0].start, [0.0, -0.0, 2.0]);
        assert_eq!(lower[1].start, [1.0, 2.0, 2.0]);
        assert_eq!(lower[1].end, lower[0].start);
        assert_eq!(lower[1].flags, 9);
        assert_eq!(lower[1].source_index, 2);
        assert!(upper.iter().all(|s| s.start[2] == 12.0 && s.end[2] == 12.0));
        assert!(b.valid_span((2 << 24) | 2));
        assert!(!b.valid_span((3 << 24) | 2));
    }
    #[test]
    fn malformed_indices_and_truncated_payloads_are_rejected() {
        let mut bytes = fixture();
        let last = bytes.len() - 2;
        bytes[last..].copy_from_slice(&6u16.to_le_bytes());
        assert!(RoadBoundaries::decode(&bytes).is_err());
        bytes.pop();
        assert!(RoadBoundaries::decode(&bytes).is_err());
    }

    #[test]
    fn original_suzuka_point_crossing_is_one_sided_and_height_independent() {
        // Unchanged boundary records10/11 and original45d550 input/output cases.
        let mut edge = BoundarySegment {
            source_index: 10,
            start: [319.66162109375, 505.9697265625, -3.8481361865997314],
            end: [324.44146728515625, 505.9697265625, -3.9816231727600098],
            flags: 0,
        };
        let from = [322.0515441894531, 506.9697265625];
        let to = [322.0515441894531, 504.9697265625];
        assert_eq!(edge.point_crossing_fraction(from, to), Some(0.5));
        assert_eq!(edge.point_crossing_fraction(to, from), None);
        edge.flags = 1;
        assert_eq!(edge.point_crossing_fraction(from, to), Some(0.5));
        edge.start[2] += 1000.0;
        edge.end[2] += 1000.0;
        assert_eq!(edge.point_crossing_fraction(from, to), Some(0.5));
        edge.flags = 2;
        assert_eq!(edge.point_crossing_fraction(from, to), None);
        edge.flags = 0;
        assert_eq!(
            edge.point_crossing_fraction([300.0, from[1]], [300.0, to[1]]),
            None
        );
    }
}
