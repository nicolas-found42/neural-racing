//! Loads source-bound GT4 road meshes and derived corridor stations. No map fitting.
use crate::{
    course_catalog::CourseTopology, road_mesh::RoadMesh, track::RecoveredStation, Circuit, Track,
    Vec2, UNITS_PER_METRE as M,
};

struct CourseBytes {
    id: &'static str,
    stations: &'static [u8],
    mesh: &'static [u8],
    length_metres: f64,
    start_v: f64,
    goal_v: f64,
}
fn assets() -> &'static [CourseBytes] {
    include!("circuit_data/gt4_files.rs")
}

fn boundary_assets() -> &'static [(&'static str, &'static [u8])] {
    static DATA: &[(&str, &[u8])] = include!("circuit_data/gt4_boundary_files.rs");
    DATA
}

fn edge_assets() -> &'static [(&'static str, &'static [u8])] {
    static DATA: &[(&str, &[u8])] = include!("circuit_data/gt4_edge_files.rs");
    DATA
}

fn visual_assets() -> &'static [(&'static str, &'static [u8])] {
    static DATA: &[(&str, &[u8])] = include!("circuit_data/gt4_visual_files.rs");
    DATA
}

pub(crate) fn build(index: u16) -> Result<Track, String> {
    let metadata = crate::course_catalog::courses()
        .get(index as usize)
        .ok_or("unknown GT4 course")?;
    if !metadata.available {
        return Err(format!(
            "GT4 course {} has no validated driving route",
            metadata.id
        ));
    }
    build_geometry(index)
}

fn build_geometry(index: u16) -> Result<Track, String> {
    let metadata = crate::course_catalog::courses()
        .get(index as usize)
        .ok_or("unknown GT4 course")?;
    let data = assets()
        .iter()
        .find(|a| a.id == metadata.id)
        .ok_or("GT4 geometry asset missing")?;
    let closed = metadata.topology == CourseTopology::Closed;
    if !matches!(
        metadata.topology,
        CourseTopology::Closed | CourseTopology::Open | CourseTopology::Area
    ) {
        return Err("GT4 resource is not a race route".into());
    }
    if data.stations.len() < 4 {
        return Err("truncated GT4 stations".into());
    }
    let count = u32::from_le_bytes(data.stations[..4].try_into().unwrap()) as usize;
    if count < 2 || count > 50000 || data.stations.len() != 4 + count * 32 {
        return Err("invalid GT4 station extents".into());
    }
    let edges = edge_assets()
        .iter()
        .find(|(id, _)| *id == metadata.id)
        .map(|(_, bytes)| *bytes);
    if let Some(bytes) = edges {
        if bytes.len() != 4 + count * 16
            || u32::from_le_bytes(bytes[..4].try_into().unwrap()) as usize != count
        {
            return Err("GT4 edge/station alignment mismatch".into());
        }
    }
    let length = data.length_metres;
    if !length.is_finite() || length <= 0.0 || !data.start_v.is_finite() || !data.goal_v.is_finite()
    {
        return Err("invalid GT4 course distance".into());
    }
    let mut source = Vec::with_capacity(count);
    for (index, record) in data.stations[4..].chunks_exact(32).enumerate() {
        let f = |i| f32::from_le_bytes(record[i..i + 4].try_into().unwrap()) as f64;
        let mut row = [0.0; 10];
        row[..6].copy_from_slice(&[f(0), f(4), f(8), f(12), f(16), f(20)]);
        if let Some(bytes) = edges {
            let e = &bytes[4 + index * 16..4 + (index + 1) * 16];
            for i in 0..4 {
                row[6 + i] = f32::from_le_bytes(e[i * 4..i * 4 + 4].try_into().unwrap()) as f64;
            }
        }
        if !row.iter().all(|x| x.is_finite()) || row[4] <= 0.0 || row[5] <= 0.0 {
            return Err("GT4 station has unknown road geometry".into());
        }
        source.push(row);
    }
    source.sort_by(|a, b| a[0].total_cmp(&b[0]));
    if source.windows(2).any(|p| p[1][0] - p[0][0] < 1e-5) {
        return Err("ambiguous GT4 progress stations".into());
    }
    // Parameterize by native TrackV, retaining source scale and elevation. Only
    // interpolate the route origin/end within the measured corridor intervals.
    let sample = |v: f64| -> [f64; 10] {
        // A partial open route has no cyclic return segment. Metadata/native
        // endpoints may differ by one f32 ULP; retain the last real position
        // rather than constructing a microscopic backwards finish tangent.
        // A sprint may be a native race section of a cyclic source road. Its
        // finish still interpolates across that road's V wrap; clamping to the
        // last sample would duplicate its position at a different progress value.
        let cyclic_source = metadata.source_topology == 0 && data.start_v == data.goal_v;
        if !closed && !cyclic_source {
            if v <= source[0][0] {
                return source[0];
            }
            if v >= source[count - 1][0] {
                return source[count - 1];
            }
        }
        let v = v.rem_euclid(length);
        let upper = source.partition_point(|p| p[0] <= v);
        let (mut a, mut b) = if upper == 0 {
            (source[count - 1], source[0])
        } else if upper == count {
            (source[count - 1], source[0])
        } else {
            (source[upper - 1], source[upper])
        };
        let mut p = v;
        if b[0] <= a[0] {
            b[0] += length;
            if p < a[0] {
                p += length;
            }
        }
        let t = ((p - a[0]) / (b[0] - a[0])).clamp(0.0, 1.0);
        for i in 1..10 {
            a[i] += (b[i] - a[i]) * t;
        }
        a[0] = v;
        a
    };
    let area = metadata.topology == CourseTopology::Area;
    // Native topology 2 has no race finish. Its header endpoint can lie beyond
    // physical ground (gymkhana_spral). Stations only provide a camera/spawn
    // navigation hint; the whole original mesh remains available for practice.
    let source_start = if area {
        source[0][0]
    } else if metadata.section.is_some() {
        f64::from(metadata.start_v)
    } else {
        data.start_v
    };
    let source_goal = if area {
        source[count - 1][0]
    } else if metadata.section.is_some() {
        f64::from(metadata.goal_v)
    } else {
        data.goal_v
    };
    let route_length = if closed {
        length
    } else {
        let d = (source_goal - source_start).rem_euclid(length);
        if d < 0.01 {
            return Err("GT4 sprint needs distinct source start and finish".into());
        }
        d
    };
    let start = source_start.rem_euclid(length);
    let mut ordered = Vec::with_capacity(count + 2);
    let mut first = sample(start);
    first[0] = 0.0;
    ordered.push(first);
    for mut row in source.iter().copied() {
        row[0] = (row[0] - start).rem_euclid(length);
        if row[0] > 1e-5 && (closed || row[0] < route_length - 1e-5) {
            ordered.push(row);
        }
    }
    ordered.sort_by(|a, b| a[0].total_cmp(&b[0]));
    if !closed {
        let mut end = sample(source_goal);
        end[0] = route_length;
        if ordered.last().is_some_and(|last| {
            (route_length - last[0]).abs() < 0.001
                && (last[1] - end[1]).hypot(last[2] - end[2]) < 0.00001
        }) {
            // Same physical endpoint at adjacent native f32 progress values.
            // Keep the authoritative section distance without a zero-length edge.
            *ordered.last_mut().unwrap() = end;
        } else {
            ordered.push(end);
        }
    }
    let stations = ordered
        .into_iter()
        .map(|r| RecoveredStation {
            arc: r[0] * M,
            point: Vec2 {
                x: r[1] * M,
                y: r[2] * M,
            },
            height: r[3] as f32,
            left: r[4] * M,
            right: r[5] * M,
            edges: edges.map(|_| {
                [
                    Vec2 {
                        x: r[6] * M,
                        y: r[7] * M,
                    },
                    Vec2 {
                        x: r[8] * M,
                        y: r[9] * M,
                    },
                ]
            }),
        })
        .collect();
    let mut mesh = RoadMesh::decode(data.mesh)?;
    let boundaries = boundary_assets()
        .iter()
        .find(|(id, _)| *id == metadata.id)
        .ok_or("GT4 native boundary asset missing")?;
    mesh.attach_boundaries(boundaries.1)?;
    let mut track = Track::from_recovered(
        Circuit::Gt4(index),
        closed,
        stations,
        route_length * M,
        mesh,
    )?;
    if let Some((_, bytes)) = visual_assets().iter().find(|(id, _)| *id == metadata.id) {
        track.attach_source_visual(bytes);
    }
    Ok(track)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn every_bundled_visual_preserves_source_faces_and_supported_texture_addressing() {
        let assets = visual_assets();
        assert!(
            !assets.is_empty(),
            "source display asset table must be generated"
        );
        for &(id, bytes) in assets {
            let source =
                crate::SourceVisual::decode_gzip(bytes).unwrap_or_else(|e| panic!("{id}: {e}"));
            assert!(!source.triangles.is_empty(), "{id}: empty visual mesh");
            let display = source
                .prepare_for_clamp()
                .unwrap_or_else(|e| panic!("{id}: texture preparation: {e}"));
            let mut source_areas = vec![0.0f64; source.triangles.len()];
            let mut rendered_areas = vec![0.0f64; source.triangles.len()];
            let area = |v: [Vec2; 3]| {
                let a = v[1] - v[0];
                let b = v[2] - v[0];
                (a.x * b.y - a.y * b.x).abs() * 0.5
            };
            for (i, t) in source.triangles.iter().enumerate() {
                source_areas[i] = area(t.indices.map(|v| source.vertices[v as usize].position));
            }
            for t in &display.triangles {
                assert!(
                    t.vertices.iter().all(|v| v
                        .uv
                        .iter()
                        .all(|&x| x.is_finite() && (-0.0001..=1.0001).contains(&x))),
                    "{id}: invalid prepared UV"
                );
                rendered_areas[t.source_triangle] += area(t.vertices.map(|v| v.position));
            }
            for (i, (&a, &b)) in source_areas.iter().zip(&rendered_areas).enumerate() {
                assert!(
                    (a - b).abs() <= 0.002 + a * 0.0001,
                    "{id}: source face {i} changed area {a}->{b}"
                );
            }
            // Every bundled texture record must be usable by the tile
            // rasterizer; a face missing its texture would silently vanish.
            let faces = display.triangles.len();
            let rasterizer = crate::SourceGroundRasterizer::new(display);
            assert_eq!(
                rasterizer.untextured_faces(),
                0,
                "{id}: prepared face lacks a usable texture record"
            );
            assert_eq!(
                rasterizer.triangle_count() + rasterizer.degenerate_faces(),
                faces,
                "{id}: source faces unaccounted for"
            );
        }
    }
    #[test]
    fn practice_ground_and_camera_bounds_are_not_limited_to_the_navigation_strip() {
        let index = crate::course_catalog::courses()
            .iter()
            .position(|c| c.id == "gymkhana")
            .unwrap();
        let track = build(index as u16).unwrap();
        assert!(track.free_roam);
        let ground = track.practice_ground_triangles();
        assert!(!ground.is_empty());
        let mut extends_past_guide = false;
        for point in ground.iter().flatten() {
            assert!(point.x >= track.bounds.min_x && point.x <= track.bounds.max_x);
            assert!(point.y >= track.bounds.min_y && point.y <= track.bounds.max_y);
            let hit = track.nearest_on_centerline(*point);
            if (*point - track.arc_point(hit.arc)).norm() > 5.0 * M {
                extends_past_guide = true;
            }
        }
        assert!(
            extends_past_guide,
            "practice ground must retain the original area beyond its guide"
        );
    }
}
