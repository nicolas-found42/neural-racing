//! Prepare GS per-axis addressing for a single clamp sampler without stretching UVs.
use crate::{SourceVisual, SourceVisualTexture, Vec2};

#[derive(Clone, Copy, Debug)]
pub struct PreparedVisualVertex {
    pub position: Vec2,
    pub height_metres: f32,
    pub uv: [f32; 2],
    pub color_rgba: [u8; 4],
}
#[derive(Clone, Debug)]
pub struct PreparedVisualTriangle {
    pub vertices: [PreparedVisualVertex; 3],
    pub texture: u32,
    pub source_triangle: usize,
}
#[derive(Clone, Debug)]
pub struct PreparedVisualTexture {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}
#[derive(Clone, Debug)]
pub struct PreparedSourceVisual {
    pub textures: Vec<PreparedVisualTexture>,
    pub triangles: Vec<PreparedVisualTriangle>,
}

#[derive(Clone, Copy)]
struct Axis {
    original: usize,
    size: usize,
    minimum: usize,
    maximum: usize,
    mode: u64,
}
impl Axis {
    fn new(size: u32, clamp: u64, axis: usize) -> Result<Self, String> {
        let mode = (clamp >> (axis * 2)) & 3;
        let minimum = ((clamp >> (4 + axis * 20)) & 1023) as usize;
        let maximum = ((clamp >> (14 + axis * 20)) & 1023) as usize;
        let original = size as usize;
        let size = match mode {
            2 => {
                if minimum > maximum || maximum >= original {
                    return Err("GT4 region clamp exceeds decoded texture".into());
                }
                maximum - minimum + 1
            }
            3 => (minimum + 1).next_power_of_two(),
            _ => original,
        };
        if mode == 3 && (minimum | maximum) >= original {
            return Err("GT4 region repeat exceeds decoded texture".into());
        }
        Ok(Self {
            original,
            size,
            minimum,
            maximum,
            mode,
        })
    }
    fn repeats(self) -> bool {
        self.mode == 0 || self.mode == 3
    }
    fn coordinate(self, uv: f32) -> f32 {
        match self.mode {
            2 => (uv * self.original as f32 - self.minimum as f32) / self.size as f32,
            3 => uv * self.original as f32 / self.size as f32,
            _ => uv,
        }
    }
    fn pixel(self, padded: usize) -> usize {
        let i = padded as isize - 1;
        let i = if self.repeats() {
            i.rem_euclid(self.size as isize) as usize
        } else {
            i.clamp(0, self.size as isize - 1) as usize
        };
        match self.mode {
            2 => i + self.minimum,
            3 => (i & self.minimum) | self.maximum,
            _ => i,
        }
    }
    fn padded_uv(self, coordinate: f32, band: i32) -> f32 {
        let u = if self.repeats() {
            coordinate - band as f32
        } else {
            coordinate.clamp(0.0, 1.0)
        };
        (u * self.size as f32 + 1.0) / (self.size + 2) as f32
    }
}

fn prepare_texture(
    texture: &SourceVisualTexture,
) -> Result<(PreparedVisualTexture, [Axis; 2]), String> {
    let axes = [
        Axis::new(texture.width, texture.gs_clamp, 0)?,
        Axis::new(texture.height, texture.gs_clamp, 1)?,
    ];
    let width = axes[0].size + 2;
    let height = axes[1].size + 2;
    let mut rgba = Vec::with_capacity(width * height * 4);
    let tcc = (texture.gs_tex0 >> 34) & 1;
    for y in 0..height {
        for x in 0..width {
            let offset = (axes[1].pixel(y) * texture.width as usize + axes[0].pixel(x)) * 4;
            let mut color: [u8; 4] = texture.rgba[offset..offset + 4].try_into().unwrap();
            // TCC=RGB leaves opacity entirely to the vertex color.
            if tcc == 0 {
                color[3] = 255;
            }
            rgba.extend(color);
        }
    }
    Ok((
        PreparedVisualTexture {
            width: width as u32,
            height: height as u32,
            rgba,
        },
        axes,
    ))
}

#[derive(Clone, Copy)]
struct Vertex {
    position: Vec2,
    height: f32,
    uv: [f32; 2],
    color: [f32; 4],
}
fn clip(polygon: Vec<Vertex>, axis: usize, boundary: f32, sign: f32) -> Vec<Vertex> {
    let mut output = Vec::new();
    let Some(mut previous) = polygon.last().copied() else {
        return output;
    };
    let mut before = (previous.uv[axis] - boundary) * sign;
    for current in polygon {
        let after = (current.uv[axis] - boundary) * sign;
        if (before >= 0.0) != (after >= 0.0) {
            let t = (before / (before - after)).clamp(0.0, 1.0);
            output.push(Vertex {
                position: previous.position + (current.position - previous.position) * f64::from(t),
                height: previous.height + (current.height - previous.height) * t,
                uv: std::array::from_fn(|i| previous.uv[i] + (current.uv[i] - previous.uv[i]) * t),
                color: std::array::from_fn(|i| {
                    previous.color[i] + (current.color[i] - previous.color[i]) * t
                }),
            });
        }
        if after >= 0.0 {
            output.push(current);
        }
        previous = current;
        before = after;
    }
    output
}

impl SourceVisual {
    pub fn prepare_for_clamp(&self) -> Result<PreparedSourceVisual, String> {
        let mut prepared = PreparedSourceVisual {
            textures: Vec::new(),
            triangles: Vec::new(),
        };
        let mut addressing = Vec::new();
        for texture in &self.textures {
            let (image, axes) = prepare_texture(texture)?;
            prepared.textures.push(image);
            addressing.push(axes);
        }
        // Splitting only subdivides an authored face; every output point
        // retains that face's affine height.
        for (source, triangle) in self.triangles.iter().enumerate() {
            let texture = &self.textures[triangle.texture as usize];
            let axes = addressing[triangle.texture as usize];
            let tfx = (texture.gs_tex0 >> 35) & 3;
            if tfx > 1 {
                return Err(format!("unsupported GT4 texture effect {tfx}"));
            }
            let tcc = (texture.gs_tex0 >> 34) & 1;
            let polygon: Vec<_> = triangle
                .indices
                .iter()
                .map(|&i| {
                    let v = self.vertices[i as usize];
                    let mut color = v
                        .native_rgba
                        .map(|x| (f32::from(x) * 255.0 / 128.0).min(255.0));
                    if tfx == 1 {
                        color[..3].fill(255.0);
                        if tcc == 1 {
                            color[3] = 255.0;
                        }
                    }
                    Vertex {
                        position: v.position,
                        height: v.height_metres,
                        uv: std::array::from_fn(|a| axes[a].coordinate(v.uv[a])),
                        color,
                    }
                })
                .collect();
            let mut pieces = vec![(polygon, [0i32; 2])];
            for axis in 0..2 {
                if !axes[axis].repeats() {
                    let mut next = Vec::new();
                    for (polygon, bands) in pieces {
                        let minimum = polygon
                            .iter()
                            .map(|v| v.uv[axis])
                            .fold(f32::INFINITY, f32::min);
                        let maximum = polygon
                            .iter()
                            .map(|v| v.uv[axis])
                            .fold(f32::NEG_INFINITY, f32::max);
                        if minimum == maximum || (minimum >= 0.0 && maximum <= 1.0) {
                            next.push((polygon, bands));
                            continue;
                        }
                        // Clamping vertices without splitting would stretch the
                        // interior when a face crosses a region boundary.
                        for p in [
                            clip(polygon.clone(), axis, 0.0, -1.0),
                            clip(clip(polygon.clone(), axis, 0.0, 1.0), axis, 1.0, -1.0),
                            clip(polygon, axis, 1.0, 1.0),
                        ] {
                            if p.len() >= 3 {
                                next.push((p, bands));
                            }
                        }
                    }
                    pieces = next;
                    continue;
                }
                let mut next = Vec::new();
                for (polygon, bands) in pieces {
                    let minimum = polygon
                        .iter()
                        .map(|v| v.uv[axis])
                        .fold(f32::INFINITY, f32::min);
                    let maximum = polygon
                        .iter()
                        .map(|v| v.uv[axis])
                        .fold(f32::NEG_INFINITY, f32::max);
                    if minimum.abs() > 1e6 || maximum.abs() > 1e6 || maximum - minimum > 4096.0 {
                        return Err("GT4 visual UV span exceeds preparation budget".into());
                    }
                    let first = minimum.floor() as i32;
                    let last = (maximum.ceil() as i32 - 1).max(first);
                    for band in first..=last {
                        let p = clip(
                            clip(polygon.clone(), axis, band as f32, 1.0),
                            axis,
                            (band + 1) as f32,
                            -1.0,
                        );
                        if p.len() >= 3 {
                            let mut b = bands;
                            b[axis] = band;
                            next.push((p, b));
                        }
                    }
                }
                pieces = next;
            }
            for (p, bands) in pieces {
                for i in 1..p.len() - 1 {
                    let v = [p[0], p[i], p[i + 1]];
                    let a = v[1].position - v[0].position;
                    let b = v[2].position - v[0].position;
                    if (a.x * b.y - a.y * b.x).abs() < 1e-9 {
                        continue;
                    }
                    prepared.triangles.push(PreparedVisualTriangle {
                        texture: triangle.texture,
                        source_triangle: source,
                        vertices: v.map(|v| PreparedVisualVertex {
                            position: v.position,
                            height_metres: v.height,
                            uv: std::array::from_fn(|a| axes[a].padded_uv(v.uv[a], bands[a])),
                            color_rgba: v.color.map(|x| x.round().clamp(0., 255.) as u8),
                        }),
                    });
                }
            }
            if prepared.triangles.len() > 8_000_000 {
                return Err("GT4 prepared visual exceeds triangle budget".into());
            }
        }
        Ok(prepared)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn faces_crossing_mixed_wrap_boundaries_keep_the_authored_pixel_addresses() {
        for (s, t) in [(0u64, 1u64), (2, 0), (3, 2)] {
            let clamp = s | (t << 2) | (1 << 4) | (2 << 14) | (1 << 24) | (2 << 34);
            let vertices = [[0f64, 0.], [4., 0.], [0., 4.]]
                .map(|p| crate::SourceVisualVertex {
                    position: Vec2 { x: p[0], y: p[1] },
                    height_metres: 0.,
                    uv: [(p[0] * 0.75 - 0.7) as f32, (p[1] * 0.75 - 0.7) as f32],
                    native_rgba: [128; 4],
                })
                .to_vec();
            let rgba = (0u8..4)
                .flat_map(|y| (0u8..4).flat_map(move |x| [x, y, 0, 255]))
                .collect();
            let source = SourceVisual {
                vertices,
                triangles: vec![crate::SourceVisualTriangle {
                    indices: [0, 1, 2],
                    texture: 0,
                    source_model: 0,
                    source_shape: 0,
                    source_packet_offset: 0,
                }],
                textures: vec![SourceVisualTexture {
                    width: 4,
                    height: 4,
                    gs_clamp: clamp,
                    gs_tex0: 1 << 34,
                    rgba,
                    source_tex1_offset: 0,
                    source_texture_index: 0,
                }],
            };
            let display = source.prepare_for_clamp().unwrap();
            let texture = &display.textures[0];
            for triangle in display.triangles {
                let x = triangle.vertices.iter().map(|v| v.position.x).sum::<f64>() / 3.;
                let y = triangle.vertices.iter().map(|v| v.position.y).sum::<f64>() / 3.;
                let mut expected = [0u8; 2];
                for (axis, coordinate) in [x, y].into_iter().enumerate() {
                    let pixel = ((coordinate * 0.75 - 0.7) * 4.).floor() as i32;
                    expected[axis] = match if axis == 0 { s } else { t } {
                        0 => pixel.rem_euclid(4),
                        1 => pixel.clamp(0, 3),
                        2 => pixel.clamp(1, 2),
                        _ => (pixel & 1) | 2,
                    } as u8;
                }
                let uv: [f32; 2] = std::array::from_fn(|a| {
                    triangle.vertices.iter().map(|v| v.uv[a]).sum::<f32>() / 3.
                });
                let tx = (uv[0] * texture.width as f32).floor() as usize;
                let ty = (uv[1] * texture.height as f32).floor() as usize;
                let at = (ty * texture.width as usize + tx) * 4;
                assert_eq!(&texture.rgba[at..at + 2], expected, "wrap{s}/{t} at{x}/{y}");
            }
        }
    }
    #[test]
    fn per_axis_region_addressing_matches_gs_integer_formula_including_seams() {
        for (mode, min, max, size) in [(0, 0, 0, 16), (1, 0, 0, 16), (2, 3, 9, 16), (3, 5, 8, 16)] {
            let clamp = mode | ((min as u64) << 4) | ((max as u64) << 14);
            let axis = Axis::new(size, clamp, 0).unwrap();
            for pixel in -40i32..40 {
                let uv = (pixel as f32 + 0.5) / size as f32;
                let coordinate = axis.coordinate(uv);
                let band = if axis.repeats() {
                    coordinate.floor() as i32
                } else {
                    0
                };
                let prepared = axis.padded_uv(coordinate, band);
                let index = (prepared * (axis.size + 2) as f32).floor() as usize;
                let actual = axis.pixel(index.min(axis.size + 1));
                let expected = match mode {
                    0 => pixel.rem_euclid(size as i32) as usize,
                    1 => pixel.clamp(0, size as i32 - 1) as usize,
                    2 => pixel.clamp(min, max) as usize,
                    _ => ((pixel as usize) & min as usize) | max as usize,
                };
                assert_eq!(actual, expected, "mode{mode} pixel{pixel}");
            }
            if axis.repeats() {
                assert_eq!(axis.pixel(0), axis.pixel(axis.size));
                assert_eq!(axis.pixel(axis.size + 1), axis.pixel(1));
            }
        }
    }
    #[test]
    fn uv_split_preserves_source_plane_and_area() {
        let v = |x: f64, y: f64, u: f32| Vertex {
            position: Vec2 { x, y },
            height: (x + 2. * y) as f32,
            uv: [u, 0.],
            color: [128.; 4],
        };
        let p = vec![v(0., 0., -0.5), v(4., 0., 1.5), v(0., 4., -0.5)];
        let mut area = 0.;
        for band in -1..=1 {
            let clipped = clip(
                clip(p.clone(), 0, band as f32, 1.),
                0,
                (band + 1) as f32,
                -1.,
            );
            for x in &clipped {
                assert!((x.height - (x.position.x + 2. * x.position.y) as f32).abs() < 1e-5);
            }
            for i in 1..clipped.len().saturating_sub(1) {
                let a = clipped[i].position - clipped[0].position;
                let b = clipped[i + 1].position - clipped[0].position;
                area += (a.x * b.y - a.y * b.x) / 2.;
            }
        }
        assert!((area - 8.0f64).abs() < 1e-6, "area={area}");
    }
}
