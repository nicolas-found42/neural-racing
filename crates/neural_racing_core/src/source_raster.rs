//! Exact top-down compositing of the original GT4 ground faces.
//!
//! The original ground is a set of overlapping, sloped and partly translucent
//! faces. Sorting whole faces by one height cannot order them: at one screen
//! pixel face A is above face B, at another the reverse. Faces are therefore
//! rasterised into world-aligned tiles and composited per pixel by their
//! interpolated height, back to front, keeping the original texture, vertex
//! colour and alpha.

use crate::{PreparedSourceVisual, UNITS_PER_METRE};

/// Square tile edge in pixels.
pub const TILE_PIXELS: u32 = 256;

/// A world-aligned tile grid at one fixed sampling density.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct GroundTileGrid {
    level: u8,
}

impl GroundTileGrid {
    /// Power-of-two densities keep tiles stable while zooming and let a tile
    /// cover a whole number of metres at every level.
    pub const DENSITIES: [f64; 8] = [0.125, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0];

    /// Level `0` is the coarsest overview; higher levels sample more densely.
    pub fn new(level: u8) -> Option<Self> {
        ((level as usize) < Self::DENSITIES.len()).then_some(Self { level })
    }

    /// Coarsest level that is at least as dense as `samples_per_metre`, so a
    /// narrow road is never sampled below one texel per displayed pixel. The
    /// caller may still coarsen this when the visible tile budget demands it.
    pub fn at_least(samples_per_metre: f64) -> Self {
        let wanted = if samples_per_metre.is_finite() {
            samples_per_metre.max(0.0)
        } else {
            1.0
        };
        let level = Self::DENSITIES
            .iter()
            .position(|density| *density >= wanted)
            .unwrap_or(Self::DENSITIES.len() - 1);
        Self { level: level as u8 }
    }

    pub fn level(self) -> u8 {
        self.level
    }

    pub fn pixels_per_metre(self) -> f64 {
        Self::DENSITIES[self.level as usize]
    }

    pub fn metres_per_tile(self) -> f64 {
        f64::from(TILE_PIXELS) / self.pixels_per_metre()
    }

    /// Inclusive tile index range covering a world-space (metres) rectangle.
    pub fn covering(self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> (i32, i32, i32, i32) {
        let metres = self.metres_per_tile();
        let index = |v: f64| (v / metres).floor() as i32;
        (index(min_x), index(min_y), index(max_x), index(max_y))
    }
}

#[derive(Clone, Copy)]
struct Face {
    position: [[f64; 2]; 3],
    height: [f32; 3],
    uv: [[f32; 2]; 3],
    rgba: [[u8; 4]; 3],
    texture: u32,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    area: f64,
}

struct Texture {
    width: usize,
    height: usize,
    rgba: Vec<u8>,
}

#[derive(Clone, Copy)]
struct Fragment {
    height: f32,
    rgba: [u8; 4],
    next: u32,
}

struct CellGrid {
    origin_x: i32,
    origin_y: i32,
    width: i32,
    height: i32,
    metres: f64,
    cells: Vec<Vec<u32>>,
    /// Faces too large for the cell budget; tested against every tile.
    large: Vec<u32>,
}

impl CellGrid {
    fn cell_x(&self, x: f64) -> i32 {
        (x / self.metres).floor() as i32
    }

    fn cell_y(&self, y: f64) -> i32 {
        (y / self.metres).floor() as i32
    }
}

/// Prepared original ground, ready for repeated tile rasterisation.
pub struct SourceGroundRasterizer {
    faces: Vec<Face>,
    textures: Vec<Texture>,
    grid: CellGrid,
    bounds: (f64, f64, f64, f64),
    degenerate: usize,
    untextured: usize,
}

impl SourceGroundRasterizer {
    pub fn new(visual: PreparedSourceVisual) -> Self {
        let textures: Vec<Texture> = visual
            .textures
            .iter()
            .map(|texture| {
                let width = texture.width as usize;
                let height = texture.height as usize;
                // A truncated texture would otherwise index past its bytes on
                // the worker thread. Blank it; its faces are skipped below.
                let rgba = if texture.rgba.len() == width * height * 4 {
                    texture.rgba.clone()
                } else {
                    Vec::new()
                };
                Texture {
                    width,
                    height,
                    rgba,
                }
            })
            .collect();
        let mut faces = Vec::with_capacity(visual.triangles.len());
        let mut degenerate = 0;
        let mut untextured = 0;
        for triangle in &visual.triangles {
            let usable = textures
                .get(triangle.texture as usize)
                .is_some_and(|texture| !texture.rgba.is_empty());
            if !usable {
                untextured += 1;
                continue;
            }
            let position = triangle.vertices.map(|v| {
                [
                    v.position.x / UNITS_PER_METRE,
                    v.position.y / UNITS_PER_METRE,
                ]
            });
            let area = (position[1][0] - position[0][0]) * (position[2][1] - position[0][1])
                - (position[1][1] - position[0][1]) * (position[2][0] - position[0][0]);
            if !area.is_finite() || area.abs() < 1e-9 {
                degenerate += 1;
                continue;
            }
            let mut min_x = f64::INFINITY;
            let mut min_y = f64::INFINITY;
            let mut max_x = f64::NEG_INFINITY;
            let mut max_y = f64::NEG_INFINITY;
            for p in position {
                min_x = min_x.min(p[0]);
                min_y = min_y.min(p[1]);
                max_x = max_x.max(p[0]);
                max_y = max_y.max(p[1]);
            }
            faces.push(Face {
                position,
                height: triangle.vertices.map(|v| v.height_metres),
                uv: triangle.vertices.map(|v| v.uv),
                rgba: triangle.vertices.map(|v| v.color_rgba),
                texture: triangle.texture,
                min_x,
                min_y,
                max_x,
                max_y,
                area,
            });
        }
        let grid = CellGrid::build(&faces);
        let mut bounds = (
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        );
        for face in &faces {
            bounds.0 = bounds.0.min(face.min_x);
            bounds.1 = bounds.1.min(face.min_y);
            bounds.2 = bounds.2.max(face.max_x);
            bounds.3 = bounds.3.max(face.max_y);
        }
        Self {
            faces,
            textures,
            grid,
            bounds,
            degenerate,
            untextured,
        }
    }

    pub fn triangle_count(&self) -> usize {
        self.faces.len()
    }

    /// Zero-area faces, which cover no pixel at any density.
    pub fn degenerate_faces(&self) -> usize {
        self.degenerate
    }

    /// Faces skipped because their texture record was absent or truncated.
    pub fn untextured_faces(&self) -> usize {
        self.untextured
    }

    /// World-space (metres) extent of the original ground, or `None` when the
    /// source contained no drawable face.
    pub fn bounds(&self) -> Option<(f64, f64, f64, f64)> {
        self.bounds.0.is_finite().then_some(self.bounds)
    }

    /// RGBA tile for `grid`, including transparent pixels outside any face.
    pub fn render_tile(&self, grid: GroundTileGrid, tile_x: i32, tile_y: i32) -> Vec<u8> {
        let metres = grid.metres_per_tile();
        self.render(
            f64::from(tile_x) * metres,
            f64::from(tile_y) * metres,
            grid.pixels_per_metre(),
            TILE_PIXELS,
        )
    }

    /// Rasterise a world-space (metres) rectangle with `pixels` square pixels.
    pub fn render(&self, min_x: f64, min_y: f64, pixels_per_metre: f64, pixels: u32) -> Vec<u8> {
        if !min_x.is_finite()
            || !min_y.is_finite()
            || !pixels_per_metre.is_finite()
            || pixels_per_metre <= 0.0
            || pixels == 0
            || pixels > 4096
        {
            return Vec::new();
        }
        let size = pixels as usize;
        let step = 1.0 / pixels_per_metre;
        let max_x = min_x + size as f64 * step;
        let max_y = min_y + size as f64 * step;
        let mut candidates = self.grid.candidates(min_x, min_y, max_x, max_y);
        candidates.sort_unstable();
        candidates.dedup();

        let pixels_total = size * size;
        let mut heads = vec![u32::MAX; pixels_total];
        let mut fragments: Vec<Fragment> = Vec::new();
        for &index in &candidates {
            let face = &self.faces[index as usize];
            if face.max_x < min_x || face.min_x > max_x || face.max_y < min_y || face.min_y > max_y
            {
                continue;
            }
            let texture = &self.textures[face.texture as usize];
            let x0 = ((face.min_x - min_x) * pixels_per_metre).floor().max(0.0) as usize;
            let x1 = (((face.max_x - min_x) * pixels_per_metre).ceil() as isize)
                .clamp(0, size as isize) as usize;
            let y0 = ((face.min_y - min_y) * pixels_per_metre).floor().max(0.0) as usize;
            let y1 = (((face.max_y - min_y) * pixels_per_metre).ceil() as isize)
                .clamp(0, size as isize) as usize;
            if x0 >= x1 || y0 >= y1 {
                continue;
            }
            let a = face.position[0];
            let b = face.position[1];
            let c = face.position[2];
            let inverse_area = 1.0 / face.area;
            for y in y0..y1 {
                let world_y = min_y + (y as f64 + 0.5) * step;
                for x in x0..x1 {
                    let world_x = min_x + (x as f64 + 0.5) * step;
                    let w0 = ((b[0] - world_x) * (c[1] - world_y)
                        - (b[1] - world_y) * (c[0] - world_x))
                        * inverse_area;
                    let w1 = ((c[0] - world_x) * (a[1] - world_y)
                        - (c[1] - world_y) * (a[0] - world_x))
                        * inverse_area;
                    let w2 = 1.0 - w0 - w1;
                    if w0 < -1e-7 || w1 < -1e-7 || w2 < -1e-7 {
                        continue;
                    }
                    let height = w0 as f32 * face.height[0]
                        + w1 as f32 * face.height[1]
                        + w2 as f32 * face.height[2];
                    let u = w0 as f32 * face.uv[0][0]
                        + w1 as f32 * face.uv[1][0]
                        + w2 as f32 * face.uv[2][0];
                    let v = w0 as f32 * face.uv[0][1]
                        + w1 as f32 * face.uv[1][1]
                        + w2 as f32 * face.uv[2][1];
                    let mut color = sample(texture, u, v);
                    let shade = |channel: usize| {
                        w0 as f32 * f32::from(face.rgba[0][channel])
                            + w1 as f32 * f32::from(face.rgba[1][channel])
                            + w2 as f32 * f32::from(face.rgba[2][channel])
                    };
                    for channel in 0..4 {
                        color[channel] = color[channel] * shade(channel) / 255.0;
                    }
                    let alpha = color[3].round().clamp(0.0, 255.0) as u8;
                    if alpha == 0 {
                        continue;
                    }
                    let pixel = y * size + x;
                    fragments.push(Fragment {
                        height,
                        rgba: [
                            color[0].round().clamp(0.0, 255.0) as u8,
                            color[1].round().clamp(0.0, 255.0) as u8,
                            color[2].round().clamp(0.0, 255.0) as u8,
                            alpha,
                        ],
                        next: heads[pixel],
                    });
                    heads[pixel] = (fragments.len() - 1) as u32;
                }
            }
        }

        let mut rgba = vec![0u8; pixels_total * 4];
        let mut stack: Vec<u32> = Vec::new();
        for (pixel, &head) in heads.iter().enumerate() {
            if head == u32::MAX {
                continue;
            }
            stack.clear();
            let mut next = head;
            while next != u32::MAX {
                stack.push(next);
                next = fragments[next as usize].next;
            }
            // Source order is the stable tie-break for equal heights.
            stack.sort_unstable_by(|a, b| {
                fragments[*a as usize]
                    .height
                    .total_cmp(&fragments[*b as usize].height)
                    .then(a.cmp(b))
            });
            let mut accumulated = [0f32; 4];
            for &index in &stack {
                let fragment = &fragments[index as usize];
                let alpha = f32::from(fragment.rgba[3]) / 255.0;
                let keep = 1.0 - alpha;
                for channel in 0..3 {
                    accumulated[channel] =
                        f32::from(fragment.rgba[channel]) * alpha + accumulated[channel] * keep;
                }
                accumulated[3] = alpha + accumulated[3] * keep;
            }
            let alpha = accumulated[3];
            if alpha <= 0.0 {
                continue;
            }
            let at = pixel * 4;
            for channel in 0..3 {
                rgba[at + channel] = (accumulated[channel] / alpha).round().clamp(0.0, 255.0) as u8;
            }
            rgba[at + 3] = (alpha * 255.0).round().clamp(0.0, 255.0) as u8;
        }
        rgba
    }
}

impl CellGrid {
    fn build(faces: &[Face]) -> Self {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        for face in faces {
            min_x = min_x.min(face.min_x);
            min_y = min_y.min(face.min_y);
            max_x = max_x.max(face.max_x);
            max_y = max_y.max(face.max_y);
        }
        if faces.is_empty() {
            min_x = 0.0;
            min_y = 0.0;
            max_x = 1.0;
            max_y = 1.0;
        }
        let mut metres = 32.0;
        let (origin_x, origin_y, width, height) = loop {
            let origin_x = (min_x / metres).floor() as i32;
            let origin_y = (min_y / metres).floor() as i32;
            let width = ((max_x / metres).floor() as i32).saturating_sub(origin_x) + 1;
            let height = ((max_y / metres).floor() as i32).saturating_sub(origin_y) + 1;
            if i64::from(width) * i64::from(height) <= 4_000_000 || metres > 1e6 {
                break (origin_x, origin_y, width, height);
            }
            metres *= 2.0;
        };
        let mut grid = Self {
            origin_x,
            origin_y,
            width,
            height,
            metres,
            cells: vec![Vec::new(); width.max(1) as usize * height.max(1) as usize],
            large: Vec::new(),
        };
        for (index, face) in faces.iter().enumerate() {
            let x0 = grid
                .cell_x(face.min_x)
                .clamp(origin_x, origin_x + width - 1);
            let x1 = grid
                .cell_x(face.max_x)
                .clamp(origin_x, origin_x + width - 1);
            let y0 = grid
                .cell_y(face.min_y)
                .clamp(origin_y, origin_y + height - 1);
            let y1 = grid
                .cell_y(face.max_y)
                .clamp(origin_y, origin_y + height - 1);
            let span = i64::from(x1 - x0 + 1) * i64::from(y1 - y0 + 1);
            if span > 4096 {
                grid.large.push(index as u32);
                continue;
            }
            for y in y0..=y1 {
                for x in x0..=x1 {
                    let slot = grid.slot(x, y);
                    grid.cells[slot].push(index as u32);
                }
            }
        }
        grid
    }

    fn slot(&self, x: i32, y: i32) -> usize {
        ((y - self.origin_y) as usize) * self.width as usize + (x - self.origin_x) as usize
    }

    fn candidates(&self, min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Vec<u32> {
        let x0 = self
            .cell_x(min_x)
            .clamp(self.origin_x, self.origin_x + self.width - 1);
        let x1 = self
            .cell_x(max_x)
            .clamp(self.origin_x, self.origin_x + self.width - 1);
        let y0 = self
            .cell_y(min_y)
            .clamp(self.origin_y, self.origin_y + self.height - 1);
        let y1 = self
            .cell_y(max_y)
            .clamp(self.origin_y, self.origin_y + self.height - 1);
        let mut candidates = self.large.clone();
        for y in y0..=y1 {
            for x in x0..=x1 {
                candidates.extend_from_slice(&self.cells[self.slot(x, y)]);
            }
        }
        candidates
    }
}

fn sample(texture: &Texture, u: f32, v: f32) -> [f32; 4] {
    let width = texture.width;
    let height = texture.height;
    if width == 0 || height == 0 {
        return [0.0; 4];
    }
    let x = (u * width as f32 - 0.5).clamp(0.0, width as f32 - 1.0);
    let y = (v * height as f32 - 0.5).clamp(0.0, height as f32 - 1.0);
    let x0 = x.floor() as usize;
    let y0 = y.floor() as usize;
    let x1 = (x0 + 1).min(width - 1);
    let y1 = (y0 + 1).min(height - 1);
    let fx = x - x0 as f32;
    let fy = y - y0 as f32;
    let texel = |x: usize, y: usize| {
        let at = (y * width + x) * 4;
        [
            f32::from(texture.rgba[at]),
            f32::from(texture.rgba[at + 1]),
            f32::from(texture.rgba[at + 2]),
            f32::from(texture.rgba[at + 3]),
        ]
    };
    let top = texel(x0, y0);
    let top_right = texel(x1, y0);
    let bottom = texel(x0, y1);
    let bottom_right = texel(x1, y1);
    std::array::from_fn(|channel| {
        let a = top[channel] + (top_right[channel] - top[channel]) * fx;
        let b = bottom[channel] + (bottom_right[channel] - bottom[channel]) * fx;
        a + (b - a) * fy
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{PreparedVisualTexture, PreparedVisualTriangle, PreparedVisualVertex, Vec2};

    fn vertex(x: f64, y: f64, height: f32, uv: [f32; 2], rgba: [u8; 4]) -> PreparedVisualVertex {
        PreparedVisualVertex {
            position: Vec2 {
                x: x * UNITS_PER_METRE,
                y: y * UNITS_PER_METRE,
            },
            height_metres: height,
            uv,
            color_rgba: rgba,
        }
    }

    fn visual(
        triangles: Vec<PreparedVisualTriangle>,
        textures: Vec<PreparedVisualTexture>,
    ) -> PreparedSourceVisual {
        PreparedSourceVisual {
            triangles,
            textures,
        }
    }

    fn solid_triangle(
        points: [[f64; 2]; 3],
        heights: [f32; 3],
        texture: u32,
    ) -> PreparedVisualTriangle {
        PreparedVisualTriangle {
            vertices: std::array::from_fn(|i| {
                vertex(points[i][0], points[i][1], heights[i], [0.5, 0.5], [255; 4])
            }),
            texture,
            source_triangle: 0,
        }
    }

    fn solid_texture(rgba: [u8; 4]) -> PreparedVisualTexture {
        PreparedVisualTexture {
            width: 1,
            height: 1,
            rgba: rgba.to_vec(),
        }
    }

    /// Two overlapping faces cross in height: red is higher on the left, blue
    /// on the right. No single face order can draw both sides, so the second
    /// pixel proves per-pixel depth rather than a mean-height sort.
    #[test]
    fn crossing_faces_use_per_pixel_height_instead_of_one_face_order() {
        // Red rises left to right, blue falls; they overlap over the same area.
        let red = solid_triangle([[0., 0.], [10., 0.], [0., 10.]], [0., 1., 0.], 0);
        let blue = solid_triangle([[0.5, 0.5], [10.5, 0.5], [0.5, 10.5]], [1., 0., 1.], 1);
        let rasterizer = SourceGroundRasterizer::new(visual(
            vec![red, blue],
            vec![
                solid_texture([255, 0, 0, 255]),
                solid_texture([0, 0, 255, 255]),
            ],
        ));
        // Mean heights: red 1/3, blue 2/3. A mean sort paints blue everywhere.
        let image = rasterizer.render(0.0, 0.0, 10.0, 100);
        let at = |x: usize, y: usize| {
            let i = (y * 100 + x) * 4;
            [image[i], image[i + 1], image[i + 2], image[i + 3]]
        };
        assert_eq!(
            at(20, 20),
            [0, 0, 255, 255],
            "blue is higher at x=2 and must win"
        );
        assert_eq!(
            at(80, 10),
            [255, 0, 0, 255],
            "red is higher at x=8 and must win"
        );
    }

    #[test]
    fn malformed_texture_records_are_skipped_instead_of_panicking() {
        let face = solid_triangle([[0., 0.], [10., 0.], [0., 10.]], [0.; 3], 0);
        let missing = solid_triangle([[0., 0.], [10., 0.], [0., 10.]], [0.; 3], 7);
        let truncated = PreparedVisualTexture {
            width: 4,
            height: 4,
            rgba: vec![255; 4],
        };
        let rasterizer = SourceGroundRasterizer::new(visual(vec![face, missing], vec![truncated]));
        assert_eq!(rasterizer.triangle_count(), 0);
        assert_eq!(rasterizer.untextured_faces(), 2);
        assert_eq!(rasterizer.degenerate_faces(), 0);
        assert!(rasterizer.bounds().is_none());
        let image = rasterizer.render(0.0, 0.0, 4.0, 8);
        assert!(image.iter().all(|byte| *byte == 0));
    }

    #[test]
    fn translucent_faces_composite_over_lower_ground() {
        let ground = solid_triangle([[0., 0.], [10., 0.], [0., 10.]], [0.; 3], 0);
        let shadow = solid_triangle([[0., 0.], [10., 0.], [0., 10.]], [5.; 3], 1);
        let rasterizer = SourceGroundRasterizer::new(visual(
            vec![ground, shadow],
            vec![
                solid_texture([200, 200, 200, 255]),
                solid_texture([0, 0, 0, 128]),
            ],
        ));
        let image = rasterizer.render(1.0, 1.0, 8.0, 8);
        let pixel = [image[0], image[1], image[2], image[3]];
        assert_eq!(pixel[3], 255, "opaque ground must keep full coverage");
        assert!(
            (99..=101).contains(&pixel[0]) && pixel[0] == pixel[1] && pixel[1] == pixel[2],
            "half-alpha black shadow over grey leaves ~100, not black or grey: {pixel:?}"
        );
    }

    #[test]
    fn adjacent_tiles_reconstruct_the_same_pixels_as_one_image() {
        let colors = [
            [255u8, 0, 0, 255],
            [0, 255, 0, 255],
            [0, 0, 255, 128],
            [255, 255, 0, 200],
        ];
        let points = [
            [[0., 0.], [37., 4.], [9., 33.]],
            [[4., 2.], [40., 14.], [12., 35.]],
            [[0., 30.], [30., 0.], [12., 12.]],
            [[6., 6.], [42., 30.], [36., 2.]],
        ];
        let triangles = points
            .iter()
            .enumerate()
            .map(|(i, p)| solid_triangle(*p, [i as f32, i as f32 + 2.0, i as f32 - 1.0], i as u32))
            .collect();
        let rasterizer =
            SourceGroundRasterizer::new(visual(triangles, colors.map(solid_texture).to_vec()));
        let grid = GroundTileGrid::new(6).unwrap();
        let edge = TILE_PIXELS as usize;
        let whole = rasterizer.render(0.0, 0.0, 8.0, 2 * TILE_PIXELS);
        let mut tiled = vec![0u8; 4 * edge * edge * 4];
        for x in 0..2 {
            for y in 0..2 {
                let tile = rasterizer.render_tile(grid, x, y);
                for row in 0..edge {
                    let src = row * edge * 4;
                    let dst = ((y as usize * edge + row) * (2 * edge) + x as usize * edge) * 4;
                    tiled[dst..dst + edge * 4].copy_from_slice(&tile[src..src + edge * 4]);
                }
            }
        }
        assert_eq!(whole, tiled, "tile boundaries must not change any pixel");
    }

    #[test]
    fn tile_grid_covers_edges_and_rejects_unsupported_density() {
        let grid = GroundTileGrid::new(5).unwrap();
        assert_eq!(grid.metres_per_tile(), 64.0);
        assert_eq!(grid.covering(0.0, 0.0, 0.0, 0.0), (0, 0, 0, 0));
        assert_eq!(grid.covering(-0.1, -0.1, 64.0, 64.0), (-1, -1, 1, 1));
        assert!(GroundTileGrid::new(8).is_none());
        assert_eq!(GroundTileGrid::at_least(0.0).pixels_per_metre(), 0.125);
        assert_eq!(GroundTileGrid::at_least(0.139).pixels_per_metre(), 0.25);
        assert_eq!(GroundTileGrid::at_least(9.5).pixels_per_metre(), 16.0);
        assert_eq!(GroundTileGrid::at_least(64.0).pixels_per_metre(), 16.0);
        assert_eq!(GroundTileGrid::at_least(0.23).pixels_per_metre(), 0.25);
        assert_eq!(GroundTileGrid::at_least(2.0).pixels_per_metre(), 2.0);
    }

    #[test]
    fn empty_and_degenerate_ground_renders_transparent_without_panicking() {
        let duplicate = solid_triangle([[0., 0.], [1., 0.], [2., 0.]], [0.; 3], 0);
        let rasterizer = SourceGroundRasterizer::new(visual(
            vec![duplicate],
            vec![solid_texture([9, 9, 9, 255])],
        ));
        assert_eq!(rasterizer.triangle_count(), 0);
        let image = rasterizer.render(-5.0, -5.0, 4.0, 16);
        assert_eq!(image.len(), 16 * 16 * 4);
        assert!(image.iter().all(|&v| v == 0));
        assert!(rasterizer.render(f64::NAN, 0.0, 4.0, 16).is_empty());
    }
}
