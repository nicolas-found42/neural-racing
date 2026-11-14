//! Original static GT4 display geometry. This data never defines physical contact.
use crate::{Vec2, UNITS_PER_METRE};
use std::io::Read;

#[derive(Clone, Copy, Debug)]
pub struct SourceVisualVertex {
    pub position: Vec2,
    pub height_metres: f32,
    pub uv: [f32; 2],
    /// GS modulation colors: 128 is unity, rather than 255.
    pub native_rgba: [u8; 4],
}

#[derive(Clone, Copy, Debug)]
pub struct SourceVisualTriangle {
    pub indices: [u32; 3],
    pub texture: u32,
    pub source_model: u32,
    pub source_shape: u32,
    pub source_packet_offset: u32,
}

#[derive(Clone, Debug)]
pub struct SourceVisualTexture {
    pub width: u32,
    pub height: u32,
    pub gs_clamp: u64,
    pub gs_tex0: u64,
    pub rgba: Vec<u8>,
    pub source_tex1_offset: u32,
    pub source_texture_index: u32,
}

#[derive(Clone, Debug)]
pub struct SourceVisual {
    pub vertices: Vec<SourceVisualVertex>,
    pub triangles: Vec<SourceVisualTriangle>,
    pub textures: Vec<SourceVisualTexture>,
}

impl SourceVisual {
    pub fn decode_gzip(bytes: &[u8]) -> Result<Self, String> {
        let mut data = Vec::new();
        // Bounded decompression also catches truncated or corrupted embedded resources.
        flate2::read::GzDecoder::new(bytes)
            .take(256 * 1024 * 1024 + 1)
            .read_to_end(&mut data)
            .map_err(|e| format!("GT4 visual gzip: {e}"))?;
        if data.len() > 256 * 1024 * 1024 {
            return Err("GT4 visual exceeds resource limit".into());
        }
        Self::decode(&data)
    }

    fn decode(data: &[u8]) -> Result<Self, String> {
        if data.len() < 32 || &data[..4] != b"GTV1" {
            return Err("invalid GT4 visual header".into());
        }
        let u = |at: usize| u32::from_le_bytes(data[at..at + 4].try_into().unwrap());
        let nv = u(4) as usize;
        let nt = u(8) as usize;
        let nx = u(12) as usize;
        if [u(16), u(20), u(24), u(28)] != [24, 28, 40, 0]
            || nv > 4_000_000
            || nt > 2_000_000
            || nx > 4096
        {
            return Err("invalid GT4 visual counts/strides".into());
        }
        let triangle_start = 32 + nv * 24;
        let texture_start = triangle_start + nt * 28;
        let payload_start = texture_start + nx * 40;
        if payload_start > data.len() {
            return Err("truncated GT4 visual tables".into());
        }
        let mut vertices = Vec::with_capacity(nv);
        for record in data[32..triangle_start].chunks_exact(24) {
            let f = |at| f32::from_le_bytes(record[at..at + 4].try_into().unwrap());
            let values = [f(0), f(4), f(8), f(12), f(16)];
            if !values.iter().all(|v| v.is_finite()) {
                return Err("nonfinite GT4 visual vertex".into());
            }
            vertices.push(SourceVisualVertex {
                position: Vec2 {
                    x: f64::from(values[0]) * UNITS_PER_METRE,
                    y: f64::from(values[1]) * UNITS_PER_METRE,
                },
                height_metres: values[2],
                uv: [values[3], values[4]],
                native_rgba: record[20..24].try_into().unwrap(),
            });
        }
        let mut triangles = Vec::with_capacity(nt);
        for record in data[triangle_start..texture_start].chunks_exact(28) {
            let u = |at| u32::from_le_bytes(record[at..at + 4].try_into().unwrap());
            let indices = [u(0), u(4), u(8)];
            if indices.iter().any(|&i| i as usize >= nv) || u(12) as usize >= nx {
                return Err("GT4 visual references missing vertex/texture".into());
            }
            triangles.push(SourceVisualTriangle {
                indices,
                texture: u(12),
                source_model: u(16),
                source_shape: u(20),
                source_packet_offset: u(24),
            });
        }
        let mut textures = Vec::with_capacity(nx);
        for record in data[texture_start..payload_start].chunks_exact(40) {
            let u = |at| u32::from_le_bytes(record[at..at + 4].try_into().unwrap());
            let q = |at| u64::from_le_bytes(record[at..at + 8].try_into().unwrap());
            let (width, height, offset, len) = (u(0), u(4), u(24) as usize, u(28) as usize);
            if width == 0
                || height == 0
                || width > 4096
                || height > 4096
                || len != width as usize * height as usize * 4
                || offset < payload_start
                || offset.checked_add(len).is_none_or(|end| end > data.len())
            {
                return Err("invalid GT4 visual texture extent".into());
            }
            textures.push(SourceVisualTexture {
                width,
                height,
                gs_clamp: q(8),
                gs_tex0: q(16),
                rgba: data[offset..offset + len].to_vec(),
                source_tex1_offset: u(32),
                source_texture_index: u(36),
            });
        }
        Ok(Self {
            vertices,
            triangles,
            textures,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Vec<u8> {
        let mut b = b"GTV1".to_vec();
        for v in [3u32, 1, 1, 24, 28, 40, 0] {
            b.extend(v.to_le_bytes());
        }
        for p in [
            [1f32, 2., 3., 0., 0.],
            [4., 2., 3., 1., 0.],
            [1., 5., 3., 0., 1.],
        ] {
            for f in p {
                b.extend(f.to_le_bytes());
            }
            b.extend([128; 4]);
        }
        for v in [0u32, 1, 2, 0, 7, 8, 9] {
            b.extend(v.to_le_bytes());
        }
        for v in [1u32, 1, 0, 0, 0, 0, 172, 4, 123, 5] {
            b.extend(v.to_le_bytes());
        }
        b.extend([255, 100, 80, 255]);
        b
    }
    #[test]
    fn source_metres_uv_colors_and_packet_identity_survive_decode() {
        let v = SourceVisual::decode(&fixture()).unwrap();
        assert_eq!(v.vertices[0].position.x, UNITS_PER_METRE);
        assert_eq!(v.vertices[2].uv, [0., 1.]);
        assert_eq!(v.vertices[0].native_rgba, [128; 4]);
        assert_eq!(v.triangles[0].source_packet_offset, 9);
        assert_eq!(v.textures[0].rgba, [255, 100, 80, 255]);
    }
    #[test]
    fn malformed_visuals_fail_before_rendering() {
        let b = fixture();
        for n in [0, 31, 32, 100, 171, 175] {
            assert!(SourceVisual::decode(&b[..n]).is_err());
        }
        let mut bad = b.clone();
        bad[104..108].copy_from_slice(&9u32.to_le_bytes());
        assert!(SourceVisual::decode(&bad).is_err());
        let mut bad = b;
        bad[32..36].copy_from_slice(&f32::NAN.to_le_bytes());
        assert!(SourceVisual::decode(&bad).is_err());
    }
}
