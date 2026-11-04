//! Real circuit centerlines projected from geographic coordinates.
//! See docs/reference for source maps, photographs, dimensions and licensing.

use crate::{Track, Vec2};

/// World scale: 28 units represent the M4 GT3's 2.917 m wheelbase.
/// Vehicle integration converts these world units to metres internally.
pub const UNITS_PER_METRE: f64 = 28.0 / 2.917;
pub const CAR_LENGTH: f64 = 5.020 * UNITS_PER_METRE;
pub const CAR_WIDTH: f64 = 2.040 * UNITS_PER_METRE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Circuit {
    Monza,
    Silverstone,
    RedBullRing,
}

impl Circuit {
    pub const ALL: [Self; 3] = [Self::Monza, Self::Silverstone, Self::RedBullRing];

    pub fn name(self) -> &'static str {
        match self {
            Self::Monza => "Monza GP",
            Self::Silverstone => "Silverstone GP",
            Self::RedBullRing => "Red Bull Ring GP",
        }
    }

    pub fn slug(self) -> &'static str {
        match self {
            Self::Monza => "monza",
            Self::Silverstone => "silverstone",
            Self::RedBullRing => "red-bull-ring",
        }
    }

    pub fn length_metres(self) -> f64 {
        match self {
            Self::Monza => 5793.0,
            Self::Silverstone => 5891.0,
            // Current operator measurement; older maps label this 4318 m.
            Self::RedBullRing => 4326.0,
        }
    }

    /// Representative racing-surface widths, excluding kerbs/runoff.
    /// These are nominal widths, not a surveyed per-corner width profile.
    pub fn width_metres(self) -> f64 {
        match self {
            Self::Monza => 12.0,
            Self::Silverstone => 15.0,
            Self::RedBullRing => 12.5,
        }
    }

    pub fn coordinates(self) -> &'static [(f64, f64)] {
        match self {
            Self::Monza => include!("circuit_data/monza.rs"),
            Self::Silverstone => include!("circuit_data/silverstone.rs"),
            Self::RedBullRing => include!("circuit_data/red_bull_ring.rs"),
        }
    }

    pub fn build(self) -> Track {
        // Fixed maps are validated once. Each race owns independent mutable
        // vectors, just like the procedural-track API.
        static TRACKS: [std::sync::OnceLock<Track>; 3] = [const { std::sync::OnceLock::new() }; 3];
        TRACKS[self as usize]
            .get_or_init(|| self.build_uncached())
            .clone()
    }

    fn build_uncached(self) -> Track {
        let coordinates = self.coordinates();
        let (lon, lat) = coordinates[0];
        let metres_per_degree = std::f64::consts::PI * 6_371_000.0 / 180.0;
        let mut points: Vec<Vec2> = coordinates
            .iter()
            .map(|&(x, y)| Vec2 {
                x: (x - lon) * metres_per_degree * lat.to_radians().cos(),
                y: -(y - lat) * metres_per_degree,
            })
            .collect();
        if (points[0] - points[points.len() - 1]).norm() < 0.01 {
            points.pop();
        }
        // Densify before rounding: smoothing then moves a corner by < 2 m,
        // rather than cutting across the ends of hundred-metre straights.
        let mut dense = Vec::new();
        for i in 0..points.len() {
            let a = points[i];
            let b = points[(i + 1) % points.len()];
            let count = ((b - a).norm() / 5.0).ceil().max(1.0) as usize;
            for j in 0..count {
                dense.push(a + (b - a) * (j as f64 / count as f64));
            }
        }
        // A bounded local filter removes sharp joins in the map polyline.
        for _ in 0..2 {
            dense = (0..dense.len())
                .map(|i| {
                    dense[(i + dense.len() - 1) % dense.len()] * 0.25
                        + dense[i] * 0.5
                        + dense[(i + 1) % dense.len()] * 0.25
                })
                .collect();
        }
        // Silverstone source starts at the old pits. Use the GP start on
        // Hamilton Straight, nearest the geographic finish-line location.
        if self == Self::Silverstone {
            let finish = Vec2 {
                x: (-1.0217 - lon) * metres_per_degree * lat.to_radians().cos(),
                y: -(52.0678 - lat) * metres_per_degree,
            };
            let start = dense
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| (**a - finish).norm().total_cmp(&(**b - finish).norm()))
                .unwrap()
                .0;
            dense.rotate_left(start);
        }
        let length: f64 = (0..dense.len())
            .map(|i| (dense[(i + 1) % dense.len()] - dense[i]).norm())
            .sum();
        let scale = self.length_metres() * UNITS_PER_METRE / length;
        for point in &mut dense {
            *point = *point * scale;
        }
        let widths = vec![self.width_metres() * UNITS_PER_METRE * 0.5; dense.len()];
        Track::from_circuit_centerline(dense, widths, self)
            .expect("embedded circuit must have valid closed road geometry")
    }
}
