//! Real circuit centerlines projected from geographic coordinates.
//! See docs/reference for source maps, photographs, dimensions and licensing.

use crate::{Track, Vec2};

/// Retained world scale: 28 units per 2.917 metres. Vehicle dimensions use metres.
/// Vehicle integration converts these world units to metres internally.
pub const UNITS_PER_METRE: f64 = 28.0 / 2.917;
/// Recovered selected GT4 M3 GTR model envelope, not the former M4 placeholder.
pub const CAR_LENGTH: f64 = 4.619 * UNITS_PER_METRE;
pub const CAR_WIDTH: f64 = 1.879 * UNITS_PER_METRE;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Circuit {
    Monza,
    Silverstone,
    RedBullRing,
    /// Index into the source-derived GT4 resource catalog. Persist its slug, not this index.
    Gt4(u16),
}

impl Circuit {
    pub const ALL: [Self; 3] = [Self::Monza, Self::Silverstone, Self::RedBullRing];

    /// Legacy maps remain available; GT4 entries include only validated drivable resources.
    pub fn all() -> &'static [Self] {
        static ALL: std::sync::OnceLock<Vec<Circuit>> = std::sync::OnceLock::new();
        ALL.get_or_init(|| {
            let mut courses: Vec<_> = Self::ALL
                .into_iter()
                .filter(|legacy| {
                    !crate::course_catalog::courses().iter().any(|c| {
                        c.available
                            && c.aliases
                                .iter()
                                .any(|a| a.eq_ignore_ascii_case(legacy.slug()))
                    })
                })
                .collect();
            courses.extend(
                crate::course_catalog::courses()
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| c.available)
                    .map(|(i, _)| Self::Gt4(u16::try_from(i).expect("GT4 catalog fits u16"))),
            );
            courses
        })
    }

    pub fn lookup(value: &str) -> Option<Self> {
        // Exact aliases only. A GT4 source explicitly declaring an overlap takes precedence.
        crate::course_catalog::courses()
            .iter()
            .enumerate()
            .find(|(_, c)| {
                c.available
                    && (c.slug.eq_ignore_ascii_case(value)
                        || c.id.eq_ignore_ascii_case(value)
                        || c.name.eq_ignore_ascii_case(value)
                        || c.aliases.iter().any(|a| a.eq_ignore_ascii_case(value)))
            })
            .map(|(i, _)| Self::Gt4(i as u16))
            .or_else(|| {
                Self::ALL.into_iter().find(|c| {
                    c.slug().eq_ignore_ascii_case(value) || c.name().eq_ignore_ascii_case(value)
                })
            })
    }

    pub fn is_practice(self) -> bool {
        matches!(self, Self::Gt4(i) if crate::course_catalog::courses().get(usize::from(i))
            .is_some_and(|c| c.topology == crate::course_catalog::CourseTopology::Area))
    }

    pub fn stats_index(self) -> usize {
        match self {
            Self::Monza => 0,
            Self::Silverstone => 1,
            Self::RedBullRing => 2,
            Self::Gt4(i) => 3 + usize::from(i),
        }
    }

    pub fn stats_count() -> usize {
        4 + crate::course_catalog::courses().len()
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Monza => "Monza GP",
            Self::Silverstone => "Silverstone GP",
            Self::RedBullRing => "Red Bull Ring GP",
            Self::Gt4(i) => crate::course_catalog::courses()[usize::from(i)].name,
        }
    }

    pub fn slug(self) -> &'static str {
        match self {
            Self::Monza => "monza",
            Self::Silverstone => "silverstone",
            Self::RedBullRing => "red-bull-ring",
            Self::Gt4(i) => crate::course_catalog::courses()[usize::from(i)].slug,
        }
    }

    pub fn length_metres(self) -> f64 {
        match self {
            Self::Monza => 5793.0,
            Self::Silverstone => 5891.0,
            // Current operator measurement; older maps label this 4318 m.
            Self::RedBullRing => 4326.0,
            Self::Gt4(_) => self.build().lap_length / UNITS_PER_METRE,
        }
    }

    /// Representative racing-surface widths, excluding kerbs/runoff.
    /// These are nominal widths, not a surveyed per-corner width profile.
    pub fn width_metres(self) -> f64 {
        match self {
            Self::Monza => 12.0,
            Self::Silverstone => 15.0,
            Self::RedBullRing => 12.5,
            Self::Gt4(_) => {
                let track = self.build();
                track.half_width.iter().sum::<f64>() * 2.0
                    / track.half_width.len() as f64
                    / UNITS_PER_METRE
            }
        }
    }

    pub fn coordinates(self) -> &'static [(f64, f64)] {
        match self {
            Self::Monza => include!("circuit_data/monza.rs"),
            Self::Silverstone => include!("circuit_data/silverstone.rs"),
            Self::RedBullRing => include!("circuit_data/red_bull_ring.rs"),
            Self::Gt4(_) => panic!("GT4 courses use source-local metres; call build instead"),
        }
    }

    /// Fallible loading boundary for source resources and external course selection.
    pub fn try_build(self) -> Result<Track, String> {
        if let Self::Gt4(i) = self {
            let course = crate::course_catalog::courses()
                .get(usize::from(i))
                .ok_or_else(|| format!("unknown GT4 course index {i}"))?;
            if !course.available {
                return Err(format!(
                    "GT4 resource {} is not a validated driving course",
                    course.id
                ));
            }
            return crate::gt4_course::build(i);
        }
        Ok(self.build())
    }

    pub fn build(self) -> Track {
        if matches!(self, Self::Gt4(_)) {
            return self.try_build().expect("validated available GT4 course");
        }
        // Fixed maps are validated once. Each race owns independent mutable
        // vectors, just like the procedural-track API.
        static TRACKS: [std::sync::OnceLock<Track>; 3] = [const { std::sync::OnceLock::new() }; 3];
        TRACKS[self.stats_index()]
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
