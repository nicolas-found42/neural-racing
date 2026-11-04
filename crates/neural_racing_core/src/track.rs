//! Closed road geometry shared by real circuits and seeded procedural courses.

use std::collections::HashMap;
use std::f64::consts::TAU;
use std::sync::Arc;

use crate::rng::Rng;
use crate::{
    py_mod, Vec2, ARENA_CENTER, ARENA_HEIGHT, ARENA_WIDTH, BASE_HALF_WIDTH, CHECKPOINT_COUNT,
    RAY_RANGE,
};

/// Centerline samples per lap.
pub const SAMPLES: usize = 320;
/// A corner keeps at least this many half-widths of radius; anything tighter
/// folds the inner wall through itself.
const MIN_CURVATURE_MARGIN: f64 = 1.70;
/// Tightest centerline radius the car can actually follow: its turning circle is
/// `WHEELBASE / tan(MAX_STEER_ANGLE)` (≈ 46 units), and a hairpin under that is
/// impossible at any speed. 74 leaves the car room to take a line at speed.
const MIN_TURN_RADIUS: f64 = 74.0;
/// Distant sections of the lap keep this many half-widths between their walls.
const MIN_CLEARANCE_MARGIN: f64 = 0.40;
/// Sections closer than this fraction of the lap count as neighbours.
const NEIGHBOUR_ARC_FRACTION: f64 = 0.22;
const MIN_LAP_LENGTH: f64 = 700.0;
const MAX_LAP_LENGTH: f64 = 1700.0;
const ARENA_MARGIN: f64 = 12.0;
/// Width variation stays inside these multiples of the nominal half width.
pub const MIN_WIDTH_FACTOR: f64 = 0.66;
pub const MAX_WIDTH_FACTOR: f64 = 1.45;

#[derive(Clone, Copy, Debug)]
pub struct Gate {
    pub center: Vec2,
    pub tangent: Vec2,
    /// Half the road width where the gate sits.
    pub half_width: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct CenterlineHit {
    pub segment: usize,
    pub lateral: f64,
    pub arc: f64,
    /// Half the road width at `segment`, interpolated along the segment.
    pub half_width: f64,
    pub tangent: Vec2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min_x: f64,
    pub min_y: f64,
    pub max_x: f64,
    pub max_y: f64,
}

#[derive(Clone)]
pub struct Track {
    pub circuit: Option<crate::Circuit>,
    pub centerline: Vec<Vec2>,
    /// Outer and inner wall polylines, indexed like the centerline.
    pub outer: Vec<Vec2>,
    pub inner: Vec<Vec2>,
    pub lap_length: f64,
    /// Half the road width at every centerline sample.
    pub half_width: Vec<f64>,
    pub start: Vec2,
    pub start_heading: f64,
    pub bounds: Bounds,
    center_vec: Vec<Vec2>,
    center_len: Vec<f64>,
    center_len_sq: Vec<f64>,
    center_tangent: Vec<Vec2>,
    center_cum: Vec<f64>,
    seg_a: Vec<Vec2>,
    seg_vec: Vec<Vec2>,
    gates: Vec<Gate>,
    wall_grid: Arc<SegmentGrid>,
    center_grid: Arc<SegmentGrid>,
    corner_table: Arc<CornerTable>,
    /// Braking-distance-aware speed limit sampled along the lap. Built once;
    /// cloned by reference because every car carries a track.
    speed_profile: Arc<Vec<f64>>,
    profile_spacing: f64,
}

/// A broad phase only: exact segment intersection/projection still decides
/// every sensor hit. Store segments in all cells touched by their bounding box.
#[derive(Clone)]
struct SegmentGrid {
    cells: HashMap<(i32, i32), Vec<usize>>,
}
impl SegmentGrid {
    fn cell(v: f64) -> i32 {
        (v / RAY_RANGE).floor() as i32
    }
    fn new(starts: &[Vec2], edges: &[Vec2]) -> Self {
        let mut cells = HashMap::<_, Vec<_>>::new();
        for (i, (&a, &edge)) in starts.iter().zip(edges).enumerate() {
            let b = a + edge;
            for x in Self::cell(a.x.min(b.x))..=Self::cell(a.x.max(b.x)) {
                for y in Self::cell(a.y.min(b.y))..=Self::cell(a.y.max(b.y)) {
                    cells.entry((x, y)).or_default().push(i);
                }
            }
        }
        Self { cells }
    }
    fn near(&self, point: Vec2) -> Vec<usize> {
        let mut result = Vec::new();
        for x in Self::cell(point.x - RAY_RANGE)..=Self::cell(point.x + RAY_RANGE) {
            for y in Self::cell(point.y - RAY_RANGE)..=Self::cell(point.y + RAY_RANGE) {
                if let Some(indices) = self.cells.get(&(x, y)) {
                    result.extend_from_slice(indices);
                }
            }
        }
        // Preserve exhaustive traversal's tie-breaking at shared vertices.
        result.sort_unstable();
        result.dedup();
        result
    }
}

/// Sample curvature once, then answer braking-preview range minima in O(1).
/// Two-metre spacing and a three-metre stencil resolve the mapped chicanes.
#[derive(Default)]
struct CornerTable {
    spacing: f64,
    levels: Vec<Vec<f64>>,
}
impl CornerTable {
    fn build(track: &Track) -> Self {
        let count = (track.lap_length / (2.0 * crate::UNITS_PER_METRE)).ceil() as usize;
        let spacing = track.lap_length / count as f64;
        let stencil = 3.0 * crate::UNITS_PER_METRE;
        let radii: Vec<_> = (0..count)
            .map(|i| {
                let arc = i as f64 * spacing;
                let a = track.arc_point(arc - stencil);
                let b = track.arc_point(arc);
                let c = track.arc_point(arc + stencil);
                let area2 = ((b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x)).abs();
                if area2 < 1e-9 {
                    f64::INFINITY
                } else {
                    (b - a).norm() * (c - b).norm() * (a - c).norm() / (2.0 * area2)
                }
            })
            .collect();
        let mut levels = vec![radii];
        let mut width = 2;
        while width <= count {
            let previous = levels.last().unwrap();
            let half = width / 2;
            levels.push(
                (0..=count - width)
                    .map(|i| previous[i].min(previous[i + half]))
                    .collect(),
            );
            width *= 2;
        }
        Self { spacing, levels }
    }
    fn range(&self, start: usize, end: usize) -> f64 {
        if start >= end {
            return f64::INFINITY;
        }
        let power = (end - start).ilog2() as usize;
        self.levels[power][start].min(self.levels[power][end - (1 << power)])
    }
    fn radius_at(&self, arc: f64, lap: f64) -> f64 {
        let count = self.levels[0].len();
        let index = (py_mod(arc, lap) / self.spacing).floor() as usize;
        self.levels[0][index.min(count - 1)]
    }
    fn ahead(&self, arc: f64, horizon: f64, lap: f64) -> f64 {
        if horizon <= 0.0 {
            return f64::INFINITY;
        }
        let count = self.levels[0].len();
        if horizon >= lap {
            return self.range(0, count);
        }
        let start_arc = py_mod(arc, lap);
        let start = (start_arc / self.spacing).floor() as usize;
        let end = ((start_arc + horizon) / self.spacing).ceil() as usize + 1;
        if end <= count {
            self.range(start, end)
        } else {
            self.range(start, count)
                .min(self.range(0, (end - count).min(count)))
        }
    }
}

impl Track {
    /// One lap of seeded random geometry. The same seed always rebuilds the
    /// same track; the generator retries until the loop is drivable.
    pub fn random(seed: u64) -> Self {
        Self::draw(seed, 22.0..32.0)
    }

    /// A track drawn from the narrow end of the width distribution. Training
    /// mixes these in so the tail of the distribution is not just tested.
    pub fn random_narrow(seed: u64) -> Self {
        Self::draw(seed, 20.5..24.5)
    }

    /// A narrow track with sharper, more frequent corners: the tail of the
    /// distribution the evaluation set only occasionally hits.
    pub fn random_hard(seed: u64) -> Self {
        Self::draw_hard(seed, 20.5..23.5)
    }

    fn draw(seed: u64, nominal: std::ops::Range<f64>) -> Self {
        Self::draw_with(seed, nominal, 116.0..198.0, 0.10..0.30)
    }

    fn draw_hard(seed: u64, nominal: std::ops::Range<f64>) -> Self {
        Self::draw_with(seed, nominal, 104.0..146.0, 0.20..0.34)
    }

    fn draw_with(
        seed: u64,
        nominal: std::ops::Range<f64>,
        base_radius: std::ops::Range<f64>,
        shape_budget: std::ops::Range<f64>,
    ) -> Self {
        let mut rng = Rng::new(seed ^ 0x5ca1_ab1e_d00d_5eed);
        for _ in 0..512 {
            if let Some(track) = Self::candidate(
                &mut rng,
                nominal.clone(),
                base_radius.clone(),
                shape_budget.clone(),
            ) {
                return track;
            }
        }
        // A plain oval always satisfies the constraints; keep the app runnable
        // even if an extreme RNG drawing never fits.
        neural_racing_diagnostics::emit!(
            "track_generation_fallback",
            neural_racing_diagnostics::current_operation(),
            neural_racing_diagnostics::json!({"seed":seed,"rejected_candidates":512})
        );
        Self::oval()
    }

    /// Deterministic fallback: a narrow unvarying oval centred in the arena.
    pub fn oval() -> Self {
        let mut points = Vec::with_capacity(SAMPLES);
        for i in 0..SAMPLES {
            let angle = i as f64 / SAMPLES as f64 * TAU;
            points.push(Vec2 {
                x: ARENA_CENTER.x + 212.0 * angle.cos(),
                y: ARENA_CENTER.y + 170.0 * angle.sin(),
            });
        }
        Self::from_centerline(points).expect("oval geometry is valid")
    }

    /// Star-shaped random loop: the radius is a few harmonics of the polar
    /// angle, so the centerline never crosses itself, while the road width
    /// breathes between pinch points and wide sweepers.
    fn candidate(
        rng: &mut Rng,
        nominal_width: std::ops::Range<f64>,
        base_radius_range: std::ops::Range<f64>,
        shape_budget_range: std::ops::Range<f64>,
    ) -> Option<Self> {
        let base_radius = rng.range(base_radius_range.start, base_radius_range.end);
        let shape_budget = rng.range(shape_budget_range.start, shape_budget_range.end);
        let harmonics = pick_harmonics(rng, [2, 3, 4, 5, 6, 7], shape_budget, 2);
        let aspect_x = rng.range(0.84, 1.16);
        let aspect_y = rng.range(0.72, 1.00);
        let tilt = rng.range(0.0, TAU);
        let center = Vec2 {
            x: ARENA_CENTER.x + rng.range(-26.0, 26.0),
            y: ARENA_CENTER.y + rng.range(-22.0, 22.0),
        };
        let nominal = rng.range(nominal_width.start, nominal_width.end);
        let width_budget = rng.range(0.14, 0.32);
        let widths = pick_harmonics(rng, [2, 3, 4, 5], width_budget, 1);

        let mut points = Vec::with_capacity(SAMPLES);
        let mut half_width = Vec::with_capacity(SAMPLES);
        for i in 0..SAMPLES {
            let angle = i as f64 / SAMPLES as f64 * TAU;
            let mut radius = 1.0;
            for &(order, amplitude, phase) in &harmonics {
                radius += amplitude * (order * angle + phase).cos();
            }
            let mut width = 1.0;
            for &(order, amplitude, phase) in &widths {
                width += amplitude * (order * angle + phase).cos();
            }
            let local = Vec2 {
                x: base_radius * radius * angle.cos() * aspect_x,
                y: base_radius * radius * angle.sin() * aspect_y,
            }
            .rotated(tilt);
            points.push(center + local);
            half_width.push((nominal * width).clamp(
                MIN_WIDTH_FACTOR * BASE_HALF_WIDTH,
                MAX_WIDTH_FACTOR * BASE_HALF_WIDTH,
            ));
        }
        if rng.next_u32() & 1 == 1 {
            points.reverse();
            half_width.reverse();
        }
        Self::from_centerline_with_widths(points, half_width)
    }

    /// Builds a track from an ordered closed polyline of constant width.
    pub fn from_centerline(centerline: Vec<Vec2>) -> Option<Self> {
        let half_width = vec![BASE_HALF_WIDTH; centerline.len()];
        Self::from_centerline_with_widths(centerline, half_width)
    }

    /// Builds a track from an ordered closed polyline and the half width at
    /// every sample, rejecting loops whose offsets would overlap. The
    /// start/finish is moved to the straightest part so every race begins with
    /// a stable launch.
    pub fn from_centerline_with_widths(
        centerline: Vec<Vec2>,
        half_width: Vec<f64>,
    ) -> Option<Self> {
        Self::build(centerline, half_width, None)
    }

    pub(crate) fn from_circuit_centerline(
        centerline: Vec<Vec2>,
        half_width: Vec<f64>,
        circuit: crate::Circuit,
    ) -> Option<Self> {
        Self::build(centerline, half_width, Some(circuit))
    }

    fn build(
        mut centerline: Vec<Vec2>,
        mut half_width: Vec<f64>,
        circuit: Option<crate::Circuit>,
    ) -> Option<Self> {
        assert_eq!(
            centerline.len(),
            half_width.len(),
            "one half width per centerline sample"
        );
        let start_index = if circuit.is_some() {
            0
        } else {
            flattest_index(&centerline, 14)
        };
        centerline.rotate_left(start_index);
        half_width.rotate_left(start_index);
        let n = centerline.len();
        assert!(n >= 8, "a track needs at least 8 centerline samples");
        let center_vec = closed_edge_vecs(&centerline);
        let mut center_len = Vec::with_capacity(n);
        let mut center_len_sq = Vec::with_capacity(n);
        let mut center_tangent = Vec::with_capacity(n);
        for diff in &center_vec {
            let len = diff.norm();
            if len < 1e-6 {
                return None;
            }
            center_len.push(len);
            center_len_sq.push((len * len).max(1e-9));
            center_tangent.push(Vec2 {
                x: diff.x / len,
                y: diff.y / len,
            });
        }
        let mut center_cum = Vec::with_capacity(n);
        let mut acc = 0.0;
        for len in &center_len {
            center_cum.push(acc);
            acc += *len;
        }
        if circuit.is_none() && !(MIN_LAP_LENGTH..=MAX_LAP_LENGTH).contains(&acc) {
            return None;
        }
        // Every corner must be wide enough for the road running through it, and
        // never tighter than the car can steer.
        let radii = curvature_radii(&centerline);
        for (index, radius) in radii.iter().enumerate() {
            let needed = (MIN_CURVATURE_MARGIN * half_width[index]).max(MIN_TURN_RADIUS);
            if circuit.is_none() && *radius < needed {
                return None;
            }
        }
        if circuit.is_none() && !clear_of_itself(&centerline, &half_width, &center_cum, acc) {
            return None;
        }

        let outer = offset_polyline(&centerline, &half_width, 1.0);
        let inner = offset_polyline(&centerline, &half_width, -1.0);
        if !polyline_is_simple(&outer) || !polyline_is_simple(&inner) {
            return None;
        }
        let bounds = bounds_of([&centerline, &outer, &inner]);
        if circuit.is_none()
            && (bounds.min_x < ARENA_MARGIN
                || bounds.min_y < ARENA_MARGIN
                || bounds.max_x > ARENA_WIDTH - ARENA_MARGIN
                || bounds.max_y > ARENA_HEIGHT - ARENA_MARGIN)
        {
            return None;
        }

        let mut seg_a = outer.clone();
        seg_a.extend_from_slice(&inner);
        let mut seg_vec = closed_edge_vecs(&outer);
        seg_vec.extend(closed_edge_vecs(&inner));

        let wall_grid = Arc::new(SegmentGrid::new(&seg_a, &seg_vec));
        let center_grid = Arc::new(SegmentGrid::new(&centerline, &center_vec));
        let start = centerline[0];
        let start_heading = center_tangent[0].angle();
        let mut track = Self {
            circuit,
            centerline,
            outer,
            inner,
            lap_length: acc,
            half_width,
            start,
            start_heading,
            bounds,
            center_vec,
            center_len,
            center_len_sq,
            center_tangent,
            center_cum,
            seg_a,
            seg_vec,
            gates: Vec::new(),
            wall_grid,
            center_grid,
            corner_table: Arc::new(CornerTable::default()),
            speed_profile: Arc::new(Vec::new()),
            profile_spacing: 1.0,
        };
        track.gates = track.make_gates();
        track.corner_table = Arc::new(CornerTable::build(&track));
        track.build_speed_profile();
        Some(track)
    }

    /// Builds the cyclic velocity profile by clamping every sample to its
    /// curvature limit, then propagating the braking limit backwards twice
    /// around the loop. `v[i] <= sqrt(v[i+1]^2 + 2 a_b ds)`: the car can always
    /// shed speed in time for the next sample, and the profile never asks for
    /// more than the tyre can hold.
    fn build_speed_profile(&mut self) {
        let count = (self.lap_length / (2.0 * crate::UNITS_PER_METRE)).ceil() as usize;
        let count = count.max(2);
        let spacing = self.lap_length / count as f64;
        let brake = crate::BRAKING_ACCEL * crate::UNITS_PER_METRE;
        let floor = crate::PROFILE_MIN_SPEED * crate::UNITS_PER_METRE;
        let mut profile = Vec::with_capacity(count);
        for index in 0..count {
            let arc = index as f64 * spacing;
            let radius = self.corner_table.radius_at(arc, self.lap_length).max(1.0);
            let corner = (crate::MAX_LATERAL_ACCEL * radius).sqrt();
            profile.push(corner.clamp(floor, crate::MAX_SPEED));
        }
        for _ in 0..2 {
            for index in (0..count).rev() {
                let next = profile[(index + 1) % count];
                let allowed = (next * next + 2.0 * brake * spacing).sqrt();
                profile[index] = profile[index].min(allowed).max(floor);
            }
        }
        self.speed_profile = Arc::new(profile);
        self.profile_spacing = spacing;
    }

    /// Speed the car may travel at `arc` and still brake for every corner
    /// reachable from here. Units per second.
    pub fn speed_profile(&self, arc: f64) -> f64 {
        let profile = &self.speed_profile;
        if profile.is_empty() {
            return crate::MAX_SPEED;
        }
        let index = (py_mod(arc, self.lap_length) / self.profile_spacing) as usize;
        profile[index.min(profile.len() - 1)]
    }

    pub fn gates(&self) -> &[Gate] {
        &self.gates
    }

    /// Narrowest and widest half width anywhere on the lap.
    pub fn width_range(&self) -> (f64, f64) {
        self.half_width
            .iter()
            .copied()
            .fold((f64::INFINITY, 0.0_f64), |(lo, hi), w| {
                (lo.min(w), hi.max(w))
            })
    }

    /// Point on the centerline at the given arc length.
    pub fn arc_point(&self, arc: f64) -> Vec2 {
        let arc = py_mod(arc, self.lap_length);
        let segment = segment_index(&self.center_cum, arc);
        let fraction = (arc - self.center_cum[segment]) / self.center_len[segment];
        self.centerline[segment] + self.center_vec[segment] * fraction
    }

    /// Half the road width at an arc length.
    pub fn arc_half_width(&self, arc: f64) -> f64 {
        let arc = py_mod(arc, self.lap_length);
        let segment = segment_index(&self.center_cum, arc);
        let fraction = (arc - self.center_cum[segment]) / self.center_len[segment];
        let width = self.half_width[segment];
        let next = self.half_width[(segment + 1) % self.half_width.len()];
        width + (next - width) * fraction
    }

    /// Tightest corner radius over the next `horizon` units of road, or
    /// infinity when the road stays straight that far ahead.
    pub fn tightest_corner_ahead(&self, arc: f64, horizon: f64) -> f64 {
        self.corner_table.ahead(arc, horizon, self.lap_length)
    }

    /// Tangent (unit vector in the driving direction) at an arc length.
    pub fn arc_tangent(&self, arc: f64) -> Vec2 {
        self.center_tangent[segment_index(&self.center_cum, py_mod(arc, self.lap_length))]
    }

    pub fn nearest_on_centerline(&self, point: Vec2) -> CenterlineHit {
        let candidates = self.center_grid.near(point);
        let hit = self.nearest_in(point, candidates.into_iter());
        // Any segment closer than RAY_RANGE must intersect the queried square.
        // Outside that guaranteed radius use the exhaustive path (e.g. a
        // caller probing a point far outside the circuit).
        if let Some((hit, distance)) = hit {
            if distance <= RAY_RANGE * RAY_RANGE {
                return hit;
            }
        }
        self.nearest_in(point, 0..self.centerline.len()).unwrap().0
    }

    fn nearest_in(
        &self,
        point: Vec2,
        indices: impl Iterator<Item = usize>,
    ) -> Option<(CenterlineHit, f64)> {
        let mut best_dist = f64::INFINITY;
        let mut best_index = 0;
        let mut best_t = 0.0;
        let mut best_dx = 0.0;
        let mut best_dy = 0.0;
        for index in indices {
            let start = self.centerline[index];
            let along = self.center_vec[index];
            let apx = point.x - start.x;
            let apy = point.y - start.y;
            let fraction =
                ((apx * along.x + apy * along.y) / self.center_len_sq[index]).clamp(0.0, 1.0);
            let dx = point.x - (start.x + fraction * along.x);
            let dy = point.y - (start.y + fraction * along.y);
            let distance = dx * dx + dy * dy;
            if distance < best_dist {
                best_dist = distance;
                best_index = index;
                best_t = fraction;
                best_dx = dx;
                best_dy = dy;
            }
        }
        let tangent = self.center_tangent[best_index];
        let width_here = self.half_width[best_index];
        let width_next = self.half_width[(best_index + 1) % self.half_width.len()];
        if !best_dist.is_finite() {
            return None;
        }
        Some((
            CenterlineHit {
                segment: best_index,
                lateral: best_dx * -tangent.y + best_dy * tangent.x,
                arc: self.center_cum[best_index] + best_t * self.center_len[best_index],
                half_width: width_here + (width_next - width_here) * best_t,
                tangent,
            },
            best_dist,
        ))
    }

    pub fn checkpoint_crossing(&self, index: usize, previous: Vec2, current: Vec2) -> i32 {
        let gate = self.gates[index];
        let before = (previous - gate.center).dot(gate.tangent);
        let after = (current - gate.center).dot(gate.tangent);
        let direction = if before <= 0.0 && after > 0.0 {
            1
        } else if after < 0.0 && before >= 0.0 {
            -1
        } else {
            return 0;
        };
        let fraction = before / (before - after);
        let intersection = previous + (current - previous) * fraction;
        let normal = Vec2 {
            x: -gate.tangent.y,
            y: gate.tangent.x,
        };
        if (intersection - gate.center).dot(normal).abs() <= gate.half_width {
            direction
        } else {
            0
        }
    }

    /// Casts one ray per angle into `out`, which must match `angles` in length.
    pub fn cast_rays_into(&self, origin: Vec2, angles: &[f64], out: &mut [f64]) {
        assert_eq!(angles.len(), out.len(), "one output distance per ray angle");
        let candidates = self.wall_grid.near(origin);
        self.cast_candidates(origin, angles, out, &candidates);
    }

    fn cast_candidates(&self, origin: Vec2, angles: &[f64], out: &mut [f64], candidates: &[usize]) {
        out.fill(RAY_RANGE);
        for (ray, angle) in angles.iter().enumerate() {
            let dx = angle.cos();
            let dy = angle.sin();
            for &segment in candidates {
                let start = self.seg_a[segment];
                let edge = self.seg_vec[segment];
                let denom = dx * edge.y - dy * edge.x;
                if denom.abs() <= 1e-9 {
                    continue;
                }
                let ax = start.x - origin.x;
                let ay = start.y - origin.y;
                let t = (ax * edge.y - ay * edge.x) / denom;
                let u = (ax * dy - ay * dx) / denom;
                if t >= 0.0 && (0.0..=1.0).contains(&u) && t < out[ray] {
                    out[ray] = t;
                }
            }
        }
    }

    fn make_gates(&self) -> Vec<Gate> {
        let mut gates = Vec::with_capacity(CHECKPOINT_COUNT);
        for index in 1..=CHECKPOINT_COUNT {
            let arc = py_mod(
                index as f64 * self.lap_length / CHECKPOINT_COUNT as f64,
                self.lap_length,
            );
            let segment = segment_index(&self.center_cum, arc);
            let fraction = (arc - self.center_cum[segment]) / self.center_len[segment];
            gates.push(Gate {
                center: self.centerline[segment] + self.center_vec[segment] * fraction,
                tangent: self.center_tangent[segment],
                half_width: self.half_width[segment],
            });
        }
        gates
    }
}

/// Index of the centerline point with the least heading change around it.
fn flattest_index(poly: &[Vec2], window: usize) -> usize {
    let n = poly.len();
    let mut best_index = 0;
    let mut best_turn = f64::INFINITY;
    for index in 0..n {
        let mut turn = 0.0;
        for offset in 1..=window {
            let previous = poly[(index + n + offset - 1) % n];
            let current = poly[(index + offset) % n];
            let next = poly[(index + offset + 1) % n];
            let first = (current - previous).angle();
            let second = (next - current).angle();
            turn += crate::wrap_angle(second - first).abs();
        }
        if turn < best_turn {
            best_turn = turn;
            best_index = index;
        }
    }
    best_index
}

fn offset_polyline(poly: &[Vec2], half_width: &[f64], side: f64) -> Vec<Vec2> {
    let n = poly.len();
    let mut offset = Vec::with_capacity(n);
    for i in 0..n {
        let dist = half_width[i] * side;
        let next = poly[(i + 1) % n];
        let prev = poly[(i + n - 1) % n];
        let tangent = next - prev;
        let len = tangent.norm();
        let tangent = Vec2 {
            x: tangent.x / len,
            y: tangent.y / len,
        };
        let normal = Vec2 {
            x: -tangent.y,
            y: tangent.x,
        };
        offset.push(poly[i] + normal * dist);
    }
    offset
}

fn closed_edge_vecs(poly: &[Vec2]) -> Vec<Vec2> {
    let n = poly.len();
    (0..n).map(|i| poly[(i + 1) % n] - poly[i]).collect()
}

fn bounds_of<const N: usize>(polys: [&Vec<Vec2>; N]) -> Bounds {
    let mut bounds = Bounds {
        min_x: f64::INFINITY,
        min_y: f64::INFINITY,
        max_x: f64::NEG_INFINITY,
        max_y: f64::NEG_INFINITY,
    };
    for poly in polys {
        for point in poly {
            bounds.min_x = bounds.min_x.min(point.x);
            bounds.min_y = bounds.min_y.min(point.y);
            bounds.max_x = bounds.max_x.max(point.x);
            bounds.max_y = bounds.max_y.max(point.y);
        }
    }
    bounds
}

/// Circumradius of every three consecutive samples; infinite means straight.
fn curvature_radii(poly: &[Vec2]) -> Vec<f64> {
    let n = poly.len();
    let mut radii = Vec::with_capacity(n);
    for i in 0..n {
        let a = poly[(i + n - 1) % n];
        let b = poly[i];
        let c = poly[(i + 1) % n];
        let ab = (b - a).norm();
        let bc = (c - b).norm();
        let ca = (a - c).norm();
        let cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
        let area2 = cross.abs();
        radii.push(if area2 < 1e-9 {
            f64::INFINITY
        } else {
            ab * bc * ca / (2.0 * area2)
        });
    }
    radii
}

/// Rejects loops whose separate sections would leave their walls touching.
fn clear_of_itself(poly: &[Vec2], half_width: &[f64], cum: &[f64], lap_length: f64) -> bool {
    let n = poly.len();
    let neighbours = NEIGHBOUR_ARC_FRACTION * lap_length;
    for i in 0..n {
        for j in (i + 1)..n {
            let arc_gap = (cum[j] - cum[i]).abs();
            if arc_gap < neighbours || lap_length - arc_gap < neighbours {
                continue;
            }
            let gap = (poly[i] - poly[j]).norm() - half_width[i] - half_width[j];
            if gap < MIN_CLEARANCE_MARGIN * BASE_HALF_WIDTH {
                return false;
            }
        }
    }
    true
}

/// Picks `min_terms..=min_terms + 2` distinct harmonic orders, splits `budget`
/// between them, and gives each a random phase.
fn pick_harmonics<const N: usize>(
    rng: &mut Rng,
    orders: [u32; N],
    budget: f64,
    min_terms: usize,
) -> Vec<(f64, f64, f64)> {
    let count = min_terms + (rng.next_u32() % 3) as usize;
    let mut used = [false; 8];
    let mut shares = Vec::with_capacity(count);
    let mut total = 0.0;
    while shares.len() < count {
        let order = orders[(rng.next_u32() % orders.len() as u32) as usize];
        if used[order as usize] {
            continue;
        }
        used[order as usize] = true;
        let share = rng.range(0.35, 1.0);
        total += share;
        shares.push((order as f64, share, rng.range(0.0, TAU)));
    }
    shares
        .into_iter()
        .map(|(order, share, phase)| (order, budget * share / total, phase))
        .collect()
}

fn polyline_is_simple(poly: &[Vec2]) -> bool {
    let n = poly.len();
    for i in 0..n {
        let a0 = poly[i];
        let a1 = poly[(i + 1) % n];
        for j in (i + 2)..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            let b0 = poly[j];
            let b1 = poly[(j + 1) % n];
            if segments_intersect(a0, a1, b0, b1) {
                return false;
            }
        }
    }
    true
}

fn segments_intersect(a0: Vec2, a1: Vec2, b0: Vec2, b1: Vec2) -> bool {
    let r = a1 - a0;
    let s = b1 - b0;
    let denom = r.x * s.y - r.y * s.x;
    if denom.abs() < 1e-12 {
        return false;
    }
    let q = b0 - a0;
    let t = (q.x * s.y - q.y * s.x) / denom;
    let u = (q.x * r.y - q.y * r.x) / denom;
    (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u)
}

/// `searchsorted(..., side="right") - 1`; -1 wraps to the last segment.
fn segment_index(cum: &[f64], arc: f64) -> usize {
    let insert = cum.partition_point(|value| *value <= arc);
    let raw = insert as isize - 1;
    let clamped = raw.min(cum.len() as isize - 1);
    if clamped < 0 {
        cum.len() - 1
    } else {
        clamped as usize
    }
}

#[cfg(test)]
mod spatial_tests {
    use super::*;
    #[test]
    fn accelerated_queries_match_exhaustive_geometry() {
        let mut tracks: Vec<_> = crate::Circuit::ALL.into_iter().map(|c| c.build()).collect();
        tracks.push(Track::random(17));
        let angles: Vec<_> = (0..37).map(|i| i as f64 * TAU / 37.0).collect();
        for track in tracks {
            let all: Vec<_> = (0..track.seg_a.len()).collect();
            for i in 0..200 {
                let base = track.arc_point(track.lap_length * i as f64 / 199.0);
                let offset = if i % 5 == 0 { RAY_RANGE * 3.0 } else { 37.0 };
                let point = base
                    + Vec2 {
                        x: (i as f64).sin() * offset,
                        y: (i as f64).cos() * offset,
                    };
                let expected = track
                    .nearest_in(point, 0..track.centerline.len())
                    .unwrap()
                    .0;
                let actual = track.nearest_on_centerline(point);
                assert_eq!(actual.segment, expected.segment);
                assert_eq!(actual.arc, expected.arc);
                assert_eq!(actual.lateral, expected.lateral);
                let mut a = vec![0.0; angles.len()];
                let mut b = a.clone();
                track.cast_rays_into(point, &angles, &mut a);
                track.cast_candidates(point, &angles, &mut b, &all);
                assert_eq!(a, b);
            }
        }
    }
}
