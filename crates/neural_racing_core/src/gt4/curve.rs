//! Endpoint-clamped table lookup recovered at GT4 address 0x00359568.

/// A safe borrowed view of the source's signed-i16-count, separate-x/y table.
#[derive(Clone, Copy, Debug)]
pub struct Curve<'a> {
    xs: &'a [f32],
    ys: &'a [f32],
}

impl<'a> Curve<'a> {
    /// Rust boundary validation; the original routine does not validate tables.
    /// The supported port domain is finite data with strictly increasing x.
    pub fn new(xs: &'a [f32], ys: &'a [f32]) -> Option<Self> {
        if xs.is_empty()
            || xs.len() != ys.len()
            || xs.len() > i16::MAX as usize
            || !xs.iter().chain(ys).all(|v| v.is_finite())
            || !xs.windows(2).all(|p| p[0] < p[1])
        {
            return None;
        }
        Some(Self { xs, ys })
    }

    /// Preserves the binary-search cutoff, exact-knot return, and scalar f32
    /// subtract/multiply/divide/add ordering of 0x00359568.
    ///
    /// Panics on a non-finite query: special R5900 float behavior is not ported.
    pub fn sample(&self, input: f32) -> f32 {
        assert!(input.is_finite(), "GT4 curve query must be finite");
        let last = self.xs.len() - 1;
        if input <= self.xs[0] {
            return self.ys[0];
        }
        if self.xs[last] <= input {
            return self.ys[last];
        }
        let mut low = 0;
        let mut high = last;
        while high - low >= 4 {
            let mid = (high + low) >> 1;
            if input < self.xs[mid] {
                high = mid;
            } else if input == self.xs[mid] {
                return self.ys[mid];
            } else {
                low = mid;
            }
        }
        let mut upper = low;
        while upper < high && self.xs[upper] <= input {
            upper += 1;
        }
        let x0 = self.xs[upper - 1];
        let y0 = self.ys[upper - 1];
        let offset = input - x0;
        let span = self.xs[upper] - x0;
        let delta = self.ys[upper] - y0;
        let product = delta * offset;
        y0 + product / span
    }

    /// Original359568 branches with bounded per-instruction EE rounding.
    pub(crate) fn sample_ee(&self, input: f32) -> f32 {
        assert!(input.is_finite(), "GT4 curve query must be finite");
        let last = self.xs.len() - 1;
        if input <= self.xs[0] {
            return self.ys[0];
        }
        if self.xs[last] <= input {
            return self.ys[last];
        }
        let mut low = 0;
        let mut high = last;
        while high - low >= 4 {
            let mid = (high + low) >> 1;
            if input < self.xs[mid] {
                high = mid;
            } else if input == self.xs[mid] {
                return self.ys[mid];
            } else {
                low = mid;
            }
        }
        let mut upper = low;
        while upper < high && self.xs[upper] <= input {
            upper += 1;
        }
        let x0 = self.xs[upper - 1];
        let y0 = self.ys[upper - 1];
        use super::eefloat::{add, div, mul};
        let offset = add(input, -x0);
        let span = add(self.xs[upper], -x0);
        let delta = add(self.ys[upper], -y0);
        add(y0, div(mul(delta, offset), span))
    }
}

/// Original loaded bytes at 0x00620c08, shared by descriptors 0x620c80/0x620c90.
/// Caller field semantics are not encoded by the names of these raw fixtures.
pub const RAW_620C08_X: [f32; 9] = [
    f32::from_bits(0x00000000),
    f32::from_bits(0x40555554),
    f32::from_bits(0x40d55554),
    f32::from_bits(0x41055554),
    f32::from_bits(0x41831c71),
    f32::from_bits(0x41b638e2),
    f32::from_bits(0x420f5554),
    f32::from_bits(0x4232e38d),
    f32::from_bits(0x424d8e38),
];
pub const RAW_620C30_Y: [f32; 9] = [10.0, 25.0, 45.0, 51.0, 70.0, 77.0, 85.0, 88.0, 90.0];
pub const RAW_620C58_Y: [f32; 9] = [0.0, 0.0, -5.0, -6.0, -9.0, -11.0, -12.5, -14.0, -15.0];

impl Curve<'_> {
    /// Linear-scan variant at 0x00359830, used by GT4's tire-force tables.
    pub fn sample_linear(&self, input: f32) -> f32 {
        assert!(input.is_finite(), "GT4 curve query must be finite");
        let last = self.xs.len() - 1;
        if input <= self.xs[0] {
            return self.ys[0];
        }
        if self.xs[last] <= input {
            return self.ys[last];
        }
        let mut upper = 1;
        while upper < last && self.xs[upper] <= input {
            upper += 1;
        }
        let offset = input - self.xs[upper - 1];
        let low = self.ys[upper - 1];
        low + (offset * (self.ys[upper] - low)) / (self.xs[upper] - self.xs[upper - 1])
    }

    /// Same original359830 control flow with bounded per-instruction EE rounding.
    pub(crate) fn sample_linear_ee(&self, input: f32) -> f32 {
        assert!(input.is_finite(), "GT4 curve query must be finite");
        let last = self.xs.len() - 1;
        if input <= self.xs[0] {
            return self.ys[0];
        }
        if self.xs[last] <= input {
            return self.ys[last];
        }
        let mut upper = 1;
        while upper < last && self.xs[upper] <= input {
            upper += 1;
        }
        use super::eefloat::{add, div, mul};
        let offset = add(input, -self.xs[upper - 1]);
        let low = self.ys[upper - 1];
        add(
            low,
            div(
                mul(offset, add(self.ys[upper], -low)),
                add(self.xs[upper], -self.xs[upper - 1]),
            ),
        )
    }
}

/// Two ordinates sharing the first descriptor's x table: 0x003598f8.
#[derive(Clone, Copy, Debug)]
pub struct PairedCurve<'a> {
    first: Curve<'a>,
    second_y: &'a [f32],
}
impl<'a> PairedCurve<'a> {
    pub fn new(xs: &'a [f32], first_y: &'a [f32], second_y: &'a [f32]) -> Option<Self> {
        let first = Curve::new(xs, first_y)?;
        Curve::new(xs, second_y)?;
        Some(Self { first, second_y })
    }
    pub fn sample(&self, input: f32) -> (f32, f32) {
        assert!(input.is_finite(), "GT4 curve query must be finite");
        let xs = self.first.xs;
        let ys = self.first.ys;
        let last = xs.len() - 1;
        if input <= xs[0] {
            return (ys[0], self.second_y[0]);
        }
        if xs[last] <= input {
            return (ys[last], self.second_y[last]);
        }
        let mut upper = 1;
        while upper < last && xs[upper] <= input {
            upper += 1;
        }
        let offset = input - xs[upper - 1];
        let span = xs[upper] - xs[upper - 1];
        let first = ys[upper - 1] + (offset * (ys[upper] - ys[upper - 1])) / span;
        let second = self.second_y[upper - 1]
            + (offset * (self.second_y[upper] - self.second_y[upper - 1])) / span;
        (first, second)
    }

    /// Original3598f8 with bounded EE rounding for the coupled force pipeline.
    pub(crate) fn sample_ee(&self, input: f32) -> (f32, f32) {
        assert!(input.is_finite(), "GT4 curve query must be finite");
        let xs = self.first.xs;
        let ys = self.first.ys;
        let last = xs.len() - 1;
        if input <= xs[0] {
            return (ys[0], self.second_y[0]);
        }
        if xs[last] <= input {
            return (ys[last], self.second_y[last]);
        }
        let mut upper = 1;
        while upper < last && xs[upper] <= input {
            upper += 1;
        }
        use super::eefloat::{add, div, mul};
        let offset = add(input, -xs[upper - 1]);
        let span = add(xs[upper], -xs[upper - 1]);
        let first = add(
            ys[upper - 1],
            div(mul(offset, add(ys[upper], -ys[upper - 1])), span),
        );
        let second = add(
            self.second_y[upper - 1],
            div(
                mul(offset, add(self.second_y[upper], -self.second_y[upper - 1])),
                span,
            ),
        );
        (first, second)
    }
}
