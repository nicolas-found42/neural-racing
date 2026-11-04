//! Row-major dense products. macOS uses the system Accelerate framework's
//! stable LP64 CBLAS ABI; other platforms retain a portable implementation.
//! Contract: https://developer.apple.com/documentation/accelerate/cblas_sgemm(_:_:_:_:_:_:_:_:_:_:_:_:_:_:)

#[cfg(target_os = "macos")]
#[link(name = "Accelerate", kind = "framework")]
extern "C" {
    fn cblas_sgemm(
        order: i32,
        trans_a: i32,
        trans_b: i32,
        m: i32,
        n: i32,
        k: i32,
        alpha: f32,
        a: *const f32,
        lda: i32,
        b: *const f32,
        ldb: i32,
        beta: f32,
        c: *mut f32,
        ldc: i32,
    );
}

pub(crate) fn product(
    a: &[f32],
    b: &[f32],
    m: usize,
    n: usize,
    k: usize,
    transpose_a: bool,
    transpose_b: bool,
) -> Vec<f32> {
    assert_eq!(a.len(), m.checked_mul(k).expect("matrix shape overflow"));
    assert_eq!(b.len(), k.checked_mul(n).expect("matrix shape overflow"));
    let mut c = vec![0.0; m.checked_mul(n).expect("matrix shape overflow")];
    if m == 0 || n == 0 || k == 0 {
        return c;
    }
    #[cfg(target_os = "macos")]
    {
        let (m, n, k) = (
            i32::try_from(m).unwrap(),
            i32::try_from(n).unwrap(),
            i32::try_from(k).unwrap(),
        );
        // SAFETY: Exact backing lengths were checked above. Row-major leading
        // dimensions equal stored row widths; output is uniquely owned and
        // never aliases either input. All dimensions fit the LP64 int ABI.
        unsafe {
            cblas_sgemm(
                101,
                if transpose_a { 112 } else { 111 },
                if transpose_b { 112 } else { 111 },
                m,
                n,
                k,
                1.0,
                a.as_ptr(),
                if transpose_a { m } else { k },
                b.as_ptr(),
                if transpose_b { k } else { n },
                0.0,
                c.as_mut_ptr(),
                n,
            );
        }
    }
    #[cfg(not(target_os = "macos"))]
    reference(a, b, &mut c, m, n, k, transpose_a, transpose_b);
    c
}

#[cfg(any(not(target_os = "macos"), test))]
fn reference(
    a: &[f32],
    b: &[f32],
    c: &mut [f32],
    m: usize,
    n: usize,
    k: usize,
    ta: bool,
    tb: bool,
) {
    for i in 0..m {
        for j in 0..n {
            c[i * n + j] = (0..k)
                .map(|l| {
                    a[if ta { l * m + i } else { i * k + l }]
                        * b[if tb { j * k + l } else { l * n + j }]
                })
                .sum();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn native_products_match_scalar_for_rectangular_transposes_and_empty_batches() {
        for (m, n, k) in [
            (0, 3, 4),
            (4, 0, 9),
            (3, 2, 0),
            (1, 96, 27),
            (4, 96, 96),
            (96, 27, 512),
            (512, 96, 96),
        ] {
            let a: Vec<_> = (0..m * k).map(|i| (i as f32 * 0.13).sin()).collect();
            let b: Vec<_> = (0..k * n).map(|i| (i as f32 * 0.07).cos()).collect();
            for ta in [false, true] {
                for tb in [false, true] {
                    let actual = product(&a, &b, m, n, k, ta, tb);
                    let mut expected = vec![0.0; m * n];
                    reference(&a, &b, &mut expected, m, n, k, ta, tb);
                    for (x, y) in actual.iter().zip(expected) {
                        assert!((x - y).abs() < 0.0003, "{m} {n} {k} {ta} {tb}: {x} {y}");
                    }
                }
            }
        }
    }
}
