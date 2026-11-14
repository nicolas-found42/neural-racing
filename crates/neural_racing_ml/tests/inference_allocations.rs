//! Count Rust allocations only; native BLAS's internal allocations are outside
//! this allocator and must not be inferred from this test.
use neural_racing_core::OBS_DIM;
use neural_racing_ml::{InferenceWorkspace, Policy};
use std::{
    alloc::{GlobalAlloc, Layout, System},
    cell::Cell,
    hint::black_box,
};

thread_local! {
    static TRACK:Cell<bool>=const {Cell::new(false)};
    static COUNT:Cell<usize>=const {Cell::new(0)};
}

struct CountingAllocator;

fn record() {
    let _ = TRACK.try_with(|enabled| {
        if enabled.get() {
            let _ = COUNT.try_with(|count| count.set(count.get() + 1));
        }
    });
}

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        record();
        unsafe { System.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        record();
        unsafe { System.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, pointer: *mut u8, layout: Layout) {
        unsafe { System.dealloc(pointer, layout) }
    }
    unsafe fn realloc(&self, pointer: *mut u8, layout: Layout, size: usize) -> *mut u8 {
        record();
        unsafe { System.realloc(pointer, layout, size) }
    }
}

#[global_allocator]
static ALLOCATOR: CountingAllocator = CountingAllocator;

#[test]
fn warmed_inference_reuses_rust_buffers_across_batch_sizes() {
    let policy = Policy::new(OBS_DIM, 256, 5, 42);
    let mut workspace = InferenceWorkspace::default();
    let obs = [0.25f32; 16 * OBS_DIM];
    black_box(policy.forward_into(&obs, 16, &mut workspace));
    COUNT.with(|count| count.set(0));
    TRACK.with(|track| track.set(true));
    for _ in 0..128 {
        for rows in [16, 4, 1, 0, 8, 16] {
            black_box(policy.forward_into(black_box(&obs[..rows * OBS_DIM]), rows, &mut workspace));
        }
    }
    TRACK.with(|track| track.set(false));
    let allocations = COUNT.with(Cell::get);
    assert_eq!(allocations, 0, "warmed inference allocated Rust storage");
}
