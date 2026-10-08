use compas_forge_core::bench_support::retained_cube_linear_hit;
use criterion::{criterion_group, criterion_main, Criterion};
use std::hint::black_box;

fn native_retained_ccd(criterion: &mut Criterion) {
    let correctness = retained_cube_linear_hit();
    assert!(correctness.0);
    assert!((correctness.1 - 0.25).abs() < 1e-9);

    criterion.bench_function("native_retained_cube_linear_hit", |bencher| {
        bencher.iter(|| black_box(retained_cube_linear_hit()))
    });
}

criterion_group!(benches, native_retained_ccd);
criterion_main!(benches);
