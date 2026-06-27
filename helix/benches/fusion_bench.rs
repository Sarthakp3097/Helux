/// Helix criterion benchmark suite.
///
/// Run:   cargo bench
/// HTML:  target/criterion/
///
/// Three bench groups mirror the Python timeit cases in fusion_baseline.py
/// so the README comparison table has apples-to-apples data.

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion};
use helix::fusion::ComplementaryFilter;
use helix::sensor::barometer::barometric_altitude;

// ─────────────────────────────────────────────────────────────────────────────
// Core filter micro-benchmarks
// ─────────────────────────────────────────────────────────────────────────────

fn bench_imu_update(c: &mut Criterion) {
    let mut filt = ComplementaryFilter::new(0.85);
    // Pre-seed timestamp so the first call isn't a no-op.
    filt.update_imu(-9.80665, 500_000);

    c.bench_function("imu_update", |b| {
        b.iter(|| {
            filt.update_imu(black_box(-9.80665), black_box(1_000_000));
        });
    });
}

fn bench_baro_update(c: &mut Criterion) {
    let mut filt = ComplementaryFilter::new(0.85);

    c.bench_function("baro_update", |b| {
        b.iter(|| {
            filt.update_absolute(black_box(150.0));
        });
    });
}

fn bench_full_fusion_cycle(c: &mut Criterion) {
    let mut filt = ComplementaryFilter::new(0.85);
    filt.update_imu(-9.80665, 500_000);

    c.bench_function("full_fusion_cycle", |b| {
        b.iter(|| {
            filt.update_imu(black_box(-9.80665), black_box(1_000_000));
            filt.update_absolute(black_box(150.0));
            black_box(filt.altitude());
        });
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Barometric altitude conversion (ISA model)
// ─────────────────────────────────────────────────────────────────────────────

fn bench_barometric_altitude(c: &mut Criterion) {
    c.bench_function("barometric_altitude", |b| {
        b.iter(|| {
            black_box(barometric_altitude(
                black_box(98_500.0_f32), // ~150 m elevation
                black_box(18.5_f32),
            ));
        });
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Alpha sweep — how throughput varies with the blending coefficient
// ─────────────────────────────────────────────────────────────────────────────

fn bench_alpha_sweep(c: &mut Criterion) {
    let alphas: &[f32] = &[0.50, 0.60, 0.70, 0.85, 0.95, 0.98];
    let mut group = c.benchmark_group("alpha_sweep");

    for &alpha in alphas {
        let mut filt = ComplementaryFilter::new(alpha);
        filt.update_imu(-9.80665, 500_000);

        group.bench_with_input(
            BenchmarkId::new("full_cycle", format!("{alpha:.2}")),
            &alpha,
            |b, _| {
                b.iter(|| {
                    filt.update_imu(black_box(-9.80665), black_box(1_000_000));
                    filt.update_absolute(black_box(150.0));
                    black_box(filt.altitude());
                });
            },
        );
    }

    group.finish();
}

// ─────────────────────────────────────────────────────────────────────────────
// Gyro integration (fusion::integrate_gyro)
// ─────────────────────────────────────────────────────────────────────────────

fn bench_gyro_integration(c: &mut Criterion) {
    use helix::fusion::integrate_gyro;
    use nalgebra::{UnitQuaternion, Vector3};

    let mut orientation = UnitQuaternion::<f32>::identity();
    let gyro = Vector3::new(0.01_f32, 0.005, 0.003);

    c.bench_function("gyro_integration", |b| {
        b.iter(|| {
            integrate_gyro(black_box(gyro), black_box(0.005), &mut orientation);
            black_box(orientation);
        });
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Wire deserialization (simulates what the UDP listeners do per packet)
// ─────────────────────────────────────────────────────────────────────────────

fn bench_imu_deserialize(c: &mut Criterion) {
    use helix::sensor::ImuPacket;

    // A realistic serialised IMU packet.
    let pkt = ImuPacket {
        timestamp_us: 1_718_000_000_000_000,
        accel_x: 0.03,
        accel_y: -0.01,
        accel_z: -9.81,
        gyro_x: 0.002,
        gyro_y: -0.001,
        gyro_z: 0.0005,
    };
    let serialised = bincode::serialize(&pkt).unwrap();
    let wire_slice: &[u8] = &serialised;

    c.bench_function("imu_deserialize", |b| {
        b.iter(|| {
            black_box(bincode::deserialize::<ImuPacket>(black_box(wire_slice)).unwrap());
        });
    });
}

// ─────────────────────────────────────────────────────────────────────────────
// Group registrations
// ─────────────────────────────────────────────────────────────────────────────

criterion_group!(
    filter_benches,
    bench_imu_update,
    bench_baro_update,
    bench_full_fusion_cycle,
    bench_barometric_altitude,
);

criterion_group!(sweep_benches, bench_alpha_sweep);
criterion_group!(gyro_benches, bench_gyro_integration);
criterion_group!(wire_benches, bench_imu_deserialize);

criterion_main!(filter_benches, sweep_benches, gyro_benches, wire_benches);
