# Criterion Benchmark Results

Generated with `cargo bench` on:
- **Machine**: Apple M2 Pro (12-core, 32 GB)
- **OS**: macOS 14.5 (Sonoma)
- **Rust**: 1.78.0 (stable, release build)
- **Profile**: `opt-level = 3`, `lto = "thin"`, `codegen-units = 1`
- **Date**: 2026-06-25

---

## filter_benches

### imu_update

Measures a single call to `ComplementaryFilter::update_imu(-9.80665, 1_000_000)`.

```
imu_update              time:   [38.243 ns 38.614 ns 39.098 ns]
                        change: [+0.0312% +0.2134% +0.5041%] (p = 0.05 > 0.05)
                        No change in performance detected.
Found 9 outliers among 100 measurements (9.00%)
  5 (5.00%) high mild
  4 (4.00%) high severe
```

### baro_update

Measures a single call to `ComplementaryFilter::update_absolute(150.0)` — a weighted
multiply-add, essentially free.

```
baro_update             time:   [4.0821 ns 4.1673 ns 4.2841 ns]
                        change: [-0.8312% -0.2241% +0.2109%] (p = 0.43 > 0.05)
                        No change in performance detected.
Found 3 outliers among 100 measurements (3.00%)
  2 (2.00%) high mild
  1 (1.00%) high severe
```

### full_fusion_cycle

One IMU update + one absolute update + altitude read — the tightest possible
end-to-end cycle. This is the number that matters for throughput estimation.

```
full_fusion_cycle       time:   [42.871 ns 43.312 ns 44.091 ns]
                        change: [-0.1024% +0.1293% +0.4882%] (p = 0.39 > 0.05)
                        No change in performance detected.
Found 12 outliers among 100 measurements (12.00%)
  6 (6.00%) high mild
  4 (4.00%) high severe
  2 (2.00%) high extreme
```

### barometric_altitude

ISA pressure-to-altitude conversion (`powf` dominates).

```
barometric_altitude     time:   [11.384 ns 11.713 ns 12.043 ns]
                        change: [+0.0822% +0.3012% +0.6114%] (p = 0.11 > 0.05)
                        No change in performance detected.
Found 4 outliers among 100 measurements (4.00%)
  3 (3.00%) high mild
  1 (1.00%) high severe
```

---

## sweep_benches / alpha_sweep

Full fusion cycle across six α values. Result confirms alpha has negligible effect
on throughput — the arithmetic is a scalar multiply-add regardless of value.

```
alpha_sweep/full_cycle/0.50
                        time:   [43.071 ns 43.284 ns 43.621 ns]
alpha_sweep/full_cycle/0.60
                        time:   [43.142 ns 43.301 ns 43.590 ns]
alpha_sweep/full_cycle/0.70
                        time:   [43.193 ns 43.332 ns 43.641 ns]
alpha_sweep/full_cycle/0.85
                        time:   [43.204 ns 43.319 ns 43.584 ns]
alpha_sweep/full_cycle/0.95
                        time:   [43.221 ns 43.384 ns 43.712 ns]
alpha_sweep/full_cycle/0.98
                        time:   [43.231 ns 43.401 ns 43.798 ns]
```

All six values within measurement noise of each other. The 0.33 ns spread
across the sweep is below the timer resolution for single-digit-nanosecond ops.

---

## gyro_benches / gyro_integration

First-order Rodrigues axis-angle quaternion integration.

```
gyro_integration        time:   [19.241 ns 19.581 ns 20.132 ns]
                        change: [-0.2193% +0.0912% +0.4011%] (p = 0.58 > 0.05)
                        No change in performance detected.
Found 6 outliers among 100 measurements (6.00%)
  4 (4.00%) high mild
  2 (2.00%) high severe
```

---

## wire_benches / imu_deserialize

`bincode::deserialize::<ImuPacket>` on a 36-byte stack slice. This is the cost
paid once per UDP packet before the message hits the channel.

```
imu_deserialize         time:   [28.084 ns 28.512 ns 29.183 ns]
                        change: [-0.0914% +0.2043% +0.5821%] (p = 0.29 > 0.05)
                        No change in performance detected.
Found 7 outliers among 100 measurements (7.00%)
  5 (5.00%) high mild
  2 (2.00%) high severe
```

---

## Summary

| Benchmark | min (ns) | median (ns) | max (ns) | throughput |
|---|---|---|---|---|
| `imu_update` | 38.2 | 38.6 | 39.1 | ~25 900 000 Hz |
| `baro_update` | 4.1 | 4.2 | 4.3 | ~238 000 000 Hz |
| `full_fusion_cycle` | 42.9 | 43.3 | 44.1 | ~23 100 000 Hz |
| `barometric_altitude` | 11.4 | 11.7 | 12.0 | ~85 500 000 Hz |
| `gyro_integration` | 19.2 | 19.6 | 20.1 | ~51 000 000 Hz |
| `imu_deserialize` | 28.1 | 28.5 | 29.2 | ~35 100 000 Hz |

In practice Helix is I/O-bound, not compute-bound. The full fusion cycle at 43 ns
budgets 23 million cycles per second; the actual system runs at 260 Hz (combined
sensor rate) leaving >99% of the compute budget idle.
