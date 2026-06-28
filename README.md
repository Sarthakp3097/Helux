# Helix

A real-time multi-sensor fusion engine written in Rust. Helix ingests GPS, IMU, and barometer data simultaneously over UDP, blends them through a complementary filter, and produces a fused navigation state — position, velocity, orientation, and altitude — at up to 500 Hz with zero heap allocations in the hot path.

I built this because most fusion libraries in this space are either Python (too slow for real embedded use), or C++ Kalman filter implementations that require careful noise-covariance tuning that nobody actually does correctly in the field. Helix aims to be the thing you reach for when you need something that *works*, that you can reason about, and that doesn't melt your CPU.

---

## How it works

Three independent async Tokio tasks listen for UDP packets — one per sensor. They all feed into a single bounded crossbeam channel. A dedicated OS thread on the other end of that channel runs the fusion loop. No async executor overhead in the hot path, no unbounded heap growth, no surprises.

```
                 ┌─────────────────────────────────────────────────┐
                 │                  Tokio runtime                  │
                 │                                                 │
  GPS  (10 Hz)  ─┤─► gps_listener task  ─────────────────────┐    │
 port 5001       │                                            │    │
                 │                                            ▼    │
  IMU (200 Hz)  ─┤─► imu_listener task  ─────► crossbeam ────────►│──► fusion thread
 port 5002       │                              channel            │    (OS thread)
                 │                              (cap: 1024)   ▲    │        │
  Baro (50 Hz)  ─┤─► baro_listener task ─────────────────────┘    │        ▼
 port 5003       │                                                 │   StateEstimate
                 └─────────────────────────────────────────────────┘   (logged / exposed)
```

### Why a dedicated OS thread for fusion?

Async executors are great for I/O-bound work but they introduce cooperative-scheduling jitter: a task only runs when it's polled, and other tasks on the same thread can delay that poll. For a fusion loop running at 500 Hz, that jitter is unacceptable. Pinning fusion to its own OS thread gives it a private CPU, eliminates executor overhead from the latency budget, and lets the three Tokio listener tasks multiplex freely on the remaining cores.

### Why a bounded channel?

The channel is capped at 1024 messages. If the fusion thread falls behind — say, because the system is under load — producers block rather than queuing unboundedly. This applies backpressure up through the stack and surfaces the problem immediately (blocked sender) rather than silently accumulating memory until the process OOMs.

### Why zero heap allocations?

Each sensor packet is a fixed-size `#[repr(C)]` struct. `bincode::deserialize` into a stack-allocated `[u8; 64]` buffer means no per-packet `Box<>` or `Vec<>`. At 200 Hz from the IMU alone, that's 200 potential allocations per second that never happen. Over hours of flight, that's a non-trivial amount of GC pressure you simply never incur.

---

## The complementary filter

The core of Helix is a single-axis complementary filter that fuses altitude from two sources with opposite error characteristics:

| Source | Strength | Weakness |
|---|---|---|
| GPS / barometer | Stable long-term, absolute reference | Noisy short-term, slow update rate |
| IMU integration | Fast, captures rapid dynamics | Accumulates drift over time |

The filter blends them with a single coefficient α:

```
altitude = α × (GPS/baro absolute) + (1 − α) × (IMU integrated delta)
```

The high-pass path `(1 − α)` captures rapid altitude changes from the IMU. The low-pass path `α` slowly corrects accumulated IMU drift using the absolute reference. Together they give you the best of both: fast transient response and long-term stability.

### Signal flow

```
accel_z ──► gravity removal ──► integrate to Δv ──► integrate to Δz ──► × (1 − α) ──┐
                                                                                        ▼
                                                                              altitude_estimate
                                                                                        ▲
GPS/baro altitude ──────────────────────────────────────────────────────── × α ────────┘
```

### Why not a Kalman filter?

A Kalman filter is the textbook answer here, and for good reason — it's optimal under Gaussian noise assumptions. But it comes with costs that matter in practice:

- **Tuning burden**: you need a process-noise covariance matrix Q and a measurement-noise covariance R. Getting these wrong produces divergence or sluggish tracking, and they're platform-specific.
- **Matrix inversion**: the covariance update step requires inverting a matrix every cycle. For a scalar altitude channel that's trivial, but it's still more complexity than a multiply-add.
- **Non-determinism**: floating-point instabilities in the covariance matrix can cause subtle drift. The complementary filter's behaviour is exactly predictable from `alpha` alone.

For altitude fusion on a single axis, the complementary filter is simply the right tool. If you need full 6-DOF state estimation with accelerometer bias modelling, reach for a proper EKF. For "give me a stable altitude number right now", this is it.

### Alpha tuning

α controls where you sit on the tradeoff spectrum. There's no universally correct value — it depends on your platform's dynamics and your sensor quality:

| Platform | Recommended α | Reasoning |
|---|---|---|
| Weather balloon / buoy | `0.98` | Slow dynamics, IMU drift dominates over minutes |
| General-purpose UAV | `0.85` | Good default, balanced transient + long-term |
| Racing drone / rocket | `0.60` | Fast altitude changes, need IMU to respond quickly |

Override at runtime: `HELIX_ALPHA=0.92 cargo run --release`

---

## Benchmark results

Run on an Apple M2 Pro (12-core), macOS 14.5, Rust 1.78.0 release build with `lto = "thin"`.

### Rust (criterion)

```
imu_update           time: [38.2 ns  38.6 ns  39.1 ns]
baro_update          time: [4.1 ns   4.2 ns   4.4 ns]
full_fusion_cycle    time: [42.8 ns  43.3 ns  44.1 ns]
barometric_altitude  time: [11.4 ns  11.7 ns  12.0 ns]
gyro_integration     time: [19.2 ns  19.6 ns  20.1 ns]
imu_deserialize      time: [28.1 ns  28.5 ns  29.2 ns]
```

Alpha sweep (full_fusion_cycle, different α values):

| α    | time (ns) |
|------|-----------|
| 0.50 | 43.1      |
| 0.60 | 43.2      |
| 0.70 | 43.3      |
| 0.85 | 43.3      |
| 0.95 | 43.4      |
| 0.98 | 43.4      |

Alpha has essentially zero effect on throughput — the arithmetic is the same regardless of the value. All variance here is measurement noise.

### Python baseline (timeit, CPython 3.12 + numpy 1.26)

```
imu_update           ~1 850 ns/iter   (~540 000 Hz theoretical)
baro_update          ~  620 ns/iter   (~1 600 000 Hz theoretical)
full_fusion_cycle    ~2 490 ns/iter   (~400 000 Hz theoretical)
```

### Comparison table

| Metric | Helix (Rust) | Baseline (Python + numpy) |
|---|---|---|
| `imu_update` latency (median) | **38.6 ns** | 1 850 ns |
| `full_fusion_cycle` latency (median) | **43.3 ns** | 2 490 ns |
| `full_fusion_cycle` p99 latency | **< 50 ns** | ~8 000 ns |
| Theoretical throughput | **~23 000 000 Hz** | ~400 000 Hz |
| Practical sustained throughput | **500+ Hz** (I/O bound) | ~35 Hz (GIL + asyncio overhead) |
| Memory, hot path | **0 heap allocs** | GC allocations per cycle |
| Binary size (release) | **~4.2 MB** | N/A |

The ~48× latency gap is real, but the more honest number for production use is "practical sustained throughput" — 500+ Hz for Rust vs ~35 Hz for Python, because the Python version also contends with the GIL and asyncio event loop overhead when doing real network I/O. The pure timeit numbers show the arithmetic cost; the practical numbers show what you actually get when sensors are firing.

Criterion HTML reports are in [`results/criterion/`](results/criterion/).

---

## Running it

### Prerequisites

- Rust 1.75+ (`rustup update stable`)
- Python 3.11+ with no extra dependencies for the simulators; `numpy` for the baseline benchmarks

### Start the sensor simulators

Open three terminals:

```bash
# Terminal 1 — GPS at 10 Hz
python sim/gps_sim.py

# Terminal 2 — IMU at 200 Hz
python sim/imu_sim.py

# Terminal 3 — Barometer at 50 Hz
python sim/baro_sim.py
```

Each simulator prints a line every N packets so you can verify they're running. They send to `127.0.0.1` by default; pass `--host <ip>` to target a remote machine.

### Start Helix

```bash
cargo run --release
```

You'll see structured log output like:

```
2026-06-25T19:43:11Z INFO helix: GPS listener bound to 0.0.0.0:5001
2026-06-25T19:43:11Z INFO helix: IMU listener bound to 0.0.0.0:5002
2026-06-25T19:43:11Z INFO helix: Baro listener bound to 0.0.0.0:5003
2026-06-25T19:43:11Z INFO helix: All tasks spawned — waiting for Ctrl-C
2026-06-25T19:43:11Z INFO helix: cycle=1 t=1719345791.123s pos=(0.00,0.00,0.00)m vel=(0.00,0.00,0.00)m/s alt_fused=150.21m rpy=(0.0°,0.0°,0.0°) α=0.85
2026-06-25T19:43:11Z INFO helix: cycle=847 t=1719345791.127s pos=(1.23,-0.45,150.21)m vel=(0.12,-0.03,0.01)m/s alt_fused=150.19m rpy=(0.7°,-0.3°,1.2°) α=0.85
```

Log output is throttled to ~500 Hz by default to avoid I/O becoming the bottleneck. Dial it with `HELIX_LOG_THROTTLE_US=0` to log every single cycle (warning: very loud).

### Configuration

All parameters are controllable via environment variables:

| Variable | Default | Description |
|---|---|---|
| `HELIX_GPS_PORT` | `5001` | UDP port for GPS packets |
| `HELIX_IMU_PORT` | `5002` | UDP port for IMU packets |
| `HELIX_BARO_PORT` | `5003` | UDP port for barometer packets |
| `HELIX_ALPHA` | `0.85` | Complementary filter coefficient |
| `HELIX_CHANNEL_DEPTH` | `1024` | Crossbeam channel capacity |
| `HELIX_LOG_THROTTLE_US` | `2000` | Min µs between log lines (0 = every cycle) |

Example — high-alpha mode for a slow platform, verbose logging:

```bash
HELIX_ALPHA=0.98 HELIX_LOG_THROTTLE_US=500000 cargo run --release
```

### Run the benchmarks

```bash
cargo bench
```

HTML reports land in `target/criterion/`. Pre-run results are in [`results/criterion/`](results/criterion/).

### Run the Python baseline benchmarks

```bash
# Needs numpy
pip install numpy

python python_baseline/fusion_baseline.py --benchmark
```

### Run the unit tests

```bash
cargo test
```

The test suite covers:
- Barometric altitude round-trips at sea level and cruising altitude
- Complementary filter convergence at rest (< 1 m drift after 1 s of stationary IMU)
- `alpha = 1.0` instantly tracks the absolute reference
- `alpha = 0.0` completely ignores the absolute reference

---

## Project layout

```
helix/
├── src/
│   ├── main.rs          — entry point, Tokio setup, OS thread spawn
│   ├── lib.rs           — re-exports for bench crate
│   ├── config.rs        — EngineConfig + env-var overrides
│   ├── state.rs         — StateEstimate struct + Display impl
│   ├── sensor/
│   │   ├── mod.rs       — SensorMessage enum
│   │   ├── gps.rs       — GpsPacket, latlon_to_enu_xy()
│   │   ├── imu.rs       — ImuPacket
│   │   └── barometer.rs — BaroPacket, barometric_altitude() + tests
│   └── fusion/
│       ├── mod.rs       — integrate_gyro()
│       └── filter.rs    — ComplementaryFilter + tests
├── benches/
│   └── fusion_bench.rs  — criterion suite (6 bench groups)
├── sim/
│   ├── gps_sim.py       — 10 Hz GPS simulator
│   ├── imu_sim.py       — 200 Hz IMU simulator
│   └── baro_sim.py      — 50 Hz barometer simulator
├── python_baseline/
│   └── fusion_baseline.py — asyncio + numpy reimplementation + timeit suite
└── results/
    ├── criterion/       — pre-run HTML benchmark reports
    └── baseline/        — Python timeit output
```

---

## Design notes and things I'd change

**What works well:**

The bounded channel + dedicated fusion thread design is clean. I've run this architecture on live hardware (a Pixhawk-compatible carrier board) and the latency profile is very predictable — no surprise spikes from the async executor, the backpressure kicks in exactly when you'd expect it to.

The barometric altitude conversion matches the Python reference to within 0.01 m at all tested altitudes, which means the simulators and the Rust engine agree on absolute altitude. That matters for tuning runs — if your absolute reference is off, alpha tuning becomes guesswork.

**What I'd add with more time:**

- A proper bias estimator for the IMU accelerometer. Right now, any constant DC offset in `accel_z` integrates into unbounded vertical velocity drift. A simple running mean subtracted before integration would fix 90% of real-world cases.
- An adaptive alpha: automatically tighten alpha (trust baro/GPS more) when the barometer's pressure variance is low, and loosen it (trust IMU more) during rapid altitude changes. This is a one-line heuristic but makes a real difference on rockets.
- UDP output of `StateEstimate` so downstream consumers can subscribe without being in-process.
- Proper timestamp synchronisation. Right now each sensor stamps packets with wall-clock time, but the fusion loop uses those timestamps naively. In real hardware, GPS PPS signals synchronise the sensor clocks. Without that, there's an implicit ~1 ms timestamp error between sensors.

**On the Kalman filter question:**

I get asked this a lot. Yes, an EKF with a well-tuned Q/R would win on raw accuracy. But the complementary filter has something the EKF doesn't: you can explain its behaviour to a non-expert in 30 seconds. "Alpha controls how much you trust the GPS versus the IMU" is something a field engineer can intuit and adjust on the spot. A 6×6 process noise matrix is not. For production systems where someone other than the original author has to maintain and tune the filter in the field, simplicity often beats theoretical optimality.

---

## License

MIT
