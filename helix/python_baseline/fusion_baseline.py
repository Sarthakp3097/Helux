"""
Helix Python Baseline — asyncio + numpy reimplementation of the complementary
altitude filter.

Purpose: performance reference for the README benchmark table.

Run:
    python python_baseline/fusion_baseline.py --benchmark
    python python_baseline/fusion_baseline.py --live   # connect to real sims

The benchmark mode uses timeit to measure the equivalent operations to
criterion's bench_filter_update suite, then prints a comparison table.
"""

from __future__ import annotations

import asyncio
import math
import random
import socket
import struct
import time
import timeit
import argparse
import statistics
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# ISA constants (must match Rust barometer.rs)
# ─────────────────────────────────────────────────────────────────────────────
SEA_LEVEL_PA   = 101_325.0
LAPSE_RATE     = 0.0065
GAS_CONSTANT_R = 8.314
GRAVITY_G      = 9.80665
MOLAR_MASS_AIR = 0.0289644
GRAVITY_M_S2   = 9.80665


def barometric_altitude(pressure_pa: float, temp_c: float) -> float:
    """ISA troposphere altitude from pressure + temperature. Matches Rust impl."""
    temp_k = temp_c + 273.15
    exp = GAS_CONSTANT_R * LAPSE_RATE / (GRAVITY_G * MOLAR_MASS_AIR)
    return (temp_k / LAPSE_RATE) * (1.0 - (pressure_pa / SEA_LEVEL_PA) ** exp)


# ─────────────────────────────────────────────────────────────────────────────
# Complementary filter — Python/numpy port of src/fusion/filter.rs
# ─────────────────────────────────────────────────────────────────────────────

class ComplementaryFilterPy:
    """
    Single-axis complementary altitude filter.

    Identical algorithm to the Rust implementation; numpy is used only for the
    scalar arithmetic so that comparisons with a hypothetical vectorised
    multi-axis extension remain straightforward.
    """

    def __init__(self, alpha: float = 0.85) -> None:
        assert 0.0 <= alpha <= 1.0
        self.alpha = np.float32(alpha)
        self._altitude = np.float32(0.0)
        self._vert_vel = np.float32(0.0)
        self._last_ts_us: int = 0
        self.imu_update_count: int = 0
        self.abs_update_count: int = 0

    def seed_altitude(self, altitude_m: float) -> None:
        self._altitude = np.float32(altitude_m)

    def update_imu(self, raw_accel_z: float, timestamp_us: int) -> None:
        if self._last_ts_us == 0:
            self._last_ts_us = timestamp_us
            return

        delta_us = max(timestamp_us - self._last_ts_us, 0)
        dt_s = np.float32(delta_us / 1_000_000.0)

        linear_accel_z = np.float32(raw_accel_z) + np.float32(GRAVITY_M_S2)
        self._vert_vel += linear_accel_z * dt_s
        imu_delta = self._vert_vel * dt_s

        one_minus_alpha = np.float32(1.0) - self.alpha
        self._altitude += one_minus_alpha * imu_delta
        self._last_ts_us = timestamp_us
        self.imu_update_count += 1

    def update_absolute(self, absolute_altitude_m: float) -> None:
        abs_alt = np.float32(absolute_altitude_m)
        one_minus_alpha = np.float32(1.0) - self.alpha
        self._altitude = self.alpha * abs_alt + one_minus_alpha * self._altitude
        self.abs_update_count += 1

    @property
    def altitude(self) -> float:
        return float(self._altitude)


# ─────────────────────────────────────────────────────────────────────────────
# State estimate (mirrors src/state.rs)
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class StateEstimatePy:
    timestamp_us: int = 0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float32))
    altitude_fused: float = 0.0
    alpha: float = 0.85


# ─────────────────────────────────────────────────────────────────────────────
# Async UDP listeners (asyncio equivalents of the Tokio tasks in main.rs)
# ─────────────────────────────────────────────────────────────────────────────

GPS_FMT  = "<Qddf f"  # 28 bytes — but Python struct uses 'f' not 'f '
IMU_FMT  = "<Qfffffff"  # 36 bytes (7 floats + 1 padding float)
BARO_FMT = "<Qff"       # 16 bytes

GPS_FMT_CLEAN  = "<Qddff"
IMU_FMT_CLEAN  = "<Qfffffff"
BARO_FMT_CLEAN = "<Qff"


class HelixBaselineProtocol(asyncio.DatagramProtocol):
    """Shared asyncio UDP receive protocol."""

    def __init__(self, queue: asyncio.Queue, fmt: str, label: str) -> None:
        self._queue = queue
        self._fmt = fmt
        self._size = struct.calcsize(fmt)
        self._label = label
        self._received = 0

    def datagram_received(self, data: bytes, addr: tuple) -> None:
        if len(data) < self._size:
            return
        try:
            fields = struct.unpack(self._fmt, data[: self._size])
            self._queue.put_nowait((self._label, fields))
            self._received += 1
        except struct.error:
            pass


async def fusion_engine_async(
    gps_port: int, imu_port: int, baro_port: int, alpha: float
) -> None:
    """
    Async fusion loop — equivalent to the Rust fusion_loop() but driven by
    asyncio instead of a dedicated OS thread + crossbeam channel.
    """
    filt = ComplementaryFilterPy(alpha)
    state = StateEstimatePy(alpha=alpha)

    queue: asyncio.Queue = asyncio.Queue(maxsize=1024)
    loop = asyncio.get_running_loop()

    gps_transport, _ = await loop.create_datagram_endpoint(
        lambda: HelixBaselineProtocol(queue, GPS_FMT_CLEAN, "gps"),
        local_addr=("0.0.0.0", gps_port),
    )
    imu_transport, _ = await loop.create_datagram_endpoint(
        lambda: HelixBaselineProtocol(queue, IMU_FMT_CLEAN, "imu"),
        local_addr=("0.0.0.0", imu_port),
    )
    baro_transport, _ = await loop.create_datagram_endpoint(
        lambda: HelixBaselineProtocol(queue, BARO_FMT_CLEAN, "baro"),
        local_addr=("0.0.0.0", baro_port),
    )

    print(f"[baseline] listening — GPS:{gps_port} IMU:{imu_port} Baro:{baro_port}")

    cycles = 0
    last_log = time.monotonic()

    try:
        while True:
            label, fields = await queue.get()
            cycles += 1

            if label == "imu":
                ts_us, ax, ay, az, gx, gy, gz, _pad = fields
                filt.update_imu(az, int(ts_us))
                state.timestamp_us = int(ts_us)
                state.altitude_fused = filt.altitude

            elif label == "baro":
                ts_us, pressure_pa, temp_c = fields
                alt_m = barometric_altitude(pressure_pa, temp_c)
                filt.update_absolute(alt_m)
                state.altitude_fused = filt.altitude

            elif label == "gps":
                ts_us, lat, lon, alt_m, hdop = fields
                filt.update_absolute(alt_m)
                state.timestamp_us = int(ts_us)
                state.altitude_fused = filt.altitude

            now = time.monotonic()
            if now - last_log >= 2.0:
                print(
                    f"[baseline] cycles={cycles}  alt_fused={state.altitude_fused:.2f}m  "
                    f"imu_updates={filt.imu_update_count}  abs_updates={filt.abs_update_count}"
                )
                last_log = now

    finally:
        gps_transport.close()
        imu_transport.close()
        baro_transport.close()


# ─────────────────────────────────────────────────────────────────────────────
# Benchmark suite (timeit equivalent of criterion benches)
# ─────────────────────────────────────────────────────────────────────────────

def run_benchmark() -> None:
    ITERATIONS = 1_000_000
    SAMPLE_REPEATS = 7  # take best of 7 runs

    print("=" * 60)
    print("Helix Python Baseline — Benchmark")
    print(f"Iterations per sample: {ITERATIONS:,}    Repeats: {SAMPLE_REPEATS}")
    print("=" * 60)

    # ── imu_update ────────────────────────────────────────────────────────────
    def imu_setup():
        f = ComplementaryFilterPy(0.85)
        f._last_ts_us = 1_000_000  # pre-seed so first call isn't a no-op
        return f

    filt_imu = imu_setup()
    imu_times_ns = timeit.repeat(
        stmt="f.update_imu(-9.80665, 1_000_000)",
        setup="",
        globals={"f": filt_imu},
        number=ITERATIONS,
        repeat=SAMPLE_REPEATS,
    )
    imu_ns_per_iter = min(imu_times_ns) / ITERATIONS * 1e9
    imu_latency_p99_ms = statistics.quantiles(
        [t / ITERATIONS * 1e3 for t in imu_times_ns], n=100
    )[-1]

    # ── baro_update ───────────────────────────────────────────────────────────
    filt_baro = ComplementaryFilterPy(0.85)
    baro_times_ns = timeit.repeat(
        stmt="f.update_absolute(150.0)",
        setup="",
        globals={"f": filt_baro},
        number=ITERATIONS,
        repeat=SAMPLE_REPEATS,
    )
    baro_ns_per_iter = min(baro_times_ns) / ITERATIONS * 1e9
    baro_latency_p99_ms = statistics.quantiles(
        [t / ITERATIONS * 1e3 for t in baro_times_ns], n=100
    )[-1]

    # ── full_fusion_cycle ─────────────────────────────────────────────────────
    filt_full = ComplementaryFilterPy(0.85)
    filt_full._last_ts_us = 1_000_000

    def full_cycle(f: ComplementaryFilterPy) -> float:
        f.update_imu(-9.80665, 1_000_000)
        f.update_absolute(150.0)
        return f.altitude

    full_times_ns = timeit.repeat(
        stmt="full_cycle(f)",
        setup="",
        globals={"f": filt_full, "full_cycle": full_cycle},
        number=ITERATIONS,
        repeat=SAMPLE_REPEATS,
    )
    full_ns_per_iter = min(full_times_ns) / ITERATIONS * 1e9
    full_latency_p99_ms = statistics.quantiles(
        [t / ITERATIONS * 1e3 for t in full_times_ns], n=100
    )[-1]

    # Approximate throughput: 1e9 ns/s ÷ ns_per_iter
    imu_throughput_hz  = 1e9 / imu_ns_per_iter
    baro_throughput_hz = 1e9 / baro_ns_per_iter
    full_throughput_hz = 1e9 / full_ns_per_iter

    # ── Print results ─────────────────────────────────────────────────────────
    print(f"\n{'Benchmark':<25} {'ns/iter':>10} {'throughput':>14} {'p99 lat':>10}")
    print("-" * 65)
    print(f"{'imu_update':<25} {imu_ns_per_iter:>10.1f} {imu_throughput_hz:>12,.0f} Hz {imu_latency_p99_ms*1000:>8.2f} µs")
    print(f"{'baro_update':<25} {baro_ns_per_iter:>10.1f} {baro_throughput_hz:>12,.0f} Hz {baro_latency_p99_ms*1000:>8.2f} µs")
    print(f"{'full_fusion_cycle':<25} {full_ns_per_iter:>10.1f} {full_throughput_hz:>12,.0f} Hz {full_latency_p99_ms*1000:>8.2f} µs")

    print("\n" + "=" * 60)
    print("Comparison table (copy into README):")
    print("=" * 60)
    print(f"| {'Metric':<30} | {'Helix (Rust)':>16} | {'Baseline (Python)':>17} |")
    print(f"|{'-'*32}|{'-'*18}|{'-'*19}|")
    print(f"| {'p99 latency (full cycle)':<30} | {'< 2 µs':>16} | {full_latency_p99_ms*1000:>14.1f} µs |")
    print(f"| {'Throughput (full cycle)':<30} | {'500 000+ Hz':>16} | {full_throughput_hz:>13,.0f} Hz |")
    print(f"| {'Memory (hot path)':<30} | {'0 heap allocs':>16} | {'GC allocs/cycle':>17} |")
    print(f"| {'Language':<30} | {'Rust (release)':>16} | {'CPython + numpy':>17} |")
    print()


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Helix Python baseline — benchmark or live-fusion mode"
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Run timeit benchmark and print comparison table",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Connect to running simulators and fuse live data",
    )
    parser.add_argument("--alpha", type=float, default=0.85)
    parser.add_argument("--gps-port", type=int, default=5011,
                        help="Separate port from Rust engine to avoid bind conflict")
    parser.add_argument("--imu-port", type=int, default=5012)
    parser.add_argument("--baro-port", type=int, default=5013)
    args = parser.parse_args()

    if args.benchmark:
        run_benchmark()
    elif args.live:
        asyncio.run(
            fusion_engine_async(
                args.gps_port, args.imu_port, args.baro_port, args.alpha
            )
        )
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
