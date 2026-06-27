"""
IMU simulator — sends 200 Hz packets to Helix on UDP port 5002.

Wire format (little-endian, 36 bytes):
  Q  timestamp_us   uint64
  f  accel_x        float32  m/s²
  f  accel_y        float32  m/s²
  f  accel_z        float32  m/s²  (includes gravity: ~−9.81 at rest)
  f  gyro_x         float32  rad/s
  f  gyro_y         float32  rad/s
  f  gyro_z         float32  rad/s

Noise model:
  Accelerometer: white Gaussian noise σ = 0.05 m/s²  (MEMS-grade)
  Gyroscope:     white Gaussian noise σ = 0.001 rad/s
  Slow sinusoidal oscillation added to gyro to simulate mild attitude changes.
"""

import socket
import struct
import time
import math
import random
import argparse

STRUCT_FMT = "<Qfffffff"
PACKET_SIZE = struct.calcsize(STRUCT_FMT)  # 36 bytes

assert PACKET_SIZE == 36, f"Unexpected packet size: {PACKET_SIZE}"


def run(host: str, port: int, rate_hz: float, duration_s: float | None) -> None:
    interval_s = 1.0 / rate_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sim_t = 0.0          # simulation time for oscillation phase
    sent = 0
    t_start = time.monotonic()

    print(f"[imu_sim] → {host}:{port}  rate={rate_hz} Hz  fmt={PACKET_SIZE}B/pkt")

    try:
        while True:
            if duration_s is not None and (time.monotonic() - t_start) >= duration_s:
                break

            timestamp_us = int(time.time() * 1_000_000)

            # Simulate a body hovering at ~150 m with gentle attitude oscillation.
            accel_x = random.gauss(0.0, 0.05)
            accel_y = random.gauss(0.0, 0.05)
            accel_z = -9.80665 + random.gauss(0.0, 0.05)   # at rest = −g

            # Slow sinusoidal roll/pitch/yaw — realistic for a hovering UAV.
            gyro_x = 0.012 * math.sin(2 * math.pi * 0.25 * sim_t) + random.gauss(0.0, 0.001)
            gyro_y = 0.010 * math.cos(2 * math.pi * 0.20 * sim_t) + random.gauss(0.0, 0.001)
            gyro_z = 0.005 * math.sin(2 * math.pi * 0.10 * sim_t) + random.gauss(0.0, 0.001)

            pkt = struct.pack(
                STRUCT_FMT,
                timestamp_us,
                accel_x, accel_y, accel_z,
                gyro_x, gyro_y, gyro_z,
                0.0,  # padding — struct has 7 floats; 8th keeps 8-byte alignment
            )
            sock.sendto(pkt, (host, port))

            sent += 1
            if sent % (int(rate_hz) * 5) == 0:
                elapsed = time.monotonic() - t_start
                actual_hz = sent / elapsed if elapsed > 0 else 0
                print(f"[imu_sim] {sent} pkts sent  actual={actual_hz:.1f} Hz")

            sim_t += interval_s
            time.sleep(interval_s)

    except KeyboardInterrupt:
        print(f"\n[imu_sim] stopped after {sent} packets")
    finally:
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Helix IMU simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5002)
    parser.add_argument("--rate", type=float, default=200.0, help="Hz")
    parser.add_argument("--duration", type=float, default=None, help="seconds (None = infinite)")
    args = parser.parse_args()
    run(args.host, args.port, args.rate, args.duration)
