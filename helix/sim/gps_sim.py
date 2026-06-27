"""
GPS simulator — sends 10 Hz packets to Helix on UDP port 5001.

Wire format (little-endian, 28 bytes):
  Q  timestamp_us   uint64
  d  latitude       float64  degrees
  d  longitude      float64  degrees
  f  altitude_m     float32  metres MSL
  f  hdop            float32  (1.2 = good fix)

Noise model:
  Horizontal: Gaussian σ = 3.0 m converted to degree-equivalent offset
              (~2.7e-5° per metre at mid-latitudes)
  Altitude:   Gaussian σ = 5.0 m (GPS vertical accuracy is 2× horizontal)
  HDOP:       uniform [0.9, 2.5] — simulates varying satellite geometry
"""

import socket
import struct
import time
import math
import random
import argparse

STRUCT_FMT = "<Qddf f"
PACKET_SIZE = struct.calcsize(STRUCT_FMT)  # 28 bytes

assert PACKET_SIZE == 28, f"Unexpected packet size: {PACKET_SIZE}"

# Simulated platform trajectory: slow circular orbit at 150 m altitude.
ORIGIN_LAT = 37.7749   # San Francisco, CA
ORIGIN_LON = -122.4194
ORBIT_RADIUS_M = 200.0
ORBIT_PERIOD_S = 120.0
CRUISE_ALT_M = 150.0

# 1 metre ≈ degrees (approximate, valid at mid-latitudes)
M_PER_DEG_LAT = 111_320.0
M_PER_DEG_LON = 111_320.0 * math.cos(math.radians(ORIGIN_LAT))


def run(host: str, port: int, rate_hz: float, duration_s: float | None) -> None:
    interval_s = 1.0 / rate_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sim_t = 0.0
    sent = 0
    t_start = time.monotonic()

    print(f"[gps_sim] → {host}:{port}  rate={rate_hz} Hz  fmt={PACKET_SIZE}B/pkt")

    try:
        while True:
            if duration_s is not None and (time.monotonic() - t_start) >= duration_s:
                break

            timestamp_us = int(time.time() * 1_000_000)

            # True position on the circular orbit.
            phase = 2 * math.pi * (sim_t / ORBIT_PERIOD_S)
            true_east_m  = ORBIT_RADIUS_M * math.cos(phase)
            true_north_m = ORBIT_RADIUS_M * math.sin(phase)
            true_alt_m   = CRUISE_ALT_M + 2.0 * math.sin(2 * math.pi * sim_t / 60.0)

            # Add realistic GPS noise.
            noisy_east_m  = true_east_m  + random.gauss(0.0, 3.0)
            noisy_north_m = true_north_m + random.gauss(0.0, 3.0)
            noisy_alt_m   = true_alt_m   + random.gauss(0.0, 5.0)

            latitude  = ORIGIN_LAT  + noisy_north_m / M_PER_DEG_LAT
            longitude = ORIGIN_LON  + noisy_east_m  / M_PER_DEG_LON
            hdop      = random.uniform(0.9, 2.5)

            pkt = struct.pack(
                STRUCT_FMT,
                timestamp_us,
                latitude,
                longitude,
                noisy_alt_m,
                hdop,
            )
            sock.sendto(pkt, (host, port))

            sent += 1
            if sent % (int(rate_hz) * 10) == 0:
                elapsed = time.monotonic() - t_start
                actual_hz = sent / elapsed if elapsed > 0 else 0
                print(
                    f"[gps_sim] {sent} pkts sent  actual={actual_hz:.1f} Hz  "
                    f"pos=({latitude:.6f}°, {longitude:.6f}°)  alt={noisy_alt_m:.1f}m"
                )

            sim_t += interval_s
            time.sleep(interval_s)

    except KeyboardInterrupt:
        print(f"\n[gps_sim] stopped after {sent} packets")
    finally:
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Helix GPS simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--rate", type=float, default=10.0, help="Hz")
    parser.add_argument("--duration", type=float, default=None, help="seconds")
    args = parser.parse_args()
    run(args.host, args.port, args.rate, args.duration)
