"""
Barometer simulator — sends 50 Hz packets to Helix on UDP port 5003.

Wire format (little-endian, 16 bytes):
  Q  timestamp_us   uint64
  f  pressure_pa    float32  Pascals
  f  temp_c         float32  Celsius

Noise model:
  Pressure: Gaussian σ = 50 Pa  (~±4 m altitude equivalent)
  Temp:     Gaussian σ = 0.3 °C

The true altitude follows the same slow sinusoidal profile as the GPS sim
(cruise at ~150 m, period 60 s).  Pressure is derived from the ISA model
so the Rust barometric_altitude() round-trips cleanly.
"""

import socket
import struct
import time
import math
import random
import argparse

STRUCT_FMT = "<Qff"
PACKET_SIZE = struct.calcsize(STRUCT_FMT)  # 16 bytes

assert PACKET_SIZE == 16, f"Unexpected packet size: {PACKET_SIZE}"

# ISA constants — must match Rust implementation exactly.
SEA_LEVEL_PA    = 101_325.0
LAPSE_RATE      = 0.0065          # K/m
GAS_CONSTANT_R  = 8.314
GRAVITY_G       = 9.80665
MOLAR_MASS_AIR  = 0.0289644

CRUISE_ALT_M    = 150.0
TEMP_SEA_LEVEL  = 288.15          # standard ISA sea-level temperature (K)


def altitude_to_pressure(alt_m: float, temp_k: float) -> float:
    """Inverse ISA: altitude → pressure in Pascals."""
    exponent = (GRAVITY_G * MOLAR_MASS_AIR) / (GAS_CONSTANT_R * LAPSE_RATE)
    temp_at_alt = temp_k - LAPSE_RATE * alt_m
    ratio = temp_at_alt / temp_k
    return SEA_LEVEL_PA * (ratio ** exponent)


def run(host: str, port: int, rate_hz: float, duration_s: float | None) -> None:
    interval_s = 1.0 / rate_hz
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sim_t = 0.0
    sent = 0
    t_start = time.monotonic()

    print(f"[baro_sim] → {host}:{port}  rate={rate_hz} Hz  fmt={PACKET_SIZE}B/pkt")

    try:
        while True:
            if duration_s is not None and (time.monotonic() - t_start) >= duration_s:
                break

            timestamp_us = int(time.time() * 1_000_000)

            # True altitude oscillates between 148 and 152 m over 60 s.
            true_alt_m = CRUISE_ALT_M + 2.0 * math.sin(2 * math.pi * sim_t / 60.0)
            true_temp_c = 20.0 + 1.0 * math.sin(2 * math.pi * sim_t / 300.0)
            true_temp_k = true_temp_c + 273.15

            true_pressure_pa = altitude_to_pressure(true_alt_m, TEMP_SEA_LEVEL)

            # Add sensor noise.
            noisy_pressure = true_pressure_pa + random.gauss(0.0, 50.0)
            noisy_temp_c   = true_temp_c + random.gauss(0.0, 0.3)

            pkt = struct.pack(STRUCT_FMT, timestamp_us, noisy_pressure, noisy_temp_c)
            sock.sendto(pkt, (host, port))

            sent += 1
            if sent % (int(rate_hz) * 10) == 0:
                elapsed = time.monotonic() - t_start
                actual_hz = sent / elapsed if elapsed > 0 else 0
                print(
                    f"[baro_sim] {sent} pkts sent  actual={actual_hz:.1f} Hz  "
                    f"alt≈{true_alt_m:.1f}m  P={noisy_pressure:.1f}Pa  T={noisy_temp_c:.2f}°C"
                )

            sim_t += interval_s
            time.sleep(interval_s)

    except KeyboardInterrupt:
        print(f"\n[baro_sim] stopped after {sent} packets")
    finally:
        sock.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Helix barometer simulator")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5003)
    parser.add_argument("--rate", type=float, default=50.0, help="Hz")
    parser.add_argument("--duration", type=float, default=None, help="seconds")
    args = parser.parse_args()
    run(args.host, args.port, args.rate, args.duration)
