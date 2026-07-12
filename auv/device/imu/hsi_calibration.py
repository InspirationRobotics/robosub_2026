#!/usr/bin/env python3
"""
hsi_calibration.py

Standalone hard/soft iron magnetometer calibration for the VectorNav VN-100.
Run this with the ROS IMU node STOPPED — it needs exclusive serial port access.

Collects raw magnetometer samples while you rotate the sub through a full 360°
yaw rotation, computes axis-aligned hard/soft iron correction, and saves the
result directly into the sub's JSON config file. vn100_serial.py loads and
applies these values automatically on next startup.

Usage:
    python3 hsi_calibration.py
    python3 hsi_calibration.py --port /dev/ttyUSB3 --duration 90 --config config/onyx.json
"""

import argparse
import json
import os
import sys
import time

import numpy as np
from serial import Serial

# Try to resolve port and config from the project's deviceHelper
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../"))
    from auv.utils import deviceHelper
    _default_port   = deviceHelper.dataFromConfig("vectornav")
    _sub_name       = deviceHelper.variables.get("sub", "onyx")
    _default_config = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        f"../../config/{_sub_name}.json"
    )
    _default_async  = deviceHelper.variables.get("VN_ASYNC_TYPE", 240)
except Exception:
    _default_port   = None
    _default_config = None
    _default_async  = 240


# ---------------------------------------------------------------------------
# VN-100 serial helpers
# ---------------------------------------------------------------------------

def _checksum(payload: str) -> str:
    cs = 0
    for c in payload:
        cs ^= ord(c)
    return f"{cs:02X}"

def _send(ser: Serial, payload: str):
    cs  = _checksum(payload)
    cmd = f"${payload}*{cs}\r\n"
    ser.write(cmd.encode())
    time.sleep(0.05)

def _set_async_output(ser: Serial, output_type: int):
    _send(ser, f"VNWRG,06,{output_type}")
    time.sleep(0.15)


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def collect_and_calibrate(port: str, duration: int, config_path: str,
                           original_async: int) -> bool:

    print(f"\nOpening {port} at 115200 baud...")
    try:
        ser = Serial(port=port, baudrate=115200, timeout=1)
    except Exception as e:
        print(f"ERROR: Could not open port: {e}", file=sys.stderr)
        return False

    time.sleep(1)

    # Switch to $VNYMR so we get raw magnetometer fields
    print("Switching VN-100 to $VNYMR output (includes raw magnetometer)...")
    _set_async_output(ser, 14)
    time.sleep(0.5)

    print(f"\nCollecting magnetometer samples for {duration} seconds.")
    print(">>> ROTATE THE SUB THROUGH A FULL 360° YAW SPIN NOW <<<\n")

    samples = []
    t0 = time.time()
    last_log = t0

    while time.time() - t0 < duration:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            continue

        if not line.startswith("$VNYMR"):
            continue

        parts = line.split(",")
        try:
            mx = float(parts[4])
            my = float(parts[5])
            mz = float(parts[6].split("*")[0])
            samples.append([mx, my, mz])
        except (IndexError, ValueError):
            continue

        now = time.time()
        if now - last_log >= 5:
            elapsed = now - t0
            remaining = duration - elapsed
            print(f"  {len(samples):4d} samples  |  {elapsed:.0f}s elapsed  |  {remaining:.0f}s remaining")
            last_log = now

    # Restore original async output type
    print(f"\nRestoring VN-100 output type to {original_async}...")
    _set_async_output(ser, original_async)
    ser.close()

    n = len(samples)
    print(f"\nCollected {n} magnetometer samples.")

    if n < 50:
        print(f"ERROR: Only {n} samples — need ≥50. Did the sensor switch to $VNYMR?",
              file=sys.stderr)
        return False

    data = np.array(samples)

    # --- Hard iron: midpoint of each axis range ---------------------------
    hard_iron = np.array([
        (data[:, 0].max() + data[:, 0].min()) / 2.0,
        (data[:, 1].max() + data[:, 1].min()) / 2.0,
        (data[:, 2].max() + data[:, 2].min()) / 2.0,
    ])

    # --- Soft iron: diagonal scale to normalise all axes to same range ----
    ranges = np.array([
        data[:, 0].max() - data[:, 0].min(),
        data[:, 1].max() - data[:, 1].min(),
        data[:, 2].max() - data[:, 2].min(),
    ])

    if np.any(ranges[:2] < 1e-6):
        print("ERROR: Near-zero range on X or Y mag axis — sub may not have rotated enough.",
              file=sys.stderr)
        return False

    # 2D calibration: only X and Y contribute to yaw heading.
    # Z is excluded from the quality gate and scale — it doesn't change during
    # a flat yaw spin (especially with upside-down mount or large Z hard iron bias).
    xy_ranges  = ranges[:2]
    avg_range  = xy_ranges.mean()
    xy_scale   = avg_range / xy_ranges
    # Z scale set to 1.0 — no correction applied to Z axis
    scale      = np.array([xy_scale[0], xy_scale[1], 1.0])
    soft_iron  = np.diag(scale)

    ratio = xy_scale.max() / xy_scale.min()
    print(f"\nHard iron bias  (Gauss): {np.round(hard_iron, 5)}")
    print(f"Soft iron scale (diag):  {np.round(scale, 5)}")
    print(f"XY axis scale ratio: {ratio:.3f}  (threshold: 2.0)")
    print(f"Z axis excluded from 2D calibration (Z hard iron bias: {hard_iron[2]:.5f} Gauss)")

    if ratio > 2.0:
        print(
            f"\nWARNING: XY scale ratio {ratio:.2f} > 2.0 — rotation was uneven or incomplete.\n"
            "Calibration NOT saved. Repeat with a smoother, more complete 360° rotation.",
            file=sys.stderr
        )
        return False

    # --- Save to JSON config ----------------------------------------------
    try:
        with open(config_path, "r") as f:
            cfg = json.load(f)

        cfg["MAG_HARD_IRON"] = hard_iron.tolist()
        cfg["MAG_SOFT_IRON"] = soft_iron.tolist()

        with open(config_path, "w") as f:
            json.dump(cfg, f, indent=4)

        print(f"\nCalibration saved to {config_path}")
        print("vn100_serial.py will apply these values automatically on next startup.")
        return True

    except Exception as e:
        print(f"ERROR: Could not write config file: {e}", file=sys.stderr)
        return False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="VectorNav VN-100 hard/soft iron calibration — saves to JSON config"
    )
    parser.add_argument(
        "--port", type=str, default=_default_port,
        help="Serial port (e.g. /dev/ttyUSB3). Defaults to project device config."
    )
    parser.add_argument(
        "--duration", type=int, default=60,
        help="Collection time in seconds (default: 60). Rotate the sub during this window."
    )
    parser.add_argument(
        "--config", type=str, default=_default_config,
        help="Path to the sub JSON config to update (e.g. config/onyx.json)."
    )
    parser.add_argument(
        "--async-type", type=int, default=_default_async, dest="async_type",
        help="VN async output type to restore after calibration (14=VNYMR, 240=VNYBA)."
    )
    args = parser.parse_args()

    if not args.port:
        print("ERROR: No port specified. Use --port /dev/ttyUSBx", file=sys.stderr)
        sys.exit(1)
    if not args.config:
        print("ERROR: No config path. Use --config config/onyx.json", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(args.config):
        print(f"ERROR: Config file not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    print("=" * 60)
    print("VN-100 Hard/Soft Iron Calibration")
    print("=" * 60)
    print(f"  Port:     {args.port}")
    print(f"  Duration: {args.duration}s")
    print(f"  Config:   {args.config}")
    print()
    print("Make sure the ROS IMU node is STOPPED before continuing.")
    input("Press Enter to begin, Ctrl+C to abort...\n")

    success = collect_and_calibrate(
        port=args.port,
        duration=args.duration,
        config_path=args.config,
        original_async=args.async_type,
    )
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
