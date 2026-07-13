#!/usr/bin/env python3
"""
hsi_calibration.py

Standalone 3D hard/soft iron magnetometer calibration for the VectorNav VN-100.
Run this with the ROS IMU node STOPPED — it needs exclusive serial port access.

Fits a full 3D ellipsoid to magnetometer samples collected while the sub is
rotated through multiple orientations (yaw spins + pitch/roll tilts).
Computes hard iron (ellipsoid center) and full 3x3 soft iron correction matrix,
then saves directly into the sub's JSON config file. vn100_serial.py loads and
applies these values automatically on next startup via Register 44.

Usage:
    python3 hsi_calibration.py
    python3 hsi_calibration.py --port /dev/ttyUSB0 --duration 180 --config config/onyx.json

Physical procedure during collection window:
    1. Two full 360 degree YAW spins (flat/level)
    2. Full 360 degree PITCH rotation (nose all the way over)
    3. Full 360 degree ROLL rotation  (roll all the way over)
    4. One more full 360 degree YAW spin to finish
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
# 3D ellipsoid fitting
# ---------------------------------------------------------------------------

def fit_ellipsoid_3d(data: np.ndarray):
    """
    Fits a general ellipsoid to 3D magnetometer data using algebraic least squares.

    The raw magnetometer traces an ellipsoid due to hard and soft iron distortion.
    This function finds the ellipsoid equation, extracts its center (hard iron bias)
    and shape matrix, then computes the 3x3 soft iron correction matrix that maps
    the ellipsoid back to a unit sphere.

    Returns:
        hard_iron  : np.ndarray shape (3,)  — center of the ellipsoid in Gauss
        soft_iron  : np.ndarray shape (3,3) — correction matrix for Register 44
        condition  : float                  — condition number (quality indicator)
    """
    x, y, z = data[:, 0], data[:, 1], data[:, 2]

    # Build design matrix — each row represents one sample in the ellipsoid equation:
    # Ax^2 + By^2 + Cz^2 + 2Dxy + 2Exz + 2Fyz + 2Gx + 2Hy + 2Iz = 1
    D = np.column_stack([
        x*x, y*y, z*z,
        2*x*y, 2*x*z, 2*y*z,
        2*x,   2*y,   2*z
    ])

    ones = np.ones(len(data))

    # Least squares solve: D v = 1
    v, residuals, rank, sv = np.linalg.lstsq(D, ones, rcond=None)
    condition = sv[0] / sv[-1]  # condition number of design matrix

    # Assemble the 4x4 algebraic ellipsoid matrix
    #  [ A  D  E  G ]
    #  [ D  B  F  H ]
    #  [ E  F  C  I ]
    #  [ G  H  I  -1]
    A4 = np.array([
        [v[0], v[3], v[4], v[6]],
        [v[3], v[1], v[5], v[7]],
        [v[4], v[5], v[2], v[8]],
        [v[6], v[7], v[8], -1.0]
    ])

    A33 = A4[:3, :3]

    # Hard iron = center of the ellipsoid
    hard_iron = -np.linalg.solve(A33, A4[:3, 3])

    # Translate the ellipsoid to be centered at origin
    T = np.eye(4)
    T[3, :3] = hard_iron
    M = T @ A4 @ T.T

    # The centered ellipsoid: x^T R x = R33/(-R44)
    R = M[:3, :3] / (-M[3, 3])

    # Eigendecompose to get principal axes and radii
    evals, evecs = np.linalg.eigh(R)

    # Guard against numerical issues
    if np.any(evals <= 0):
        raise ValueError(
            f"Ellipsoid fit produced non-positive eigenvalues {evals} — "
            "data may not span all three axes. Rotate through more orientations."
        )

    # Radii of the ellipsoid along its principal axes
    radii = 1.0 / np.sqrt(evals)

    # Soft iron matrix: maps the ellipsoid back to a sphere.
    # We use only the diagonal scale in the SENSOR frame (not the ellipsoid's
    # principal frame) to avoid introducing a rotation into the correction.
    # Off-diagonal terms from an unevenly sampled ellipsoid cause heading rotation
    # errors (e.g. 90 deg left turn after calibration).
    avg_radius = np.mean(radii)

    # Reconstruct the full shape matrix in sensor frame
    M_shape = evecs @ np.diag(1.0 / radii**2) @ evecs.T

    # Extract only the diagonal — per-axis scale in sensor frame,
    # ignoring cross-axis coupling that requires full 360 pitch/roll to resolve
    diag_scale = np.sqrt(np.diag(M_shape))
    diag_scale = diag_scale / np.mean(diag_scale)  # normalise to ~1.0

    soft_iron = np.diag(avg_radius * diag_scale / np.mean(radii * diag_scale))

    return hard_iron, soft_iron, condition


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

    print("Switching VN-100 to $VNYMR output (includes raw magnetometer)...")
    _set_async_output(ser, 14)
    time.sleep(0.5)

    print(f"\nCollecting magnetometer samples for {duration} seconds.")
    print("=" * 55)
    print("ROTATION PROCEDURE (full 360 in all three axes):")
    print("  1. Two full 360 deg YAW spins (flat/level)")
    print("  2. Full 360 deg PITCH rotation (nose all the way over)")
    print("  3. Full 360 deg ROLL rotation  (roll all the way over)")
    print("  4. One more full 360 deg YAW spin to finish")
    print()
    print("  Each rotation must be a COMPLETE 360 — partial rotations")
    print("  cause the ellipsoid fit to introduce heading errors.")
    print("=" * 55)
    print()

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
            # VN-100 is mounted upside down (roll ~180 deg).
            # Flip Y and Z axes to transform raw mag readings into the sub's
            # body frame before fitting — prevents the ellipsoid from being
            # mirrored, which would introduce a 90/180 deg heading error.
            samples.append([mx, -my, -mz])
        except (IndexError, ValueError):
            continue

        now = time.time()
        if now - last_log >= 5:
            elapsed   = now - t0
            remaining = duration - elapsed
            print(f"  {len(samples):4d} samples  |  {elapsed:.0f}s elapsed  |  {remaining:.0f}s remaining")
            last_log = now

    print(f"\nRestoring VN-100 output type to {original_async}...")
    _set_async_output(ser, original_async)
    ser.close()

    n = len(samples)
    print(f"\nCollected {n} magnetometer samples.")

    if n < 500:
        print(f"ERROR: Only {n} samples — need >=500 for 3D ellipsoid fit.",
              file=sys.stderr)
        return False

    data = np.array(samples)

    # Print raw data stats so the user can see coverage
    print("\nRaw data ranges (body frame after mount correction):")
    axes = ['X', 'Y', 'Z']
    for i, ax in enumerate(axes):
        lo, hi = data[:, i].min(), data[:, i].max()
        print(f"  mag{ax}: {lo:.5f}  to  {hi:.5f}  (range {hi-lo:.5f} Gauss)")

    # Check that all three axes have meaningful variation
    ranges = np.array([data[:, i].max() - data[:, i].min() for i in range(3)])
    if np.any(ranges < 0.01):
        print(
            f"ERROR: One or more axes have near-zero range {np.round(ranges, 5)} — "
            "the sub was not rotated through enough orientations.",
            file=sys.stderr
        )
        return False

    print("\nFitting 3D ellipsoid to magnetometer data...")
    try:
        hard_iron, soft_iron, condition = fit_ellipsoid_3d(data)
    except Exception as e:
        print(f"ERROR: Ellipsoid fit failed: {e}", file=sys.stderr)
        return False

    print(f"\nHard iron bias  (Gauss):     {np.round(hard_iron, 5)}")
    print(f"Soft iron matrix:")
    for row in soft_iron:
        print(f"  {np.round(row, 5)}")
    print(f"Fit condition number: {condition:.1f}  (lower is better, <1000 is good)")

    # Quality gate: reject if fit is numerically unstable
    if condition > 5000:
        print(
            f"\nWARNING: Condition number {condition:.0f} > 5000 — fit is numerically unstable.\n"
            "Calibration NOT saved. Rotate through more orientations and try again.",
            file=sys.stderr
        )
        return False

    # Verify soft iron is reasonable — diagonal should be near 1.0
    diag = np.diag(soft_iron)
    if diag.max() > 3.0 or diag.min() < 0.1:
        print(
            f"\nWARNING: Soft iron diagonal {np.round(diag, 3)} is unreasonable.\n"
            "Calibration NOT saved. Ensure the sub rotates through all orientations evenly.",
            file=sys.stderr
        )
        return False

    # Verify hard iron magnitude is physically reasonable
    if np.linalg.norm(hard_iron) > 0.8:
        print(
            f"\nWARNING: Hard iron magnitude {np.linalg.norm(hard_iron):.4f} Gauss is very large.\n"
            "This may indicate a strong permanent magnet very close to the sensor.\n"
            "Saving anyway — verify heading accuracy after restart.",
            file=sys.stderr
        )

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
        description="VectorNav VN-100 3D hard/soft iron calibration — saves to JSON config"
    )
    parser.add_argument(
        "--port", type=str, default=_default_port,
        help="Serial port (e.g. /dev/ttyUSB0). Defaults to project device config."
    )
    parser.add_argument(
        "--duration", type=int, default=180,
        help="Collection time in seconds (default: 180). Rotate the sub during this window."
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

    print("=" * 55)
    print("VN-100 3D Hard/Soft Iron Calibration")
    print("=" * 55)
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
