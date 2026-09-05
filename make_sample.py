"""Build a fake but realistic sweep and render both views, to see what the
viewer actually looks like. No sonar needed.

Scene: a pipe-like structure ahead and to starboard, a pool wall behind and to
one side, and background speckle.
"""
import sys

import cv2
import numpy as np

sys.path.insert(0, ".")
from auv.sonar_detect.sonar import Sweep
from auv.sonar_detect.detect import detect
from auv.sonar_detect.viewer import render

N_BINS = 1000
METRES_PER_BIN = 0.01          # 10 m range
STEP_GRAD = 2
ANGLES = list(range(0, 400, STEP_GRAD))
FORWARD_GRAD = 200             # straight ahead sits in the middle of the image

rng = np.random.default_rng(4)
img = rng.integers(0, 34, size=(len(ANGLES), N_BINS), dtype=np.uint8)   # background speckle


def grad_of(bearing_deg):
    return (bearing_deg / 0.9 + FORWARD_GRAD) % 400


def row_of(bearing_deg):
    return int(grad_of(bearing_deg) / STEP_GRAD)


def paint(bearing_deg, width_deg, range_m, depth_m, value):
    """Walk outward from the centre bearing so a wide object never accidentally
    paints the complement of the arc it was meant to cover when it crosses the
    image seam."""
    c0 = int((range_m - depth_m / 2) / METRES_PER_BIN)
    c1 = int((range_m + depth_m / 2) / METRES_PER_BIN)
    n_steps = max(1, int(width_deg / 0.9 / STEP_GRAD / 2))
    centre_row = row_of(bearing_deg)
    for k in range(-n_steps, n_steps + 1):
        img[(centre_row + k) % len(ANGLES), max(0, c0):c1] = value


# the target: a pipe structure, 20 deg to starboard at 4 m. Narrow in range
# (it is a thin object) but spread across several degrees because the beam
# smears it sideways.
paint(bearing_deg=20, width_deg=9, range_m=4.0, depth_m=0.25, value=215)
paint(bearing_deg=24, width_deg=4, range_m=4.15, depth_m=0.2, value=180)   # an elbow sticking out

# a pool wall behind us at 7 m - broad, spans a lot of bearing, fairly bright
paint(bearing_deg=150, width_deg=70, range_m=7.0, depth_m=0.35, value=170)

# a small piece of junk to port
paint(bearing_deg=-55, width_deg=5, range_m=2.6, depth_m=0.15, value=120)

# near-field ringing: the sonar hearing itself. Always there, always bright,
# always blanked by the detector before anything else happens.
img[:, :70] = 240

sweep = Sweep(img, ANGLES, METRES_PER_BIN, FORWARD_GRAD)

PROFILE = {
    "size_m":     {"min": 0.2, "ideal": 0.7, "max": 2.5},
    "elongation": {"min": 1.2, "ideal": 3.0, "max": 12.0},
    "solidity":   {"min": 0.4, "ideal": 0.85, "max": 1.0},
    "brightness": {"min": 60,  "ideal": 150, "max": 255},
}

dets = detect(sweep, profile=PROFILE)
print(f"{len(dets)} detections")
for i, d in enumerate(dets):
    parts = " ".join(f"{k.replace('_m','')}={v:.2f}" for k, v in d.scores.items())
    print(f"  {i}: {d.range_m:5.2f} m {d.bearing_deg:+7.1f} deg  score {d.score:.2f}   [{parts}]")

combined = render(sweep, dets)
cv2.imwrite("sample_view.png", combined)
print(f"\nwrote sample_view.png  {combined.shape[1]}x{combined.shape[0]}")
