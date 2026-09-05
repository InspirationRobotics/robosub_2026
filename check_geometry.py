"""Verification with a synthetic sweep - no sonar, no water.

Plants a bright patch at a known range and bearing, runs the real detection
pipeline over it, and checks the numbers that come back match what was planted.
This is checking the geometry conversions, which are the part most likely to be
silently wrong in a way you would not notice until nothing lined up in a pool.
"""
import sys

import numpy as np

sys.path.insert(0, ".")
from auv.sonar_detect.sonar import Sweep
from auv.sonar_detect.detect import detect, measure, find_blobs
from auv.sonar_detect.viewer import render

METRES_PER_BIN = 0.01     # 10 m over 1000 bins
N_BINS = 1000
STEP_GRAD = 2
ANGLES = list(range(0, 400, STEP_GRAD))


def make_sweep(range_m, bearing_deg, radial_m=0.4, angular_deg=6.0, value=200,
               forward_gradian=0):
    img = np.zeros((len(ANGLES), N_BINS), dtype=np.uint8)

    centre_col = int(range_m / METRES_PER_BIN)
    half_cols = max(1, int((radial_m / 2) / METRES_PER_BIN))

    centre_grad = (bearing_deg / 0.9) + forward_gradian
    centre_row = int((centre_grad % 400) / STEP_GRAD)
    half_rows = max(1, int((angular_deg / 0.9) / STEP_GRAD / 2))

    r0, r1 = centre_row - half_rows, centre_row + half_rows + 1
    c0, c1 = centre_col - half_cols, centre_col + half_cols + 1
    img[max(0, r0):r1, max(0, c0):c1] = value

    return Sweep(img, ANGLES, METRES_PER_BIN, forward_gradian)


def check(name, got, want, tol):
    ok = abs(got - want) <= tol
    print(f"{'PASS' if ok else 'FAIL'}  {name}: got {got:.3f}, want {want:.3f} (+/-{tol})")
    return ok


results = []

# --- range and bearing recovery, target dead ahead -------------------------
# forward_gradian=200 puts "straight ahead" in the MIDDLE of the image rather
# than at row 0. At row 0 the blob is clipped by the top edge, so blur and
# close can only grow it downwards and the centroid drifts - which is a real
# effect at the seam of a full sweep, not just a test artifact. Worth knowing:
# a target sitting exactly on the wrap-around point of a 360 sweep will have
# its bearing pulled slightly off.
sw = make_sweep(range_m=4.0, bearing_deg=0.0, forward_gradian=200)
dets = detect(sw, profile=None)
print(f"\ndead ahead at 4 m -> {len(dets)} detection(s)")
assert dets, "planted a bright patch and detected nothing"
d = dets[0]
results.append(check("range", d.range_m, 4.0, 0.15))
results.append(check("bearing", d.bearing_deg, 0.0, 2.0))

# --- bearing recovery, target off to starboard -----------------------------
sw = make_sweep(range_m=6.0, bearing_deg=30.0)
d = detect(sw, profile=None)[0]
print(f"\n30 deg starboard at 6 m")
results.append(check("range", d.range_m, 6.0, 0.15))
results.append(check("bearing", d.bearing_deg, 30.0, 2.0))

# --- bearing recovery, target to port (negative) ---------------------------
sw = make_sweep(range_m=6.0, bearing_deg=-45.0)
d = detect(sw, profile=None)[0]
print(f"\n45 deg port at 6 m")
results.append(check("bearing", d.bearing_deg, -45.0, 2.0))

# --- the one that matters: angular size scales with range ------------------
# Same angular width at two ranges must give DIFFERENT physical sizes. If this
# fails, the code is treating the angle axis as a fixed number of metres, which
# is the bug the old Obstacle class has.
near = detect(make_sweep(2.0, 0.0, radial_m=0.1, angular_deg=10.0), profile=None)[0]
far = detect(make_sweep(8.0, 0.0, radial_m=0.1, angular_deg=10.0), profile=None)[0]
print(f"\nsame 10 deg width, near vs far")
print(f"  at 2 m: size {near.features['size_m']:.2f} m")
print(f"  at 8 m: size {far.features['size_m']:.2f} m")
ratio = far.features["size_m"] / near.features["size_m"]
results.append(check("far/near size ratio", ratio, 4.0, 0.6))

# --- solidity of a filled rectangle should be near 1 -----------------------
sw = make_sweep(5.0, 0.0, radial_m=0.5, angular_deg=8.0)
d = detect(sw, profile=None)[0]
print(f"\nsolid rectangle")
results.append(check("solidity", d.features["solidity"], 1.0, 0.15))

# Brightness reads BELOW the planted peak of 200, and that is expected, not a
# bug. Blur and close grow the contour beyond the genuinely bright patch, so the
# mean is taken over the bright region plus a dim halo. That is the merged
# "how bright AND how filled" measure doing its job - a blob whose outline
# encloses a lot of dim space reads dimmer.
#
# It does not need to be absolutely accurate, only consistent, because the
# profile is calibrated from these same numbers off the debug viewer. Read 148
# for your target, put 148 in the profile.
#
# The known cost: small blobs collect proportionally more halo than large ones,
# so brightness is slightly coupled to size instead of being independent of it.
results.append(check("brightness (under peak, over halo)",
                     d.features["brightness"], 150.0, 40.0))

# --- scoring ---------------------------------------------------------------
profile = {
    "size_m": {"min": 0.1, "ideal": 0.7, "max": 2.0},
    "brightness": {"min": 100, "ideal": 200, "max": 255},
}
sw = make_sweep(5.0, 0.0, radial_m=0.5, angular_deg=8.0)
d = detect(sw, profile=profile)[0]
print(f"\nscored against a profile: total {d.score:.2f}, per-feature {d.scores}")
results.append(check("score is in range", 1.0 if 0.0 <= d.score <= 1.0 else -1.0, 1.0, 0.0))

# a profile the target cannot possibly match must score zero, not crash
bad = {"size_m": {"min": 50.0, "ideal": 60.0, "max": 70.0}}
d = detect(sw, profile=bad)[0]
results.append(check("impossible profile scores 0", d.score, 0.0, 1e-9))

# --- empty sweep must not crash --------------------------------------------
empty = Sweep(np.zeros((len(ANGLES), N_BINS), dtype=np.uint8), ANGLES, METRES_PER_BIN, 0)
print(f"\nblank sweep -> {len(detect(empty, profile=None))} detections (want 0)")
results.append(check("blank sweep", len(detect(empty, profile=None)), 0, 0))

# --- viewer must render without crashing -----------------------------------
sw = make_sweep(4.0, 20.0)
img = render(sw, detect(sw, profile=profile))
print(f"\nviewer rendered {img.shape[1]}x{img.shape[0]}")
results.append(check("viewer produced an image", 1.0 if img.size > 0 else -1.0, 1.0, 0.0))

print(f"\n{sum(results)}/{len(results)} checks passed")
sys.exit(0 if all(results) else 1)
