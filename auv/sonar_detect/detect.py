"""Find objects in a sonar sweep, measure them, and score them against a target.

Three jobs kept deliberately apart:

  find_blobs()  - turn a sweep into candidate shapes. Knows nothing about what
                  you are looking for.
  measure()     - turn a shape into numbers. Also knows nothing about what you
                  are looking for.
  score()       - compare those numbers against a description of your target.
                  This is the only part that knows what a pipe is, and the
                  description is passed in, not written here.

That split is what makes this reusable. Point it at a different object by
handing it a different description - no code changes.

The caller decides what score is good enough. This module never decides. It
hands back everything it found, ranked, and you pick.
"""
import math

import cv2
import numpy as np

# Detection tuning. These control what counts as a blob at all, before any
# question of what the blob IS. Expect to retune these when the water changes.
DEFAULT_TUNING = {
    "threshold": 60,        # 0-255. Echo strength below this is not a blob.
    "blur": 5,              # smoothing kernel. Kills speckle, keeps solid patches.
    "close": 11,            # gap-filling kernel. Merges a patchy object into one blob.
    "min_range_m": 0.75,    # the near-field blind zone - the sonar hearing its own
                            # ringing. Everything closer than this is guaranteed junk.
    "max_range_m": None,    # optional far limit; None means use the sweep's own range
    "min_blob_px": 12,      # contours smaller than this are not worth measuring
}

FEATURE_NAMES = ("size_m", "elongation", "solidity", "brightness")


class Detection:
    """One candidate object, with everything known about it."""

    def __init__(self, range_m, bearing_deg, features, scores, score, contour):
        self.range_m = range_m
        self.bearing_deg = bearing_deg
        self.features = features    # raw measurements
        self.scores = scores        # per-feature 0-1, empty if no profile given
        self.score = score          # combined 0-1, or None if no profile given
        self.contour = contour

    def __repr__(self):
        s = "unscored" if self.score is None else f"{self.score:.2f}"
        return (f"Detection({self.range_m:.2f} m, {self.bearing_deg:+.1f} deg, "
                f"score={s})")


def find_blobs(sweep, tuning=None):
    """Sweep -> list of contours. The image-processing half."""
    t = dict(DEFAULT_TUNING)
    if tuning:
        t.update(tuning)

    if sweep.image.size == 0:
        return [], np.zeros_like(sweep.image)

    gray = sweep.image.copy()

    # Blank the near field. Not optional - that region is the sonar hearing
    # itself ring after the transmit pulse, and it is always bright.
    near_cols = int(t["min_range_m"] / sweep.metres_per_bin)
    gray[:, :near_cols] = 0

    if t["max_range_m"] is not None:
        far_cols = int(t["max_range_m"] / sweep.metres_per_bin)
        gray[:, far_cols:] = 0

    blurred = cv2.blur(gray, (t["blur"], t["blur"]))
    _, thresh = cv2.threshold(blurred, t["threshold"], 255, cv2.THRESH_BINARY)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (t["close"], t["close"]))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= t["min_blob_px"]]
    return contours, closed


def measure(contour, sweep):
    """Contour -> the four numbers, in real units.

    The care needed here is that the two axes of a sonar image are not the same
    kind of thing, and one of them changes scale with distance:

      Across the image (columns) is range. One column is always the same number
      of metres.

      Down the image (rows) is angle. How many METRES one row covers depends on
      how far away you are - the beam fans out. Ten rows at 2 m is a much
      smaller object than ten rows at 8 m.

    So converting a blob's pixel size into a physical size means treating the
    two axes separately and using the blob's own range for the angular one.
    Multiplying a pixel area by a single metres-per-pixel number, which is the
    obvious thing to do, gives you a quantity that is not an area in any unit.
    """
    x, y, w, h = cv2.boundingRect(contour)     # x = column/range, y = row/angle

    mean_range_m = sweep.col_to_range_m(x + w / 2.0)

    radial_m = w * sweep.metres_per_bin
    angular_rad = h * sweep.step_grad * (2 * math.pi / 400.0)
    angular_m = mean_range_m * angular_rad

    long_m = max(radial_m, angular_m)
    short_m = min(radial_m, angular_m)

    area = cv2.contourArea(contour)
    hull_area = cv2.contourArea(cv2.convexHull(contour))

    mask = np.zeros(sweep.image.shape, dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    brightness = float(cv2.mean(sweep.image, mask=mask)[0])

    return {
        "size_m": long_m,
        # how stretched it is. A pipe seen side-on is long and thin; a compact
        # blob is near 1. Which of those you want is the profile's business.
        "elongation": (long_m / short_m) if short_m > 1e-6 else 1.0,
        # how ragged the outline is: blob area over the area of a rubber band
        # stretched round it. Solid patch ~1, spiky mess much lower.
        "solidity": (area / hull_area) if hull_area > 0 else 0.0,
        # average echo strength inside the blob. Hard man-made surfaces come
        # back brighter than soft sediment; noise that barely crossed the
        # threshold comes back dim. This also does the work of "how much of the
        # region is genuinely bright", because dim gaps drag the average down.
        "brightness": brightness,
    }


def _score_one(value, spec):
    """1.0 at the ideal, tapering straight down to 0 at either edge.

    A straight taper is a choice, not a law - it is just easy to reason about
    when a score comes out low and you want to know why.
    """
    lo, hi, ideal = spec["min"], spec["max"], spec["ideal"]
    if value <= lo or value >= hi:
        return 0.0
    if value < ideal:
        return (value - lo) / (ideal - lo) if ideal > lo else 1.0
    if value > ideal:
        return (hi - value) / (hi - ideal) if hi > ideal else 1.0
    return 1.0


def score(features, profile):
    """Measured features vs a target description -> (per-feature, combined).

    Combined is the PRODUCT, not the average. A candidate has to be plausible
    on every feature you asked about; one near-zero kills it. Averaging would
    let a great brightness score paper over a completely wrong shape.

    Only features named in the profile are scored, so a profile can use two
    features or all four.
    """
    per = {}
    for name, spec in profile.items():
        if name in features:
            per[name] = _score_one(features[name], spec)

    total = 1.0
    for v in per.values():
        total *= v
    return per, (total if per else 0.0)


def detect(sweep, profile=None, tuning=None):
    """The whole pipeline. Returns Detections, best first.

    profile=None is discovery mode: every blob is measured but nothing is
    scored. That is how you find out what your target's numbers actually ARE -
    put the real object in the water, look at what comes back, and those
    measurements become your profile.
    """
    contours, _ = find_blobs(sweep, tuning)
    out = []

    for c in contours:
        feats = measure(c, sweep)
        x, y, w, h = cv2.boundingRect(c)
        range_m = sweep.col_to_range_m(x + w / 2.0)
        bearing_deg = sweep.row_to_bearing_deg(y + h / 2.0)

        if profile:
            per, total = score(feats, profile)
        else:
            per, total = {}, None

        out.append(Detection(range_m, bearing_deg, feats, per, total, c))

    if profile:
        out.sort(key=lambda d: d.score, reverse=True)
    else:
        out.sort(key=lambda d: d.range_m)
    return out
