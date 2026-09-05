"""Debug view: a radar display of what the sonar saw, with what the code
measured about each object beside it.

Ping Viewer shows you what is out there. It cannot show you what your code
decided about it. This shows both - the same scene, plus the numbers your
detector produced and how each one scored.

Those numbers are the point. It is how you find out what your target's features
actually are: put the real object in the water, read its row off this panel,
and that becomes the profile you hand to detect().

Nothing here is stretched or rescaled to fill space. A thing 4 m away is drawn
at the 4 m ring, and an arc that was never swept stays empty.
"""
import math

import cv2
import numpy as np

from .detect import FEATURE_NAMES

BG = (18, 17, 16)
FACE = (30, 28, 26)
INK = (238, 238, 238)
DIM = (135, 135, 135)
GRID = (62, 60, 58)
HIT = (70, 200, 255)
BEST = (90, 255, 150)
BLIND = (150, 170, 255)


def render(sweep, detections, radar_size=560, panel_w=600):
    """Radar on the left, measurements on the right."""
    radar = _draw_radar(sweep, detections, radar_size)
    panel = _draw_panel(detections, panel_w, radar_size)
    return np.hstack([radar, panel])


def _draw_radar(sweep, detections, size):
    canvas = np.full((size, size, 3), BG, dtype=np.uint8)
    centre = size // 2
    margin = 34

    if sweep.image.size == 0:
        return canvas

    n_rows, n_cols = sweep.image.shape
    max_range = sweep.col_to_range_m(n_cols)
    px_per_m = (size / 2.0 - margin) / max_range

    cv2.circle(canvas, (centre, centre), int(size / 2 - margin), FACE, -1)

    # --- the returns -------------------------------------------------------
    # Each sample is drawn as a short arc: the width of one beam step
    # angularly, a couple of pixels radially. That is the actual patch of water
    # that ping measured. Drawing a round dot instead would stretch every
    # object radially and make things look bigger than they are.
    #
    # Each arc spans exactly one beam step, so at a fine step they tile with no
    # visible gap and at a coarse step gaps open up. Those gaps are honest: the
    # sonar has no information about the water between two pings, and the wedge
    # it knows nothing about grows wider the further out you look.
    floor = max(40, int(sweep.image.mean() + 2.0 * sweep.image.std()))
    hot_rows, hot_cols = np.where(sweep.image >= floor)

    step_deg = max(sweep.step_grad * (360.0 / 400.0), 0.9)
    lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1),
                            cv2.COLORMAP_INFERNO).reshape(-1, 3)

    for row, col in zip(hot_rows, hot_cols):
        r_px = sweep.col_to_range_m(col) * px_per_m
        if r_px < 2 or r_px > (size / 2 - margin):
            continue
        bearing = sweep.row_to_bearing_deg(row)
        a0 = bearing - 90.0 - step_deg / 2.0
        colour = tuple(int(c) for c in lut[int(sweep.image[row, col])])
        cv2.ellipse(canvas, (centre, centre), (int(r_px), int(r_px)),
                    0, a0, a0 + step_deg, colour, 2)

    # --- rings, bearings, blind zone --------------------------------------
    ring = 1.0 if max_range <= 12 else 2.0
    r = ring
    while r <= max_range + 1e-6:
        cv2.circle(canvas, (centre, centre), int(r * px_per_m), GRID, 1)
        label = f"{r:.0f}m"
        cv2.putText(canvas, label, (centre + 4, centre - int(r * px_per_m) - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, DIM, 1, cv2.LINE_AA)
        r += ring

    for b in range(0, 360, 30):
        ang = math.radians(b - 90.0)
        r_out = size / 2 - margin
        x1 = int(centre + (r_out - 7) * math.cos(ang))
        y1 = int(centre + (r_out - 7) * math.sin(ang))
        x2 = int(centre + r_out * math.cos(ang))
        y2 = int(centre + r_out * math.sin(ang))
        cv2.line(canvas, (x1, y1), (x2, y2), DIM, 1)

        shown = b if b <= 180 else b - 360
        tx = int(centre + (r_out + 17) * math.cos(ang))
        ty = int(centre + (r_out + 17) * math.sin(ang))
        txt = "ahead" if b == 0 else f"{shown:+d}"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.36, 1)
        cv2.putText(canvas, txt, (tx - tw // 2, ty + th // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36,
                    INK if b == 0 else DIM, 1, cv2.LINE_AA)

    # the near-field blind zone: inside here the sonar is still hearing its own
    # transmit pulse, so nothing is trustworthy and the detector blanks it

    # --- detections --------------------------------------------------------
    for i, d in enumerate(detections):
        ang = math.radians(d.bearing_deg - 90.0)
        r_px = d.range_m * px_per_m
        x = int(centre + r_px * math.cos(ang))
        y = int(centre + r_px * math.sin(ang))
        colour = BEST if i == 0 and d.score is not None else HIT

        cv2.circle(canvas, (x, y), 13, colour, 2)
        cv2.line(canvas, (x + 9, y - 9), (x + 20, y - 18), colour, 1)
        cv2.putText(canvas, str(i), (x + 23, y - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)

    # Blind zone drawn LAST so it stays visible over the near-field glare -
    # which is exactly the bright self-noise this ring is there to explain.
    from .detect import DEFAULT_TUNING
    blind_px = int(DEFAULT_TUNING["min_range_m"] * px_per_m)
    _dashed_circle(canvas, (centre, centre), blind_px, BLIND)
    cv2.putText(canvas, "blind", (centre + blind_px + 5, centre + blind_px + 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, BLIND, 1, cv2.LINE_AA)

    cv2.line(canvas, (centre, centre - 6), (centre, centre + 6), INK, 1)
    cv2.line(canvas, (centre - 6, centre), (centre + 6, centre), INK, 1)

    return canvas


def _dashed_circle(img, centre, radius, colour, dashes=36):
    if radius < 3:
        return
    for k in range(0, dashes, 2):
        a0 = (360.0 / dashes) * k
        cv2.ellipse(img, centre, (radius, radius), 0, a0, a0 + 360.0 / dashes,
                    colour, 1)


def _draw_panel(detections, width, height, max_rows=7):
    """The measurements. This is the half you actually tune against."""
    panel = np.full((height, width, 3), BG, dtype=np.uint8)

    cv2.putText(panel, "detections", (14, 26), cv2.FONT_HERSHEY_SIMPLEX,
                0.52, INK, 1, cv2.LINE_AA)

    labels = ["#", "range", "bearing"] + [n.replace("_m", "") for n in FEATURE_NAMES] + ["score"]
    # Columns sit at fixed x positions. OpenCV's fonts are not monospaced, so
    # space-padded columns drift out of line as soon as the digits change.
    xs = [14, 44, 108, 180, 250, 330, 410, 510]
    xs = [min(x, width - 60) for x in xs]

    for x, label in zip(xs, labels):
        cv2.putText(panel, label, (x, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    DIM, 1, cv2.LINE_AA)
    cv2.line(panel, (14, 60), (width - 14, 60), GRID, 1)

    if not detections:
        cv2.putText(panel, "nothing detected", (14, 86),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, DIM, 1, cv2.LINE_AA)
    else:
        line_h = 34
        for i, d in enumerate(detections[:max_rows]):
            y = 84 + line_h * i
            colour = BEST if i == 0 and d.score is not None else INK

            cells = [str(i), f"{d.range_m:.2f}m", f"{d.bearing_deg:+.1f}"]
            cells += [f"{d.features.get(n, 0):.2f}" for n in FEATURE_NAMES]
            cells.append("-" if d.score is None else f"{d.score:.2f}")

            for x, cell in zip(xs, cells):
                cv2.putText(panel, cell, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40, colour, 1, cv2.LINE_AA)

            # Per-feature breakdown. This is the line that tells you WHICH
            # feature rejected something - without it a low score is just a
            # number you have to guess at.
            if d.scores:
                parts = "  ".join(f"{k.replace('_m','')} {v:.2f}"
                                  for k, v in d.scores.items())
                cv2.putText(panel, parts, (xs[1], y + 13),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32, DIM, 1, cv2.LINE_AA)

    _draw_legend(panel, 14, height - 108, width)
    return panel


def _draw_legend(panel, x, y, width):
    cv2.line(panel, (14, y - 14), (width - 14, y - 14), GRID, 1)
    cv2.putText(panel, "legend", (x, y + 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.42, INK, 1, cv2.LINE_AA)

    row = y + 26
    cv2.circle(panel, (x + 8, row - 4), 7, BEST, 2)
    cv2.putText(panel, "best match", (x + 24, row), cv2.FONT_HERSHEY_SIMPLEX,
                0.36, DIM, 1, cv2.LINE_AA)

    cv2.circle(panel, (x + 138, row - 4), 7, HIT, 2)
    cv2.putText(panel, "other candidate", (x + 154, row),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, DIM, 1, cv2.LINE_AA)

    cv2.ellipse(panel, (x + 310, row - 4), (7, 7), 0, 0, 120, BLIND, 1)
    cv2.ellipse(panel, (x + 310, row - 4), (7, 7), 0, 180, 300, BLIND, 1)
    cv2.putText(panel, "blind zone (self-noise)", (x + 326, row),
                cv2.FONT_HERSHEY_SIMPLEX, 0.36, DIM, 1, cv2.LINE_AA)

    # intensity ramp
    row2 = y + 56
    bar_w = 150
    lut = cv2.applyColorMap(np.arange(256, dtype=np.uint8).reshape(-1, 1),
                            cv2.COLORMAP_INFERNO).reshape(-1, 3)
    for k in range(bar_w):
        colour = tuple(int(c) for c in lut[int(k / bar_w * 255)])
        cv2.line(panel, (x + k, row2 - 9), (x + k, row2 + 1), colour, 1)
    cv2.putText(panel, "weak", (x, row2 + 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.32, DIM, 1, cv2.LINE_AA)
    cv2.putText(panel, "strong echo", (x + 96, row2 + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.32, DIM, 1, cv2.LINE_AA)

    # Always-true wording. An earlier version claimed you would see gaps between
    # arcs, which is only true at coarse angle steps - at step 2 the beams tile
    # with no gap, and a legend describing something absent is worse than none.
    cv2.putText(panel, "each arc = one beam step wide; widen --step and gaps appear",
                (x + 190, row2 + 1), cv2.FONT_HERSHEY_SIMPLEX, 0.33, DIM, 1, cv2.LINE_AA)
