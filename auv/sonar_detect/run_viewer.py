"""Debug viewer. Sweep, detect, and show what was found with its numbers.

This is the tool you use FIRST, before writing any profile. Run it, put your
target in the water, and read its size / elongation / solidity / brightness off
the table. Those measured numbers are what you put in the profile for
run_turn.py - you are not meant to guess them.

Run:
    python3 -m auv.sonar_detect.run_viewer --udp 127.0.0.1:9092
    python3 -m auv.sonar_detect.run_viewer --device /dev/serial/by-id/usb-FTDI_FT230X...

Keys:  q quit    s save the current view to a png
"""
import argparse
import time

import cv2

from .detect import detect
from .sonar import Sonar
from .viewer import render


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--udp", help="host:port of pingproxy, e.g. 127.0.0.1:9092")
    p.add_argument("--device", help="serial by-id path, if not using pingproxy")
    p.add_argument("--range", type=float, default=10.0, help="max range, metres")
    p.add_argument("--start", type=int, default=0, help="sweep start, gradians (0-399)")
    p.add_argument("--end", type=int, default=399, help="sweep end, gradians")
    p.add_argument("--step", type=int, default=2, help="gradians between pings")
    p.add_argument("--forward", type=int, default=0,
                   help="which gradian points out the front of the sub")
    p.add_argument("--threshold", type=int, default=60, help="echo strength cutoff 0-255")
    args = p.parse_args()

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))

    sonar = Sonar(device=args.device, udp=udp, max_range_m=args.range,
                  forward_gradian=args.forward)
    print(f"[INFO] {sonar.n_samples} range bins, {sonar.metres_per_bin*100:.1f} cm per bin")

    pings = len(range(args.start, args.end + 1, args.step))
    print(f"[INFO] {pings} pings per sweep. Fewer pings or shorter range = faster sweeps.")

    tuning = {"threshold": args.threshold}

    while True:
        t0 = time.time()
        sweep = sonar.sweep(args.start, args.end, args.step)
        # No profile: measure everything, score nothing. Discovery mode.
        detections = detect(sweep, profile=None, tuning=tuning)
        elapsed = time.time() - t0

        view = render(sweep, detections)
        cv2.putText(view, f"sweep {elapsed:.1f}s   {len(detections)} blobs",
                    (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 235, 235), 1)
        cv2.imshow("sonar detect", view)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("s"):
            name = f"sweep_{int(time.time())}.png"
            cv2.imwrite(name, view)
            print(f"[INFO] saved {name}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
