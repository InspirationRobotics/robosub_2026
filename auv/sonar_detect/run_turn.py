"""Water test: find the target with the sonar, then turn to face it.

Turn only - no approach. Onyx has no working DVL, so it has no idea how far it
has travelled, which means driving forward would be open-loop guessing. Turning
needs only heading, which the compass gives you. So this tests the whole chain
end to end without needing position:

    sweep -> detect -> score -> pick -> bearing -> turn

Run:
    python3 -m auv.sonar_detect.run_turn --udp 127.0.0.1:9092            # dry, no thrusters
    python3 -m auv.sonar_detect.run_turn --udp 127.0.0.1:9092 --live     # actually turns

READ THIS BEFORE --live
-----------------------
RobotControl(debug=True) disables the DVL AND stops thruster output, so it is a
dry run. RobotControl(debug=False) enables both. Those are coupled in their
code, which is awkward for you: to actually drive the thrusters you must also
switch the DVL on, and Onyx's DVL is dead.

I do not know how their DVL class behaves when the hardware is absent - it may
return nothing harmlessly, or it may block on a connection that never comes.
Find that out on the bench, before you are in the water with a fin in one hand.
If it does hang, the fix is a small edit on your side to separate "use the DVL"
from "drive the thrusters".
"""
import argparse
import time

# What you are looking for. These numbers are NOT guesses to leave as they are -
# get them from run_viewer.py with the real object in the water. min and max are
# how far off ideal a candidate can be before it scores zero on that feature.
PROFILE = {
    "size_m":     {"min": 0.3, "ideal": 1.5, "max": 3.0},
    "elongation": {"min": 1.5, "ideal": 4.0, "max": 15.0},
    "solidity":   {"min": 0.4, "ideal": 0.7, "max": 1.0},
    "brightness": {"min": 70,  "ideal": 170, "max": 255},
}

MIN_SCORE = 0.25          # below this, treat it as "I did not find it"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--udp", help="host:port of pingproxy")
    p.add_argument("--device", help="serial by-id path")
    p.add_argument("--range", type=float, default=10.0)
    p.add_argument("--start", type=int, default=0)
    p.add_argument("--end", type=int, default=399)
    p.add_argument("--step", type=int, default=2)
    p.add_argument("--forward", type=int, default=0)
    p.add_argument("--threshold", type=int, default=60)
    p.add_argument("--live", action="store_true",
                   help="actually command the turn (default is print-only)")
    args = p.parse_args()

    from .detect import detect
    from .sonar import Sonar

    udp = None
    if args.udp:
        host, port = args.udp.split(":")
        udp = (host, int(port))

    sonar = Sonar(device=args.device, udp=udp, max_range_m=args.range,
                  forward_gradian=args.forward)

    rc = None
    if args.live:
        import rospy
        from auv.motion.robot_control import RobotControl
        rospy.init_node("sonar_turn_test", anonymous=True)
        rc = RobotControl()
        time.sleep(2)      # let the pose callbacks populate before trusting heading
        print(f"[INFO] live. current heading {rc.get_heading():.1f} deg")
    else:
        print("[INFO] dry run - will print the turn it would make, not command it")

    print("[INFO] sweeping...")
    sweep = sonar.sweep(args.start, args.end, args.step)
    detections = detect(sweep, profile=PROFILE, tuning={"threshold": args.threshold})

    if not detections:
        print("[INFO] nothing detected at all - is the threshold too high?")
        return

    print(f"[INFO] {len(detections)} candidates:")
    for i, d in enumerate(detections):
        parts = "  ".join(f"{k}={v:.2f}" for k, v in d.scores.items())
        print(f"   {i}: {d.range_m:5.2f} m  {d.bearing_deg:+6.1f} deg  "
              f"score {d.score:.2f}   [{parts}]")

    best = detections[0]
    if best.score < MIN_SCORE:
        # This is the "I don't know" answer, and it is deliberately distinct
        # from a bad guess. Something is out there, but nothing looks like the
        # thing we were told to find.
        print(f"[INFO] best score {best.score:.2f} is below {MIN_SCORE} - "
              f"not confident this is the target. Not turning.")
        return

    print(f"[INFO] target: {best.range_m:.2f} m away, {best.bearing_deg:+.1f} deg off the bow")

    if not args.live:
        print(f"[DRY] would turn {best.bearing_deg:+.1f} deg to face it")
        return

    heading = rc.get_heading()
    target = (heading + best.bearing_deg) % 360
    print(f"[INFO] heading {heading:.1f} -> {target:.1f}")
    rc.go_to_heading(target)
    print(f"[INFO] done. heading now {rc.get_heading():.1f}")


if __name__ == "__main__":
    main()
