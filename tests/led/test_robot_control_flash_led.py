#!/usr/bin/env python3

import time
import argparse
import rospy

from auv.motion.robot_control import RobotControl
from auv.utils import disarm


def main():
    parser = argparse.ArgumentParser(description="Test Graey receive LED through RobotControl.flash_led()")
    parser.add_argument("--count", type=int, default=5, help="Number of LED flashes")
    parser.add_argument("--delay", type=float, default=1.0, help="Delay between flashes in seconds")
    args = parser.parse_args()

    rospy.init_node("robot_control_flash_led_test", anonymous=True)

    rospy.logwarn("Creating RobotControl. This may arm the robot because RobotControl auto-arms on init.")
    rospy.logwarn("Run this only with the sub safe, thrusters disabled, or on the bench.")

    rc = None

    try:
        rc = RobotControl(debug=True)

        rospy.loginfo("Waiting for LED receive service...")
        rospy.wait_for_service("/auv/devices/LED/received", timeout=10)
        rospy.loginfo("LED receive service found.")

        for i in range(args.count):
            rospy.loginfo(f"Flashing receive LED {i + 1}/{args.count}")
            response_msg = rc.flash_led()
            rospy.loginfo(f"LED response: {response_msg}")
            time.sleep(args.delay)

        rospy.loginfo("LED flash test complete.")

    except rospy.ROSException as e:
        rospy.logerr(f"Timed out waiting for LED service: {e}")

    except Exception as e:
        rospy.logerr(f"LED test failed: {e}")

    finally:
        rospy.loginfo("Cleaning up RobotControl and disarming.")
        try:
            if rc is not None:
                rc.exit()
            else:
                disarm.disarm()
        except Exception as e:
            rospy.logwarn(f"Cleanup/disarm failed: {e}")


if __name__ == "__main__":
    main()
