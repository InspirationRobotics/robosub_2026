#!/usr/bin/env python3

import rospy
import time

from auv.motion.robot_control import RobotControl

def main():
    rospy.init_node("led_test")

    rc = RobotControl()

    rospy.loginfo("Waiting for LED service...")
    rospy.sleep(2)

    for i in range(10):
        rospy.loginfo(f"Flash #{i+1}")

        try:
            response = rc.flash_led()
            rospy.loginfo(f"Service response: {response}")
        except Exception as e:
            rospy.logerr(f"LED flash failed: {e}")

        time.sleep(1)

    rospy.loginfo("LED test complete")
    rc.exit()

if __name__ == "__main__":
    main()
