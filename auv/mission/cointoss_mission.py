"""
Runs the cointoss mission.
Turns to the desired heading.
"""

import time
import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the thrusters of the sub


class CoinTossMission:
    """
    Class to run the Cointoss mission
    """
    cv_files = []

    def __init__(self, **config):
        """
        Initialize the CoinTossMission class

        Args:
            config (dict): Configuration settings to run the mission
        """
        self.config = config

        self.robot_control = robot_control.RobotControl()

        print("[INFO] Coin Toss mission init")

    def run(self, heading):
        """
        Run the cointoss mission.

        Args:
            heading: the desired heading to turn the sub to.
        """

        print("[INFO] Coin Toss")
        time.sleep(1)
        self.robot_control.set_depth(0.3)  # Robot descends to specified depth (0.65 m)
        rospy.loginfo("Descending to depth: 0.3 m")
        time.sleep(1)
        self.robot_control.go_to_heading(heading) # Robot Turns to heading of 0* based on calibration
        rospy.loginfo("Turning to heading of 0* based on calibration")
        self.robot_control.go_forward_distance(3)
        rospy.loginfo("Moving forward 3 meters")
        self.robot_control.go_to_depth(0)
        rospy.loginfo("Ascending to surface")

    def cleanup(self):
        """
        Clean up the cointoss mission. Idles the sub.
        """

        # Idle the robot
        self.robot_control.movement()
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.cointoss_mission"
    # You can also import it in a mission file outside of the package

    rospy.init_node("coin_toss", anonymous=True)
    # Create a mission object with arguments
    mission = CoinTossMission()

    # Run the mission
    mission.run(0)
    time.sleep(2)
    mission.cleanup()
