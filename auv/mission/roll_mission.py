"""
Runs the roll mission.
"""

import time
import json

import rospy
from std_msgs.msg import String

from ..device import cv_handler # For running mission-specific CV scripts
from ..motion import robot_control # For controlling the thrusters of the sub


class RollMission:
    """
    Class to run the Roll mission
    """
    cv_files = []

    def __init__(self, **config):
        """
        Initialize the RollMission class

        Args:
            config (dict): Configuration settings to run the mission
        """
        self.config = config

        self.robot_control = robot_control.RobotControl()

        print("[INFO] Roll mission init")
    
    def do_roll(self):
        rospy.loginfo("Doing roll maneuver")
        self.robot_control.set_absolute_z(0.5)
        time.sleep(5)
        self.robot_control.set_flight_mode("ACRO")
        self.robot_control.set_control_mode("direct")
        self.robot_control.movement(roll=5)
        time.sleep(4)
        self.robot_control.movement()
        self.robot_control.set_control_mode("pid")
        time.sleep(3)
        rospy.loginfo("Roll maneuver complete")
  

    def run(self):
        """
        Run the roll mission.

        """

        print("[INFO] Rollling...")
        time.sleep(1)
        self.robot_control.go_to_depth(0.5) # Robot descends to specified depth 
        rospy.loginfo("Descending to depth: 0.5 m")
        time.sleep(1)
        self.do_roll() # Executes the roll maneuver if requested
        self.robot_control.go_to_depth(0)
        rospy.loginfo("Ascending to surface")

    def cleanup(self):
        """
        Clean up the roll mission. Idles the sub.
        """

        # Idle the robot
        self.robot_control.movement(lateral=0, forward=0, yaw=0, vertical=0)
        print("[INFO] Template mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.roll_mission"
    # You can also import it in a mission file outside of the package

    rospy.init_node("roll", anonymous=True)
    # Create a mission object with arguments
    mission = RollMission()

    # Run the mission
    mission.run()
    time.sleep(2)
    mission.cleanup()
