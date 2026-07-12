"""
NOTE: need to make sure the class and script names match
"""

import rospy
import time
import json

from auv.mission import poles_mission_right
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

with open("./missions/A7-12.json", "r==") as file:
    data = json.load(file)

# Build a dictionary where the keys are the labels (e.g., "G1", "S1")
waypoint_dict = {wp["label"]: wp for wp in data["waypoints"]}

def navigate_to(name):
    if name in waypoint_dict:
        # Grab the specific waypoint object using its name
        target_wp = waypoint_dict[name]
        
        # Use the 'x' and 'y' keys directly
        rc.waypointNav(target_wp["x"], target_wp["y"])
        rospy.loginfo(f"Reached {name} waypoint")
    else:
        # Prevents a crash if you misspell a waypoint name
        rospy.logwarn(f"ERROR: Waypoint '{name}' not found in the JSON!")
        raise ValueError(f"Missing waypoint: {name}")

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
rc.activate_heading_control(False)
config = deviceHelper.variables
gate_side = "right"

rospy.loginfo("Finish initialization")

rc.go_to_depth(0.6)

"""GATE MISSION"""
try:
    #COIN FLIP
    rc.go_to_heading(0)
    navigate_to("S1")
    rospy.loginfo("GATE MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""SLALOM MISSION"""

try: 
    navigate_to("S1")
    rospy.loginfo("Start of poles mission...")
    slalomMission = poles_mission_right.PoleSlalomMission(rc=rc)
    slalomMission.run()
    slalomMission.cleanup()
    rospy.loginfo(" POLES MISSION COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN POLES MISSION")
    rospy.logerr(e)

"""RETURN HOME MISSION"""
try:
    rospy.loginfo("Return Home")
    navigate_to("R1")
    rospy.loginfo("RETURN HOME MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR RETURNING HOME")
    rospy.logerr(e)

print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()

#######
"""TEMPLATE MISSION"""
"""try:
    rospy.loginfo("Test Mission")
    Mission = test.test(robotControl=rc)
    Mission.run()
    rospy.loginfo("MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING MISSION")
    rospy.logerr(e)
"""
