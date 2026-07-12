"""
NOTE: need to make sure the class and script names match
"""

import rospy
import time
import json

print("10s before onyx initialize")
time.sleep(10)

from auv.mission import poles_mission_right, bins_approach2026_mission, octagon_approach_mission, intersub_com_mission, torpedo_approach_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

with open("./missions/waypoints_delta.json", "r") as file:
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
    navigate_to("G1")
    rospy.loginfo("GATE MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

"""INTERSUB MISSION"""
try:
    rospy.loginfo("Intersub Communication")
    Intersub = intersub_com_mission.IntersubMission(robotControl=rc)
    Intersub.run()
    rospy.loginfo("INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING INTERSUB MISSION")
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

"""INTERSUB MISSION"""
try:
    rospy.loginfo("Intersub Communication")
    Intersub = intersub_com_mission.IntersubMission(robotControl=rc)
    Intersub.run()
    rospy.loginfo("INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING INTERSUB MISSION")
    rospy.logerr(e)

"""BIN MISSION"""
try:
    navigate_to("B1")
    binApproachMission = bins_approach2026_mission.BinsApproachMission(robotControl=rc)
    binApproachMission.run()
    binApproachMission.cleanup()
    rospy.loginfo("BIN APPROACH MISSION FINISHED")

    binDropMission = bin_drop_mission.BinsDropMission(robotControl=rc)
    binDropMission.run()
    binDropMission.cleanup()

    rospy.loginfo("BIN DROP MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING BIN MISSION")
    rospy.logerr(e)

"""INTERSUB MISSION"""
try:
    rospy.loginfo("Intersub Communication")
    Intersub = intersub_com_mission.IntersubMission(robotControl=rc)
    Intersub.run()
    rospy.loginfo("INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING INTERSUB MISSION")
    rospy.logerr(e)

"""OCTAGON MISSION"""
try:
    navigate_to("O1")
    octagonMission = octagon_approach_mission.OctagonMission(robotControl=rc)
    octagonMission.run()
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
   rospy.logerr("ERROR DOING OCTAGON MISSION")
   rospy.logerr(e)

"""INTERSUB MISSION"""
try:
    rospy.loginfo("Intersub Communication")
    Intersub = intersub_com_mission.IntersubMission(robotControl=rc)
    Intersub.run()
    rospy.loginfo("INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING INTERSUB MISSION")
    rospy.logerr(e)

"""TORPEDO MISSION"""
try:
    navigate_to("T1")
    torpedoMission = torpedo_approach_mission.TorpedoApproachMission(robotControl=rc)
    torpedoMission.run()
    rc.move_servo("/auv/devices/torpedo")    
except Exception as e:
    rospy.logerr("ERROR DOING TORPEDO MISSION")
    rospy.logerr(e)

"""INTERSUB MISSION"""
try:
    rospy.loginfo("Intersub Communication")
    Intersub = intersub_com_mission.IntersubMission(robotControl=rc)
    Intersub.run()
    rospy.loginfo("INTERSUB MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING INTERSUB MISSION")
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
