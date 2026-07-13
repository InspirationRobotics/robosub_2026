"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
#from auv.mission import intersub_com_mission, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

with open("./missions/waypoints/C-713", "r") as file:
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
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
rc.go_to_depth(0.6)
rospy.loginfo("Robot armed and set to depth 0.6 m")
gate_heading = 0 # CALIBRATE EACH TIME 
return_heading = 180
config = deviceHelper.variables

"""COIN TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   navigate_to("G1")
   rospy.loginfo("Passed through gate")
   print("[INFO] GATE MISSION COMPLETE")
except Exception as e:
   rospy.logerr("ERROR OCCUR IN GATE MISSION")
   rospy.logerr(e)

"""ROLL MANEUVER"""
try:
    rc.go_to_depth(0.4)
    rospy.loginfo("Performing roll")
    rc.set_flight_mode("ACRO")
    rc.set_control_mode("direct")
    rc.movement(roll=5)
    time.sleep(4)

    rc.movement()
    time.sleep(2)

    rc.set_flight_mode("STABILIZE")
    rc.set_control_mode("depth_hold")
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING ROLL MANEUVER")
    rospy.logerr(e)

"""RETURN HOME + MAKE SPACE FOR ONYX"""
try:
    navigate_to("G2")
    rc.go_lateral_distance(1)
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING ROLL MANEUVER")
    rospy.logerr(e)

disarm.disarm()
rc.exit()

"""MODEMS"""
"""try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()  # <-- Comms only
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)
""" 
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
