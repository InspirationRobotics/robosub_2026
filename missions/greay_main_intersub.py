"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
from auv.mission import poles_mission, poles_mission_preset, gate_intersub_mission, communication_helper
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
rc.go_to_depth(0.6) # we can try 0.45
rospy.loginfo("Robot armed and set to depth 0.6 m")
gate_heading = 0 # CALIBRATE EACH TIME 
return_heading = 180
config = deviceHelper.variables
eventflags = [False,False,False,False,False]


"""GATE INTERSUB MISSION"""

try:
    gateIntersub = gate_intersub_mission.GateIntersubMission(robotControl=rc)
    gateIntersub.run()  # <-- Comms only
    rospy.loginfo("FINISHED GATE INTERSUB MISSION")
except Exception as e:
    rospy.logerr("ERROR DURING GATE INTERSUB MISSION")
    rospy.logerr(e)

"""COINT TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   rc.set_absolute_yaw(gate_heading)
   rospy.loginfo("Robot heading set to gate heading")
   
   # set event flag for coin toss mission to True
   eventflags[0] = True
   
   gate_forward_distance = 3 # m
   rospy.loginfo(f"Start moving forward {gate_forward_distance} m")
   rc.go_forward_distance(gate_forward_distance)
   rospy.loginfo(f"Moved {gate_forward_distance} m")
   
   print("[INFO] GATE MISSION COMPLETE")
   # set event flag for gate mission to True
   eventflags[1] = True
except KeyboardInterrupt as e:
   rospy.logwarn("Skipping current mission")
   eventflags[0] = True
   eventflags[1] = True
except Exception as e:
   rospy.logerr("ERROR OCCUR IN GATE MISSION")
   rospy.logerr(e)
   eventflags[0] = True
   eventflags[1] = True

"""MODEMS"""

try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.run()  # <-- Comms only
    rospy.loginfo("FINISHED INTERSUB COMMUNICATION")
    eventflags[3] = True
except Exception as e:
    rospy.logerr("ERROR DURING MODEM MISSION")
    rospy.logerr(e)
    eventflags[3] = True  

"""RETURN HOME"""
try:
    rc.go_to_heading(return_heading)
    rc.go_forward_distance(2)
    rospy.loginfo("RETURN HOME COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING RETURN HOME")
    rospy.logerr(e)

"""ROLL MANEUVER"""
try:
    intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
    intersubMission.do_roll()
    time.sleep(1.5)  # <-- Execute roll here
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING ROLL MANEUVER")
    rospy.logerr(e)
    

disarm.disarm()
rc.exit()




