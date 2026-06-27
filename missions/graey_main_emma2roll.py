"""
To create a sequential order of missions for Graey to follow.
"""

import rospy
import time
from auv.utils import deviceHelper
from auv.mission import intersub_com_mission, gate_intersub_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
rc.go_to_depth(0.45)
rospy.loginfo("Robot armed and set to depth 0.8 m")
gate_heading = 0 # CALIBRATE EACH TIME 
return_heading = 180
config = deviceHelper.variables
eventflags = [False,False,False,False,False]



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


"""ROLL MANEUVER"""
try:
    intersubMission.do_roll()  # <-- Execute roll here
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING ROLL MANEUVER")
    rospy.logerr(e)
    

disarm.disarm()
rc.exit()


