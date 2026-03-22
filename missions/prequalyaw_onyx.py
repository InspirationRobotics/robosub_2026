import rospy
import time

from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

#time.sleep(40)
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
initial_heading = 0
return_heading = 175
config = deviceHelper.variables

# Dive down to desire depth
rc.go_to_depth(.5)
"""
rospy.loginfo("Finish initialization")
"""
try:
   # ros.loginfo("mission start")
    rc.go_to_heading(initial_heading)
    rc.activate_heading_control(True)
    # what is the purpose of creating names in the code if defined in the next line
    rc.go_forward_distance(5)
    rc.go_lateral_distance(1)

    rc.go_to_heading(return_heading)
    #turn 180 degrees
    rc.go_forward_distance(2)
    rc.go_lateral_distance(1)
    rc.go_forward_distance(2)
    #ros.log_info("mission end")
    rc.go_to_heading(initial_heading)
    rc.go_to_heading(return_heading)
    rc.go_to_heading(initial_heading)
    rc.go_to_heading(return_heading)
    rc.go_to_heading(initial_heading)
    rc.go_to_heading(return_heading)
    rc.go_to_heading(initial_heading)
    rc.go_to_heading(return_heading)
    rc.go_to_heading(initial_heading)
    rc.go_to_heading(return_heading)
    rc.go_to_heading(initial_heading)
    rc.go_to_heading(return_heading)

except Exception as e:
    rospy.logerr("ERROR OCCUR IN MISSION CONTROL")
    rospy.logerr(e)

print("[INFO] Mission run terminate")
disarm.disarm()
