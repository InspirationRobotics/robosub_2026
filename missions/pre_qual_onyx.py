import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous=True)
rc = robot_control.RobotControl()
#time.sleep(40)#sleep for 40 seconds to get ready
arm.arm()
rc.go_to_depth(0.5)
ros.log_info("Robot armed and set to depth 0.5 m")
initial_heading = 0  # CALIBRATE EACH TIME
return_heading = 180 # what is difference if i do this here or lower
config = deviceHelper.variables

try:
   # ros.loginfo("mission start")
    rc.go_to_heading(initial_heading)
    rc.activate_heading_control(True)
    rc.set_absolute_yaw(initial_heading)

    rc.go_forward_distance(4)
    rc.go_lateral_distance(1)

    rc.go_to_heading(return_heading)
    # turn 180 degrees clockwise
    rc.go_forward_distance(3)
    rc.go_lateral_distance(1)
    rc.go_forward_distance(1)

    #ros.log_info("mission end")

except Exception as e:
    rospy.logerr("ERROR OCCUR IN MISSION CONTROL")
    rospy.logerr(e)

print("[INFO] Mission run terminate")
disarm.disarm()

