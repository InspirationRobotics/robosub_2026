import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm


rospy.init_node("NavTest", anonymous=True)
rc = robot_control.RobotControl()
rc.set_control_mode("depth_hold")
rc.set_flight_mode("STABILIZE")

# Diving down
rc.go_to_depth(0.4)

# set to 0
rc.go_to_heading(0)

rospy.loginfo("Reached depth")

# move to differnet waypoints
rc.waypointNav(0, 0)
rc.waypointNav(2, 0)
rc.waypointNav(2, 2)
rc.waypointNav(0, 2)
rc.waypointNav(0, 0)

#time.sleep(15)

# Exit
rospy.loginfo("Reached the end")
disarm.disarm()
rc.exit()
