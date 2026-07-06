import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper


rospy.init_node("Graey", anonymous = True)
rc = robot_control.RobotControl()
rospy.loginfo("Sub test starting")


rc.set_flight_mode("STABILIZE") # prevents pitch and roll movement
rc.set_control_mode("depth_hold")


depth = 0.5 # change as needed
rc.go_to_depth(depth)  # change this based on the depth of the pool
print(f"set depth to {depth}")
time.sleep(1)  # let depth hold and flight mode settle


rc.go_to_heading(0)  # set heading to 0 degrees (calibrate IMU before running)
rc.activate_heading_control(True)
print("heading set to 0")
time.sleep(6) # let heading PID settle

print("forward")
rc.movement(forward=-2)
time.sleep(6)
print("right")
rc.movement(lateral=-2)
time.sleep(6)
rc.movement(lateral=0, forward=0)

print("sleep 15") 
time.sleep(15)

rospy.loginfo("Starting flag manuever")

#rc.go_forward_distance(3)
print("Forward 3 meters")

rc.go_lateral_distance(1.5)
print("Right 0.75 meters")


#rc.go_forward_distance(-0.75)
print("Backward 0.75 meters")

rc.go_lateral_distance(1.5)
print("Left 0.75 meters")




rospy.loginfo("Starting reverse square")


rc.go_to_heading(90)
print("Yaw right 90 degrees")


for i in range(3):
   rc.go_forward_distance(0.75)
   print("Forward 0.75 meters")


   rc.go_to_heading(-90 * i)
   print("Yaw left 90 degrees")




print("Mission complete")


