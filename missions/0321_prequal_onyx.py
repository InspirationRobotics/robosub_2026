#prequalifcation for onyx
import rospy
import time




from auv.mission import intersub_com_mission
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper


time.sleep(25)
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_flight_mode("STABILIZE")
rc.set_control_mode("depth_hold")
initial_heading = 0
return_heading = 180
config = deviceHelper.variables


# Dive down to desire depth
rc.go_to_depth(0.5)
"""
rospy.loginfo("Finish initialization")
"""
try:
  # ros.loginfo("mission start")
   rc.go_to_heading(initial_heading)
   rc.activate_heading_control(True)
   # what is the purpose of creating names in the code if defined in the next line
   rc.go_forward_distance(6.2)
   rc.go_lateral_distance(-1.4)
   rc.go_forward_distance(1.7)
   rc.go_lateral_distance(2)
   rc.go_forward_distance(-2)
   rc.go_lateral_distance(-1.2)
   rc.go_forward_distance(-9.3)


   #rc.go_to_heading(return_heading)
   #turn 180 degrees
   #rc.go_forward_distance(2)
   #rc.go_lateral_distance(1)
   #rc.go_forward_distance(2)
except Exception as e:
   rospy.logerr("ERROR OCCUR IN MISSION CONTROL")
   rospy.logerr(e)


#this is the intersub communication mission,
#which will wait for a message from the other sub and
#then execute the corresponding maneuver
#although, my question is when should we execute the communication to graey
#should it be during the square movement or prior to that


#try:
 #  intersubMission = intersub_com_mission.intersubComMission(robotControl=rc)
  # intersubMission.run_mission()
#except Exception as e:
 #  rospy.logerr("ERROR OCCUR IN INTERSUB MISSION")
  # rospy.logerr(e)


print("[INFO] Mission run terminate")
disarm.disarm()
