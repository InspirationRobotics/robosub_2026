#this mission sequence is for Onyx to perform Gate, Coin flip, and intersub
"""
ONYX Mission SEQUENCE
"""
import rospy # for ROS node and logging
import time # for time sleep and delays for semi-finals

from auv.mission import communication_helper_revised
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

"""INITIALIZE"""
print("30 seconds")
time.sleep(30)

rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
rc.go_to_depth(1)
rospy.loginfo("Robot armed and set to depth 1 m")
comms = communication_helper_revised.intersubComMission(robotControl=rc)
gate_heading = 0 # CALIBRATE EACH TIME 
return_heading = 180
config = deviceHelper.variables
GRAEY_ADDR = "010"

#throughout the missions want to send repeated messages to Graey
"""COIN TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   rc.set_absolute_yaw(gate_heading)
   rospy.loginfo("Robot heading set to gate heading")
   
   rospy.loginfo(f"Start moving forward")
   comms.send_repeated(dest_addr=GRAEY_ADDR,message="GATE_START",count=5,delay=0.5)
   rc.movement(forward=2)
   time.sleep(17)
   rc.movement()
   rospy.loginfo("[INFO] GATE MISSION COMPLETE")

   #send message to Graey that gate mission is complete
   comms.send_repeated(dest_addr=GRAEY_ADDR,message="GATE_FINISH",count=5,delay=0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)

# back through the gate
try:
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="RETURN_START",count=5,delay=0.5)
    rc.movement(forward=-2)
    time.sleep(12)
    rc.movement()
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="RETURN_FINISH",count=5,delay=0.5)
    rospy.loginfo("FINSHED RETURNING HOME")
except Exception as e:
    rospy.logerr("ERROR OCCUR IN RETURNING HOME")
    rospy.logerr(e)

try:
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="ROLL_START",count=5,delay=0.5)
    rc.go_to_depth(0.7)
    rospy.loginfo("Performing roll")
    rc.set_flight_mode("ACRO")
    rc.set_control_mode("direct")
    rc.movement(roll=5)
    time.sleep(4)

    rc.movement()
    time.sleep(2)

    rc.set_flight_mode("STABILIZE")
    rc.set_control_mode("depth_hold")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="ROLL_END",count=5,delay=0.5)
    rospy.loginfo("FINSHED STYLE")
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING STYLE")
    rospy.logerr(e)

comms.send_repeated(dest_addr=GRAEY_ADDR,message="ONYX_ENDING_RUN",count=5,delay=0.5)
print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()
