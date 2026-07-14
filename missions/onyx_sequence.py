#this mission sequence is for Onyx to perform Gate, Coin flip, Poles(slalom), torpedos, octagon missions and bins
#It will also perform the intersub communication with Graey for the intersub mission.
"""
ONYX Mission SEQUENCE
"""
from auv.mission import torpedo_approach_mission, bin_approach_mission, bin_drop_mission, octagon_approach_mission
import rospy # for ROS node and logging
import time # for time sleep and delays for semi-finals
import json #to load waypoints from json file

from auv.mission import communication_helper_revised
from auv.motion import robot_control
from auv.utils import arm, disarm, deviceHelper

with open("./missions/A7-12.json", "r") as file:
    waypoints = json.load(file)

#build dictionary of waypoints for easy access
waypoint_dict = {wp["label"]: wp for wp in waypoints["waypoints"]}

"""INITIALIZE"""
rospy.init_node("Onyx", anonymous = True)
rc = robot_control.RobotControl()
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
rc.go_to_depth(0.6)
rospy.loginfo("Robot armed and set to depth 0.6 m")
comms = communication_helper_revised.intersubComMission(robotControl=rc)
gate_heading = 0 # CALIBRATE EACH TIME 
return_heading = 180
config = deviceHelper.variables
eventflags = [False,False,False,False,False,False,False,False]
GRAEY_ADDR = "010"



#throughout the missions want to send repeated messages to Graey
"""COIN TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   rc.set_absolute_yaw(gate_heading)
   rospy.loginfo("Robot heading set to gate heading")
   
   # set event flag for coin toss mission to True
   eventflags[0] = True
   
   gate_forward_distance = 3 # m
   rospy.loginfo(f"Start moving forward {gate_forward_distance} m")
   comms.send_repeated(dest_addr=GRAEY_ADDR,message="GATE_START",count=5,delay=0.5)
   rc.go_forward_distance(gate_forward_distance)
   rospy.loginfo(f"Moved {gate_forward_distance} m")
   rospy.loginfo("[INFO] GATE MISSION COMPLETE")

   #send message to Graey that gate mission is complete
   comms.send_repeated(dest_addr=GRAEY_ADDR,message="GATE_FINISH",count=5,delay=0.5)

   # set event flag for gate mission to True
   eventflags[1] = True

   comms.send_repeated(dest_addr=GRAEY_ADDR,message="Moving on to Poles",count=5,delay=0.5)
except Exception as e:
    rospy.logerr("ERROR DOING GATE MISSION")
    rospy.logerr(e)




#slalom mission
try:
    rospy.loginfo("Start of poles mission...")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="POLES_START",count=5,delay=0.5)
    rospy.loginfo("Robot heading to set waypoint")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="Robot heading set to poles waypoint(s)",count=5,delay=0.5)

    #navigate to slalom poles
    #might need to navigate to more waypoints for the slalom mission, 
    #but for now just navigate to S1
    navigate_to("S1")
    






    #at the end of every waypoint, send a message to Graey that the waypoint is complete
    rospy.loginfo("[INFO] POLES MISSION COMPLETE")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="POLES_FINISH",count=5,delay=0.5)

    eventflags[2] = True
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="Moving on to Torpedo",count=5,delay=0.5)
except Exception as e:
    rospy.logerr("ERROR DOING POLES MISSION")
    rospy.logerr(e)

"""
#torpedo mission
try:
    rospy.loginfo("Start of torpedo mission...")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="TORPEDO_START",count=5,delay=0.5)
    rospy.loginfo("Robot heading to set waypoint")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="Robot heading set to torpedo waypoint(s)",count=5,delay=0.5)
    #navigate to torpedo
    navigate_to("T1")
    rospy.loginfo("[INFO] TORPEDO MISSION COMPLETE")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="TORPEDO_FINISH",count=5,delay=0.5)
    eventflags[3] = True
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="MOVING TO OCTAGON",count=5,delay=0.5)
except Exception as e:
    rospy.logerr("ERROR DOING TORPEDO MISSION")
    rospy.logerr(e)
"""



#octagon mission
try:
    rospy.loginfo("Start of octagon mission...")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="OCTAGON_START",count=5,delay=0.5)
    rospy.loginfo("Robot heading to set waypoint")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="Robot heading set to octagon waypoint(s)",count=5,delay=0.5)
    #navigate to octagon
    navigate_to("O1")
    rospy.loginfo("[INFO] OCTAGON MISSION COMPLETE")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="OCTAGON_FINISH",count=5,delay=0.5)






    eventflags[4] = True
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="MOVING TO BINS",count=5,delay=0.5)
except Exception as e:
    rospy.logerr("ERROR DOING OCTAGON MISSION")
    rospy.logerr(e)


""""
#bins mission
try:
    rospy.loginfo("Start of bins mission...")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="BINS_START",count=5,delay=0.5)
    rospy.loginfo("Robot heading to set waypoint")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="Robot heading set to bins waypoint(s)",count=5,delay=0.5)
    #navigate to bins
    navigate_to("B1")
    rospy.loginfo("[INFO] BINS MISSION COMPLETE")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="BINS_FINISH",count=5,delay=0.5)
    eventflags[5] = True
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="RETURNING HOME",count=5,delay=0.5)
except Exception as e:
    rospy.logerr("ERROR DOING BINS MISSION")
    rospy.logerr(e)
"""

#turn 180 degrees to return home
try:
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="RETURNING_HEADING",count=5,delay=0.5)
    rc.go_to_heading(return_heading)
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="GOING TO H1 WAYPOINT",count=5,delay=0.5)
    navigate_to("H1") #navigate to home waypoint
    rospy.loginfo("RETURN HOME COMPLETE")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="ROLL",count=5,delay=0.5)
    eventflags[6] = True
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING RETURN HOME")
    rospy.logerr(e)


#forward after roll
try:
    navigate_to("H2") #navigate to home waypoint 2
    rospy.loginfo("RETURN HOME COMPLETE")
except Exception as e:
    rospy.logerr("ERROR OCCUR DURING FORWARD AFTER ROLL")
    rospy.logerr(e)
print("[INFO] Mission run terminate")
disarm.disarm()
rc.exit()