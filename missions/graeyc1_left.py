"""
To create a sequential order of missions for Graey to follow.
"""
import threading
import json
import rospy
import time
from auv.utils import deviceHelper

from auv.motion import robot_control
from auv.mission import communication_helper_revised
from auv.utils import arm, disarm, deviceHelper

with open("./missions/waypoints/c1_left.json", "r") as file:
    data = json.load(file)

# Build a dictionary where the keys are the labels (e.g., "G1", "S1")
waypoint_dict = {wp["label"]: wp for wp in data["waypoints"]}

# Start the listening thread
listener_thread = threading.Thread(target=communication_helper_revised.listen_for_modem, daemon=True)
listener_thread.start()

rospy.loginfo("Modem listener thread started")
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
print("30 secs to run")
time.sleep(30)
rc.set_control_mode('depth_hold')
rc.set_flight_mode("STABILIZE")
comms = communication_helper_revised.intersubComMission(robotControl=rc)
gate_heading = 0 # CALIBRATE EACH TIME 
config = deviceHelper.variables
ONYX_ADDR = "010"

rc.go_to_depth(1)
rospy.loginfo("Robot armed and set to depth 1 m")

"""COIN TOSS + GATE MISSION"""
try:
   rc.go_to_heading(gate_heading)
   rc.activate_heading_control(True)
   comms.rec_callback()
   navigate_to("G1")
   comms.send_repeated(dest_addr=GRAEY_ADDR,message="GATE_START",count=5,delay=0.5)
   rospy.loginfo("Passed through gate")
   comms.send_repeated(dest_addr=GRAEY_ADDR,message="GATE_FINISH",count=5,delay=0.5)
   print("[INFO] GATE MISSION COMPLETE")
except Exception as e:
   rospy.logerr("ERROR OCCUR IN GATE MISSION")
   rospy.logerr(e)

"""SLALOM MISSION"""
try:
    rospy.loginfo("Slalom Mission")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="SLALOM_START",count=5,delay=0.5)
    navigate_to("S1")
    navigate_to("S2")
    navigate_to("S3")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="SLALOM_FINISH",count=5,delay=0.5)
    #navigate_to("S4")
    rospy.loginfo("SLALOM MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING SLALOM MISSION")
    rospy.logerr(e)

"""OCTAGON MISSION"""
try:
    rospy.loginfo("Octagon Mission")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="OCTAGON_START",count=5,delay=0.5)
    navigate_to("O1")
    navigate_to("O2")
    rc.go_to_depth(0.1)
    time.sleep(3)
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="OCTAGON_FINISH",count=5,delay=0.5)
    rc.go_to_depth(0.8)
    rospy.loginfo("OCTAGON MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING OCTAGON MISSION")
    rospy.logerr(e)

"""RETURN HOME MISSION"""
try:
    rospy.loginfo("Return Home Mission")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="RETURN_START",count=5,delay=0.5)
    navigate_to("R1")
    navigate_to("R2")
    comms.send_repeated(dest_addr=GRAEY_ADDR,message="RETURN_FINISH",count=5,delay=0.5)
    rospy.loginfo("RETURN HOME MISSION FINISHED")
except Exception as e:
    rospy.logerr("ERROR DOING RETURN HOME MISSION")
    rospy.logerr(e)

"""ROLL MANEUVER"""
"""
try:
    #rc.go_to_depth(0.7)
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
"""

rospy.loginfo("Ending mission sequence")
disarm.disarm()
rc.exit()

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
