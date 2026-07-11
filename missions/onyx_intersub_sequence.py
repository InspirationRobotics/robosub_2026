"""
===========================================================
ONYX INTERSUB SEQUENCE
===========================================================

Onyx is mostly the listener.

This file is only the mission order.
Communication is handled by communication_helper.py.

Flow:
1. Initialize robot
2. Dive to depth
3. Wait for MISSION_START
4. Wait for GATE_START
5. Wait for GATE_FINISH
6. Send GO_HOME
7. Wait for GOING_HOME
8. Wait for STARTING_STYLE_POINTS
9. Wait for ROLL_DONE
10. Disarm and exit
===========================================================
"""

import rospy

from auv.motion import robot_control
from auv.utils import disarm, deviceHelper
from auv.mission import communication_helper


"""INITIALIZE"""
rospy.init_node("onyx_intersub_sequence", anonymous=True)

rc = robot_control.RobotControl()
comms = communication_helper.intersubComMission(robotControl=rc)

config = deviceHelper.variables
eventflags = [False, False, False, False, False]

GRAEY_ADDR = "010"

operating_depth = 0.45  # Change to 0.45 if that is your final comp depth


try:
    """SETUP"""
    rc.set_flight_mode("STABILIZE")
    rc.set_control_mode("depth_hold")

    comms.log_event("Onyx initialized in STABILIZE + depth_hold")


    """DIVE TO DEPTH"""
    try:
        rc.go_to_depth(operating_depth)

        eventflags[0] = True
        comms.log_event(f"Onyx reached depth {operating_depth} m and is listening")

    except Exception as e:
        comms.log_event(f"ERROR during dive to depth: {e}", level="error")
        eventflags[0] = True


    """WAIT FOR MISSION_START"""
    try:
        if not comms.wait_for_expected_message("MISSION_START", timeout=120):
            comms.log_event("Did not receive MISSION_START. Ending Onyx mission.", level="warn")
            raise RuntimeError("MISSION_START not received")

        eventflags[1] = True
        comms.log_event("Confirmed MISSION_START from Graey")

    except Exception as e:
        comms.log_event(f"ERROR waiting for MISSION_START: {e}", level="error")
        raise


    """WAIT FOR GATE_START"""
    try:
        if not comms.wait_for_expected_message("GATE_START", timeout=120):
            comms.log_event("Did not receive GATE_START. Ending Onyx mission.", level="warn")
            raise RuntimeError("GATE_START not received")

        comms.log_event("Confirmed GATE_START from Graey")

    except Exception as e:
        comms.log_event(f"ERROR waiting for GATE_START: {e}", level="error")
        raise


    """WAIT FOR GATE_FINISH"""
    try:
        if not comms.wait_for_expected_message("GATE_FINISH", timeout=180):
            comms.log_event("Did not receive GATE_FINISH. Ending Onyx mission.", level="warn")
            raise RuntimeError("GATE_FINISH not received")

        eventflags[2] = True
        comms.log_event("Confirmed GATE_FINISH from Graey")

    except Exception as e:
        comms.log_event(f"ERROR waiting for GATE_FINISH: {e}", level="error")
        raise


    """SEND GO_HOME AND LISTEN FOR GOING_HOME"""
    try:
        # This is better than send_repeated + wait_for_expected_message
        # because Graey may reply while Onyx is still sending GO_HOME.
        got_reply = comms.send_until_reply(dest_addr=GRAEY_ADDR,message="GO_HOME", expected_reply="GOING_HOME", count=20, delay=1, final_timeout=40)

        if not got_reply:
            comms.log_event("Did not receive GOING_HOME from Graey. Ending Onyx mission.", level="warn")
            raise RuntimeError("GOING_HOME not received")

        eventflags[3] = True
        comms.log_event("Confirmed GOING_HOME from Graey")

    except Exception as e:
        comms.log_event(f"ERROR during GO_HOME / GOING_HOME exchange: {e}", level="error")
        raise


    """WAIT FOR STARTING_STYLE_POINTS"""
    try:
        if not comms.wait_for_expected_message("STARTING_STYLE_POINTS", timeout=120):
            comms.log_event("Did not receive STARTING_STYLE_POINTS. Ending Onyx mission.", level="warn")
            raise RuntimeError("STARTING_STYLE_POINTS not received")

        comms.log_event("Confirmed STARTING_STYLE_POINTS from Graey")

    except Exception as e:
        comms.log_event(f"ERROR waiting for STARTING_STYLE_POINTS: {e}", level="error")
        raise


    """WAIT FOR ROLL_DONE"""
    try:
        if not comms.wait_for_expected_message("ROLL_DONE", timeout=180):
            comms.log_event("Did not receive ROLL_DONE. Ending Onyx mission.", level="warn")
            raise RuntimeError("ROLL_DONE not received")

        eventflags[4] = True
        comms.log_event("Confirmed ROLL_DONE from Graey")

    except Exception as e:
        comms.log_event(f"ERROR waiting for ROLL_DONE: {e}", level="error")
        raise


    comms.log_event("Onyx intersub sequence complete")


except KeyboardInterrupt:
    rospy.logwarn("Onyx mission interrupted")

except Exception as e:
    comms.log_event(f"Onyx mission ended early: {e}", level="error")

finally:
    disarm.disarm()
    rc.exit()