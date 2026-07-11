"""
===========================================================
GRAEY INTERSUB SEQUENCE
===========================================================

Graey is the main mission leader.

This file is only the mission order.
Communication is handled by communication_helper.py.

Flow:
1. Initialize robot
2. Set heading
3. Dive to depth
4. Send MISSION_START
5. Send GATE_START
6. Drive through gate
7. Send GATE_FINISH
8. Wait for GO_HOME
9. Send GOING_HOME
10. Return home
11. Send STARTING_STYLE_POINTS
12. Roll
13. Send ROLL_DONE
14. Disarm and exit
===========================================================
"""

import time
import rospy

from auv.motion import robot_control
from auv.utils import disarm, deviceHelper
from auv.mission import communication_helper


"""INITIALIZE"""
rospy.init_node("graey_intersub_sequence", anonymous=True)

rc = robot_control.RobotControl()
comms = communication_helper.intersubComMission(robotControl=rc)

config = deviceHelper.variables
eventflags = [False, False, False, False, False]

ONYX_ADDR = "020"

operating_depth = 0.6      # Change to 0.45 if that is your final comp depth
gate_heading = 0           # Calibrate each time
return_heading = 180
gate_forward_distance = 2  # meters
return_distance = 0.5        # meters


try:
    """SETUP"""
    rc.set_control_mode("depth_hold")
    rc.set_flight_mode("STABILIZE")

    comms.log_event("Graey initialized in STABILIZE + depth_hold")


    """COIN TOSS / HEADING ALIGNMENT"""
    try:
        rc.go_to_heading(gate_heading)
        rc.activate_heading_control(True)
        rc.set_absolute_yaw(gate_heading)

        eventflags[0] = True
        comms.log_event(f"Graey heading set to gate heading: {gate_heading}")

    except KeyboardInterrupt:
        rospy.logwarn("Skipping heading alignment")
        eventflags[0] = True

    except Exception as e:
        comms.log_event(f"ERROR during heading alignment: {e}", level="error")
        eventflags[0] = True


    """DIVE TO DEPTH"""
    try:
        rc.go_to_depth(operating_depth)
        comms.log_event(f"Graey reached depth {operating_depth} m")

    except Exception as e:
        comms.log_event(f"ERROR during dive to depth: {e}", level="error")


    """SEND MISSION_START"""
    try:
        comms.send_repeated(dest_addr=ONYX_ADDR,message="MISSION_START", count=3,delay=1)

    except Exception as e:
        comms.log_event(f"ERROR sending MISSION_START: {e}", level="error")


    """SEND GATE_START"""
    try:
        comms.send_repeated(dest_addr=ONYX_ADDR,message="GATE_START", count=3,delay=1)

    except Exception as e:
        comms.log_event(f"ERROR sending GATE_START: {e}", level="error")


    """GATE MISSION"""
    try:
        comms.log_event(f"Start moving forward {gate_forward_distance} m")

        rc.go_forward_distance(gate_forward_distance)

        eventflags[1] = True
        comms.log_event("GATE MISSION COMPLETE")

    except KeyboardInterrupt:
        rospy.logwarn("Skipping gate mission")
        eventflags[1] = True

    except Exception as e:
        comms.log_event(f"ERROR during gate mission: {e}", level="error")
        eventflags[1] = True


    """SEND GATE_FINISH"""
    try:
        comms.send_repeated(dest_addr=ONYX_ADDR,message="GATE_FINISH", count=5,delay=1)

    except Exception as e:
        comms.log_event(f"ERROR sending GATE_FINISH: {e}", level="error")


    """WAIT FOR GO_HOME"""
    try:
        got_go_home = comms.wait_for_expected_message(expected_msg="GO_HOME", timeout=120)

        if not got_go_home:
            comms.log_event("Did not receive GO_HOME. Ending Graey mission safely.", level="warn")
            raise RuntimeError("GO_HOME not received")

        comms.log_event("Received GO_HOME from Onyx")

    except Exception as e:
        comms.log_event(f"ERROR waiting for GO_HOME: {e}", level="error")
        raise


    """SEND GOING_HOME"""
    try:
        comms.send_repeated(dest_addr=ONYX_ADDR,message="GOING_HOME",count=5,delay=1)

    except Exception as e:
        comms.log_event(f"ERROR sending GOING_HOME: {e}", level="error")


    """RETURN HOME"""
    try:
        rc.go_to_heading(return_heading)
        rc.go_forward_distance(return_distance)

        eventflags[2] = True
        comms.log_event("RETURN HOME COMPLETE")

    except Exception as e:
        comms.log_event(f"ERROR during return home: {e}", level="error")
        eventflags[2] = True


    """SEND STARTING_STYLE_POINTS"""
    try:
        comms.send_repeated(dest_addr=ONYX_ADDR,message="STARTING_STYLE_POINTS",count=3,delay=1)

    except Exception as e:
        comms.log_event(f"ERROR sending STARTING_STYLE_POINTS: {e}", level="error")


    """ROLL MANEUVER"""
    try:
        comms.log_event("Starting ACRO roll maneuver")

        rc.set_flight_mode("ACRO")
        rc.set_control_mode("direct")

        rc.movement(roll=5)
        time.sleep(4)

        rc.movement()
        time.sleep(2)

        rc.set_flight_mode("STABILIZE")
        rc.set_control_mode("depth_hold")

        eventflags[3] = True
        comms.log_event("ROLL MANEUVER COMPLETE")

    except Exception as e:
        comms.log_event(f"ERROR during roll maneuver: {e}", level="error")
        eventflags[3] = True


    """SEND ROLL_DONE"""
    try:
        comms.send_repeated( dest_addr=ONYX_ADDR,message="ROLL_DONE",count=5,delay=1)

        eventflags[4] = True
        comms.log_event("Graey intersub sequence complete")

    except Exception as e:
        comms.log_event(f"ERROR sending ROLL_DONE: {e}", level="error")


except KeyboardInterrupt:
    rospy.logwarn("Graey mission interrupted")

except Exception as e:
    comms.log_event(f"Graey mission ended early: {e}", level="error")

finally:
    disarm.disarm()
    rc.exit()