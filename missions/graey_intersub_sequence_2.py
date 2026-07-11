"""
===========================================================
GRAEY INTERSUB SEQUENCE
===========================================================

Graey is the mission leader.
Every mission transition is confirmed by Onyx before Graey continues.
Communication is handled by communication_helper.py.

Handshake flow:
1. MISSION_START          -> ACK_MISSION_START
2. GATE_START             -> ACK_GATE_START
3. GATE_FINISH            -> GO_HOME
4. GOING_HOME             -> ACK_GOING_HOME
5. STARTING_STYLE_POINTS  -> ACK_STARTING_STYLE_POINTS
6. ROLL_DONE              -> ACK_ROLL_DONE
===========================================================
"""

import time

import rospy

from auv.mission import communication_helper_revised
from auv.motion import robot_control
from auv.utils import deviceHelper, disarm


rospy.init_node("graey_intersub_sequence", anonymous=True)

rc = robot_control.RobotControl()
comms = communication_helper_revised.intersubComMission(robotControl=rc)

config = deviceHelper.variables
eventflags = [False, False, False, False, False]

ONYX_ADDR = "020"

operating_depth = 0.45
gate_heading = 0
return_heading = 180
gate_forward_distance = 2
return_distance = 0.5


def require_ack(message, expected_ack, count=20, final_timeout=40):
    """Send one mission-state command and stop the mission if Onyx never confirms it."""
    confirmed = comms.send_and_wait_for_ack(dest_addr=ONYX_ADDR,message=message,expected_ack=expected_ack,count=count,delay=1,final_timeout=final_timeout, settle_delay=2.0)

    if not confirmed:
        raise RuntimeError(f"Onyx did not confirm '{message}' with '{expected_ack}'")


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

    except Exception as exc:
        comms.log_event(f"ERROR during heading alignment: {exc}", level="error")
        eventflags[0] = True

    """DIVE TO DEPTH"""
    rc.go_to_depth(operating_depth)
    comms.log_event(f"Graey reached depth {operating_depth} m")

    """MISSION_START HANDSHAKE"""
    require_ack("MISSION_START", "ACK_MISSION_START")
    comms.log_event("Onyx confirmed MISSION_START")

    """GATE_START HANDSHAKE"""
    require_ack("GATE_START", "ACK_GATE_START")
    comms.log_event("Onyx confirmed GATE_START")

    """GATE MISSION"""
    try:
        comms.log_event(f"Starting forward movement: {gate_forward_distance} m")
        rc.go_forward_distance(gate_forward_distance)

        eventflags[1] = True
        comms.log_event("GATE MISSION COMPLETE")

    except KeyboardInterrupt:
        rospy.logwarn("Skipping gate mission")
        eventflags[1] = True

    except Exception as exc:
        comms.log_event(f"ERROR during gate mission: {exc}", level="error")
        eventflags[1] = True
        raise

    """GATE_FINISH / GO_HOME HANDSHAKE"""
    require_ack(message="GATE_FINISH", expected_ack="GO_HOME", count=20,final_timeout=120,)
    comms.log_event("Received GO_HOME from Onyx")

    """GOING_HOME HANDSHAKE"""
    require_ack("GOING_HOME", "ACK_GOING_HOME")
    comms.log_event("Onyx confirmed GOING_HOME")

    """RETURN HOME"""
    try:
        rc.go_to_heading(return_heading)
        rc.go_forward_distance(return_distance)

        eventflags[2] = True
        comms.log_event("RETURN HOME COMPLETE")

    except Exception as exc:
        comms.log_event(f"ERROR during return home: {exc}", level="error")
        eventflags[2] = True
        raise

    """STARTING_STYLE_POINTS HANDSHAKE"""
    require_ack("STARTING_STYLE_POINTS", "ACK_STARTING_STYLE_POINTS")
    comms.log_event("Onyx confirmed STARTING_STYLE_POINTS")

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

    except Exception as exc:
        comms.log_event(f"ERROR during roll maneuver: {exc}", level="error")
        eventflags[3] = True
        raise

    """ROLL_DONE HANDSHAKE"""
    require_ack("ROLL_DONE", "ACK_ROLL_DONE", final_timeout=120)

    eventflags[4] = True
    comms.log_event("Onyx confirmed ROLL_DONE")
    comms.log_event("Graey intersub sequence complete")

except KeyboardInterrupt:
    rospy.logwarn("Graey mission interrupted")

except Exception as exc:
    comms.log_event(f"Graey mission ended early: {exc}", level="error")

finally:
    try:
        rc.movement()
    except Exception:
        pass

    disarm.disarm()
    rc.exit()