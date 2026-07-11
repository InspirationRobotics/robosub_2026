"""
===========================================================
ONYX INTERSUB SEQUENCE
===========================================================

Onyx listens for each Graey mission state and sends a unique response.
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

import rospy

from auv.mission import communication_helper_revised
from auv.motion import robot_control
from auv.utils import deviceHelper, disarm


rospy.init_node("onyx_intersub_sequence", anonymous=True)

rc = robot_control.RobotControl()
comms = communication_helper_revised.intersubComMission(robotControl=rc)

config = deviceHelper.variables
eventflags = [False, False, False, False, False]

GRAEY_ADDR = "010"
operating_depth = 0.45


def require_receive_and_reply(
    expected_msg,
    reply_msg,
    wait_timeout,
    ack_count=3,
):
    """Wait for one Graey state and stop the mission if it cannot be confirmed."""
    completed = comms.receive_and_ack(
        expected_msg=expected_msg,
        reply_addr=GRAEY_ADDR,
        reply_msg=reply_msg,
        wait_timeout=wait_timeout,
        ack_count=ack_count,
        ack_delay=0.5,
    )

    if not completed:
        raise RuntimeError(
            f"Failed communication exchange: '{expected_msg}' -> '{reply_msg}'"
        )


try:
    """SETUP"""
    rc.set_flight_mode("STABILIZE")
    rc.set_control_mode("depth_hold")
    comms.log_event("Onyx initialized in STABILIZE + depth_hold")

    """DIVE TO DEPTH"""
    rc.go_to_depth(operating_depth)

    eventflags[0] = True
    comms.log_event(f"Onyx reached depth {operating_depth} m and is listening")

    """MISSION_START HANDSHAKE"""
    require_receive_and_reply(
        expected_msg="MISSION_START",
        reply_msg="ACK_MISSION_START",
        wait_timeout=120,
    )
    eventflags[1] = True
    comms.log_event("Confirmed MISSION_START to Graey")

    """GATE_START HANDSHAKE"""
    require_receive_and_reply(
        expected_msg="GATE_START",
        reply_msg="ACK_GATE_START",
        wait_timeout=120,
    )
    comms.log_event("Confirmed GATE_START to Graey")

    """GATE_FINISH / GO_HOME HANDSHAKE"""
    require_receive_and_reply(
        expected_msg="GATE_FINISH",
        reply_msg="GO_HOME",
        wait_timeout=180,
        ack_count=5,
    )
    eventflags[2] = True
    comms.log_event("Received GATE_FINISH and sent GO_HOME")

    """GOING_HOME HANDSHAKE"""
    require_receive_and_reply(
        expected_msg="GOING_HOME",
        reply_msg="ACK_GOING_HOME",
        wait_timeout=120,
    )
    eventflags[3] = True
    comms.log_event("Confirmed GOING_HOME to Graey")

    """STARTING_STYLE_POINTS HANDSHAKE"""
    require_receive_and_reply(
        expected_msg="STARTING_STYLE_POINTS",
        reply_msg="ACK_STARTING_STYLE_POINTS",
        wait_timeout=120,
    )
    comms.log_event("Confirmed STARTING_STYLE_POINTS to Graey")

    """ROLL_DONE HANDSHAKE"""
    require_receive_and_reply(
        expected_msg="ROLL_DONE",
        reply_msg="ACK_ROLL_DONE",
        wait_timeout=180,
        ack_count=5,
    )

    eventflags[4] = True
    comms.log_event("Confirmed ROLL_DONE to Graey")
    comms.log_event("Onyx intersub sequence complete")

except KeyboardInterrupt:
    rospy.logwarn("Onyx mission interrupted")

except Exception as exc:
    comms.log_event(f"Onyx mission ended early: {exc}", level="error")

finally:
    try:
        rc.movement()
    except Exception:
        pass

    disarm.disarm()
    rc.exit()