"""
===========================================================
GRAEY INTERSUB MISSION LOGIC
===========================================================

Graey is the main mission leader.

Updated Flow:
1. Set heading / coin toss alignment while surfaced
2. Dive to operating depth
3. Send MISSION_START to Onyx
4. Send GATE_START
5. Drive through gate
6. Send GATE_FINISH
7. Wait for GO_HOME from Onyx
8. Send GOING_HOME
9. Back up 1 meter after GO_HOME
10. Send STARTING_STYLE_POINTS
11. Execute ACRO roll
12. Send ROLL_DONE
13. Move after roll, skipped for style-points-only test
14. End mission
===========================================================
"""

import time
import rospy

from std_msgs.msg import String
from auv.motion.robot_control import RobotControl
from auv.utils import deviceHelper, disarm


class IntersubGraeyMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.sub = deviceHelper.variables.get("sub")
        self.last_msg = None
        self.end = False

        self.ONYX_ADDR = "020"

        self.pub_modem = rospy.Publisher(
            "/auv/devices/modem/send",
            String,
            queue_size=10
        )

        self.sub_modem = rospy.Subscriber(
            "/auv/devices/modem/received",
            String,
            self.rec_callback
        )

    def log_event(self, message, level="info"):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_msg = f"[{timestamp}] [GRAEY] {message}"

        if level == "error":
            rospy.logerr(log_msg)
        elif level == "warn":
            rospy.logwarn(log_msg)
        else:
            rospy.loginfo(log_msg)

        with open("coms.log", "a") as file:
            file.write(log_msg + "\n")

    def rec_callback(self, msg):
        self.last_msg = msg.data
        self.log_event(f"Received message: {msg.data}")

    def send_modem_message(self, dest_addr, move, repeat=3, delay=1):
        for i in range(repeat):
            try:
                self.rc.send_modem(addr=dest_addr, movement=move)
                self.log_event(f"Sent {i + 1}/{repeat} to {dest_addr}: {move}")
                time.sleep(delay)
            except Exception as e:
                self.log_event(f"Failed to send modem message: {e}", level="error")

    def wait_for_message(self, expected_msg, timeout=90):
        self.log_event(f"Waiting for message: {expected_msg}")
        start_time = time.time()

        while not rospy.is_shutdown():
            if self.last_msg == expected_msg:
                self.log_event(f"Confirmed received: {expected_msg}")
                return True

            if time.time() - start_time >= timeout:
                self.log_event(f"Timeout waiting for: {expected_msg}", level="warn")
                return False

            time.sleep(0.5)

    def do_heading_alignment(self):
        # STEP 1:
        # Heading / coin toss alignment must happen before submerging.
        # No modem messages are sent until after Graey reaches depth.
        gate_heading = 0

        self.log_event("Starting heading alignment while surfaced")
        self.rc.go_to_heading(gate_heading)
        self.rc.activate_heading_control(True)
        self.rc.set_absolute_yaw(gate_heading)
        self.log_event("Heading alignment complete")

    def do_gate_mission(self):
        # STEP 5:
        # Gate mission logic.
        # Current simple implementation: drive forward 3 meters.
        gate_forward_distance = 3

        self.log_event(f"Driving through gate: {gate_forward_distance} m")
        self.rc.go_forward_distance(gate_forward_distance)
        self.log_event("Gate traversal complete")

    def go_home(self):
    # STEP 9:
    # After receiving GO_HOME from Onyx,
    # Graey backs up 1 meter before beginning style points.

        self.log_event("Starting return-home movement")

        self.rc.go_forward_distance(-1)
        time.sleep(2)
        self.log_event("Moved backward 1 meter")
        self.log_event("Return-home movement complete")

    def do_roll(self):
        # STEP 11:
        # Real ACRO roll maneuver from old intersub mission file.
        self.log_event("Starting ACRO roll maneuver")

        self.rc.set_flight_mode("ACRO")
        self.rc.set_control_mode("direct")

        self.rc.movement(roll=5)
        time.sleep(4)

        self.rc.movement()
        time.sleep(2)

        self.rc.set_flight_mode("STABILIZE")
        self.rc.set_control_mode("depth_hold")

        self.log_event("ACRO roll maneuver complete")

    def move_after_roll(self):
        # STEP 13:
        # Move-after-roll is skipped for style-points-only testing.
        # Original movement was:
        #     forward_after_roll_distance = 1
        #     self.rc.go_forward_distance(-forward_after_roll_distance)
        self.log_event("Move-after-roll skipped for style-points-only test")

    def run(self):
        # This function is the whole Graey sequence controller:
        # heading + depth + modem messages + gate + roll.
        self.log_event("Starting Graey intersub mission")

        # =========================================================
        # STEP 1: SET HEADING BEFORE SUBMERGING
        # =========================================================
        self.do_heading_alignment()

        # =========================================================
        # STEP 2: DIVE TO OPERATING DEPTH
        # No modem messages should be sent before this completes.
        # =========================================================
        self.rc.set_control_mode("depth_hold")
        self.rc.go_to_depth(0.45)
        self.log_event("Graey reached depth 0.45 m")

        # =========================================================
        # STEP 3: TELL ONYX THE MISSION HAS STARTED
        # Message: MISSION_START
        # =========================================================
        self.send_modem_message(self.ONYX_ADDR, "MISSION_START", repeat=3)

        # =========================================================
        # STEP 4: TELL ONYX GATE IS STARTING
        # Message: GATE_START
        # =========================================================
        self.send_modem_message(self.ONYX_ADDR, "GATE_START", repeat=3)

        # =========================================================
        # STEP 5: DRIVE THROUGH GATE
        # =========================================================
        self.do_gate_mission()

        # =========================================================
        # STEP 6: TELL ONYX GATE IS FINISHED
        # Message: GATE_FINISH
        # =========================================================
        self.send_modem_message(self.ONYX_ADDR, "GATE_FINISH", repeat=5)

        # =========================================================
        # STEP 7: WAIT FOR ONYX TO COMMAND GO_HOME
        # Expected message: GO_HOME
        # =========================================================
        if not self.wait_for_message("GO_HOME", timeout=120):
            self.log_event("Did not receive GO_HOME. Ending mission safely.", level="warn")
            self.end = True
            return

        # =========================================================
        # STEP 8: ACKNOWLEDGE GO_HOME
        # Message: GOING_HOME
        # =========================================================
        self.send_modem_message(self.ONYX_ADDR, "GOING_HOME", repeat=5)

        # =========================================================
        # STEP 9: RETURN HOME
        # Skipped for style-points-only test.
        # =========================================================
        self.go_home()

        # =========================================================
        # STEP 10: TELL ONYX STYLE POINTS ARE STARTING
        # Message: STARTING_STYLE_POINTS
        # =========================================================
        self.send_modem_message(self.ONYX_ADDR, "STARTING_STYLE_POINTS", repeat=3)

        # =========================================================
        # STEP 11: EXECUTE STYLE POINTS / ACRO ROLL
        # =========================================================
        self.do_roll()

        # =========================================================
        # STEP 12: TELL ONYX ROLL IS DONE
        # Message: ROLL_DONE
        # =========================================================
        self.send_modem_message(self.ONYX_ADDR, "ROLL_DONE", repeat=5)

        # =========================================================
        # STEP 13: MOVE AFTER ROLL
        # Skipped for style-points-only test.
        # =========================================================
        self.move_after_roll()

        self.end = True
        self.log_event("Graey mission complete")


if __name__ == "__main__":
    rospy.init_node("intersub_graey_mission", anonymous=True)

    rc = RobotControl()
    mission = IntersubGraeyMission(robotControl=rc)

    try:
        mission.run()
    except KeyboardInterrupt:
        rospy.logwarn("Graey mission interrupted")
    except Exception as e:
        rospy.logerr(f"Graey mission error: {e}")
    finally:
        disarm.disarm()
        rc.exit()
