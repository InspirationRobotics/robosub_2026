"""
===========================================================
ONYX INTERSUB MISSION LOGIC
===========================================================

Onyx is mostly the listener.

Updated Flow:
1. Dive to operating depth
2. Wait for MISSION_START from Graey
3. Wait for GATE_START from Graey
4. Wait for GATE_FINISH from Graey
5. Send GO_HOME to Graey
6. Wait for GOING_HOME from Graey
7. Wait for STARTING_STYLE_POINTS from Graey
8. Wait for ROLL_DONE from Graey
9. End mission
===========================================================
"""

import time
import rospy

from std_msgs.msg import String
from auv.motion.robot_control import RobotControl
from auv.utils import deviceHelper, disarm


class IntersubOnyxMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.sub = deviceHelper.variables.get("sub")
        self.last_msg = None
        self.end = False

        self.GRAEY_ADDR = "010"

        self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)

    def log_event(self, message, level="info"):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_msg = f"[{timestamp}] [ONYX] {message}"

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

    def wait_for_message(self, expected_msg, timeout=120):
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

    def run(self):
        self.log_event("Starting Onyx intersub mission")

        # =========================================================
        # STEP 1: DIVE TO OPERATING DEPTH
        # =========================================================
        self.rc.set_flight_mode("STABILIZE")
        self.rc.set_control_mode("depth_hold")
        self.rc.go_to_depth(0.45)
        self.log_event("Onyx reached depth 0.45 m and is listening")

        # =========================================================
        # STEP 2: WAIT FOR GRAEY TO SAY MISSION STARTED
        # Expected message: MISSION_START
        # =========================================================
        if not self.wait_for_message("MISSION_START", timeout=120):
            self.end = True
            return

        # =========================================================
        # STEP 3: WAIT FOR GRAEY TO SAY GATE IS STARTING
        # Expected message: GATE_START
        # =========================================================
        if not self.wait_for_message("GATE_START", timeout=120):
            self.end = True
            return

        # =========================================================
        # STEP 4: WAIT FOR GRAEY TO SAY GATE IS FINISHED
        # Expected message: GATE_FINISH
        # =========================================================
        if not self.wait_for_message("GATE_FINISH", timeout=180):
            self.end = True
            return

        # =========================================================
        # STEP 5: COMMAND GRAEY TO GO HOME
        # Message: GO_HOME
        # =========================================================
        self.log_event("Graey finished gate. Sending GO_HOME.")
        self.send_modem_message(self.GRAEY_ADDR, "GO_HOME", repeat=5)

        # =========================================================
        # STEP 6: WAIT FOR GRAEY TO ACKNOWLEDGE GOING HOME
        # Expected message: GOING_HOME
        # =========================================================
        if not self.wait_for_message("GOING_HOME", timeout=120):
            self.end = True
            return

        # =========================================================
        # STEP 7: WAIT FOR GRAEY TO START STYLE POINTS
        # Expected message: STARTING_STYLE_POINTS
        # =========================================================
        if not self.wait_for_message("STARTING_STYLE_POINTS", timeout=120):
            self.end = True
            return

        # =========================================================
        # STEP 8: WAIT FOR GRAEY TO FINISH ROLL
        # Expected message: ROLL_DONE
        # =========================================================
        if not self.wait_for_message("ROLL_DONE", timeout=180):
            self.end = True
            return

        self.end = True
        self.log_event("Onyx mission complete")


if __name__ == "__main__":
    rospy.init_node("intersub_onyx_mission", anonymous=True)

    rc = RobotControl()
    mission = IntersubOnyxMission(robotControl=rc)

    try:
        mission.run()
    except KeyboardInterrupt:
        rospy.logwarn("Onyx mission interrupted")
    except Exception as e:
        rospy.logerr(f"Onyx mission error: {e}")
    finally:
        disarm.disarm()
        rc.exit()
