import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper


class intersubComMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)
        self.sub = deviceHelper.variables.get("sub")
        self.end = False
        self.roll_requested = False

    def log_event(self, message, level="info"):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_msg = f"[{timestamp}] {message}"

        if level == "error":
            rospy.logerr(log_msg)
        elif level == "warn":
            rospy.logwarn(log_msg)
        else:
            rospy.loginfo(log_msg)

        with open("coms.log", "a") as file:
            file.write(log_msg + "\n")

    def send_modem_message(self, dest_addr, move):
        try:
            self.rc.send_modem(addr=dest_addr, movement=move)
            self.log_event(f"Sent message to address {dest_addr} with movement: {move}")
            time.sleep(1)
        except Exception as e:
            self.log_event(f"Failed to send modem message: {e}", level="error")

    def rec_callback(self, msg):
        if self.end:
            return

        if self.sub == "graey":
            self.log_event(f"Received message: {msg.data}")

            if msg.data == "YAW":
                self.log_event("Graey received YAW from Onyx")

                self.send_modem_message(dest_addr="020", move="graey received YAW")

                self.log_event("Attempting to YAW")
                # self.rc.go_to_heading(90)
                # self.rc.go_to_heading(180)
                # self.rc.go_to_heading(270)
                # self.rc.go_to_heading(360)

                self.log_event("Graey yaw done, repeatedly telling Onyx to YAW")

                for i in range(3):
                    if self.end:
                        break

                    self.log_event(f"Graey sending YAW response {i + 1}/3 to Onyx")
                    self.send_modem_message(dest_addr="020", move="YAW")

                self.log_event(
                    "Graey finished sending YAW responses, waiting for Onyx completion message"
                )

            elif msg.data == "Onyx has completed YAW movement":
                self.log_event("Graey received final completion message from Onyx")
                self.end = True

        elif self.sub == "onyx":
            self.log_event(f"Received message: {msg.data}")

            if msg.data == "YAW":
                self.log_event("Onyx received YAW from Graey")

                self.send_modem_message(dest_addr="010", move="onyx received YAW")

                self.log_event("Attempting to YAW")
                # self.rc.go_to_heading(90)
                # self.rc.go_to_heading(180)
                # self.rc.go_to_heading(270)
                # self.rc.go_to_heading(360)

                self.log_event("Onyx has completed YAW movement")
                self.send_modem_message(
                    dest_addr="010",
                    move="Onyx has completed YAW movement"
                )

                self.end = True

    def run(self):
        current_sub = self.sub
        self.log_event(f"Starting intersub communication mission as: {current_sub}")

        self.rc.set_control_mode("depth_hold")
        self.rc.activate_heading_control(True)
        time.sleep(1)

        if current_sub == "graey":
            self.log_event("Graey listening for YAW from Onyx")

            time_counter = 0
            while not self.end:
                if time_counter >= 60:
                    self.log_event("Graey timed out waiting for Onyx completion", level="warn")
                    self.end = True
                    break

                time.sleep(1)
                time_counter += 1

        elif current_sub == "onyx":
            destination_addr = "010"

            self.log_event("Onyx sending YAW messages to Graey")

            for i in range(3): #
                if self.end:
                    break

                self.log_event(f"Onyx sending YAW message {i + 1}/3 to Graey")
                self.send_modem_message(dest_addr=destination_addr, move="YAW")

            self.log_event("Onyx switching to receiving mode, waiting for Graey to signal YAW")

            time_counter = 0
            while not self.end:
                if time_counter >= 20: #change this to 20
                    self.log_event(
                        "Timeout, Onyx did not receive message. Ending mission.",
                        level="warn"
                    )
                    self.end = True
                    break

                time.sleep(1)
                time_counter += 1


if __name__ == "__main__":
    rospy.init_node("intersub_coms_mission", anonymous=True)

    rospy.loginfo("Starting modem test script...")

    rc = RobotControl()
    rc.set_control_mode("depth_hold")
    rc.go_to_depth(0.5)

    time.sleep(20)

    mission = intersubComMission(robotControl=rc)
    mission.run()

    rc.exit()
