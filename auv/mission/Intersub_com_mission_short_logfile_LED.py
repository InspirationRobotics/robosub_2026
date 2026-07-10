import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from std_srvs.srv import Trigger
from auv.utils import deviceHelper


class intersubComMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl

        # Modem ROS interfaces
        self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)

        # Sub identity comes from config/hostname through deviceHelper
        self.sub = deviceHelper.variables.get("sub")

        self.end = False
        self.roll_requested = False

        # LED service names
        self.led_send_service = "/auv/devices/LED/send"
        self.led_receive_service = "/auv/devices/LED/received"

        # Prevent repeated warning spam if LED node is not running
        self.led_warned = { "send": False,"receive": False }

    def log_event(self, message, level="info"):
        """
        Logs messages to ROS and to coms.log with a timestamp.
        This helps confirm when messages were sent, received, acknowledged,
        or timed out.
        """
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

    def flash_led(self, led_type):
        """
        Flashes the send or receive LED.

        led_type:
            "send"    -> flashes /auv/devices/LED/send
            "receive" -> flashes /auv/devices/LED/received

        If the LED service is missing or fails, the modem mission continues.
        """
        if led_type == "send":
            service_name = self.led_send_service
        elif led_type == "receive":
            service_name = self.led_receive_service
        else:
            self.log_event(f"Unknown LED type requested: {led_type}", level="warn")
            return False

        try:
            # Small timeout so a missing LED node does not freeze modem communication
            rospy.wait_for_service(service_name, timeout=0.25)

            led_client = rospy.ServiceProxy(service_name, Trigger)
            led_client()

            self.log_event(f"Flashed {led_type} LED")
            return True

        except rospy.ROSException as e:
            if not self.led_warned[led_type]:
                self.log_event(
                    f"{led_type.capitalize()} LED service not available: "
                    f"{service_name}. Continuing without LED. Error: {e}",
                    level="warn"
                )
                self.led_warned[led_type] = True
            return False

        except rospy.ServiceException as e:
            self.log_event(
                f"Failed to call {led_type} LED service {service_name}: {e}",
                level="warn"
            )
            return False

    def send_modem_message(self, dest_addr, move):
        """
        Sends a modem message through RobotControl.

        Every successful send attempt flashes the send LED and writes to coms.log.
        """
        try:
            self.rc.send_modem(addr=dest_addr, movement=move)

            # Visual indicator that this sub sent a modem message
            self.flash_led("send")

            self.log_event(
                f"Sent message to address {dest_addr} with movement: {move}"
            )

            time.sleep(1)

        except Exception as e:
            self.log_event(f"Failed to send modem message: {e}", level="error")

    def rec_callback(self, msg):
        """
        Runs automatically whenever a message is received on:
        /auv/devices/modem/received

        Every received message flashes the receive LED, then the message logic
        decides what the sub should do next.
        """
        if self.end:
            return

        # Visual indicator that this sub received a modem message
        self.flash_led("receive")

        self.log_event(f"Received message: {msg.data}")

        if self.sub == "graey":
            if msg.data == "YAW":
                self.log_event("Graey received YAW from Onyx")

                self.send_modem_message(
                    dest_addr="020",
                    move="graey received YAW"
                )

                self.log_event("Attempting to YAW")

                # Actual yaw movement can be re-enabled later
                # self.rc.go_to_heading(90)
                # self.rc.go_to_heading(180)
                # self.rc.go_to_heading(270)
                # self.rc.go_to_heading(360)

                self.log_event("Graey yaw done, repeatedly telling Onyx to YAW")

                for i in range(10):
                    if self.end or rospy.is_shutdown():
                        break

                    self.log_event(f"Graey sending YAW response {i + 1}/10 to Onyx")
                    self.send_modem_message(dest_addr="020", move="YAW")

                self.log_event(
                    "Graey finished sending YAW responses, "
                    "waiting for Onyx completion message"
                )

            elif msg.data == "Onyx has completed YAW movement":
                self.log_event("Graey received final completion message from Onyx")
                self.end = True

        elif self.sub == "onyx":
            if msg.data == "YAW":
                self.log_event("Onyx received YAW from Graey")

                self.send_modem_message(
                    dest_addr="010",
                    move="onyx received YAW"
                )

                self.log_event("Attempting to YAW")

                # Actual yaw movement can be re-enabled later
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

        else:
            self.log_event(
                f"Unknown sub identity from deviceHelper: {self.sub}",
                level="warn"
            )

    def run(self):
        current_sub = self.sub
        self.log_event(f"Starting intersub communication mission as: {current_sub}")

        self.rc.set_control_mode("depth_hold")
        self.rc.activate_heading_control(True)
        time.sleep(1)

        if current_sub == "graey":
            self.log_event("Graey listening for YAW from Onyx")

            time_counter = 0

            while not self.end and not rospy.is_shutdown():
                if time_counter >= 60:
                    self.log_event(
                        "Graey timed out waiting for Onyx completion",
                        level="warn"
                    )
                    self.end = True
                    break

                time.sleep(1)
                time_counter += 1

        elif current_sub == "onyx":
            destination_addr = "010"

            self.log_event("Onyx sending YAW messages to Graey")

            for i in range(10):
                if self.end or rospy.is_shutdown():
                    break

                self.log_event(f"Onyx sending YAW message {i + 1}/10 to Graey")
                self.send_modem_message(dest_addr=destination_addr, move="YAW")

            self.log_event(
                "Onyx switching to receiving mode, waiting for Graey to signal YAW"
            )

            time_counter = 0

            while not self.end and not rospy.is_shutdown():
                if time_counter >= 20:
                    self.log_event(
                        "Timeout, Onyx did not receive message. Ending mission.",
                        level="warn"
                    )
                    self.end = True
                    break

                time.sleep(1)
                time_counter += 1

        else:
            self.log_event(
                f"Cannot run intersub mission because sub identity is unknown: {current_sub}",
                level="error"
            )


if __name__ == "__main__":
    rospy.init_node("intersub_coms_mission", anonymous=True)

    rospy.loginfo("Starting modem LED test script...")

    rc = RobotControl()

    try:
        rc.set_control_mode("depth_hold")
        rc.go_to_depth(0.5)

        time.sleep(20)

        mission = intersubComMission(robotControl=rc)
        mission.run()

    finally:
        rc.exit()