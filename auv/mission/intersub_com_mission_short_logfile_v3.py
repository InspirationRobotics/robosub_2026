import time
import rospy

from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper
#this version has time stamps for all messages sent and received


class intersubComMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
        self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)
        self.sub = deviceHelper.variables.get('sub')
        self.end = False
        self.roll_requested = False

#this log event function will log all messages sent and received to a file called coms.log, with a time stamp for each message
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
        if not self.end and self.sub == "graey":
            self.log_event(f"Received message: {msg.data}")

            if msg.data == "YAW":
                self.send_modem_message(dest_addr="020", move="graey received YAW")

                self.log_event("Attempting to YAW")
                #self.rc.go_to_heading(90)
                #self.rc.go_to_heading(180)
                #self.rc.go_to_heading(270)
                #self.rc.go_to_heading(360)

                self.log_event("Graey yaw done, telling Onyx to YAW")
                self.send_modem_message(dest_addr="020", move="YAW")

            self.end = True

        if self.sub == "onyx":
            self.log_event(f"Received message: {msg.data}")
            if msg.data == "YAW":
                self.send_modem_message(dest_addr="010", move="onyx attempting to YAW")

                self.log_event("Attempting to YAW")
                #self.rc.go_to_heading(90)
                #self.rc.go_to_heading(180)
                #self.rc.go_to_heading(270)
                #self.rc.go_to_heading(360)

                self.log_event("Onyx has completed YAW movement")
                self.send_modem_message(dest_addr="010", move="Onyx has completed YAW movement")
            self.end = True

    def run(self):
        """Only handles communication, not the yaw maneuver"""
        current_sub = self.sub
        self.log_event(f"Starting intersub communication mission as: {current_sub}")
        self.rc.set_control_mode("depth_hold")
        self.rc.activate_heading_control(True)
        time.sleep(1)

        if current_sub == "graey":
            time_counter = 0

            while not self.end:
                if time_counter >= 40:
                    self.log_event("Time out, no message received", level="warn")

                    #okay so after time out, we will Graey start sending YAW messages to Onyx to see if it can reach Onyx and trigger the YAW maneuver, this is to test the communication in both directions
                    self.log_event("Graey switching to sending mode, attempting to signal to Onyx to YAW")
                    destination_addr = "020" #Graey is sending 20 messages to Onyx, if no response after 20 messages, end the mission

                    for i in range(20):
                        if self.end:
                            break

                        self.log_event(f"Graey sending YAW message {i + 1}/20 to Onyx")
                        self.send_modem_message(dest_addr=destination_addr, move="YAW") # send YAW message to Onyx when Graey times out

                time.sleep(1)
                time_counter += 1

        elif current_sub == "onyx":
            self.log_event("Attempting to signal to Graey to YAW")
            destination_addr = "010"
            self.log_event("Sending message to Graey")
            for i in range(20):
                if self.end:
                    break

                self.log_event(f"Onyx sending YAW message {i + 1}/20 to Graey")
                self.send_modem_message(dest_addr=destination_addr, move="YAW")
                time.sleep(1)

            time_counter = 0
            while not self.end:
                if time_counter >= 20:
                    self.log_event("Time out, Onyx did not receive message, ending transmission", level="warn")
                    #so now that Onyx has timed it its going to wait for Graey to start sending YAW messages to it, to see if it can trigger the YAW maneuver, this is to test the communication in both directions
                    self.log_event("Onyx switching to receiving mode, waiting for Graey to signal to YAW")
                    while not self.end:
                        time.sleep(1)

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
