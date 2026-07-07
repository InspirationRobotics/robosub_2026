import time
import rospy

from std_msgs.msg import String
from auv.utils import deviceHelper


class intersubComMission:
    def __init__(self, robotControl=None):
        self.rc = robotControl

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

        self.sub = deviceHelper.variables.get("sub")

        self.last_msg = None
        self.received_messages = []
        self.message_received = False

    # Logs events to both ROS terminal and coms.log with timestamps.
    # Use this instead of plain rospy.loginfo when tracking modem behavior.
    def log_event(self, message, level="info"):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_msg = f"[{timestamp}] [{self.sub}] {message}"

        if level == "error":
            rospy.logerr(log_msg)
        elif level == "warn":
            rospy.logwarn(log_msg)
        else:
            rospy.loginfo(log_msg)

        with open("coms.log", "a") as file:
            file.write(log_msg + "\n")

    # Callback that runs automatically whenever a modem message is received.
    # It does not move the robot. It only stores and logs the message.
    def rec_callback(self, msg):
        self.last_msg = msg.data
        self.received_messages.append(msg.data)
        self.message_received = True

        self.log_event(f"Received modem message: {msg.data}")

    # Clears the current receive state before starting a new wait/send cycle.
    # This prevents old messages from accidentally being used as new replies.
    def reset_receive_state(self):
        self.last_msg = None
        self.message_received = False

    # Sends one modem message to the target address.
    # This is the lowest-level send function.
    def send_modem_message(self, dest_addr, message):
        try:
            self.rc.send_modem(addr=dest_addr, movement=message)
            self.log_event(f"Sent message to {dest_addr}: {message}")
            return True

        except Exception as e:
            self.log_event(f"Failed to send message to {dest_addr}: {e}", level="error")
            return False

    # Waits for any modem message until timeout.
    # Returns the received message as a string, or None if nothing is received.
    def wait_for_message(self, timeout=40):
        self.reset_receive_state()

        self.log_event(f"Waiting for any modem message for {timeout} seconds")

        start_time = time.time()

        while not rospy.is_shutdown():
            if self.message_received:
                self.log_event(f"Done waiting. Received: {self.last_msg}")
                return self.last_msg

            if time.time() - start_time >= timeout:
                self.log_event("Timeout: no modem message received", level="warn")
                return None

            time.sleep(0.1)

    # Waits for a specific expected message until timeout.
    # Returns True if the expected message is received, otherwise False.
    def wait_for_expected_message(self, expected_msg, timeout=40):
        self.reset_receive_state()

        self.log_event(f"Waiting for expected message '{expected_msg}' for {timeout} seconds")

        start_time = time.time()

        while not rospy.is_shutdown():
            if self.message_received:
                if self.last_msg == expected_msg:
                    self.log_event(f"Received expected message: {expected_msg}")
                    return True

                self.log_event(
                    f"Received different message. Expected '{expected_msg}', got '{self.last_msg}'",
                    level="warn"
                )

            if time.time() - start_time >= timeout:
                self.log_event(f"Timeout: did not receive expected message '{expected_msg}'", level="warn")
                return False

            time.sleep(0.1)

    # Sends the same message multiple times.
    # This is useful because acoustic modem messages can drop.
    # This method does not wait for a reply.
    def send_repeated(self, dest_addr, message, count=5, delay=1):
        self.log_event(f"Sending '{message}' to {dest_addr}, {count} times")

        for i in range(count):
            if rospy.is_shutdown():
                return False

            self.log_event(f"Sending repeat {i + 1}/{count}: {message}")
            success = self.send_modem_message(dest_addr, message)

            if not success:
                self.log_event(f"Send attempt {i + 1}/{count} failed", level="warn")

            time.sleep(delay)

        self.log_event(f"Finished sending repeated message: {message}")
        return True

    # Sends a message repeatedly while also listening for a reply.
    # If the expected reply is received during the send loop, it stops early.
    # This allows either AUV to stop sending once the other AUV responds.
    def send_until_reply(
        self,
        dest_addr,
        message,
        expected_reply,
        count=20,
        delay=1,
        final_timeout=40
    ):
        self.reset_receive_state()

        self.log_event(
            f"Sending '{message}' to {dest_addr} until reply '{expected_reply}' is received"
        )

        for i in range(count):
            if rospy.is_shutdown():
                return False

            if self.last_msg == expected_reply:
                self.log_event(f"Received expected reply '{expected_reply}', stopping send loop early")
                return True

            self.log_event(f"Sending attempt {i + 1}/{count}: {message}")
            self.send_modem_message(dest_addr, message)

            if self.last_msg == expected_reply:
                self.log_event(f"Received expected reply '{expected_reply}', stopping send loop early")
                return True

            time.sleep(delay)

        self.log_event(
            f"Finished {count} sends. Waiting final {final_timeout} seconds for reply '{expected_reply}'"
        )

        return self.wait_for_expected_message(expected_reply, timeout=final_timeout)

    # Receiver-side handshake.
    # Waits for a specific message, then replies with an ACK message.
    # Example: wait for START_GATE, then reply ACK_START_GATE.
    def receive_and_ack(
        self,
        expected_msg,
        reply_addr,
        reply_msg,
        wait_timeout=40,
        ack_count=3
    ):
        got_expected = self.wait_for_expected_message(expected_msg, timeout=wait_timeout)

        if got_expected:
            self.log_event(f"Received '{expected_msg}', sending ACK '{reply_msg}'")
            self.send_repeated(reply_addr, reply_msg, count=ack_count)
            return True

        self.log_event(f"Did not receive '{expected_msg}', no ACK sent", level="warn")
        return False

    # Sender-side handshake.
    # Sends a message until the expected ACK is received.
    # Example: send START_GATE until ACK_START_GATE comes back.
    def send_and_wait_for_ack(
        self,
        dest_addr,
        message,
        expected_ack,
        count=20,
        delay=1,
        final_timeout=40
    ):
        return self.send_until_reply(
            dest_addr=dest_addr,
            message=message,
            expected_reply=expected_ack,
            count=count,
            delay=delay,
            final_timeout=final_timeout
        )


if __name__ == "__main__":
    from auv.motion.robot_control import RobotControl

    rospy.init_node("intersub_coms_helper", anonymous=True)

    rospy.loginfo("Starting intersub communication helper test...")

    rc = RobotControl()
    mission = intersubComMission(robotControl=rc)

    rospy.loginfo("This file is now communication-only. Movement should stay in main mission files.")

    rc.exit()