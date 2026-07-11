import threading
import time
from collections import deque

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from auv.utils import deviceHelper


class intersubComMission:
    """Communication-only helper for reliable intersub mission handshakes."""

    def __init__(self, robotControl=None):
        self.rc = robotControl
        self.sub = deviceHelper.variables.get("sub")

        self.last_msg = None
        self.message_received = False

        # received_messages is a short history for debugging.
        # _message_inbox contains messages that mission logic has not consumed yet.
        self.received_messages = deque(maxlen=500)
        self._message_inbox = deque(maxlen=200)
        self._message_condition = threading.Condition()

        # RobotControl owns the modem send path. This subscriber stays active for
        # the entire mission, including while movement methods are running.
        self.sub_modem = rospy.Subscriber(
            "/auv/devices/modem/received",
            String,
            self.rec_callback,
        )

        # LED services supplied by the LED node.
        self.led_send_service_name = "/auv/devices/LED/send"
        self.led_received_service_name = "/auv/devices/LED/received"
        self.led_send_srv = rospy.ServiceProxy(self.led_send_service_name, Trigger)
        self.led_received_srv = rospy.ServiceProxy(self.led_received_service_name, Trigger)

    # Logs events to both the ROS terminal and coms.log with timestamps.
    def log_event(self, message, level="info"):
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        log_msg = f"[{timestamp}] [{self.sub}] {message}"

        if level == "error":
            rospy.logerr(log_msg)
        elif level == "warn":
            rospy.logwarn(log_msg)
        else:
            rospy.loginfo(log_msg)

        try:
            with open("coms.log", "a") as file:
                file.write(log_msg + "\n")
        except OSError as exc:
            rospy.logwarn(f"Could not write to coms.log: {exc}")

    # Flashes either the SEND LED or RECEIVED LED without stopping the mission
    # if the LED node is unavailable.
    def flash_led(self, led_type):
        try:
            if led_type == "send":
                rospy.wait_for_service(self.led_send_service_name, timeout=0.25)
                self.led_send_srv()
            elif led_type == "received":
                rospy.wait_for_service(self.led_received_service_name, timeout=0.25)
                self.led_received_srv()
            else:
                self.log_event(f"Unknown LED type requested: {led_type}", level="warn")

        except rospy.ROSException:
            self.log_event(f"LED service not available for {led_type}", level="warn")
        except rospy.ServiceException as exc:
            self.log_event(f"LED service call failed for {led_type}: {exc}", level="warn")

    # Runs automatically whenever a modem message is received.
    # The message is placed in an inbox so it cannot be erased by a later wait.
    def rec_callback(self, msg):
        message = msg.data.strip()

        with self._message_condition:
            self.last_msg = message
            self.message_received = True
            self.received_messages.append(message)
            self._message_inbox.append(message)
            self._message_condition.notify_all()

        self.log_event(f"Received modem message: {message}")
        self.flash_led("received")

    # Clears the display flags only. It intentionally does not erase the inbox.
    # Erasing the inbox could discard a reply that arrived just before a wait began.
    def reset_receive_state(self):
        self.last_msg = None
        self.message_received = False

    def _consume_expected_message(self, expected_msg):
        """Remove and return one matching message from the inbox."""
        expected_msg = expected_msg.strip()

        with self._message_condition:
            try:
                self._message_inbox.remove(expected_msg)
                return True
            except ValueError:
                return False

    def _wait_for_expected_no_log(self, expected_msg, timeout):
        """Wait for an expected message without clearing previously received data."""
        expected_msg = expected_msg.strip()
        deadline = time.monotonic() + timeout

        with self._message_condition:
            while not rospy.is_shutdown():
                try:
                    self._message_inbox.remove(expected_msg)
                    return True
                except ValueError:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return False

                    self._message_condition.wait(timeout=min(0.25, remaining))

        return False

    def discard_message(self, message):
        """Remove duplicate copies of one message from the unconsumed inbox."""
        message = message.strip()

        with self._message_condition:
            kept_messages = [item for item in self._message_inbox if item != message]
            self._message_inbox.clear()
            self._message_inbox.extend(kept_messages)

    # Sends one modem message to the target address.
    def send_modem_message(self, dest_addr, message):
        if self.rc is None:
            self.log_event("RobotControl was not provided to communication helper", level="error")
            return False

        try:
            self.rc.send_modem(addr=dest_addr, movement=message)
            self.log_event(f"Sent message to {dest_addr}: {message}")
            self.flash_led("send")
            return True

        except Exception as exc:
            self.log_event(f"Failed to send message to {dest_addr}: {exc}", level="error")
            return False

    # Waits for the next unconsumed modem message.
    def wait_for_message(self, timeout=40):
        self.log_event(f"Waiting for any modem message for {timeout} seconds")
        deadline = time.monotonic() + timeout

        with self._message_condition:
            while not rospy.is_shutdown():
                if self._message_inbox:
                    message = self._message_inbox.popleft()
                    self.log_event(f"Done waiting. Received: {message}")
                    return message

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self.log_event("Timeout: no modem message received", level="warn")
                    return None

                self._message_condition.wait(timeout=min(0.25, remaining))

        return None

    # Waits for a particular message. Messages that arrived before this method
    # began are still eligible, which prevents callback/wait race conditions.
    def wait_for_expected_message(self, expected_msg, timeout=40):
        self.log_event(
            f"Waiting for expected message '{expected_msg}' for {timeout} seconds"
        )

        received = self._wait_for_expected_no_log(expected_msg, timeout)

        if received:
            self.log_event(f"Received expected message: {expected_msg}")
            return True

        self.log_event(
            f"Timeout: did not receive expected message '{expected_msg}'",
            level="warn",
        )
        return False

    # Sends the same message multiple times without waiting for a response.
    def send_repeated(self, dest_addr, message, count=5, delay=1):
        self.log_event(f"Sending '{message}' to {dest_addr}, {count} times")
        at_least_one_success = False

        for attempt in range(1, count + 1):
            if rospy.is_shutdown():
                return False

            self.log_event(f"Sending repeat {attempt}/{count}: {message}")
            success = self.send_modem_message(dest_addr, message)
            at_least_one_success = at_least_one_success or success

            if not success:
                self.log_event(f"Send attempt {attempt}/{count} failed", level="warn")

            if attempt < count:
                rospy.sleep(delay)

        self.log_event(f"Finished sending repeated message: {message}")
        return at_least_one_success

    # Sends a command, then listens during each retry interval for the reply.
    # It stops as soon as the expected reply is consumed from the inbox.
    def send_until_reply(
        self,
        dest_addr,
        message,
        expected_reply,
        count=20,
        delay=1,
        final_timeout=40,
    ):
        self.log_event(
            f"Sending '{message}' to {dest_addr} until reply "
            f"'{expected_reply}' is received"
        )

        # Handle a reply that arrived immediately before this method was entered.
        if self._consume_expected_message(expected_reply):
            self.log_event(f"Expected reply was already received: {expected_reply}")
            return True

        for attempt in range(1, count + 1):
            if rospy.is_shutdown():
                return False

            self.log_event(f"Sending attempt {attempt}/{count}: {message}")
            self.send_modem_message(dest_addr, message)

            if self._wait_for_expected_no_log(expected_reply, timeout=delay):
                self.log_event(
                    f"Received expected reply '{expected_reply}', stopping send loop"
                )
                return True

        self.log_event(
            f"Finished {count} sends. Waiting a final {final_timeout} seconds "
            f"for reply '{expected_reply}'"
        )

        received = self._wait_for_expected_no_log(expected_reply, final_timeout)
        if received:
            self.log_event(f"Received expected reply: {expected_reply}")
            return True

        self.log_event(
            f"Did not receive expected reply '{expected_reply}'",
            level="warn",
        )
        return False

    # Receiver-side handshake: wait for a command and answer with its ACK/status.
    def receive_and_ack(
        self,
        expected_msg,
        reply_addr,
        reply_msg,
        wait_timeout=40,
        ack_count=3,
        ack_delay=0.5,
    ):
        if not self.wait_for_expected_message(expected_msg, timeout=wait_timeout):
            self.log_event(f"Did not receive '{expected_msg}', no reply sent", level="warn")
            return False

        self.log_event(f"Received '{expected_msg}', replying with '{reply_msg}'")
        sent = self.send_repeated(
            dest_addr=reply_addr,
            message=reply_msg,
            count=ack_count,
            delay=ack_delay,
        )

        if not sent:
            self.log_event(f"Failed to send reply '{reply_msg}'", level="error")
            return False

        # Remove duplicate command packets that arrived during the ACK burst.
        self.discard_message(expected_msg)
        return True

    # Sender-side handshake: retry the command until the receiver confirms it.
    def send_and_wait_for_ack(
        self,
        dest_addr,
        message,
        expected_ack,
        count=20,
        delay=1,
        final_timeout=40,
        settle_delay=2.0,
    ):
        received = self.send_until_reply(
            dest_addr=dest_addr,
            message=message,
            expected_reply=expected_ack,
            count=count,
            delay=delay,
            final_timeout=final_timeout,
        )

        if received and settle_delay > 0:
            # Gives the receiver time to finish its short ACK burst before the
            # sender begins the next acoustic transmission.
            rospy.sleep(settle_delay)
            self.discard_message(expected_ack)

        return received


if __name__ == "__main__":
    from auv.motion.robot_control import RobotControl

    rospy.init_node("intersub_coms_helper", anonymous=True)
    rospy.loginfo("Starting intersub communication helper test...")

    rc = RobotControl()
    mission = intersubComMission(robotControl=rc)

    rospy.loginfo(
        "Communication helper initialized. Movement remains in mission sequence files."
    )

    rc.exit()