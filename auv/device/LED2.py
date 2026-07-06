#!/usr/bin/env python3
import time
import Jetson.GPIO as GPIO
import rospy
from std_srvs.srv import Trigger, TriggerResponse

class LED:
    def __init__(self):
        self.t_pin = 31
        self.r_pin = 32
        self.enabled = True
        
        # Setup pins
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.t_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.r_pin, GPIO.OUT, initial=GPIO.LOW)
        
        rospy.init_node("LED_node", anonymous=True)
        # Service setup
        self.service_recv = rospy.Service("/auv/devices/LED/received", Trigger, self.on_recv_msg)
        self.service_send = rospy.Service("/auv/devices/LED/send", Trigger, self.on_send_msg)
        
        rospy.loginfo(f"Enabled: {self.enabled}")
        rospy.loginfo("Ready to take LED requests.")
        rospy.spin()

    def on_send_msg(self, request):
        rospy.loginfo("Received") # The spam you were looking for
        GPIO.output(self.t_pin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.t_pin, GPIO.LOW)
        return TriggerResponse(success=True, message="Modem message sent")

    def on_recv_msg(self, request):
        rospy.loginfo("Received") # The spam you were looking for
        GPIO.output(self.r_pin, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(self.r_pin, GPIO.LOW)
        return TriggerResponse(success=True, message="Modem message received")

if __name__ == "__main__":
    try:
        LED()
    except rospy.ROSInterruptException:
        pass
