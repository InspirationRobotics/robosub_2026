import rospy
import RPi.GPIO as GPIO
from std_srvs.srv import Trigger, TriggerResponse

# Setup Pin 31
LED_PIN = 31
GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_PIN, GPIO.OUT)

def handle_led_command(req):
    # This toggles the pin when the ROS service is called
    GPIO.output(LED_PIN, GPIO.HIGH)
    rospy.sleep(0.5)
    GPIO.output(LED_PIN, GPIO.LOW)
    return TriggerResponse(success=True, message="LED toggled")

def start_bridge():
    rospy.init_node('led_bridge_node')
    # This creates the service the rest of your robot expects
    rospy.Service('/auv/devices/LED/send', Trigger, handle_led_command)
    rospy.spin()

if __name__ == "__main__":
    start_bridge()
