import time
import Jetson.GPIO as GPIO  # Changed to Jetson.GPIO
import serial
import threading
import rospy
import atexit
from std_msgs.msg import String

# --- Hardware Constants ---
PIN_RED = 31    # Transmit
PIN_WHITE = 32  # Receive

# --- Jetson GPIO Setup ---
GPIO.setmode(GPIO.BOARD)
# Initialize the board (Crucial step for Jetson)
GPIO.setup(PIN_RED, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PIN_WHITE, GPIO.OUT, initial=GPIO.LOW)

def safe_exit():
    GPIO.cleanup()
    print("\nSystem shut down safely.")

atexit.register(safe_exit)

class Modem:
    def __init__(self):
        # Explicit port definition
        self.__port = "/dev/ttyUSB1"
        rospy.init_node('modem_node')
        
        try:
            self.ser = serial.Serial(self.__port, 9600)
        except Exception as e:
            rospy.logerr(f"Could not open port {self.__port}: {e}")
            
        self.pub = rospy.Publisher('/auv/devices/modem/received', String, queue_size=10)
        rospy.Subscriber("/auv/devices/modem/send", String, self.send_callback)
        
        threading.Thread(target=self.receive_loop, daemon=True).start()
        print("Modem Node Running.")

    def send_callback(self, msg):
        GPIO.output(PIN_RED, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(PIN_RED, GPIO.LOW)

    def receive_loop(self):
        while not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                GPIO.output(PIN_WHITE, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(PIN_WHITE, GPIO.LOW)
                data = self.ser.readline().decode().strip()
                if data:
                    self.pub.publish(String(data=data))
            time.sleep(0.01)

if __name__ == "__main__":
    try:
        Modem()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
