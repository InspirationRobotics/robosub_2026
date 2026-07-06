import time
import RPi.GPIO as GPIO
import serial
import threading
import rospy
import atexit
from std_msgs.msg import String

# --- Hardware Constants ---
PIN_RED = 31    # Transmit Indicator
PIN_WHITE = 32  # Receive Indicator

# --- GPIO Setup ---
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN_RED, GPIO.OUT)
GPIO.setup(PIN_WHITE, GPIO.OUT)
GPIO.output(PIN_RED, GPIO.LOW)
GPIO.output(PIN_WHITE, GPIO.LOW)

def safe_exit():
    GPIO.output(PIN_RED, GPIO.LOW)
    GPIO.output(PIN_WHITE, GPIO.LOW)
    GPIO.cleanup()
    print("\nSystem shut down safely.")

atexit.register(safe_exit)

class Modem:
    def __init__(self):
        # Hardcoded port to bypass config errors
        self.__port = "/dev/ttyUSB1"
        
        rospy.loginfo(f"Connecting to modem on port: {self.__port}")
        
        self.ser = serial.Serial(
            port=self.__port, 
            baudrate=9600, 
            parity=serial.PARITY_NONE, 
            stopbits=serial.STOPBITS_ONE, 
            bytesize=serial.EIGHTBITS
        )
        
        # ROS setup
        rospy.init_node('modem_node')
        self.pub = rospy.Publisher('/auv/devices/modem/received', String, queue_size=10)
        self.sub = rospy.Subscriber("/auv/devices/modem/send", String, self.send_callback)

        self.receive_active = True
        self.recv_thread = threading.Thread(target=self.receive_loop)
        self.recv_thread.start()

    def send_callback(self, msg):
        # Blink Red LED on Send
        GPIO.output(PIN_RED, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(PIN_RED, GPIO.LOW)
        rospy.loginfo(f"Sending: {msg.data}")

    def receive_loop(self):
        while self.receive_active and not rospy.is_shutdown():
            if self.ser.in_waiting > 0:
                raw_data = self.ser.readline().decode("utf-8").strip()
                if raw_data:
                    # Blink White LED on Receive
                    GPIO.output(PIN_WHITE, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(PIN_WHITE, GPIO.LOW)
                    self.pub.publish(String(data=raw_data))
            time.sleep(0.01)

if __name__ == "__main__":
    try:
        modem = Modem()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
