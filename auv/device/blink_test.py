import RPi.GPIO as GPIO
import time
import atexit

# Set up the mode
GPIO.setmode(GPIO.BOARD)

# Define the pin
LED_PIN = 31

# Setup the pin
GPIO.setup(LED_PIN, GPIO.OUT)

# This function runs automatically whenever the script stops
def safe_exit():
    print("\nShutting down safely...")
    GPIO.output(LED_PIN, GPIO.LOW) # Force pin OFF
    GPIO.cleanup()                 # Reset GPIO state
    print("Cleanup complete.")

# Register the function to run on exit
atexit.register(safe_exit)

print("Blinking... Press Ctrl+C to stop.")

try:
    while True:
        GPIO.output(LED_PIN, GPIO.HIGH) # Turn ON
        time.sleep(1)
        GPIO.output(LED_PIN, GPIO.LOW)  # Turn OFF
        time.sleep(1)
except KeyboardInterrupt:
    # This specifically catches the Ctrl+C signal
    pass
