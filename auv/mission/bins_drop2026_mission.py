"""
bins_drop_mission: for now this code centers on one bin then drops both markers, further implementation to navigate is to be done
once the base capability of dropping the marker into one bin is sucessful
"""

import json

import rospy
import time
from std_msgs.msg import String

from auv.device import cv_handler # For running mission-specific CV scripts
from auv.motion import robot_control # For running the motors on the sub
from auv.utils import arm, disarm

class BinsDropMission:
    cv_files = ["bins_drop2026_cv"] # CV file to run

    def __init__(self, target=None, **config):
        """
        Initialize the mission class; here should be all of the things needed in the run function. 

        Args:
            config: Mission-specific parameters to run the mission.
        """
        self.config = config
        self.data = {}  # Dictionary to store the data from the CV handler
        self.next_data = {}  # Dictionary to store the newest data from the CV handler; this data will be merged with self.data.
        self.received = False
        self.robot_control = robot_control.RobotControl()
        self.heading = self.robot_control.get_heading

        self.robot_control.set_flight_mode("STABLIZE")
        self.robot_control.set_control_mode('depth_hold')

        self.robot_control.go_to_depth(0.2) #change later for comp (0.2 is for house pool)
        time.sleep(0.1)
        
        #self.robot_control.go_to_heading(self.heading)
        self.robot_control.activate_heading_control(True)

        self.cv_handler = cv_handler.CVHandler(**self.config)
        # Initialize the CV handlers; dummys are used to input a video file instead of the camera stream as data for the CV script to run on
        for file_name in self.cv_files:
            self.cv_handler.start_cv(file_name, self.callback)

        self.cv_handler.set_target("bins_drop2026_cv", target)

        

    def callback(self, msg):
        """
        Calls back the cv_handler output -- you can have multiple callbacks for multiple CV handlers. Converts the output into JSON format.

        Args:
            msg: cv_handler output -- this will be a dictionary of motion commands and potentially the visualized frame as well as servo commands (like the torpedo launcher)
        """
        file_name = msg._connection_header["topic"].split("/")[-1] # Get the file name from the topic name
        data = json.loads(msg.data) # Convert the data to JSON
        self.next_data[file_name] = data 
        self.received = True

        # print(f"[DEBUG] Received data from {file_name}")


    def move_servo(self):
        try:
            self.robot_control.move_servo("/auv/device/dropper")
            time.sleep(0.5)
            self.robot_control.move_servo("/auv/device/dropper")
        except:
            print('no servo detected')

    def move_offset(self): #move the offset for marker dropper write once centering is good
        pass


    def run(self):
        """
        Here should be all the code required to run the mission.
        This could be a loop, a finite state machine, etc.
        """

        while not rospy.is_shutdown():
            time.sleep(0.05)
            if not self.received:
                continue

            # Merge self.next_data, which contains the updated CV handler output, with self.data, which contains the previous CV handler output.
            # self.next_data will be wiped so that it can be updated with the new CV handler output.
            for key in self.next_data.keys():
                if key in self.data.keys():
                    self.data[key].update(self.next_data[key]) # Merge the data
                else:
                    self.data[key] = self.next_data[key] # Update the keys if necessary
            self.received = False
            self.next_data = {}

            # Do something with the data.
            lateral = self.data["bins_drop2026_cv"].get("lateral", None)
            forward = self.data["bins_drop2026_cv"].get("forward", None)
            yaw = self.data["bins_drop2026_cv"].get("yaw", None)
            vertical = self.data["bins_drop2026_cv"].get("vertical", None)
            drop = self.data["bins_drop2026_cv"].get("drop", None)

            if drop == True:
                print("aligning dropper")
                self.move_offset()
                print("Bins Over, Dropping Now!")
                self.robot_control.movement()
                self.move_servo()
            else:
                #if heading is fixed change this to robot_control.movement() with inputs of yaw lateral and forward
                self.robot_control.movement(yaw = yaw, forward = forward, lateral = lateral)
                print('forward: ', forward, 'lateral: ', lateral)


    def cleanup(self):
        """
        Here should be all the code required after the run function.
        This could be cleanup, saving data, closing files, etc.
        """
        for file_name in self.cv_files:
            self.cv_handler.stop_cv(file_name)

        # Idle the robot and decrease depth for safetey
        self.robot_control.movement(lateral = 0, forward = 0, yaw = 0)
        self.robot_control.set_flight_mode("STABLIZE")
        self.robot_control.go_to_depth(0.4)
        print("[INFO] Bins drop mission terminate")


if __name__ == "__main__":
    # This is the code that will be executed if you run this file directly
    # It is here for testing purposes
    # you can run this file independently using: "python -m auv.mission.template_mission"
    # You can also import it in a mission file outside of the package
    import time
    from auv.utils import deviceHelper
    from auv.motion import robot_control
    rospy.init_node("bins_drop2026_cv", anonymous=True)

    config = deviceHelper.variables
    config.update(
        {
            # # this dummy video file will be used instead of the camera if uncommented
            # "cv_dummy": ["/somepath/thisisavideo.mp4"],
        }
    )

    # Create a mission object with arguments
    mission = BinsDropMission(**config)

    # Run the mission

    arm.arm()
    mission.run() uncomment to do mission fr, just commented for servo test
    mission.cleanup()

    disarm.disarm()
