"""
Bins CV. Center's the sub's camera on the bin 
"""
import math
import time
import cv2 
import numpy as np
import time
import math
class CV:
    """
    Bins CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawBottom"
    model = "bins_icon" # Change later once data is collected for the platform

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """

        self.config = config
        self.shape = (640, 480)
        self.x_midpoint = self.shape[0]/2
        self.y_midpoint = self.shape[1]/2
        self.missed_frames = 0
        self.tolerance = 50
        self.prev_detected = False
        self.state = None
        self.start_time = None
        self.stage = "lateral" #we center laterally first
        self.drop = False
        self.prev_time = time.time()
        self.search_time = 0
        print("[INFO] Bins Drop CV Initialization")

    def center_x(self, target_x):
        '''takes the detected center of the bin and returns the lateral value to move the sub towards it'''
        offset = target_x - self.x_midpoint
        proportion = 0.205 if offset > 0 else -0.205 # the distance decides thruster power
        lateral = proportion * (abs(offset) ** 0.275) 
        self.state = "centering x"
        return lateral
    
    def center_y(self, target_y):
        '''takes the detected center of the bin and returns the forward value to move the sub towards it'''
        offset = target_y - self.x_midpoint
        proportion = 0.168 if offset > 0 else -0.168 # the distance decides thruster power
        forward = proportion * (abs(offset) ** 0.326) 
        self.state = "centering x"
        return forward

    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame: The frame from the camera stream
            target: This can be any type of information, for example, the object to look for
            detections: This only applies to OAK-D cameras; this is the list of detections from the ML model output

        Here should be all the code required to run the CV.
        This could be a loop, grabbing frames using ROS, etc.

        Returns:
            dictionary, visualized frame: {motion commands/flags for servos and other indication flags}, visualized frame
        """

        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        target_x = None
        target_y = None

        # Find the bin if no detection is found
        # Align with the bin and move forward (through strafe should be fine)
        # If we have lost sight of the bin, then end

        # So we do not get a NoneType error
        if detections is None:
            detections = []
        if len(detections) == 0 and self.prev_detected == False:
            self.missed_frames += 1
            self.state = "missed"

        if len(detections) == 0 and self.prev_detected == True:
            self.state = 'missed'

        if len(detections) >= 1:
            if len(detections) == 1:
                for detection in detections:
                    print(f"[DEBUG] Detection confidence: {detection.confidence}")
                    if detection.confidence > 0.65:
                        target_x = (detection.xmin + detection.xmax) / 2
                        target_y = (detection.ymin + detection.ymax) / 2
                    # Might need to do something in an elif or else here
                    else:
                        target_x = None
                
            elif len(detections) > 1:
                # Target the detection with the highest confidence. The detection targeted
                # doesn't matter since this is a localization script, not a mission script

                # Increased required confidence to 0.65 to account for multiple false positives without
                # true positive
                detection_confidence = 0.65
                for detection in detections:
                    if detection.confidence > detection_confidence:
                        target_x = (detection.xmin + detection.xmax) / 2
                        target_y = (detection.ymin + detection.ymax) / 2
                        detection_confidence = detection.confidence
        #if the target_x and target_y are within 50 pixles of the midpoints we are then close to being centered and can drop
        if abs(target_x - self.x_midpoint) <= self.tolerance and abs(target_y - self.y_midpoint) <= self.tolerance: 
            
            self.drop = True
            self.state = None
            #since we are dropping we hold the sub in place (lines redundant)
            forward = 0
            lateral = 0
            yaw = 0
            vertical = 0

        if abs(target_y - self.y_midpoint) > self.tolerance:
            self.stage = 'lateral'
        elif abs(target_x - self.x_midpoint) > self.tolerance:
            self.stage = 'forward'
        else:
            self.stage = 'lateral'

        #If no detection is made we enter missed mode, if we have a detection then we enter centering mode
        if target_x is None or target_y is None:
            self.state = "missed"
        else:
            self.prev_detected = True
            self.state = "centering"


        if self.state == "missed": 
            self.missed_frames += 1
            if self.missed_frames >= 100: #100 missed frames
                self.drop = True

        if self.state == "centering":
            if self.stage == 'lateral':
                lateral = self.center_x(target_x)
            if self.stage == 'forward':
                forward = self.center_y(target_y)
            

        # Continuously return motion commands, the state of the mission, and the visualized frame.
        motion_commands_and_state = {
            "lateral": lateral,
            "forward": forward, 
            "yaw": yaw, 
            "vertical" : vertical, 
            "drop": self.drop
            }
        return motion_commands_and_state, frame