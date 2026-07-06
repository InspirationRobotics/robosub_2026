"""Slalom Code (right). Searches for the first red pole, centers on it, and approaches it until within range."""

"""
Note:
set waypoint to the front (~0.5 m) of the channel near the right side of the channel

Mission logic: 
1. strafe left in searching mode until any pole is detected in the frame
2. sort through detections to find the red poles 
    a. filter out all the detections that that have a width (xmax - xmin) that is less than a certain number of pixels (need to do testing to find number) so that the sub doesn't accidentally target the 2nd row red pole when it in the first row
    b. if more than one, take the one with the largest area
3. calculate the center of the red pole's bounding box and center its x coordinates in the camera frame (tolerance of 20 pixels)
    a. set the mode to
4. once centered, move forward until the width of the red pole is a certain number of pixels in the frame(need to do testing to find number
5. when the sub is close enough to the pole (~0.5 m), end mission
    a. the main mission script will move by distance to get next to row
"""

import cv2
import time
import numpy as np 
import os

class CV:
    """
    Slalom CV class. DO NOT change the name of the class, as this will mess up all of the backend files to run the CV scripts.
    """

    # Camera to get the camera stream from.
    camera = "/auv/camera/videoOAKdRawForward" 
    model = "/home/inspiration/robosub_2026/auv/device/cams/models/Poles613Model/" 

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """

        self.camera = "/auv/camera/videoOAKdRawForward"
        self.model = "/home/inspiration/robosub_2026/auv/device/cams/models/Poles613Model/"

        self.config = config 
        self.cam_frame = (640, 480)
        self.framecenter_x = self.cam_frame[0]/2
        self.framecenter_y = self.cam_frame[1]/2
        self.tolerance = 50 # pixels from the center of the frame
        self.prev_offset = None
        self.prev_detect_time = time.time()
        self.start_time = time.time()

        self.end = False
        self.state = "searching"

        print("[INFO] Slalom CV init")

    def run(self, frame, target, detections):
        """
        Run the CV script.
        """
        forward = 0
        lateral = 0
        yaw = 0
        vertical = 0

        # Configure search state if there aren't detections
        if detections is None:
            detections = []
        
        # Utilize pole detections with at least 55% confidence
        detected_list = []
        detection_confidence = 0.55
        for det in detections:
            if "red" in det.label:
                print(f"[DEBUG] Detected {det.label} with confidence {det.confidence}")
                if det.confidence > detection_confidence and (det.xmax - det.xmin) > 20: # filter out small detections
                    detected_list.append(det)


        if len(detected_list) > 0:
            target_pole = max(detected_list, key=lambda d: (d.xmax - d.xmin)) # take the detection with the largest width (closest to the sub)
            target_x_center = (target_pole.xmin + target_pole.xmax) / 2
            offset = target_x_center - self.framecenter_x
            
            self.prev_offset = offset
            self.prev_detect_time = time.time()
            
            if abs(offset) > self.tolerance: # if the pole is outside the tolerance, strafe to center it
                lateral = 0.5 if offset > 0 else -0.5
                self.state = "centering"

            else: # if the pole is centered, move forward
                self.state = "approaching"
                if (target_pole.xmax - target_pole.xmin) < 150: # need to figure out the pixel width that corresponds to ~0.5 m, but this is a placeholder for now
                    forward = 0.25

                else: # if the pole is close enough, end the mission
                    self.state = "finished"
                    self.end = True
                    forward = 0

        elif len(detected_list) == 0:
            if self.state == "searching": 
                lateral = -0.5 # strafe left when searching

            elif self.state != "searching" and (time.time() - self.prev_detect_time) < 5: # if it's been less than 5 seconds since the last detection, rerun the loop
                    print(f"[WARN] Lost pole while {self.state}. Rerunning loop.")
                    return {"state": self.state, "prev_offset": self.prev_offset, "lateral": lateral, "forward": forward, "yaw": yaw, "vertical": vertical, "end": self.end}, frame
            
            else: # if there are no detections, strafe left to search for the pole
                print(f"[WARN] Lost pole while {self.state} and timeout exceeded. Ending mission.")
                self.end = True

        return {
            "state": self.state, 
            "prev_offset": self.prev_offset,
            "lateral": lateral, 
            "forward": forward, 
            "yaw": yaw, 
            "vertical": vertical, 
            "end": self.end
        }, frame
