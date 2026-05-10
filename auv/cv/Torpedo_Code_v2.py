"""Torpedo Code v2.0"""

import cv2
import numpy as np
import time
from ultralytics import YOLO

class CV:
    """
    CV class for torpedo approach.
    """

    def __init__(self, **config):
        """
        Initialize the CV class. 
        Setup/attributes here will contain everything needed for the run function.
        
        Args:
            config: Dictionary that contains the configuration of the devices on the sub.
        """
        # Camera to get the camera stream from.
        self.camera = "/auv/camera/videoOAKdRawForward"
        self.focal_length = 286.2  # focal length of OAK-D WIDE on Onyx | Unit: pixel | https://en.wikipedia.org/wiki/Pinhole_camera_model | we resize the frame to 640*480, focal length in pixel will change
        self.model_path = config.get("torpedo_model", None)
        self.wb_strength = config.get("wb_strength", 0.5) # Strength of white balance adjustment, between 0 and 1. Higher values will result in stronger white balance correction. Default is 0.5.

        self.config = config
        self.shape = (640, 480) # resolution of the camera feed

        # Calculate the center of the frame once and store it
        self.framecenter_x = self.shape[0]/2
        self.framecenter_y = self.shape[1]/2

        # Initialize bounding box coordinates for visualization
        self.xmin = None
        self.ymin = None
        self.xmax = None
        self.ymax = None

        self.tolerance = 120 # Pixels

        self.prev_detected = False # Used to track if the target was detected in the previous frame during approach, for ending conditions
        self.state = "search"
        self.estimated_distance = None # Used to track the estimated distance to the target during approach, for ending conditions

        self.switch_back_time = None # Used to track when the sub lost the target during approach
        self.switch_count = 0 # Used to track how many times the sub has lost and regained the target
        self.end = False
        self.prev_offset = None # Stores the last known position of the bin
        self.prev_time = time.time() # Used to track how long it's been since the target was last seen during approach
        self.start_time = self.prev_time # Used to track total mission time for timeout

        print("[INFO] Torpedo approach CV init")

        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

        if self.model_path is not None:
            self.yolo = YOLO(self.model_path)  # Load the model once
            print(f"[INFO] YOLO model loaded from {self.model_path}")
        else:
            print("[WARNING] No model path provided. Detection will not work.")

    def preprocess(self, frame):
        """
        Pre-process the frame for better detection.
        
        Args:
            frame: The input frame from the camera."""
        # apply white balance
        b, g, r = cv2.split(frame.astype(np.float32)) # Split the channels
        r_mean = np.mean(r)
        g_mean = np.mean(g)
        b_mean = np.mean(b)

        r *= 1 + self.wb_strength * (g_mean / r_mean - 1)
        b *= 1 + self.wb_strength * (g_mean / b_mean - 1)

        white_balanced = cv2.merge([
            np.clip(b, 0, 255).astype(np.uint8),
            np.clip(g, 0, 255).astype(np.uint8),
            np.clip(r, 0, 255).astype(np.uint8)
        ])

        hsv = cv2.cvtColor(white_balanced, cv2.COLOR_BGR2HSV) # Convert to HSV color space for better color segmentation

        # apply CLAHE (Contrast Limited Adaptive Histogram Equalization)
        h, s, v = cv2.split(hsv)
        clahe_v = self.clahe.apply(v) # Apply CLAHE to the V channel
        clahe_hsv = cv2.merge([h, s, clahe_v])

        # convert back to BGR
        processed_frame = cv2.cvtColor(clahe_hsv, cv2.COLOR_HSV2BGR)

        return processed_frame
    
    def process_detection_offset(self, box):
        coords = box.xyxy[0].tolist()
        xmin, ymin, xmax, ymax = coords

        # Store the bounding box coordinates for later use in visualization
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax

        self.target_x = (xmin + xmax) / 2
        self.target_y = (ymin + ymax) / 2
        self.curr_offset = self.target_x - self.framecenter_x
    
        # distance estimation
        height = abs(ymax - ymin)
        target_height = 0.6096
        self.estimated_distance = (target_height * self.focal_length) / height
    
        self.prev_offset = self.curr_offset
        self.prev_detected = True
        self.state = "approach"

    def approach(self, offset):
        """Function to properly yaw and move forward
        
        Args:
            offset (int): Difference (in pixels) between frame x-midpoint and bounding box x-midpoint
        
        Returns:
            forward (float): Forward PWM (between -5 and 5)
            yaw (float): Yaw PWM (between -5 and 5)"""
        forward = 0
        yaw = 0
        
        # If the detection is centered or there is none, center it
        if offset is None or abs(offset) < self.tolerance:
            yaw = 0
            forward = 2.0
        
        # Yaw right if detection is too far right
        elif offset > 0:
            yaw = 0.8
        
        # Yaw left if detection is too far left
        elif offset < 0:
            yaw = -0.8
        
        return forward, yaw
    
    def run(self, frame, target, detections):
        """
        Run the CV script.

        Args:
            frame(cvFrame): The frame from the camera stream
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
        
        preprocessed_frame = self.preprocess(frame)
        yolo_results = self.yolo(preprocessed_frame, conf=0.55, verbose=False)[0]

        detected_list = []
        for box in yolo_results.boxes:
            # determines what the detected object is based on the class id and the model's names list
            class_id = int(box.cls) 
            label = yolo_results.names[class_id]

            if "torpedo_target" in label:
                detected_list.append(box)

        if len(detected_list) == 0: # If no detections, switch to search mode and check if we should end the mission
            print("[INFO] No detections")
            self.state = "search"
            processed_frame = preprocessed_frame
            if time.time() - self.prev_time > 20: 
                print("[INFO] Target lost for 20 seconds, ending mission")
                self.end = True

        elif len(detected_list) > 0: 
            print(f"[INFO] Detected {len(detected_list)} object(s)")
            self.state = "approach"
            self.prev_time = time.time()

            if self.estimated_distance is not None and self.estimated_distance < 0.5: # If we're close enough to the target, end the mission
                print("[INFO] Target within 0.5 meters, ending mission")
                processed_frame = preprocessed_frame
                self.end = True
            else:
                print("[INFO] Processing detection and approaching target")
                self.process_detection_offset(max(detected_list, key=lambda box: box.conf))
                forward, yaw = self.approach(self.curr_offset)
                processed_frame = cv2.rectangle(preprocessed_frame, (int(self.xmin), int(self.ymin)), (int(self.xmax), int(self.ymax)), (0, 255, 0), 2) # Draw bounding box on the frame

        return {
            "state": self.state, 
            "prev_offset": self.prev_offset,
            "lateral": lateral, 
            "forward": forward, 
            "yaw": yaw, 
            "vertical": vertical, 
            "end": self.end
        }, processed_frame

    def stop(self):
        """
        Stop the CV script and clean up any resources if necessary.
        """
        print("[INFO] Stopping CV script")
        self.end = True