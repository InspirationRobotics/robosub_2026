"""Slalom Code (right). Searches for the first red pole, centers on it, and approaches it until within range."""

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
  model = "/home/inspiration/robosub_2026/auv/device/cams/models/Poles629Model/"




  def __init__(self, **config):
      """
      Initialize the CV class.
      Setup/attributes here will contain everything needed for the run function.
    
      Args:
          config: Dictionary that contains the configuration of the devices on the sub.
      """

      self.camera = "/auv/camera/videoOAKdRawForward"
      self.model = "/home/inspiration/robosub_2026/auv/device/cams/models/Poles629Model/"

      self.config = config
      self.cam_frame = (640, 480)
      self.framecenter_x = self.cam_frame[0]//2
      self.framecenter_y = self.cam_frame[1]//2
      self.tolerance = 60 # pixels from the center of the frame
      self.prev_offset = None
      self.prev_detect_time = time.time()
      self.start_time = time.time()
      self.undetected_count = 0 # counts the amount of frames in a row without detection
      self.detected = False
      self.end = False
      self.state = "searching"

      print("[INFO] Slalom CV init")

  def center_pole(self, offset):
      proportion = 0.0315 if offset > 0 else -0.0315 # the distance decides thruster power
      lateral = proportion * (abs(offset) ** 0.72) 
      self.state = "centering"
      return lateral

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
    
      # Utilize pole detections with at least 50% confidence
      detected_list = []
      detection_confidence = 0.50


      for det in detections:
          if "red" in det.label:
              print(f"[DEBUG] Detected {det.label} with confidence {det.confidence}")
              if det.confidence > detection_confidence and (det.xmax - det.xmin) > 25: # filter out small detections
                  detected_list.append(det)

      if len(detected_list) > 0:
          self.undetected_count = 0
          self.detected = True

          target_pole = max(detected_list, key=lambda d: (d.xmax - d.xmin)) # take the detection with the largest width (closest to the sub)
          target_x_center = (target_pole.xmin + target_pole.xmax) / 2
          offset = target_x_center - self.framecenter_x
        
          self.prev_offset = offset
          self.prev_detect_time = time.time()
        
          if abs(offset) > self.tolerance: # if the pole is outside the tolerance, strafe to center it
              lateral = self.center_pole(offset)
          else:
              self.end = True
              self.state = "finished"
      elif len(detected_list) == 0:
          self.detected = False
          self.undetected_count += 1
        
          if self.state == "searching":
              if self.undetected_count < 20:
                  print(f"Searching frame for poles.")
              else:
                lateral = -1 # strafe left when searching
          elif self.state == "centering":
             if self.undetected_count < 100: # if it's been less than 100 frames since the last detection while centering, use previous values to rerun the loop
                print(f"[WARN] Lost pole while centering. Using previous offset to continue.")
                lateral = self.center_pole(self.prev_offset)
             elif self.undetected_count > 100 and self.undetected_count < 200: # if it's been more than 100 frames since the last detection
                print(f"[WARN] Lost pole while {self.state} for more than 100 consecutive frames. Rerunning loop without further movement until another pole is detected.")
          elif self.undetected_count > 200: # if there are no detections for 200 frames
              print(f"[WARN] Lost pole while {self.state} and timeout exceeded. Ending mission.")
              self.end = True

      return {
          "state": self.state,
          "detected": self.detected,
          "prev_offset": self.prev_offset,
          "lateral": lateral,
          "forward": forward,
          "yaw": yaw,
          "vertical": vertical,
          "end": self.end
      }, frame

