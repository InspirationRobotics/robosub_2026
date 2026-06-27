"""
Build of the Template CV Script


"""
import cv2
import time
import numpy as np
import os

class CV:
    """CV class, DO NOT change the name of the class."""
    camera = "/auv/camera/videoOAKdRawBottom"
    
    def __init__(self, assignment='Blood', **config: dict): #Assigment is either 'Blood' or 'Fire' (based on gate assigment)
        # Config is a way of passing in an argument to indicate to the logic what actions to take. Take a look at 
        # buoy_test_cv.py for an example.
        self.shape = (640, 480) #(x, y)
        self.framecenter = (self.shape[0]/2, self.shape[1]/2) #(x center, y center)
        self.assignment = assignment
        # Switcher variables which can be used as needed to switch states.
        self.aligned = False
        self.detected = False

        self.config = config #includes: role assignment
        self.step = 0 # Step counter variable.

        self.end = False # End variable to denote when the mission has finished.

        # Add variables as needed below.

    # You can put detection functions to detect a specific object as needed. 
    
    def get_midpoint(self, x_min, x_max, y_min, y_max):
        x_mid = (x_min + x_max) / 2
        y_mid = (y_min + y_max) / 2
        return (x_mid, y_mid)
        ''' Gets the midpoint of the detected frame
            
            Args:
                x_min = min x value of bounding box
                x_max = max x value of bbox
                y_min = same idea but it's the y
                y_max = same idea but it's the y
        '''

        pass
    def run(self, raw_frame, detections):
        """ Run the CV logic. Returns the motion commands and visualized frame. """
        '''Args
                raw_frame = the raw video frame
                detections = ML output, exclusive to OAKd Cameras
                

        '''
        '''
        # Here is where all of the actual CV logic should be.
        # It should return motion values and the visualized frame.
        # This function will run basically every time a new frame is to be processed, which means multiple times a second.
        # This is why we use the self.<variable_name> variables to detail the state/alignment and other variables that depend 
        # on the current action, as those are global and will not be reinitialized to default every time this function runs.
        '''
        # Initializing motion variables -- it is in most forseeable cases advisable to do this, as not doing so may result in trying to return something that has not 
        # been initalized, resulting in the program crashing. 
        lateral = 0
        forward = 0
        yaw = 0
        
        #Set Assigment to what the model annotations are (they are capitazlied)
       
        #Attempting to Detect an object before we get anything This just no..... leaving for now but prob delete
        
        #Set detections to an empty list if nothing is detected prevents errors when nothing is seen
        if detections is None: 
            detections = []
        

        #Attempting to Detect an object before we get anything This just no..... leaving for now but prob delete
        sucessful_detections = [] #becomes a list of the detection items
        for detection in detections:
            if self.assignment in detection.label:
                print(print(f"[DEBUG] Detected {detection.label} with confidence {detection.confidence}"))
                if detection.confidence > 0.5: #comparing confience to thershold of 50 percent
                    print(self.assignment, 'detected')
                    sucessful_detections.append(detection)
        
        used_detection = None
        if len(sucessful_detections) == 0:
            print('no detections above thershold')
        elif len(sucessful_detections) == 1:
            used_detection = sucessful_detections[0]
        else:
            max_confidence_id = 0
            for i in range(0, len(sucessful_detections)):
                if i == 0:
                   continue
                elif sucessful_detections[i].confidence > sucessful_detections[i-1]:
                    max_confidence_id = i
            used_detection = sucessful_detections[max_confidence_id]
            detection_middle = self.get_midpoint(used_detection.xmin, used_detection.xmax, used_detection.ymin, used_detection.ymax)
            print('Middle is', detection_middle)
                

            
        
        # Return the frame and the motion values.
        return {"lateral" : lateral, "forward" : forward, "yaw" : yaw}, raw_frame

# This if statement is just saying what to do if this script is run directly. 
if __name__ == "__main__":
    # Example of how to obtain a training video. Make sure to follow this template when capturing your own video, in case 
    # another team member needs to run this code on his/her device. 
    
    # NOTE: When downloading the training data, the training data folder itself, which contains all of the data.
    video_root_path = "/home/kc/Desktop/Team Inspiration/RoboSub 2024/Training Data/" # Computer path through the training data folder.
    mission_name = "Buoy/" # Mission folder
    video_name = "Train Video 1.mp4" # Specified video
    video_path = os.path.join(video_root_path, mission_name, video_name)

    # For testing
    print(f"Video path: {video_path}")

    # Initialize an instance of the class.
    cv = CV("Blue")

    # Verify the path exists.
    if not os.path.exists(video_path):
        print(f"[ERROR] Video file not found {video_path}")
    else:
        # Capture the video object (basically access the specified video) at the specified path.
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            print(f"[ERROR] Unable to open video file: {video_path}")
        else:
            while True:
                # Access each frame of the video.
                ret, frame = cap.read()
                if not ret:
                    print("[INFO] End of file.")
                    break

                # Run the run function on the frame, and get back the relevant results.
                motion_values, viz_frame = cv.run(frame)
                if viz_frame is not None:
                    cv2.imshow("frame", viz_frame)
                else:
                    print("[ERROR] Unable to display frame.")

                # For testing purposes.
                print(f"Motion: {motion_values}")
                
                time.sleep(0.05)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break