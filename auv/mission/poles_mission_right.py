"""
Mission file for red pole slalom
NOTE: the sub must be facing the slalom perpendicularly since the cv currently doesn't account for that
"""

import json
import rospy
from std_msgs.msg import String
import time

from auv.device import cv_handler  # For running mission-specific CV scripts
from auv.motion import robot_control  # For running the motors on the sub
from auv.motion.utils import get_distance # For calculating distance
from auv.utils import disarm

class PoleSlalomMission:
  # Name of your red pole CV script file (no .py extension)
  def __init__(self, rc, target= None, **config):
      """
      Initialize the mission class; configure everything needed in the run function.
      """
      self.config = config
      self.cv_files = ["poles_cv_right"]
      self.data = {}
      self.next_data = {}
      self.received = False
      self.start_time = time.time()

      self.rc = rc
      self.cv_handler = cv_handler.CVHandler(**self.config)
      self.row_count = 0
      self.end = False
      self.target = target
      
      self.row_anchor = None
      self.returning_to_anchor = False

      rospy.loginfo("[INFO] Pole Slalom Mission Init")

  def callback(self, msg):
      file_name = msg._connection_header["topic"].split("/")[-1]
      data = json.loads(msg.data)
      self.next_data[file_name] = data
      self.received = True

  def run(self):
      """
      Run the pole slalom mission loop.
      """
      rospy.loginfo("Pole Slalom mission running")
      heading = self.rc.get_heading()
      self.rc.set_flight_mode("STABILIZE") # prevents pitch and roll movement
      self.rc.set_control_mode("depth_hold")
      self.rc.go_to_depth(0.6)  # change this based on the depth of the pool
      time.sleep(1)  # let depth hold and flight mode settle
      
      self.rc.go_to_heading(heading)  # set heading to 0 degrees (facing the slalom)
      self.rc.activate_heading_control(True)
      time.sleep(4) # let heading PID settle
      
      for file_name in self.cv_files:
        self.cv_handler.start_cv(file_name, self.callback)

      self.cv_handler.set_target("poles_cv_right", self.target)

      self.row_anchor = (self.rc.position['x'], self.rc.position['y'])  # anchor for row 1

      while not rospy.is_shutdown():
          time.sleep(0.05)
          if not self.received:
              rospy.logwarn("Did not receive frame")
              continue
          else:
              print(f"Received data: {self.next_data}")

          for key in self.next_data.keys():
              if key in self.data.keys():
                  self.data[key].update(self.next_data[key])
              else:
                  self.data[key] = self.next_data[key]

          self.received = False
          self.next_data = {}

          cv_data = self.data.get("poles_cv_right", {})
          lateral = cv_data.get("lateral", 0)
          state = cv_data.get("state", "searching")
          end = cv_data.get("end", False)

          # rospy.loginfo(cv_data)
          if end: # cv script ended
              if state != "finished":
                  rospy.loginfo("Pole Slalom mission timed out.")
                  break
              else: #cv was successful
                  rospy.loginfo(f"Attempting row {self.row_count + 1} out of 3")

                  # Stop current CV, do the inter-row move, restart CV fresh
                  self.cv_handler.stop_cv("poles_cv_right")

                  self.rc.go_lateral_distance(0.75) # strafe right for 0.75m to go between white and red pole
                  self.rc.go_forward_distance(1.5) # move forward for 1.25m to get next to the next row of poles

                  # Increase row count by one
                  self.row_count += 1
                  self.received = False
                  self.data = {}
                  self.row_anchor = (self.rc.position['x'], self.rc.position['y']) # reset anchor for new row
                  self.returning_to_anchor = False
                  
                  if self.row_count==3: # navigated through all three rows successfully
                    rospy.loginfo("All 3 rows completed")
                    break
                  
                  # Restart CV so end=False and state="searching" are fresh
                  self.cv_handler.start_cv("poles_cv_right", self.callback)
                  self.cv_handler.set_target("poles_cv_right", None)

                  # print status
                  rospy.loginfo(f"Current row: {self.row_count}")

          else: # still running cv script, take in the movement commands
              if state == "searching":
                    displacement = get_distance(
                        (self.rc.position['x'], self.rc.position['y']),
                        self.row_anchor)
                    
                    if not self.returning_to_anchor and displacement > 2.0:
                        self.returning_to_anchor = True
                    elif self.returning_to_anchor and displacement < 0.3:
                        self.returning_to_anchor = False

                    if self.returning_to_anchor:
                        lateral = 1.0   # opposite sign of the CV's search-lateral (-1)

                    if self.row_count == 0 and time.time() < 4:
                        print("First row search")
                        elapsed_time = time.time() - self.start_time
                        lateral = -1 if elapsed_time < 2 else 1

              self.rc.movement(lateral=lateral)

  def cleanup(self):
      """
      Clean up after the mission.
      """
      for file_name in self.cv_files:
          self.cv_handler.stop_cv(file_name)

      self.rc.movement(lateral=0, forward=0, yaw=0)
      print("[INFO] Pole Slalom mission terminated")

if __name__ == "__main__":
  from auv.utils import deviceHelper
  from auv.motion import robot_control
  import time

  rospy.init_node("pole_slalom_mission", anonymous=True)
  rc = robot_control.RobotControl()
  config = deviceHelper.variables

  mission = PoleSlalomMission(rc = rc, **config)
  mission.run()
  mission.cleanup()
  disarm.disarm()
