import time
import rospy




from auv.motion.robot_control import RobotControl
from std_msgs.msg import String
from auv.utils import deviceHelper


class intersubComMission:
   def __init__(self, robotControl=None):
       self.rc = robotControl
       self.pub_modem = rospy.Publisher("/auv/devices/modem/send", String, queue_size=10)
       self.sub_modem = rospy.Subscriber("/auv/devices/modem/received", String, self.rec_callback)
       self.sub = deviceHelper.variables.get('sub')
       self.end = False
       self.roll_requested = False  # <-- New flag


   def send_modem_message(self, dest_addr, move):
       try:
           self.rc.send_modem(addr=dest_addr, movement=move)
           with open('coms.log', 'a') as file:
               file.write(f"[{time.time()}] Sent message to address {dest_addr} with movement {move}\n")
           time.sleep(1)
       except Exception as e:
           rospy.logerr(f"Failed to send modem message: {e}")


   def rec_callback(self, msg):
       if not self.end and self.sub == "graey":
           with open('coms.log', 'a') as file:
               file.write(f"[{time.time()}] Received message: {msg.data}\n")
           if msg.data == "YAW":
               self.send_modem_message(dest_addr="020", move="graey received YAW")# send YAW message to ONYX when Graey receives a YAW message
               rospy.loginfo("Attempting to YAW")
               self.rc.go_to_heading(90)
               self.rc.go_to_heading(180)
               self.rc.go_to_heading(270)
               self.rc.go_to_heading(360)
               rospy.loginfo("Graey yaw done, telling Onyx to YAW")
               self.send_modem_message(dest_addr="020", move="YAW")
           self.end = True
           #if msg.data == "ROLL":
              # self.send_modem_message(dest_addr="020", move="graey attempting to ROLL") # send ROLL message to ONYX when Graey receives a ROLL message
               #rospy.loginfo("Roll maneuver requested")
               #self.roll_requested = True  # <-- Store request, but don't execute yet
           #self.end = True


       if self.sub =="onyx":
           with open('coms.log', 'a') as file:
               file.write(f"[{time.time()}] Received message: {msg.data}\n")
           if msg.data == "YAW": # if the message content is "YAW", then execute the yaw maneuver
               self.send_modem_message(dest_addr="010", move="onyx attempting to YAW") # send YAW message to Graey when ONYX receives a YAW message
               rospy.loginfo("Attempting to YAW")
               self.rc.go_to_heading(90)
               self.rc.go_to_heading(180)
               self.rc.go_to_heading(270)
               self.rc.go_to_heading(360)
               self.send_modem_message(dest_addr="010", move="Onyx has completed YAW movement") # send YAW message to Graey when ONYX finishes YAW maneuver
           self.end = True
      


   def do_roll(self):
       """Executes the roll maneuver if requested"""
       if self.roll_requested:
           rospy.loginfo("Doing roll maneuver")
           self.rc.set_flight_mode("ACRO")
           self.rc.set_control_mode("direct")
           self.rc.movement(roll=5)
           time.sleep(4)
           self.rc.movement()
           time.sleep(2)
           self.rc.set_flight_mode("STABILIZE")
           self.rc.set_control_mode("depth_hold")
           rospy.loginfo("Roll maneuver complete")
       else:
           rospy.logwarn("Roll was not requested, skipping roll maneuver")


   def run(self):
       """Only handles communication, not the yaw maneuver"""
       current_sub = self.sub
       self.rc.set_control_mode("depth_hold")
       self.rc.activate_heading_control(True)
       time.sleep(1)


       if current_sub == "graey":
           time_counter = 0
           while not self.end:
               if time_counter >= 20:
                   rospy.loginfo("Time out, no message received")
                   break
                   #fake_msg = String()
                   #fake_msg.data = "YAW"
                   #self.rec_callback(fake_msg)
               time.sleep(1)
               time_counter += 1


       elif current_sub == "onyx":
           rospy.loginfo("Attempting to signal to Graey to YAW")
           destination_addr = "010"
           rospy.loginfo("Sending message to Graey")
           for i in range(5):
               self.send_modem_message(dest_addr=destination_addr, move="YAW")
               time.sleep(1)
               time_counter = 0
               if time_counter >= 20: # after Onyx has sent 20 messages, if it hasn't received a response, end the mission
                   rospy.loginfo("Time out, Onyx did not receive message, ending transmission")
                   break #not too sure if this is the best way to handle this, but it should work for now
               time.sleep(1)
               time_counter += 1




if __name__=="__main__":
   #rostopic pub -1 /chatter std_msgs/String "data: 'hello world'"
   #Please use the above example to simulate a msg received when testing
   rospy.loginfo("Starting modem test script...")
   rospy.init_node("intersub_coms_mission", anonymous=True)
   rc = RobotControl()
   rc.set_flight_mode("STABILIZE")
   rc.set_control_mode("depth_hold")
   rc.go_to_depth(0.5)
   time.sleep(10)
   mission = intersubComMission(robotControl=rc)
   mission.run()
   rc.exit()



