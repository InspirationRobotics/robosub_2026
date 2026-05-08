import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm
if __name__ == "__main__":
    try:
        rospy.init_node("station_keeping_test", anonymous=True)  # avoid hiearchy issue
        rc = robot_control.RobotControl()
        rc.set_control_mode("pid")
        arm.arm()
        time.sleep(2.0)
        print("[INFO}This is the start of station keeping test")
        rc.set_absolute_yaw(0)
        rc.set_absolute_x(0)
        rc.set_absolute_y(0)
        rc.set_absolute_z(0.3)
        rospy.loginfo("start station keeping for 60s...")
        time.sleep(60)

        rospy.loginfo("Reached the end of the program")
        rc.reset()
        rc.exit()
        disarm.disarm()
    except KeyboardInterrupt:
        disarm.disarm()
        
