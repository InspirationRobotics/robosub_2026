import rospy
import time
from auv.motion import robot_control
from auv.utils import arm, disarm

def waypoint_wrap(input_x, input_y, input_z, input_yaw, input_pitch, input_roll):
    rc = robot_control.RobotControl()
    tolerance = 0.5  # meters (tbd until PID tuning should be 0.5 meters)
    time_out = 10.0

    rc.set_control_mode("pid")
    rc.set_absolute_pitch(input_pitch)
    rc.set_absolute_roll(input_roll)
    rc.set_absolute_yaw(input_yaw)
    rc.set_absolute_z(input_z)
    rc.set_absolute_y(input_y)
    rc.set_absolute_x(input_x)

    start_time = time.time()

    while not rospy.is_shutdown():
        current_x = rc.position['x']
        current_y = rc.position['y']
        current_z = rc.position['z']
        error_x = abs (current_x - input_x)
        error_y = abs (current_y - input_y)
        error_z = abs (current_z - input_z)
        rospy.loginfo(f"Error: x: {error_x:.3f}, y: {error_y:.3f}, z: {error_z:.3f}")
        if error_x < tolerance and error_y < tolerance and error_z < tolerance:
            rospy.loginfo("Position Hold: success")
            break
        elif time.time() - start_time > time_out:
            rospy.loginfo("Position Hold: fail due to time out")
            break
    rc.reset()
    rc.set_control_mode("depth_hold") #activate depth hold, consider if necessary

if __name__=="__main__":
    """Testing controls"""
    rc = robot_control.RobotControl(True)
    rc.set_control_mode("depth_hold")
    rc.set_flight_mode("STABILIZE")
    rc.activate_heading_control(True)
    depth = .5
    rc.go_to_depth(depth)
    rc.go_to_heading(0)
    rc.activate_heading_control()
    rc.waypointNav(1, .3)
    waypoint_wrap(1, .3, depth, 0, 0, 0 )
    rc.waypointNav(0, 0)
    waypoint_wrap(0, 0, depth, 0, 0, 0 )
    rospy.loginfo("Reached the end")
    disarm.disarm()
    rc.exit()
