
import math
import rospy
import numpy as np

from geometry_msgs.msg import (
    PoseStamped, TwistStamped, Vector3Stamped
)
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class PixhawkEKFBridge:
    def __init__(self):
        rospy.init_node("pixhawk_ekf_bridge")

        # Latest state from sensors
        self._roll  = 0.0   # radians
        self._pitch = 0.0   # radians
        self._yaw   = 0.0   # radians, world frame
        self._imu_ready = False

        #publishes position, velocity
        self._pub_vision_pose  = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=5)
        self._pub_vision_speed = rospy.Publisher("/mavros/vision_speed/speed_twist", TwistStamped, queue_size=5)
        rospy.Subscriber("/auv/devices/vectornav", Imu, self._imu_cb, queue_size=1)
        rospy.Subscriber("/auv/devices/dvl/velocity", TwistStamped, self._dvl_cb, queue_size=10)
        rospy.loginfo("pixhawk_ekf_bridge: ready — forwarding VN-100 + DVL to EKF2")
        rospy.spin()

  #gives attitude 
    def _imu_cb(self, msg: Imu):
        # vn100_serial.py packs RPY into the orientation quaternion fields
        # as (roll_rad, pitch_rad, yaw_rad, 1.0) — not a real quaternion.
        # Extract RPY and build a proper quaternion for MAVROS.
        self._roll  = msg.orientation.x
        self._pitch = msg.orientation.y
        self._yaw   = msg.orientation.z
        self._imu_ready = True

        q = _rpy_to_quaternion(self._roll, self._pitch, self._yaw)

        pose = PoseStamped()
        pose.header.stamp    = msg.header.stamp
        pose.header.frame_id = "map"
        pose.pose.position.x = 0.0   # position unknown from IMU alone
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self._pub_vision_pose.publish(pose)

    # vision_speed_estimate of dvl
  
    def _dvl_cb(self, msg: TwistStamped):
        # dvl.py already publishes in sub body frame (forward=x, right=y, down=z).
        # MAVROS vision_speed expects velocity in the ENU world frame, so we
        # rotate body-frame DVL velocity by the current yaw from VN-100.
        if not self._imu_ready:
            return

        vx_body = msg.twist.linear.x
        vy_body = msg.twist.linear.y
        vz_body = msg.twist.linear.z

        # Yaw rotation: body → world (ENU)
        # ArduSub EKF2 uses NED internally; MAVROS handles the ENU↔NED
        # conversion automatically when you publish to vision_speed/speed_twist.
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)

        vx_world =  cos_y * vx_body - sin_y * vy_body
        vy_world =  sin_y * vx_body + cos_y * vy_body
        vz_world =  vz_body   # down is down in both frames

        speed = TwistStamped()
        speed.header.stamp    = msg.header.stamp
        speed.header.frame_id = "map"
        speed.twist.linear.x  = vx_world
        speed.twist.linear.y  = vy_world
        speed.twist.linear.z  = vz_world

        self._pub_vision_speed.publish(speed)


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert roll/pitch/yaw (radians) to quaternion (x, y, z, w)."""
    cr = math.cos(roll  / 2)
    sr = math.sin(roll  / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw   / 2)
    sy = math.sin(yaw   / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


if __name__ == "__main__":
    try:
        PixhawkEKFBridge()
    except rospy.ROSInterruptException:
        pass
