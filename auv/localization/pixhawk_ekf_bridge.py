#!/usr/bin/env python3
"""
pixhawk_ekf_bridge.py

Bridges VectorNav VN-100 orientation and Teledyne DVL velocity into the
Orange Cube Pixhawk's EKF2 via MAVROS.

EKF2 fuses:
  - Internal IMU (accel + gyro) for prediction
  - External attitude from VN-100  → VISION_POSITION_ESTIMATE (attitude only)
  - External velocity from DVL     → VISION_SPEED_ESTIMATE (body-frame velocity)

Depth does NOT come from the EKF: the Cube's internal baro measures air
pressure inside the enclosure, so EKF z is meaningless underwater. Instead
this node decodes SCALED_PRESSURE2 (msgid 143, the external Bar30 water
pressure sensor) from /mavlink/from, zeroes it at the surface on startup,
and publishes depth in meters (down-positive) on /auv/state/depth.
Re-zero anytime with:  rosservice call /auv/services/calibrate/depth

Required ArduSub parameters (set via QGroundControl or MAVProxy):
  EK2_ENABLE        = 1
  EK2_GPS_TYPE      = 3   (external vision, no GPS)
  AHRS_EKF_TYPE     = 2
  GPS_TYPE          = 0

Published MAVROS topics (consumed by Pixhawk EKF2):
  /mavros/vision_pose/pose         — VN-100 attitude as external reference
  /mavros/vision_speed/speed_twist — DVL body-frame velocity

EKF2 output is consumed directly by robot_control.py from:
  /mavros/local_position/pose      — Pixhawk EKF2 estimated pose (ENU)
"""

import math
import time
from struct import pack, unpack

import rospy
import numpy as np

from geometry_msgs.msg import (
    PoseStamped, TwistStamped, Vector3Stamped
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from std_srvs.srv import Trigger, TriggerResponse
from nav_msgs.msg import Odometry
from mavros_msgs.msg import Mavlink


class PixhawkEKFBridge:
    def __init__(self):
        rospy.init_node("pixhawk_ekf_bridge")

        # Latest state from sensors
        self._roll  = 0.0   # radians
        self._pitch = 0.0   # radians
        self._yaw   = 0.0   # radians, world frame
        self._imu_ready = False

        # Depth from external water pressure sensor (Bar30 via Pixhawk)
        self._depth_raw   = None   # uncalibrated depth in meters
        self._depth_calib = 0.0    # surface offset
        self._depth_calibrated = False

        # Publishers → MAVROS → Pixhawk EKF2
        self._pub_vision_pose  = rospy.Publisher(
            "/mavros/vision_pose/pose", PoseStamped, queue_size=5
        )
        self._pub_vision_speed = rospy.Publisher(
            "/mavros/vision_speed/speed_twist", TwistStamped, queue_size=5
        )
        self._pub_depth = rospy.Publisher(
            "/auv/state/depth", Float64, queue_size=5
        )

        # Subscribers from sensor drivers
        rospy.Subscriber(
            "/auv/devices/vectornav", Imu, self._imu_cb, queue_size=10
        )
        rospy.Subscriber(
            "/auv/devices/dvl/velocity", TwistStamped, self._dvl_cb, queue_size=10
        )
        rospy.Subscriber(
            "/mavlink/from", Mavlink, self._mavlink_cb, queue_size=20
        )

        rospy.Service(
            "/auv/services/calibrate/depth", Trigger, self._depth_calib_srv
        )

        # Zero the depth at the surface before reporting anything
        self._calibrate_depth()

        rospy.loginfo("pixhawk_ekf_bridge: ready — forwarding VN-100 + DVL to EKF")
        rospy.spin()

    # ------------------------------------------------------------------
    # Water pressure sensor → /auv/state/depth
    # ------------------------------------------------------------------
    def _mavlink_cb(self, msg):
        # SCALED_PRESSURE2 (msgid 143) carries the external Bar30 water
        # pressure sensor. press_abs is in hPa; convert to meters of water.
        try:
            if msg.msgid == 143:
                p = pack("QQ", *msg.payload64)
                _, press_abs, _, _ = unpack("Iffhxx", p)
                self._depth_raw = press_abs / (997.0474 * 9.80665 * 0.01)
                if self._depth_calibrated:
                    self._pub_depth.publish(
                        Float64(self._depth_raw - self._depth_calib)
                    )
        except Exception:
            pass

    def _calibrate_depth(self, sample_time=3):
        rospy.loginfo("pixhawk_ekf_bridge: calibrating surface depth...")
        while self._depth_raw is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        samples = []
        prev = None
        start = time.time()
        while time.time() - start < sample_time:
            if self._depth_raw != prev:
                samples.append(self._depth_raw)
                prev = self._depth_raw
            rospy.sleep(0.02)

        if samples:
            self._depth_calib = float(np.mean(samples))
        self._depth_calibrated = True
        rospy.loginfo(
            f"pixhawk_ekf_bridge: depth zeroed, surface = {self._depth_calib:.3f} m"
        )

    def _depth_calib_srv(self, request):
        self._depth_calibrated = False
        self._calibrate_depth()
        return TriggerResponse(
            success=True, message=f"Depth re-zeroed at {self._depth_calib:.3f}"
        )

    # ------------------------------------------------------------------
    # VN-100 IMU callback → VISION_POSITION_ESTIMATE (attitude only)
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        # vn100_serial.py packs RPY into the orientation quaternion fields
        # as (roll_rad, pitch_rad, yaw_rad, 1.0) — not a real quaternion.
        # Extract RPY and build a proper quaternion for MAVROs
# fixed — convert to radians and remove the 180° pitch mount offset
        self._roll  = math.radians(msg.orientation.x)
        self._pitch = math.radians(msg.orientation.y - 180)   # vn100_serial centers level at 180°
        self._yaw   = math.radians(msg.orientation.z)
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

    # ------------------------------------------------------------------
    # DVL callback → VISION_SPEED_ESTIMATE
    # ------------------------------------------------------------------
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


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

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
