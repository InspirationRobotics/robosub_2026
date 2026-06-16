#!/usr/bin/env python3
import numpy as np
import rospy
import time
from statistics import mean
from geometry_msgs.msg import TwistStamped, PoseStamped, Vector3Stamped
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from std_msgs.msg import Float64
from auv.utils import deviceHelper
from sensor_msgs.msg import Imu
from mavros_msgs.msg import Mavlink
from transforms3d.euler import euler2mat


class EKF6State:
    def __init__(self, dt):
        self.dt = dt
        self.reset()

    def predict(self, dt=None):
        dt = dt if dt is not None else self.dt


        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # imu acceleration added to self.x
        B = np.zeros((6, 3))
        B[0, 0] = 0.5 * dt ** 2
        B[1, 1] = 0.5 * dt ** 2
        B[2, 2] = 0.5 * dt ** 2
        B[3, 0] = dt
        B[4, 1] = dt
        B[5, 2] = dt

        self.x = F @ self.x + B @ self.accel_world
        self.P = F @ self.P @ F.T + self.Q

    def update_dvl(self, z):
        H = np.zeros((3, 6))
        H[0, 3] = 1
        H[1, 4] = 1
        H[2, 5] = 1

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_dvl
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P


    def update_depth(self, depth):
        H = np.zeros((1, 6))
        H[0, 2] = 1

        z = np.array([[depth]])
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R_baro
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x += K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def reset(self):
        rospy.loginfo("EKF State being reset....")
        self.x = np.zeros((6, 1))
        self.P = np.eye(6) * 0.1
        self.Q = np.diag([0.01] * 3 + [0.3] * 3)
        self.R_dvl = np.eye(3) * 0.05
        self.R_baro = np.array([[0.05]])
        self.accel_world = np.zeros((3, 1))


class EKFNode:
    def __init__(self):
        rospy.init_node("ekf_6d_node")
        self.sub = deviceHelper.variables.get("sub")
        self.dt = 1.0 / 50.0  # 50Hz nominal

        self.ekf = EKF6State(self.dt)

        self.dvl_velocity = np.zeros((3, 1))
        self.imu_acc_data = {"ax": 0, "ay": 0, "az": 0}
        self.orientation = {"yaw": 0, "pitch": 0, "roll": 0}

        # IMU acceleration rotated to world frame with gravity removed, ready for predict step
        self.imu_accel_world = np.zeros((3, 1))

        self.depth = None
        self.depth_calib = 0
        self.calibrated = False

        # Tracks real elapsed time between ekf_step calls for accurate dt
        self.last_step_time = None
        self.last_imu_time = rospy.Time(0)

        self.pub = rospy.Publisher("/auv/state/pose", PoseStamped, queue_size=10)
        self.accel_pub = rospy.Publisher("/auv/state/imu_accel", Vector3Stamped, queue_size=10)

        self.imu_sub  = rospy.Subscriber("/auv/devices/vectornav", Imu, self.imu_callback)
        self.dvl_sub  = rospy.Subscriber("/auv/devices/dvl/velocity", TwistStamped, self.dvl_callback)
        #self.fog_sub  = rospy.Subscriber("/auv/devices/fog", Float64, self.fog_callback)
        self.baro_sub = rospy.Subscriber("/mavlink/from", Mavlink, self.barometer_callback)

        self.recalibrate_service = rospy.Service('/auv/services/calibrate/EKF', Trigger, self.serviceCallback)

        self.calibrate_depth()
        rospy.Timer(rospy.Duration(self.dt), self.ekf_step)

    def rot_matrix(self):
        yaw = (np.deg2rad(self.orientation['yaw']) + np.pi) % (2 * np.pi) - np.pi
        pitch = (np.deg2rad(self.orientation['pitch']) + np.pi) % (2 * np.pi) - np.pi
        roll = (np.deg2rad(self.orientation['roll']) + np.pi) % (2 * np.pi) - np.pi
        return euler2mat(ai=yaw, aj=pitch, ak=roll, axes='szyx')

    def imu_callback(self, msg):
        self.imu_acc_data["ax"] = msg.linear_acceleration.x
        self.imu_acc_data["ay"] = msg.linear_acceleration.y
        self.imu_acc_data["az"] = msg.linear_acceleration.z

        self.orientation['roll']  = msg.orientation.x
        self.orientation['pitch'] = (msg.orientation.y + 180) % 360
        self.orientation['yaw']   = msg.orientation.z

        a_world = self.rot_matrix() @ np.array([
            [self.imu_acc_data["ax"]],
            [self.imu_acc_data["ay"]],
            [self.imu_acc_data["az"]]])
        # Z is positive downward (depth convention), so gravity is +9.80665 in world Z
        a_world[2, 0] -= 9.80665

        self.imu_accel_world = a_world
        self.ekf.accel_world = a_world
        self.last_imu_time = rospy.Time.now()

    def fog_callback(self, msg):
        # FOG provides low-drift heading — use it for yaw in DVL frame rotation
       # self.orientation['yaw'] = msg.data
        pass
    def dvl_callback(self, msg):

        self.dvl_velocity = self.rot_matrix() @ np.array([
            [msg.twist.linear.x],
            [msg.twist.linear.y],
            [msg.twist.linear.z]
        ])

    def serviceCallback(self, request):
        rospy.loginfo("Recalibrating EKF...")
        self.reset()
        return TriggerResponse(success=True, message="EKF reset!")

    def barometer_callback(self, msg):
        try:
            if msg.msgid == 143:
                from struct import pack, unpack
                p = pack("QQ", *msg.payload64)
                _, press_abs, _, _ = unpack("Iffhxx", p)
                self.depth = (press_abs / (997.0474 * 9.80665 * 0.01)) - self.depth_calib
        except:
            pass

    def ekf_step(self, event):
        # Compute actual elapsed time since last step for accurate kinematics
        now = rospy.Time.now()
        if self.last_step_time is None:
            dt = self.dt
        else:
            dt = (now - self.last_step_time).to_sec()
            # Clamp dt to avoid large jumps on startup or timer hiccups
            dt = np.clip(dt, 0.001, 0.1)
        self.last_step_time = now
        if rospy.Time.now() - self.last_imu_time > rospy.Duration(0.5):
            self.ekf.accel_world = np.zeros((3, 1))
            self.ekf.predict(dt)
            self.ekf.update_dvl(self.dvl_velocity)
        if self.depth is not None and self.calibrated:
            self.ekf.update_depth(self.depth)
            self.publish_pose()
            self.publish_imu_accel()

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "base_link"

        pose_msg.pose.position.x = self.ekf.x[0, 0]
        pose_msg.pose.position.y = self.ekf.x[1, 0]
        pose_msg.pose.position.z = self.ekf.x[2, 0]

        pose_msg.pose.orientation.x = self.orientation['roll']
        pose_msg.pose.orientation.y = self.orientation['pitch']
        pose_msg.pose.orientation.z = self.orientation['yaw']
        pose_msg.pose.orientation.w = 1.0
        self.pub.publish(pose_msg)

    def publish_imu_accel(self):
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.vector.x = self.imu_accel_world[0, 0]
        msg.vector.y = self.imu_accel_world[1, 0]
        msg.vector.z = self.imu_accel_world[2, 0]
        self.accel_pub.publish(msg)

    def calibrate_depth(self, sample_time=3):
        rospy.loginfo("Starting Depth Calibration...")
        samples = []
        self.depth_calib = 0

        while self.depth is None:
            rospy.sleep(0.1)

        prev_depth = self.depth
        start_time = time.time()

        while time.time() - start_time < sample_time:
            if self.depth == prev_depth:
                continue
            samples.append(self.depth)
            prev_depth = self.depth

        self.depth_calib = mean(samples)
        self.calibrated = True
        rospy.loginfo(f"depth calibration Finished. Surface is: {self.depth_calib}")

    def reset(self):
        self.last_step_time = None
        self.calibrate_depth(sample_time=3)
        self.ekf.reset()


if __name__ == "__main__": 
    node = EKFNode()
    rospy.sleep(2)
    rospy.loginfo("Running the simple ekf node")
    rospy.spin()
