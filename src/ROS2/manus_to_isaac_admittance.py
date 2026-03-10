#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import signal
from nav_msgs.msg import Odometry
import roboticstoolbox as rtb
import spatialmath.base as smb
from std_msgs.msg import Float64MultiArray


# --- [A] Servo-Isaacsim Global Setup ---
URDF_PATH = '/home/vision/isaacsim/chkwon_isaac/urdf/m0609.white.urdf'
robot_left = rtb.Robot.URDF(URDF_PATH)
robot_right = rtb.Robot.URDF(URDF_PATH)

# T_robot2station_left (servo_isaacsim_ultimate.py에서 가져옴)
T_robot2station_left = SE3.CopyFrom(
    np.array(
    [[-0.9825364091725799, -0.010098366311354199, -0.18579619922941024, 2.077475419386988],
        [-0.13825783249305348, 0.707882164373932, 0.6926670266149078, -0.7383425188920228],
        [0.12452701027641133, 0.7062583528733649, -0.6969161781392058, 0.7977745272135249],
        [0, 0, 0, 1]]

    ),
    check=False
)

baseline = 0.707
T_R2L = SE3([
    [-1, 0, 0, 0],
    [0, 0, -1, baseline/np.sqrt(2)],
    [0, -1, 0, baseline/np.sqrt(2)],
    [0, 0, 0, 1]
])
T_robot2station_right = T_R2L * T_robot2station_left
correction_matrix = SE3.CopyFrom(
    np.array([
       [ -1,  0, 0, 0],
       [ 0,  0.707, 0.707, 0],
       [ 0,  0.707, -0.707, 0],
       [ 0,  0, 0, 1]
    ]),
    check=False
)
correction_matrix_right = SE3.CopyFrom(
    np.array([
       [ 1,  0, 0, 0],
       [ 0,  -0.707, -0.707, 0],
       [ 0, 0.707, -0.707, 0],
       [ 0,  0, 0, 1]
    ]),
    check=False
)

# R_correction: coordinate frame correction for tracker delta calculation
R_correction = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, 1, 0]
])




# --- [C] IK/FK Helper Functions (from servo_isaacsim_ultimate.py) ---
class LowPassFilter:
    """Low-pass filter for smoothing tracker movement"""
    def __init__(self, alpha=0.3):
        """
        alpha: smoothing factor (0 to 1)
        - Lower alpha = more smoothing (less responsive)
        - Higher alpha = less smoothing (more responsive)
        """
        self.alpha = alpha
        self.prev_value = None
    
    def update(self, current_value):
        if self.prev_value is None:
            self.prev_value = current_value.copy() if isinstance(current_value, np.ndarray) else current_value
            return current_value
        
        filtered = self.alpha * current_value + (1 - self.alpha) * self.prev_value
        self.prev_value = filtered.copy() if isinstance(filtered, np.ndarray) else filtered
        return filtered


class SE3LowPassFilter:
    """Low-pass filter specifically for SE3 poses"""
    def __init__(self, alpha=0.4):
        """
        alpha: smoothing factor (0 to 1)
        """
        self.alpha = alpha
        self.prev_se3 = None
    
    def update(self, current_se3):
        if self.prev_se3 is None:
            self.prev_se3 = current_se3
            return current_se3
        
        # Extract position
        curr_pos = current_se3.t
        prev_pos = self.prev_se3.t
        filtered_pos = self.alpha * curr_pos + (1 - self.alpha) * prev_pos
        
        # Extract quaternion for smooth rotation interpolation
        curr_quat = R.from_matrix(current_se3.R).as_quat()  # [x, y, z, w]
        prev_quat = R.from_matrix(self.prev_se3.R).as_quat()  # [x, y, z, w]
        
        # Ensure shortest path interpolation (same hemisphere)
        if np.dot(curr_quat, prev_quat) < 0:
            curr_quat = -curr_quat
        
        # Linear interpolation in quaternion space
        filtered_quat = self.alpha * curr_quat + (1 - self.alpha) * prev_quat
        # Normalize quaternion
        filtered_quat = filtered_quat / np.linalg.norm(filtered_quat)
        
        # Convert back to rotation matrix
        filtered_rot = R.from_quat(filtered_quat).as_matrix()
        
        # Construct filtered SE3
        filtered_se3 = SE3.Rt(filtered_rot, filtered_pos)
        self.prev_se3 = filtered_se3
        
        return filtered_se3

def pose_to_se3(msg):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    trans = np.array([pos.x, pos.y, pos.z])
    quat = np.array([ori.x, ori.y, ori.z, ori.w])
    rot = R.from_quat(quat).as_matrix()
    T = SE3.Rt(rot, trans)
    return T



# --- [D] Signal Handler ---
running = True
def signal_handler(sig, frame):
    global running
    running = False
    print("Shutting down...")

signal.signal(signal.SIGINT, signal_handler)


# --- [E] Tracker Delta Calculator Node Class ---
class DualTrackerDeltaCalculator(Node):
    def __init__(self):
        super().__init__('dual_tracker_delta_calculator')
        
        # --- [A] Tracker Pose Storage ---
        self.T_tracker_1_initial = None
        self.T_tracker_1_current = None
        self.T_tracker_2_initial = None
        self.T_tracker_2_current = None
        
        # Low-pass filters for tracker smoothing (reduces vibration)
        self.filter_1 = SE3LowPassFilter(alpha=0.4)
        self.filter_2 = SE3LowPassFilter(alpha=0.4)
        
        # --- [B] Tracker Subscriptions ---
        self.create_subscription(Odometry, '/tracker_1/pose', self.tracker_1_callback, 10)
        self.create_subscription(Odometry, '/tracker_2/pose', self.tracker_2_callback, 10)
        
        # --- [C] Delta Pose Publisher ---
        self.delta_pub = self.create_publisher(Float64MultiArray, '/delta_arm_cartesian_pose', 10)
        
        # --- [D] Timer for publishing delta poses ---
        self.timer = self.create_timer(0.02, self.publish_delta_poses)  # 50Hz


    # --- [F] Tracker Callbacks ---
    def tracker_1_callback(self, msg):
        """Callback for tracker 1 pose"""
        T = pose_to_se3(msg)
        T_corr = correction_matrix @ T
        T_tracker_1 = T_robot2station_left @ T_corr

        # Apply SE3-aware low-pass filter to reduce vibration
        T_tracker_1 = self.filter_1.update(T_tracker_1)

        if self.T_tracker_1_initial is None:
            self.T_tracker_1_initial = T_tracker_1
        self.T_tracker_1_current = T_tracker_1

    def tracker_2_callback(self, msg):
        """Callback for tracker 2 pose"""
        T = pose_to_se3(msg)
        T_corr = correction_matrix_right @ T
        T_tracker_2 = T_robot2station_right @ T_corr

        # Apply SE3-aware low-pass filter to reduce vibration
        T_tracker_2 = self.filter_2.update(T_tracker_2)

        if self.T_tracker_2_initial is None:
            self.T_tracker_2_initial = T_tracker_2
        self.T_tracker_2_current = T_tracker_2

    # --- [G] Delta Pose Publisher ---
    def publish_delta_poses(self):
        """Publish delta poses from tracker data using servo_isaac_ultimate calibration"""
        if (self.T_tracker_1_initial is None or self.T_tracker_1_current is None or
            self.T_tracker_2_initial is None or self.T_tracker_2_current is None):
            return
        
        # Calculate delta (relative transformation from initial pose)
        T_delta_1 = self.T_tracker_1_initial.inv() * self.T_tracker_1_current
        T_delta_2 = self.T_tracker_2_initial.inv() * self.T_tracker_2_current
        
        # Apply R_correction to position and rotation deltas (same as servo_isaac_ultimate)
        delta_pos_1 = R_correction @ T_delta_1.t  # [x, y, z] corrected
        delta_rpy_1 = smb.tr2rpy(T_delta_1.A, unit='rad')  # [roll, pitch, yaw]
        delta_rot_1 = R_correction @ delta_rpy_1  # Apply correction to rotation
        
        # Apply correction to tracker 2
        delta_pos_2 = R_correction @ T_delta_2.t  # [x, y, z] corrected
        delta_rpy_2 = smb.tr2rpy(T_delta_2.A, unit='rad')  # [roll, pitch, yaw]
        delta_rot_2 = R_correction @ delta_rpy_2  # Apply correction to rotation
        
        # Create message with format: [x1, y1, z1, roll1, pitch1, yaw1, x2, y2, z2, roll2, pitch2, yaw2]
        delta_data = Float64MultiArray()
        delta_data.data = np.concatenate([
            delta_pos_1, delta_rot_1,  # Tracker 1: x, y, z, roll, pitch, yaw (corrected)
            delta_pos_2, delta_rot_2   # Tracker 2: x, y, z, roll, pitch, yaw (corrected)
        ]).tolist()
        
        self.delta_pub.publish(delta_data)


def main(args=None):
    global running
    rclpy.init(args=args)
    node = DualTrackerDeltaCalculator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
