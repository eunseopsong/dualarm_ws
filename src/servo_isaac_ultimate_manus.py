#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R
import time
import signal
from nav_msgs.msg import Odometry
import roboticstoolbox as rtb
import spatialmath.base as smb
from manus_ros2_msgs.msg import ManusGlove, ManusErgonomics
from collections import deque
from std_msgs.msg import Float64MultiArray
from collections import deque
import math


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


# --- [B] Manus-to-Aidin Global Setup ---
LEFT_SCALE = np.array([0.8, -1.9, 1.6,  # left thumb
                 0.7, 0.85, 0.9,  # left index
                 0.7, 0.85, 0.9,  # left middle
                 0.6, 0.85, 0.9,  # left ring
                 0.6, 0.85, 0.9], dtype=np.float64) # left baby

RIGHT_SCALE = np.array([0.8, -1.9, 1.6,  # right thumb
                 0.7, 0.85, 0.9,  # right index
                 0.7, 0.85, 0.9,  # right middle
                 0.6, 0.85, 0.9,  # right ring
                 0.6, 0.85, 0.9], dtype=np.float64) # right baby

LEFT_OFFSET = np.array([0, 2, 0,
                    -0.08, -0.43, -0.20,
                    -0.12, -0.43, -0.20,
                    -0.18, -0.58, 0,
                    -0.22, -0.58, 0], dtype=np.float64)

RIGHT_OFFSET = np.array([-0, 2, 0,
                    0, -0.43, -0.20,
                    0, -0.43, -0.20,
                    -0.18, -0.58, 0,
                    -0.22, -0.58, 0], dtype=np.float64)

LEFT_MASKING = np.array([1, 1, 1,  # left thumb
                    0, 1, 1,  # left index
                    0, 1, 1,  # left middle
                    0, 1, 1,  # left ring
                    0, 1, 1], dtype=np.float64)  # left baby

RIGHT_MASKING = np.array([1, 1, 1,  # right thumb
                    0, 1, 1,  # right index
                    0, 1, 1,  # right middle
                    0, 1, 1,  # right ring
                    0, 1, 1], dtype=np.float64)  # right baby

ERG_ORDER = [
    "ThumbMCPSpread", "ThumbMCPStretch",  "ThumbPIPStretch",  "ThumbDIPStretch",
    "IndexSpread",    "IndexMCPStretch",  "IndexPIPStretch",  "IndexDIPStretch",
    "MiddleSpread",   "MiddleMCPStretch", "MiddlePIPStretch", "MiddleDIPStretch",
    "RingSpread",     "RingMCPStretch",   "RingPIPStretch",   "RingDIPStretch",
    "PinkySpread",    "PinkyMCPStretch",  "PinkyPIPStretch",  "PinkyDIPStretch"]

NAME2IDX = {n: i for i, n in enumerate(ERG_ORDER)}

MANUS2AIDIN = np.asarray([   # ERG_ORDER의 몇번째 index인지 
    NAME2IDX["ThumbMCPSpread"],NAME2IDX["ThumbMCPStretch"], NAME2IDX["ThumbPIPStretch"],
    NAME2IDX["IndexSpread"],  NAME2IDX["IndexMCPStretch"], NAME2IDX["IndexPIPStretch"],
    NAME2IDX["MiddleSpread"], NAME2IDX["MiddleMCPStretch"],NAME2IDX["MiddlePIPStretch"],
    NAME2IDX["RingSpread"],   NAME2IDX["RingMCPStretch"],  NAME2IDX["RingPIPStretch"],
    NAME2IDX["PinkySpread"],  NAME2IDX["PinkyMCPStretch"], NAME2IDX["PinkyPIPStretch"]  
], dtype=np.int64)

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

def calculate_target_pose(current_pose_se3, pose_init, tracker_init, tracker_current, T_robot2station, Kp_pos=0.6, Kp_rot=1.0):
    if tracker_init is None or tracker_current is None or pose_init is None:
        return None
    R_correction = np.array([
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0]
    ])
    T_tracker_delta = tracker_init.inv() * tracker_current
    delta_vr_pos = R_correction @ T_tracker_delta.t
    err_rot_ee = smb.tr2rpy(T_tracker_delta.A, unit='rad')
    delta_vr_rot = R_correction @ err_rot_ee

    # --- Apply Deadzone Filter to reduce vibration ---
    # Position deadzone (meters)
    pos_deadzone = 0.0015
    # Rotation deadzone (radians)
    rot_deadzone = 0.015
    
    pos_norm = np.linalg.norm(delta_vr_pos)
    rot_norm = np.linalg.norm(delta_vr_rot)
    
    # Apply deadzone: if movement is smaller than threshold, set to zero
    if pos_norm < pos_deadzone:
        delta_vr_pos = np.zeros_like(delta_vr_pos)
    else:
        # Scale back the position so that movements just above deadzone are reduced
        delta_vr_pos = delta_vr_pos * (pos_norm - pos_deadzone) / pos_norm
    
    if rot_norm < rot_deadzone:
        delta_vr_rot = np.zeros_like(delta_vr_rot)
    else:
        # Scale back the rotation similarly
        delta_vr_rot = delta_vr_rot * (rot_norm - rot_deadzone) / rot_norm
    
    R_base2station = T_robot2station.R
    R_tracker_init = tracker_init.R
    R_end2base_init = pose_init.inv().R
    delta_pos_end = R_end2base_init @ R_base2station @ R_tracker_init @ delta_vr_pos
    delta_rot_end = R_end2base_init @ R_base2station @ R_tracker_init @ delta_vr_rot
    scaled_delta_pos = Kp_pos * delta_pos_end
    scaled_delta_rot = Kp_rot * delta_rot_end
    
    target_pose = pose_init * SE3(scaled_delta_pos) * SE3.RPY(*scaled_delta_rot, unit='rad')
    return target_pose

def calculate_joint_delta(robot, current_joints, current_pose_se3, target_pose, acc_limit=10.0, max_joint_vel=np.deg2rad(10.0)):
    if current_pose_se3 is None or robot is None or target_pose is None:
        return None
    
    pose_error = current_pose_se3.inv() * target_pose
    err_pos = target_pose.t - current_pose_se3.t
    
    # 회전 오차: SE3의 회전 오차(tr2rpy)를 엔드이펙터(EE) 좌표계에서 계산
    err_rot_ee = smb.tr2rpy(pose_error.A, unit='rad')
    # 회전 오차를 베이스(Base) 좌표계로 변환 (Jacobian 사용을 위해)
    err_rot_base = current_pose_se3.R @ err_rot_ee
    
    err_6d = np.concatenate((err_pos, err_rot_base))

    # 오차가 매우 작으면 움직임을 멈춥니다.
    if np.linalg.norm(err_6d) < 1e-4:
        return np.zeros_like(current_joints)

    try:
        # End-effector link is robot.links[7] for this URDF
        J = robot.jacob0(current_joints, end=robot.links[7])
    except Exception as e:
        # print(f"Jacobian calculation failed: {e}")
        return None
    
    # 6D 에러를 조인트 속도로 변환 (Pseudo-inverse)
    dq = np.linalg.pinv(J) @ err_6d

    # 속도 제한 적용
    if np.linalg.norm(dq) > acc_limit:
        dq *= acc_limit / np.linalg.norm(dq)
    dq = np.clip(dq, -max_joint_vel, max_joint_vel)
    
    return dq

def manus_to_aidin_qpos(manus20_deg: np.ndarray) -> np.ndarray:
    assert manus20_deg.shape == (20,), "Expect 20-D ergonomics vector"
    aidin15_deg = manus20_deg[MANUS2AIDIN]
    aidin15_rad = aidin15_deg * np.pi / 180

    return aidin15_rad  

# --- [D] Signal Handler ---
running = True
def signal_handler(sig, frame):
    global running
    running = False
    print("Shutting down...")

signal.signal(signal.SIGINT, signal_handler)


# --- [E] Combined ROS 2 Node Class ---
class DualArmHandController(Node):
    def __init__(self):
        super().__init__('dual_arm_hand_controller')
        
        # --- [A] Manus-to-Aidin (손 관절) 초기화 ---
        self.left_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)
        self.right_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)
        
        # 마누스 구독자
        self.create_subscription(ManusGlove, '/manus_glove_0', self.left_hand_callback, 1)
        self.create_subscription(ManusGlove, '/manus_glove_1', self.right_hand_callback, 1)

        # --- [B] Servo-to-Isaac (팔 관절) 초기화 ---
        self.initial_joints_left = None
        self.initial_joints_right = None
        self.current_joints_left = None
        self.current_joints_right = None
        
        self.T_tracker_initial_left = None
        self.T_tracker_initial_right = None
        self.T_tracker_current_left = None
        self.T_tracker_current_right = None
        
        # Low-pass filters for tracker smoothing (reduces vibration)
        # Using SE3-aware filters that properly handle rotation matrices
        self.filter_left = SE3LowPassFilter(alpha=0.4)
        self.filter_right = SE3LowPassFilter(alpha=0.4)
        
        self.robot_pose_init_left = None
        self.robot_pose_init_right = None
        self.tracker_initialized = False

        # IK Parameters
        self.step_gain = 0.5
        self.acc_limit = 10.0
        self.max_joint_vel = np.deg2rad(10.0)

        # 팔 관절 초기 상태 수신용 구독자
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/isaac_joint_states', self.joint_state_callback, qos_profile
        )
        # 트래커 구독자
        self.create_subscription(Odometry, '/tracker_1/pose', self.tracker_left_callback, 10)
        self.create_subscription(Odometry, '/tracker_2/pose', self.tracker_right_callback, 10)
        
        # --- [C] 통합 발행자 및 타이머 ---
        self.joint_command_pub = self.create_publisher(JointState, '/isaac_joint_command', 10)
        
        # 통합 명령 발행 주기 설정: 0.02초 (50Hz)
        self.timer = self.create_timer(0.02, self.combined_command_publish) 

        # --- [D] 전체 관절 이름 정의 (Arm + Hand) ---
        self.all_joint_names = [
            # 1. Arm Joints (12개)
            'left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6',
            'right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6',
            # 2. Hand Joints (30개)
            'left_thumb_joint1',  'left_thumb_joint2',  'left_thumb_joint3',
            'left_index_joint1',  'left_index_joint2',  'left_index_joint3',
            'left_middle_joint1', 'left_middle_joint2', 'left_middle_joint3',
            'left_ring_joint1',   'left_ring_joint2',   'left_ring_joint3',
            'left_baby_joint1',   'left_baby_joint2',   'left_baby_joint3',
            'right_thumb_joint1', 'right_thumb_joint2', 'right_thumb_joint3',
            'right_index_joint1', 'right_index_joint2', 'right_index_joint3',
            'right_middle_joint1','right_middle_joint2','right_middle_joint3',
            'right_ring_joint1',  'right_ring_joint2',  'right_ring_joint3',
            'right_baby_joint1',  'right_baby_joint2',  'right_baby_joint3'
        ]


    # --- [F] Servo-Isaacsim Callbacks (Tracker & Joint State Init) ---
    def joint_state_callback(self, msg):
        # Initial joint states 설정 (한번만)
        if self.initial_joints_left is not None and self.initial_joints_right is not None:
            return

        joint_map = dict(zip(msg.name, msg.position))
        left_names = [f'left_joint_{i+1}' for i in range(6)]
        right_names = [f'right_joint_{i+1}' for i in range(6)]
        try:
            self.initial_joints_left = np.array([joint_map[n] for n in left_names])
            self.initial_joints_right = np.array([joint_map[n] for n in right_names])
            self.current_joints_left = self.initial_joints_left.copy()
            self.current_joints_right = self.initial_joints_right.copy()
            self.get_logger().info('Arm initial joints set.')
            # 초기화 완료 후 구독 해제
            self.destroy_subscription(self.joint_state_sub)
        except Exception as e:
            self.get_logger().error(f"Error extracting initial joint states: {e}")

    def tracker_left_callback(self, msg):
        T = pose_to_se3(msg)
        T_corr = correction_matrix @ T
        T_robot2track_left = T_robot2station_left @ T_corr

        # Apply SE3-aware low-pass filter to reduce vibration
        T_robot2track_left = self.filter_left.update(T_robot2track_left)

        if self.T_tracker_initial_left is None:
            self.T_tracker_initial_left = T_robot2track_left
        self.T_tracker_current_left = T_robot2track_left
        self.check_tracker_ready()

    def tracker_right_callback(self, msg):
        T = pose_to_se3(msg)
        T_corr = correction_matrix_right @ T
        T_robot2track_right = T_robot2station_right @ T_corr

        # Apply SE3-aware low-pass filter to reduce vibration
        T_robot2track_right = self.filter_right.update(T_robot2track_right)

        if self.T_tracker_initial_right is None:
            self.T_tracker_initial_right = T_robot2track_right
        self.T_tracker_current_right = T_robot2track_right
        self.check_tracker_ready()

    def check_tracker_ready(self):
        if self.T_tracker_initial_left is not None and self.T_tracker_initial_right is not None:
            self.tracker_initialized = True


    # --- [G] Manus-to-Aidin Callbacks (Data Update) ---
    def left_hand_callback(self, msg):
        for ergo in msg.ergonomics:
            name = ergo.type
            i = NAME2IDX.get(name)  
            if i is not None:
                self.left_manus_data[i] = ergo.value

    def right_hand_callback(self, msg):
        for ergo in msg.ergonomics:
            name = ergo.type
            i = NAME2IDX.get(name)
            if i is not None:
                self.right_manus_data[i] = ergo.value


    # --- [H] Combined Command Publish (Core Logic) ---
    def combined_command_publish(self):
        
        # 1. 팔 관절 명령 계산 (Servo Logic)
        arm_data = None
        if (self.current_joints_left is not None and self.current_joints_right is not None
            and self.tracker_initialized):
            
            # 초기 포즈 저장 (한번만)
            if self.robot_pose_init_left is None:
                self.robot_pose_init_left = robot_left.fkine(self.initial_joints_left, end=robot_left.links[7])
            if self.robot_pose_init_right is None:
                self.robot_pose_init_right = robot_right.fkine(self.initial_joints_right, end=robot_right.links[7])

            # 목표 pose 계산
            target_pose_left = calculate_target_pose(
                robot_left.fkine(self.current_joints_left, end=robot_left.links[7]),
                self.robot_pose_init_left,
                self.T_tracker_initial_left,
                self.T_tracker_current_left,
                T_robot2station_left
            )
            target_pose_right = calculate_target_pose(
                robot_right.fkine(self.current_joints_right, end=robot_right.links[7]),
                self.robot_pose_init_right,
                self.T_tracker_initial_right,
                self.T_tracker_current_right,
                T_robot2station_right
            )

            # 증분 IK 계산
            dq_left = calculate_joint_delta(
                robot_left,
                self.current_joints_left,
                robot_left.fkine(self.current_joints_left, end=robot_left.links[7]),
                target_pose_left,
                acc_limit=self.acc_limit,
                max_joint_vel=self.max_joint_vel
            )
            dq_right = calculate_joint_delta(
                robot_right,
                self.current_joints_right,
                robot_right.fkine(self.current_joints_right, end=robot_right.links[7]),
                target_pose_right,
                acc_limit=self.acc_limit,
                max_joint_vel=self.max_joint_vel
            )

            # 조인트 업데이트
            if dq_left is not None:
                self.current_joints_left = self.current_joints_left + dq_left * self.step_gain
            if dq_right is not None:
                self.current_joints_right = self.current_joints_right + dq_right * self.step_gain
            
            arm_data = np.concatenate([self.current_joints_left, self.current_joints_right])
        
        # IK 초기화가 안 되었으면 발행하지 않습니다.
        if arm_data is None:
            return 
            
        # 2. 손 관절 명령 계산 (Manus Logic)
        left_aidin_data = manus_to_aidin_qpos(self.left_manus_data)
        right_aidin_data = manus_to_aidin_qpos(self.right_manus_data)
        
        # 스케일, 오프셋, 마스킹 적용
        left_hand_data = (left_aidin_data * LEFT_SCALE + LEFT_OFFSET) * LEFT_MASKING
        right_hand_data = (right_aidin_data * RIGHT_SCALE + RIGHT_OFFSET) * RIGHT_MASKING
        hand_data = np.concatenate([left_hand_data, right_hand_data])

        # 조건 적용: 0보다 작으면 0.01으로
        # [1,2,4,5,7,8,10,11,13,14]는 왼손, [+15]는 오른손
        target_hand_idx = [1,2,4,5,7,8,10,11,13,14,
                          15+1,15+2,15+4,15+5,15+7,15+8,15+10,15+11,15+13,15+14]
        hand_data[target_hand_idx] = np.where(hand_data[target_hand_idx] < 0.01, 0.01, hand_data[target_hand_idx])

        # 3. 데이터 병합 및 발행
        combined_position = np.concatenate([arm_data, hand_data])
        
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.all_joint_names
        joint_msg.position = combined_position.tolist()
        
        self.joint_command_pub.publish(joint_msg)

        # np.set_printoptions(precision=5, suppress=True)
        # print('Arm L/R:\n', arm_data.reshape(-1, 6))
        # print('Hand L:\n', hand_data[:15].reshape(-1, 3))


def main(args=None):
    global running
    rclpy.init(args=args)
    node = DualArmHandController()

    try:
        # rclpy.spin_once 대신 rclpy.spin을 사용하여 모든 콜백과 타이머를 처리합니다.
        # 타이머가 주 제어 루프 역할을 하므로, spin만으로 충분합니다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()