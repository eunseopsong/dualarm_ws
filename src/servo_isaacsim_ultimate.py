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

URDF_PATH = '/home/vision/isaacsim/chkwon_isaac/urdf/m0609.white.urdf'
robot_left = rtb.Robot.URDF(URDF_PATH)
robot_right = rtb.Robot.URDF(URDF_PATH)

T_robot2station_left = SE3.CopyFrom(
    np.array(
   [[-0.9825364091725799, -0.010098366311354199, -0.18579619922941024, 2.077475419386988],
        [-0.13825783249305348, 0.707882164373932, 0.6926670266149078, -0.7383425188920228],
        [0.12452701027641133, 0.7062583528733649, -0.6969161781392058, 0.7977745272135249],
        [0, 0, 0, 1]]

    ),
    check=False
)

baseline = 0.707  # 두 팔 사이 거리(예시, 실제 값으로 수정)
T_R2L = SE3([
    [-1, 0, 0, 0],
    [0, 0, -1, baseline/np.sqrt(2)],
    [0, -1, 0, baseline/np.sqrt(2)],
    [0, 0, 0, 1]
])
T_robot2station_right = T_R2L * T_robot2station_left

# Correction matrix to fix coordinate axis mapping
# Tracker X -> Robot -Y (45°), Z (45°)
# Tracker Y -> Robot -X
# Tracker Z -> Robot +Y (45°), -Z (45°)
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



# === Global Variables ===
running = True
node = None

# Initial joint states (to be set once)
initial_joints_left = None
initial_joints_right = None
initial_hand_joints_left = None
initial_hand_joints_right = None

# Current joint states (internal, not from robot)
current_joints_left = None
current_joints_right = None
current_hand_joints_left = None
current_hand_joints_right = None

# Tracker initial pose
T_tracker_initial_left = None
T_tracker_initial_right = None

# Tracker current pose (updated every callback)
T_tracker_current_left = None
T_tracker_current_right = None

# Open loop: only use internal joint state after initialization
tracker_initialized = False

def signal_handler(sig, frame):
    global running
    running = False
    print("Shutting down...")

signal.signal(signal.SIGINT, signal_handler)

class OpenLoopServoNode(Node):
    def __init__(self):
        super().__init__('open_loop_servo_node')
        # QoS for compatibility
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        qos_profile = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.joint_command_pub = self.create_publisher(
            JointState,
            '/isaac_joint_command',
            qos_profile
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.joint_state_callback,
            10
        )
        self.tracker_left_sub = self.create_subscription(
            Odometry,
            '/tracker_1/pose',
            self.tracker_left_callback,
            10
        )
        self.tracker_right_sub = self.create_subscription(
            Odometry,
            '/tracker_2/pose',
            self.tracker_right_callback,
            10
        )
        # Subscribe to hand joint states if available
        self.hand_state_sub = self.create_subscription(
            JointState,
            '/hand_joint_states',
            self.hand_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        global initial_joints_left, initial_joints_right, current_joints_left, current_joints_right
        global initial_hand_joints_left, initial_hand_joints_right, current_hand_joints_left, current_hand_joints_right
        # Only set once
        if initial_joints_left is not None and initial_joints_right is not None:
            return

        joint_map = dict(zip(msg.name, msg.position))
        left_names = [f'left_joint_{i+1}' for i in range(6)]
        right_names = [f'right_joint_{i+1}' for i in range(6)]
        hand_names_left = [
            'left_baby_joint1', 'left_baby_joint2', 'left_baby_joint3', 'left_baby_joint4',
            'left_index_joint1', 'left_index_joint2', 'left_index_joint3', 'left_index_joint4',
            'left_middle_joint1', 'left_middle_joint2', 'left_middle_joint3', 'left_middle_joint4',
            'left_ring_joint1', 'left_ring_joint2', 'left_ring_joint3', 'left_ring_joint4',
            'left_thumb_joint1', 'left_thumb_joint2', 'left_thumb_joint3', 'left_thumb_joint4'
        ]
        hand_names_right = [
            'right_baby_joint1', 'right_baby_joint2', 'right_baby_joint3', 'right_baby_joint4',
            'right_index_joint1', 'right_index_joint2', 'right_index_joint3', 'right_index_joint4',
            'right_middle_joint1', 'right_middle_joint2', 'right_middle_joint3', 'right_middle_joint4',
            'right_ring_joint1', 'right_ring_joint2', 'right_ring_joint3', 'right_ring_joint4',
            'right_thumb_joint1', 'right_thumb_joint2', 'right_thumb_joint3', 'right_thumb_joint4'
        ]
        try:
            initial_joints_left = np.array([joint_map[n] for n in left_names])
            initial_joints_right = np.array([joint_map[n] for n in right_names])
            initial_hand_joints_left = np.array([joint_map.get(n, 0.0) for n in hand_names_left])
            initial_hand_joints_right = np.array([joint_map.get(n, 0.0) for n in hand_names_right])
            
            current_joints_left = initial_joints_left.copy()
            current_joints_right = initial_joints_right.copy()
            current_hand_joints_left = initial_hand_joints_left.copy()
            current_hand_joints_right = initial_hand_joints_right.copy()
            self.destroy_subscription(self.joint_state_sub)
        except Exception as e:
            print(f"Error extracting initial joint states: {e}")

    # correction_matrix를 SE3로 변환
    
    def hand_state_callback(self, msg):
        """Callback to update hand joint states from dedicated hand tracking"""
        global current_hand_joints_left, current_hand_joints_right
        joint_map = dict(zip(msg.name, msg.position))
        hand_names_left = [
            'left_baby_joint1', 'left_baby_joint2', 'left_baby_joint3', 'left_baby_joint4',
            'left_index_joint1', 'left_index_joint2', 'left_index_joint3', 'left_index_joint4',
            'left_middle_joint1', 'left_middle_joint2', 'left_middle_joint3', 'left_middle_joint4',
            'left_ring_joint1', 'left_ring_joint2', 'left_ring_joint3', 'left_ring_joint4',
            'left_thumb_joint1', 'left_thumb_joint2', 'left_thumb_joint3', 'left_thumb_joint4'
        ]
        hand_names_right = [
            'right_baby_joint1', 'right_baby_joint2', 'right_baby_joint3', 'right_baby_joint4',
            'right_index_joint1', 'right_index_joint2', 'right_index_joint3', 'right_index_joint4',
            'right_middle_joint1', 'right_middle_joint2', 'right_middle_joint3', 'right_middle_joint4',
            'right_ring_joint1', 'right_ring_joint2', 'right_ring_joint3', 'right_ring_joint4',
            'right_thumb_joint1', 'right_thumb_joint2', 'right_thumb_joint3', 'right_thumb_joint4'
        ]
        try:
            current_hand_joints_left = np.array([joint_map.get(n, 0.0) for n in hand_names_left])
            current_hand_joints_right = np.array([joint_map.get(n, 0.0) for n in hand_names_right])
        except Exception as e:
            pass  # Silently continue if hand states not available
    

    def tracker_left_callback(self, msg):
        global T_tracker_initial_left, T_tracker_current_left, tracker_initialized
        T = pose_to_se3(msg)
        # correction_SE3 적용
        # T_corr = T
        T_corr = correction_matrix @ T
        # 로봇 기준으로 변환
        T_robot2track_left = T_robot2station_left @ T_corr

        if T_tracker_initial_left is None:
            T_tracker_initial_left = T_robot2track_left
        T_tracker_current_left = T_robot2track_left
        check_tracker_ready()

    def tracker_right_callback(self, msg):
        global T_tracker_initial_right, T_tracker_current_right, tracker_initialized
        T = pose_to_se3(msg)
        # correction_matrix와 T_robot2station_right 적용
        T_corr = correction_matrix_right @ T
        # T_corr =  T
        T_robot2track_right = T_robot2station_right @ T_corr

        if T_tracker_initial_right is None:
            T_tracker_initial_right = T_robot2track_right
        T_tracker_current_right = T_robot2track_right
        check_tracker_ready()

def pose_to_se3(msg):
    pos = msg.pose.pose.position
    ori = msg.pose.pose.orientation
    trans = np.array([pos.x, pos.y, pos.z])
    quat = np.array([ori.x, ori.y, ori.z, ori.w])
    rot = R.from_quat(quat).as_matrix()
    T = SE3.Rt(rot, trans)  # <-- 여기서 수정!
    return T

def check_tracker_ready():
    global tracker_initialized, T_tracker_initial_left, T_tracker_initial_right
    if T_tracker_initial_left is not None and T_tracker_initial_right is not None:
        tracker_initialized = True

def publish_joint_command(left_joints, right_joints, left_hand_joints=None, right_hand_joints=None):
    global node
    joint_msg = JointState()
    joint_msg.header.stamp = node.get_clock().now().to_msg()
    
    joint_names = [
        'left_joint_1', 'left_joint_2', 'left_joint_3',
        'left_joint_4', 'left_joint_5', 'left_joint_6',
        'right_joint_1', 'right_joint_2', 'right_joint_3',
        'right_joint_4', 'right_joint_5', 'right_joint_6'
    ]
    
    joint_positions = left_joints.tolist() + right_joints.tolist()
    
    # Add hand joints if provided
    if left_hand_joints is not None:
        hand_names_left = [
            'left_baby_joint1', 'left_baby_joint2', 'left_baby_joint3', 'left_baby_joint4',
            'left_index_joint1', 'left_index_joint2', 'left_index_joint3', 'left_index_joint4',
            'left_middle_joint1', 'left_middle_joint2', 'left_middle_joint3', 'left_middle_joint4',
            'left_ring_joint1', 'left_ring_joint2', 'left_ring_joint3', 'left_ring_joint4',
            'left_thumb_joint1', 'left_thumb_joint2', 'left_thumb_joint3', 'left_thumb_joint4'
        ]
        joint_names.extend(hand_names_left)
        joint_positions.extend(left_hand_joints.tolist() if isinstance(left_hand_joints, np.ndarray) else left_hand_joints)
    
    if right_hand_joints is not None:
        hand_names_right = [
            'right_baby_joint1', 'right_baby_joint2', 'right_baby_joint3', 'right_baby_joint4',
            'right_index_joint1', 'right_index_joint2', 'right_index_joint3', 'right_index_joint4',
            'right_middle_joint1', 'right_middle_joint2', 'right_middle_joint3', 'right_middle_joint4',
            'right_ring_joint1', 'right_ring_joint2', 'right_ring_joint3', 'right_ring_joint4',
            'right_thumb_joint1', 'right_thumb_joint2', 'right_thumb_joint3', 'right_thumb_joint4'
        ]
        joint_names.extend(hand_names_right)
        joint_positions.extend(right_hand_joints.tolist() if isinstance(right_hand_joints, np.ndarray) else right_hand_joints)
    
    joint_msg.name = joint_names
    joint_msg.position = joint_positions
    joint_msg.velocity = []
    joint_msg.effort = []

    node.joint_command_pub.publish(joint_msg)

def tracker_to_target_pose(T_tracker_initial, T_tracker_current, T_robot2station):
    """
    트래커의 초기 pose와 현재 pose, 로봇-스테이션 변환을 받아
    엔드이펙터의 목표 pose(SE3)를 계산
    """
    if T_tracker_initial is None or T_tracker_current is None:
        return None
    # 트래커 이동량
    T_delta = T_tracker_initial.inv() * T_tracker_current
    # 엔드이펙터 기준 목표 pose (초기 pose에서 delta만큼 이동)
    T_ee_target = T_robot2station * T_tracker_initial * T_delta
    return T_ee_target

def compute_ik(robot, T_target, q_init):
    """
    robot: roboticstoolbox Robot
    T_target: 목표 SE3 pose
    q_init: 초기 joint angles (numpy array)
    """
    sol = robot.ikine_LM(T_target, q0=q_init)
    if sol.success:
        return sol.q
    else:
        print("IK 실패: ", sol.reason)
        return q_init  # 실패시 기존 값 유지

def calculate_target_pose(current_pose_se3, pose_init, tracker_init, tracker_current, T_robot2station, Kp_pos=0.5, Kp_rot=0.3):
    if tracker_init is None or tracker_current is None or pose_init is None:
        return None
    R_correction = np.array([
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0]
    ])
    T_tracker_delta = tracker_init.inv() * tracker_current
    # delta_vr_pos = T_tracker_delta.t
    # delta_vr_rot = smb.tr2rpy(T_tracker_delta.A, unit='rad')
    
    delta_vr_pos = R_correction @ T_tracker_delta.t
    delta_vr_rot = R_correction @ smb.tr2rpy(T_tracker_delta.A, unit='rad')
    if np.linalg.norm(delta_vr_pos) < 0.01:
        delta_vr_pos = np.zeros_like(delta_vr_pos)
        delta_vr_rot = np.zeros_like(delta_vr_rot)
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
    if current_pose_se3 is None or robot is None:
        return None
    pose_error = current_pose_se3.inv() * target_pose
    err_pos = target_pose.t - current_pose_se3.t
    err_rot_ee = smb.tr2rpy(pose_error.A, unit='rad')
    err_rot_base = current_pose_se3.R @ err_rot_ee
    err_6d = np.concatenate((err_pos, err_rot_base))
    try:
        J = robot.jacob0(current_joints, end=robot.links[7])
    except:
        try:
            J = robot.jacob0(current_joints, end=robot.links[5])
        except Exception as e:
            print(f"Jacobian calculation failed: {e}")
            return None
    dq = np.linalg.pinv(J) @ err_6d
    if np.linalg.norm(dq) > acc_limit:
        dq *= acc_limit / np.linalg.norm(dq)
    dq = np.clip(dq, -max_joint_vel, max_joint_vel)
    return dq

def main():
    global node, running, initial_joints_left, initial_joints_right, current_joints_left, current_joints_right
    global initial_hand_joints_left, initial_hand_joints_right, current_hand_joints_left, current_hand_joints_right
    rclpy.init()
    node = OpenLoopServoNode()

    while rclpy.ok() and running:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (initial_joints_left is not None and initial_joints_right is not None
            and tracker_initialized):
            break
        time.sleep(0.1)

  

    # 초기 pose 저장
    robot_pose_init_left = None
    robot_pose_init_right = None

    step_gain = 0.5  # 증분 비율
    acc_limit = 10.0
    max_joint_vel = np.deg2rad(10.0)

    while rclpy.ok() and running:
        rclpy.spin_once(node, timeout_sec=0.01)
        if (T_tracker_initial_left is not None and T_tracker_current_left is not None and
            T_tracker_initial_right is not None and T_tracker_current_right is not None):

            # 현재 pose 계산
            if robot_pose_init_left is None:
                robot_pose_init_left = robot_left.fkine(initial_joints_left, end=robot_left.links[7])
            if robot_pose_init_right is None:
                robot_pose_init_right = robot_right.fkine(initial_joints_right, end=robot_right.links[7])

            # 목표 pose 계산 (servo_ultimate_isaacsim 방식)
            target_pose_left = calculate_target_pose(
                robot_left.fkine(current_joints_left, end=robot_left.links[7]),
                robot_pose_init_left,
                T_tracker_initial_left,
                T_tracker_current_left,
                T_robot2station_left
            )
            target_pose_right = calculate_target_pose(
                robot_right.fkine(current_joints_right, end=robot_right.links[7]),
                robot_pose_init_right,
                T_tracker_initial_right,
                T_tracker_current_right,
                T_robot2station_right
            )

            # 증분 IK 계산
            dq_left = calculate_joint_delta(
                robot_left,
                current_joints_left,
                robot_left.fkine(current_joints_left, end=robot_left.links[7]),
                target_pose_left,
                acc_limit=acc_limit,
                max_joint_vel=max_joint_vel
            )
            dq_right = calculate_joint_delta(
                robot_right,
                current_joints_right,
                robot_right.fkine(current_joints_right, end=robot_right.links[7]),
                target_pose_right,
                acc_limit=acc_limit,
                max_joint_vel=max_joint_vel
            )

            # 조인트 업데이트
            if dq_left is not None:
                current_joints_left = current_joints_left + dq_left * step_gain
            if dq_right is not None:
                current_joints_right = current_joints_right + dq_right * step_gain

            # 조인트 명령 발행 (hand joints included)
            publish_joint_command(
                current_joints_left, 
                current_joints_right,
                current_hand_joints_left,
                current_hand_joints_right
            )

        time.sleep(0.02)  # 50Hz update rate for faster hand responsiveness

    print("Shutting down...")
    rclpy.shutdown()

if __name__ == "__main__":
    main()