import os
import h5py
import numpy as np
from pynput import keyboard
import cv2
import time
from datetime import datetime
import subprocess
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

# --- Data Collection Setup ---
today = datetime.now().strftime('%m%d_%H%M')
data_dir = f'/home/vision/isaacsim/chkwon_isaac/imitation_learning/data/{today}'
os.makedirs(data_dir, exist_ok=True)
f = h5py.File(os.path.join(data_dir, 'common_data.hdf5'), 'w')
data = f.create_group('data')

# --- Homing positions ---
HOME_POSITIONS = [0.0016629954, 8.8492256e-05, -0.7872923, 0.7908614, -1.578184e-21, -2.0037353, 
                  1.9984876, -0.7853982, -0.24072896, 0.24419716, 1.3368195, -1.3341434, 
                  0.36657256, -0.42248344, 0.031378478, 7.269613e-06, -0.0012201909, 0.03194526, 
                  -0.10795939, -0.00048610225, -0.00044009602, -0.00041728042, -0.00048544328, 
                  0.0013096752, 0.17406371, 0.19926439, 0.16966505, 0.17376398, 0.00045802223, 
                  0.00050135946, 0.00022791198, 0.00055643334, 0.0005783597, 0.0019642455, 
                  0.28178254, 0.00018276226, 0.2863391, 0.28145367, 0.00028455444, -1.8377019e-05, 
                  -3.238982e-06, -5.573311e-07, -7.0894765e-07, 9.391724e-09, 0.18958478, 
                  0.16205524, 0.18738356, 0.18923509, 0.051507153, 0.953932, 1.0419573, 
                  1.0075854, 0.9823318, -0.10833493]

# Actual joint names from Isaac Sim
JOINT_NAMES = [
    'left_joint_1', 'right_joint_1', 'left_joint_2', 'right_joint_2', 'yaw_joint',
    'left_joint_3', 'right_joint_3', 'pitch_joint', 'left_joint_4', 'right_joint_4',
    'left_joint_5', 'right_joint_5', 'left_joint_6', 'right_joint_6',
    'left_baby_joint1', 'left_index_joint1', 'left_middle_joint1', 'left_ring_joint1', 'left_thumb_joint1',
    'right_baby_joint1', 'right_index_joint1', 'right_middle_joint1', 'right_ring_joint1', 'right_thumb_joint1',
    'left_baby_joint2', 'left_index_joint2', 'left_middle_joint2', 'left_ring_joint2', 'left_thumb_joint2',
    'right_baby_joint2', 'right_index_joint2', 'right_middle_joint2', 'right_ring_joint2', 'right_thumb_joint2',
    'left_baby_joint3', 'left_index_joint3', 'left_middle_joint3', 'left_ring_joint3', 'left_thumb_joint3',
    'right_baby_joint3', 'right_index_joint3', 'right_middle_joint3', 'right_ring_joint3', 'right_thumb_joint3',
    'left_baby_joint4', 'left_index_joint4', 'left_middle_joint4', 'left_ring_joint4', 'left_thumb_joint4',
    'right_baby_joint4', 'right_index_joint4', 'right_middle_joint4', 'right_ring_joint4', 'right_thumb_joint4'
]

def init_buffer():
    return {
        'joint_L': [],
        'joint_R': [],
        'hand_L': [],
        'hand_R': [],
        'image_H': [],
        'image_F': []
    }

recording = False
servoing = False
terminal = False
homing = False
waiting_for_save_decision = False
buffer = init_buffer()
servo_process = None
servo_process_left = None
servo_process_right = None

def get_save_decision():
    """별도 스레드에서 사용자 입력을 받아 데이터 저장 여부를 결정"""
    global buffer, waiting_for_save_decision
    
    while True:
        try:
            choice = input("Do you want to save the recorded data? (y/n): ").strip().lower()
            if choice == 'y':
                save_data(buffer)
                break
            elif choice == 'n':
                print("Data discarded.")
                break
            else:
                print("Please enter 'y' or 'n'")
        except:
            break
    
    # 버퍼 초기화 및 상태 리셋
    buffer = init_buffer()
    waiting_for_save_decision = False

def on_press(key):
    global recording, servoing, terminal, servo_process, servo_process_left, servo_process_right, homing, waiting_for_save_decision
    try:
        if key.char == 's':
            if not waiting_for_save_decision:  # 저장 결정을 기다리는 중이 아닐 때만
                recording = True
                servoing = True
                print("Start servoing and recording")
                # Ultimate 트래커 제어
                servo_process = subprocess.Popen(
                    ['python3', '/home/vision/manus_ws/src/ROS2/servo_isaac_ultimate_manus.py']
                )
                # Manus 트래커 제어
                # servo_process_left = subprocess.Popen(
                #         ['python3', '/home/vision/isaacsim/chkwon_isaac/imitation_learning/teleop/servo_leftarm.py']
                #     )
                # servo_process_right = subprocess.Popen(
                #         ['python3', '/home/vision/isaacsim/chkwon_isaac/imitation_learning/teleop/servo_rightarm.py']
                #     )
        elif key.char == 'q':    
            if recording and not waiting_for_save_decision:  # 녹화 중이고 저장 결정을 기다리는 중이 아닐 때만
                servoing = False
                recording = False
                homing = True
                waiting_for_save_decision = True
                print("Stop servoing and recording, homing robot...")
                #Ultimate 트래커 제어 종료
                if servo_process:
                    servo_process.terminate()
                    servo_process.wait()
                    servo_process = None

                #Vive Tracker 제어 종료
                # servo_process_left.terminate()
                # servo_process_left.wait()
                # servo_process_left = None
                # servo_process_right.terminate()
                # servo_process_right.wait()
                # servo_process_right = None
                # 별도 스레드에서 사용자 입력 받기
                input_thread = threading.Thread(target=get_save_decision)
                input_thread.daemon = True
                input_thread.start()
        elif key.char == 't':
            terminal = True
    except AttributeError:
        pass

def save_data(buffer):
    demo_n = len(data.keys())
    demo = data.create_group(f'demo_{demo_n}')
    for k, v in buffer.items():
        demo.create_dataset(k, data=np.array(v))
    print(f"Saved demo_{demo_n}, {len(buffer['joint_L'])} steps")
    print(f"  - Arm joints (L/R): {len(buffer['joint_L'][0]) if buffer['joint_L'] else 0}/{len(buffer['joint_R'][0]) if buffer['joint_R'] else 0}")
    print(f"  - Hand joints (L/R): {len(buffer['hand_L'][0]) if buffer['hand_L'] else 0}/{len(buffer['hand_R'][0]) if buffer['hand_R'] else 0}")
    print(f"  - Images (H/F): {len(buffer['image_H'])}/{len(buffer['image_F'])}")

class JointAndImageSubscriber(Node):
    def __init__(self):
        super().__init__('joint_image_node')
        # Updated joint names to match actual robot
        self.joint_name_L = ['left_joint_1', 'left_joint_2', 'left_joint_3', 'left_joint_4', 'left_joint_5', 'left_joint_6']
        self.joint_name_R = ['right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6']
        
        # Hand joint names
        self.hand_name_L = [
            'left_baby_joint1', 'left_baby_joint2', 'left_baby_joint3', 'left_baby_joint4',
            'left_index_joint1', 'left_index_joint2', 'left_index_joint3', 'left_index_joint4',
            'left_middle_joint1', 'left_middle_joint2', 'left_middle_joint3', 'left_middle_joint4',
            'left_ring_joint1', 'left_ring_joint2', 'left_ring_joint3', 'left_ring_joint4',
            'left_thumb_joint1', 'left_thumb_joint2', 'left_thumb_joint3', 'left_thumb_joint4'
        ]
        self.hand_name_R = [
            'right_baby_joint1', 'right_baby_joint2', 'right_baby_joint3', 'right_baby_joint4',
            'right_index_joint1', 'right_index_joint2', 'right_index_joint3', 'right_index_joint4',
            'right_middle_joint1', 'right_middle_joint2', 'right_middle_joint3', 'right_middle_joint4',
            'right_ring_joint1', 'right_ring_joint2', 'right_ring_joint3', 'right_ring_joint4',
            'right_thumb_joint1', 'right_thumb_joint2', 'right_thumb_joint3', 'right_thumb_joint4'
        ]
        
        self.latest_joint_L = None
        self.latest_joint_R = None
        self.latest_hand_L = None
        self.latest_hand_R = None
        self.latest_image_H = None
        self.latest_image_F = None
        self.bridge = CvBridge()
        
        # Subscribers with Isaac Sim compatible QoS
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        
        # Use QoS compatible with Isaac Sim
        isaac_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        
        self.create_subscription(JointState, '/isaac_joint_states', self.joint_callback, isaac_qos)
        self.create_subscription(Image, '/isaac_camera_front/rgb', self.front_callback, isaac_qos)
        self.create_subscription(Image, '/isaac_camera_top/rgb', self.top_callback, isaac_qos)
        
        # Publisher for homing
        self.joint_publisher = self.create_publisher(JointState, '/isaac_joint_command', 10)

    def joint_callback(self, msg):
        joint_mapping = {n: p for n, p in zip(msg.name, msg.position)}
        
        # Extract left and right arm joints
        joint_position_L = [joint_mapping.get(j) for j in self.joint_name_L]
        joint_position_R = [joint_mapping.get(j) for j in self.joint_name_R]
        
        # Extract left and right hand joints
        hand_position_L = [joint_mapping.get(j) for j in self.hand_name_L]
        hand_position_R = [joint_mapping.get(j) for j in self.hand_name_R]
        
        self.latest_joint_L = joint_position_L
        self.latest_joint_R = joint_position_R
        self.latest_hand_L = hand_position_L
        self.latest_hand_R = hand_position_R

    def front_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_F = cv_img
        except Exception as e:
            self.latest_image_F = None

    def top_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_H = cv_img
        except Exception as e:
            self.latest_image_H = None

    def publish_homing_positions(self):
        """Publish the homing joint positions"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Use the actual joint names from Isaac Sim
        msg.name = JOINT_NAMES
        msg.position = HOME_POSITIONS
        msg.velocity = [0.0] * len(HOME_POSITIONS)
        msg.effort = [0.0] * len(HOME_POSITIONS)
        
        self.joint_publisher.publish(msg)
        print(f"Published homing positions to {len(HOME_POSITIONS)} joints")

def show_images(img_H, img_F, recording=False, waiting_for_decision=False):
    if img_H is not None:
        overlay_H = img_H.copy()
        if waiting_for_decision:
            status_text = 'HOMING/WAITING'
            color = (0, 255, 255)  # Yellow
        else:
            status_text = 'REC' if recording else 'LIVE'
            color = (0, 0, 255) if recording else (255, 255, 255)
        
        cv2.putText(
            overlay_H,
            status_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
            cv2.LINE_AA
        )
        cv2.imshow('IsaacSim Top Camera', overlay_H)
    
    if img_F is not None:
        overlay_F = img_F.copy()
        if waiting_for_decision:
            status_text = 'HOMING/WAITING'
            color = (0, 255, 255)  # Yellow
        else:
            status_text = 'REC' if recording else 'LIVE'
            color = (0, 0, 255) if recording else (255, 255, 255)
        
        cv2.putText(
            overlay_F,
            status_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            color,
            2,
            cv2.LINE_AA
        )
        cv2.imshow('IsaacSim Front Camera', overlay_F)
    cv2.waitKey(1)

def servo_step():
    # --- Place your servoing logic from servo_ultimate_isaacsim.py here ---
    # For example, calculate target poses, joint deltas, and publish joint commands
    # You may need to import and use global variables as in your original code
    pass

def main(args=None):
    global recording, servoing, terminal, buffer, servo_process, servo_process_left, servo_process_right, homing, waiting_for_save_decision
    recording = False
    servoing = False
    terminal = False
    homing = False
    waiting_for_save_decision = False
    buffer = init_buffer()

    rclpy.init(args=args)
    node = JointAndImageSubscriber()

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    Hz = 20
    rate = 1.0 / Hz

    print('s: start servoing/recording, q: stop and save, h: homing, t: terminate')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            start_time = time.monotonic()

            if terminal:
                #Ultimate 트래커 제어 종료
                if servo_process:
                    servo_process.terminate()
                    servo_process.wait()
                    servo_process = None    
                
                #Vive Tracker 제어 종료
                # servo_process_left.terminate()
                # servo_process_left.wait()
                # servo_process_left = None
                # servo_process_right.terminate()
                # servo_process_right.wait()
                # servo_process_right = None
                f.close()
                print("Terminating.")
                break

            # Handle homing command
            if homing:
                node.publish_homing_positions()
                homing = False

            img_H = node.latest_image_H
            img_F = node.latest_image_F
            show_images(img_H, img_F, recording=recording, waiting_for_decision=waiting_for_save_decision)

            # --- Servoing logic ---
            if servoing:
                servo_step()  # Call your servoing logic here

            # --- Data recording ---
            if recording:
                joint_L = node.latest_joint_L
                joint_R = node.latest_joint_R
                hand_L = node.latest_hand_L
                hand_R = node.latest_hand_R
                
                buffer['joint_L'].append(joint_L)
                buffer['joint_R'].append(joint_R)
                buffer['hand_L'].append(hand_L)
                buffer['hand_R'].append(hand_R)
                
                if img_H is not None:
                    buffer['image_H'].append(img_H.copy())
                if img_F is not None:
                    buffer['image_F'].append(img_F.copy())

            current_time = time.monotonic()
            elapsed_time = current_time - start_time
            reward_time = rate - elapsed_time
            if reward_time > 0:
                time.sleep(reward_time)

    except KeyboardInterrupt:
        pass

    finally:
        listener.stop()
        #Ultimate 트래커 제어 종료
        if servo_process:
            servo_process.terminate()
            servo_process.wait()
            servo_process = None    

        #Vive Tracker 제어 종료
        # servo_process_left.terminate()
        # servo_process_left.wait()
        # servo_process_left = None

        # servo_process_right.terminate()
        # servo_process_right.wait()
        # servo_process_right = None

        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.shutdown()
        print("ROS2 shutdown.")

if __name__ == "__main__":
    main()