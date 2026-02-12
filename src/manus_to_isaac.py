import time
import os
import numpy as np
import argparse
import matplotlib.pyplot as plt
import h5py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from manus_ros2_msgs.msg import ManusGlove, ManusErgonomics
from collections import deque
from std_msgs.msg import Float64MultiArray

# 마누스 글러브 실행 코드
# ros2 run manus_ros2 manus_data_publisher


LEFT_SCALE = np.array([-0.8, -3, 2.5,  # left thumb
                 -0.9, 1.4, 1.5,  # left index
                 -0.9, 1.4, 1.5,  # left middle
                 -0.8, 1.4, 1.5,  # left ring
                 -0.8, 1.4, 1.5], dtype=np.float64) # left baby

RIGHT_SCALE = np.array([0.8, -1.9, 1.6,  # right thumb
                 0.7, 1.4, 1.5,  # right index
                 0.7, 1.4, 1.5,  # right middle
                 0.6, 1.4, 1.5,  # right ring
                 0.6, 1.4, 1.5], dtype=np.float64) # right baby

LEFT_OFFSET = np.array([0, 2, 0,
                    -0.08, -0.43, -0.20,
                    -0.12, -0.43, -0.20,
                    -0.18, -0.58, 0,
                    -0.22, -0.58, 0], dtype=np.float64)

RIGHT_OFFSET = np.array([0, 1.4, 0,
                    0, -0.43, 0,
                    0, -0.43, -0.20,
                    0, -0.58, 0,
                    0, -0.58, 0], dtype=np.float64)


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


class ManusToIsaac(Node):
    def __init__(self):
        super().__init__('manus_to_isaac')

        self.left_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)
        self.right_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)

        self.subscription = self.create_subscription(
            ManusGlove,
            '/manus_glove_0',
            self.left_hand_callback,
            1
        )
        self.subscription = self.create_subscription(
            ManusGlove,
            '/manus_glove_1',
            self.right_hand_callback,
            1
        )

        # self.left_hand_publisher = self.create_publisher(
        #     JointState, 
        #     # '/remote_position_controller/left_hand_joints',
        #     # '/hand_joint_controller/joint_state_command',   # hand controller용
        #     '/aidin_dualarm_joint_controller/joint_state_command',   # dual arm controller용
        #     # '/left_dsr_joint_controller/joint_state_command',   # dual arm controller용
        #     # '/isaac_joint_command', 
        #     10)
        # self.right_hand_publisher = self.create_publisher(
        #     JointState, 
        #     # '/remote_position_controller/right_hand_joints',
        #     '/aidin_dualarm_joint_controller/joint_state_command',   # dual arm controller용
        #     # '/right_dsr_joint_controller/joint_state_command',   # dual arm controller용
        #     # '/isaac_joint_command', 
        #     10)
        
        self.hand_publisher = self.create_publisher(
            JointState,
            '/isaac_joint_command',
            10
            )
        self.timer = self.create_timer(0.005, self.aidin_hand_command_publish)
        
        # self.timer_left = self.create_timer(0.005, self.left_aidin_hand_command_publish)
        # self.timer_right = self.create_timer(0.005, self.right_aidin_hand_command_publish)

    def left_hand_callback(self, msg):
        # receive left manus data
        for ergo in msg.ergonomics:
            name = ergo.type
            i = NAME2IDX.get(name)  
            if i is not None:
                self.left_manus_data[i] = ergo.value   # 왼손 마누스 데이터 저장

    def right_hand_callback(self, msg):
        # receive right manus data
        for ergo in msg.ergonomics:
            name = ergo.type
            i = NAME2IDX.get(name)
            if i is not None:
                self.right_manus_data[i] = ergo.value   # 오른손 마누스 데이터 저장
    
    def aidin_hand_command_publish(self):
        aidin_msg = JointState()
        aidin_msg.name = [
            'left_thumb_joint1',  'left_thumb_joint2',  'left_thumb_joint3',
            'left_index_joint1',  'left_index_joint2',  'left_index_joint3',
            'left_middle_joint1', 'left_middle_joint2', 'left_middle_joint3',
            'left_ring_joint1',   'left_ring_joint2',   'left_ring_joint3',
            'left_baby_joint1',   'left_baby_joint2',   'left_baby_joint3',
            'right_thumb_joint1', 'right_thumb_joint2', 'right_thumb_joint3',
            'right_index_joint1', 'right_index_joint2', 'right_index_joint3',
            'right_middle_joint1','right_middle_joint2','right_middle_joint3',
            'right_ring_joint1',  'right_ring_joint2',  'right_ring_joint3',
            'right_baby_joint1',  'right_baby_joint2',  'right_baby_joint3']
        
        left_aidin_data = self.manus_to_aidin_qpos(self.left_manus_data)
        right_aidin_data = self.manus_to_aidin_qpos(self.right_manus_data)
        aidin_data = np.concatenate([left_aidin_data, right_aidin_data])
        assert len(aidin_data) == 30, "Expect 30-D Aidin joint position"
        
        left_aidin_data = (left_aidin_data * LEFT_SCALE + LEFT_OFFSET) * LEFT_MASKING
        right_aidin_data = (right_aidin_data * RIGHT_SCALE + RIGHT_OFFSET) * RIGHT_MASKING
        aidin_data = np.concatenate([left_aidin_data, right_aidin_data])

        
        # 조건 적용: 0보다 작으면 0으로
        target_idx = [1,2,4,5,7,8,10,11,13,14,
                      15+1,15+2,15+4,15+5,15+7,15+8,15+10,15+11,15+13,15+14]
        aidin_data[target_idx] = np.where(aidin_data[target_idx] < 0.01, 0.01, aidin_data[target_idx])
        aidin_msg.position = aidin_data.tolist()

        np.set_printoptions(precision=5, suppress=True)
        print('left_aidin_hand\n', np.array(aidin_msg.position[:15]).reshape(-1,3))
        print('right_aidin_hand\n', np.array(aidin_msg.position[15:]).reshape(-1,3))

        self.hand_publisher.publish(aidin_msg)



    def manus_to_aidin_qpos(self, manus20_deg: np.ndarray) -> np.ndarray:
        assert manus20_deg.shape == (20,), "Expect 20-D ergonomics vector"
        aidin15_deg = manus20_deg[MANUS2AIDIN]
        aidin15_rad = aidin15_deg * np.pi / 180

        return aidin15_rad  
    
        

def main(args=None):
    rclpy.init(args=args)
    node = ManusToIsaac()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


'''
에이딘 ROS1 Manus to Aidin 변환

glove_data_r_renew[0] = ((-1.0)*(glove_data_r[0]-glove_data_r_bias[0])+15);
glove_data_r_renew[1] = ((1.0)*(glove_data_r[1]-glove_data_r_bias[1])+25);
glove_data_r_renew[2] = ((-1.0)*(glove_data_r[2]-glove_data_r_bias[2])+154);

for(int i = 1 ; i < 5 ; i++) 456 8910 121314 161718 202122
{
	glove_data_r_renew[4*i] = ((-1.0)*(glove_data_r[4*i]-glove_data_r_bias[4*i])+0);  
	glove_data_r_renew[4*i+1] = ((-1.0)*(glove_data_r[4*i+1]-glove_data_r_bias[4*i+1])+50);
	glove_data_r_renew[4*i+2] = ((-1.0)*(glove_data_r[4*i+2]-glove_data_r_bias[4*i+2])+220);
}

traj_TQ[0] = 2.0*glove_data_r_renew[1]*PI/180;  // q1 (MCP F/E)
traj_TQ[1] = 1.2*glove_data_r_renew[0]*PI/180;  // q2 (MCP A/A)
traj_TQ[2] = 1.2*glove_data_r_renew[2]*PI/180;  // q3 (PIP,DIP F/E)

traj_TQ[4] = 0.8*glove_data_r_renew[5]*PI/180;  
traj_TQ[5] = 1.5*glove_data_r_renew[4]*PI/180;
traj_TQ[6] = 0.8*glove_data_r_renew[6]*PI/180; 

traj_TQ[8] = 0.8*glove_data_r_renew[9]*PI/180;  
traj_TQ[9] = 1.5*glove_data_r_renew[8]*PI/180;
traj_TQ[10] = 0.8*glove_data_r_renew[10]*PI/180; 

traj_TQ[12] = 0.8*glove_data_r_renew[13]*PI/180;  
traj_TQ[13] = glove_data_r_renew[12]*PI/180;
traj_TQ[14] = 0.8*glove_data_r_renew[14]*PI/180; 

traj_TQ[16] = 0.8*glove_data_r_renew[17]*PI/180;  
traj_TQ[17] = glove_data_r_renew[16]*PI/180;
traj_TQ[18] = 0.8*glove_data_r_renew[18]*PI/180;
'''