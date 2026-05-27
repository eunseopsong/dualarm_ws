from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="manus_hand_control",
                executable="aidin_hand_control",
                name="aidin_hand_control",
                output="screen",
                parameters=[
                    {
                        "left_glove_topic": "/manus_glove_0",
                        "right_glove_topic": "/manus_glove_1",
                        "output_topic": "/forward_hand_joint_targets",
                        "publish_rate_hz": 200.0,
                        "legacy_reverse_finger_order": False,
                        "auto_set_forcecon_mode": False,
                        "forcecon_arm_mode": "inverse",
                        "forcecon_hand_mode": "forward",
                    }
                ],
            )
        ]
    )
