#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from manus_ros2_msgs.msg import ManusGlove


# ============================================================================
# Manus ergonomics order
# ============================================================================
ERG_ORDER = [
    "ThumbMCPSpread", "ThumbMCPStretch",  "ThumbPIPStretch",  "ThumbDIPStretch",
    "IndexSpread",    "IndexMCPStretch",  "IndexPIPStretch",  "IndexDIPStretch",
    "MiddleSpread",   "MiddleMCPStretch", "MiddlePIPStretch", "MiddleDIPStretch",
    "RingSpread",     "RingMCPStretch",   "RingPIPStretch",   "RingDIPStretch",
    "PinkySpread",    "PinkyMCPStretch",  "PinkyPIPStretch",  "PinkyDIPStretch"
]
NAME2IDX = {name: i for i, name in enumerate(ERG_ORDER)}

# ============================================================================
# Manus 20D -> Aidin 15D mapping
# Current DualArmForceControl hand target order per hand:
# [thumb1,2,3, index1,2,3, middle1,2,3, ring1,2,3, baby1,2,3]
# ============================================================================
MANUS2AIDIN = np.asarray([
    NAME2IDX["ThumbMCPSpread"],  NAME2IDX["ThumbMCPStretch"],  NAME2IDX["ThumbPIPStretch"],
    NAME2IDX["IndexSpread"],     NAME2IDX["IndexMCPStretch"],  NAME2IDX["IndexPIPStretch"],
    NAME2IDX["MiddleSpread"],    NAME2IDX["MiddleMCPStretch"], NAME2IDX["MiddlePIPStretch"],
    NAME2IDX["RingSpread"],      NAME2IDX["RingMCPStretch"],   NAME2IDX["RingPIPStretch"],
    NAME2IDX["PinkySpread"],     NAME2IDX["PinkyMCPStretch"],  NAME2IDX["PinkyPIPStretch"],
], dtype=np.int64)

# ============================================================================
# User calibration / mapping parameters
# These produce final 15D hand target vectors [rad] for each hand.
# ============================================================================
LEFT_SCALE = np.array([
    -0.8, -3.0, 2.5,   # thumb
    -0.9,  1.4, 1.5,   # index
    -0.9,  1.4, 1.5,   # middle
    -0.8,  1.4, 1.5,   # ring
    -0.8,  1.4, 1.5,   # baby
], dtype=np.float64)

RIGHT_SCALE = np.array([
     0.8, -1.9, 1.6,   # thumb
     0.7,  1.4, 1.5,   # index
     0.7,  1.4, 1.5,   # middle
     0.6,  1.4, 1.5,   # ring
     0.6,  1.4, 1.5,   # baby
], dtype=np.float64)

LEFT_OFFSET = np.array([
     0.0,  2.0,  0.0,
    -0.08, -0.43, -0.20,
    -0.12, -0.43, -0.20,
    -0.18, -0.58,  0.0,
    -0.22, -0.58,  0.0,
], dtype=np.float64)

RIGHT_OFFSET = np.array([
    0.0,  1.4,  0.0,
    0.0, -0.43, 0.0,
    0.0, -0.43, -0.20,
    0.0, -0.58, 0.0,
    0.0, -0.58, 0.0,
], dtype=np.float64)

LEFT_MASKING = np.array([
    1.0, 1.0, 1.0,   # thumb
    0.0, 1.0, 1.0,   # index
    0.0, 1.0, 1.0,   # middle
    0.0, 1.0, 1.0,   # ring
    0.0, 1.0, 1.0,   # baby
], dtype=np.float64)

RIGHT_MASKING = np.array([
    1.0, 1.0, 1.0,   # thumb
    0.0, 1.0, 1.0,   # index
    0.0, 1.0, 1.0,   # middle
    0.0, 1.0, 1.0,   # ring
    0.0, 1.0, 1.0,   # baby
], dtype=np.float64)


class ManusToHandTargets(Node):
    def __init__(self):
        super().__init__("manus_to_hand_targets")

        # ---------------------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------------------
        self.left_topic = self.declare_parameter("left_glove_topic", "/manus_glove_0").value
        self.right_topic = self.declare_parameter("right_glove_topic", "/manus_glove_1").value
        self.output_topic = self.declare_parameter("output_topic", "/forward_hand_joint_targets").value
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 200.0).value)
        self.min_positive_cmd = float(self.declare_parameter("min_positive_cmd", 0.01).value)
        self.enable_smoothing = bool(self.declare_parameter("enable_smoothing", False).value)
        self.smoothing_alpha = float(self.declare_parameter("smoothing_alpha", 0.2).value)
        self.print_decimation = int(self.declare_parameter("print_decimation", 50).value)

        if self.publish_rate_hz <= 0.0:
            self.publish_rate_hz = 200.0
        self.timer_period = 1.0 / self.publish_rate_hz

        # ---------------------------------------------------------------------
        # State
        # ---------------------------------------------------------------------
        self.left_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)
        self.right_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)

        self.left_ready = False
        self.right_ready = False

        self.prev_output = np.zeros(30, dtype=np.float64)
        self.print_counter = 0

        # ---------------------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------------------
        self.left_sub = self.create_subscription(
            ManusGlove,
            self.left_topic,
            self.left_hand_callback,
            10,
        )
        self.right_sub = self.create_subscription(
            ManusGlove,
            self.right_topic,
            self.right_hand_callback,
            10,
        )

        self.hand_target_pub = self.create_publisher(
            Float64MultiArray,
            self.output_topic,
            10,
        )

        self.timer = self.create_timer(self.timer_period, self.publish_hand_targets)

        self.get_logger().info(
            f"manus_to_hand_targets started | left={self.left_topic} right={self.right_topic} "
            f"-> output={self.output_topic} @ {self.publish_rate_hz:.1f} Hz"
        )

    # =========================================================================
    # Callbacks
    # =========================================================================
    def left_hand_callback(self, msg: ManusGlove) -> None:
        for ergo in msg.ergonomics:
            idx = NAME2IDX.get(ergo.type, None)
            if idx is not None:
                self.left_manus_data[idx] = float(ergo.value)
        self.left_ready = True

    def right_hand_callback(self, msg: ManusGlove) -> None:
        for ergo in msg.ergonomics:
            idx = NAME2IDX.get(ergo.type, None)
            if idx is not None:
                self.right_manus_data[idx] = float(ergo.value)
        self.right_ready = True

    # =========================================================================
    # Conversion
    # =========================================================================
    def manus_to_aidin_qpos(self, manus20_deg: np.ndarray) -> np.ndarray:
        """
        Input:
            manus20_deg: shape (20,), unit = degree
        Output:
            aidin15_rad: shape (15,), unit = rad

        Order per hand:
            [thumb1, thumb2, thumb3,
             index1, index2, index3,
             middle1, middle2, middle3,
             ring1, ring2, ring3,
             baby1, baby2, baby3]
        """
        if manus20_deg.shape != (20,):
            raise ValueError(f"Expected (20,), got {manus20_deg.shape}")

        aidin15_deg = manus20_deg[MANUS2AIDIN]
        aidin15_rad = aidin15_deg * np.pi / 180.0
        return aidin15_rad

    def apply_left_mapping(self, q15_rad: np.ndarray) -> np.ndarray:
        out = (q15_rad * LEFT_SCALE + LEFT_OFFSET) * LEFT_MASKING
        return out

    def apply_right_mapping(self, q15_rad: np.ndarray) -> np.ndarray:
        out = (q15_rad * RIGHT_SCALE + RIGHT_OFFSET) * RIGHT_MASKING
        return out

    def apply_lower_bound(self, q30: np.ndarray) -> np.ndarray:
        """
        Current user rule:
        for specific MCP/PIP-like joints, clamp values below 0.01 to 0.01
        """
        out = q30.copy()

        target_idx = [
            1, 2, 4, 5, 7, 8, 10, 11, 13, 14,
            15 + 1, 15 + 2, 15 + 4, 15 + 5, 15 + 7, 15 + 8, 15 + 10, 15 + 11, 15 + 13, 15 + 14
        ]
        out[target_idx] = np.where(out[target_idx] < self.min_positive_cmd,
                                   self.min_positive_cmd,
                                   out[target_idx])
        return out

    def maybe_smooth(self, q30: np.ndarray) -> np.ndarray:
        if not self.enable_smoothing:
            return q30
        alpha = np.clip(self.smoothing_alpha, 0.0, 1.0)
        smoothed = alpha * q30 + (1.0 - alpha) * self.prev_output
        return smoothed

    # =========================================================================
    # Publish
    # =========================================================================
    def publish_hand_targets(self) -> None:
        if not (self.left_ready and self.right_ready):
            return

        try:
            left_q15 = self.manus_to_aidin_qpos(self.left_manus_data)
            right_q15 = self.manus_to_aidin_qpos(self.right_manus_data)

            left_cmd15 = self.apply_left_mapping(left_q15)
            right_cmd15 = self.apply_right_mapping(right_q15)

            hand_targets = np.concatenate([left_cmd15, right_cmd15], axis=0)
            if hand_targets.shape != (30,):
                raise RuntimeError(f"Expected 30-dim output, got {hand_targets.shape}")

            hand_targets = self.apply_lower_bound(hand_targets)
            hand_targets = self.maybe_smooth(hand_targets)

            msg = Float64MultiArray()
            msg.data = hand_targets.astype(np.float64).tolist()
            self.hand_target_pub.publish(msg)

            self.prev_output = hand_targets.copy()

            # decimated debug print
            self.print_counter += 1
            if self.print_counter % max(1, self.print_decimation) == 0:
                np.set_printoptions(precision=4, suppress=True)
                left_dbg = hand_targets[:15].reshape(5, 3)
                right_dbg = hand_targets[15:].reshape(5, 3)

                print("\n[left_hand_target_15]  order=(thumb,index,middle,ring,baby) x (j1,j2,j3)")
                print(left_dbg)
                print("[right_hand_target_15] order=(thumb,index,middle,ring,baby) x (j1,j2,j3)")
                print(right_dbg)

        except Exception as e:
            self.get_logger().error(f"publish_hand_targets failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ManusToHandTargets()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()