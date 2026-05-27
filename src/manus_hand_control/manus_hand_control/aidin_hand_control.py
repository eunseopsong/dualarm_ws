#!/usr/bin/env python3

from __future__ import annotations

import math
from typing import Iterable

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

try:
    from manus_ros2_msgs.msg import ManusGlove
except ImportError as exc:  # pragma: no cover - runtime environment check
    ManusGlove = None
    MANUS_IMPORT_ERROR = exc
else:
    MANUS_IMPORT_ERROR = None

try:
    from dualarm_forcecon_interfaces.srv import SetControlMode
except ImportError:  # pragma: no cover - optional runtime integration
    SetControlMode = None


ERG_ORDER = [
    "ThumbMCPSpread",
    "ThumbMCPStretch",
    "ThumbPIPStretch",
    "ThumbDIPStretch",
    "IndexSpread",
    "IndexMCPStretch",
    "IndexPIPStretch",
    "IndexDIPStretch",
    "MiddleSpread",
    "MiddleMCPStretch",
    "MiddlePIPStretch",
    "MiddleDIPStretch",
    "RingSpread",
    "RingMCPStretch",
    "RingPIPStretch",
    "RingDIPStretch",
    "PinkySpread",
    "PinkyMCPStretch",
    "PinkyPIPStretch",
    "PinkyDIPStretch",
]

NAME2IDX = {name: idx for idx, name in enumerate(ERG_ORDER)}

# Output order: [thumb1..3, index1..3, middle1..3, ring1..3, pinky1..3]
MANUS2AIDIN = np.asarray(
    [
        NAME2IDX["ThumbMCPSpread"],
        NAME2IDX["ThumbMCPStretch"],
        NAME2IDX["ThumbPIPStretch"],
        NAME2IDX["IndexSpread"],
        NAME2IDX["IndexMCPStretch"],
        NAME2IDX["IndexPIPStretch"],
        NAME2IDX["MiddleSpread"],
        NAME2IDX["MiddleMCPStretch"],
        NAME2IDX["MiddlePIPStretch"],
        NAME2IDX["RingSpread"],
        NAME2IDX["RingMCPStretch"],
        NAME2IDX["RingPIPStretch"],
        NAME2IDX["PinkySpread"],
        NAME2IDX["PinkyMCPStretch"],
        NAME2IDX["PinkyPIPStretch"],
    ],
    dtype=np.int64,
)

LEFT_SCALE = np.array(
    [
        -0.8,
        -3.0,
        2.5,
        -0.9,
        1.4,
        1.5,
        -0.9,
        1.4,
        1.5,
        -0.8,
        1.4,
        1.5,
        -0.8,
        1.4,
        1.5,
    ],
    dtype=np.float64,
)

RIGHT_SCALE = np.array(
    [
        0.8,
        -1.9,
        1.6,
        0.7,
        1.4,
        1.5,
        0.7,
        1.4,
        1.5,
        0.6,
        1.4,
        1.5,
        0.6,
        1.4,
        1.5,
    ],
    dtype=np.float64,
)

LEFT_OFFSET = np.array(
    [
        0.0,
        2.0,
        0.0,
        -0.08,
        -0.43,
        -0.20,
        -0.12,
        -0.43,
        -0.20,
        -0.18,
        -0.58,
        0.0,
        -0.22,
        -0.58,
        0.0,
    ],
    dtype=np.float64,
)

RIGHT_OFFSET = np.array(
    [
        0.0,
        1.4,
        0.0,
        0.0,
        -0.43,
        0.0,
        0.0,
        -0.43,
        -0.20,
        0.0,
        -0.58,
        0.0,
        0.0,
        -0.58,
        0.0,
    ],
    dtype=np.float64,
)

LEFT_MASKING = np.array(
    [1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0],
    dtype=np.float64,
)

RIGHT_MASKING = np.array(
    [1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0],
    dtype=np.float64,
)

REORDER_THUMB_INDEX_MIDDLE_RING_PINKY_TO_LEGACY_REVERSE = np.array(
    [12, 13, 14, 9, 10, 11, 6, 7, 8, 3, 4, 5, 0, 1, 2],
    dtype=np.int64,
)

NON_SPREAD_INDICES_PER_HAND = np.array([1, 2, 4, 5, 7, 8, 10, 11, 13, 14], dtype=np.int64)


def _array_param(node: Node, name: str, default: np.ndarray) -> np.ndarray:
    node.declare_parameter(name, default.tolist())
    values = node.get_parameter(name).value
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != default.shape:
        raise ValueError(f"parameter '{name}' must have {default.size} values")
    return arr


def _bool_param(node: Node, name: str, default: bool) -> bool:
    node.declare_parameter(name, default)
    return bool(node.get_parameter(name).value)


def _float_param(node: Node, name: str, default: float) -> float:
    node.declare_parameter(name, default)
    return float(node.get_parameter(name).value)


def _string_param(node: Node, name: str, default: str) -> str:
    node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


class AidinHandControl(Node):
    def __init__(self) -> None:
        super().__init__("aidin_hand_control")

        if ManusGlove is None:
            raise RuntimeError(
                "manus_ros2_msgs is not available. Source the MANUS workspace first, "
                "for example: source ~/manus_ws/install/setup.bash"
            ) from MANUS_IMPORT_ERROR

        self.left_scale = _array_param(self, "left_scale", LEFT_SCALE)
        self.right_scale = _array_param(self, "right_scale", RIGHT_SCALE)
        self.left_offset = _array_param(self, "left_offset", LEFT_OFFSET)
        self.right_offset = _array_param(self, "right_offset", RIGHT_OFFSET)
        self.left_mask = _array_param(self, "left_mask", LEFT_MASKING)
        self.right_mask = _array_param(self, "right_mask", RIGHT_MASKING)

        self.left_glove_topic = _string_param(self, "left_glove_topic", "/manus_glove_0")
        self.right_glove_topic = _string_param(self, "right_glove_topic", "/manus_glove_1")
        self.output_topic = _string_param(self, "output_topic", "/forward_hand_joint_targets")
        self.publish_rate_hz = _float_param(self, "publish_rate_hz", 200.0)
        self.min_non_spread_position = _float_param(self, "min_non_spread_position", 0.01)
        self.debug_print = _bool_param(self, "debug_print", False)
        self.print_manus_input = _bool_param(self, "print_manus_input", False)
        self.print_interval_s = _float_param(self, "print_interval_s", 0.5)
        self.legacy_reverse_finger_order = _bool_param(self, "legacy_reverse_finger_order", False)

        self.auto_set_forcecon_mode = _bool_param(self, "auto_set_forcecon_mode", False)
        self.forcecon_mode_service = _string_param(self, "forcecon_mode_service", "/change_control_mode")
        self.forcecon_arm_mode = _string_param(self, "forcecon_arm_mode", "inverse")
        self.forcecon_hand_mode = _string_param(self, "forcecon_hand_mode", "forward")

        self.left_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)
        self.right_manus_data = np.zeros(len(ERG_ORDER), dtype=np.float64)
        self.left_seen = False
        self.right_seen = False
        self.left_msg_count = 0
        self.right_msg_count = 0
        self.left_last_msg_ns = 0
        self.right_last_msg_ns = 0
        self.left_battery_percentage = None
        self.right_battery_percentage = None
        self.left_transmission_strength = None
        self.right_transmission_strength = None
        self.last_print_ns = 0

        self.left_subscription = self.create_subscription(
            ManusGlove,
            self.left_glove_topic,
            self._left_hand_callback,
            1,
        )
        self.right_subscription = self.create_subscription(
            ManusGlove,
            self.right_glove_topic,
            self._right_hand_callback,
            1,
        )
        self.publisher = self.create_publisher(Float64MultiArray, self.output_topic, 10)

        period_s = 1.0 / max(self.publish_rate_hz, 1.0)
        self.create_timer(period_s, self._publish_hand_targets)

        self.mode_client = None
        if self.auto_set_forcecon_mode:
            self._request_forcecon_mode()

        self.get_logger().info(
            f"publishing AIDIN hand targets to {self.output_topic} at {self.publish_rate_hz:.1f} Hz"
        )

    def _left_hand_callback(self, msg: ManusGlove) -> None:
        self._store_ergonomics(msg.ergonomics, self.left_manus_data)
        self.left_seen = True
        self.left_msg_count += 1
        self.left_last_msg_ns = self.get_clock().now().nanoseconds
        self.left_battery_percentage = self._optional_int_field(msg, "battery_percentage")
        self.left_transmission_strength = self._optional_int_field(msg, "transmission_strength")

    def _right_hand_callback(self, msg: ManusGlove) -> None:
        self._store_ergonomics(msg.ergonomics, self.right_manus_data)
        self.right_seen = True
        self.right_msg_count += 1
        self.right_last_msg_ns = self.get_clock().now().nanoseconds
        self.right_battery_percentage = self._optional_int_field(msg, "battery_percentage")
        self.right_transmission_strength = self._optional_int_field(msg, "transmission_strength")

    @staticmethod
    def _store_ergonomics(ergonomics: Iterable, out: np.ndarray) -> None:
        for ergo in ergonomics:
            idx = NAME2IDX.get(ergo.type)
            if idx is not None and math.isfinite(float(ergo.value)):
                out[idx] = float(ergo.value)

    def _publish_hand_targets(self) -> None:
        left = self._manus_to_forcecon_hand(
            self.left_manus_data,
            self.left_scale,
            self.left_offset,
            self.left_mask,
        )
        right = self._manus_to_forcecon_hand(
            self.right_manus_data,
            self.right_scale,
            self.right_offset,
            self.right_mask,
        )

        hand_targets = np.concatenate([left, right])
        msg = Float64MultiArray()
        msg.data = hand_targets.tolist()
        self.publisher.publish(msg)

        if self.debug_print:
            np.set_printoptions(precision=4, suppress=True)
            self.get_logger().info(
                "left/right forcecon hand targets\n"
                f"L={left.reshape(5, 3)}\n"
                f"R={right.reshape(5, 3)}"
            )

        if self.print_manus_input:
            self._maybe_print_manus_input()

    def _maybe_print_manus_input(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(max(self.print_interval_s, 0.05) * 1e9)
        if now_ns - self.last_print_ns < interval_ns:
            return

        self.last_print_ns = now_ns
        np.set_printoptions(precision=2, suppress=True)
        left_age_s = self._age_s(now_ns, self.left_last_msg_ns)
        right_age_s = self._age_s(now_ns, self.right_last_msg_ns)
        left_publishers = self.count_publishers(self.left_glove_topic)
        right_publishers = self.count_publishers(self.right_glove_topic)
        self.get_logger().info(
            "MANUS ergonomics deg "
            "(rows: thumb/index/middle/ring/pinky, cols: spread/mcp/pip/dip)\n"
            f"L topic={self.left_glove_topic} publishers={left_publishers} "
            f"seen={self.left_seen} count={self.left_msg_count} age_s={left_age_s:.2f} "
            f"battery={self._fmt_optional(self.left_battery_percentage, '%')} "
            f"rssi={self._fmt_optional(self.left_transmission_strength, '')}:\n"
            f"{self.left_manus_data.reshape(5, 4)}\n"
            f"R topic={self.right_glove_topic} publishers={right_publishers} "
            f"seen={self.right_seen} count={self.right_msg_count} age_s={right_age_s:.2f} "
            f"battery={self._fmt_optional(self.right_battery_percentage, '%')} "
            f"rssi={self._fmt_optional(self.right_transmission_strength, '')}:\n"
            f"{self.right_manus_data.reshape(5, 4)}"
        )

    @staticmethod
    def _optional_int_field(msg, field_name: str):
        return int(getattr(msg, field_name)) if hasattr(msg, field_name) else None

    @staticmethod
    def _fmt_optional(value, suffix: str) -> str:
        if value is None:
            return "n/a"
        return f"{value}{suffix}"

    @staticmethod
    def _age_s(now_ns: int, last_ns: int) -> float:
        if last_ns <= 0:
            return math.inf
        return max(0.0, (now_ns - last_ns) * 1e-9)

    def _manus_to_forcecon_hand(
        self,
        manus20_deg: np.ndarray,
        scale: np.ndarray,
        offset: np.ndarray,
        mask: np.ndarray,
    ) -> np.ndarray:
        aidin15_rad = manus20_deg[MANUS2AIDIN] * np.pi / 180.0
        aidin15 = (aidin15_rad * scale + offset) * mask
        forcecon_order = aidin15.copy()
        if self.legacy_reverse_finger_order:
            forcecon_order = forcecon_order[REORDER_THUMB_INDEX_MIDDLE_RING_PINKY_TO_LEGACY_REVERSE].copy()
        idx = NON_SPREAD_INDICES_PER_HAND
        forcecon_order[idx] = np.maximum(forcecon_order[idx], self.min_non_spread_position)
        return forcecon_order

    def _request_forcecon_mode(self) -> None:
        if SetControlMode is None:
            self.get_logger().warn(
                "auto_set_forcecon_mode requested, but dualarm_forcecon_interfaces is not available"
            )
            return

        self.mode_client = self.create_client(SetControlMode, self.forcecon_mode_service)
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"forcecon mode service not available: {self.forcecon_mode_service}")
            return

        req = SetControlMode.Request()
        req.arm_mode = self.forcecon_arm_mode
        req.hand_mode = self.forcecon_hand_mode
        future = self.mode_client.call_async(req)
        future.add_done_callback(self._handle_mode_response)

    def _handle_mode_response(self, future) -> None:
        try:
            res = future.result()
        except Exception as exc:  # pragma: no cover - ROS service failure path
            self.get_logger().warn(f"failed to set forcecon mode: {exc}")
            return

        if res.success:
            self.get_logger().info(res.message)
        else:
            self.get_logger().warn(res.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AidinHandControl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
