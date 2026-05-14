#!/usr/bin/env python3
"""
camera_tf_broadcaster.py

Publishes the base_link → camera_link transform at 10 Hz to /tf.
Used in fleet-only mode (no Gazebo / no robot_state_publisher).
Publishing to /tf (VOLATILE) avoids TRANSIENT_LOCAL DDS issues.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from typing import Optional


class CameraTFBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("camera_tf_broadcaster")
        self.declare_parameter("robot_namespace", "agv1")
        self._ns: str = str(self.get_parameter("robot_namespace").value)

        # Publish to global /tf with VOLATILE QoS (not namespaced, no TRANSIENT_LOCAL)
        self._tf_pub = self.create_publisher(
            TFMessage, "/tf",
            QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                       durability=DurabilityPolicy.VOLATILE)
        )
        self.create_timer(0.1, self._publish_tf)  # 10 Hz
        self.get_logger().info(
            f"[{self._ns}] camera_tf_broadcaster: {self._ns}/base_link → {self._ns}/camera_link"
        )

    def _publish_tf(self) -> None:
        import time as _time
        from builtin_interfaces.msg import Time as RosTime
        now_ns = int(_time.time() * 1e9)
        t = TransformStamped()
        t.header.stamp = RosTime(sec=now_ns // 10**9, nanosec=now_ns % 10**9)
        t.header.frame_id = f"{self._ns}/base_link"
        t.child_frame_id = f"{self._ns}/camera_link"
        t.transform.translation.x = 0.21
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0375
        t.transform.rotation.w = 1.0
        self._tf_pub.publish(TFMessage(transforms=[t]))


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = CameraTFBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
