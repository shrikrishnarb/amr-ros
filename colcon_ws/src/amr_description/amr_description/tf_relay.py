#!/usr/bin/env python3
"""
tf_relay.py
Relays global /tf and /tf_static to namespaced /{ns}/tf and /{ns}/tf_static.

Needed because:
  - nav2_bringup with use_namespace:=True adds remapping '/tf' → 'tf', so
    all Nav2 nodes (AMCL, costmaps, etc.) read from /{ns}/tf and /{ns}/tf_static.
  - display.launch.py / spawn_agv.launch.py start odom_sim_filter and
    robot_state_publisher without TF remapping, so they publish to global /tf
    and /tf_static.
  - This relay bridges the gap without changing any existing launch files.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_msgs.msg import TFMessage


DYNAMIC_QOS = QoSProfile(depth=100)
STATIC_QOS = QoSProfile(
    depth=1,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    reliability=QoSReliabilityPolicy.RELIABLE,
)


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        self.declare_parameter('robot_namespace', '')
        ns = self.get_parameter('robot_namespace').get_parameter_value().string_value.strip()

        if not ns:
            raise RuntimeError('[tf_relay] robot_namespace parameter must be set')

        self._tf_pub = self.create_publisher(TFMessage, f'/{ns}/tf', DYNAMIC_QOS)
        self._tf_static_pub = self.create_publisher(TFMessage, f'/{ns}/tf_static', STATIC_QOS)

        self.create_subscription(TFMessage, '/tf', self._on_tf, DYNAMIC_QOS)
        # Transient local on subscriber so we receive the last-published static TFs
        # even if we start after robot_state_publisher.
        self.create_subscription(TFMessage, '/tf_static', self._on_tf_static, STATIC_QOS)

        self.get_logger().info(
            f'[tf_relay/{ns}] Relaying /tf → /{ns}/tf  |  /tf_static → /{ns}/tf_static'
        )

    def _on_tf(self, msg: TFMessage) -> None:
        self._tf_pub.publish(msg)

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._tf_static_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TFRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
