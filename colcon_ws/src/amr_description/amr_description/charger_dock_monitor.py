#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def hypot2(a: float, b: float) -> float:
    return math.sqrt(a * a + b * b)


class ChargerDockMonitor(Node):
    """Publishes /<ns>/charger_contact=True when robot is near a charger center."""

    def __init__(self):
        super().__init__('charger_dock_monitor')

        # -------- Namespace parameter (same pattern as your odom_sim_filter) --------
        self.declare_parameter('robot_namespace', 'default_namespace')
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.get_logger().info(f"Robot namespace is: {self.robot_namespace}")

        # -------- Parameters --------
        # Topics (with leading slash)
        self.declare_parameter('odom_topic', f"/{self.robot_namespace}/odom")
        self.declare_parameter('contact_topic', f"/{self.robot_namespace}/charger_contact")

        # Charger pad centers in world (flattened list: [x1, y1, x2, y2, ...])
        self.declare_parameter('charger_centers_xy', [-20.0, -10.0, 20.0, -10.0])

        # Detection tuning
        self.declare_parameter('enter_radius', 0.8)          # m
        self.declare_parameter('exit_radius', 1.1)           # m (for hysteresis)
        self.declare_parameter('min_dock_dwell_sec', 0.5)    # s
        self.declare_parameter('max_linear_speed', 0.15)     # m/s
        self.declare_parameter('publish_rate_hz', 5.0)       # Hz

        # -------- Read parameters --------
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.contact_topic = self.get_parameter('contact_topic').get_parameter_value().string_value

        centers_list = list(self.get_parameter('charger_centers_xy').value)
        if len(centers_list) < 2 or len(centers_list) % 2 != 0:
            self.get_logger().warn("Invalid charger_centers_xy; using defaults [-20,-10, 20,-10]")
            centers_list = [-20.0, -10.0, 20.0, -10.0]
        self.chargers: List[Tuple[float, float]] = [
            (centers_list[i], centers_list[i + 1]) for i in range(0, len(centers_list), 2)
        ]

        self.enter_radius = float(self.get_parameter('enter_radius').value)
        self.exit_radius = float(self.get_parameter('exit_radius').value)
        self.min_dock_dwell = float(self.get_parameter('min_dock_dwell_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        # -------- State --------
        self._odom: Optional[Odometry] = None
        self._entered_time: Optional[Time] = None
        self._docked: bool = False

        # -------- I/O --------
        self.pub_contact = self.create_publisher(Bool, self.contact_topic, 10)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 50)

        # Timer
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"odom_topic='{self.odom_topic}', contact_topic='{self.contact_topic}', "
            f"enter_r={self.enter_radius:.2f}, exit_r={self.exit_radius:.2f}, "
            f"dwell={self.min_dock_dwell:.2f}s, max_v={self.max_linear_speed:.2f} m/s"
        )

    # -------- Callbacks --------
    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _on_timer(self):
        contact = Bool()
        contact.data = False

        if self._odom is None:
            self.pub_contact.publish(contact)
            return

        # Position & speed
        x = self._odom.pose.pose.position.x
        y = self._odom.pose.pose.position.y
        vx = self._odom.twist.twist.linear.x
        vy = self._odom.twist.twist.linear.y
        speed = hypot2(vx, vy)

        # Distance to closest charger
        closest_dist = min(hypot2(x - cx, y - cy) for (cx, cy) in self.chargers)

        now = self.get_clock().now()

        if self._docked:
            if closest_dist > self.exit_radius:
                self._docked = False
                self._entered_time = None
            else:
                contact.data = True
        else:
            if closest_dist <= self.enter_radius and speed <= self.max_linear_speed:
                if self._entered_time is None:
                    self._entered_time = now
                else:
                    dwell = (now - self._entered_time).nanoseconds * 1e-9
                    if dwell >= self.min_dock_dwell:
                        self._docked = True
                        contact.data = True
            else:
                self._entered_time = None

        self.pub_contact.publish(contact)


def main():
    rclpy.init()
    node = ChargerDockMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
