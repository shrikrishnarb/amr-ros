#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class BatterySim(Node):
    """Publishes /<ns>/battery_state and /<ns>/battery_low.
       Subscribes to /<ns>/odom and /<ns>/charger_contact (optional)."""

    def __init__(self):
        super().__init__('battery_sim')

        # -------- Namespace parameter (same pattern as your odom_sim_filter) --------
        self.declare_parameter('robot_namespace', 'default_namespace')
        self.robot_namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value
        self.get_logger().info(f"Robot namespace is: {self.robot_namespace}")

        # -------- Parameters (defaults; override from launch if you want) --------
        # Topics (with leading slash)
        self.declare_parameter('battery_topic', f"/{self.robot_namespace}/battery_state")
        self.declare_parameter('battery_low_topic', f"/{self.robot_namespace}/battery_low")
        self.declare_parameter('odom_topic', f"/{self.robot_namespace}/odom")
        # Optional contact topic (leave default; you can publish to it from charger monitor)
        self.declare_parameter('charger_contact_topic', f"/{self.robot_namespace}/charger_contact")

        # Frames (NO leading slash)
        self.declare_parameter('frame_id', f"{self.robot_namespace}/base_link")

        # Battery model numbers
        self.declare_parameter('initial_charge', 1.0)            # 0..1
        self.declare_parameter('publish_rate_hz', 2.0)           # Hz
        self.declare_parameter('drain_per_meter', 0.005)         # fraction per meter
        self.declare_parameter('drain_per_second_idle', 0.0002)  # fraction per second
        self.declare_parameter('charge_per_second', 0.02)        # fraction per second while charging
        self.declare_parameter('nominal_voltage', 24.0)          # V
        self.declare_parameter('design_capacity_Ah', 20.0)       # Ah
        self.declare_parameter('battery_low_threshold', 0.2)     # 20%

        # -------- Read parameters --------
        self.battery_topic = self.get_parameter('battery_topic').get_parameter_value().string_value
        self.battery_low_topic = self.get_parameter('battery_low_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.charger_contact_topic = self.get_parameter('charger_contact_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.charge = float(self.get_parameter('initial_charge').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.drain_per_meter = float(self.get_parameter('drain_per_meter').value)
        self.drain_per_second_idle = float(self.get_parameter('drain_per_second_idle').value)
        self.charge_per_second = float(self.get_parameter('charge_per_second').value)
        self.nominal_voltage = float(self.get_parameter('nominal_voltage').value)
        self.design_capacity_Ah = float(self.get_parameter('design_capacity_Ah').value)
        self.battery_low_threshold = float(self.get_parameter('battery_low_threshold').value)

        # -------- Internal state --------
        self.last_pose: Optional[tuple[float, float]] = None
        self.accum_distance = 0.0
        self.last_time: Optional[Time] = None
        self.charging = False

        # -------- I/O --------
        self.pub_batt = self.create_publisher(BatteryState, self.battery_topic, 10)
        self.pub_low = self.create_publisher(Bool, self.battery_low_topic, 1)
        self.sub_odom = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 50)
        # charger_contact is optional; if no one publishes, node will never charge
        self.sub_charger = self.create_subscription(Bool, self.charger_contact_topic, self._on_charger_contact, 10)

        # Timer for periodic publishing
        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"battery_topic='{self.battery_topic}', odom_topic='{self.odom_topic}', "
            f"charger_contact_topic='{self.charger_contact_topic}', frame_id='{self.frame_id}'"
        )

    # -------- Callbacks --------
    def _on_charger_contact(self, msg: Bool):
        self.charging = bool(msg.data)

    def _on_odom(self, odom: Odometry):
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y
        t = odom.header.stamp

        if self.last_pose is not None:
            dx = x - self.last_pose[0]
            dy = y - self.last_pose[1]
            self.accum_distance += math.hypot(dx, dy)

        self.last_pose = (x, y)
        # Use odom timestamps to respect sim time
        self.last_time = Time.from_msg(t)

    def _on_timer(self):
        now = self.get_clock().now()

        if self.last_time is None:
            # No odom yet; still publish with idle drain against wall clock
            self.last_time = now

        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt < 0.0:
            # Clock reset; re-sync
            self.last_time = now
            return

        # Update charge
        if self.charging:
            self.charge += self.charge_per_second * dt
        else:
            drain = self.accum_distance * self.drain_per_meter + self.drain_per_second_idle * dt
            self.charge -= drain

        # Clamp and reset
        self.charge = max(0.0, min(1.0, self.charge))
        self.accum_distance = 0.0
        self.last_time = now

        # Publish BatteryState
        msg = BatteryState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.frame_id

        # Simple linear mapping (tweak as needed)
        msg.voltage = float(self.nominal_voltage * (0.1 + 0.9 * self.charge))
        msg.current = (0.5 if self.charging else -0.5) * self.design_capacity_Ah  # placeholder current
        msg.charge = float(self.design_capacity_Ah * self.charge)     # Ah remaining
        msg.capacity = msg.charge
        msg.design_capacity = float(self.design_capacity_Ah)          # Ah
        msg.percentage = float(self.charge)                           # 0..1
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING
            if self.charging else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        )
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True

        self.pub_batt.publish(msg)

        # Publish low-battery Bool
        low = Bool()
        low.data = self.charge <= self.battery_low_threshold
        self.pub_low.publish(low)


def main():
    rclpy.init()
    node = BatterySim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
