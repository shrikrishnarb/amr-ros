#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        self.threshold = 0.5  # Stop if any object closer than 0.5m
        self.hysteresis_margin = 0.05  # For stability (resume at 0.55m)

        self.declare_parameter('robot_namespace', '')
        namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        self.cmd_pub = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, f'/{namespace}/scan', self.scan_callback, 10)
        self.obstacle_flag_pub = self.create_publisher(Bool, f'/{namespace}/obstacle_detected', 10)

        self.obstacle_present = False

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges)]

        # If no valid points (all inf), assume clear path and resume immediately
        if len(valid_ranges) == 0:
            if self.obstacle_present:
                self.obstacle_present = False
                self.obstacle_flag_pub.publish(Bool(data=False))
                self.get_logger().info("No obstacle detected (all inf). Resuming.")
            return

        min_distance = np.min(valid_ranges)

        if self.obstacle_present:
            # Resume only if object is far enough away
            if min_distance > self.threshold + self.hysteresis_margin:
                self.obstacle_present = False
                self.obstacle_flag_pub.publish(Bool(data=False))
                self.get_logger().info("Obstacle cleared. Resuming.")
        else:
            if min_distance < self.threshold:
                self.obstacle_present = True
                self.obstacle_flag_pub.publish(Bool(data=True))
                self.get_logger().info(f"Obstacle at {min_distance:.2f}m! Stopping.")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
