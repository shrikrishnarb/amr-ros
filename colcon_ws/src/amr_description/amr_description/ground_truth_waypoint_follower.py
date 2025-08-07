#!/usr/bin/env python3

import math
import sys
import rclpy
import angles
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Bool


class GtSimpleWaypointFollower(Node):
    def __init__(self):
        super().__init__("gt_simple_waypoint_follower")

        # Declare parameters
        self.target_x = self.declare_parameter("target_x", 0.0).value
        self.target_y = self.declare_parameter("target_y", 0.0).value
        self.max_linear = self.declare_parameter("max_linear", 1.0).value
        self.min_linear = self.declare_parameter("min_linear", 0.05).value
        self.linear_gain = self.declare_parameter("linear_gain", 1.0).value
        self.max_angular = self.declare_parameter("max_angular", 1.57).value
        self.angular_gain = self.declare_parameter("angular_gain", 1.0).value
        self.acc_x_lim = self.declare_parameter("acc_x_lim", 0.2).value
        self.acc_theta_lim = self.declare_parameter("acc_theta_lim", 0.5).value
        self.control_rate = self.declare_parameter("control_rate", 30.0).value
        self.goal_reach_dist = self.declare_parameter("goal_reach_dist", 0.1).value
        self.rotate_move_th = self.declare_parameter("rotate_move_th", 0.1745).value
        self.delay_start_time = self.declare_parameter("delay_start_time", 0.0).value
        self.declare_parameter('robot_namespace', '')
        namespace = self.get_parameter('robot_namespace').get_parameter_value().string_value

        # Pub/sub
        self.pub_cmd_vel = self.create_publisher(Twist, f"/{namespace}/cmd_vel", 10)
        self.sub_gt = self.create_subscription(Odometry, f"/{namespace}/ground_truth", self.cbGroundTruth, 10)
        self.sub_obstacle = self.create_subscription(Bool, f"/{namespace}/obstacle_detected", self.cbObstacle, 10)

        self.obstacle_detected = False
        self.prev_obstacle_state = False

        self.gt_received = False
        self.gt = Odometry()
        self.pre_linear = 0.0
        self.pre_angular = 0.0
        self.dt = 1.0 / self.control_rate

        # Timer
        if self.delay_start_time < 0.0:
            self.get_logger().warning("Negative delay_start_time. Resetting to 0.0")
            self.delay_start_time = 0.0

        self.start_update_timer = self.create_timer(self.delay_start_time, self.start_timer)

    def cbObstacle(self, msg):
        self.obstacle_detected = msg.data

    def cbGroundTruth(self, msg):
        self.gt_received = True
        self.gt = msg

    def start_timer(self):
        self.get_logger().info(f"Starting waypoint follower to ({self.target_x}, {self.target_y})")
        self.start_update_timer.cancel()
        self.update_timer = self.create_timer(self.dt, self.update)

    def update(self):
        if not self.gt_received:
            self.get_logger().info("Waiting for ground truth...", throttle_duration_sec=10.0)
            self.fnMove(0.0, 0.0)
            return

        if self.obstacle_detected:
            if not self.prev_obstacle_state:
                self.get_logger().info("Obstacle detected! Pausing movement.")
                self.pre_linear = 0.0  # Reset velocity memory
                self.pre_angular = 0.0
                self.prev_obstacle_state = True
            self.fnMove(0.0, 0.0)
            return
        else:
            if self.prev_obstacle_state:
                self.get_logger().info("Obstacle cleared. Resuming movement.")
                self.prev_obstacle_state = False

        # Extract robot pose
        x = self.gt.pose.pose.position.x
        y = self.gt.pose.pose.position.y
        q = self.gt.pose.pose.orientation
        yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]

        dx = self.target_x - x
        dy = self.target_y - y
        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        dyaw = angles.normalize_angle(target_yaw - yaw)

        if dist < self.goal_reach_dist:
            self.get_logger().info(f"Reached target at ({self.target_x}, {self.target_y})")
            self.fnMove(0.0, 0.0)
            self.update_timer.cancel()
            self.destroy_node()
            sys.exit(0)
            return

        if abs(dyaw) > self.rotate_move_th:
            linear = 0.0
            angular = self.clamp(self.angular_gain * dyaw, -self.max_angular, self.max_angular)
        else:
            desired_linear = self.clamp(self.linear_gain * dist, self.min_linear, self.max_linear)
            desired_angular = self.clamp(self.angular_gain * dyaw, -self.max_angular, self.max_angular)
            linear = self.adjust_acceleration(desired_linear, self.pre_linear, self.acc_x_lim)
            angular = self.adjust_acceleration(desired_angular, self.pre_angular, self.acc_theta_lim)

        self.fnMove(linear, angular)

    def fnMove(self, linear, angular):
        self.pre_linear = linear
        self.pre_angular = angular
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.pub_cmd_vel.publish(twist)

    def clamp(self, val, min_val, max_val):
        return min(max(val, min_val), max_val)

    def adjust_acceleration(self, target, current, limit):
        delta = target - current
        max_delta = limit * self.dt
        if abs(delta) > max_delta:
            delta = math.copysign(max_delta, delta)
        return current + delta


def main(args=None):
    rclpy.init(args=args)
    node = GtSimpleWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
