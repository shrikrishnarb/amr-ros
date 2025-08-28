#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Quaternion

# Nav2 Simple Commander: BasicNavigator is provided from robot_navigator
# Result enum has been named TaskResult (newer) or NavigationResult (older) in different releases.
try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult as NavResult
except ImportError:
    from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult as NavResult


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class Nav2Navigator:
    def __init__(self) -> None:
        # Init ROS and create the commander (it is an rclpy Node)
        rclpy.init(args=None)
        self.navigator = BasicNavigator()

        # --- Parameters (declared on this node) ---
        self.robot_namespace: str = self._declare_get('robot_namespace', '')
        self.goal_x: float = self._declare_get('target_x', 0.0)
        self.goal_y: float = self._declare_get('target_y', 0.0)
        self.goal_yaw: float = self._declare_get('target_yaw', 0.0)  # radians

        self.global_frame: str = self._declare_get('global_frame', 'map')
        self.base_frame: str = self._declare_get('base_frame', 'base_link')

        self.use_initial_pose: bool = self._declare_get('use_initial_pose', False)
        self.init_x: float = self._declare_get('initial_x', 0.0)
        self.init_y: float = self._declare_get('initial_y', 0.0)
        self.init_yaw: float = self._declare_get('initial_yaw', 0.0)

        self.obstacle_topic_name: str = self._declare_get('obstacle_topic', 'obstacle_detected')
        self.obstacle_wait_replan_sec: float = self._declare_get('obstacle_wait_replan_sec', 5.0)
        self.clear_costmaps_before_replan: bool = self._declare_get('clear_costmaps_before_replan', True)

        # Topic (optional manual namespacing parameter)
        ns_prefix = f"/{self.robot_namespace}" if self.robot_namespace else ""
        self.obstacle_topic = f"{ns_prefix}/{self.obstacle_topic_name}".replace('//', '/')

        # --- State ---
        self.obstacle_detected: bool = False
        self.blocked_since: Optional[rclpy.time.Time] = None
        self.goal_active: bool = False
        self.goal_sent_once: bool = False

        # --- Subscription ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.navigator.create_subscription(Bool, self.obstacle_topic, self._obstacle_cb, qos)

        # --- Initial pose then activate Nav2 (order per Nav2 examples) ---
        # See docs: set initial pose, then wait until Nav2 active if autostarted.
        if self.use_initial_pose:
            self._set_initial_pose(self.init_x, self.init_y, self.init_yaw)

        self._activate_nav2()

        # --- Goal Pose ---
        self.goal_pose = self._build_pose(self.goal_x, self.goal_y, self.goal_yaw, self.global_frame)

    # ---------- Helpers ----------

    def _declare_get(self, name, default):
        self.navigator.declare_parameter(name, default)
        return self.navigator.get_parameter(name).value

    def _activate_nav2(self):
        self.navigator.get_logger().info("Waiting for Nav2 to become active...")
        # If not autostart, you could use lifecycleStartup(). For autostarted stacks use waitUntilNav2Active().
        self.navigator.waitUntilNav2Active()
        self.navigator.get_logger().info("Nav2 is active.")

    def _set_initial_pose(self, x: float, y: float, yaw: float):
        pose = self._build_pose(x, y, yaw, self.global_frame)
        self.navigator.setInitialPose(pose)
        self.navigator.get_logger().info(
            f"Initial pose set to x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad in '{self.global_frame}'"
        )

    def _build_pose(self, x: float, y: float, yaw: float, frame: str) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation = yaw_to_quaternion(float(yaw))
        return pose

    def _obstacle_cb(self, msg: Bool):
        prev = self.obstacle_detected
        self.obstacle_detected = bool(msg.data)

        if self.obstacle_detected and not prev:
            self.navigator.get_logger().info("Obstacle detected → canceling current task.")
            self._cancel_if_active()
            self.blocked_since = self.navigator.get_clock().now()

        elif not self.obstacle_detected and prev:
            self.navigator.get_logger().info("Obstacle cleared → resuming navigation.")
            self.blocked_since = None
            if not self.goal_active:
                self._send_goal(self.goal_pose)

    def _cancel_if_active(self):
        if self.goal_active:
            try:
                self.navigator.cancelTask()
            except Exception as e:
                self.navigator.get_logger().warn(f"Failed to cancel task: {e}")
        self.goal_active = False

    def _force_replan(self):
        if self.clear_costmaps_before_replan:
            self.navigator.get_logger().info("Clearing Nav2 costmaps before replanning...")
            try:
                self.navigator.clearAllCostmaps()
            except Exception as e:
                self.navigator.get_logger().warn(f"Failed to clear costmaps: {e}")

        self.navigator.get_logger().info("Re-sending goal to force replanning...")
        self._send_goal(self.goal_pose)

    def _send_goal(self, pose: PoseStamped):
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.navigator.goToPose(pose)
        self.goal_active = True
        self.goal_sent_once = True
        self.navigator.get_logger().info(
            f"Navigating to ({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}) in '{pose.header.frame_id}'"
        )

    # ---------- Main loop ----------

    def run(self):
        # If no obstacle present at start, send the goal; else wait.
        if not self.obstacle_detected:
            self._send_goal(self.goal_pose)
        else:
            self.navigator.get_logger().info("Start delayed: obstacle present.")

        replan_triggered_on_this_block = False

        try:
            while rclpy.ok():
                # Spin the navigator node (it is an rclpy Node)
                rclpy.spin_once(self.navigator, timeout_sec=0.1)

                # If obstacle is present, optionally trigger a replan after a timeout
                if self.obstacle_detected:
                    if self.blocked_since is None:
                        self.blocked_since = self.navigator.get_clock().now()
                        replan_triggered_on_this_block = False
                    else:
                        elapsed = self.navigator.get_clock().now() - self.blocked_since
                        # Compare using nanoseconds for robustness
                        if (elapsed.nanoseconds > int(self.obstacle_wait_replan_sec * 1e9)
                                and not replan_triggered_on_this_block):
                            self.navigator.get_logger().info(
                                f"Obstacle persisted for > {self.obstacle_wait_replan_sec:.1f}s → forcing replanning."
                            )
                            self._force_replan()
                            replan_triggered_on_this_block = True
                    continue

                # If navigating, check completion
                if self.goal_active and self.navigator.isTaskComplete():
                    result = self.navigator.getResult()
                    self.goal_active = False

                    if result == NavResult.SUCCEEDED:
                        self.navigator.get_logger().info("Goal reached successfully ✅")
                        break
                    elif result == NavResult.CANCELED:
                        self.navigator.get_logger().info("Goal canceled (likely due to obstacle).")
                    else:
                        self.navigator.get_logger().warn(f"Navigation failed with result code: {result}")

        finally:
            # Clean shutdown
            self.navigator.get_logger().info("Shutting down navigator.")
            rclpy.shutdown()


def main():
    app = Nav2Navigator()
    app.run()


if __name__ == "__main__":
    main()
