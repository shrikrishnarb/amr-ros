#!/usr/bin/env python3
"""
nav2_goal_bridge.py

Bridges fleet_manager's /<ns>/goal_pose topic to Nav2's navigate_to_pose action,
replacing ground_truth_waypoint_follower in the AMR fleet management system.

Key design:
  - waitUntilNav2Active() is called in run(), NOT in __init__, so callbacks
    are not triggered while blocking in __init__.
  - Goals received before Nav2 is active are buffered and executed once active.
  - Re-sent goals for the same destination (fleet_manager re-sends every 5s)
    are ignored so they don't cancel ongoing navigation.
"""

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult as NavResult
except ImportError:
    from nav2_simple_commander.robot_navigator import BasicNavigator, NavigationResult as NavResult


class Nav2GoalBridge:
    def __init__(self) -> None:
        rclpy.init(args=None)

        # Read robot_namespace / initial pose before creating BasicNavigator
        _tmp = Node('_nav2_bridge_param_reader')
        _tmp.declare_parameter('robot_namespace', '')
        _tmp.declare_parameter('initial_x', 0.0)
        _tmp.declare_parameter('initial_y', 0.0)
        ns = str(_tmp.get_parameter('robot_namespace').value).strip()
        initial_x = float(_tmp.get_parameter('initial_x').value)
        initial_y = float(_tmp.get_parameter('initial_y').value)
        _tmp.destroy_node()

        self.ns = ns
        ns_prefix = f'/{ns}' if ns else ''

        self.nav = BasicNavigator(namespace=ns)

        # State
        self._goal_active: bool = False
        self._nav2_active: bool = False
        self._pending_goal: PoseStamped | None = None   # goal buffered before Nav2 is active
        self._current_goal: PoseStamped | None = None   # goal currently being executed
        self._failed_at: float = 0.0                    # monotonic time of last navigation failure
        self._failure_cooldown: float = 6.0             # seconds to ignore same-dest resends after failure

        # Publisher
        self._reached_pub = self.nav.create_publisher(
            Bool, f'{ns_prefix}/goal_reached', 10
        )

        # Subscriber — stores goals; execution deferred until Nav2 is active
        self.nav.create_subscription(
            PoseStamped,
            f'{ns_prefix}/goal_pose',
            self._on_goal,
            10,
        )

        # Poll Nav2 completion at 10 Hz
        self.nav.create_timer(0.1, self._poll_nav2)

        # Publish initial pose now — waitUntilNav2Active() will retry until
        # amcl_pose is received, so it's fine if AMCL isn't active yet here.
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = initial_x
        initial_pose.pose.position.y = initial_y
        initial_pose.pose.orientation.w = 1.0
        self.nav.setInitialPose(initial_pose)

        self.nav.get_logger().info(
            f'[nav2_goal_bridge/{ns}] Initial pose queued: '
            f'x={initial_x:.2f}  y={initial_y:.2f}. '
            f'Waiting for Nav2 to become active (in run())...'
        )

    # ------------------------------------------------------------------
    # Main entry point
    # ------------------------------------------------------------------

    def run(self) -> None:
        # Block here until Nav2 (bt_navigator + AMCL) is fully active.
        # waitUntilNav2Active() internally retries setInitialPose until
        # amcl_pose is received, so AMCL will converge here.
        # Callbacks fire via spin_once() inside waitUntilNav2Active(), but
        # _on_goal only buffers goals rather than calling goToPose(), so
        # there is no nested blocking.
        self.nav.waitUntilNav2Active()
        self._nav2_active = True
        self.nav.get_logger().info(
            f'[nav2_goal_bridge/{self.ns}] Nav2 is active — '
            f'listening on /{self.ns}/goal_pose'
        )

        # Execute any goal that arrived while Nav2 was activating
        if self._pending_goal is not None:
            self.nav.get_logger().info(
                f'[{self.ns}] Executing buffered goal: '
                f'x={self._pending_goal.pose.position.x:.2f}  '
                f'y={self._pending_goal.pose.position.y:.2f}'
            )
            self._execute_goal(self._pending_goal)
            self._pending_goal = None

        rclpy.spin(self.nav)
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_goal(self, msg: PoseStamped) -> None:
        """Receive a goal from fleet_manager."""
        if not self._nav2_active:
            # Nav2 not ready yet — buffer the latest goal (overwrite previous)
            self._pending_goal = msg
            self.nav.get_logger().info(
                f'[{self.ns}] Goal received before Nav2 active, buffered: '
                f'x={msg.pose.position.x:.2f}  y={msg.pose.position.y:.2f}'
            )
            return

        # Nav2 is active — check if this is a re-send of the same goal.
        # fleet_manager re-publishes every 10 s to handle missed messages.
        if self._current_goal is not None:
            dx = abs(msg.pose.position.x - self._current_goal.pose.position.x)
            dy = abs(msg.pose.position.y - self._current_goal.pose.position.y)
            if dx < 0.05 and dy < 0.05:
                if self._goal_active:
                    # Navigation is running — ignore re-send to prevent preempting
                    # BT recovery actions (spin/backup) that are still cleaning up.
                    return
                # Navigation completed (failed) for this destination.
                # Apply a cooldown so BT cleanup finishes before we retry.
                if time.monotonic() - self._failed_at < self._failure_cooldown:
                    return

        self.nav.get_logger().info(
            f'[{self.ns}] New goal: '
            f'x={msg.pose.position.x:.2f}  y={msg.pose.position.y:.2f}  '
            f'frame={msg.header.frame_id}'
        )
        self._execute_goal(msg)

    def _execute_goal(self, msg: PoseStamped) -> None:
        """Cancel any active task and send a new navigation goal."""
        if self._goal_active:
            try:
                self.nav.cancelTask()
            except Exception as exc:
                self.nav.get_logger().warn(
                    f'[{self.ns}] cancelTask failed (ignoring): {exc}'
                )
            self._goal_active = False

        msg.header.stamp = self.nav.get_clock().now().to_msg()
        self._current_goal = msg
        self.nav.goToPose(msg)
        self._goal_active = True

    def _poll_nav2(self) -> None:
        """Periodically check whether Nav2 finished the current goal."""
        if not self._goal_active:
            return
        if not self.nav.isTaskComplete():
            return

        result = self.nav.getResult()
        self._goal_active = False
        self._current_goal = None

        if result == NavResult.SUCCEEDED:
            self.nav.get_logger().info(f'[{self.ns}] Goal reached successfully.')
            self._current_goal = None  # Clear so next goal is always accepted
            self._reached_pub.publish(Bool(data=True))
        else:
            self.nav.get_logger().warn(
                f'[{self.ns}] Navigation ended with result: {result}'
            )
            self._failed_at = time.monotonic()  # Start cooldown before retry
            self._reached_pub.publish(Bool(data=False))


def main() -> None:
    bridge = Nav2GoalBridge()
    bridge.run()


if __name__ == '__main__':
    main()
