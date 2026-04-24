#!/usr/bin/env python3
"""
fleet_manager_ai.py

Semantically-aware fleet/task manager for the amr_vision stack.

Extends the original fleet_manager FSM (IDLE → TO_PICKUP → LOADING →
TO_DROPOFF → UNLOADING, with charging detour) with a semantic perception
layer driven by camera_detection_node:

  - person  → cancel Nav2 goal, stop in place, wait for "clear"
  - pallet  → do NOT cancel goal (Nav2 reroutes via costmap), publish
              reduced cmd_vel so the robot slows while Nav2 plans around it
  - clear   → if previously stopped, re-send last goal and resume

All original FSM logic is preserved unchanged.
"""

import os
import math
import time
import yaml
from typing import Optional

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Float32, String


# ---------------------------------------------------------------------------
# Helpers (identical to fleet_manager.py)
# ---------------------------------------------------------------------------

def dist(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def stamp_now(node: Node) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "map"
    return msg


# ---------------------------------------------------------------------------
# Per-robot state  (CHANGE 4: three new fields added at the bottom)
# ---------------------------------------------------------------------------

class AGVState:
    def __init__(self, ns: str) -> None:
        self.ns = ns
        self.pose: Optional[tuple[float, float]] = None   # (x, y)
        self.on_charger: bool = False
        self.battery: float = 1.0                          # 0..1
        self.state: str = "IDLE"
        self.task: Optional[dict] = None                   # {id, pickup, dropoff, load_time, unload_time}
        self.state_since: float = time.time()
        self.carrying: bool = False

        # --- Semantic perception state (new for FleetManagerAI) ---
        self.semantic_obstacle: str = "clear"
        self.semantic_stop_active: bool = False
        self.last_nav_goal: Optional[PoseStamped] = None   # last goal sent; used to resume after stop


# ---------------------------------------------------------------------------
# Fleet manager  (CHANGE 1: class name; CHANGE 2: node name)
# ---------------------------------------------------------------------------

class FleetManagerAI(Node):
    """
    Minimal fleet/task manager with semantic perception layer.

    Identical to FleetManager except:
      - Subscribes to /{ns}/camera/obstacle_semantic per robot
      - Stops for person, slows for pallet, resumes on clear
      - Logs fleet status with semantic state every 5 s
    """

    def __init__(self) -> None:
        super().__init__("fleet_manager_ai")  # CHANGE 2: node name

        # ----------------------------------------------------------------
        # Parameters (identical to fleet_manager)
        # ----------------------------------------------------------------
        self.declare_parameter("robot_namespaces", ["agv1"])
        self.declare_parameter("tasks_file", "")
        self.declare_parameter("zones_yaml", "")
        self.declare_parameter("tasks_yaml", "")
        self.declare_parameter("battery_low_threshold", 0.20)
        self.declare_parameter("battery_resume_threshold", 0.60)
        self.declare_parameter("goal_reach_dist", 0.25)
        self.declare_parameter("charge_check_period", 1.0)
        self.declare_parameter("load_unload_poll_period", 0.2)
        self.declare_parameter("battery_topic_type", "auto")   # 'auto'|'battery_state'|'float32'
        self.declare_parameter("charger_zone_names", ["charger_1", "charger_2"])

        self.ns_list: list[str] = list(self.get_parameter("robot_namespaces").value)
        self.low_thr: float = float(self.get_parameter("battery_low_threshold").value)
        self.resume_thr: float = float(self.get_parameter("battery_resume_threshold").value)
        self.reach_dist: float = float(self.get_parameter("goal_reach_dist").value)
        self.batt_topic_type: str = str(self.get_parameter("battery_topic_type").value)
        self.charger_zone_names: list[str] = list(self.get_parameter("charger_zone_names").value)

        # Load zones/tasks from YAML if provided, else from params
        tasks_file = str(self.get_parameter("tasks_file").value).strip()
        if tasks_file and os.path.exists(tasks_file):
            with open(tasks_file, "r") as f:
                data = yaml.safe_load(f) or {}
            self.zones: dict = data.get("zones", {}) or {}
            self.task_queue: list[dict] = data.get("tasks", []) or []
        else:
            zones_yaml = str(self.get_parameter("zones_yaml").value or "").strip()
            tasks_yaml = str(self.get_parameter("tasks_yaml").value or "").strip()

            self.zones = yaml.safe_load(zones_yaml) if zones_yaml else {}
            if not isinstance(self.zones, dict):
                self.get_logger().warn("zones_yaml did not parse to a dict; ignoring.")
                self.zones = {}

            parsed_tasks = yaml.safe_load(tasks_yaml) if tasks_yaml else []
            if parsed_tasks is None:
                parsed_tasks = []
            if not isinstance(parsed_tasks, list):
                self.get_logger().warn("tasks_yaml did not parse to a list; ignoring.")
                parsed_tasks = []
            self.task_queue = parsed_tasks

        # ----------------------------------------------------------------
        # Per-AGV state (CHANGE 4: AGVState now carries semantic fields)
        # ----------------------------------------------------------------
        self.agv: dict[str, AGVState] = {ns: AGVState(ns) for ns in self.ns_list}

        # ----------------------------------------------------------------
        # Publishers / subscribers per AGV
        # ----------------------------------------------------------------
        self.goal_pub: dict = {}
        self.carry_pub: dict = {}
        self.cmd_vel_pub: dict = {}   # new: used to publish reduced speed on pallet detection

        for ns in self.ns_list:
            self.goal_pub[ns] = self.create_publisher(PoseStamped, f"/{ns}/goal_pose", 10)
            self.carry_pub[ns] = self.create_publisher(Bool, f"/{ns}/carrying_load", 10)
            self.cmd_vel_pub[ns] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)

            self.create_subscription(
                Odometry,
                f"/{ns}/ground_truth",
                lambda msg, ns=ns: self.cb_odom(ns, msg),
                10,
            )
            self.create_subscription(
                Bool,
                f"/{ns}/on_charger",
                lambda msg, ns=ns: self.cb_on_charger(ns, msg),
                10,
            )
            # Battery (identical to fleet_manager)
            if self.batt_topic_type in ("auto", "battery_state"):
                self.create_subscription(
                    BatteryState,
                    f"/{ns}/battery_state",
                    lambda msg, ns=ns: self.cb_batt_state(ns, msg),
                    10,
                )
            if self.batt_topic_type in ("auto", "float32"):
                self.create_subscription(
                    Float32,
                    f"/{ns}/battery_percentage",
                    lambda msg, ns=ns: self.cb_batt_float(ns, msg),
                    10,
                )

            # CHANGE 3: semantic perception subscription (new for FleetManagerAI)
            self.create_subscription(
                String,
                f"/{ns}/camera/obstacle_semantic",
                lambda msg, n=ns: self._semantic_callback(msg, n),
                10,
            )

        # ----------------------------------------------------------------
        # Timers
        # ----------------------------------------------------------------
        self.control_timer = self.create_timer(0.25, self.step)   # main FSM loop

        # CHANGE 8: periodic fleet status log including semantic state
        self.status_timer = self.create_timer(5.0, self._log_fleet_status)

        # Goal re-send tracking (identical to fleet_manager)
        self._goal_last_sent: dict[str, float] = {ns: 0.0 for ns in self.ns_list}
        self._goal_resend_interval: float = 10.0

        self.get_logger().info(
            f"FleetManagerAI started. AGVs: {self.ns_list}, tasks in queue: {len(self.task_queue)}"
        )

    # ====================================================================
    # Callbacks — identical to fleet_manager
    # ====================================================================

    def cb_odom(self, ns: str, msg: Odometry) -> None:
        self.agv[ns].pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def cb_on_charger(self, ns: str, msg: Bool) -> None:
        self.agv[ns].on_charger = bool(msg.data)

    def cb_batt_state(self, ns: str, msg: BatteryState) -> None:
        pct = msg.percentage
        if pct > 1.5:
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    def cb_batt_float(self, ns: str, msg: Float32) -> None:
        pct = msg.data
        if pct > 1.5:
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    # ====================================================================
    # Semantic perception callbacks (CHANGE 5 & 6: new for FleetManagerAI)
    # ====================================================================

    def _semantic_callback(self, msg: String, ns: str) -> None:
        """Receive semantic label from camera_detection_node and react."""
        semantic: str = msg.data
        self.agv[ns].semantic_obstacle = semantic
        self._handle_semantic_change(ns, semantic)

    def _handle_semantic_change(self, ns: str, semantic: str) -> None:
        """Apply semantic-aware navigation behavior on top of the existing FSM.

        Only acts when the robot is actively navigating (TO_PICKUP / TO_DROPOFF).
        All other FSM states are left unaffected.
        """
        agv = self.agv[ns]

        # Semantic layer only applies during active navigation
        if agv.state not in ("TO_PICKUP", "TO_DROPOFF"):
            return

        if semantic == "person" and not agv.semantic_stop_active:
            # Cancel Nav2 goal and stop the robot
            agv.semantic_stop_active = True
            self._cancel_nav_goal(ns)
            self.get_logger().warn(
                f"[FleetAI] {ns}: PERSON detected — STOPPING for safety"
            )

        elif semantic == "pallet":
            # Do NOT cancel the Nav2 goal — let Nav2 reroute via dynamic costmap.
            # Publish a reduced cmd_vel so the robot slows while Nav2 replans.
            slow_twist = Twist()
            slow_twist.linear.x = 0.1    # m/s — reduced from normal navigation speed
            slow_twist.angular.z = 0.0
            self.cmd_vel_pub[ns].publish(slow_twist)
            self.get_logger().info(
                f"[FleetAI] {ns}: PALLET detected — slowing, Nav2 will reroute"
            )

        elif semantic == "clear" and agv.semantic_stop_active:
            # Path is clear again — lift the stop and re-send the last goal
            agv.semantic_stop_active = False
            self.get_logger().info(
                f"[FleetAI] {ns}: path clear — resuming navigation"
            )
            if agv.last_nav_goal is not None:
                # Refresh the stamp so nav2_goal_bridge treats it as a new goal
                agv.last_nav_goal.header.stamp = self.get_clock().now().to_msg()
                self.goal_pub[ns].publish(agv.last_nav_goal)
                self._goal_last_sent[ns] = time.time()
                self.get_logger().info(
                    f"[FleetAI] {ns}: Re-sent last goal to resume navigation"
                )

    def _cancel_nav_goal(self, ns: str) -> None:
        """Cancel active Nav2 navigation by sending a goal at the robot's current pose.

        nav2_goal_bridge calls cancelTask() whenever a new (different-position) goal
        arrives, so publishing the current position triggers BT cancellation.
        If pose is not yet known, goal re-sending is simply suspended (via
        semantic_stop_active) until the semantic label returns to "clear".
        """
        agv = self.agv[ns]
        if agv.pose is None:
            self.get_logger().warn(
                f"[FleetAI] {ns}: Cannot cancel Nav2 goal — pose unknown. "
                "Goal re-sending suspended until semantic clears."
            )
            return
        stop_goal = stamp_now(self)
        stop_goal.pose.position.x = agv.pose[0]
        stop_goal.pose.position.y = agv.pose[1]
        stop_goal.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(stop_goal)
        self._goal_last_sent[ns] = time.time()

    # ====================================================================
    # Periodic fleet status log (CHANGE 8: new for FleetManagerAI)
    # ====================================================================

    def _log_fleet_status(self) -> None:
        """Log fleet state including semantic information every 5 s."""
        for ns in self.ns_list:
            agv = self.agv[ns]
            safety_tag = "| STOPPED_FOR_SAFETY" if agv.semantic_stop_active else "| nominal"
            self.get_logger().info(
                f"[FleetAI] {ns}: {agv.state} "
                f"| semantic={agv.semantic_obstacle} "
                f"{safety_tag}"
            )

    # ====================================================================
    # Main control loop — identical to fleet_manager
    # ====================================================================

    def step(self) -> None:
        for ns in self.ns_list:
            self.run_agv(ns)
        self.assign_tasks()

    # ====================================================================
    # Task assignment — identical to fleet_manager
    # ====================================================================

    def assign_tasks(self) -> None:
        if not self.task_queue:
            return
        idle = [
            a for a in self.agv.values()
            if a.state == "IDLE" and a.pose is not None and a.battery >= self.low_thr
        ]
        if not idle:
            return

        for agv in idle:
            best = None
            best_d = float("inf")
            for idx, t in enumerate(self.task_queue):
                pz = self.zones.get(t["pickup"])
                if not pz:
                    continue
                d = dist(agv.pose, (pz["cx"], pz["cy"]))
                if d < best_d:
                    best_d = d
                    best = (idx, t)
            if best is None:
                continue
            idx, task = best
            agv.task = task
            agv.state = "TO_PICKUP"
            agv.state_since = time.time()
            self.nav_to_zone(agv.ns, task["pickup"])
            self.get_logger().info(
                f"[{agv.ns}] Assigned task {task.get('id', '?')} → pickup {task['pickup']}"
            )
            self.task_queue.pop(idx)
            break  # one task per loop iteration

    # ====================================================================
    # Per-AGV FSM — identical to fleet_manager
    # ====================================================================

    def run_agv(self, ns: str) -> None:
        agv = self.agv[ns]
        now = time.time()

        # Battery low → head to charger (preempts all navigation states)
        if agv.state in ("IDLE", "TO_PICKUP", "TO_DROPOFF") and agv.battery < self.low_thr and not agv.on_charger:
            cz_name = self.nearest_charger(agv.pose)
            if cz_name:
                agv.state = "TO_CHARGER"
                agv.state_since = now
                self.nav_to_zone(ns, cz_name)
                self.get_logger().info(
                    f"[{ns}] Battery low ({agv.battery:.2f}). Heading to charger {cz_name}."
                )
            return

        # Battery recovered while at charger
        if agv.state in ("CHARGING", "TO_CHARGER") and agv.battery >= self.resume_thr and agv.on_charger:
            agv.state = "IDLE"
            agv.state_since = now
            self.get_logger().info(
                f"[{ns}] Battery recovered ({agv.battery:.2f}). Back to IDLE."
            )
            return

        if agv.state == "IDLE":
            self.set_carrying(ns, False)
            return

        if agv.state == "TO_CHARGER":
            if self.at_zone(ns, agv.pose, self.target_zone_of(ns)):
                agv.state = "CHARGING"
                agv.state_since = now
                self.get_logger().info(f"[{ns}] Arrived at charger. Waiting to charge...")
            else:
                self._resend_goal_if_due(ns, self.target_zone_of(ns), now)
            return

        if agv.state == "CHARGING":
            return

        if agv.state == "TO_PICKUP":
            if agv.pose is None:
                return
            tz = self.target_zone_of(ns)
            if self.at_zone(ns, agv.pose, tz):
                agv.state = "LOADING"
                agv.state_since = now
                self.get_logger().info(f"[{ns}] At pickup {agv.task['pickup']}. Loading...")
            else:
                self._resend_goal_if_due(ns, tz, now)
            return

        if agv.state == "LOADING":
            load_time = float(agv.task.get("load_time", 3.0))
            if now - agv.state_since >= load_time:
                self.set_carrying(ns, True)
                agv.state = "TO_DROPOFF"
                agv.state_since = now
                self.nav_to_zone(ns, agv.task["dropoff"])
                self.get_logger().info(
                    f"[{ns}] Loaded. Heading to dropoff {agv.task['dropoff']}."
                )
            return

        if agv.state == "TO_DROPOFF":
            if agv.pose is None:
                return
            tz = self.target_zone_of(ns)
            if self.at_zone(ns, agv.pose, tz):
                agv.state = "UNLOADING"
                agv.state_since = now
                self.get_logger().info(f"[{ns}] At dropoff {agv.task['dropoff']}. Unloading...")
            else:
                self._resend_goal_if_due(ns, tz, now)
            return

        if agv.state == "UNLOADING":
            unload_time = float(agv.task.get("unload_time", 3.0))
            if now - agv.state_since >= unload_time:
                self.set_carrying(ns, False)
                self.get_logger().info(
                    f"[{ns}] Task {agv.task.get('id', '?')} complete."
                )
                agv.task = None
                agv.state = "IDLE"
                agv.state_since = now
            return

    # ====================================================================
    # Helpers — nav_to_zone modified (CHANGE 7); rest identical to fleet_manager
    # ====================================================================

    def nav_to_zone(self, ns: str, zone_name: str) -> None:
        z = self.zones[zone_name]
        msg = stamp_now(self)
        msg.pose.position.x = float(z["cx"])
        msg.pose.position.y = float(z["cy"])
        msg.pose.orientation.w = 1.0
        # CHANGE 7: store goal before publishing so _handle_semantic_change
        # can re-send it when the path clears after a person-detection stop.
        self.agv[ns].last_nav_goal = msg
        self.goal_pub[ns].publish(msg)
        setattr(self, f"_target_zone_{ns}", zone_name)
        self._goal_last_sent[ns] = time.time()

    def _resend_goal_if_due(self, ns: str, zone_name: Optional[str], now: float) -> None:
        """Re-publish the navigation goal if the resend interval has elapsed.

        Suppressed while semantic_stop_active is True — we must not re-send
        the real goal while the robot is stopped for a person detection.
        """
        if self.agv[ns].semantic_stop_active:
            return   # semantic stop in effect — do not re-send
        if zone_name and now - self._goal_last_sent.get(ns, 0.0) >= self._goal_resend_interval:
            self.get_logger().info(
                f"[{ns}] Re-sending goal to {zone_name} "
                "(nav2_goal_bridge may have missed initial publish)."
            )
            self.nav_to_zone(ns, zone_name)

    def target_zone_of(self, ns: str) -> Optional[str]:
        return getattr(self, f"_target_zone_{ns}", None)

    def at_zone(self, ns: str, pose_xy: Optional[tuple[float, float]], zone_name: Optional[str]) -> bool:
        if not pose_xy or not zone_name:
            return False
        z = self.zones.get(zone_name)
        if not z:
            return False
        x, y = pose_xy
        hx = float(z["sx"]) / 2.0 + 0.05
        hy = float(z["sy"]) / 2.0 + 0.05
        return (abs(x - float(z["cx"])) <= hx) and (abs(y - float(z["cy"])) <= hy)

    def nearest_charger(self, pose_xy: Optional[tuple[float, float]]) -> Optional[str]:
        if not pose_xy:
            return None
        best: Optional[str] = None
        best_d = float("inf")
        for name in self.charger_zone_names:
            z = self.zones.get(name)
            if not z:
                continue
            d = dist(pose_xy, (z["cx"], z["cy"]))
            if d < best_d:
                best_d = d
                best = name
        return best

    def set_carrying(self, ns: str, val: bool) -> None:
        self.carry_pub[ns].publish(Bool(data=val))
        self.agv[ns].carrying = val


# ---------------------------------------------------------------------------
# Entry point (CHANGE 9: creates FleetManagerAI)
# ---------------------------------------------------------------------------

def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = FleetManagerAI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
