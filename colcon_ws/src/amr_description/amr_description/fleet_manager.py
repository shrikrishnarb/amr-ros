#!/usr/bin/env python3
import os
import math
import yaml
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import BatteryState

def dist(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def stamp_now(node):
    msg = PoseStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "world"
    return msg

class AGVState:
    def __init__(self, ns):
        self.ns = ns
        self.pose = None  # (x,y)
        self.on_charger = False
        self.battery = 1.0  # 0..1
        self.state = 'IDLE'
        self.task = None  # {'id','pickup','dropoff','load_time','unload_time'}
        self.state_since = time.time()
        self.carrying = False

class FleetManager(Node):
    """
    Minimal fleet / task manager:
      - Assigns pickup->dropoff tasks from a queue
      - Drives AGVs via /<ns>/goal_pose (used by your follower)
      - Considers battery thresholds and uses charger zones
      - Simulates loading/unloading by timed delays
      - Publishes /<ns>/carrying_load Bool for visualization/logic
    """
    def __init__(self):
        super().__init__('fleet_manager')

        # Params
        self.declare_parameter('robot_namespaces', ['agv1'])
        self.declare_parameter('tasks_file', '')
        self.declare_parameter('zones_yaml', '')
        self.declare_parameter('tasks_yaml', '')
        self.declare_parameter('battery_low_threshold', 0.20)
        self.declare_parameter('battery_resume_threshold', 0.60)
        self.declare_parameter('goal_reach_dist', 0.25)
        self.declare_parameter('charge_check_period', 1.0)
        self.declare_parameter('load_unload_poll_period', 0.2)
        self.declare_parameter('battery_topic_type', 'auto')  # 'auto'|'battery_state'|'float32'
        self.declare_parameter('charger_zone_names', ['charger_1','charger_2'])

        self.ns_list = list(self.get_parameter('robot_namespaces').value)
        self.low_thr = float(self.get_parameter('battery_low_threshold').value)
        self.resume_thr = float(self.get_parameter('battery_resume_threshold').value)
        self.reach_dist = float(self.get_parameter('goal_reach_dist').value)
        self.batt_topic_type = str(self.get_parameter('battery_topic_type').value)
        self.charger_zone_names = list(self.get_parameter('charger_zone_names').value)

        # Load zones/tasks from YAML if provided, else from params
        tasks_file = str(self.get_parameter('tasks_file').value).strip()

        if tasks_file and os.path.exists(tasks_file):
            with open(tasks_file, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.zones = data.get('zones', {}) or {}
            self.task_queue = data.get('tasks', []) or []
        else:
            # Fallback: allow passing YAML as a string param (still not required by your launch)
            zones_yaml = str(self.get_parameter('zones_yaml').value or '').strip()
            tasks_yaml = str(self.get_parameter('tasks_yaml').value or '').strip()

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

        # Internal state per AGV
        self.agv = {ns: AGVState(ns) for ns in self.ns_list}

        # Publishers/subscribers per AGV
        self.goal_pub = {}
        self.carry_pub = {}
        for ns in self.ns_list:
            self.goal_pub[ns] = self.create_publisher(PoseStamped, f'/{ns}/goal_pose', 10)
            self.carry_pub[ns] = self.create_publisher(Bool, f'/{ns}/carrying_load', 10)
            self.create_subscription(Odometry, f'/{ns}/ground_truth', lambda msg, ns=ns: self.cb_odom(ns, msg), 10)
            self.create_subscription(Bool, f'/{ns}/on_charger', lambda msg, ns=ns: self.cb_on_charger(ns, msg), 10)
            # Battery
            if self.batt_topic_type in ('auto','battery_state'):
                self.create_subscription(BatteryState, f'/{ns}/battery_state', lambda msg, ns=ns: self.cb_batt_state(ns, msg), 10)
            if self.batt_topic_type in ('auto','float32'):
                self.create_subscription(Float32, f'/{ns}/battery_percentage', lambda msg, ns=ns: self.cb_batt_float(ns, msg), 10)

        # Timers
        self.control_timer = self.create_timer(0.25, self.step)  # main loop

        self.get_logger().info(f"FleetManager started. AGVs: {self.ns_list}, tasks in queue: {len(self.task_queue)}")

    # ======== Callbacks ========

    def cb_odom(self, ns, msg: Odometry):
        self.agv[ns].pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def cb_on_charger(self, ns, msg: Bool):
        self.agv[ns].on_charger = bool(msg.data)

    def cb_batt_state(self, ns, msg: BatteryState):
        # percentage may be 0..100 or 0..1 depending on your battery_sim; normalize
        pct = msg.percentage
        if pct > 1.5:  # looks like 0..100
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    def cb_batt_float(self, ns, msg: Float32):
        pct = msg.data
        if pct > 1.5:
            pct = pct / 100.0
        pct = max(0.0, min(1.0, pct))
        self.agv[ns].battery = pct

    # ======== Main control loop ========

    def step(self):
        # Update each AGV state machine
        for ns in self.ns_list:
            self.run_agv(ns)

        # Assign tasks to idle AGVs
        self.assign_tasks()

    # ======== Assignment ========

    def assign_tasks(self):
        if not self.task_queue:
            return
        # candidates: idle AGVs with battery >= low_thr
        idle = [a for a in self.agv.values() if a.state == 'IDLE' and a.pose is not None and a.battery >= self.low_thr]
        if not idle:
            return

        # Greedy: pick best (AGV, task) by distance to pickup
        for agv in idle:
            # pick best task for this AGV
            best = None
            best_d = float('inf')
            for idx, t in enumerate(self.task_queue):
                pz = self.zones.get(t['pickup'])
                if not pz:
                    continue
                d = dist(agv.pose, (pz['cx'], pz['cy']))
                if d < best_d:
                    best_d = d
                    best = (idx, t)
            if best is None:
                continue
            # Assign
            idx, task = best
            agv.task = task
            agv.state = 'TO_PICKUP'
            agv.state_since = time.time()
            self.nav_to_zone(agv.ns, task['pickup'])
            self.get_logger().info(f"[{agv.ns}] Assigned task {task.get('id','?')} → pickup {task['pickup']}")
            # Remove from queue
            self.task_queue.pop(idx)
            # Assign one task at a time per loop to keep it simple
            break

    # ======== Per-AGV FSM ========

    def run_agv(self, ns):
        agv = self.agv[ns]
        now = time.time()
        # If battery low and not charging nor going to charger, send to the nearest charger
        if agv.state in ('IDLE','TO_PICKUP','TO_DROPOFF') and agv.battery < self.low_thr and not agv.on_charger:
            cz_name = self.nearest_charger(agv.pose)
            if cz_name:
                agv.state = 'TO_CHARGER'
                agv.state_since = now
                self.nav_to_zone(ns, cz_name)
                self.get_logger().info(f"[{ns}] Battery low ({agv.battery:.2f}). Heading to charger {cz_name}.")
            return

        # If charging and recovered
        if agv.state in ('CHARGING','TO_CHARGER') and (agv.battery >= self.resume_thr) and agv.on_charger:
            agv.state = 'IDLE'
            agv.state_since = now
            self.get_logger().info(f"[{ns}] Battery recovered ({agv.battery:.2f}). Back to IDLE.")
            return

        # State logic
        if agv.state == 'IDLE':
            # Publish carrying false just to be explicit
            self.set_carrying(ns, False)
            return

        if agv.state == 'TO_CHARGER':
            if self.at_zone(ns, agv.pose, self.target_zone_of(ns)):
                agv.state = 'CHARGING'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] Arrived at charger. Waiting to charge...")
            return

        if agv.state == 'CHARGING':
            # Passive: wait until resume_thr while on_charger
            return

        if agv.state == 'TO_PICKUP':
            if agv.pose is None:
                return
            tz = self.target_zone_of(ns)
            if self.at_zone(ns, agv.pose, tz):
                agv.state = 'LOADING'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] At pickup {agv.task['pickup']}. Loading...")
            return

        if agv.state == 'LOADING':
            load_time = float(agv.task.get('load_time', 3.0))
            if now - agv.state_since >= load_time:
                self.set_carrying(ns, True)
                agv.state = 'TO_DROPOFF'
                agv.state_since = now
                self.nav_to_zone(ns, agv.task['dropoff'])
                self.get_logger().info(f"[{ns}] Loaded. Heading to dropoff {agv.task['dropoff']}.")
            return

        if agv.state == 'TO_DROPOFF':
            if agv.pose is None:
                return
            tz = self.target_zone_of(ns)
            if self.at_zone(ns, agv.pose, tz):
                agv.state = 'UNLOADING'
                agv.state_since = now
                self.get_logger().info(f"[{ns}] At dropoff {agv.task['dropoff']}. Unloading...")
            return

        if agv.state == 'UNLOADING':
            unload_time = float(agv.task.get('unload_time', 3.0))
            if now - agv.state_since >= unload_time:
                self.set_carrying(ns, False)
                self.get_logger().info(f"[{ns}] Task {agv.task.get('id','?')} complete.")
                agv.task = None
                agv.state = 'IDLE'
                agv.state_since = now
            return

    # ======== Helpers ========

    def nav_to_zone(self, ns, zone_name):
        z = self.zones[zone_name]
        msg = stamp_now(self)
        msg.pose.position.x = float(z['cx'])
        msg.pose.position.y = float(z['cy'])
        msg.pose.orientation.w = 1.0
        self.goal_pub[ns].publish(msg)
        # Track current navigation target per AGV for zone checks
        setattr(self, f'_target_zone_{ns}', zone_name)

    def target_zone_of(self, ns):
        return getattr(self, f'_target_zone_{ns}', None)

    def at_zone(self, ns, pose_xy, zone_name):
        if not pose_xy or not zone_name:
            return False
        z = self.zones.get(zone_name)
        if not z:
            return False
        x, y = pose_xy
        # axis-aligned rectangle with small margin
        hx = float(z['sx']) / 2.0 + 0.05
        hy = float(z['sy']) / 2.0 + 0.05
        return (abs(x - float(z['cx'])) <= hx) and (abs(y - float(z['cy'])) <= hy)

    def nearest_charger(self, pose_xy):
        if not pose_xy:
            return None
        best = None
        best_d = float('inf')
        for name in self.charger_zone_names:
            z = self.zones.get(name)
            if not z:
                continue
            d = dist(pose_xy, (z['cx'], z['cy']))
            if d < best_d:
                best_d = d
                best = name
        return best

    def set_carrying(self, ns, val: bool):
        self.carry_pub[ns].publish(Bool(data=val))
        self.agv[ns].carrying = val


def main(args=None):
    rclpy.init(args=args)
    node = FleetManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()