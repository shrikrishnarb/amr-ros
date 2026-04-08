# AMR Project — Evolution Changelog

This file traces the project from its first version (a simple differential-drive
robot controlled by a hand-written waypoint follower) to the current version (a
proper AMR fleet with real autonomous navigation, localization, and multi-robot
dynamic obstacle avoidance).

---

## Version 1 — Basic Robot in Gazebo

**What was built:** A differential-drive robot that could be spawned in Gazebo
and driven with a keyboard.

- Wrote the first URDF for the robot chassis, two wheels, a caster, and a
  LiDAR sensor.
- Set up a Gazebo world and a RViz launch file so the robot could be visualised.
- Added the `diff_drive` Gazebo plugin so the robot responds to `/cmd_vel`.
- Added teleop keyboard integration so a person could drive it manually.

```xml
<!-- amr.urdf.xacro — diff_drive plugin (added early) -->
<plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.35</wheel_separation>
  <wheel_diameter>0.10</wheel_diameter>
  <publish_odom>true</publish_odom>
  <publish_odom_tf>true</publish_odom_tf>
  <odometry_topic>odom</odometry_topic>
  <ros>
    <namespace>$(arg namespace)</namespace>
  </ros>
</plugin>
```

At this stage the robot could move around but had no intelligence at all — a
human had to steer it.

---

## Version 2 — SLAM and a Real Map

**What was built:** The ability to drive the robot around and record a map of
the environment using SLAM Toolbox.

- Added `slam.launch.py` that starts SLAM Toolbox alongside Gazebo.
- Driving the robot with the keyboard while SLAM ran produced a 2D occupancy
  grid of the warehouse world.
- Saved the finished map as `maps/my_map.pgm` + `maps/my_map.yaml` — this same
  map is used by all later versions.

At this stage we had a map, but the robot still did not know where it was on
that map once you restarted it.

---

## Version 3 — Localisation with AMCL

**What was built:** The robot could now figure out where it was on the saved map
automatically using AMCL (Adaptive Monte Carlo Localisation).

- Added `localization.launch.py` that loads the pre-built map and starts AMCL.
- AMCL listens to the LiDAR scan and compares it with the map to estimate the
  robot's pose. The initial pose is given once, and from that point the robot
  tracks itself.

This was the first step toward the robot having self-awareness of its position.
But motion was still manual (keyboard).

---

## Version 4 — Multi-Robot Namespacing

**What was built:** The ability to spawn multiple robots in the same Gazebo
world, each with its own independent ROS topics, TF tree, and sensors.

- Converted the URDF to Xacro so the robot namespace (e.g. `agv1`, `agv2`) is
  passed as a parameter at launch time.
- Every topic and TF frame got the namespace prefix:
  `/agv1/scan`, `/agv1/odom`, `agv1/base_footprint`, etc.
- Added `odom_sim_filter.py` which converts Gazebo's raw ground-truth odometry
  into a proper `odom → base_footprint` TF transform for each robot.

```python
# odom_sim_filter.py — publishes TF from ground truth
t.header.frame_id = f'{self.ns}/odom'
t.child_frame_id  = f'{self.ns}/base_footprint'
self.tf_broadcaster.sendTransform(t)
```

At this point two robots could live in the same world without interfering with
each other's data.

---

## Version 5 — Ground-Truth Waypoint Follower (first autonomous motion)

**What was built:** The robot's first autonomous motion. Given a target `(x, y)`
position, the robot drives itself there using its ground-truth pose.

`ground_truth_waypoint_follower.py` works like this:

1. Subscribe to `/agv1/ground_truth` (Gazebo's perfect pose, no sensor noise).
2. Compute the angle to the goal and rotate in place to face it.
3. Drive forward, scaling speed down as it gets close.
4. Publish `goal_reached` when it arrives.

```python
# Rotate toward goal, then drive
dx = goal_x - current_x
dy = goal_y - current_y
target_yaw = math.atan2(dy, dx)
dyaw = normalize_angle(target_yaw - current_yaw)

if abs(dyaw) > 0.3:          # still needs to turn
    cmd.angular.z = angular_gain * dyaw
else:                         # facing goal — drive
    cmd.linear.x  = clamp(linear_gain * dist, min_linear, max_linear)
    cmd.angular.z = angular_gain * dyaw
```

**The limitation:** this follower only uses ground truth, which does not exist
on a real robot. It also draws a straight line to the goal — if a wall is in
the way it has no idea.

---

## Version 6 — Obstacle Detection and Progressive Slow-Down

**What was built:** The robot could detect obstacles in front of it with the
LiDAR and slow down smoothly rather than crashing.

- Added `obstacle_detection_node.py` which reads `/scan` and publishes
  `/obstacle_detected` (Bool) when something is within a threshold distance.
- Upgraded the waypoint follower to read the raw LiDAR scan and compute a
  "slowdown scale" based on the closest forward reading:

```python
def compute_slowdown_scale(self) -> float:
    fmin = self.sector_min_range(scan, -sector, +sector)
    if fmin <= self.stop_dist:          return 0.0   # fully stopped
    if fmin >= self.slow_down_start_dist: return 1.0  # full speed
    return (fmin - self.stop_dist) / (self.slow_down_start_dist - self.stop_dist)
```

The robot would slow down, stop in front of obstacles, and resume when they
cleared — but it could not go around them.

---

## Version 7 — Battery Simulation and Charging

**What was built:** Each robot now has a simulated battery that drains as it
moves and recharges when it docks.

- `battery_sim.py` — publishes `/agv1/battery_state` (0–100 %). Drains at a
  configured rate while moving, charges when `/on_charger` is True.
- `charger_dock_monitor.py` — compares the robot's ground-truth position to
  charger zone centres. Publishes `/on_charger` (Bool) when the robot is close
  enough.

At this point the robot's infrastructure was mostly ready. The next step was
to give it real jobs to do.

---

## Version 8 — Fleet Manager (multi-robot task assignment)

**What was built:** A central brain that assigns pickup→dropoff tasks to
multiple robots, monitors battery, and sends robots to chargers when needed.

`fleet_manager.py` runs a simple FSM per robot:

```
IDLE → TO_PICKUP → LOADING → TO_DROPOFF → UNLOADING → IDLE
                                     ↑
               (battery low) → TO_CHARGER → CHARGING
```

Tasks and zones are defined in `yaml/tasks.yaml`. The manager picks the
closest idle robot to each task pickup zone (greedy assignment) and sends it a
`goal_pose` message that the waypoint follower (at the time) would execute.

At this stage both robots could be given independent jobs and would attempt to
complete them — but still using the dumb straight-line follower with no real
path planning.

---

## Version 9 — Nav2 Integration (real autonomous navigation)

**What was built:** The ground-truth follower was replaced with the full Nav2
navigation stack — the same system used on real robots.

This was the biggest change. Instead of "drive straight to (x, y) and hope",
the robot now:
1. Localises itself on the pre-built map with AMCL.
2. Plans a global path around walls using NavFn (A* / Dijkstra).
3. Executes the path with DWB (Dynamic Window Approach) local planner, which
   generates smooth velocity commands that respect the robot's kinematic limits.
4. Runs recovery behaviours (spin in place, back up) if it gets stuck.

### New node: `nav2_goal_bridge.py`

This bridges the fleet manager's simple `goal_pose` topic to Nav2's proper
action server:

```python
# nav2_goal_bridge.py — simplified core
self.nav = BasicNavigator(namespace=ns)
self.nav.waitUntilNav2Active()          # waits for AMCL + all Nav2 servers

def _on_goal(self, msg: PoseStamped):
    self.nav.goToPose(msg)              # sends goal to bt_navigator action server
    self._goal_active = True

def _poll_nav2(self):
    if self.nav.isTaskComplete():
        result = self.nav.getResult()   # SUCCEEDED / FAILED
        self._goal_active = False
```

### New node: `tf_relay.py`

Nav2 with namespacing reads TF from `/{ns}/tf`, but the existing
`odom_sim_filter` and `robot_state_publisher` published to the global `/tf`.
Rather than rewriting those nodes, a relay was added:

```python
# tf_relay.py — bridges /tf → /agv1/tf for AMCL
self.create_subscription(TFMessage, '/tf',      self._on_tf,        DYNAMIC_QOS)
self.create_publisher  (TFMessage, f'/{ns}/tf', DYNAMIC_QOS)
# same for /tf_static → /{ns}/tf_static  (TRANSIENT_LOCAL so late joiners get it)
```

### Navigation params (`nav2_params_amr.yaml`)

A full Nav2 parameter file was written covering AMCL, planner_server
(NavFn), controller_server (DWB), local costmap, global costmap, behaviour
server, and bt_navigator — all with `{NS}` placeholders replaced at launch
time for each robot.

### Bug fixed — goal race on startup

Fleet manager sends the first goal at ~t=2s. Nav2 takes 30+ seconds to
activate. The goal message was lost because nav2_goal_bridge had not yet
subscribed.

Fix: goals arriving before Nav2 is active are buffered. Fleet manager also
re-sends the active goal every 10 seconds so the bridge always gets it:

```python
# fleet_manager.py
self._goal_resend_interval = 10.0  # re-publish if robot hasn't arrived yet

def _resend_goal_if_due(self, ns, zone_name, now):
    if now - self._goal_last_sent[ns] >= self._goal_resend_interval:
        self.nav_to_zone(ns, zone_name)
```

---

## Version 10 — LiDAR Upgraded to 360°

**What was built:** The LiDAR scan arc was extended from 180° (front only) to
full 360°.

```xml
<!-- amr.urdf.xacro — before -->
<min_angle>-1.5708</min_angle>   <!-- −90° -->
<max_angle>1.5708</max_angle>    <!--  +90° -->

<!-- after -->
<min_angle>-3.14159</min_angle>  <!-- −180° = full circle -->
<max_angle>3.14159</max_angle>
```

With a full 360° scan, AMCL converges much faster (more landmark readings
available), which eliminated the storm of ~70 initial-pose retries that
previously happened at startup.

---

## Version 11 — Two-Robot Dynamic Obstacle Avoidance (current version)

**What was built:** Smooth navigation for two robots moving simultaneously,
including proper rerouting around each other as dynamic obstacles.

### The problem

With two robots running, each robot's LiDAR would see the other one. The
global path was computed only once per goal. When the other robot moved into
the planned path, the local planner (DWB) would try to squeeze around it,
fail, and stop. The robot could not reroute because the global path never
changed.

### Fix 1 — Continuous global replanning (Behaviour Tree)

The behaviour tree in `navigate_w_recovery.xml` was rewritten. The key change
is wrapping `ComputePathToPose` in a `RateController` so the global path is
replanned every 2 seconds while the robot is moving:

```xml
<!-- Before: path computed once, then followed forever -->
<Sequence name="NavigateSequence">
  <ComputePathToPose goal="{goal}" path="{path}"/>
  <FollowPath path="{path}"/>
</Sequence>

<!-- After: path recomputed every 2 s via RateController -->
<PipelineSequence name="NavigateWithReplanning">
  <RateController hz="0.5" name="PlannerRate">
    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
  </RateController>
  <FollowPath path="{path}" controller_id="FollowPath"/>
</PipelineSequence>
```

`PipelineSequence` (unlike plain `Sequence`) lets `FollowPath` keep running
while `ComputePathToPose` is computing — so the robot never pauses to replan.

### Fix 2 — Obstacle decay in the global costmap

Without decay, once the other robot was seen in the global costmap it left a
permanent obstacle mark even after it moved away, causing the planner to keep
routing around a ghost.

```yaml
# nav2_params_amr.yaml — global costmap obstacle layer
obstacle_layer:
  decay_time: 3.0   # marks clear 3 s after the obstacle moves away
```

### Fix 3 — Nav2 parameter tuning for dual-robot CPU budget

Running two full Nav2 stacks simultaneously doubled CPU load. Several
parameters were relaxed so both stacks fit within the available budget:

| Parameter | Before | After | Effect |
|---|---|---|---|
| `bt_loop_duration` | 20 ms | 50 ms | BT ticks at 20 Hz instead of 50 Hz |
| `default_server_timeout` | 20 ms | 1000 ms | Planner has time to ACK under load |
| `controller_frequency` | 5 Hz | 3 Hz | DWB fits in 333 ms per cycle |
| `vx/vy/vtheta_samples` | 10/5/20 | 5/1/10 | 50 trajectories instead of 1000 |
| `global_costmap.update_frequency` | 1 Hz | 2 Hz | Obstacles visible within 0.5 s |
| `progress_checker.movement_time_allowance` | 30 s | 10 s | Recovery triggers faster |

### Fix 4 — Goal re-send cooldown in nav2_goal_bridge

The fleet manager re-sends the active goal every 10 s. If this re-send
arrived while the BT was in a recovery action (spin/backup), it would
preempt and restart the navigation, interrupting recovery.

Fix: after a navigation failure, same-destination goals are ignored for 6
seconds so recovery can finish cleanly:

```python
# nav2_goal_bridge.py
if self._current_goal is not None:
    dx = abs(msg.pose.position.x - self._current_goal.pose.position.x)
    dy = abs(msg.pose.position.y - self._current_goal.pose.position.y)
    if dx < 0.05 and dy < 0.05:
        if self._goal_active:
            return   # navigation in progress — ignore resend
        if time.monotonic() - self._failed_at < self._failure_cooldown:
            return   # recovery cooldown — ignore resend
```

### Result

```
[fleet_manager]: [agv2] At dropoff dropoff_zone_B. Unloading...
[fleet_manager]: [agv2] Task T1 complete.
[fleet_manager]: [agv1] At dropoff dropoff_zone_A. Unloading...
[fleet_manager]: [agv1] Task T2 complete.
```

Both robots completed full pickup→dropoff cycles simultaneously, crossing
each other's paths in the centre of the map without collision.

---

## Summary — What Changed From Start to Now

| Capability | v1 | Current |
|---|---|---|
| Robot control | Manual keyboard | Fully autonomous Nav2 |
| Localisation | None (teleop) | AMCL on pre-built map |
| Path planning | Straight line to goal | NavFn global + DWB local |
| Obstacle handling | Stop and wait | Replan route around obstacle |
| Number of robots | 1 | 2+ (namespaced fleet) |
| Task assignment | Manual | Fleet manager FSM |
| Battery / charging | None | Simulated with auto-docking |
| Dynamic obstacles | Robot stops | Replans every 2 s, routes around |
