# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment

All development is done inside a Docker container (ROS 2 Humble). GUI apps (Gazebo, RViz) require X11 forwarding.

```bash
# Build and start container
xhost +local:root
cd docker && docker compose up --build -d
docker exec -it amr-ros-dev bash
```

> Always `docker restart amr-ros-dev` (not just pkill) between simulation runs to avoid stale DDS shared-memory segments and duplicate node processes from previous launches.

## Build & Source

Inside the container:
```bash
cd /workspace/colcon_ws
colcon build --symlink-install
source /opt/ros/humble/setup.bash
source /workspace/colcon_ws/install/setup.bash
```

After editing Python nodes (with `--symlink-install`), a rebuild is usually not needed unless you changed `setup.py` (entry points, data_files).

## Running the Simulation

**Fleet management — standalone (Gazebo + Nav2 + fleet, all in one):**
```bash
ros2 launch amr_description amr_fleet_management.launch.py \
    num_agvs:=2 launch_gazebo:=true spawn_poses:="0.0,0.0;2.0,2.0"
```

**Fleet management — fleet-only (Gazebo already running):**
```bash
# Terminal 1
ros2 launch amr_description display.launch.py namespace:=agv1
# Terminal 2
ros2 launch amr_description spawn_agv.launch.py namespace:=agv2 x:=2.0 y:=2.0
# Terminal 3
ros2 launch amr_description amr_fleet_management.launch.py num_agvs:=2 spawn_poses:="0.0,0.0;2.0,2.0"
```

**Single AGV + display:**
```bash
ros2 launch amr_description display.launch.py namespace:=agv1
```

**Send AGV to a pose (ground-truth follower, no Nav2):**
```bash
ros2 launch amr_description ground_truth_waypoint_follower.launch.py namespace:=agv1 x:=-5.0 y:=-6.0
```

**SLAM mapping:**
```bash
ros2 launch amr_description mapping.launch.py
```

## Tests

```bash
cd /workspace/colcon_ws
colcon test --packages-select amr_description
colcon test-result --verbose
```

## Architecture

### Package

- **`amr_description`** — single package containing all robot nodes, launch files, URDF, worlds, Nav2 config, and behaviour trees.

### Key Nodes (`colcon_ws/src/amr_description/amr_description/`)

| Node | Executable | Role |
|------|-----------|------|
| `fleet_manager.py` | `fleet_manager` | Loads zones/tasks from `tasks.yaml`, assigns tasks to AGVs via a greedy FSM, monitors battery, sends AGVs to chargers. Re-sends active goal every 10 s to survive Nav2 startup delay. |
| `nav2_goal_bridge.py` | `nav2_goal_bridge` | Bridges `/<ns>/goal_pose` (from fleet_manager) to Nav2 `navigate_to_pose` action. Buffers goals until Nav2 active; ignores re-sends to same destination. |
| `tf_relay.py` | `tf_relay` | Relays `/tf` → `/<ns>/tf` and `/tf_static` → `/<ns>/tf_static`. Required because Nav2 with `use_namespace:=True` reads from namespaced TF topics. |
| `odom_sim_filter.py` | `odom_sim_filter` | Converts Gazebo ground-truth odometry into `odom→base_footprint` TF for each robot. |
| `obstacle_detection_node.py` | `obstacle_detection_node` | Reads LiDAR scan → publishes `/<ns>/obstacle_detected` (Bool). |
| `battery_sim.py` | `battery_sim` | Simulates battery drain/charge; publishes `/<ns>/battery_state`. |
| `charger_dock_monitor.py` | `charger_dock_monitor` | Monitors proximity to charging stations → publishes `/<ns>/on_charger` (Bool). |
| `ground_truth_waypoint_follower.py` | `ground_truth_waypoint_follower.py` | Simple straight-line follower using ground-truth odometry. Useful for testing without Nav2. |

### Fleet Manager FSM

```
IDLE → TO_PICKUP → LOADING → TO_DROPOFF → UNLOADING → IDLE
                                    ↑
         (battery low) → TO_CHARGER → CHARGING → IDLE
```

Task/zone configuration is driven entirely by `yaml/tasks.yaml`.

### Nav2 Behaviour Tree

`behavior_trees/navigate_w_recovery.xml` — uses `PipelineSequence` + `RateController hz="0.5"` so the global path is recomputed every 2 seconds while the robot is moving. This is what enables smooth rerouting around dynamic obstacles (other robots).

### Topic Conventions (per AGV namespace `/<ns>/`)

- `goal_pose` (PoseStamped, frame `map`) — navigation target, consumed by nav2_goal_bridge
- `ground_truth` (Odometry) — Gazebo perfect pose, used by fleet_manager and odom_sim_filter
- `cmd_vel` (Twist) — motion commands, published by Nav2 controller_server
- `obstacle_detected` (Bool) — from obstacle_detection_node
- `on_charger` (Bool) — from charger_dock_monitor
- `carrying_load` (Bool) — published by fleet_manager during LOADING/TO_DROPOFF
- `battery_state` (BatteryState) or `battery_percentage` (Float32) — from battery_sim

### Configuration

- **`yaml/tasks.yaml`** — named zones (`cx`, `cy`, `sx`, `sy` for centre/size) and task queue.
- **`yaml/nav2_params_amr.yaml`** — full Nav2 stack parameters (AMCL, NavFn, DWB, costmaps, BT navigator). `{NS}` and `{BT_FILE}` placeholders are substituted per-robot at launch time.

## Docker Setup
- Container: amr-ros-dev
- ROS logs: /root/.ros/log/
- Run commands inside container: `docker exec amr-ros-dev <cmd>`
- Get latest logs: `docker exec amr-ros-dev ls /root/.ros/log/ | tail -1`
