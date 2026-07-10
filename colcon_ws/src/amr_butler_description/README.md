# amr_butler_description

"Butler" service robot for the **Gazebo Harmonic (gz-sim 8)** stack: a
differential-drive base (cylindrical chassis, Ø 0.70 m) with a rear pillar,
two serving trays, and a sensor head at ~1.2 m carrying a forward-facing RGB-D
camera. A 360° 2D lidar sits at 0.25 m for later Nav2 use.

Unlike `amr_description` / `amr_vision` (Gazebo Classic, frozen), this package
uses **`ros_gz` only** — never add `gazebo_ros` dependencies here (see
`CLAUDE.md`, "Gazebo Classic vs Harmonic").

## Contents

| Path | Purpose |
|---|---|
| `urdf/amr_butler.urdf.xacro` | robot model, gz-sim plugins (DiffDrive, JointStatePublisher) + gpu_lidar & rgbd_camera sensors, all topics model-scoped `/model/<ns>/...` |
| `config/bridge.yaml` | ros_gz_bridge template; `{NS}` substituted at launch |
| `worlds/butler_test.sdf` | minimal Harmonic test world (Physics, UserCommands, SceneBroadcaster, Sensors/ogre2) |
| `launch/butler_sim.launch.py` | sim + spawn + robot_state_publisher + bridge + RViz |
| `rviz/butler.rviz` | RobotModel, TF, LaserScan, camera Image (assumes namespace `butler1`) |

## Launch (inside the amr-butler-dev container)

```bash
# host, once per session
xhost +local:root
cd docker && docker compose up -d amr-butler-dev && docker exec -it amr-butler-dev bash

# container
cd /workspace/colcon_ws
colcon build --symlink-install --packages-select amr_butler_description
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch amr_butler_description butler_sim.launch.py
```

Launch arguments (defaults): `namespace:=butler1`, `gui:=true` (false = headless
server, sensors still render), `rviz:=true`, `world:=<butler_test.sdf>`,
`x/y/yaw:=0`, `launch_gazebo:=true`, `bridge_clock:=true`.

Note: this container runs on `ROS_DOMAIN_ID=42` — topics are not visible from
the Classic container unless you match domain IDs deliberately.

## Expected topics

```
$ ros2 topic list
/butler1/camera/camera_info      sensor_msgs/CameraInfo
/butler1/camera/depth/image_raw  sensor_msgs/Image        (rgbd depth, 640x480 @ 10 Hz)
/butler1/camera/image_raw        sensor_msgs/Image        (rgb, 640x480 @ 10 Hz)
/butler1/cmd_vel                 geometry_msgs/Twist      (ros -> gz)
/butler1/joint_states            sensor_msgs/JointState
/butler1/odom                    nav_msgs/Odometry        (DiffDrive)
/butler1/robot_description       std_msgs/String
/butler1/scan                    sensor_msgs/LaserScan    (360°, 8 m, 10 Hz)
/clock                           rosgraph_msgs/Clock
/tf, /tf_static                  tf2_msgs/TFMessage
```

TF chain: `butler1/odom → butler1/base_footprint` (DiffDrive via bridge)
`→ butler1/base_link → sensors` (robot_state_publisher). The global `map` frame
arrives later with Nav2/AMCL; RViz's fixed frame defaults to `butler1/odom`.

## Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/butler1/cmd_vel
```

(`ros-humble-teleop-twist-keyboard` is pre-installed in the amr-butler-dev image.)

## Multiple robots (later)

- Spawn extra robots by reusing the launch with `launch_gazebo:=false`,
  `bridge_clock:=false` and a new `namespace` / `x` / `y` — all gz topics are
  model-scoped, `/clock` must be bridged exactly once per simulation.
- The saved RViz config hardcodes `butler1` topic names; other namespaces need
  their own config (RViz files cannot be parameterized).

## Known quirks

- The slim pole holding the pillar crosses the lidar plane: ~10 rays return a
  constant ~0.35 m hit behind the robot. The points are inside the robot
  footprint (chassis radius 0.35 m), so Nav2 costmaps ignore them.
- The camera image is expressed in `butler1/camera_link_optical` (REP-103
  optical convention, z forward), tilted 0.15 rad downward.

## Tests

```bash
colcon test --packages-select amr_butler_description && colcon test-result --verbose
```

Covers: xacro → URDF expansion (tree validity, namespaced frames, physical
inertias, model-scoped gz topics), bridge.yaml validity + topic contract,
launch file import, flake8/pep257. The launch-description test skips itself
outside the amr-butler-dev container (needs `ros_gz_sim`).
