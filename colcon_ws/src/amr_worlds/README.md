# amr_worlds

Restaurant world for the **Gazebo Harmonic (gz-sim 8)** stack, plus the mapping
workflow that produces the occupancy map consumed by Nav2 in the next phase.
Runs inside the **`amr-butler-dev`** container (`docker/README.butler.md`,
`ROS_DOMAIN_ID=42`) together with `amr_butler_description`. Primitive geometry
only (boxes/cylinders, flat colors): no meshes, no Gazebo Fuel downloads —
VM/software-rendering friendly.

## World layout (`worlds/restaurant.sdf`, 16 m x 10 m)

```
  y=+5 ┌───────────────────────────────┬─────[bar_counter]─────┐
       │ [kitchen_shelf]               │(5.0,4.5)              │
       │            KITCHEN            │ (p1)                  │
       │                       kitchen │(-4.3,4)               │
       │                       wall    │ [table_1] [table_2] [table_3]
       │  [kitchen_counter]            │ (-2.5,2.2) (1.0,2.2) (4.5,2.2)
  y=0  │  (-5.5,-2.3)               ═══╡ ← doorway 1.4 m at (-5, 0)
       │        food pickup            │
       │                               │ [table_4] [table_5] [table_6]
       │                               │ (-2.5,-2.2)(1.0,-2.2)(4.5,-2.2)
       │                               │            (p2)  [divider]  ▣ dock
       │                               │          (0.5,-4.4)(3.2,-3.4)(7,-4.3)
  y=-5 └───────────────────────────────┴──────────═══════──────────────┘
      x=-8                                    entrance 2 m         x=+8
                                              (2.2 < x < 4.2)
```

Named models (the basis for `semantic_map.yaml` in the next phase — the full
list with coordinates is in the comment block at the top of `restaurant.sdf`):
`table_1`…`table_6`, `kitchen_counter`, `bar_counter`, `charging_dock`.
Obstacles: two plant pots (p1/p2), an entrance divider, a kitchen shelf.

Design notes:

- Tables stand on **four cylinder legs** — the butler lidar scans at z = 0.25 m
  and must see legs, not tabletops. Chairs (2–4 per table) are seat + 4 legs.
- All aisles between table/chair envelopes are **>= 1.4 m** for the 0.70 m robot.
- The `charging_dock` pad is **visual-only** (green, below the lidar plane) so
  the robot can drive onto it without a collision bump; the wall-mounted
  charger box is solid.

## Launch

```bash
# inside amr-butler-dev (cd docker && docker compose up -d amr-butler-dev)
cd /workspace/colcon_ws
colcon build --symlink-install --packages-select amr_worlds
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch amr_worlds restaurant.launch.py namespace:=butler1 gui:=true rviz:=true
```

The robot spawns at (6.3, -4.0, yaw pi), next to the `charging_dock`.
Arguments: `namespace` (default `butler1`), `gui`, `rviz`, `x`, `y`, `yaw` —
all forwarded to `amr_butler_description/launch/butler_sim.launch.py`.

## Mapping workflow (slam_toolbox)

1. **Launch the mapping stack** (sim with `rviz:=false` + slam_toolbox
   online-async + RViz mapping view):

   ```bash
   ros2 launch amr_worlds mapping.launch.py namespace:=butler1
   ```

   `config/slam_params.yaml` is a `{NS}` template (like the butler
   `bridge.yaml`): frames resolve to `butler1/base_footprint`,
   `butler1/odom`, global `map`, scan topic `/butler1/scan`, sim time on.

2. **Teleop-drive the robot** from a **second terminal** in the same container
   (`docker exec -it amr-butler-dev bash`, source the workspace). The remap to
   the namespaced cmd_vel topic is required:

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
       --ros-args -r cmd_vel:=/butler1/cmd_vel
   ```

   Drive slowly through the dining area, around every table, through the
   kitchen doorway and along the walls until the map in RViz has no gray
   frontiers left.

3. **Save the map** (third terminal, or stop teleop first):

   ```bash
   ros2 run nav2_map_server map_saver_cli \
       -f /workspace/colcon_ws/src/amr_worlds/maps/restaurant_map \
       --ros-args -p use_sim_time:=true
   ```

   This writes `restaurant_map.yaml` + `restaurant_map.pgm` into
   `maps/` (kept in git via `.gitkeep`).

4. **Rebuild** so the map is installed into the package share
   (`data_files` globs `maps/*`; new files need a rebuild even with
   `--symlink-install`):

   ```bash
   cd /workspace/colcon_ws && colcon build --packages-select amr_worlds
   ```

> The saved `maps/restaurant_map.yaml` is consumed by **Nav2** (map_server +
> AMCL) in the next phase; the named-model coordinates in `restaurant.sdf`
> become `semantic_map.yaml` for the concierge/butler task layer.

## Tests

```bash
colcon test --packages-select amr_worlds && colcon test-result --verbose
```

- `test_restaurant_sdf.py` — XML structure, plugin set (Physics, UserCommands,
  SceneBroadcaster, Sensors/ogre2), required named models, table legs crossing
  the lidar plane; runs `gz sdf --check` when a Harmonic-capable `gz` CLI
  exists (skipped with the Classic CLI, which only understands SDF <= 1.7).
- `test_slam_params.py` — template substitution + namespaced frame keys.
- `test_launch_import.py` — both launch files import and build.
- flake8 / pep257 style tests (as in the other packages).
