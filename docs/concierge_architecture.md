# AMR Concierge — Planned Architecture

**Status: design only — nothing in this document is implemented yet.**

Goal: extend the existing AMR fleet (see `CLAUDE.md`) with a natural-language "concierge"
layer for a restaurant scenario. A user says *"take this order to table 4, then check if
anyone is waiting at the entrance and come back"*; a VLM planner turns that into a JSON plan
of skills; a skill executor runs the plan through the **existing, unmodified** navigation
stack; language tasks join the fleet task queue alongside pickup/dropoff tasks.

## Simulator stack decision (2026-07)

The new packages target **Gazebo Harmonic (gz-sim 8)** with ROS 2 Humble, using the
non-official `ros-humble-ros-gzharmonic` binaries from packages.osrfoundation.org. The
existing packages (`amr_description`, `amr_vision`) stay on **Gazebo Classic, unchanged**.
The two stacks cannot coexist in one container (ros_gz Harmonic packages conflict with the
gazebo-classic ROS packages), so a second docker compose service **`amr-butler-dev`** will be
added, sharing the same `colcon_ws` volume as the existing `amr-ros-dev` service.

New-stack conventions:

- Worlds authored in **modern SDF** (`amr_worlds`); robots spawned via **`ros_gz_sim` `create`**.
- ROS ↔ gz topics bridged with **`ros_gz_bridge`** (`parameter_bridge` or a bridge YAML
  config). The bridge is the *only* component where gz-side topic names appear; everything
  downstream sees the same `/<ns>/...` ROS topics documented below.
- Sensors/actuation via **gz-sim system plugins**: `DiffDrive`, `OdometryPublisher`,
  `JointStatePublisher`, `gpu_lidar`, `rgbd_camera`.
- Sim time bridged via `/clock` (nodes keep using `use_sim_time`).
- **Dependency rule:** never add `gazebo_ros`/gazebo-classic dependencies to the new packages;
  never add `ros_gz` dependencies to the old ones.
- `fleet_manager`, `nav2_goal_bridge`, and **all `amr_concierge` nodes are
  simulator-agnostic** — plain ROS 2 topics/actions only. Provided the bridge reproduces the
  topic contract in the table below, the concierge layer runs identically against Classic or
  Harmonic.

## Constraints inherited from the existing code

These are the actual interfaces the new code must use (verified in the repo):

| Interface | Type | Where defined |
|---|---|---|
| `/<ns>/goal_pose` | `geometry_msgs/PoseStamped`, `frame_id: "map"` | consumed by `nav2_goal_bridge` (`amr_description/nav2_goal_bridge.py`), published by `fleet_manager` |
| `/<ns>/goal_reached` | `std_msgs/Bool` (True = succeeded, False = failed) | published by `nav2_goal_bridge` on Nav2 task completion |
| `navigate_to_pose` (namespaced → `/<ns>/navigate_to_pose`) | Nav2 action, driven via `BasicNavigator(namespace=ns)` | inside `nav2_goal_bridge` — **new code should NOT call it directly**; publish to `/<ns>/goal_pose` instead |
| `/<ns>/camera/image_raw` | `sensor_msgs/Image` | Gazebo camera plugin in `amr_vision/urdf/amr_vision.urdf.xacro` |
| `/<ns>/camera/detections` | `std_msgs/String` (JSON: `{timestamp, detections:[{class_name, confidence, bbox}], inference_ms, frame_id}`) | `amr_vision/camera_detection_node.py` |
| `/<ns>/camera/obstacle_semantic` | `std_msgs/String` (`clear\|person\|pallet\|unknown_obstacle`) | `amr_vision/camera_detection_node.py` |
| `/<ns>/ground_truth` | `nav_msgs/Odometry` (Gazebo p3d, `world` frame) | used by `fleet_manager` for zone-arrival checks |
| `/<ns>/battery_state`, `/<ns>/on_charger`, `/<ns>/carrying_load` | `sensor_msgs/BatteryState`, `Bool`, `Bool` | `battery_sim`, `charger_dock_monitor`, `fleet_manager` |
| Zones/tasks schema | YAML: `zones: {name: {cx, cy, sx, sy, yaw}}`, `tasks: [...]` | `amr_description/yaml/tasks.yaml`, loaded once at startup by `fleet_manager` via the `tasks_file` parameter |
| Node names | `fleet_manager` (base) / `fleet_manager_ai` (vision), `nav2_goal_bridge`, `tf_relay`, `odom_sim_filter`, `camera_detection_node_<ns>` | launch files in both packages |

Under Gazebo Harmonic, the sensor/actuation topics in this table (`/<ns>/cmd_vel`,
`/<ns>/odom`, `/<ns>/scan`, `/<ns>/camera/image_raw`, `/<ns>/ground_truth`, `/clock`) are
produced by gz-sim system plugins and mapped to these exact ROS names by `ros_gz_bridge`
config in `amr_worlds` / `amr_butler_description` — the contract stays identical for every
node above the bridge.

Key implication: `fleet_manager` has **no runtime task-injection interface** (queue is read
from `tasks.yaml` at startup and never amended). Since `amr_description` must not be modified,
fleet integration follows the precedent set by `amr_vision/fleet_manager_ai.py`: a **new node
in a new package that extends the FSM by copy/subclass** and adds a task-submission topic.

## Architecture

```mermaid
flowchart TB
    subgraph amr_concierge [amr_concierge (NEW)]
        UI["/concierge/command\n(std_msgs/String, natural language)"]
        VLM[vlm_planner\nGemini API primary\nGroq API fallback]
        PLAN["/concierge/plan\n(String, JSON skill list)"]
        EXEC[skill_executor\none per robot ns]
        SEM[semantic_map.yaml\nnamed locations → map x,y,yaw]
        GROUND[vision_grounding\nlook_for resolver]
        FMC[fleet_manager_concierge\nFSM copied from fleet_manager,\n+ /concierge/task_request sub]
    end

    subgraph existing [Existing packages (UNMODIFIED)]
        BRIDGE[nav2_goal_bridge]
        NAV2[Nav2 stack per robot]
        CAM[camera_detection_node]
        GZ[Simulator: Gazebo Classic today /\nGazebo Harmonic + ros_gz_bridge for new worlds]
    end

    UI --> VLM
    SEM --> VLM
    VLM --> PLAN --> FMC
    FMC -- "assigns language task" --> EXEC
    FMC -- "pickup/dropoff tasks\n(tasks.yaml, unchanged flow)" --> BRIDGE
    EXEC -- "/agvN/goal_pose (PoseStamped, map frame)" --> BRIDGE
    BRIDGE -- "navigate_to_pose action" --> NAV2
    BRIDGE -- "/agvN/goal_reached (Bool)" --> EXEC
    EXEC -- "look_for skill" --> GROUND
    GROUND -- "/agvN/camera/detections (JSON)" --> CAM
    GZ -- "/agvN/camera/image_raw" --> CAM
    GZ -- "/agvN/ground_truth" --> FMC
```

## Components

### 1. `vlm_planner` (node, `amr_concierge`)
- Subscribes `/concierge/command` (`std_msgs/String`, free-form natural language).
- Calls **Gemini API** (primary). On error/timeout/quota → **automatic fallback to Groq API**
  with the same prompt. Provider order, model names, timeouts and retry counts come from
  `concierge_params.yaml`; API keys from environment variables (`GEMINI_API_KEY`,
  `GROQ_API_KEY`) — never from YAML committed to git.
- Prompt includes the location names from `semantic_map.yaml` so the model can only ground to
  known places; output is validated against a JSON schema, invalid plans are rejected and
  retried once with the validation error appended.
- Publishes the validated plan on `/concierge/plan` (`std_msgs/String`, JSON).

Planned plan format (one plan = ordered skill list):

```json
{
  "task_id": "lang-2026-07-10-001",
  "command": "bring the menu to table 4 and announce it",
  "skills": [
    {"skill": "navigate_to", "target": "table_4"},
    {"skill": "announce", "text": "Here is your menu."},
    {"skill": "wait", "seconds": 5.0},
    {"skill": "look_for", "object": "person", "timeout": 10.0},
    {"skill": "return_to_dock"}
  ]
}
```

Skill set (v1): `navigate_to(target)`, `look_for(object, timeout)`, `announce(text)`,
`wait(seconds)`, `return_to_dock()`.

### 2. `skill_executor` (node, `amr_concierge`, one per robot namespace)
- Parameter `robot_namespace` (same pattern as every existing node).
- Executes one plan at a time, skill by skill:
  - `navigate_to`: resolve `target` via `semantic_map.yaml` → publish `PoseStamped`
    (`frame_id: "map"`) on `/<ns>/goal_pose`; wait for `/<ns>/goal_reached`. Mirror the
    fleet_manager convention of **re-sending the goal every 10 s** until reached (the bridge
    de-duplicates goals within 5 cm, so re-sends are safe — see `nav2_goal_bridge._on_goal`).
  - `look_for`: delegate to `vision_grounding` (service or topic round-trip), succeed/fail
    within `timeout`.
  - `announce`: publish `/<ns>/announce` (`std_msgs/String`); v1 just logs / publishes for a
    UI, no audio.
  - `wait`: timer.
  - `return_to_dock`: `navigate_to` the dock entry in `semantic_map.yaml` (reuse the existing
    charger zones `charger_1`/`charger_2` at (−20,−10)/(20,−10) or a new `dock` entry).
- Publishes skill-level progress on `/concierge/status` (String JSON: task_id, skill index,
  state) so the fleet layer and UI can track execution.
- Failure policy: a failed skill (`goal_reached == False` after N retries, or `look_for`
  timeout) aborts the remaining plan and reports failure on `/concierge/status`.

### 3. `semantic_map.yaml` (config, `amr_concierge`)
Named restaurant locations in the **`map` frame** (same frame `goal_pose` requires). Schema
deliberately compatible with the existing zone format (`cx, cy, yaw`) so entries can be
cross-referenced with `tasks.yaml` zones:

```yaml
locations:
  table_1:   {cx: -12.0, cy: -6.0, yaw: 0.0, aliases: ["first table"]}
  table_4:   {cx:  15.0, cy: 10.0, yaw: 1.57}
  kitchen:   {cx: -15.0, cy: 10.0, yaw: 3.14, aliases: ["the pass"]}
  entrance:  {cx:   0.0, cy: -8.0, yaw: 0.0, aliases: ["front door"]}
  dock:      {cx: -20.0, cy: -10.0, yaw: 0.0}   # co-located with existing charger_1 zone
```

Coordinates above are placeholders pending the new restaurant world in `amr_worlds`; the
`dock` default reuses `charger_1` from `amr_description/yaml/tasks.yaml`.

### 4. `vision_grounding` (node, `amr_concierge`)
- Consumes the existing `/<ns>/camera/detections` JSON stream from
  `amr_vision/camera_detection_node.py` (COCO classes; currently only `person` and
  `pallet`/suitcase are mapped — extending the class map means a **new** detection config in
  `amr_concierge`, not editing `amr_vision`).
- For objects outside the YOLO class map, falls back to a VLM image query: grab one frame from
  `/<ns>/camera/image_raw`, send to Gemini (Groq fallback follows the same provider chain as
  `vlm_planner`) with "is <object> visible?".
- Exposes `look_for` as a ROS service (`/<ns>/look_for`) returning found/not-found +
  confidence; used by `skill_executor`.

### 5. `fleet_manager_concierge` (node, `amr_concierge`)
- Follows the `fleet_manager_ai` precedent: FSM copied/extended from
  `amr_description/fleet_manager.py`, original packages untouched.
- Adds a subscription `/concierge/task_request` (String JSON, published by `vlm_planner` or a
  thin adapter on `/concierge/plan`): language tasks are appended to the same in-memory
  `task_queue` used for YAML pickup/dropoff tasks, as a new task type
  `{type: "language", plan: {...}}` alongside `{type: "transport", pickup, dropoff, ...}`.
- Assignment stays greedy-by-distance for transport tasks; language tasks go to the nearest
  IDLE robot, whose namespace is handed to that robot's `skill_executor`
  (`/concierge/assign/<ns>`). New FSM state `EXECUTING_PLAN` (entered from IDLE, returns to
  IDLE on `/concierge/status` completion) so battery preemption (`TO_CHARGER`) still works.
- Replaces `fleet_manager`/`fleet_manager_ai` in the concierge launch file (only one fleet
  manager may run at a time — they'd double-publish `/<ns>/goal_pose`).

## Planned package layout

```
colcon_ws/src/
├── amr_butler_description/        # NEW — butler robot model, Gazebo Harmonic (ros_gz)
│   ├── urdf/amr_butler.urdf.xacro #   same frame/topic naming conventions as amr_vision,
│   │                              #   but gz-sim system plugins (DiffDrive, OdometryPublisher,
│   │                              #   JointStatePublisher, gpu_lidar, rgbd_camera)
│   ├── config/bridge.yaml         #   ros_gz_bridge topic mapping (incl. /clock)
│   ├── launch/                    #   spawn via ros_gz_sim create + parameter_bridge
│   ├── README.md
│   └── test/                      #   pytest (e.g. xacro parses, frames prefixed)
├── amr_worlds/                    # NEW — restaurant world, Gazebo Harmonic
│   ├── worlds/restaurant.sdf      #   modern SDF
│   ├── maps/restaurant.{pgm,yaml} #   occupancy map for Nav2/AMCL
│   ├── README.md
│   └── test/
└── amr_concierge/                 # NEW — language layer
    ├── amr_concierge/
    │   ├── vlm_planner.py
    │   ├── skill_executor.py
    │   ├── vision_grounding.py
    │   └── fleet_manager_concierge.py
    ├── yaml/
    │   ├── semantic_map.yaml
    │   └── concierge_params.yaml  #   provider config, timeouts, skill params
    ├── launch/concierge_fleet.launch.py   # composes existing Nav2/bridge/tf_relay pattern
    ├── README.md
    └── test/                      #   pytest: plan schema validation, semantic map lookup,
                                   #   skill sequencing with mocked goal_reached
```

All nodes: Python/rclpy, params via YAML + `declare_parameter`, topics namespaced `/<ns>/...`.

## Open design risks

See the "known fragile points" list in the completion report / CLAUDE.md — notably: the
`on_charger` vs `charger_contact` topic mismatch, ground-truth-based arrival checks, the
tasks.yaml startup-only queue, hardcoded `agvN` namespaces, and Gazebo Classic (EOL) specifics.
The concierge layer deliberately touches none of these mechanisms directly — it only speaks
`/<ns>/goal_pose` / `/<ns>/goal_reached` and the camera topics.

Additional risk from the simulator split: the `ros-humble-ros-gzharmonic` binaries are
non-official (Humble's official pairing is Gazebo Fortress), so upgrades from
packages.osrfoundation.org may break; and the two-container setup means the old and new worlds
can never run in the same simulation — cross-stack testing happens only at the ROS topic level
over the shared DDS network / shared `colcon_ws` volume.
