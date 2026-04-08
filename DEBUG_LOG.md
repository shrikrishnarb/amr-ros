# DEBUG LOG — AMR Fleet Navigation Debugging

## Current Status
Navigation is PARTIALLY WORKING. Robot spawns, Nav2 starts, AMCL receives scan, goal is sent. Robot movement in Gazebo is unconfirmed (session ended before result).

---

## Issue 1: Missing TF (agv1/odom)
- **Error:** Transform agv1/base_footprint → agv1/odom not available
- **Cause:** odom_sim_filter not receiving ground truth data
- **Root Cause:** Gazebo was not running
- **Fix Applied:** Started Gazebo using display.launch.py
- **Status:** ✅ Fixed

---

## Issue 2: Duplicate Nodes / Architecture Conflict
- **Error:** Conflicts between display.launch.py and amr_fleet_management.launch.py
- **Cause:** Both launch files start overlapping nodes (odom_sim_filter, etc.)
- **Fix Applied:** Rewrote amr_fleet_management.launch.py to be self-contained:
  - Starts Gazebo
  - Spawns robot
  - Runs Nav2 stack
  - Runs fleet_manager
- **Status:** ✅ Fixed

---

## Issue 3: AMCL Not Localizing Initially
- **Error:** nav2_goal_bridge stuck retrying setInitialPose
- **Cause:** AMCL not publishing amcl_pose
- **Root Cause:** Missing sensor input
- **Status:** ✅ Fixed — AMCL now calls createLaserObject and converges once scan arrives

---

## Issue 4: Lidar / Scan Topic Not Publishing
- **Error:** /agv1/scan topic exists but not publishing
- **Impact:** AMCL cannot localize properly
- **Root Cause:** Lidar sensor not working correctly in simulation
- **Status:** ✅ Fixed — scan is now publishing (AMCL confirmed receipt via createLaserObject log)

---

## Issue 5: LiDAR Only 180° Coverage (Front Half)
- **Observation:**
  - LiDAR min_angle=-1.5708, max_angle=1.5708 → only ±90° = 180° front arc
  - AMCL reset storm: BasicNavigator retries setInitialPose every ~100ms until amcl_pose published
  - Limited scan coverage means AMCL takes longer to converge
- **Root Cause:** URDF lidar scan angle range too narrow
- **Fix Applied:** Extended scan to full 360° (min=-3.14159, max=3.14159) in amr.urdf.xacro
- **Status:** ✅ Fixed (rebuilt required)

---

## Issue 6: Navigation Result Unknown
- **Observation:**
  - Latest session (2026-04-03-05-03-27): all systems start correctly
  - Fleet manager assigns task T2 → pickup_zone_B
  - nav2_goal_bridge sends goal (12, -6) to Nav2
  - Log ends at "Navigating to goal: 12.0 -6.0..." — session was stopped manually
- **Impact:** Cannot confirm if robot physically moves in Gazebo
- **Status:** ❌ NOT CONFIRMED — need longer run to observe navigation result

---

## System State (Last Known — session 2026-04-03-05-03-27)
- Gazebo: ✅ Running (diff_drive confirmed, odometry publishing on /agv1/odom)
- TF tree: ✅ Working (odom → base_footprint, costmap started)
- Nav2: ✅ Active (all servers activated)
- AMCL: ✅ Working (received scan, createLaserObject called, amcl_pose published)
- Lidar (/scan): ✅ Publishing
- Planning: ⚠️ Goal sent, result unknown
- Robot movement: ⚠️ Unknown (need longer run)

---

## Latest Conclusion
System is now mostly functional. The primary remaining unknown is whether the robot actually moves to its goal.

👉 PRIMARY ACTION NEEDED:
- Run `ros2 launch amr_description amr_fleet_management.launch.py` and let it run >60s
- Watch for: navigation result (SUCCEEDED/FAILED) in nav2_goal_bridge log
- Watch for: robot movement in Gazebo

👉 FIXED THIS SESSION:
- LiDAR scan extended from 180° to 360° (better AMCL localization)

---

## Next Steps
1. Run full simulation and observe for 60+ seconds:
   - Check nav2_goal_bridge log for SUCCEEDED/FAILED
   - Check if robot moves in Gazebo

2. If navigation FAILS after goal sent:
   - Check planner_server log for "No path found" errors
   - Check controller_server log for DWB failures
   - Verify map frame alignment (world→map TF is identity: OK)

3. If robot doesn't move but SUCCEEDED:
   - Check cmd_vel is reaching diff_drive
   - Verify diff_drive is subscribed to /agv1/cmd_vel

---

---

## Issue 9: Goals lost — race condition between fleet_manager and nav2_goal_bridge startup
- **Error:** Robots stuck in `TO_PICKUP` forever; `cmd_vel` never published
- **Cause:** fleet_manager subscribes to `/{ns}/ground_truth` (available from Gazebo immediately) → assigns tasks and publishes `goal_pose` at ~t=2s. nav2_goal_bridge is still in `waitUntilNav2Active()` (takes 30+ seconds for Nav2 to activate) → the goal_pose message has no subscriber, gets dropped. fleet_manager moves to `TO_PICKUP` state and never re-sends the goal.
- **Fix Applied:** Added `_resend_goal_if_due()` helper in `fleet_manager.py`. In `TO_PICKUP`, `TO_DROPOFF`, and `TO_CHARGER` states, re-sends the navigation goal every 5 seconds if the robot hasn't arrived yet. This ensures nav2_goal_bridge receives the goal once Nav2 finishes activating.
- **Status:** ✅ Fixed

---

## Issue 8: AMCL stuck — TF topic mismatch in fleet-only mode
- **Error:** `nav2_goal_bridge` stuck in infinite `waitUntilNav2Active()` loop; `amcl_pose` never published
- **Cause:** TF topic mismatch between the infrastructure and Nav2
  - `display.launch.py` / `spawn_agv.launch.py` start `odom_sim_filter` and `robot_state_publisher` WITHOUT TF remapping → publish `agv1/odom → agv1/base_footprint` TF to global `/tf`
  - `nav2_bringup` with `use_namespace:=True` adds `remappings=[('/tf', 'tf')]` to `nav2_container` → AMCL subscribes to `/{ns}/tf`
  - Result: AMCL looks on `/agv1/tf`, odom_sim_filter publishes on `/tf` → AMCL never gets the TF chain → message filter drops scans → no `amcl_pose`
- **Evidence:** `Message Filter dropping message: frame 'agv1/base_footprint' at time X for reason 'the timestamp on the message is earlier than all the data in the transform cache'`
- **Fix Applied:**
  - Added `tf_relay.py` node to `amr_description` package
  - Relay subscribes to `/tf` (global) and re-publishes to `/{ns}/tf`; same for `/tf_static` → `/{ns}/tf_static`
  - One relay started per robot in `amr_fleet_management.launch.py` (before Nav2 bringup)
  - Uses `TRANSIENT_LOCAL` QoS for `/tf_static` relay so late-joiners get static transforms
- **Status:** ✅ Fixed

---

## Issue 7: `NameError: name 'false' is not defined` in nav2_bringup
- **Error:** `launch_service.py` crashes with `NameError: name 'false' is not defined`
- **Cause:** `amr_fleet_management.launch.py` passed `'slam': 'false'` (lowercase) to `bringup_launch.py`
- **Root Cause:** `nav2_bringup/bringup_launch.py` line 155 uses `IfCondition(PythonExpression(['not ', slam]))`. `PythonExpression` evaluates the string as Python code — `not false` is invalid Python (`False` with capital F is correct)
- **Fix Applied:** Changed all boolean launch_arguments for nav2_bringup to use Python-capitalised values: `'False'` / `'True'` instead of `'false'` / `'true'`
- **Status:** ✅ Fixed — launch now starts cleanly without NameError

---

---

## Issue 10: Two-robot dynamic obstacle avoidance broken — path computed once per goal
- **Observation:** With both AGVs running, a dynamic obstacle (including the other robot) causes the AMR to stop and fail to reroute, even though single-robot avoidance worked fine.
- **Root Cause (3 layers):**
  1. `navigate_w_recovery.xml` used plain `<Sequence>` — global path computed ONCE at goal assignment, never updated during navigation. The BT even had a comment "No periodic replanning".
  2. `global_costmap.update_frequency: 1.0 Hz` — obstacle positions updated only once per second; global planner unaware of fast-moving robots.
  3. `progress_checker.movement_time_allowance: 30.0` — robot waits 30 s of no progress before triggering recovery.
- **Fix Applied:**
  1. Rewrote BT: replaced `<Sequence>` with `<PipelineSequence>` + `<RateController hz="2.0">` wrapping `ComputePathToPose` — global path now recomputed every 0.5 s while the robot is moving. Other robot positions in the costmap are seen immediately by the global planner → smooth rerouting instead of stopping.
  2. `global_costmap.update_frequency`: 1.0 → 5.0 Hz; `publish_frequency`: 1.0 → 2.0 Hz — global planner sees obstacle changes within 0.2 s.
  3. Added `decay_time: 3.0` to global costmap obstacle layer — other robot's old position clears from the costmap 3 s after it moves, preventing phantom obstacles.
  4. `progress_checker.movement_time_allowance`: 30.0 → 10.0 s — stuck robot triggers recovery and replanning faster.
- **Status:** ✅ CONFIRMED WORKING
  - agv2: At dropoff dropoff_zone_B → Task T1 complete
  - agv1: At dropoff dropoff_zone_A → Task T2 complete
  - Both robots crossed each other in the center of the map without collision
  - Average navigation speed: ~0.13 m/s through complex environment

---

## Notes
- Fleet manager is assigning goals correctly
- Nav2 pipeline is functional
- AMCL convergence improved significantly once lidar started publishing
- The 360° LiDAR fix should eliminate the ~10-retry AMCL reset storm
