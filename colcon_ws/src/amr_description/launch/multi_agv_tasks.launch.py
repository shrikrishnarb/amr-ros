# multi_agv_tasks.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # --- Only argument we accept ---
    num_agvs = int(context.launch_configurations.get('num_agvs', '1'))
    num_agvs = max(1, num_agvs)

    # Use simulation time everywhere
    use_sim_time = True
    delay_start_time = 0.0

    # Resolve tasks.yaml from package share
    pkg_share = get_package_share_directory('amr_description')
    tasks_file = os.path.join(pkg_share, 'yaml', 'tasks.yaml')

    if not os.path.exists(tasks_file):
        # Fail early with a clear message (don’t try to pass nested dict params).
        raise RuntimeError(
            f"[multi_agv_tasks.launch] tasks.yaml not found at: {tasks_file}\n"
            f"Expected file: amr_description/config/tasks.yaml (installed to share)"
        )

    robot_namespaces = [f"agv{i}" for i in range(1, num_agvs + 1)]
    nodes = []

    # Optional: show what we’re launching
    nodes.append(LogInfo(msg=f"[multi_agv_tasks] Starting for AGVs: {robot_namespaces}"))
    nodes.append(LogInfo(msg=f"[multi_agv_tasks] Using tasks file: {tasks_file}"))

    # Per-AGV nodes
    for ns in robot_namespaces:

        # Motion controller (executes /<ns>/goal_pose -> /<ns>/cmd_vel)
        nodes.append(Node(
            package="amr_description",
            executable="ground_truth_waypoint_follower.py",
            name="ground_truth_waypoint_follower",
            namespace=ns,
            parameters=[{
                "use_sim_time": use_sim_time,
                "delay_start_time": delay_start_time,
                "robot_namespace": ns,
                # If you installed the gradual slow-down version, you can tune here:
                # "slow_down_start_dist": 1.3,
                # "stop_dist": 0.5,
                # "forward_sector_deg": 25.0,
                "shutdown_when_idle": False,  # stay alive waiting for next goal
            }],
            output="screen",
            emulate_tty=True,
        ))

        # Obstacle detection -> publishes /<ns>/obstacle_detected
        nodes.append(Node(
            package="amr_description",
            executable="obstacle_detection_node",
            name="obstacle_detection",
            namespace=ns,
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_namespace": ns,
            }],
            output="screen",
            emulate_tty=True,
        ))

        # Charging proximity monitor -> publishes /<ns>/on_charger
        # Uses /<ns>/ground_truth (pose + velocity) instead of /odom
        nodes.append(Node(
            package="amr_description",
            executable="charger_dock_monitor",
            name="charger_dock_monitor",
            namespace=ns,
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_namespace": ns,
                "odom_topic": f"/{ns}/ground_truth",
                "contact_topic": f"/{ns}/on_charger",
                # If your charger centers change in tasks.yaml, update here to match.
                # Defaults in the node are [-20,-10, 20,-10]; you can override:
                "charger_centers_xy": [-20.0, -10.0, 20.0, -10.0],
                "enter_radius": 0.8,
                "exit_radius": 1.1,
                "min_dock_dwell_sec": 0.5,
                "max_linear_speed": 0.15,
                "publish_rate_hz": 5.0,
            }],
            output="screen",
            emulate_tty=True,
        ))

        # (Optional) Battery simulator. Comment out if you don’t run it.
        # Ensure your battery_sim publishes either:
        #  - sensor_msgs/BatteryState  on /<ns>/battery_state
        #  - std_msgs/Float32 (0..1 or 0..100) on /<ns>/battery_percentage
        nodes.append(Node(
            package="amr_description",
            executable="battery_sim",
            name="battery_sim",
            namespace=ns,
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_namespace": ns,
            }],
            output="screen",
            emulate_tty=True,
        ))

    # Fleet Manager (single instance)
    nodes.append(Node(
        package="amr_description",
        executable="fleet_manager",
        name="fleet_manager",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_namespaces": robot_namespaces,  # e.g., ["agv1","agv2"]
            "tasks_file": tasks_file,              # let the node load zones & tasks
            "battery_topic_type": "auto",          # auto-detect BatteryState/Float32
            # Charger zone names must match those in tasks.yaml:
            "charger_zone_names": ["charger_1", "charger_2"],
            # Thresholds (tune as needed)
            "battery_low_threshold": 0.20,
            "battery_resume_threshold": 0.60,
            "goal_reach_dist": 0.25,
        }],
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("num_agvs", default_value="1"),
        OpaqueFunction(function=launch_setup),
    ])