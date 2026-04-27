# amr_vision_fleet.launch.py
#
# AI-enabled AMR Fleet launch — two modes:
#
#   Fleet-only mode (default, launch_gazebo:=false):
#     Assumes display_vision.launch.py + spawn_agv.launch.py are already running.
#     Starts: Nav2 stack + nav2_goal_bridge + fleet_manager_ai only.
#     Usage (3-terminal workflow):
#       Terminal 1: ros2 launch amr_vision display_vision.launch.py namespace:=agv1
#       Terminal 2: ros2 launch amr_vision display_vision.launch.py namespace:=agv2 x:=2.0 y:=2.0
#       Terminal 3: ros2 launch amr_vision amr_vision_fleet.launch.py num_agvs:=2 spawn_poses:="0.0,0.0;2.0,2.0"
#
#   Standalone mode (launch_gazebo:=true):
#     Self-contained: starts Gazebo, spawns robots, all per-robot nodes, and fleet_manager_ai.
#     Usage:
#       ros2 launch amr_vision amr_vision_fleet.launch.py num_agvs:=2 launch_gazebo:=true spawn_poses:="0.0,0.0;2.0,2.0"
#
# Arguments:
#   num_agvs      (int, default 1)   — number of robots (must be named agv1, agv2, ...)
#   launch_gazebo (bool, default false) — set true for standalone mode
#   spawn_poses   (str, default "0.0,0.0") — semicolon-separated "x,y" pairs, one per robot.
#                 Used for Nav2 initial pose (AMCL seed) and spawn position in standalone mode.
#   model_path    (str, default "yolov8n.pt") — path to YOLO model weights file passed to
#                 each camera_detection_node instance.

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    num_agvs = max(1, int(context.launch_configurations.get('num_agvs', '1')))
    launch_gazebo = context.launch_configurations.get('launch_gazebo', 'false').lower() == 'true'
    model_path = context.launch_configurations.get('model_path', 'yolov8n.pt')  # CHANGE A

    spawn_poses_str = context.launch_configurations.get('spawn_poses', '0.0,0.0')
    spawn_poses: list[tuple[float, float]] = []
    for pair in spawn_poses_str.split(';'):
        pair = pair.strip()
        if not pair:
            continue
        parts = pair.split(',')
        try:
            spawn_poses.append((float(parts[0]), float(parts[1])))
        except (IndexError, ValueError):
            spawn_poses.append((0.0, 0.0))

    def _spawn_pose(idx: int) -> tuple[float, float]:
        return spawn_poses[idx] if idx < len(spawn_poses) else (float(idx) * 2.0, 0.0)

    pkg_share = get_package_share_directory('amr_vision')  # CHANGE 1
    tasks_file = os.path.join(pkg_share, 'yaml', 'tasks.yaml')
    nav2_params_template = os.path.join(pkg_share, 'yaml', 'nav2_params_amr.yaml')
    map_file = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    if not os.path.exists(tasks_file):
        raise RuntimeError(f'[amr_vision_fleet] tasks.yaml not found: {tasks_file}')
    if not os.path.exists(nav2_params_template):
        raise RuntimeError(f'[amr_vision_fleet] nav2_params_amr.yaml not found: {nav2_params_template}')
    if not os.path.exists(map_file):
        raise RuntimeError(f'[amr_vision_fleet] my_map.yaml not found: {map_file}')

    # Read the nav2 params template once
    with open(nav2_params_template, 'r') as f:
        nav2_params_template_content = f.read()

    bt_file = os.path.join(pkg_share, 'behavior_trees', 'navigate_w_recovery.xml')

    robot_namespaces = [f'agv{i}' for i in range(1, num_agvs + 1)]
    nodes = []

    mode = 'standalone' if launch_gazebo else 'fleet-only'
    nodes.append(LogInfo(msg=f'[amr_vision_fleet] Mode: {mode}, robots: {robot_namespaces}'))

    # -----------------------------------------------------------------------
    # Standalone mode: start Gazebo + all per-robot setup nodes
    # -----------------------------------------------------------------------
    if launch_gazebo:
        xacro_path = os.path.join(pkg_share, 'urdf', 'amr_vision.urdf.xacro')  # CHANGE 2
        world_file = os.path.join(pkg_share, 'worlds', 'map.world')
        gazebo_launch = os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )

        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={'world': world_file}.items(),
        ))

        for idx, ns in enumerate(robot_namespaces):
            init_x, init_y = _spawn_pose(idx)

            nodes.append(Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=ns,
                parameters=[{
                    'robot_description': Command(['xacro ', xacro_path, f' namespace:={ns}']),
                    'use_sim_time': True,
                }],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
                output='screen',
            ))

            nodes.append(Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name=f'spawn_{ns}',
                arguments=[
                    '-entity', ns,
                    '-topic', f'/{ns}/robot_description',
                    '-x', str(init_x),
                    '-y', str(init_y),
                    '-z', '0.1',
                ],
                output='screen',
            ))

            nodes.append(Node(
                package='amr_vision',  # CHANGE 3
                executable='odom_sim_filter',
                name='odom_sim_filter',
                namespace=ns,
                parameters=[{
                    'use_sim_time': True,
                    'robot_namespace': ns,
                }],
                remappings=[('/tf', 'tf')],
                output='screen',
                emulate_tty=True,
            ))

            nodes.append(Node(
                package='amr_vision',  # CHANGE 4
                executable='battery_sim',
                name='battery_sim',
                namespace=ns,
                parameters=[{
                    'use_sim_time': True,
                    'robot_namespace': ns,
                }],
                output='screen',
                emulate_tty=True,
            ))

            nodes.append(Node(
                package='amr_vision',  # CHANGE 5
                executable='charger_dock_monitor',
                name='charger_dock_monitor',
                namespace=ns,
                parameters=[{
                    'use_sim_time': True,
                    'robot_namespace': ns,
                    'odom_topic': f'/{ns}/ground_truth',
                    'contact_topic': f'/{ns}/on_charger',
                    'charger_centers_xy': [-20.0, -10.0, 20.0, -10.0],
                    'enter_radius': 0.8,
                    'exit_radius': 1.1,
                    'min_dock_dwell_sec': 0.5,
                    'max_linear_speed': 0.15,
                    'publish_rate_hz': 5.0,
                }],
                output='screen',
                emulate_tty=True,
            ))

            # CHANGE B: camera_detection_node — one per robot, standalone mode only
            nodes.append(Node(
                package='amr_vision',
                executable='camera_detection_node',
                name=f'camera_detection_node_{ns}',
                parameters=[{
                    'namespace': ns,
                    'model_path': model_path,
                    'conf_threshold': 0.35,
                }],
                output='screen',
                emulate_tty=True,
            ))

    # -----------------------------------------------------------------------
    # Per-robot TF relay — bridges global /tf and /tf_static to /{ns}/tf and
    # /{ns}/tf_static.
    #
    # Why: nav2_bringup with use_namespace:=True remaps all Nav2 nodes so they
    # read TF from /{ns}/tf.  But display_vision.launch.py / spawn_agv.launch.py
    # start odom_sim_filter and robot_state_publisher without TF remapping, so
    # they publish to the global /tf topic.  The relay bridges this mismatch
    # without requiring any changes to the existing launch files.
    # -----------------------------------------------------------------------
    for ns in robot_namespaces:
        nodes.append(Node(
            package='amr_vision',  # CHANGE 6
            executable='tf_relay',
            name='tf_relay',
            namespace=ns,
            parameters=[{
                'use_sim_time': True,
                'robot_namespace': ns,
            }],
            output='screen',
            emulate_tty=True,
        ))

    # -----------------------------------------------------------------------
    # Per-robot Nav2 stack + nav2_goal_bridge — started in both modes.
    #
    # Nav2 handles path planning, dynamic obstacle avoidance, and publishes
    # /<ns>/cmd_vel directly.  nav2_goal_bridge bridges /<ns>/goal_pose (from
    # fleet_manager_ai) to the Nav2 navigate_to_pose action server.
    # -----------------------------------------------------------------------
    for idx, ns in enumerate(robot_namespaces):
        init_x, init_y = _spawn_pose(idx)

        # Substitute {NS} and {BT_FILE} placeholders
        nav2_params_content = nav2_params_template_content.replace('{NS}', ns).replace('{BT_FILE}', bt_file)

        # Write substituted params to a temp file (persists for the process lifetime)
        tmp = tempfile.NamedTemporaryFile(
            mode='w',
            suffix=f'_{ns}_nav2_params.yaml',
            delete=False,
        )
        tmp.write(nav2_params_content)
        tmp.flush()
        tmp.close()
        nav2_params_file = tmp.name

        # Nav2 bringup + goal bridge for this robot.
        # use_composition:=False runs each Nav2 node as a standalone process instead
        # of loading composable nodes into a component_container_isolated.  This avoids
        # the load_node service-discovery race that prevented agv1's container from ever
        # loading its nodes (agv2 always won the DDS race).
        nav2_actions = [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': ns,
                    'use_namespace': 'True',
                    'slam': 'False',
                    'map': map_file,
                    'use_sim_time': 'True',
                    'params_file': nav2_params_file,
                    'autostart': 'True',
                    'use_composition': 'False',
                }.items(),
            ),
            Node(
                package='amr_vision',  # CHANGE 7
                executable='nav2_goal_bridge',
                name='nav2_goal_bridge',
                namespace=ns,
                parameters=[{
                    'use_sim_time': True,
                    'robot_namespace': ns,
                    'initial_x': init_x,
                    'initial_y': init_y,
                }],
                output='screen',
                emulate_tty=True,
            ),
        ]

        if idx == 0:
            nodes.extend(nav2_actions)
        else:
            nodes.append(TimerAction(period=float(idx * 15), actions=nav2_actions))

    # -----------------------------------------------------------------------
    # Fleet Manager AI (single instance)  — CHANGE 8 & 9
    # -----------------------------------------------------------------------
    nodes.append(Node(
        package='amr_vision',          # CHANGE 8
        executable='fleet_manager_ai', # CHANGE 9
        name='fleet_manager_ai',       # CHANGE 9
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': robot_namespaces,
            'tasks_file': tasks_file,
            'battery_topic_type': 'auto',
            'charger_zone_names': ['charger_1', 'charger_2'],
            'battery_low_threshold': 0.20,
            'battery_resume_threshold': 0.60,
            'goal_reach_dist': 0.25,
        }],
        output='screen',
        emulate_tty=True,
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_agvs',
            default_value='1',
            description='Number of AMRs in the fleet.',
        ),
        DeclareLaunchArgument(
            'launch_gazebo',
            default_value='false',
            description=(
                'Set true for standalone mode (starts Gazebo + all robot nodes). '
                'Default false: assumes display_vision.launch.py and spawn_agv.launch.py are already running.'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_poses',
            default_value='0.0,0.0',
            description=(
                'Semicolon-separated "x,y" spawn positions, one per robot. '
                'Used for Nav2 initial pose (AMCL seed) and Gazebo spawn in standalone mode. '
                'Example for 2 robots: "0.0,0.0;2.0,2.0".'
            ),
        ),
        DeclareLaunchArgument(  # CHANGE C
            'model_path',
            default_value='yolov8n.pt',
            description=(
                'Path to YOLO model weights file. '
                'Passed to each camera_detection_node instance. '
                'Default uses the YOLOv8 nano model downloaded on first run.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
