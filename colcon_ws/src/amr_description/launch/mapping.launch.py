# mapping.launch.py
#
# All-in-one launch for mapping the environment with SLAM Toolbox.
# Starts: Gazebo + robot spawn + robot_state_publisher + odom_sim_filter
#         + SLAM Toolbox + RViz + teleop keyboard (in a separate xterm window).
#
# Usage:
#   ros2 launch amr_description mapping.launch.py
#
# When done mapping, save the map:
#   ros2 run nav2_map_server map_saver_cli -f /workspace/colcon_ws/src/amr_description/maps/my_map
# Then rebuild so the map is installed to the share directory:
#   cd /workspace/colcon_ws && colcon build --symlink-install --packages-select amr_description

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

NS = 'agv1'

TELEOP_CMD = (
    'source /opt/ros/humble/setup.bash && '
    'source /workspace/colcon_ws/install/setup.bash && '
    f'ros2 run teleop_twist_keyboard teleop_twist_keyboard '
    f'--ros-args --remap cmd_vel:=/{NS}/cmd_vel'
)


def generate_launch_description():
    pkg_amr = FindPackageShare('amr_description')

    xacro_file = PathJoinSubstitution([pkg_amr, 'urdf', 'amr.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file, ' namespace:=', NS])

    world = os.path.join(
        get_package_share_directory('amr_description'),
        'worlds', 'map.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    return LaunchDescription([
        gazebo,

        # Publishes robot_description on /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True},
            ],
        ),

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr',
            output='screen',
            arguments=[
                '-entity', NS,
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.5',
            ],
        ),

        # Publishes TF: agv1/odom → agv1/base_footprint (required by SLAM Toolbox)
        Node(
            package='amr_description',
            executable='odom_sim_filter',
            name='odom_sim_filter',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_namespace': NS,
                'odom_frame': f'{NS}/odom',
                'base_frame': f'{NS}/base_footprint',
            }],
        ),

        # SLAM Toolbox — subscribes to /agv1/scan, publishes /map + TF map→agv1/odom
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'odom_frame': f'{NS}/odom',
                'map_frame': 'map',
                'base_frame': f'{NS}/base_footprint',
                'scan_topic': f'/{NS}/scan',
                'mode': 'mapping',
            }],
        ),

        # RViz — open without a config so you can add Map/LaserScan/TF displays manually,
        # or add an rviz config file with map + laser + robot model displays configured.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # Teleop keyboard in a separate xterm window (delayed 5s to let Gazebo start)
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['xterm', '-title', 'Teleop', '-e', 'bash', '-c', TELEOP_CMD],
                    output='screen',
                ),
            ],
        ),
    ])
