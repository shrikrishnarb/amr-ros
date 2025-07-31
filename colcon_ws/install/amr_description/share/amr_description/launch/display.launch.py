from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from launch.substitutions import Command, TextSubstitution

def generate_launch_description():
    pkg_amr = FindPackageShare('amr_description')

    urdf_path = PathJoinSubstitution([
        pkg_amr, 'urdf', 'amr.urdf'
    ])

    rviz_config_path = PathJoinSubstitution([
        pkg_amr, 'rviz', 'amr_config.rviz'
    ])

    robot_desc = Command([
        TextSubstitution(text='cat '),
        urdf_path
    ])

    return LaunchDescription([
        # Start Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('gazebo_ros'),
                    'launch',
                    'gazebo.launch.py'
                ])
            ])
        ),

        Node(
            package='amr_description',
            executable='odom_sim_filter',
            name='odom_sim_filter',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[
                {'use_sim_time': True}
            ],
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True},
                {'frame_prefix': 'agv1/'}
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'amr_bot',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ]
        )
    ])