from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, TextSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def load_urdf():
    urdf_path = os.path.join(
        os.path.dirname(__file__), '..', 'urdf', 'amr.urdf'
    )
    with open(urdf_path, 'r') as infp:
        return infp.read()

def generate_launch_description():
    map_yaml = PathJoinSubstitution([
        FindPackageShare('amr_description'),
        'maps',
        'my_map.yaml'
    ])
    pkg_amr = FindPackageShare('amr_description')

    urdf_path = PathJoinSubstitution([
        pkg_amr, 'urdf', 'amr.urdf'
    ])

    rviz_config = PathJoinSubstitution([
        pkg_amr, 'rviz', 'amr_config.rviz'
    ])

    robot_desc = Command([
        TextSubstitution(text='cat '),
        urdf_path
    ])

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        ),

        # Start Gazebo (make sure world uses sim time)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gazebo_launch])
        ),

        # Robot description publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                use_sim_time
            ]
        ),

        # Joint state publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[use_sim_time]
        ),

        # Spawn robot in Gazebo
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
                '-z', '0.3'
            ],
            parameters=[use_sim_time]
        ),

        # Delay RViz to allow /clock + map to initialize
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    parameters=[use_sim_time]
                )
            ]
        )
    ])
