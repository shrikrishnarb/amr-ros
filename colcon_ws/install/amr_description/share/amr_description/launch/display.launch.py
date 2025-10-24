from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare namespace argument
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='agv1',
        description='Robot namespace'
    )

    namespace = LaunchConfiguration('namespace')

    pkg_amr = FindPackageShare('amr_description')

    xacro_file = PathJoinSubstitution([
        pkg_amr, 'urdf', 'amr.urdf.xacro'
    ])

    rviz_config_path = PathJoinSubstitution([
        pkg_amr, 'rviz', 'amr_config.rviz'
    ])

    # Generate robot_description from Xacro and pass namespace
    robot_description = Command([
        'xacro ', xacro_file, ' namespace:=', namespace
    ])

    world = os.path.join(
        get_package_share_directory('amr_description'),
        'worlds', 'map.world'
    )

    declare_world_fname = DeclareLaunchArgument(
        "world_fname", default_value=world, description="absolute path of gazebo world file"
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world_fname"),
        }.items()
    )

    return LaunchDescription([
        namespace_arg,
        declare_world_fname,
        gazebo,

        # Simulated Odom Filter Node
        Node(
            package='amr_description',
            executable='odom_sim_filter',
            name='odom_sim_filter',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True},
                {'robot_namespace': namespace}
            ],
        ),

        # Battery simulator
        Node(
            package='amr_description',
            executable='battery_sim',
            name='battery_sim',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_namespace': namespace}
            ],
        ),

        
        # Charger proximity monitor
        Node(
            package='amr_description',
            executable='charger_dock_monitor',
            name='charger_dock_monitor',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_namespace': namespace}
            ],
        ),


        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'use_sim_time': True}
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

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_amr',
            output='screen',
            arguments=[
                '-entity', namespace,
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.5'
            ]
        )
    ])
