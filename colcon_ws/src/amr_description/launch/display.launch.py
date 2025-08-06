from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

    return LaunchDescription([
        namespace_arg,

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
