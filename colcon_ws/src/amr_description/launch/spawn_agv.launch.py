from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command

def generate_launch_description():
    # Declare input arguments
    namespace_arg = DeclareLaunchArgument('namespace', default_value='agv1')
    x_arg = DeclareLaunchArgument('x', default_value='2.0')
    y_arg = DeclareLaunchArgument('y', default_value='2.0')

    namespace = LaunchConfiguration('namespace')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')

    pkg_amr = FindPackageShare('amr_description')
    urdf_path = PathJoinSubstitution([pkg_amr, 'urdf', 'amr.urdf.xacro'])

    robot_desc = Command(['xacro ', urdf_path, ' namespace:=', namespace])

    spawn_robot = GroupAction([
        PushRosNamespace(namespace),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': True}
            ]
        ),

        # odom_sim_filter with dynamic frame names
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

        # Spawn robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', namespace,
                '-topic', 'robot_description',
                '-x', x,
                '-y', y,
                '-z', '0.5'
            ]
        ),
    ])

    return LaunchDescription([
        namespace_arg,
        x_arg,
        y_arg,
        spawn_robot
    ])
