from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, TextSubstitution
from launch_ros.substitutions import FindPackageShare

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

    gazebo_launch = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
    ])

    use_sim_time = {'use_sim_time': True}

    return LaunchDescription([
        # Start Gazebo
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

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[use_sim_time]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[use_sim_time]
        )
    ])
