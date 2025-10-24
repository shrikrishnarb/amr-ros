from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="agv1"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("delay_start_time", default_value="0.0"),

        Node(
            package="amr_description",
            executable="ground_truth_waypoint_follower.py",
            name="ground_truth_waypoint_follower",
            namespace=LaunchConfiguration("namespace"),
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "target_x": LaunchConfiguration("x"),
                "target_y": LaunchConfiguration("y"),
                "delay_start_time": LaunchConfiguration("delay_start_time"),
                'robot_namespace': LaunchConfiguration("namespace"),
            }],
            output="screen",
            emulate_tty=True,
            on_exit=Shutdown()
        ),

        Node(
            package="amr_description",
            executable="obstacle_detection_node",
            name="obstacle_detection",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                'robot_namespace': LaunchConfiguration("namespace"),
            }],
        )
    ])
