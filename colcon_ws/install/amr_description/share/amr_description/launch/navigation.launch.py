from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ----------------------------
    # Launch arguments (CLI-settable)
    # ----------------------------
    namespace = LaunchConfiguration("namespace")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Obstacle/replan behavior
    obstacle_wait_replan_sec = LaunchConfiguration("obstacle_wait_replan_sec")
    clear_costmaps_before_replan = LaunchConfiguration("clear_costmaps_before_replan")
    obstacle_topic = LaunchConfiguration("obstacle_topic")

    # Frames & initial pose
    global_frame = LaunchConfiguration("global_frame")
    base_frame = LaunchConfiguration("base_frame")
    use_initial_pose = LaunchConfiguration("use_initial_pose")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    # Optional: bring up Nav2 here
    bringup_nav2 = LaunchConfiguration("bringup_nav2")
    autostart = LaunchConfiguration("autostart")
    nav2_params = LaunchConfiguration("nav2_params")
    nav2_map = LaunchConfiguration("map")  # Provide when using map-based localization

    decls = [
        DeclareLaunchArgument("namespace", default_value="agv1"),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # Obstacle/replanning
        DeclareLaunchArgument("obstacle_wait_replan_sec", default_value="5.0"),
        DeclareLaunchArgument("clear_costmaps_before_replan", default_value="true"),
        DeclareLaunchArgument("obstacle_topic", default_value="obstacle_detected"),

        # Frames & initial pose
        DeclareLaunchArgument("global_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("use_initial_pose", default_value="false"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),

        # Nav2 bringup toggles
        DeclareLaunchArgument("bringup_nav2", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        # Default params file: point this to your robot-specific Nav2 YAML if you enable bringup
        DeclareLaunchArgument(
            "nav2_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "params", "nav2_params.yaml"
            ])
        ),
        # Optional map file for AMCL (leave empty if using SLAM)
        DeclareLaunchArgument("map", default_value=""),
    ]

    # ---------------------------------
    # Obstacle detection node
    # ---------------------------------
    obstacle_node = Node(
        package="amr_description",
        executable="obstacle_detection_node",
        name="obstacle_detection",
        namespace=namespace,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "robot_namespace": namespace,
        }],
    )

    # ---------------------------------
    # Nav2-based navigator
    # ---------------------------------
    navigator_node = Node(
        package="amr_description",
        executable="nav2navigator",
        name="navigation",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": use_sim_time,
            # Target pose
            "target_x": x,
            "target_y": y,
            "target_yaw": yaw,
            # Namespacing & frames
            "robot_namespace": namespace,
            "global_frame": global_frame,
            "base_frame": base_frame,
            # Initial pose (optional)
            "use_initial_pose": use_initial_pose,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw": initial_yaw,
            # Obstacle + replanning behavior
            "obstacle_wait_replan_sec": obstacle_wait_replan_sec,
            "clear_costmaps_before_replan": clear_costmaps_before_replan,
            "obstacle_topic": obstacle_topic,
        }],
        on_exit=Shutdown(),
    )

    # ---------------------------------
    # Optional: bring up Nav2 in namespace
    # ---------------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"])
        ),
        condition=IfCondition(bringup_nav2),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": nav2_params,
            "map": nav2_map,
            # Add "slam": "true" here if you want SLAM instead of AMCL
        }.items()
    )

    # IMPORTANT: include the nodes in the LaunchDescription
    return LaunchDescription(decls + [
        nav2_bringup,
        obstacle_node,
        navigator_node,
    ])
