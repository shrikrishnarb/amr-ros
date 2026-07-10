"""
Launch the butler robot in Gazebo Harmonic (gz-sim 8).

Starts gz-sim with worlds/butler_test.sdf, processes the robot xacro, spawns it
via ros_gz_sim `create`, and starts robot_state_publisher, ros_gz_bridge (from
the namespace-substituted config/bridge.yaml template) and optionally RViz.

Run inside the amr-butler-dev container (see docker/README.butler.md):

    ros2 launch amr_butler_description butler_sim.launch.py \
        namespace:=butler1 gui:=true rviz:=true

For a second robot in the same world, reuse this file with launch_gazebo:=false
and bridge_clock:=false (clock must be bridged exactly once per simulation).
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _make_bridge_config(pkg_share, ns, bridge_clock):
    """
    Substitute {NS} in the bridge.yaml template; write it to a temp file.

    Mirrors the {NS} substitution pattern used for nav2_params_amr.yaml in
    amr_description/launch/amr_fleet_management.launch.py. When bridge_clock is
    false the /clock entry is dropped so extra robots do not double-bridge sim
    time.
    """
    template_path = os.path.join(pkg_share, 'config', 'bridge.yaml')
    with open(template_path, 'r') as f:
        content = f.read().replace('{NS}', ns)

    entries = yaml.safe_load(content)
    if not bridge_clock:
        entries = [e for e in entries if e['ros_topic_name'] != '/clock']

    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix=f'_{ns}_bridge.yaml',
        delete=False,
    )
    yaml.safe_dump(entries, tmp)
    tmp.flush()
    tmp.close()
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    """Build the launch actions once launch configurations are resolved."""
    ns = LaunchConfiguration('namespace').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1')
    use_rviz = LaunchConfiguration('rviz').perform(context)
    launch_gazebo = LaunchConfiguration('launch_gazebo').perform(context)
    bridge_clock = (
        LaunchConfiguration('bridge_clock').perform(context).lower() in ('true', '1')
    )
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    pkg_share = get_package_share_directory('amr_butler_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'amr_butler.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'butler.rviz')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', ns]),
        value_type=str,
    )

    # -r: start unpaused. Headless server (-s) when gui:=false; sensors still
    # render via --headless-rendering (software ogre2, see CLAUDE.md).
    gz_args = f'-r {world}' if gui else f'-s -r --headless-rendering {world}'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=IfCondition(launch_gazebo),
    )

    # robot_state_publisher: namespaced (reads /<ns>/joint_states, publishes
    # /<ns>/robot_description) but its TF goes to the GLOBAL /tf, matching the
    # repo convention (tf_relay fans out to /<ns>/tf when Nav2 is added later).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=ns,
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True},
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
    )

    # Spawn from the namespaced robot_description topic. Model name = namespace,
    # so gz entity names stay unique when more robots are added.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ns}',
        output='screen',
        arguments=[
            '-name', ns,
            '-topic', f'/{ns}/robot_description',
            '-x', x,
            '-y', y,
            '-z', '0.05',
            '-Y', yaw,
        ],
    )

    bridge_config = _make_bridge_config(pkg_share, ns, bridge_clock)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'bridge_{ns}',
        output='screen',
        parameters=[
            {'config_file': bridge_config},
            {'use_sim_time': True},
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(use_rviz),
    )

    return [gazebo, robot_state_publisher, spawn_robot, bridge, rviz]


def generate_launch_description():
    """Declare launch arguments and defer the rest to _launch_setup."""
    default_world = os.path.join(
        get_package_share_directory('amr_butler_description'),
        'worlds', 'butler_test.sdf'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='butler1',
            description='Robot namespace (frame prefix, topic prefix, gz model name)'
        ),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Start the Gazebo GUI (false = headless server only)'
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the saved butler config'
        ),
        DeclareLaunchArgument(
            'launch_gazebo', default_value='true',
            description='Start gz-sim (false = attach to an already-running sim)'
        ),
        DeclareLaunchArgument(
            'bridge_clock', default_value='true',
            description='Bridge /clock gz->ros. Must be true for exactly ONE robot'
        ),
        DeclareLaunchArgument(
            'world', default_value=default_world,
            description='Absolute path of the SDF world file'
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn yaw [rad]'),
        OpaqueFunction(function=_launch_setup),
    ])
