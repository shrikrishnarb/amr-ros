"""
Map the restaurant world with slam_toolbox (online async).

Starts the restaurant sim (launch/restaurant.launch.py with rviz:=false),
slam_toolbox configured for the namespaced frames from the
config/slam_params.yaml template ({NS} substituted at launch, same pattern as
the butler bridge.yaml), and RViz with a mapping-oriented config.

Drive the robot from a SECOND terminal (see the amr_worlds README):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=/butler1/cmd_vel

slam_toolbox runs un-namespaced on purpose: it reads the GLOBAL /tf (where
robot_state_publisher and the gz bridge publish, per repo convention) and gets
the robot-specific bits via the substituted frame names and scan topic.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _make_slam_config(pkg_share, ns):
    """Substitute {NS} in the slam_params.yaml template; write a temp file."""
    template_path = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    with open(template_path, 'r') as f:
        content = f.read().replace('{NS}', ns)

    tmp = tempfile.NamedTemporaryFile(
        mode='w',
        suffix=f'_{ns}_slam.yaml',
        delete=False,
    )
    tmp.write(content)
    tmp.flush()
    tmp.close()
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    """Build the launch actions once launch configurations are resolved."""
    ns = LaunchConfiguration('namespace').perform(context)
    use_rviz = LaunchConfiguration('rviz').perform(context)

    pkg_share = get_package_share_directory('amr_worlds')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')

    restaurant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'restaurant.launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            'gui': LaunchConfiguration('gui'),
            'rviz': 'false',  # mapping uses this package's RViz config instead
        }.items(),
    )

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            _make_slam_config(pkg_share, ns),
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

    return [restaurant, slam, rviz]


def generate_launch_description():
    """Declare launch arguments and defer the rest to _launch_setup."""
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
            description='Start RViz with the mapping config (Map, LaserScan, TF)'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
