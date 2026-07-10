"""
Launch the butler robot in the restaurant world (Gazebo Harmonic).

Thin wrapper around amr_butler_description/launch/butler_sim.launch.py that
points its `world` argument at this package's worlds/restaurant.sdf and spawns
the robot next to the charging_dock model in the south-east corner (see the
named-locations comment block at the top of restaurant.sdf).

Run inside the amr-butler-dev container (see docker/README.butler.md):

    ros2 launch amr_worlds restaurant.launch.py \
        namespace:=butler1 gui:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Declare pass-through arguments and include butler_sim.launch.py."""
    worlds_share = get_package_share_directory('amr_worlds')
    butler_share = get_package_share_directory('amr_butler_description')
    world = os.path.join(worlds_share, 'worlds', 'restaurant.sdf')

    butler_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(butler_share, 'launch', 'butler_sim.launch.py')
        ),
        launch_arguments={
            'world': world,
            'namespace': LaunchConfiguration('namespace'),
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
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
            description='Start RViz with the butler config'
        ),
        # spawn next to the charging_dock at (7.0, -4.3), facing west into the room
        DeclareLaunchArgument('x', default_value='6.3', description='Spawn x [m]'),
        DeclareLaunchArgument('y', default_value='-4.0', description='Spawn y [m]'),
        DeclareLaunchArgument('yaw', default_value='3.1416', description='Spawn yaw [rad]'),
        butler_sim,
    ])
