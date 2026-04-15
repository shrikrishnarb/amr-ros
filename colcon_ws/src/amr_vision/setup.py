from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'amr_vision'


def get_data_files(directory):
    paths = []
    for root, _, files in os.walk(directory):
        for f in files:
            paths.append(os.path.join(root, f))
    return paths


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'yaml'), get_data_files('yaml')),
        (os.path.join('share', package_name, 'behavior_trees'), glob('behavior_trees/*')),
        (os.path.join('share', package_name, 'worlds'), get_data_files('worlds')),
        (os.path.join('share', package_name, 'maps'), get_data_files('maps')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shrikrishnarb',
    maintainer_email='shrikrishnarb@gmail.com',
    description='AI-enabled AMR fleet with YOLO semantic perception',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_detection_node = amr_vision.camera_detection_node:main',
            'fleet_manager_ai = amr_vision.fleet_manager_ai:main',
            'nav2_goal_bridge = amr_vision.nav2_goal_bridge:main',
            'tf_relay = amr_vision.tf_relay:main',
            'odom_sim_filter = amr_vision.odom_sim_filter:main',
            'obstacle_detection_node = amr_vision.obstacle_detection_node:main',
            'battery_sim = amr_vision.battery_sim:main',
            'charger_dock_monitor = amr_vision.charger_dock_monitor:main',
        ],
    },
)
