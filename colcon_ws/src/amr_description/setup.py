from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'amr_description'

def get_data_files(directory):
    # Recursively get all files (exclude directories)
    paths = []
    for root, _, files in os.walk(directory):
        for f in files:
            paths.append(os.path.join(root, f))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'worlds'), get_data_files('worlds')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='shrikrishnarb@gmail.com',
    description='AMR Description Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_sim_filter = amr_description.odom_sim_filter:main',
            'ground_truth_waypoint_follower.py = amr_description.ground_truth_waypoint_follower:main',
            'obstacle_detection_node = amr_description.obstacle_detection_node:main',
            'battery_sim = amr_description.battery_sim:main',
            'charger_dock_monitor = amr_description.charger_dock_monitor:main',
            'nav2navigator = amr_description.navigation:main',
        ],
    },
)
