from setuptools import setup
import os
from glob import glob

package_name = 'amr_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/talker_listener.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Minimal talker node for AMR bringup',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = amr_bringup.talker:main',
            'listener = amr_bringup.listener:main',
        ],
    },
)

