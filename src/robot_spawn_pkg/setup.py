from setuptools import setup
from setuptools import find_packages
import os

package_name = 'robot_spawn_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch/', [
            'launch/spawn_ttbot.launch.py',
            'launch/spawn_ttbot_pose.launch.py',
            ]),
        ('share/' + package_name + '/urdf/', ['urdf/ttbot.urdf.xacro']),
        ('share/' + package_name + '/worlds/', [
            'worlds/office.world',
            'worlds/empty.world'
            ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='hi@kshitijtiwari.com',
    description='ROS 2 python package to spawn a robot in Gazebo',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_spawner = robot_spawn_pkg.spawn_bot:main',
            'gazebo_launcher = robot_spawn_pkg.gazebo_launcher:main'
        ],
    },
)
