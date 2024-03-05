from setuptools import setup
import glob,os
from robot_mover_pkg import PACKAGE_NAME


setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch/', [
            'launch/robot_mover.launch.py',
            ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='hi@kshitijtiwari.com',
    description='ROS 2 Python package to move a 4 wheeled robot in Gazebo',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_mover = robot_mover_pkg.move_bot:main'
        ],
    },
)
