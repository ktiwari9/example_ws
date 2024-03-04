from setuptools import setup
from robot_spawn_pkg import PACKAGE_NAME



setup(
    name=PACKAGE_NAME,
    version='0.0.0',
    packages=[PACKAGE_NAME],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        ('share/' + PACKAGE_NAME + '/launch/', [
            'launch/spawn_ttbot.launch.py',
            'launch/spawn_ttbot_pose.launch.py',
            ]),
        ('share/' + PACKAGE_NAME + '/urdf/', [
            'urdf/ttbot.urdf.xacro',
            'urdf/ttbot.gazebo',
            'urdf/ttbot.urdf']),
        ('share/' + PACKAGE_NAME + '/worlds/', [
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
        ],
    },
)
