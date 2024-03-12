#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from robot_spawn_pkg import PACKAGE_NAME

def get_xacro_to_doc(xacro_file_path, mappings):
	doc = xacro.parse(open(xacro_file_path))
	xacro.process_doc(doc, mappings=mappings)
	return doc

def generate_launch_description():

	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	xacro_path = os.path.join(get_package_share_directory(PACKAGE_NAME), 'urdf/', 'ttbot.urdf.xacro')
	assert os.path.exists(xacro_path), "The xacro file doesnt exist in " + str(xacro_path)


	rviz = Node(
		package='rviz2',
		executable='rviz2',
		name='rviz2',
		output='screen',
		arguments=['-d', os.path.join(get_package_share_directory(PACKAGE_NAME), 'rviz', 'office_world.rviz')]
	)

	return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
     	rviz
    ])