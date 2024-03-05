#!/usr/bin/python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os, xacro, math
from robot_spawn_pkg import PACKAGE_NAME
from ament_index_python.packages import get_package_prefix,get_package_share_directory


def generate_launch_description():
    entity_name =DeclareLaunchArgument(
        'entity_name',
        default_value='ttbot',
        description='Name of entity to move')
    
    
    #Create a Node action to launch the RobotMoverNode
    robot_mover_node = Node(
        package=PACKAGE_NAME,
        executable='robot_mover',
        output='screen'
        ,
        parameters=[
            {'entity_name': LaunchConfiguration('entity_name')},
            {'linear_speed': 0.0}, 
            {'angular_speed': 0.e}
        ]
    )

    return LaunchDescription([
        entity_name,
        robot_mover_node
    ])