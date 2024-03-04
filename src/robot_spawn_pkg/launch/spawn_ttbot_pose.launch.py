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
    # Set Gazebo Path Variables

    install_dir = get_package_prefix(PACKAGE_NAME)
    gazebo_models_path = os.path.join(PACKAGE_NAME,"urdf")

    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] = install_dir + '/share' + ':' + gazebo_models_path
    # print("Gazebo Models Path=="+str(os.environ['GAZEBO_MODEL_PATH']))


    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':' + gazebo_models_path

    # print("Gazebo Plugin Path=="+str(os.environ['GAZEBO_PLUGIN_PATH']))


    # Declare a launch argument for the world file
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_types = {
    1: 'empty.world',
    2: 'office.world',
    # Add more worlds as needed
    }
    default_world_idx = 2 # default index for which world to spawn. Change as needed

    world_type = world_types.get(default_world_idx, 'empty.world') # Spawn Empty in case of invalid index

    print("Default World Type for Gazebo =="+ world_type)

    world_name = DeclareLaunchArgument(
        'world_name',
        default_value=world_type,
        description='Name of default Gazebo world file to spawn')
    
    entity_name =DeclareLaunchArgument(
        'entity_name',
        default_value='ttbot',
        description='Name of robot to spawn')
    
    # Prepare robot description from Xacro file
    xacro_file = os.path.join(get_package_share_directory(PACKAGE_NAME), 'urdf/', 'ttbot.urdf.xacro')
    assert os.path.exists(xacro_file), "The xacro file doesnt exist in " + str(xacro_file)
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Create a Node action to launch the GazeboLauncherNode
    gazebo_launcher_node = Node(
        package='robot_spawn_pkg',
        executable='robot_spawner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'entity_name': LaunchConfiguration('entity_name')},
            {'world_name': LaunchConfiguration('world_name')},
            {'x': 0.0}, # 3D position
            {'y': 0.0},
            {'z': 0.0},
            {'R': 0.0}, # roll, pitch, yaw in Radians
            {'P': 0.0},
            {'Y': math.pi/2}
        ],
        arguments=["-topic", "/robot_description"],
        remappings=[
            ('/gazebo/spawn_entity','/spawn_entity'),
            ('/gazebo/delete_entity','delete_entity')
        ]
        )
    
    # Publish Robot's URDF on ROS topic for displaying robot on screen
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen")
    
    # Open Rviz
    # rviz_config_dir = os.path.join(get_package_share_directory(PACKAGE_NAME))
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time':use_sim_time}],
        # arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription([
        world_name,
        entity_name,
        gazebo_launcher_node,
        robot_state_publisher,
        rviz_node
    ])