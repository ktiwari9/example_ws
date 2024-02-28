#!/usr/bin/env python3

import rclpy
import os
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from tf2_ros import TransformStamped
from geometry_msgs.msg import TransformStamped as TransformStampedMsg
from tf2_ros import Buffer, TransformListener

def euler_to_quaternion(roll, pitch, yaw):
    transform = TransformStampedMsg()
    transform.transform.rotation.w = 1.0
    transform.transform.rotation.x = roll
    transform.transform.rotation.y = pitch
    transform.transform.rotation.z = yaw
    return transform.transform.rotation

def spawn_robot(node, x, y, z, roll, pitch, yaw, world_name):
    client = node.create_client(SpawnEntity, '/gazebo/spawn_entity')

    # Directory and file names to spawn robot
    pkg_name = 'robot_spawn_pkg'
    robot_urdf = 'ttbot.urdf.xacro'

    print("urdf_file_name : {}".format(robot_urdf))

    # Set world type
    worlds = {
        'empty': 'empty.world',
        'office': 'office.world',
        # Add more worlds as needed
    }
    desired_world = 'empty'

    # full path for urdf file
    urdf = os.path.join(get_package_share_directory(pkg_name), 'urdf', robot_urdf)

    if not client.service_is_ready():
        node.get_logger().info('Waiting for service...')
        rclpy.spin_once(node, timeout_sec=1)  # Spin once to process any callbacks

    try:
        node.get_logger().info('Trying to spawn the robot')
        request = SpawnEntity.Request()
        node.get_logger().info('Processed Gazebo Service Request to Spawn Entity')
        request.name = 'ttbot'
        request.xml = open(urdf, 'r').read()
        request.robot_namespace = 'ttbot_ns'
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.initial_pose.orientation = euler_to_quaternion(roll, pitch, yaw)

        request.reference_frame = desired_world
        node.get_logger().info('Set request params')

        future = client.call_async(request)
        node.get_logger().info('Calling future client')
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        node.get_logger().info('Spinning ROS Node')

        if future.result() is not None:
            node.get_logger().info('Robot spawned successfully')
        elif future.exception():
            node.get_logger().error(f'Error in service call: {future.exception()}')
        else:
            node.get_logger().error(f'Failed to spawn robot:{future.result()}')
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        import traceback
        traceback.print_exc()

def main(args=None):
    rclpy.init(args=args)
    rclpy.logging.set_logger_level('spawn_bot', rclpy.logging.LoggingSeverity.DEBUG)

    node = rclpy.create_node('spawn_bot')

    # Set your desired initial pose here
    initial_x = 10.0
    initial_y = 10.0
    initial_z = 0.0
    initial_roll = 0.0
    initial_pitch = 0.0
    initial_yaw = 0.0

    # Set the desired world
    pkg_name = 'robot_spawn_pkg'
    worlds = {
        'empty': 'empty.world',
        'office': 'office.world',
        # Add more worlds as needed
    }
    desired_world = "empty"  # spawn robot in empty world
    # desired_world = worlds['office']  # spawn robot in office world
    world = os.path.join(get_package_share_directory(pkg_name), 'worlds', desired_world)

    try:
        spawn_robot(node, initial_x, initial_y, initial_z, initial_roll, initial_pitch, initial_yaw, world)
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
