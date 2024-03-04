#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import subprocess
import os
import time
import math
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, SetEntityState, GetEntityState
from geometry_msgs.msg import Pose
from robot_spawn_pkg import PACKAGE_NAME
from tf_transformations import quaternion_from_euler


class GazeboLauncherNode(Node):
    """
    ros_parameters:
            'entity_name': string entity name to spawn
            'world_name' : string world name [options: office, empty]
            'x': float x-position
            'y': float y-position
            'z': float z-position
            'r': float roll in radians
            'p': float pitch
            'y': float yaw
    """

    def __init__(self):
        super().__init__('robot_spawn_node')



        # Declare node parameters with values from launch file
        self.declare_parameter('entity_name')
        self.declare_parameter('world_name')
        self.declare_parameter('x')
        self.declare_parameter('y')
        self.declare_parameter('z')
        self.declare_parameter('R')
        self.declare_parameter('P')
        self.declare_parameter('Y')

        # Assign parameter values to class variables
        self.entity_name= self.get_parameter('entity_name').value
        self.world_name= self.get_parameter('world_name').value
        self.x= self.get_parameter('x').value
        self.y= self.get_parameter('y').value
        self.z= self.get_parameter('z').value
        self.R = self.get_parameter('R').value
        self.P = self.get_parameter('P').value
        self.Y = self.get_parameter('Y').value
        

        # Set the world
        world = os.path.abspath(os.path.join(get_package_share_directory(PACKAGE_NAME), 'worlds', self.world_name))
        self.get_logger().info(f"Setting World File Type: {self.world_name}")

        # self.get_logger().info(f'World path: {world}')
        
        # Create a subprocess to launch Gazebo server
        cmd = ['gazebo', '--verbose', 
               '-s','libgazebo_ros_init.so',
               '-s', 'libgazebo_ros_factory.so',
               world]
        # self.get_logger().info(f"Subprocess command: {' '.join(cmd)}")
        self.gazebo_process = subprocess.Popen(cmd)
        self.get_logger().info("Done opening world file~~")
        
        # Wait for Gazebo to start
        time.sleep(5)  # Adjust the time delay as needed

        # Spawn the robot inside the world at preset initial pose
        initial_pose = Pose()
        initial_pose.position.x = self.x
        initial_pose.position.y = self.y
        initial_pose.position.z = self.z
        (initial_pose.orientation.x, 
        initial_pose.orientation.y,
        initial_pose.orientation.z,
        initial_pose.orientation.w) = quaternion_from_euler(self.R,self.P,self.Y)

        # Call Function to Spawn Robot
        self.spawn_robot('ttbot_ns', initial_pose)

    def quaternion_to_euler(self, quaternion):
        """
        Utility function to convert quaternion to euler anlges in radians
        """
        x = quaternion.x    #[1]
        y = quaternion.y    #[2]
        z = quaternion.Z    #[3]
        w = quaternion.w    #[0]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    def euler_to_quaternion(self, roll,pitch,yaw):
        """
        Helper function to convert euler angles (radians)
        to quaternion x,y,z,w
        """

    def spawn_robot(self, namespace, initial_pose):
        """
        class method to spawn a robot at a desired initial pose
        """
        try:
            robot_urdf = self.entity_name + '.urdf.xacro'
            urdf = os.path.abspath(os.path.join(get_package_share_directory(PACKAGE_NAME), 'urdf', robot_urdf))

            # Create a client for the SpawnEntity service
            client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
            self.get_logger().info("Calling Spawn Entity Service")

            # Wait for the service to be ready
            if not client.wait_for_service(timeout_sec=10.0):
                self.get_logger().info('Waiting for service to Spawn Robot...')

            # Check if the service is ready
            if client.service_is_ready():
                request = SpawnEntity.Request()
                request.name = self.entity_name
                request.xml = open(urdf, 'r').read()
                request.robot_namespace = namespace
                request.initial_pose = initial_pose

                # Call the SpawnEntity service
                response = client.call(request)

                if response.success:
                    self.get_logger().info('Robot spawned successfully!')
                else:
                    raise RuntimeError('Failed to spawn robot entity')
            else:
                raise RuntimeError('Spawn Entity service not ready.')

        except Exception as e:
            self.get_logger().error(f'Error: {e}')



    def destroy(self):
        # Kill the Gazebo subprocess
        self.get_logger().info("Destroying GazeboLauncherNode")
        self.gazebo_process.kill()
        self.get_logger().info("Gazebo subprocess killed")


class DeleteEntityNode(Node):
    """
    Class to remove entities from a Gazebo Scene.

    Task 1: Remove only the ball from the scene
    Task 2: Complete Task 1, remove the TTBot and respawn it at some other location
    """
    def __init__(self, entities_to_delete=None):
        super().__init__('Remove_Entity')
        self.cli = self.create_client(srv_type=DeleteEntity, srv_name="/gazebo/delete_entity")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.entities_to_delete = entities_to_delete or []

    async def get_entities_list(self):
        # Implement the logic to get the list of entities from Gazebo
        # This may involve using Gazebo services or other methods
        # Return a list of entities
        try:
            # Create a client for the GetEntityState service
            client = self.create_client(GetEntityState, '/gazebo/get_entity_state')

            # Wait for the service to be ready
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError('Timeout waiting for service to get entity states...')

            # Create a request object
            req = GetEntityState.Request()

            # Send the request to get all entity states
            future = client.call_async(req)
            response = await future

            # Extract the list of entities from the response
            entities = [state.entity_name for state in response.status]

            return entities

        except Exception as e:
            self.get_logger().error(f'Error getting entity list: {e}')
            return []


    def delete_entities(self, entities=None):
        entities = entities or self.entities_to_delete
        entity_to_find = ["ball"] # Add more entities here as needed
        # find case insensitive match for all entities to delete
        entity_to_del = [entity for target_entity in entity_to_find for entity in entities if entity.lower() == target_entity.lower()]
        for entity in entity_to_del:
            # Iteratively delete all requested entities
            req = DeleteEntity.Request()
            req.name = entity
            self.future = self.cli.call_async(req)
            self.get_logger().info(f'Deleting entity {entity} from scene')
            rclpy.spin_until_future_complete(self, self.future)

        self.destroy_node()


class ResetModelPose(Node):
    """
    Query the pose of a model and reset it to a new pose
    Currently unused.
    """
    def __init__(self):
        super().__init__('set_model_pose')
        # Call the set_parameters service of the robot state publisher to update the pose of the robot
        self.cli = self.create_client(
            srv_type=SetEntityState, srv_name="/set_entity_state")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service %s...')
        # Set the initial pose of the robot

    def send_request(self, model_name, pose):
        req = SetEntityState.Request()
        req.state._name = model_name
        req.state._pose = pose
        self.future = self.cli.call_async(req)
        
        rclpy.spin_until_future_complete(self, self.future)


def main(args=None):
    rclpy.init(args=args)

    rclpy.logging.set_logger_level('gazebo_launcher', rclpy.logging.LoggingSeverity.DEBUG)

    # Create a GazeboLauncherNode with the desired world
    gazebo_launcher_node = GazeboLauncherNode()
    gazebo_launcher_node.get_logger().info("Initializing Gazebo Launcher Node")

    try:
        # Spin the node
        gazebo_launcher_node.get_logger().info("Spinning Gazebo Launcher Node")
        rclpy.spin(gazebo_launcher_node)
        gazebo_launcher_node.get_logger().info("Done Spinning")
    finally:
        # Destroy the node
        gazebo_launcher_node.get_logger().info("Done! Shutting down node.")
        gazebo_launcher_node.destroy()
        rclpy.shutdown()

    ##################################################################################

    # Create Entity Deletion instance from the currently open world
    entity_deletion_node = DeleteEntityNode()
    entity_deletion_node.get_logger().info("Initializing Entity Deletion Node")
    try:
        entity_deletion_node.get_logger().info("Spinning Entity Deletion Node")
        rclpy.spin_once(entity_deletion_node)

    finally:
        # Destroy the node
        entity_deletion_node.get_logger().info("Done deletion! Shutting down node.")
        entity_deletion_node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
