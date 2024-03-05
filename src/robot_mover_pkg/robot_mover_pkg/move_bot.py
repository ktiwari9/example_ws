#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from std_srvs.srv import Empty

from robot_mover_pkg import PACKAGE_NAME

class RobotMoverNode(Node):
    """
    ros_parameters:
            'entity_name': string entity name to move
            'linear_speed': float linear speed of the robot
            'angular_speed': float angular speed in rad/sec of the robot
    """

    def __init__(self):
        super().__init__('robot_move_node')

        # Declare node parameters with values obtained from launch file
        self.declare_parameter('entity_name')
        self.declare_parameter('linear_speed')
        self.declare_parameter('angular_speed')

        # Set parameter values
        self.entity_name = self.get_parameter('entity_name').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value

        # Create publisher to send velocit info to entity on a topic

        self.vel_publisher= self.create_publisher(Twist, '/ttbot/cmd_vel',10)
        
        # Create timer
        time_period = 0.01
        self.counter = 0
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.t0 = time.time() # start time

        # Create reset service client
        self.client = self.create_client(Empty,'reset')
        self.request = Empty.Request()
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self,self.future)
        print('TTbot Reset.....')

    def timer_callback(self):
        msg = Twist()

        # Assign linear and angular velocities to publish
        msg.linear.x = self.linear_speed
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_speed

        # Compute elapsed time
        t1 = time.time()
        time_elapsed = t1-self.t0
        print("Time Elapsed: ", time_elapsed)

        # Compute distance travelled
        dist_travelled = time_elapsed * msg.linear.x
        print("Distance travelled: ", dist_travelled)

def main(args=None):
    rclpy.init(args=args)

    rclpy.logging.set_logger_level('robot_mover', rclpy.logging.LoggingSeverity.DEBUG) 

    # Create Node instance   
    robot_mover_node = RobotMoverNode()
    robot_mover_node.get_logger().info("Initializing Robot Mover Node")

    try:
        robot_mover_node.get_logger().info("Spinning Robot Mover Node")
        rclpy.spin(robot_mover_node)
    finally:
        # Destroy the node
        robot_mover_node.get_logger().info("Shutting down Robot Mover Node")
        robot_mover_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
