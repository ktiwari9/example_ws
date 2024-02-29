import rclpy
from rclpy.node import Node
import subprocess
import os, shutil
from ament_index_python.packages import get_package_share_directory

class GazeboLauncherNode(Node):

    def __init__(self):
        super().__init__('gazebo_launcher')

        # Set the world
        self.get_logger().info("Setting World File Location")
        pkg_name = 'robot_spawn_pkg'
        worlds = {
            'empty': 'empty.world',
            'office': 'office.world',
            # Add more worlds as needed
        }
        desired_world = worlds['office']  # spawn robot in office world
        # world = os.path.join(get_package_share_directory(pkg_name), 'worlds', desired_world)
        world = os.path.abspath(os.path.join(get_package_share_directory(pkg_name), 'worlds', desired_world))

        self.get_logger().info(f'World path: {world}')
        
        # Create a subprocess to launch Gazebo server
        # self.get_logger().info("Moving world file to Default Gazebo Path")
        # self.copy_world_file_to_default_location(pkg_name, desired_world)
        # self.gazebo_process = subprocess.Popen(['gzserver', '--verbose', '4', '-s', 'libgazebo_ros_factory.so', world, '--port', '6080'])
        cmd = ['gazebo', '--verbose', world]
        self.get_logger().info(f"Subprocess command: {' '.join(cmd)}")
        self.gazebo_process = subprocess.Popen(cmd)
        self.get_logger().info("Done opening world file~~")

    def copy_world_file_to_default_location(self, pkg_name, desired_world):
        # Get the full path to your world file
        self.world_source = os.path.join(get_package_share_directory(pkg_name), 'worlds', desired_world)
        
        # Set the destination directory
        gazebo_worlds_dir = '/usr/share/gazebo-11/worlds'
        
        try:
            # Copy the world file to the default Gazebo worlds directory
            self.get_logger().info("Copying World file~~")
            shutil.copy(self.world_source, gazebo_worlds_dir)
            self.get_logger().info(f"World file copied successfully to {gazebo_worlds_dir}")
        except Exception as e:
            self.get_logger().info(f"Error copying world file: {e}")

    def destroy(self):
        # Kill the Gazebo subprocess
        self.get_logger().info("Destroying GazeboLauncherNode")
        self.gazebo_process.kill()
        self.get_logger().info("Gazebo subprocess killed")


def main(args=None):
    rclpy.init(args=args)

    rclpy.logging.set_logger_level('gazebo_launcher', rclpy.logging.LoggingSeverity.DEBUG)

    # Create a GazeboLauncherNode
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

if __name__ == '__main__':
    main()