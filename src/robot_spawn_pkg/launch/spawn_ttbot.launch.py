import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# this is the function launch  system will look for

def generate_launch_description():

    pkg_name = 'robot_spawn_pkg'
    robot_urdf = 'ttbot.urdf.xacro'
    world_file_name = 'office.world'
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    print("urdf_file_name : {}".format(robot_urdf))

    # full  path to urdf and world file
    
    world = os.path.join(get_package_share_directory(pkg_name), 'worlds', world_file_name)

    urdf = os.path.join(get_package_share_directory(pkg_name), 'urdf', robot_urdf)
    
    # read urdf contents because to spawn an entity in 
    # gazebo we need to provide entire urdf as string on  command line

    xml = open(urdf, 'r').read()

    # double quotes need to be with escape sequence
    xml = xml.replace('"', '\\"')

    # this is argument format for spwan_entity service 
    spwan_args = '{name: \"ttbot\", xml: \"'  +  xml + '\" }'

    # create and return launch description object
    return LaunchDescription([

        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # tell gazebo to spwan your robot in the world by calling service
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spwan_args],
            output='screen'),
    ])
