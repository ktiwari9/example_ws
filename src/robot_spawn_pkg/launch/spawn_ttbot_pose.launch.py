from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='robot_spawn_pkg',  # Replace with your actual package name
            executable='robot_spawner',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'initial_x': 10.0},
                {'initial_y': 10.0},
                {'initial_z': 0.0},
                {'initial_roll': 0.0},
                {'initial_pitch': 0.0},
                {'initial_yaw': 0.0},
                {'worlds': {'empty': 'empty.world', 'office': 'office.world'}},  # Customize as needed
                {'desired_world': 'empty'},  # Change to the desired world
                {'use_sim_time': use_sim_time},  # Add use_sim_time to parameters
            ],
        ),
    ])
