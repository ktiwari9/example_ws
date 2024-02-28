from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_spawn_pkg',  # Replace with your actual package name
            executable='robot_spawner',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'initial_x': 0.0},
                {'initial_y': 0.0},
                {'initial_z': 0.0},
                {'initial_roll': 0.0},
                {'initial_pitch': 0.0},
                {'initial_yaw': 0.0},
                {'worlds': {'empty': 'empty.world', 'office': 'office.world'}},  # Customize as needed
                {'desired_world': 'office'},  # Change to the desired world
            ],
        ),
    ])
