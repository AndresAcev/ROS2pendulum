from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control',
            executable='state_space_controller.py',
            name='state_space_controller',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )
    ]) 