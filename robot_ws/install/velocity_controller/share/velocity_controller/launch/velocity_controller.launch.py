from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_controller',
            executable='velocity_controller',
            name='velocity_controller',
            output='screen',
        )
    ])