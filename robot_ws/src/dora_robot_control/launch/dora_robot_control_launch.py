from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dora_robot_control',
            executable='odometry_node',
            name='odometry_node',
            output='screen',
        ),
        Node(
            package='dora_robot_control',
            executable='serial_node',
            name='serial_node',
            # output='screen',
        ),
        Node(
            package='dora_robot_control',
            executable='wheel_vel_node',
            name='wheel_vel_node',
            # output='screen',
        ),
    ])