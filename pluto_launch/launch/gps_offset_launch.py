from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_offset',
            executable='gps_offsetter',
            name='gps_offsetter'
        ),
    ])