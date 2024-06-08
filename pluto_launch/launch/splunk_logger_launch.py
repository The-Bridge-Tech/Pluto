from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='splunk_logger',
            executable='splunk_logger',
            name='splunk_logger'
        ),
    ])