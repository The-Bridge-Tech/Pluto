from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIG FILES

logic_tester_config = os.path.join(
        get_package_share_directory('pluto_launch'), 
        'config',
        'logic_tester.yaml'
    )


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulator',
            executable='logic_tester',
            name='logic_tester',
            output='screen',
            parameters=[logic_tester_config]
        ),
    ])