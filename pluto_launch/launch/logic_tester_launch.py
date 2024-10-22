from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to YAML file with params
    params_file = os.path.join(
        get_package_share_directory('pluto_launch'), 
        'config',
        'logic_tester.yaml'
    )
    # Return launch description for logic_tester node
    return LaunchDescription([
        Node(
            package='simulator',
            executable='logic_tester',
            name='logic_tester',
            output='screen',
            parameters=[params_file]
        ),
    ])