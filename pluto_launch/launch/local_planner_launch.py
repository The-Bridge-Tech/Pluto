from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to YAML file with params
    params_file = os.path.join(
        get_package_share_directory('pluto_launch'), 
        'config',
        'local_planner2.yaml'
    )
    # Return launch description for localPlanner node
    return LaunchDescription([
        Node(
            package='customize_local_planner',
            executable='localPlanner',
            name='local_planner',
            output='screen',
            parameters=[params_file]
        ),
    ])

# the YAML file path is correct
# the package name matches
# the executable name matches

# the problem is on the node's end

