from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIG FILES

local_planner2_config = os.path.join(
    get_package_share_directory('pluto_launch'), 
    'config',
    'local_planner2.yaml'
)
servos_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'servos.yaml'
)


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        # customize_local_planner/local_planner2.py
        Node(
            package='customize_local_planner',
            executable='local_planner',
            name='local_planner',
            output='screen',
            parameters=[
                local_planner2_config,
                servos_config
            ]
        ),
    ])