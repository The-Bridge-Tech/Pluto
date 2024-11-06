from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIG FILES

location_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'location.yaml'
)


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        # customize_local_planner/phase_one_demo.py
        Node(
            package='customize_local_planner',  # TODO customize_global_planner
            executable='phase_one_demo',        # TODO global_planner
            name='phase_one_demo',              # TODO global_planner
            output='screen',
            parameters=[location_config]
        ),
    ])