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
        # simulator/gps_plotter.py
        Node(
            package='simulator',
            executable='gps_plotter',
            name='gps_plotter',
            output='screen',
            parameters=[location_config]
        )
    ])