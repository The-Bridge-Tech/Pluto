from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIG FILES

differential_odometry_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'odometry_calcuation.yaml'
)


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry_calculation',
            executable='differentialOdometry',
            name='differentialOdometry',
            parameters=[differential_odometry_config]
        ),
    ])
