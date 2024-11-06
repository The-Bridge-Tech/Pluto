from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIG FILES

imu_filter_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'imu_filter.yaml'
)


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[imu_filter_config],
            remappings=[
                #('/imu/data','/imu')
            ]
        ),
    ])