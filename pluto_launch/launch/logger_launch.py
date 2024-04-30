import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Set the package name and path
    package_name = 'splunk_logger'
    package_path = get_package_share_directory('pluto_launch'),

    # Include the logger.py node
    logger_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            package_path, 'launch', 'logger_launch.py')])
        launch_arguments={
            'imu_data_topic': 'default_value',
            'fix_filtered_topic': 'default_value',
            'joy_topic': 'default_value',
            'odom_topic': 'default_value',
        }.topics()
    )

    # Create the launch description
    return LaunchDescription([
        logger_launch_description
    ])

