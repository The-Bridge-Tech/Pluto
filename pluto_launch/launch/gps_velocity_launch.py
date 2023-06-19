

import os

from pytest import param

from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    
    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='gps_velocity',
            executable='gps_velocity',
            name='gps_velocity',

            ),

    ])
