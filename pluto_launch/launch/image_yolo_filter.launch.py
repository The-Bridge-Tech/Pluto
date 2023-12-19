

# https://stackoverflow.com/questions/73579586/how-to-read-specific-parameter-from-yaml-in-ros2-py-launch-file


# colcon build --packages-select controller
#ros2 launch controller controller_launch.py

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
            package='image_yolo_filter',
            executable='image_filter',
            name='image_yolo_filter',
       parameters=[
            {"cacheSize": 20},
            {"debug" : True}
            
        ],
            ),

    ])
