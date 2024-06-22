
# Top-most level of the launch file

import os
import launch_ros.actions

import time
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
gazebo_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'gazebo.yaml'
)
gazebo_launch_dir = os.path.join(
    get_package_share_directory('gazebo_ros'), 'launch')
# yolo_model_directory = os.path.join(

#        get_package_share_directory('pluto_launch'),
#       'model',
#       'yolox_s.xml'
#   )
# yolo_config = os.path.join(

#     get_package_share_directory('pluto_launch'),
#     'config',
#     'yolox_openvino.yaml'
# )


def generate_launch_description():

    # TODO: need have tf read
    fishbot_nav2_launch =  IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('fishbot_navigation2'), 'launch'),
        '/fishbot_navigation_humble.launch.py'])
    )
    coverage_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('select_area'), 'launch'),
            '/coverage_area_client_launch.py'])
        )
    gps_navigate_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('navigate_by_pose'), 'launch'),
            '/navigate_by_gps_demo_launch.py'])
        )
    return LaunchDescription([
    fishbot_nav2_launch,
    coverage_client,
    gps_navigate_demo,






    ])
