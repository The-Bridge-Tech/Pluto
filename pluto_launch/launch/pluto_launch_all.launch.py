
# Top-most level of the launch file

import os
import launch_ros.actions

import time
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    imu_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pluto_launch')),
         '/imu.launch.py'])
      )

    joystick_interpreter = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pluto_launch')),
         '/joystick_mode_launch.py'])
      )
    
    controller_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pluto_launch')),
         '/controller_launch.py'])
      )
    gps_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pluto_launch')),
         '/nmea_serial_driver.launch.py'])
      )
    camera_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense2_camera'),"launch"),
         '/rs_launch.py'])
      )


    return LaunchDescription([
        joystick_interpreter,
        
        camera_launch,
        imu_launch,

        controller_launch,
        gps_launch,
        launch_ros.actions.Node(
            package='maestro_controller',
            executable='controller', 
            name='maestro_Controller',
    
        ),


   ])