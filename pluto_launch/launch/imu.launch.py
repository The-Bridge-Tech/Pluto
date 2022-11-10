import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource



def generate_launch_description():

    spatial_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pluto_launch'), ),
         '/spatial-launch.py'])
      )
    imu_tool = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('pluto_launch'), ),
         '/imu_filter.launch.py'])
      )
    
    return LaunchDescription([
      spatial_launch,
      imu_tool
   ])