
# Top-most level of the launch file

import os
import launch_ros.actions

import time
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

gazebo_config = os.path.join(
      get_package_share_directory('pluto_launch'),
      'config',
      'gazebo.yaml'
      )
def generate_launch_description():
  
    #TODO: The following launch gazebo. Needed for it to publish /clock.
    # In the future, I will replace gazebo with a launch file that launch robot's description
    # warning info :https://answers.ros.org/question/378362/how-to-set-gazebo-clock-publish-rate-foxy/
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so','--ros-args', '--params-file', gazebo_config],
        output='screen')
    
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
        start_gazebo_cmd,
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        # 'use_sim_time' will be set on all nodes following the line above
        #https://answers.ros.org/question/371458/should-i-manually-pass-use_sim_time-to-all-nodes-in-a-ros-2-launchfile/
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