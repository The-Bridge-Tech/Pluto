#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import launch_ros.actions

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node




def generate_launch_description():
    
    
    bringup_dir = get_package_share_directory('fishbot_navigation2')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    # Parameters
    lifecycle_nodes = ['map_saver']
    use_sim_time = True
    autostart = True
    save_map_timeout = 2.0
    free_thresh_default = 0.25
    occupied_thresh_default = 0.65
    
    default_map_params = os.path.join(bringup_dir,'maps','fishbot_map.yaml')
    
    # print(bringup_dir)
    # print("^^^^^^^^^^^^^^")
    # print(default_map_params    )

    # Nodes launching commands
    start_map_saver_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_saver_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'save_map_timeout': save_map_timeout},
                        {'free_thresh_default': free_thresh_default},
                        {'occupied_thresh_default': occupied_thresh_default}])

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    
    
    
    map_server = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': default_map_params}]
    )
    

    ld = LaunchDescription()

    ld.add_action(map_server)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld