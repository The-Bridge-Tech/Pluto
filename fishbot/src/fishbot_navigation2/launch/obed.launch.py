from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    map_config = os.path.join(
        get_package_share_directory('fishbot_navigation2'),
        'params',
        'nav2_params.yaml'
    )

    bt_path = os.path.join(
        get_package_share_directory('fishbot_navigation2'),
        'config',
        'behavior_tree.xml'
    )

    nav2_launch_file_dir = os.path.join(get_package_share_directory('fishbot_navigation2'), 'launch')
    print(nav2_launch_file_dir)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={
                "use_sim_time": "False",
                "autostart": "True",
                "params_file": map_config,
                "default_nav_through_poses_bt_xml": bt_path
            }.items()
        ),
    ])