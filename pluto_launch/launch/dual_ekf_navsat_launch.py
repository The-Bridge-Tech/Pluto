
from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
ekf_navsat_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'dual_ekf_navsat.yaml'
)
navsat_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'navsat_transform.yaml'
)


def generate_launch_description():

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'output_final_position',
            default_value='false'),
        launch.actions.DeclareLaunchArgument(
            'output_location',
            default_value='~/dual_ekf_navsat_example_debug.txt'),

        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_navsat_config],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_navsat_config],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_navsat_config],
            remappings=[#('imu', 'imu/data_raw'),
                        ('gps/fix', 'fix'),
                        ('gps/filtered', 'gps/filtered'),
                        ('odometry/gps', 'odometry/gps'),
                        ('odometry/filtered', 'odometry/global'),
                        ('imu','imu/data')
                        ]

        )
    ])
