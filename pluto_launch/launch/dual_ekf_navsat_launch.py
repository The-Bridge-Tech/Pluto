
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument


# CONFIG FILES

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


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        # launch arguments
        DeclareLaunchArgument(
            'output_final_position',
            default_value='false'
        ),
        DeclareLaunchArgument(
            'output_location',
            default_value='~/dual_ekf_navsat_example_debug.txt'
        ),
        # filters /odometry/local
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_navsat_config],
            remappings=[('odometry/filtered', 'odometry/local')]
        ),
        # filters /odometry/global
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[ekf_navsat_config],
            remappings=[('odometry/filtered', 'odometry/global')]
        ),
        # publishes odometry data using gps and imu topics
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_navsat_config],
            remappings=[
                #('imu', 'imu/data_raw'),
                ('gps/fix', 'fix/filtered'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
                ('imu','imu/data')
            ]
        )
    ])