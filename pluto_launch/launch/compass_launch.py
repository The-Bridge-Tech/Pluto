import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    qos_profile = LaunchConfiguration('qos_profile', default='system_default')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_qos_profile_argument = DeclareLaunchArgument(
        'qos_profile',
        default_value='system_default',
        description='Configuration for QoS profile')

    return LaunchDescription([
        declare_use_sim_time_argument,
        declare_qos_profile_argument,
        launch_ros.actions.Node(
            package='compass',
            executable='compass',
            name='compass',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'qos_profile': qos_profile}
            ]
        ),
    ])