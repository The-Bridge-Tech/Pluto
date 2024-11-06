from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIGURATION FILES

xbox_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'xbox.config.yaml'
)
joy_config = LaunchConfiguration('joy_config')
joy_dev = LaunchConfiguration('joy_dev')
config_filepath = LaunchConfiguration('config_filepath')
joy_vel = LaunchConfiguration('joy_vel')


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        # launch arguments
        DeclareLaunchArgument(
            'joy_vel', 
            default_value='cmd_vel_joy'
        ),
        DeclareLaunchArgument(
            'joy_dev', 
            default_value='/dev/input/js0'
        ),
        DeclareLaunchArgument(
            'config_filepath', 
            default_value=[xbox_config]
        ),
        Node(
            package='joy_linux', 
            executable='joy_linux_node', 
            name='joy_linux_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='teleop_twist_joy', 
            executable='teleop_node',
            name='teleop_twist_joy_node', 
            parameters=[config_filepath],
            remappings={
                ('/cmd_vel', joy_vel)
            },
        ),
    ])