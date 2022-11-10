# from struct import pack
# import launch
# import launch_ros.actions

# colcon build --packages-select joystick_launch
#ros2 launch joystick_launch joystick_mode_launch.py

import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions


launch_file_path = os.path.join(
      get_package_share_directory('pluto_launch'),
      'config',
      'xbox.config.yaml'
      )
print(launch_file_path)
def generate_launch_description():
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    config_filepath = launch.substitutions.LaunchConfiguration('config_filepath')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('joy_vel', default_value='cmd_vel_joy'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='xbox'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js5'),
        # launch.actions.DeclareLaunchArgument('config_filepath', default_value=[
        #     launch.substitutions.TextSubstitution(text=os.path.join(
        #         get_package_share_directory('teleop_twist_joy'), 'config', '')),
        #     joy_config, launch.substitutions.TextSubstitution(text='.config.yaml')]),
        launch.actions.DeclareLaunchArgument('config_filepath', default_value=[launch_file_path]),

        launch_ros.actions.Node(
            package='joy_linux', executable='joy_linux_node', name='joy_linux_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]),
        
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[config_filepath],
            
            remappings={('/cmd_vel', launch.substitutions.LaunchConfiguration('joy_vel'))},
            ),
    ])