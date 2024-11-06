from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


# CONFIG FILES

servos_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'servos.yaml'
)
controller_keycodes_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'controller_keycodes.yaml'
)


# LAUNCH DESCRIPTION

def generate_launch_description():
    return LaunchDescription([
        # controller/ControllerNode.py
        Node(
            package='controller',
            executable='controller',
            name='controller',
            parameters=[servos_config]
        ),
        # controller/JoystickInterpreter.py
        Node(
            package='controller',
            executable='joystickInterpreter', 
            name='joystickInterpreter',
            parameters=[
                servos_config,
                controller_keycodes_config
            ]
        ),
    ])
