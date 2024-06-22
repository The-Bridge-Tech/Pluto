import launch
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os
def generate_launch_description():


    container = Node(
        name='phidget_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
    )

    load_composable_nodes = LoadComposableNodes(
        target_container='phidget_container',
        composable_node_descriptions=[
            ComposableNode(
                package='phidgets_spatial',
                plugin='phidgets::SpatialRosI',
                name='phidgets_spatial',

            ),
        ],
    )

    return launch.LaunchDescription([        container,        load_composable_nodes,    ])
