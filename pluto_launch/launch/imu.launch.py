from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


# use_sim_time = LaunchConfiguration('use_sim_time', default='true')


# def generate_launch_description():

#     # spatial_launch = IncludeLaunchDescription(
#     #   PythonLaunchDescriptionSource([os.path.join(
#     #      get_package_share_directory('phidgets_spatial') ),
#     #      '/launch/spatial-launch.py']),
#     #     launch_arguments={
#     #         "use_sim_time": use_sim_time
#     #     }.items(),
#     #   )

#     spatial_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([os.path.join(
#             get_package_share_directory('pluto_launch')),
#             '/spatial-launch.py']),
#         launch_arguments={
#             'angular_velocity_stdev': '0.349056',
#             'linear_acceleration_stdev': '0.20',
#             'magnetic_field_stdev': '0.001658',
#         }.items()

#     )
#     # imu_tool = IncludeLaunchDescription(
#     #   PythonLaunchDescriptionSource([os.path.join(
#     #      get_package_share_directory('pluto_launch')),
#     #      '/imu_filter.launch.py'])
#     #   )

#     return LaunchDescription([
#         spatial_launch,
#         # imu_tool
#     ])


# LAUNCH DESCRIPTION

def generate_launch_description():
    """Generate launch description with multiple components."""
    return LaunchDescription([
        ComposableNodeContainer(
            name='phidget_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='phidgets_spatial',
                        plugin='phidgets::SpatialRosI',
                        name='phidgets_spatial',
                        parameters=[{
                            'angular_velocity_stdev': 0.20,
                            'linear_acceleration_stdev': 0.57,
                            'magnetic_field_stdev': 0.001658,
                        }]),

            ],
            output='both',
        ),
    ])