from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_to_base_link = Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['0','0','1','0','0','0','base_link', 'imu_link']
        )
    gps_to_base_link =Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['0','0','1.5','0','0','0','base_link', 'gps']
    )
    return LaunchDescription(
        
        [
            imu_to_base_link, gps_to_base_link
        ]
    )