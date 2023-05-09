from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
def generate_launch_description():
    imu_to_base_link = Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['-0.5588','0','0','0','0','0','base_link', 'imu_link'],
            # parameters=[{"use_sim_time":use_sim_time}]
        )
    gps_to_base_link =Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['-0.5588','-0.0508','0','0','0','0','base_link', 'gps_link'],
            # parameters=[{"use_sim_time":use_sim_time}]
    )
    camera_to_baselink = Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['-0/75','0','0','0','0','0','base_link', 'camera_link'],
            # parameters=[{"use_sim_time":use_sim_time}]
    )
    return LaunchDescription(
        
        [
            imu_to_base_link, gps_to_base_link,camera_to_baselink
        ]
    )