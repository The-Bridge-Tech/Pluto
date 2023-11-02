from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

#TODO: comeback to correct later
# use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
def generate_launch_description():
    imu_to_base_link = Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['0','0','0','0','0','0','base_link', 'imu_link'],
            # parameters=[{"use_sim_time":use_sim_time}]
        )
    gps_to_base_link =Node(
            package="tf2_ros", executable="static_transform_publisher",
            #arguments=['0','0','1','0', '0','0','base_link', 'gps_link'],
            arguments=['0','0','1','1.5707963', '0','3.1415926','base_link', 'gps_link'],  # transform NED to ENU frame
            # parameters=[{"use_sim_time":use_sim_time}]
    )
    camera_to_baselink = Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=['0','0','0','0','0','0','base_link', 'camera_link'],
            # parameters=[{"use_sim_time":use_sim_time}]
    )
    return LaunchDescription(
        
        [
            imu_to_base_link, gps_to_base_link,camera_to_baselink
        ]
    )