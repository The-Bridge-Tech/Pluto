
# Top-most level of the launch file

import os
import launch_ros.actions

import time
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
gazebo_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'gazebo.yaml'
)
gazebo_launch_dir = os.path.join(
    get_package_share_directory('gazebo_ros'), 'launch')
# yolo_model_directory = os.path.join(

#        get_package_share_directory('pluto_launch'),
#       'model',
#       'yolox_s.xml'
#   )
# yolo_config = os.path.join(

#     get_package_share_directory('pluto_launch'),
#     'config',
#     'yolox_openvino.yaml'
# )


def generate_launch_description():

    # TODO: The following launch gazebo. Needed for it to publish /clock.
    # In the future, I will replace gazebo with a launch file that launch robot's description
    # warning info :https://answers.ros.org/question/378362/how-to-set-gazebo-clock-publish-rate-foxy/

    launch_gazebo_config_directory = 'extra_gazebo_args:="--ros-args --params-file {0}"', gazebo_config
    start_gazebo_cmd = ExecuteProcess(
        # ros2 launch gazebo_ros gzserver.launch.py extra_gazebo_args:="--ros-args --params-file params.yaml"

        cmd=['ros2', 'launch', 'gazebo_ros',
             'gzserver.launch.py', launch_gazebo_config_directory],
        output='screen')

    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(gazebo_launch_dir, '/gzserver.launch.py')])
    # ),

    # gz_process = ExecuteProcess(cmd='gazebo', '--verbose', LaunchConfiguration('world'),
    #                             '-s', 'libgazebo_ros_factory.so'], output='screen')

    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/imu.launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/imu_filter.launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )

    joystick_interpreter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/joystick_mode_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/controller_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    gps_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/nmea_serial_driver.launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    camera_launch  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/realsense_camera_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )

    # yolo_mode_launch = IncludeLaunchDescription(
    #   PythonLaunchDescriptionSource([os.path.join(
    #      get_package_share_directory('yolox_ros_cpp'), "launch"),
    #      '/yolox_openvino.launch.py']),
    #       launch_arguments={
    #             'model_path': yolo_model_directory,
    #             "src_image_topic_name": "/camera/color/image_raw" ,
    #             "imshow_isshow":"true"
    #             }.items(),

    #  )
    static_transform_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/static_transform_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    maestro_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/maestro_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    differential_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/differential_odometry.launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),       
    )
    ekf_filter = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/dual_ekf_navsat_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    
    gps_filter_launch =IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/gps_filter_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )
    gps_velocity_launch =IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('pluto_launch')),
            '/gps_velocity_launch.py']),
        # launch_arguments={
        #     "use_sim_time": use_sim_time
        # }.items(),
    )

    return LaunchDescription([
        # #imu_launch,
        # #https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Substitutions.html
        # #DeclareLaunchArgument('use_sim_time',default_value="True", description="All node use simulate clock"),
        #start_gazebo_cmd,

        #controller_launch,
        # #ekf_filter,
        # #launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        # # #controller_launch,
        # # # 'use_sim_time' will be set on all nodes following the line above
        # # #yolo_mode_launch,

        # # #https://answers.ros.org/question/371458/should-i-manually-pass-use_sim_time-to-all-nodes-in-a-ros-2-launchfile/
        joystick_interpreter,
        imu_launch,
        imu_filter_launch,
        controller_launch,
        gps_launch,
        static_transform_launch,
        maestro_launch,
        gps_filter_launch,
        gps_velocity_launch,
        ekf_filter










    ])
