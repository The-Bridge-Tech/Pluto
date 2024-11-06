from launch import LaunchDescription
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# CONFIG FILES

gazebo_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'gazebo.yaml'
)
gazebo_launch_dir = os.path.join(
    get_package_share_directory('gazebo_ros'), 
    'launch'
)
launch_gazebo_config_directory = f'extra_gazebo_args:="--ros-args --params-file {gazebo_config}"'
yolo_model_directory = os.path.join(
       get_package_share_directory('pluto_launch'),
      'model',
      'yolox_s.xml'
  )
yolo_config = os.path.join(
    get_package_share_directory('pluto_launch'),
    'config',
    'yolox_openvino.yaml'
)


# HELPERS

def use_launch_file(filename: str) -> IncludeLaunchDescription:
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('pluto_launch'),
                filename
            ),
        ]),
    )


# LAUNCH DESCRIPTION

def generate_launch_description():
    # GAZEBO
    # TODO: The following launch gazebo. Needed for it to publish / clock.
    # In the future, I will replace gazebo with a launch file that launch robot's description
    # warning info: https://answers.ros.org/question/378362/how-to-set-gazebo-clock-publish-rate-foxy/
    # start_gazebo_cmd = ExecuteProcess(
    #     cmd=[
    #         'ros2', 
    #         'launch', 
    #         'gazebo_ros',
    #         'gzserver.launch.py', 
    #         launch_gazebo_config_directory
    #     ],
    #     output='screen'
    # )
    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             gazebo_launch_dir, 
    #             '/gzserver.launch.py'
    #         )
    #     ])
    # ),
    # gz_process = ExecuteProcess(
    #     cmd=[
    #         'gazebo', 
    #         '--verbose', 
    #         LaunchConfiguration('world'),
    #         '-s', 
    #         'libgazebo_ros_factory.so'
    #     ], 
    #     output='screen'
    # )
    # YOLO
    # yolo_mode_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('yolox_ros_cpp'), 
    #             "launch",
    #             'yolox_openvino.launch.py'
    #         ),
    #     ]),
    #     launch_arguments={
    #         'model_path': yolo_model_directory,
    #         "src_image_topic_name": "/camera/color/image_raw",
    #         "imshow_isshow": "true"
    #     }.items(),
    # )

    # SENSOR - IMU
    imu_launch = use_launch_file('imu.launch.py')
    imu_filter_launch = use_launch_file('imu_filter.launch.py')
    # SENSOR - GPS
    gps_launch = use_launch_file('nmea_serial_driver.launch.py')
    gps_filter_launch =use_launch_file('gps_filter_launch.py')
    gps_velocity_launch =use_launch_file('gps_velocity_launch.py')
    # SENSOR - CAMERA
    camera_launch  = use_launch_file('realsense_camera_launch.py')
    # ODOMETRY
    ekf_filter_launch = use_launch_file('dual_ekf_navsat_launch.py')
    differential_odometry_launch = use_launch_file('differential_odometry.launch.py')
    # STATIC TRANSFORM
    static_transform_launch = use_launch_file('static_transform_launch.py')
    # CONTROLLER
    joystick_launch = use_launch_file('joystick_mode_launch.py')
    controller_launch = use_launch_file('controller_launch.py')
    # SERVOS
    maestro_launch = use_launch_file('maestro_launch.py')
    # LOGGER
    splunk_logger_launch = use_launch_file('splunk_logger_launch.py')

    return LaunchDescription([
        # imu_launch,
        # # https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Using-Substitutions.html
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value="True", 
        #     description="All node use simulate clock"
        # ),
        # start_gazebo_cmd,
        # controller_launch,
        # ekf_filter_launch,
        # SetParameter(name='use_sim_time', value=True),
        # controller_launch,
        # # 'use_sim_time' will be set on all nodes following the line above
        # yolo_mode_launch,
        
        # SENSOR - IMU
        imu_launch,
        imu_filter_launch,
        # SENSOR - GPS
        gps_launch,
        gps_filter_launch,
        gps_velocity_launch,
        # SENSOR - CAMERA
        # camera_launch,
        # ODOMETRY
        ekf_filter_launch, 
        # STATIC TRANSFORM
        static_transform_launch,
        # CONTROLLER
        joystick_launch,
        controller_launch,
        # SERVOS
        maestro_launch,
        # LOGGER
        splunk_logger_launch,
    ])