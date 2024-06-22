import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

yolo_model_folder  = os.path.join(
      get_package_share_directory('pluto_launch'),
      'yolo_model',

      )
yolo_x_info = [os.path.join(yolo_model_folder, "/yolox_x.py"), os.path.join(yolo_model_folder, "/yolox_x.pth")]
yolo_nano_info = [os.path.join(yolo_model_folder, "/yolox_nano.py"), os.path.join(yolo_model_folder, "/yolox_nano.pth")]

def generate_launch_description():
    yolox_ros_share_dir = get_package_share_directory('yolox_ros_py')

    # video_device = LaunchConfiguration('video_device', default='/dev/video0')
    # video_device_arg = DeclareLaunchArgument(
    #     'video_device',
    #     default_value='/dev/video0',
    #     description='Video device'
    # )

    # webcam = launch_ros.actions.Node(
    #     package="v4l2_camera", executable="v4l2_camera_node",
    #     parameters=[
    #         {"image_size": [640,480]},
    #         {"video_device": video_device},
    #     ],
    # )

    yolox_ros = launch_ros.actions.Node(
        package="yolox_ros_py", executable="yolox_ros",
        name="yolox_ros",
        parameters=[
            {"yolox_exp_py" : yolox_ros_share_dir+'/yolox_nano.py'},
            #{"yolox_exp_py" : yolo_x_info[0]},
            {"device" : 'cpu'},
            {"fp16" : True},
            {"fuse" : False},
            {"legacy" : False},
            {"trt" : False},
            {"ckpt" : yolox_ros_share_dir+"/yolox_nano.pth"},
            #{"ckpt" : yolo_x_info[1]},
            {"conf" : 0.3},
            {"threshold" : 0.65},
            {"resize" : 640},
            #{"imshow_isshow": False}
        ],
        remappings=[('image_raw', '/camera/color/image_raw'),]           

    )

    rqt_graph = launch_ros.actions.Node(
        package="rqt_graph", executable="rqt_graph",
    )

    return launch.LaunchDescription([
        # video_device_arg,
        # webcam,
        yolox_ros,
        # rqt_graph
    ])