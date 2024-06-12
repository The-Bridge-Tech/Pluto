# Matthew Lauriault
# [Description]
# 6/12/24


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
# Image Classifier AI Model
from ImageClassifier import classify

# NODE
class CameraNode(Node):
    
    def __init__(self):
        super().__init__('camera_node')
        # Subscribe to raw image topic from realsense_camera
        self.image_subscriber = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )
    
    def image_callback(self, msg: Image):
        # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
        image = msg.data
        # TODO change the message data into a format that the AI model can recognize
        cut = classify(image)
        # Log the classification
        self.get_logger().info(f"The grass is {'cut' if cut else 'NOT cut'}")
            


# MAIN
def main(args=None):
    rclpy.init(args=args)

    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.destroy_node()
    rclpy.shutdown()


# When this file is run as a script
if __name__ == '__main__':
    main()