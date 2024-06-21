"""
[Description]
Author: Matthew Lauriault
Created: 6/12/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
# Data Preprocessing
from cv_bridge import CvBridge
import cv2
import numpy as np

# HELPER MODULES
import ImageClassifier
import LineDetector


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
        self.processing = False
        # Load AI image classifier model
        # self.model = ImageClassifier.load("model1")
    
    def image_callback(self, msg: Image):
        """Processes image messages synchronously such that the most recent message is processed each time this method finishes.
        This way, asynchronous requests don't pile up and processing remains closer to realtime."""
        # ignore the current image if there is already one being processed
        if not self.processing:
            self.processing = True
            try:
                # Get image array from message
                image = self.msg_to_image_array(msg)
                # Detect line from image
                line = LineDetector.detectLine(image)
                # Log the line
                self.get_logger().info(f"Detected {line}")
                # Record the image with detected line drawn on top
                line.record(image)

                # TODO STEERING CALCULATIONS
                # * use line attributes

                # *********** CUT / UNCUT CLASSIFICATION ***********
                # Divide image into left and right regions based on the line
                # left_region, right_region = line.divideImage(image)
                # Classify the regions as cut or uncut
                # left_cut = self.model.classify(left)
                # right_cut = self.model.classify(right)
                # self.get_logger().info("Left side is " + "cut" if left_cut else "UNCUT")
                # self.get_logger().info("Right side is " + "cut" if right_cut else "UNCUT")
                # ***************************************************
            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
            self.processing = False

    def msg_to_image_array(self, msg: Image) -> np.ndarray:
        """Converts ROS Image msg to a numpy array containing the image data."""
        # Convert the Image msg to a cv2 image
        cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Resize the image to 224x224 pixels
        resized_image = cv2.resize(cv_image, (224, 224))
        # Convert the image to a numpy array
        image_array = np.array(resized_image, dtype=np.float32)
        # Normalize the image
        image_array /= 255.0
        # Add batch dimension
        image_array = np.expand_dims(image_array, axis=0)
        return image_array



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