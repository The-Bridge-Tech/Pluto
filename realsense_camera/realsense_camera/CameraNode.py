"""
[Description]
Author: Matthew Lauriault
Created: 6/12/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image # https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
# Data Preprocessing
from cv_bridge import CvBridge
import cv2
import numpy as np

# HELPER MODULES
# from .ImageClassifier import load
from .LineDetector import detectLine, Line
from custom_msgs.msg import LineMsg


class CameraNode(Node):
    
    def __init__(self):
        super().__init__('camera_node')
        # Subscribe to raw image topic from realsense_camera
        self.image_subscriber = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10
        )
        # Publish detected line (point1, point2, angle)
        self.line_publisher = self.create_publisher(
            LineMsg,
            "/line",
            10
        )
        # Control if node should save images from the camera with the detected line drawn on top
        self.declare_parameter("record_line_detection", False)
        self.processing = False
        # Load AI image classifier model
        # self.model = load("model1")
    
    def image_callback(self, msg: Image):
        """Processes image messages synchronously such that the most recent message is processed each time this method finishes.
        This way, asynchronous requests don't pile up and processing remains closer to realtime."""
        # ignore the current image if there is already one being processed or if msg is None
        if not self.processing:
            self.processing = True
            # Get image array from message
            image = self.msg_to_image_array(msg)
            # Detect line from image
            line = detectLine(image)
            # If a line was detected
            if line:
                # Log the line
                self.get_logger().info(f"Detected line: {line}")
                # Record the image with detected line drawn on top
                if self.get_parameter("record_line_detection").value:
                    line.record(image)
                # Publish line message
                self.publish_line(line)

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
            self.processing = False

    def msg_to_image_array(self, msg: Image) -> np.ndarray:
        """Converts ROS Image message to OpenCV image array."""
        # Convert the Image msg to a cv2 image
        image_array = CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Resize the image to 224x224 pixels
        # resized_image = cv2.resize(image_array, (224, 224))
        # Convert the image to a numpy array
        # image_array = np.array(image_array, dtype=np.float32)
        # Normalize the image
        # image_array /= 255.0
        # Add batch dimension
        # image_array = np.expand_dims(image_array, axis=0)
        return image_array
    
    def publish_line(self, line: Line):
        """Publishes custom line message containing the Line object's attributes."""
        # Construct line message from the object attributes
        msg = LineMsg(
            point1 = Point(
                x=float(line.x1),
                y=float(line.y1),
                z=0.0
            ),
            point2 = Point(
                x=float(line.x2),
                y=float(line.y2),
                z=0.0
            ),
            angle = line.getDegrees()
        )
        # Publish the line message
        self.line_publisher.publish(msg)





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