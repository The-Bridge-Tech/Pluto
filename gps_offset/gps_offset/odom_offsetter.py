"""
Calculate initial odometry UTM coordinate error and compensate further messages with an offset
Author: Matthew Lauriault
Created: 9/5/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# LOGIC:
# 1. wait until autonomous mode first starts -> calculate offset 
# 2. apply offset forever (doesn't matter if you enter manual mode)


class OdomOffsetter(Node):

        def __init__(self):
                super().__init__('odom_offsetter')

                # SUBSCRIBERS
                self.odom_sub = self.create_subscription(
                        Odometry, 
                        "/odometry/global", 
                        self.odom_callback, 
                        10
                )
                self.initial_odom: Odometry = None
                self.current_odom: Odometry = None
                self.is_autonomous_mode_sub = self.create_subscription(
                        Bool, 
                        "is_autonomous_mode", 
                        self.is_autonomous_mode_callback, 
                        1
                )
                self.autonomous_mode_started = False

                # PUBLISHERS
                self.offset_odom_pub = self.create_publisher(
                        Odometry,
                        "/odometry/offset",
                        10
                )

                # VARIABLES
                self.x_offset = None
                self.y_offset = None


        # HELPERS
        
        def calculateOffset(self):
                """Calculate offset error in initial odometry UTM coordinate."""
                # Calculate the offset in UTM coordinates - it should be (0, 0)
                self.x_offset = 0.0 - self.initial_odom.pose.pose.position.x
                self.y_offset = 0.0 - self.initial_odom.pose.pose.position.y
                self.get_logger().info(f"Calculated offset: ({self.x_offset}, {self.y_offset})")

        def applyOffset(self):
                """Apply offset to current odometry UTM coordinate."""
                # Get current UTM coordinates
                current_x = self.current_odom.pose.pose.position.x
                current_y = self.current_odom.pose.pose.position.y
                # Compensate the offset
                corrected_x = current_x + self.x_offset
                corrected_y = current_y + self.y_offset
                # Apply the corrected UTM coordinate to the current Odometry message
                self.current_odom.pose.pose.position.x = corrected_x
                self.current_odom.pose.pose.position.y = corrected_y
                self.get_logger().info(f"Corrected Odom: ({self.current_odom.pose.pose.position.x}, {self.current_odom.pose.pose.position.y})")


        # SUBSCRIBER CALLBACKS

        def odom_callback(self, msg: Odometry):
                """For initial odom message, calculate offset. Apply offset and publish corrected odom message."""
                # Wait for autonous mode to start -> then get initial odom
                if self.autonomous_mode_started:
                        if not self.initial_odom:
                                self.initial_odom = msg
                                self.calculateOffset()
                        self.current_odom = msg
                        self.applyOffset()
                        self.offset_odom_pub.publish(self.current_odom)
                else:
                        self.get_logger().info("Waiting for autonomous mode...")
                        
        def is_autonomous_mode_callback(self, msg: Bool):
                """Trigger when autonomous mode starts."""
                if msg.data: 
                        self.autonomous_mode_started = True



# MAIN

def main(args=None):
        rclpy.init(args=args)
        odom_offsetter = OdomOffsetter()

        rclpy.spin(odom_offsetter)

        odom_offsetter.destroy_node()
        rclpy.shutdown()


# When this file is run as a script
if __name__ == '__main__':
        main()