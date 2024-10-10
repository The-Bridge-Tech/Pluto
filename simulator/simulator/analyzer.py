"""
Sensor Data Analyzer
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from nav_msgs.msg import Odometry

# FILE WRITING
import csv

# HELPER MODULES
from .conversions import *

# CONSTANTS
PROCESS_RATE = 10 # Hz (times / second)
CSV_FILE = "analysis.csv"


class Analyzer(Node):

        def __init__(self):
                super().__init__("analyzer")

                # TIMERS
                self.process_timer = self.create_timer(
                        1 / PROCESS_RATE,
                        self.process
                )

                # SUBSCRIBERS - INPUTS (CONTROL VARIABLES)
                self.left_pwm_sub = self.create_subscription(
                        UInt32,
                        "/steering_left",
                        self.left_pwm_callback,
                        10
                )
                self.right_pwm_sub = self.create_subscription(
                        UInt32,
                        "/steering_right",
                        self.right_pwm_callback,
                        10
                )

                # SUBSCRIBERS - OUTPUTS (SENSOR DATA)
                self.odom_sub = self.create_subscription(
                        Odometry, 
                        "/odometry/global", 
                        self.odom_callback, 
                        10
                )

                # VARIABLES
                self.x = None
                self.y = None
                self.heading = None
                self.left_pwm = None
                self.right_pwm = None


        # SUBSCRIBER CALLBACKS
        def odom_callback(self, msg: Odometry):
                self.x = msg.pose.pose.position.x
                self.y = msg.pose.pose.position.y
                self.heading = angle_from_odometry(msg)

        def left_pwm_callback(self, msg: UInt32):
                self.left_pwm = pwm_value_to_percentage(msg.data)

        def right_pwm_callback(self, msg: UInt32):
                self.right_pwm = pwm_value_to_percentage(msg.data)


        # TIMER CALLBACKS

        def process(self):
                """Analyze inputs (control variables) and outputs (sensor data)"""
                data = [
                        # inputs
                        self.left_pwm,
                        self.right_pwm,
                        # outputs
                        self.x,
                        self.y,
                        self.heading
                ]
                # save current data as a row in csv file
                with open(CSV_FILE, mode='a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(data)
                        self.get_logger().info(f"p_L: {self.left_pwm}% p_R: {self.right_pwm}% x: {self.x} y: {self.y} heading: {self.heading}°")


# MAIN

def main(args=None):
        rclpy.init(args=args)

        analyzer = Analyzer()

        rclpy.spin(analyzer)

        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()