"""
Sensor Data Analyzer
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf_transformations

# CALCULATION MODULES
import math

# FILE WRITING
import csv

# CONSTANTS
PROCESS_RATE = 10 # Hz (times / second)
CSV_FILE = "analysis.csv"
PWM_MIN = 992
PWM_NEUTRAL = 1376
PWM_MAX = 1765

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


        # HELPERS

        def pwm_value_to_percentage(self, pwm: int) -> float:
                if pwm == PWM_NEUTRAL:
                        return 0
                elif pwm < PWM_NEUTRAL:
                        return ((pwm - PWM_NEUTRAL) / (PWM_NEUTRAL - PWM_MIN)) * 100
                elif pwm > PWM_NEUTRAL:
                        return ((pwm - PWM_NEUTRAL) / (PWM_MAX - PWM_NEUTRAL)) * 100
                
        def angle_from_odometry(self, odom: Odometry):
                """Angle is returned in the range -180 to 180 degrees"""
                q = [
                        odom.pose.pose.orientation.x, 
                        odom.pose.pose.orientation.y, 
                        odom.pose.pose.orientation.z, 
                        odom.pose.pose.orientation.w
                ]
                # rpy = [roll, pitch, yaw]
                rpy = tf_transformations.euler_from_quaternion(q)
                # rpy[2] = yaw (orientation around the vertical axis)
                return math.degrees(rpy[2])


        # SUBSCRIBER CALLBACKS
        def odom_callback(self, msg: Odometry):
                self.x = msg.pose.pose.position.x
                self.y = msg.pose.pose.position.y
                self.heading = self.angle_from_odometry(msg)

        def left_pwm_callback(self, msg: UInt32):
                self.left_pwm = self.pwm_value_to_percentage(msg.data)

        def right_pwm_callback(self, msg: UInt32):
                self.right_pwm = self.pwm_value_to_percentage(msg.data)


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