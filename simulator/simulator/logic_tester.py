"""
Logic Test Simulator
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
import rclpy.time_source
from std_msgs.msg import Header, Bool, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion, Vector3

# CALCULATION MODULES
import math

# HELPER MODULES
from .conversions import *

# CONSTANTS
BASE_GPS = (34.841400, -82.411743)

# PARAMETERS
PUBLISH_RATE = 10 # Hz (publishes / second)
INITIAL_HEADING = 127.0 # degrees (-180 to 180)
ANG_VEL_PER_PWM_PERCENT = 0.5 # (degrees / second) / pwm difference %



class LogicTester(Node):

        def __init__(self):
                super().__init__("logic_tester")

                # TIMERS
                self.publish_timer = self.create_timer(
                        1 / PUBLISH_RATE,
                        self.publish_sensor_data
                )

                # PUBLISHERS - SENSOR DATA
                self.gps_pub = self.create_publisher(
                        NavSatFix,
                        "/fix",
                        10
                )
                self.imu_pub = self.create_publisher(
                        Imu,
                        "/imu/data",
                        10
                )

                # PUBLISHERS - OTHER
                self.is_autonomous_mode_pub = self.create_publisher(
                        Bool, 
                        "is_autonomous_mode", 
                        1
                )

                # SUBSCRIBERS
                self.left_pwm_sub = self.create_subscription(
                        Float64,
                        "/steering_left/percentage",
                        self.left_pwm_callback,
                        10
                )
                self.left_pwm = 0
                self.right_pwm_sub = self.create_subscription(
                        Float64,
                        "/steering_right/percentage",
                        self.right_pwm_callback,
                        10
                )
                self.right_pwm = 0

                # VARIABLES
                self.counter = 0
                self.heading = INITIAL_HEADING
                self.x = 0
                self.y = 0


        # TIMER CALLBACKS

        def publish_sensor_data(self):
                """Simulate sensor data to observe logic in other nodes"""
                # publish initial gps
                if self.counter == self.seconds_to_counts(0):
                        self.publish_gps(*BASE_GPS)
                # publish initial heading (Stop -> Turn)
                elif self.counter == self.seconds_to_counts(0.5):
                        self.publish_heading(INITIAL_HEADING)
                # start autonomous mode (to allow some nodes to start subscribing)
                elif self.counter == self.seconds_to_counts(1.0):
                        self.publish_autonomous_mode(True)
                # re-publish initial gps
                elif self.counter == self.seconds_to_counts(1.5):
                        self.publish_gps(*BASE_GPS)
                # re-publish initial heading
                elif self.counter == self.seconds_to_counts(2):
                        self.publish_heading(INITIAL_HEADING)
 
                # publish gps & heading dynamically based on pwm values
                elif self.counter > self.seconds_to_counts(2):
                        # HEADING
                        # convert pwm difference to angular displacement 
                        rotation_pwm = self.right_pwm - self.left_pwm  # (positive = counter-clockwise, negative = clockwise)
                        angular_vel = ANG_VEL_PER_PWM_PERCENT * rotation_pwm
                        time_interval = 1 / PUBLISH_RATE
                        angular_displacement = angular_vel * time_interval
                        # update heading
                        self.heading += angular_displacement
                        # publish new heading
                        self.publish_heading(self.heading)

                        # GPS
                        # only publish every second
                        if self.counter % PUBLISH_RATE == 0:
                                self.publish_gps(*BASE_GPS)


                # update counter
                self.counter += 1


        # HELPERS - PUBLISHING

        def publish_gps(self, lat: float, lon: float):
                # msg = NavSatFix(
                #         header = Header(
                #                 stamp = self.get_clock().now().to_msg(),
                #                 frame_id = "gps_link"
                #         ),
                #         status = NavSatStatus(
                #                 status = 0,
                #                 service = 1
                #         ),
                #         latitude = lat,
                #         longitude = lon,
                #         altitude = 278.299,
                #         position_covariance = [
                #                 0.0169, 0.0,    0.0,
                #                 0.0,    0.0169, 0.0,
                #                 0.0,    0.0,    0.270
                #         ],
                #         position_covariance_type = 1
                # )
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "gps_link"
                msg.status.status = 0
                msg.status.service = 1
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = 278.299
                msg.position_covariance = [
                        0.0169, 0.0,    0.0,
                        0.0,    0.0169, 0.0,
                        0.0,    0.0,    0.270
                ]
                msg.position_covariance_type = 1
                self.gps_pub.publish(msg)
                self.get_logger().info(f"Published GPS: lat={lat}, lon={lon}")

        def publish_heading(self, angle: float):
                """Angle in degrees from -180 to 180"""
                quaternion = euler_to_quaternion(
                        roll = 0,
                        pitch = 0,
                        yaw = math.radians(angle)
                )
                msg = Imu(
                        header = Header(
                                stamp = self.get_clock().now().to_msg(),
                                frame_id = "imu_link"
                        ),
                        orientation = Quaternion(
                                x = quaternion[0],
                                y = quaternion[1],
                                z = quaternion[2],
                                w = quaternion[3],
                        ),
                        orientation_covariance = [
                                0.0324,    0.0,       0.0,
                                0.0,       0.0324,    0.0,
                                0.0,       0.0,       0.0324
                        ],
                        angular_velocity = Vector3(
                                x = 0.0,
                                y = 0.0,
                                z = 0.0
                        ),
                        angular_velocity_covariance = [
                                0.04000000000000001,    0.0,                    0.0,
                                0.0,                    0.04000000000000001,    0.0,
                                0.0,                    0.0,                    0.04000000000000001
                        ],
                        linear_acceleration = Vector3(
                                x = 0.0,
                                y = 0.0,
                                z = 0.0
                        ),
                        linear_acceleration_covariance = [
                                0.32489999999999997,    0.0,                    0.0,
                                0.0,                    0.32489999999999997,    0.0,
                                0.0,                    0.0,                    0.32489999999999997
                        ]
                )
                self.imu_pub.publish(msg)
                self.get_logger().info(f"Published Heading: angle={angle}")

        def publish_autonomous_mode(self, is_autonomous_mode: bool):
                msg = Bool(
                        data = is_autonomous_mode
                )
                self.is_autonomous_mode_pub.publish(msg)
                self.get_logger().info(f"Published is_autonomous_mode: {is_autonomous_mode}")


        # HELPERS - TIMING

        def seconds_to_counts(self, seconds: int | float) -> int:
                # publish_rate = counts / second
                return round(PUBLISH_RATE * seconds)
        

        # SUBSCRIBER CALLBACKS

        def left_pwm_callback(self, msg: Float64):
                self.left_pwm = msg.data

        def right_pwm_callback(self, msg: Float64):
                self.right_pwm = msg.data

# MAIN

def main(args=None):
        rclpy.init(args=args)

        logic_tester = LogicTester()

        rclpy.spin(logic_tester)

        logic_tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()