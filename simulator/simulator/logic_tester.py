"""
Logic Test Simulator
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
import rclpy.time_source
from std_msgs.msg import Header, Bool
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
                        10
                )

                # VARIABLES
                self.counter = 0


        # TIMER CALLBACKS

        def publish_sensor_data(self):
                """Simulate sensor data to observe logic in other nodes"""
                # initial gps & heading (None -> Stop)
                if self.at_seconds(0):
                        self.publish_gps(*BASE_GPS)
                        self.publish_heading(INITIAL_HEADING)
                # start autonomous mode (Stop -> Turn)
                elif self.at_seconds(1):
                        self.publish_autonomous_mode(True)
                        
                # gradually change heading to the goal angle (Turn -> Straight)

                # gradually change gps to the goal gps (Straight -> Stop)

                # update counter
                self.counter += 1


        # HELPERS - PUBLISHING

        def publish_gps(self, lat: float, lon: float):
                msg = NavSatFix(
                        header = Header(
                                stamp = self.get_clock().now().to_msg(),
                                frame_id = "gps_link"
                        ),
                        status = NavSatStatus(
                                status = 0,
                                service = 1
                        ),
                        latitude = lat,
                        longitude = lon,
                        altitude = 278.299,
                        position_covariance = [
                                0.0169, 0.0,    0.0,
                                0.0,    0.0169, 0.0,
                                0.0,    0.0,    0.270
                        ],
                        position_covariance_type = 1
                )
                self.gps_pub.publish(msg)

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

        def publish_autonomous_mode(self, is_autonomous_mode: bool):
                msg = Bool(
                        data = is_autonomous_mode
                )
                self.is_autonomous_mode_pub.publish(msg)

        # HELPERS - TIMING

        def seconds_to_counts(self, seconds: int | float) -> int:
                # publish_rate = counts / second
                return round(PUBLISH_RATE * seconds)
        
        def at_seconds(self, seconds: int | float) -> bool:
                return self.counter == self.seconds_to_counts(seconds)


# MAIN

def main(args=None):
        rclpy.init(args=args)

        logic_tester = LogicTester()

        rclpy.spin(logic_tester)

        logic_tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()