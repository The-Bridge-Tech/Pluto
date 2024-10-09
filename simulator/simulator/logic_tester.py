"""
Logic Test Simulator
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
import rclpy.time_source
from sensor_msgs.msg import NavSatFix, Imu

# CONSTANTS
PUBLISH_RATE = 10 # Hz (publishes / second)

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

                # VARIABLES
                self.counter = 0


        # TIMER CALLBACKS

        def publish_sensor_data(self):
                """Simulate sensor data to observe logic in other nodes"""
                # initial gps & heading (Stop -> Turn)
                if self.at_seconds(0):
                        pass
                # gradually change heading to the goal angle (Turn -> Straight)

                # gradually change gps to the goal gps (Straight -> Stop)

                # update counter
                self.counter += 1


        # HELPERS - PUBLISHING

        def publish_gps(self, lat: float, lon: float):
                pass

        def publish_heading(self, angle: int | float):
                pass


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