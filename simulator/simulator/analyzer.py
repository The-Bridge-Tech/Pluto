"""
Sensor Data Analyzer
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node

# FILE WRITING
import os
import csv

# HELPER MODULES
from custom_msgs.msg import AnalysisMsg

# CONSTANTS
TEST_DATE = "1/8/25"
CSV_FILE = os.path.join(
        "src",
        "Pluto",
        "simulator",
        "simulator",
        f"analysis_{TEST_DATE.replace('/', '_')}.csv",
)
# csv header: "time (s)", "state", "heading (°)", "goal heading (°)", "x (m)", "y (m)", "goal x (m)", "goal y (m)", "left pwm (%)", "right pwm (%)"


class Analyzer(Node):

        def __init__(self):
                super().__init__("analyzer")

                # SUBSCRIBERS
                self.analysis_sub = self.create_subscription(
                        AnalysisMsg,
                        "/analysis/all",
                        self.analysis_callback,
                        10
                )


        # SUBSCRIBER CALLBACKS

        def analysis_callback(self, msg: AnalysisMsg):
                # extract data from msg
                data = [
                        # float64 seconds
                        msg.seconds,
                        # std_msgs/String state
                        msg.state.data,
                        # float64 heading
                        msg.heading,
                        # float64 goal_heading
                        msg.goal_heading,
                        # geometry_msgs/Point local_position
                        msg.local_position.x,
                        msg.local_position.y,
                        # geometry_msgs/Point goal_position
                        msg.goal_position.x,
                        msg.goal_position.y,
                        # float64 left_pwm
                        msg.left_pwm,
                        # float64 right_pwm
                        msg.right_pwm,
                ]
                # write data to new row in csv file
                with open(CSV_FILE, mode='a', newline='') as f:
                        writer = csv.writer(f)
                        writer.writerow(data)


# MAIN

def main(args=None):
        rclpy.init(args=args)

        analyzer = Analyzer()

        rclpy.spin(analyzer)

        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()