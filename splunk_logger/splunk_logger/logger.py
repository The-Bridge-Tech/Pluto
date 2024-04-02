import rclpy
from rclpy.node import Node
import json
import time
import requests
from std_msgs.msg import String
from rcl_interfaces.msg import Log
from sensor_msgs.msg import NavSatFix, Imu, Joy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Declare parameters for topic names with default values
        self.declare_parameter('imu_data_topic', '/imu/data')
        self.declare_parameter('fix_filtered_topic', '/fix/filtered')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('odom_topic', '/odometry/global')
        
        # Initialize subscriptions using the parameter-defined topics
        self.initialize_subscriptions()

    def initialize_subscriptions(self):
        # Retrieve parameter values for topic names
        imu_data_topic = self.get_parameter('imu_data_topic').get_parameter_value().string_value
        fix_filtered_topic = self.get_parameter('fix_filtered_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        
        # Subscriptions using parameter-defined topics
        self.imu_data_subscription = self.create_subscription(
            Imu,
            imu_data_topic,
            self.imu_data_callback,
            10
        )
        
        self.fix_filtered_subscription = self.create_subscription(
            NavSatFix,
            fix_filtered_topic,
            self.fix_filtered_callback,
            10
        )

        self.joy_subscription = self.create_subscription(
            Joy,
            joy_topic,
            self.joy_callback,
            10    
        )
        
        self.odometry_global_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odometry_global_callback,
            10   
        )

    def odometry_global_callback(self, msg: Odometry):
        events = {
            "index": "plutotest",
            "event": {self.get_parameter('odom_topic').get_parameter_value().string_value: msg}
        }
        self.sendDataToSplunk(events)      

    def joy_callback(self, msg: Joy):
        events = {
            "index": "plutotest",
            "event": {self.get_parameter('joy_topic').get_parameter_value().string_value: msg}
        }
        self.sendDataToSplunk(events)

    def fix_filtered_callback(self, msg: NavSatFix):
        events = {
            "index": "plutotest",
            "event": {self.get_parameter('fix_filtered_topic').get_parameter_value().string_value: msg}
        }
        self.sendDataToSplunk(events)

    def imu_data_callback(self, msg: Imu):
        events = {
            "index": "plutotest",
            "event": {self.get_parameter('imu_data_topic').get_parameter_value().string_value: msg}
        }
        self.sendDataToSplunk(events)

    def sendDataToSplunk(self, eventMessage):
        response = requests.post(
            "http://splunk.thebridgetech.org:8088/services/collector/event",
            headers={"Authorization": "Splunk 0380a1f0-c1a8-4ac8-8ab7-3e17bf71e147"},
            data=json.dumps(eventMessage, ensure_ascii=False, default=str).encode("utf-8"),
        )

        print("Splunk Server Response:", response.text)

        if response.status_code != 200:
            print("Error sending data to Splunk. Status Code:", response.status_code)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()