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
        self.declare_parameter('odom_topic', 'odom/global')
        
        # Initialize subscriptions using the parameter-defined topics
        self.initialize_subscriptions()

    def initialize_subscriptions(self):
        # Retrieve parameter values for topic names
        imu_data_topic = self.get_parameter('imu_data_topic').get_parameter_value().string_value
        fix_filtered_topic = self.get_parameter('fix_filtered_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value

        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        self.filtered_publisher = self.create_publisher(Filtered,'filtered', 10)
        self.joy_publisher = self.create_publisher(Joy, 'imu', 10)
        self.odometry_publisher = self.create_publisher(TwistStamped, 'odometry', 10)
        
        
        #subscriptions
        self.imu_data_subscription=self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_data_callback,
            10
        )
        
        self.fix_filtered_subscription=self.create_subscription(
            NavSatFix,
            "/fix/filtered",
            self.fix_filtered_callback,
            10
        )

        self.joy_subscription = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10    
        )
        
        self.odometry_global_subscription = self.create_subscription(
            Odometry,
             "/odometry/global",
             self.odometry_global_callback,
             10   
        )
         #odometry/global
         #odometry local
    
        
    def odometry_global_callback(self, msg:Odometry):
         events = {
             "index":  "plutotest",
             "event":{"/odometry/global": msg}
         }
         self.sendDataToSplunk(events)      
    def joy_callback(self, msg:Joy):
        events = {
            "index":  "plutotest",
            "event":{"/joy": msg}
        }
        self.sendDataToSplunk(events)
    def fix_filtered_callback(self, msg:NavSatFix):
        events = {
            "index":  "plutotest",
            "event":{"/fix/filtered": msg}
        }
        self.sendDataToSplunk(events)
    def imu_data_callback(self, msg:Imu):
        events = {
            "index":  "plutotest",
            "event":{"/imu/data": msg}
        }

        self.sendDataToSplunk(events)
    def velocity_gps_nonholonomic_callback(self, msg:TwistWithCovarianceStamped):
        events = {
            "index":  "plutotest",
            "event":{"/velocity/gps_nonholonomic": msg}
        }

        self.sendDataToSplunk(events)
    def ros2_logger_callback(self, msg:Log):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        
        timeStamp = "sec {0} nanosec{1}".format(msg.stamp.sec,msg.stamp.nanosec)
        events = {
            "index": "plutotest",
            # "event":{
            # "stamp":timeStamp,
            # "level":msg.level,
            # "name": msg.name,
            # "msg":msg.msg,
            # "file":msg.file,
            # "function": msg.function,
            # "line":msg.line}
            "event":{
                "/rosout":msg
            }
        }


    def sendDataToSplunk(self, eventMessage):
        response = requests.post(
        "http://splunk.thebridgetech.org:8088/services/collector/event",
        headers={"Authorization": "Splunk 0380a1f0-c1a8-4ac8-8ab7-3e17bf71e147"},
        data=json.dumps(eventMessage, ensure_ascii=False, default=str).encode("utf-8"),
        )


        params = self.get_parameter_names()
        for param in params:
            param_value = self.get_parameter(param)
        if 'topic' in param_value.value.data:
            topic_name = param_value.value.data['topic']

    def readYAML(self):
        with open('topics.yml', 'r') as file:
            data_node = yaml.safe_load(file)
        print(data_node['imudata'])
        print(data_node['fixfiltered'])
        print(data_node['joy'])
        print(data_node['odometry'])
     


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