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
        # self.log_subscription = self.create_subscription(
        #     Log,
        #     'rosout',
        #     self.ros2_logger_callback,
        #     10)
        # self.log_subscription  # prevent unused variable warning
        
        # self.velocity_gpsNonholonomic = self.create_subscription(
        #     TwistWithCovarianceStamped,
        #     "/velocity/gps_nonholonomic",
        #     self.velocity_gps_nonholonomic_callback,
        #     10
        # )
        
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
            
        )
        ## odometry/global
        ## odometry local
    
        
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

        self.sendDataToSplunk(events)
        # response = requests.post(
        # "http://splunk.thebridgetech.org:8088/services/collector/event",
        # headers={"Authorization": "Splunk 0380a1f0-c1a8-4ac8-8ab7-3e17bf71e147"},
        # data=json.dumps(events, ensure_ascii=False, default=str).encode("utf-8"),
        # )
        # print(f"Event is sent, status code: {response.status_code}")


    def sendDataToSplunk(self, eventMessage):
        response = requests.post(
        "http://splunk.thebridgetech.org:8088/services/collector/event",
        headers={"Authorization": "Splunk 0380a1f0-c1a8-4ac8-8ab7-3e17bf71e147"},
        data=json.dumps(eventMessage, ensure_ascii=False, default=str).encode("utf-8"),
        )
        print(f"Event is sent, status code: {response.status_code}")


    
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