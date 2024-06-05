# Matthew Lauriault
# Send GPS data to splunk server
# 6/5/24


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu, Joy
from rcl_interfaces.msg import Log
from nav_msgs.msg import Odometry
# SENDING DATA
import json
import requests
from socket import gethostname


# NODE
class SplunkLogger(Node):
    
    def __init__(self):
        super().__init__('splunk_logger')
        # Logger
        self.log_subscriber = self.create_subscription(
            Log,
            "rosout",
            self.log_callback,
            10
        )
        # GPS
        self.fix_subscriber = self.create_subscription(
            NavSatFix,
            "/fix/filtered",
            self.fix_callback,
            10
        )
        # IMU
        self.imu_subscriber = self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10
        )
        # Joystick
        self.joy_subscriber = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )
        # Odometry
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "/odometry/global",
            self.odometry_callback,
            10
        )

    def sendToSplunk(self, event: dict) -> int:
        """Make HTTP POST request to splunk server. Return status code."""
        response = requests.post(
            "http://23.126.4.97:8013/services/collector/event",
            headers={"Authorization": "Splunk 87ba4168-a30a-48f6-b809-2859b7052a46"},
            data=json.dumps(event, ensure_ascii=False).encode("utf-8"),
        )
        return response.status_code
    
    def log_callback(self, msg: Log) -> None:
        """Callback for log_subscriber"""
        levels = {
            "10": "debug",
            "20": "info",
            "30": "warn",
            "40": "error",
            "50": "fatal"
        }
        event = {
            "index": "pluto",
            "event": {
                "sensor": "log", # I know that this is not a sensor
                "devicename": gethostname(),
                "timestamp": str(msg.stamp),
                "level": levels[str(msg.level)], 
                "name": msg.name,
                "msg": msg.msg,
                "file": msg.file,
                "function": msg.function,
                "line": msg.line
            },
        }
        status_code = self.sendToSplunk(event)
        print(f"{status_code}\tLog sent.")

    def fix_callback(self, msg: NavSatFix) -> None:
        """Callback for fix_subscriber"""
        event = {
            "index": "pluto",
            "event": {
                "sensor": "gps",
                "devicename": gethostname(),
                "lat": msg.latitude, 
                "log": msg.longitude, 
                "alt": msg.altitude
            },
        }
        status_code = self.sendToSplunk(event)
        print(f"{status_code}\tGPS data sent.")

    def imu_callback(self, msg: Imu) -> None:
        """Callback for imu_subscriber"""
        o = msg.orientation
        v = msg.angular_velocity
        a = msg.linear_acceleration
        event = {
            "index": "pluto",
            "event": {
                "sensor": "imu",
                "devicename": gethostname(),
                "orientation": [o.x, o.y, o.z, o.w],
                "angular_velocity": [v.x, v.y, v.z],
                "linear_acceleration": [a.x, a.y, a.z]
            },
        }
        status_code = self.sendToSplunk(event)
        print(f"{status_code}\tImu data sent.")
        
    def joy_callback(self, msg: Joy) -> None:
        """Callback for joy_subscriber"""
        event = {
            "index": "pluto",
            "event": {
                "sensor": "joy",
                "devicename": gethostname(),
                "axes": msg.axes,
                "buttons": msg.buttons
            },
        }
        status_code = self.sendToSplunk(event)
        print(f"{status_code}\tJoy data sent.")

    def odometry_callback(self, msg: Odometry) -> None:
        """Callback for odometry_subscriber"""
        event = {
            "index": "pluto",
            "event": {
                "sensor": "odometry", # I know that this is not a sensor
                "devicename": gethostname(),
                "pose": str(msg.pose.pose),
                "twist": str(msg.twist.twist)
            },
        }
        status_code = self.sendToSplunk(event)
        print(f"{status_code}\tOdometry data sent.")


# MAIN
def main(args=None):
    rclpy.init(args=args)

    splunk_logger = SplunkLogger()
    rclpy.spin(splunk_logger)

    splunk_logger.destroy_node()
    rclpy.shutdown()


# When this file is run as a script
if __name__ == '__main__':
    main()