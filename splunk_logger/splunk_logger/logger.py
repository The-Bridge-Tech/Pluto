# Matthew Lauriault
# Send GPS data to splunk server
# 6/5/24


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
# SENDING DATA
import json
import requests


# NODE
class SplunkLogger(Node):
    
    def __init__(self):
        super().__init__('splunk_logger')
        # Subscriber to GPS data
        self.fix_subscriber = self.create_subscription(
            NavSatFix,
            "/fix",
            self.fix_callback,
            10
        )

    def fix_callback(self, msg: NavSatFix) -> None:
        """Callback for fix_subscriber"""
        event = {
            "index": "pluto",
            "event": {
                "sensor": "gps",
                "lat": msg.latitude, 
                "log": msg.longitude, 
                "type": "info", 
                "devicename": "nuc1"
            },
        }
        status_code = self.sendToSplunk(event)
        self.get_logger().info(f"GPS data sent and returned status code {status_code}")

    def sendToSplunk(self, event) -> int:
        """Make HTTP POST request to splunk server. Return status code."""
        response = requests.post(
            "http://23.126.4.97:8013/services/collector/event",
            headers={"Authorization": "Splunk 87ba4168-a30a-48f6-b809-2859b7052a46"},
            data=json.dumps(event, ensure_ascii=False).encode("utf-8"),
        )
        return response.status_code


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