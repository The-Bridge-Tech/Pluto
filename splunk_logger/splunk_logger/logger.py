"""
Send logs and sensor data to splunk server.
Author: Matthew Lauriault
Created: 6/5/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import NavSatFix, Imu, Joy
from rcl_interfaces.msg import Log
from nav_msgs.msg import Odometry
from custom_msgs.msg import LineMsg, WaypointMsg, AnalysisMsg
# SENDING DATA
import json
import requests
from socket import gethostname
# MOCK SPLUNK (temporary)
import json
from datetime import datetime
import os
# MESSAGE QUEUE
from threading import Thread
from queue import Queue, PriorityQueue
import tf_transformations
import math
# CONSTANTS
PARENT_DIR = os.path.join("src", "Pluto", "splunk_logger", "splunk_logger")
MOCK_SPLUNK_FILE = os.path.join(PARENT_DIR, "mock_splunk.json")
PRIORITY = { # for message priority queue - adjust as needed
    "ping": 1,
    "message": 1, # 2
    "sensor": 1, # 3
    "calculation": 1 # 3
}


# STRUCTS
# class Message:
#     """Data structure for message priority queue."""
#     def __init__(self, event: dict):
#         self.priority = PRIORITY[event["event"]["type"]]
#         self.event = event
#     # compare by priority
#     def __lt__(self, other):
#         return self.priority < other.priority
#     def __eq__(self, other):
#         return self.priority == other.priority
#     def __gt__(self, other):
#         return self.priority > other.priority


# NODE
class SplunkLogger(Node):
    
    def __init__(self):
        super().__init__('splunk_logger')
        # Message Queue (to store messages until they're sent to splunk)
        # self.messageQueue = Queue() # PriorityQueue()
        # self.messageQueueManagerThread = Thread(
        #     target=self.manageMessageQueue,
        #     daemon=True
        # )

        # SUBSCRIBERS
        
        # ros logs
        self.log_sub = self.create_subscription(
            Log,
            "rosout",
            self.log_callback,
            10,
            callback_group = MutuallyExclusiveCallbackGroup()
        )

        # sensor data (raw)
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/fix/filtered",
            self.gps_callback,
            10,
            callback_group = MutuallyExclusiveCallbackGroup()
        )
        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10,
            callback_group = MutuallyExclusiveCallbackGroup()
        )
        # NOTE imu data is included in odometry data
        
        # calculated data
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odometry/global",
            self.odom_callback,
            10,
            callback_group = MutuallyExclusiveCallbackGroup()
        )
        self.line_sub = self.create_subscription(
            LineMsg,
            "/line",
            self.line_callback,
            10,
            callback_group = MutuallyExclusiveCallbackGroup()
        )
        # self.ping_sub = self.create_subscription(
        #     WaypointMsg,
        #     "/waypoint_ping",
        #     self.ping_callback,
        #     10,
        #     callback_group = MutuallyExclusiveCallbackGroup()
        # )

        # analysis data (from local_planner)
        self.analysis_sub = self.create_subscription(
            AnalysisMsg,
            "/analysis/all",
            self.analysis_callback,
            10,
            callback_group = MutuallyExclusiveCallbackGroup()
        )


    # HELPERS - TIME

    def get_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * (10**-9)

    
    # HELPERS - SPLUNK

    def saveToMockSplunk(self, event: dict) -> bool:
        """Save event to mock splunk json file (temporary). Return if sucessful or not."""
        # extract message json
        message_json = event["event"]
        # read existing messages from json file
        with open(MOCK_SPLUNK_FILE, 'r') as f:
            try:
                messages: list = json.load(f)
            # if file is corrupted or empty
            except json.JSONDecodeError as e:
                messages = []
        # add new message
        messages.append(message_json)
        # write updated messages list to json file
        with open(MOCK_SPLUNK_FILE, 'w') as f:
            json.dump(messages, f, indent=4)

    def sendToSplunk(self, event: dict) -> int:
        """Make HTTP POST request to splunk server. Return status code."""
        response = requests.post(
            "http://23.126.4.97:8013/services/collector/event",
            headers={"Authorization": "Splunk 311596bb-ad45-4507-b9d1-ef7a7c19338d"},
            data=json.dumps(event, ensure_ascii=False).encode("utf-8"),
        )
        # self.get_logger().info(f"{response.status_code}\t {event['event']['id']}")
        return response.status_code
    
    # HELPERS - MESSAGE QUEUE

    def pushMessage(self, event: dict):
        """Create Message object from event with priority and push it to the queue to be sent to splunk."""
        # self.messageQueue.put(Message(event))
        # self.messageQueue.put(event)
        try:
            status_code = self.sendToSplunk(event)
            print(f"{status_code}\t{event['id']}\tsent to splunk")
        except Exception as e:
            pass
        self.saveToMockSplunk(event)


    # def manageMessageQueue(self):
    #     """Continously try to send messages from messageQueue to splunk."""
    #     network_connection = False
    #     while True:
    #         # wait until messageQueue has an item -> then pop it
    #         # message: Message = self.messageQueue.get(block = True)
    #         # event = message.event
    #         event = self.messageQueue.get(block = True)
    #         self.saveToMockSplunk(event)
    #         sent = False
    #         while not sent:
    #             try:
    #                 status_code = self.sendToSplunk(event)
    #                 print(f"{status_code}\t{event['id']}\tsent to splunk")
    #                 network_connection = True
    #                 sent = True
    #             except Exception as e:
    #                 # if this is the first failure to send
    #                 if network_connection:
    #                     network_connection = False
    #                     network_error_event = {
    #                         "index": "pluto",
    #                         "event": {
    #                             "id": "startup",
    #                             "type": "message",
    #                             "devicename": gethostname(),
    #                             "time": str(datetime.now()),
    #                             "timestamp": self.get_seconds(), 
    #                             "level": "error",
    #                             "msg": f"Error sending to splunk: {e}",
    #                         }
    #                     }
    #                     self.saveToMockSplunk(network_error_event)
    

    # SUBSCRIBER CALLBACKS

    # ros logs
    def log_callback(self, msg: Log) -> None:
        """Callback for ROS log messages."""
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
                "id": "log",
                "type": "message",
                "devicename": gethostname(),
                "time": str(datetime.now()),
                "timestamp": msg.stamp.sec + msg.stamp.nanosec*(10**-9), 
                "level": levels[str(msg.level)], 
                "name": msg.name,
                "msg": msg.msg,
                "file": msg.file,
                "function": msg.function,
                "line": msg.line,
            },
        }
        self.pushMessage(event)

    # sensor data (raw)
    def gps_callback(self, msg: NavSatFix) -> None:
        """Callback for raw filtered GPS sensor data."""
        event = {
            "index": "pluto",
            "event": {
                "id": "gps",
                "type": "sensor",
                "devicename": gethostname(),
                "time": str(datetime.now()),
                "timestamp": self.get_seconds(), 
                "lat": msg.latitude, 
                "log": msg.longitude, 
                "alt": msg.altitude,
            },
        }
        self.pushMessage(event)
    def joy_callback(self, msg: Joy) -> None:
        """Callback for raw data received from Joystick (controller)."""
        event = {
            "index": "pluto",
            "event": {
                "id": "joystick",
                "type": "sensor",
                "devicename": gethostname(),
                "time": str(datetime.now()),
                "timestamp": self.get_seconds(), 
                "axes": str(msg.axes),
                "buttons": str(msg.buttons)
            },
        }
        self.pushMessage(event)

    # calculated data
    def odom_callback(self, msg: Odometry) -> None:
        """Callback for Odometry data (calculated from GPS and IMU sensor data)."""
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        l = msg.twist.twist.linear
        a = msg.twist.twist.angular
        # calculate roll, pitch, and yaw
        rpy = tf_transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
        roll_degree, pitch_degree, yaw_degree = (math.degrees(x) for x in rpy)
        yaw_degree_compass = (yaw_degree - 90) % 360
        event = {
            "index": "pluto",
            "event": {
                "id": "odometry",
                "type": "calculation",
                "devicename": gethostname(),
                "time": str(datetime.now()),
                "timestamp": self.get_seconds(), 
                "position": {
                    "x": p.x,
                    "y": p.y,
                    "z": p.z
                },
                "orientation": {
                    "x": o.x,
                    "y": o.y,
                    "z": o.z,
                    "w": o.w
                },
                "linear_twist": {
                    "x": l.x,
                    "y": l.y,
                    "z": l.z
                },
                "angular_twist": {
                    "x": a.x,
                    "y": a.y,
                    "z": a.z
                },
                "euler_angle": {
                    "roll": roll_degree,
                    "pitch": pitch_degree,
                    "yaw": yaw_degree,
                    "yaw_compass": yaw_degree_compass,
                }
            },
        }
        self.pushMessage(event)
    def line_callback(self, msg: LineMsg) -> None:
        """Callback for Line Detection data (calculated from camera image data)."""
        event = {
            "index": "pluto",
            "event": {
                "id": "line",
                "type": "calculation",
                "devicename": gethostname(),
                "time": str(datetime.now()),
                "timestamp": self.get_seconds(), 
                "point1": {
                    "x": msg.point1.x,
                    "y": msg.point1.y
                },
                "point2": {
                    "x": msg.point2.x,
                    "y": msg.point2.y
                },
                "angle": msg.angle
            },
        }
        self.pushMessage(event)
    # def ping_callback(self, msg: WaypointMsg) -> None:
    #     """Callback for ping_subscriber"""
    #     event = {
    #         "index": "pluto",
    #         "event": {
    #             "id": "waypoint_ping",
    #             "type": "ping",
    #             "devicename": gethostname(),
    #             "time": str(datetime.now()),
    #             "timestamp": self.get_seconds(), 
    #             "waypoint_number": msg.waypoint_number,
    #             "distance": msg.distance,
    #             "yaw": msg.yaw
    #         },
    #     }
    #     self.pushMessage(event)

    # analysis data
    def analysis_callback(self, msg: AnalysisMsg):
        """Callback for Analysis data from local_planner."""
        event = {
            "index": "pluto",
            "event": {
                "id": "local_planner",
                "type": "calculation",
                "devicename": gethostname(),
                "time": str(datetime.now()),
                "timestamp": msg.seconds,
                "state": msg.state.data,
                "heading": msg.heading,
                "goal_heading": msg.goal_heading,
                "local_position": {
                    "x": msg.local_position.x,
                    "y": msg.local_position.y
                },
                "goal_position": {
                    "x": msg.goal_position.x,
                    "y": msg.goal_position.y
                },
                "left_pwm": msg.left_pwm,
                "right_pwm": msg.right_pwm,
            }
        }
        self.pushMessage(event)


# MAIN
def main(args=None):
    rclpy.init(args=args)
    splunk_logger = SplunkLogger()
    executor = MultiThreadedExecutor(num_threads = 8)
    executor.add_node(splunk_logger)
    try:
            executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
            pass
    splunk_logger.destroy_node()
    rclpy.shutdown()


# When this file is run as a script
if __name__ == '__main__':
    main()