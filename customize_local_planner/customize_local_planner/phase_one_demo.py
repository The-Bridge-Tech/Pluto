"""
Publishes path with goal poses to local_planner node
Author: Matthew Lauriault
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionClient
from rclpy.action.client import Future, ClientGoalHandle
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Point
from geodesy import utm
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateThroughPoses

# HELPER MODULES
from custom_msgs.msg import WaypointMsg
from custom_msgs.srv import GPS
from .conversions import *

# CONSTANTS
WAYPOINTS = [
    (34.841384, -82.411669),    # front-left corner
    (34.841254, -82.411731),    # back-left corner
    (34.841327, -82.411853),    # back-right corner
    (34.841434, -82.411776),    # front-right corner
    (34.841384, -82.411669),    # front-left corner (return to #1)
]


class PhaseOneDemo(Node):

    def __init__(self):
        super().__init__('phase_one_demo')

        # PARAMETERS
        # YAML File: pluto_launch/config/location.yaml
        self.base_lat = self.load_param_double("base_lat")
        self.base_lon = self.load_param_double("base_lon")

        # CALLBACK GROUPS
        # Mutually Exclusive Callback Group
        #       * individual callbacks block themselves
        #       * callbacks within group block each other
        #       * groups execute in parallel with other groups
        self.local_plan_callback_group = MutuallyExclusiveCallbackGroup()
        self.initial_gps_callback_group = MutuallyExclusiveCallbackGroup()
        self.process_callback_group = MutuallyExclusiveCallbackGroup()
        # Reentrant Callback Group
        #       * individual callbacks overlap themselves
        #       * callbacks within group execute in parallel
        #       * groups execute in parallel with other groups
        self.sub_callback_group = ReentrantCallbackGroup()

        # ACTION CLIENT
        self.local_plan_action_client = ActionClient(
            self,
            NavigateThroughPoses,
            "/local_plan",
            callback_group = self.local_plan_callback_group
        )

        # CLIENTS
        self.initial_gps_client = self.create_client(
            GPS,
            "/initial_gps",
            callback_group = self.initial_gps_callback_group
        )
        self.local_origin: Point = None

        # SUBSCRIBERS
        self.is_autonomous_mode_sub = self.create_subscription(
            Bool, 
            "is_autonomous_mode",
            self.is_autonomous_mode_callback,
            1,
            callback_group = self.sub_callback_group
        )
        self.is_autonomous_mode = False

        # PUBLISHERS
        self.ping_publisher = self.create_publisher(
            WaypointMsg,
            "/waypoint_ping",
            10
        )

        # TIMERS
        self.process_timer = self.create_timer(
                1 / 10, 
                self.process,
                callback_group = self.process_callback_group
        )
        # ping_timer_period = 1 # seconds
        # self.ping_timer = self.create_timer(
        #     ping_timer_period,
        #     self.publish_waypoint_ping
        # )
        self.ready_to_ping = False

        # Initialize goal poses list
        self.reset()


    # HELPERS - PARAMETERS

    def load_param(self, param_name: str, init_value):
        self.declare_parameter(param_name, init_value)
        return self.get_parameter(param_name).get_parameter_value()
    
    def load_param_double(self, param_name: str) -> float:
        return self.load_param(param_name, 0.0).double_value

    
    # ACTION CLIENT

    def send_local_plan_goal(self, goal_msg: NavigateThroughPoses.Goal):
        """Send action 'goal service' message (with path to navigate) to local_planner's action server."""
        # wait for local_planner's action server
        self.local_plan_action_client.wait_for_server()
        # send goal asynchronously (will send when action server is ready to receive it)
        self.goal_future = self.local_plan_action_client.send_goal_async(
            goal_msg,
            feedback_callback = self.local_plan_feedback_callback
        )
        # set callback for when response is returned
        self.goal_future.add_done_callback(self.local_plan_goal_response_callback)


    def local_plan_goal_response_callback(self, future: Future):
        """Receive response from local_planner's action server after sending it a goal."""
        # See how the goal was received by local_planner
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by local_planner")
            return
        self.get_logger().info("Goal accepted by local_planner")
        # request the goal result asynchronously (will receive when action server is ready to send it)
        self.result_future: Future = goal_handle.get_result_async()
        # set callback for when result is returned (after goal has either been completed or failed)
        self.result_future.add_done_callback(self.local_plan_result_callback)
        

    def local_plan_feedback_callback(self, feedback_msg: NavigateThroughPoses.Impl.FeedbackMessage):
        """Receive feedback from local_planner's action server as it completes a goal."""
        # Extract data from feedback message
        feedback = feedback_msg.feedback
        # geometry_msgs/PoseStamped current_pose
        current_pose = feedback.current_pose
        x, y = current_pose.pose.position.x, current_pose.pose.position.y
        # TODO builtin_interfaces/Duration navigation_time
        navigation_time = feedback.navigation_time
        # TODO builtin_interfaces/Duration estimated_time_remaining
        estimated_time_remaining = feedback.estimated_time_remaining
        # TODO int16 number_of_recoveries
        number_of_recoveries = feedback.number_of_recoveries
        # float32 distance_remaining
        distance_remaining = feedback.distance_remaining
        # int16 number_of_poses_remaining
        number_of_poses_remaining = feedback.number_of_poses_remaining
        # Log the feedback data
        self.get_logger().info(f"at {(round(x, 3), round(y, 3))}\t {round(distance_remaining, 3)}m remaining\t {number_of_poses_remaining} poses left")


    def local_plan_result_callback(self, future: Future):
        """Receive response from local_planner's action server after it completed or failed a goal."""
        # get result from asynchronous request (future)
        result: NavigateThroughPoses.Result = future.result().result
        # get error code from result
        error_code = result.error_code
        self.get_logger().info(f"Result from local_planner: {error_code}")
        # interpret / respond to error code
        if error_code == 0:
            self.get_logger().info("Goal completed successfully!")
            # PHASE ONE -> STOP
            # PHASE TWO -> SEND NEXT GOAL PATH
        else:
            self.get_logger().error("Goal failed :(")
            # STOP


    # CLIENT REQUESTS

    def request_initial_gps(self):
            """Request and Return the initial gps message after autonomous mode was started for the first time."""
            # wait for service to be available
            while not self.initial_gps_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for '/initial_gps' service to be available")
            # send request asynchronously
            request = GPS.Request()
            self.get_logger().info("Requesting '/initial_gps' service")
            future = self.initial_gps_client.call_async(request)
            # set callback for when response is returned
            future.add_done_callback(self.initial_gps_response_callback)
            
    def initial_gps_response_callback(self, future: Future):
        """Calculate local origin using initial gps from response."""
        # get response
        response: GPS.Response = future.result()
        initial_gps: NavSatFix = response.data
        self.get_logger().info(f"Received '/initial_gps' response: ({initial_gps.latitude}, {initial_gps.longitude})")
        # calculate local origin
        self.local_origin = utm.fromLatLong(initial_gps.latitude, initial_gps.longitude).toPoint()

    

    # HELPERS

    def lat_lon_to_local_point(self, lat: float, lon: float) -> Point:
        """Converts latitude & longitude to a point (x, y) relative to local origin (base pin)"""
        # convert lat & lon to UTM coordinates (easting, northing) and then to points (x, y)
        # base_point = utm.fromLatLong(self.base_lat, self.base_lon).toPoint()
        goal_point = utm.fromLatLong(lat, lon).toPoint()
        # local = goal - base
        local_point = Point(
            x = goal_point.x - self.local_origin.x,
            y = goal_point.y - self.local_origin.y,
            z = goal_point.z - self.local_origin.z
        )
        return local_point
    
    def reset(self):
        """Reset goal poses and current goal pose."""
        self.goal_poses = []
        self.get_logger().info("Reset.")


    # PHASE 1 METHODS

    def send_waypoint_path_goal(self):
        """Phase 1 method: send path with hardcoded waypoints."""
        # convert waypoint lat & lon's to local points (origin at the base pin)
        goal_points = [
            self.lat_lon_to_local_point(*waypoint) 
            for waypoint in WAYPOINTS
        ]
        # construct pose messages with relative UTM coordinates
        goal_poses = [
            PoseStamped(pose = Pose(position = point)) 
            for point in goal_points
        ]
        self.get_logger().info("Calculated goal poses.")
        # debugging info
        for i, point in enumerate(goal_points):
            self.get_logger().info(f"{i+1}: {(round(point.x, 3), round(point.y, 3), round(point.z, 3))}")
        # construct action goal message
        goal_msg = NavigateThroughPoses.Goal(
            poses = goal_poses,
            behavior_tree = "Not Implemented"
        )
        # send goal message to local_planner's action server
        self.send_local_plan_goal(goal_msg)

        
    # TIMER CALLBACKS

    def process(self):
        if not self.local_origin:
            return
        self.send_waypoint_path_goal()
        # destroy timer (phase 1)
        self.process_timer.cancel()
        self.get_logger().info("cancelled process timer")

    # def publish_waypoint_ping(self):
    #     """Publish waypoint info to splunk_logger node and gps_plotter node."""
    #     # wait until publish_local_plan() has created the data to publish
    #     if self.ready_to_ping:
    #         roll, pitch, yaw = quaternion_to_euler(self.current_odom.pose.pose.orientation)
    #         yaw_degrees = math.degrees(yaw)
    #         ping = WaypointMsg(
    #             waypoint_number = self.pose_i+1,
    #             distance = self.distance_from_goal,
    #             yaw = yaw_degrees # orientation around the vertical axis
    #         )
    #         # log so that you can see realtime messages on monitor in phaseOne terminal
    #         lat = self.initial_gps.latitude
    #         long = self.initial_gps.longitude
    #         lat_str = f"{int(lat)}° {((lat - int(lat)) * 60)}"
    #         long_str = f"{int(long)}° {((long - int(long)) * 60)}"
    #         # self.get_logger().info(f"GPS: {(lat_str, long_str)}\tHeading: {yaw_degrees}°\tDistance: {self.distance_from_goal}\tWaypoint #{self.pose_i+1}")
    #         self.ping_publisher.publish(ping)
    

    # SUBSCRIBER CALLBACKS

    def is_autonomous_mode_callback(self, msg: Bool):
        """Update if in autonomous mode."""
        # if manual to autonomous -> send goal
        if not self.is_autonomous_mode and msg.data:
            # if local origin hasn't been calculated yet
            if not self.local_origin:
                # request initial gps from local_planner
                self.request_initial_gps()
                # self.local_origin = utm.fromLatLong(self.base_lat, self.base_lon).toPoint()
        # if autonomous to manual -> reset
        elif self.is_autonomous_mode and not msg.data:
            self.reset()
            self.ready_to_ping = False
        self.is_autonomous_mode = msg.data


# MAIN

def main(args=None):
    rclpy.init(args=args)
    phase_one_demo = PhaseOneDemo()
    executor = MultiThreadedExecutor(num_threads = 8)
    executor.add_node(phase_one_demo)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    phase_one_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# 2. try timer thing