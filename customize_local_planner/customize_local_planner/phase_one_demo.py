"""
Publishes path with goal poses to local_planner node
Author: Matthew Lauriault
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import Future, ClientGoalHandle
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav2_msgs.action import NavigateThroughPoses
from custom_msgs.msg import WaypointMsg

# HELPER MODULES
from .conversions import *

# CONSTANTS
BASE_GPS = (34.841400, -82.411743)
WAYPOINTS = [
    (34.841384, -82.411669),    # front-left corner
    (34.841254, -82.411731),    # back-left corner
    (34.841327, -82.411853),    # back-right corner
    (34.841434, -82.411776),    # front-right corner
    (34.841384, -82.411669),    # front-left corner (return to #1)
]
WAYPOINT_RADIUS = 1.0           # should match local_planner's 'distance_error_tolerance' parameter


class PhaseOneDemo(Node):

    def __init__(self):
        super().__init__('PhaseOneDemo')

        # ACTION CLIENT
        self.local_plan_action_client = ActionClient(
            self,
            NavigateThroughPoses,
            "/local_plan",
        )

        # PUBLISHERS
        self.ping_publisher = self.create_publisher(
            WaypointMsg,
            "/waypoint_ping",
            10
        )

        # SUBSCRIBERS
        self.is_autonomous_mode_sub = self.create_subscription(
            Bool, 
            "is_autonomous_mode",
            self.is_autonomous_mode_callback,
            1
        )
        self.is_autonomous_mode = False

        # TIMERS
        self.ready_to_ping = False
        ping_timer_period = 1 # seconds
        # self.ping_timer = self.create_timer(
        #     ping_timer_period,
        #     self.publish_waypoint_ping
        # )

        # Initialize goal poses
        self.reset()

    
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


    # HELPERS

    def absolute_to_relative_utm(self, x: float, y: float) -> tuple:
        """Converts absolute UTM coordinate (x = easting, y = northing)
        to one relative to the known origin UTM (base pin)"""
        base_x, base_y = lat_lon_to_utm(*BASE_GPS)
        return (x - base_x, y - base_y)

    def lat_lon_to_relative_utm(self, lat: float, lon: float) -> tuple:
        """Convert lat & lon to UTM coordinate relative to origin (base pin)"""
        # convert lat & lon to absolute UTM
        abs_goal_utm = lat_lon_to_utm(lat, lon)
        # return UTM relative to known origin UTM (base pin)
        return self.absolute_to_relative_utm(*abs_goal_utm)
    
    def reset(self):
        """Reset goal poses and current goal pose."""
        self.goal_poses = []
        self.get_logger().info("Reset.")


    # PHASE 1 METHODS

    def send_waypoint_path_goal(self):
        """Phase 1 method: send path with hardcoded waypoints."""
        # calculate relative UTM coordinates of waypoints (origin at the base pin)
        rel_goal_utms = [self.lat_lon_to_relative_utm(*waypoint) for waypoint in WAYPOINTS]
        # construct pose messages with relative UTM coordinates
        goal_poses = [
            PoseStamped(pose = Pose(position = Point(
                x = rel_goal_utm[0], 
                y = rel_goal_utm[1],
                z = 0.0
            )))
            for rel_goal_utm in rel_goal_utms
        ]
        self.get_logger().info("Calculated goal poses.")
        # debugging info
        for i, goal_pose in enumerate(rel_goal_utms):
            self.get_logger().info(f"{i+1}: {goal_pose}")
        # construct action goal message
        goal_msg = NavigateThroughPoses.Goal(
            poses = goal_poses,
            behavior_tree = "Not Implemented"
        )
        # send goal message to local_planner's action server
        self.send_local_plan_goal(goal_msg)

        
    # TIMER CALLBACKS

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

            # PHASE 1
            self.send_waypoint_path_goal()

        # if autonomous to manual -> reset
        if self.is_autonomous_mode and not msg.data:
            self.reset()
            self.ready_to_ping = False
        self.is_autonomous_mode = msg.data


# MAIN

def main(args=None):
    rclpy.init(args=args)
    phase_one_demo = PhaseOneDemo()

    rclpy.spin(phase_one_demo)

    phase_one_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
