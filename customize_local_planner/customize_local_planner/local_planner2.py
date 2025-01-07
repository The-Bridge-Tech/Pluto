"""
Local Planner 2.0
Author: Matthew Lauriault
Created: 10/1/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import Bool, String, UInt32, Float32
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Pose, Point
from geodesy import utm
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from nav2_msgs.action import NavigateThroughPoses

# CALCULATION MODULES
import math
import time

# HELPER MODULES
from custom_msgs.msg import AnalysisMsg
from custom_msgs.srv import GPS
from .local_plan import LocalPlan
from .pwm import PWM
from .pending_data_log import PendingDataLog
from .conversions import *


class LocalPlanner(Node):

        def __init__(self):
                super().__init__("local_planner")

                # PARAMETERS
                # YAML File: pluto_launch/config/servos.yaml
                # PWM
                self.min_pwm = self.load_param_int("min_pwm")
                self.neutral_pwm = self.load_param_int("neutral_pwm")
                self.max_pwm = self.load_param_int("max_pwm")
                # YAML File: pluto_launch/config/local_planner.yaml
                # STATE: STRAIGHT
                self.straight_initial_pwm = self.load_param_double("straight_initial_pwm")
                self.straight_distance_tolerance = self.load_param_double("straight_distance_tolerance")
                self.straight_kp = self.load_param_double("straight_kp")
                self.straight_ki = self.load_param_double("straight_ki")
                self.straight_kd = self.load_param_double("straight_kd")
                # STATE: TURN
                self.turn_max_pwm = self.load_param_double("turn_max_pwm")
                self.turn_angle_tolerance = self.load_param_double("turn_angle_tolerance")
                self.turn_kp = self.load_param_double("turn_kp")
                self.turn_ki = self.load_param_double("turn_ki")
                self.turn_kd = self.load_param_double("turn_kd")
                # OTHER
                self.process_frequency = self.load_param_int("process_frequency")
                self.feedback_frequency = self.load_param_int("feedback_frequency")
                self.compensate_utm_error = self.load_param_bool("compensate_utm_error")
                # load parameter values from YAML file (pluto_launch/config/location.yaml)
                self.base_lat = self.load_param_double("base_lat")
                self.base_lon = self.load_param_double("base_lon")

                # CALLBACK GROUPS
                # Mutually Exclusive Callback Group
                #       * individual callbacks block themselves
                #       * callbacks within group block each other
                #       * groups execute in parallel with other groups
                self.local_plan_callback_group = MutuallyExclusiveCallbackGroup()
                self.process_callback_group = MutuallyExclusiveCallbackGroup()
                # Reentrant Callback Group
                #       * individual callbacks overlap themselves
                #       * callbacks within group execute in parallel
                #       * groups execute in parallel with other groups
                self.service_callback_group = ReentrantCallbackGroup()
                self.sub_callback_group = ReentrantCallbackGroup()

                # ACTION SERVER
                self.local_plan_action_server = ActionServer(
                        self,
                        NavigateThroughPoses,
                        "/local_plan",
                        self.local_plan_callback,
                        callback_group = self.local_plan_callback_group
                )
                self.local_plan = LocalPlan()

                # SERVICES
                self.origin_gps_service = self.create_service(
                        GPS,
                        "/origin_gps",
                        self.origin_gps_service_callback,
                        callback_group = self.service_callback_group
                )
                self.origin_gps: NavSatFix = None
                self.utm_error: Point = None

                # TIMERS
                self.process_timer = self.create_timer(
                        1 / self.process_frequency, 
                        self.process,
                        callback_group = self.process_callback_group
                )
                self.pending_data_log = PendingDataLog()

                # SUBSCRIBERS
                self.odom_sub = self.create_subscription(
                        Odometry, 
                        "/odometry/global", 
                        self.odom_callback, 
                        10,
                        callback_group = self.sub_callback_group
                )
                self.origin_odom: Odometry = None
                self.current_odom: Odometry = None
                self.gps_sub = self.create_subscription(
                        NavSatFix,
                        "/fix/filtered", 
                        self.gps_callback, 
                        10,
                        callback_group = self.sub_callback_group
                )
                self.is_autonomous_mode_sub = self.create_subscription(
                        Bool, 
                        "/is_autonomous_mode", 
                        self.is_autonomous_mode_callback, 
                        1,
                        callback_group = self.sub_callback_group
                )
                self.is_autonomous_mode = False

                # PUBLISHERS
                self.analysis_pub = self.create_publisher(
                        AnalysisMsg,
                        "/analysis/all",
                        10
                )
                self.state_pub = self.create_publisher(
                        String,
                        "/analysis/state",
                        10
                )
                self.conditions_pub = self.create_publisher(
                        String,
                        "/analysis/conditions",
                        10
                )
                self.subscribed_pub = self.create_publisher(
                        String,
                        "/analysis/subscribed",
                        10
                )
                self.position_pub = self.create_publisher(
                        utm.GeoPoint,
                        "/analysis/position",
                        10
                )

                # PWM CONTROLLERS
                self.left_pwm = PWM(
                        neutral = self.neutral_pwm,
                        min = self.min_pwm,
                        max = self.max_pwm,
                        value_pub = self.create_publisher(
                                UInt32, 
                                "/steering_left", 
                                10
                        ),
                        percent_pub = self.create_publisher(
                                Float32,
                                "/steering_left/percentage",
                                10
                        ),
                        logger = self.get_logger()
                )
                self.right_pwm = PWM(
                        neutral = self.neutral_pwm,
                        min = self.min_pwm,
                        max = self.max_pwm,
                        value_pub = self.create_publisher(
                                UInt32, 
                                "/steering_right", 
                                10
                        ),
                        percent_pub = self.create_publisher(
                                Float32,
                                "/steering_right/percentage",
                                10
                        ),
                        logger = self.get_logger()
                )

                # PID CONTROLLER VARIABLES
                self.reset_PID()

                # CONDITION VARIABLES
                self.heading = 0.0
                self.local_position = Point()
                self.angle_diff = 0.0
                self.distance_diff = 0.0

                # STATE MACHINE
                self.state = None


        # HELPERS - PARAMETERS

        def load_param(self, param_name: str, init_value):
                self.declare_parameter(param_name, init_value)
                return self.get_parameter(param_name).get_parameter_value()
        
        def load_param_int(self, param_name: str) -> int:
                return self.load_param(param_name, 0).integer_value
        
        def load_param_double(self, param_name: str) -> float:
                return self.load_param(param_name, 0.0).double_value
        
        def load_param_bool(self, param_name: str) -> bool:
                return self.load_param(param_name, False).bool_value
        

        # HELPERS - PID

        def reset_PID(self):
                self.prev_error = 0
                self.integral_error = 0
                self.prev_t = self.get_seconds()

        def get_seconds(self) -> float:
                return self.get_clock().now().nanoseconds * (10**-9)
        

        # HELPERS - POSITION

        def update_local_position(self):
                """Calculate current position (x, y) relative to local origin. 
                If parameter "compensate_utm_error" is set to True, this will compensate the UTM error."""
                # get position from current odometry reading
                current_position_reading = position_from_odom(self.current_odom)
                # get position from odometry reading at the local origin
                origin_position_reading = position_from_odom(self.origin_odom)
                # get UTM error
                utm_error = self.utm_error if self.compensate_utm_error else Point()
                # calculate/update local position
                self.local_position.x = (current_position_reading.x - origin_position_reading.x) - utm_error.x
                self.local_position.y = (current_position_reading.y - origin_position_reading.y) - utm_error.y
                # debugging info
                # self.get_logger().info(f"x = {round(current_position_reading.x, 3)} - {round(origin_position_reading.x, 3)} - {round(utm_error.x, 3)}   = {round(self.local_position.x, 3)}")
                # self.get_logger().info(f"y = {round(current_position_reading.y, 3)} - {round(origin_position_reading.y, 3)} - {round(utm_error.y, 3)}   = {round(self.local_position.y, 3)}")
                # publish global position (local position converted to global UTM coordinate)
                self.position_pub.publish(self.get_global_position().toMsg())

        def get_global_position(self) -> utm.UTMPoint:
                """Return local position converted to a global UTM coordinate."""
                origin_utm = utm.fromLatLong(self.origin_gps.latitude, self.origin_gps.longitude)
                global_origin = origin_utm.toPoint()
                global_position = utm.UTMPoint(
                        easting = global_origin.x + self.local_position.x,
                        northing = global_origin.y + self.local_position.y,
                        altitude = origin_utm.altitude,
                        zone = origin_utm.zone,
                        band = origin_utm.band
                )
                return global_position
        

        # SERVICE CALLBACKS

        def origin_gps_service_callback(self, request, response):
                """Serve request for origin gps message when autonomous mode is started for first time."""
                # log request
                self.get_logger().info(f"Incoming request for origin gps: {request}")
                # wait for gps subscriber callback to set origin gps
                while not self.origin_gps:
                        time.sleep(0.1)
                # log response
                response.data = self.origin_gps
                self.get_logger().info(f"Serving request for origin gps: ({response.data.latitude}, {response.data.longitude})")
                # return response
                return response


        # TIMER CALLBACKS
        
        def process(self):
                # first time callback is called -> set state to "Stop" and servos to neutral
                if not self.state:
                        self.stop()
                # construct log message for pending prerequisite data
                self.pending_data_log.clear()
                # wait for odometry data
                if not self.current_odom:
                        self.pending_data_log.add("odom data")
                # wait for origin odometry reading
                elif not self.origin_odom:
                        self.pending_data_log.add("origin odom")
                # wait for origin gps reading
                if not self.origin_gps:
                        self.pending_data_log.add("origin gps")
                # wait for first path
                if not self.local_plan.has_path():
                        self.pending_data_log.add("first path")
                # send log with pending prerequisite data and return
                if self.pending_data_log.has_pending_data():
                        self.get_logger().info(f"{self.pending_data_log}")
                        return
                # update current conditions
                self.update_conditions()
                # update current state
                self.update_state()
                # maintain current state
                self.maintain_state()


        # STATE MANAGEMENT

        def update_conditions(self):
                """Update conditions that will determine the next state."""
                # update current direction
                self.heading = angle_from_odom(self.current_odom)
                # update current position
                self.update_local_position()
                # if path hasn't been fully navigated yet -> there is still a goal pose
                if not self.local_plan.is_path_navigated():
                        # update current goal position
                        goal_position = self.local_plan.get_goal_position()
                        # calculate current angle difference (between current angle and goal angle)
                        self.goal_heading = math.degrees(math.atan2(
                                goal_position.y - self.local_position.y, 
                                goal_position.x - self.local_position.x
                        ))
                        self.angle_diff = self.goal_heading - self.heading
                        # calculate current distance between current position and goal position
                        self.distance_diff  = math.dist( 
                                [self.local_position.x, self.local_position.y], 
                                [goal_position.x, goal_position.y]
                        )
                self.conditions_pub.publish(String(data = f"[{self.get_seconds()}] angle_diff = {round(self.angle_diff, 3)}° distance = {round(self.distance_diff, 3)}m"))
                # publish all current analysis data
                self.analysis_pub.publish(AnalysisMsg(
                        # context
                        seconds = self.get_seconds(),
                        state = String(data = self.state),
                        # feedback input - heading
                        heading = self.heading,
                        goal_heading = self.goal_heading,
                        # feedback input - position
                        local_position = self.local_position,
                        goal_position = goal_position,
                        # control output
                        left_pwm = self.left_pwm.percentage,
                        right_pwm = self.right_pwm.percentage
                ))

        def update_state(self):
                """Update state based on current conditions."""
                if self.state == "Stop":
                        # wait for autonomous mode to be True
                        if not self.is_autonomous_mode:
                                self.get_logger().info("Stopped. Waiting for autonomous mode.")
                                return
                        # path has been fully navigated -> wait for new goal
                        if self.local_plan.is_path_navigated():
                                self.get_logger().info("Stopped. Waiting for new path")
                                return
                        # path has poses to navigate -> Turn
                        else:
                                self.get_logger().info("Navigating to current goal pose.")
                                self.turn()
                elif self.state == "Turn":
                        # If angle difference is within tolerance -> Straight
                        if abs(self.angle_diff) < self.turn_angle_tolerance:
                                self.straight()
                        else:
                                pass # self.get_logger().info(f"angle_diff = {round(self.angle_diff, 3)}°")
                elif self.state == "Straight":
                        # If within distance tolerance of goal position -> Stop
                        if self.distance_diff < self.straight_distance_tolerance:
                                self.get_logger().info("Reached goal pose.")
                                self.local_plan.complete_goal_pose()
                                self.stop()
                        else:
                                pass # self.get_logger().info(f"angle_diff = {round(self.angle_diff, 3)}° distance = {round(self.distance_diff, 3)}m")
                else:
                        self.get_logger().error(f"Invalid state: '{self.state}'.")
                        self.stop() # default


        def maintain_state(self):
                """Execute the function of the current state."""
                if self.state == "Stop":
                        pass # no maintenance needed
                elif self.state == "Turn":
                        self.maintain_turn()
                elif self.state == "Straight":
                        self.maintain_straight()
                else:
                        self.get_logger().error(f"Invalid state: '{self.state}'.")
                

        # STATE INITIATION

        def set_state(self, state: str):
                """Set next state."""
                # log state change
                self.get_logger().info(f"State: {self.state} -> {state}")
                # publish state change
                self.state_pub.publish(String(data=f"[{self.get_seconds()}] {self.state} -> {state}"))
                # actually change the state
                self.state = state

        def stop(self):
                """Set left and right pwm values to neutral."""
                self.set_state("Stop")
                self.left_pwm.set_neutral()
                self.right_pwm.set_neutral()

        def turn(self):
                """Start turning in place towards the next waypoint."""
                self.set_state("Turn")
                # reset PID variables
                self.reset_PID()
                # set initial pwm's
                self.left_pwm.set_neutral()
                self.right_pwm.set_neutral()

        def straight(self):
                """Start moving straight towards the next waypoint."""
                self.set_state("Straight")
                # reset PID variables
                self.reset_PID()
                # set initial pwm's
                self.left_pwm.percentage = self.straight_initial_pwm
                self.right_pwm.percentage = self.straight_initial_pwm


        # STATE MAINTENANCE

        def maintain_turn(self):
                """Adjust left and right servo pwm's from neutral using PID controller
                to correct the mower's direction in place (no linear movement)."""
                # update PID controller error terms
                t = self.get_seconds()
                dt = t - self.prev_t
                error = self.angle_diff
                self.integral_error += error * dt
                derivative_error = ((error - self.prev_error) / dt) if dt > 0 else 0
                # calculate PID error correction
                correction = (
                        # P = Proportional error (current)
                        self.turn_kp * error +
                        # I = Integral error (past)
                        self.turn_ki * self.integral_error +
                        # D = Derivative error (future)
                        self.turn_kd * derivative_error
                )
                # limit correction to prevent instability
                correction = max(
                        min(correction, self.turn_max_pwm), 
                        -self.turn_max_pwm
                )
                # self.get_logger().info(f"error: {error}\t P: {self.turn_kp * error} I: {self.turn_ki * self.integral_error} D: {self.turn_kd * derivative_error}")
                # update PID previous values
                self.prev_error = error
                self.prev_time = t
                # apply PID error correction
                self.left_pwm.percentage = -correction
                self.right_pwm.percentage = correction

        def maintain_straight(self):
                """Adjust right servo pwm from initial straight pwm using PID controller 
                to correct the mower's direction (maintaining linear movement)."""
                # update PID controller error terms
                t = self.get_seconds()
                dt = t - self.prev_t
                error = self.angle_diff
                self.integral_error += error * dt
                derivative_error = ((error - self.prev_error) / dt) if dt > 0 else 0
                # calculate PID error correction
                correction = (
                        # P = Proportional error (current)
                        self.straight_kp * error +
                        # I = Integral error (past)
                        self.straight_ki * self.integral_error +
                        # D = Derivative error (future)
                        self.straight_kd * derivative_error
                )
                # self.get_logger().info(f"error: {error}\t P: {self.straight_kp * error} I: {self.straight_ki * self.integral_error} D: {self.straight_kd * derivative_error}")
                # update PID previous values
                self.prev_error = error
                self.prev_time = t
                # apply PID error correction
                self.right_pwm.percentage = self.straight_initial_pwm + correction
                        

        # SUBSCRIBER CALLBACKS

        def odom_callback(self, msg: Odometry):
                """Get odometry data for UTM and angle reading."""
                # wait for autonomous mode to start the first time -> get origin UTM coordinate (relative to where nodes were launched)
                if self.is_autonomous_mode and not self.origin_odom:
                        self.origin_odom = msg
                # get current UTM coordinate (relative to where nodes were launched)
                self.current_odom = msg
                self.subscribed_pub.publish(String(data = f"[{self.get_seconds()}] odom: x = {msg.pose.pose.position.x} y = {msg.pose.pose.position.y}"))

        def gps_callback(self, msg: NavSatFix):
                """Get origin lat & lon when autonomous mode is started for the first time."""
                # Wait for autonomous mode to start the first time -> get origin gps -> calculate UTM error
                if self.is_autonomous_mode and not self.origin_gps:
                        self.origin_gps = msg
                        # convert lat & lon reading at base pin to a UTM coodinate -> then to a point
                        reading_base_utm = utm.fromLatLong(msg.latitude, msg.longitude).toPoint()
                        # convert actual lat & lon at base pin to a UTM coordinate -> then to a point
                        actual_base_utm = utm.fromLatLong(self.base_lat, self.base_lon).toPoint()
                        # calculate UTM error
                        self.utm_error = Point(
                                x = reading_base_utm.x - actual_base_utm.x,
                                y = reading_base_utm.y - actual_base_utm.y
                        )
                        # log the subscription and calculation
                        self.subscribed_pub.publish(String(data = f"[{self.get_seconds()}] gps: lat = {msg.latitude} lon = {msg.longitude}"))
                        self.get_logger().info(f"UTM error: {(round(self.utm_error.x, 3), round(self.utm_error.y, 3))}")

        def is_autonomous_mode_callback(self, msg: Bool):
                # force into stop state
                self.stop()
                # if autonomous to manual -> reset local plan
                if self.is_autonomous_mode and not msg.data:
                        self.local_plan = LocalPlan()
                self.is_autonomous_mode = msg.data
                self.subscribed_pub.publish(String(data = f"[{self.get_seconds()}] is_autonomous_mode: {msg.data}"))

        
        # ACTION CALLBACKS
        
        def local_plan_callback(self, goal_handle: ServerGoalHandle):
                """Executes accepted goal sent by action client (path planner node)."""
                # GOAL
                goal: NavigateThroughPoses.Goal = goal_handle.request
                # geometry_msgs/PoseStamped[] poses
                path = goal.poses
                # string behavior_tree
                behavior_tree = goal.behavior_tree
                # if path is empty -> cancel goal
                if len(path) == 0:
                        self.get_logger().warn("empty path from /local_plan. Cancelling goal...")
                        # cancel goal
                        goal_handle.canceled()
                # path has poses -> execute goal
                else:
                        # if first path
                        if not self.local_plan.has_path():
                                self.get_logger().info("first path from /local_plan. Accepting goal...")
                        # if new path
                        else:
                                self.get_logger().info("new path from /local_plan. Accepting goal...")
                        # set path as the new local plan
                        self.local_plan.set_path(path)
                # FEEDBACK
                # return feedback until path has been navigated
                while not self.local_plan.is_path_navigated():
                        # construct feedback message
                        feedback_msg = NavigateThroughPoses.Feedback(
                                # geometry_msgs/PoseStamped current_pose
                                current_pose = PoseStamped(pose = Pose(position = self.local_position)),
                                # TODO builtin_interfaces/Duration navigation_time
                                navigation_time = Duration(
                                        sec = 0,
                                        nanosec = 0
                                ),
                                # TODO builtin_interfaces/Duration estimated_time_remaining
                                estimated_time_remaining = Duration(
                                        sec = 0,
                                        nanosec = 0
                                ),
                                # TODO int16 number_of_recoveries
                                number_of_recoveries = 0,
                                # float32 distance_remaining
                                distance_remaining = self.distance_diff,
                                # int16 number_of_poses_remaining
                                number_of_poses_remaining = len(self.local_plan.future_poses)
                        )
                        # publish feedback message
                        goal_handle.publish_feedback(feedback_msg)
                        # loop delay
                        time.sleep(1 / self.feedback_frequency)
                # RESULT
                # return result that path has been navigated OR fatal error occured
                goal_handle.succeed()
                # construct result
                result = NavigateThroughPoses.Result(
                        # TODO uint16 error_code
                        error_code = 0
                )
                # return result
                return result

# MAIN

def main(args=None):
        rclpy.init(args=args)
        local_planner = LocalPlanner()
        executor = MultiThreadedExecutor(num_threads = 8)
        executor.add_node(local_planner)
        try:
                executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
                pass
        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()