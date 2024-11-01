"""
Local Planner 2.0
Author: Matthew Lauriault
Created: 10/1/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from std_msgs.msg import Bool, String, Int16, UInt16, UInt32, Float32
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses

# CALCULATION MODULES
import math
import time

# HELPER MODULES
from .local_plan import LocalPlan
from .pwm import PWM
from .conversions import *


class LocalPlanner(Node):

        def __init__(self):
                super().__init__("local_planner")

                # PARAMETERS
                # load parameter values from YAML file (pluto_launch/config/local_planner.yaml)
                # PWM
                self.min_pwm = self.load_param_int("min_pwm")
                self.neutral_pwm = self.load_param_int("neutral_pwm")
                self.max_pwm = self.load_param_int("max_pwm")
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

                # TIMERS
                self.process_timer = self.create_timer(
                        1 / self.process_frequency, 
                        self.process
                )

                # ACTION SERVER
                self.local_plan_action_server = ActionServer(
                        self,
                        NavigateThroughPoses,
                        "/local_plan", 
                        self.local_plan_callback, 
                )
                self.local_plan = LocalPlan()

                # PUBLISHERS
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

                # SUBSCRIBERS
                self.odom_sub = self.create_subscription(
                        Odometry, 
                        "/odometry/global", 
                        self.odom_callback, 
                        10
                )
                self.current_odom: Odometry = None
                self.is_autonomous_mode_sub = self.create_subscription(
                        Bool, 
                        "/is_autonomous_mode", 
                        self.is_autonomous_mode_callback, 
                        1
                )
                self.is_autonomous_mode = False


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
                self.heading = None
                self.current_x = None
                self.current_y = None
                self.angle_diff = None
                self.distance_diff = None

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
        

        # HELPERS

        def reset_PID(self):
                self.prev_error = 0
                self.integral_error = 0
                self.prev_t = time.time()

        def get_seconds(self) -> float:
                return self.get_clock().now().nanoseconds * (10**-9)
        

        # TIMER CALLBACKS
        
        def process(self):
                # first time callback is called -> set state to "Stop" and servos to neutral
                if not self.state:
                        self.stop()
                # wait for odometry data
                if not self.current_odom:
                        self.get_logger().info("Waiting for odometry from /odometry/global")
                        return
                # wait for first path
                if not self.local_plan.has_path():
                        self.get_logger().info("Waiting for first path from /local_plan")
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
                self.heading = angle_from_odometry(self.current_odom)
                # update current position
                self.current_x = self.current_odom.pose.pose.position.x
                self.current_y = self.current_odom.pose.pose.position.y
                # if path hasn't been fully navigated yet -> there is still a goal pose
                if not self.local_plan.is_path_navigated():
                        # update current goal position
                        goal_x, goal_y = self.local_plan.get_goal_xy()
                        # calculate current angle difference (between current angle and goal angle)
                        self.goal_heading = math.degrees(math.atan2(
                                goal_y - self.current_y, 
                                goal_x - self.current_x
                        ))
                        self.angle_diff = self.goal_heading - self.heading
                        # calculate current distance between current position and goal position
                        self.distance_diff  = math.dist( 
                                [self.current_x, self.current_y], 
                                [goal_x, goal_y]
                        )
                self.conditions_pub.publish(String(data = f"[{self.get_seconds()}] goal_angle = {round(self.goal_heading, 3)} angle_diff = {round(self.angle_diff, 3)} distance = {round(self.distance_diff, 3)}"))

        def update_state(self):
                """Update state based on current conditions."""
                if self.state == "Stop":
                        # wait for autonomous mode to be True
                        if not self.is_autonomous_mode:
                                self.get_logger().info("Waiting for autonomous mode.")
                                return
                        # path has been fully navigated -> wait for new path
                        if self.local_plan.is_path_navigated():
                                self.get_logger().info("Waiting for new path from /local_plan")
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
                                self.get_logger().info(f"angle_diff = {self.angle_diff}")
                elif self.state == "Straight":
                        # If within distance tolerance of goal position -> Stop
                        if self.distance_diff < self.straight_distance_tolerance:
                                self.get_logger().info("Reached goal pose.")
                                self.local_plan.complete_goal_pose()
                                self.stop()
                        else:
                                # self.get_logger().info(f"distance_diff = {self.distance_diff}")
                                # self.get_logger().info(f"current = {(self.current_x, self.current_y)} \tgoal = {self.local_plan.get_goal_xy()}")
                                self.get_logger().info(f"current = {self.heading}° \tgoal = {self.goal_heading}°")
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
                t = time.time()
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
                self.get_logger().info(f"error: {error}\t P: {self.turn_kp * error} I: {self.turn_ki * self.integral_error} D: {self.turn_kd * derivative_error}")
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
                t = time.time()
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
                self.current_odom = msg
                self.subscribed_pub.publish(String(data = f"[{self.get_seconds()}] odom: x = {msg.pose.pose.position.x} y = {msg.pose.pose.position.y}"))

        def is_autonomous_mode_callback(self, msg: Bool):
                # force into stop state
                self.stop()
                # if autonomous to manual -> reset local plan
                if self.is_autonomous_mode and not msg.data:
                        self.local_plan = LocalPlan()
                self.is_autonomous_mode = msg.data
                self.subscribed_pub.publish(String(data = f"[{self.get_seconds()}] is_autonomous_mode: {msg.data}"))

        def local_plan_callback(self, goal_handle: ServerGoalHandle):
                """Executes accepted goal from action server."""
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
                                self.get_logger().info("first path from /local_plan")
                        # if new path
                        else:
                                self.get_logger().info("new path from /local_plan")
                        # set path as the new local plan
                        self.local_plan.set_path(path)
                        # self.subscribed_pub.publish(String(data = f"[{self.get_seconds()}] local_plan: {[(pose.position.x, pose.position.y) for pose in path]}"))
                # FEEDBACK
                # return feedback until path has been navigated
                while not self.local_plan.is_path_navigated():
                        # construct feedback message
                        feedback_msg = NavigateThroughPoses.Feedback(
                                # geometry_msgs/PoseStamped current_pose
                                current_pose = PoseStamped(pose = Pose(position = Point(
                                        x = self.current_x,
                                        y = self.current_y
                                ))),
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
                                number_of_recoveries = Int16(
                                        data = 0
                                ),
                                # float32 distance_remaining
                                distance_remaining = Float32(
                                        data = self.distance_diff
                                ),
                                # int16 number_of_poses_remaining
                                number_of_poses_remaining = Int16(
                                        data = len(self.local_plan.future_poses)
                                )
                        )
                        # publish feedback message
                        goal_handle.publish_feedback(feedback_msg)
                        time.sleep(1)
                # RESULT
                # return result that path has been navigated OR fatal error occured
                goal_handle.succeed()
                # construct result
                result = NavigateThroughPoses.Result(
                        # TODO uint16 error_code
                        error_code = UInt16(
                                data = 0
                        )
                )
                # return result
                return result


# MAIN

def main(args=None):
        rclpy.init(args=args)

        local_planner = LocalPlanner()

        rclpy.spin(local_planner)

        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()