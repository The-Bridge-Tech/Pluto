"""
Local Planner 2.0
Author: Matthew Lauriault
Created: 10/1/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, Bool, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

# CALCULATION MODULES
import math

# HELPER MODULES
from .pwm import PWM
from .untilit import *


# PARAMETERS (must be declared even when using a YAML file)
DEFAULT_PARAMS = {
        # PWM
        "min_pwm": 992,
        "neutral_pwm": 1376,
        "max_pwm": 1765,

        # STATE: STRAIGHT
        "straight_initial_pwm": 30.0, # percent
        "straight_adjustment_pwm": 2.0, # percent
        "straight_angle_tolerance": 5.0, # degrees - will only correct angle difference if outside this tolerance
        "straight_distance_tolerance": 1.0, # meters - will stop once within this distance of the waypoint

        # STATE: TURN
        "turn_initial_pwm": 10.0, # percent
        "turn_angle_tolerance": 5.0, # will begin straight state once angle difference is within this tolerance

        # OTHER
        "process_frequency": 10, # Hz (times / second)
}


class LocalPlanner(Node):

        def __init__(self):
                super().__init__("local_planner", allow_undeclared_parameters=True)

                # PARAMETERS
                # declare all parameters with default values
                for name, value in DEFAULT_PARAMS.items():
                        self.declare_parameter(name, value)
                # load parameter values from YAML file (pluto_launch/config/local_planner.yaml)
                load_param = lambda param_name: self.get_parameter(param_name).get_parameter_value()
                # PWM
                self.min_pwm = load_param("min_pwm").integer_value
                self.neutral_pwm = load_param("neutral_pwm").integer_value
                self.max_pwm = load_param("max_pwm").integer_value
                # STATE: STRAIGHT
                self.straight_initial_pwm = load_param("straight_initial_pwm").double_value
                self.straight_adjustment_pwm = load_param("straight_adjustment_pwm").double_value
                self.straight_angle_tolerance = load_param("straight_angle_tolerance").double_value
                self.straight_distance_tolerance = load_param("straight_distance_tolerance").double_value
                # STATE: TURN
                self.turn_initial_pwm = load_param("turn_initial_pwm").double_value
                self.turn_angle_tolerance = load_param("turn_angle_tolerance").double_value
                # OTHER
                self.process_frequency = load_param("process_frequency").integer_value

                # TIMERS
                self.process_timer = self.create_timer(
                        1 / self.process_frequency, 
                        self.process
                )

                # PUBLISHERS
                self.state_publisher = self.create_publisher(
                        String,
                        "/state",
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
                        "is_autonomous_mode", 
                        self.is_autonomous_mode_callback, 
                        10
                )
                self.is_autonomous_mode = False
                self.local_plan = self.create_subscription(
                        Path, 
                        "local_plan", 
                        self.local_plan_callback, 
                        10
                )
                self.goal_pose: PoseStamped = None

                # PWM CONTROLLERS
                self.left_pwm = PWM(
                        neutral = self.neutral_pwm,
                        min = self.min_pwm,
                        max = self.max_pwm,
                        publisher = self.create_publisher(
                                UInt32, 
                                "/steering_left", 
                                10
                        ),
                        logger = self.get_logger()
                )
                self.right_pwm = PWM(
                        neutral = self.neutral_pwm,
                        min = self.min_pwm,
                        max = self.max_pwm,
                        publisher = self.create_publisher(
                                UInt32, 
                                "/steering_right", 
                                10
                        ),
                        logger = self.get_logger()
                )

                # CONDITION VARIABLES
                self.heading = None
                self.current_x = None
                self.current_y = None
                self.goal_x = None
                self.goal_y = None
                self.angle_diff = None
                self.distance_diff = None

                # STATE MACHINE
                self.state = None
                # set state to "Stop" and servos to neutral
                self.stop()
        

        # TIMER CALLBACKS
        
        def process(self):
                # wait until /odometry/global and /local_plan have been subscribed
                if self.current_odom is None or self.goal_pose is None:
                        self.get_logger().info("Waiting for odom and local_plan data to be initialized.")
                        return
                # wait for autonomous mode to be True
                if not self.is_autonomous_mode:
                        self.get_logger().info("Waiting for autonomous mode.")
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
                self.heading = calculateEulerAngleFromOdometry(self.current_odom)
                # update current position
                self.current_x = self.current_odom.pose.pose.position.x
                self.current_y = self.current_odom.pose.pose.position.y
                # update current goal position
                self.goal_x = self.goal_pose.pose.position.x
                self.goal_y = self.goal_pose.pose.position.y
                # calculate current angle difference (between current angle and goal angle)
                self.angle_diff = angle_difference_in_degree(
                        current_angle_in_degree = self.heading,
                        goal_position_x = self.goal_x, 
                        goal_position_y = self.goal_y
                )
                # calculate current distance between current position and goal position
                self.distance_diff  = math.dist( 
                        [self.current_x, self.current_y], 
                        [self.goal_x, self.goal_y]
                )

        def update_state(self):
                """Update state based on current conditions."""
                if self.state == "Stop":
                        # If in autonomous mode -> Turn
                        if self.is_autonomous_mode:
                                self.turn()
                        # TODO check if at last waypoint
                elif self.state == "Turn":
                        # If angle difference is within tolerance -> Straight
                        if abs(self.angle_diff) < self.turn_angle_tolerance:
                                self.straight()
                        else:
                                self.get_logger().info(f"angle_diff = {self.angle_diff}")
                elif self.state == "Straight":
                        # If within distance tolerance of goal position -> Stop
                        if self.distance_diff < self.straight_distance_tolerance:
                                self.stop()
                        else:
                                self.get_logger().info(f"distance_diff = {self.distance_diff}")
                else:
                        self.get_logger().error(f"Invalid state: '{self.state}'.")
                        self.stop() # default


        def maintain_state(self):
                """Execute the function of the current state."""
                if self.state == "Stop":
                        pass # no maintenance needed
                elif self.state == "Turn":
                        # self.maintain_turn()
                        pass
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
                self.state_publisher.publish(String(data=f"{self.state} -> {state}"))
                # actually change the state
                self.state = state

        def stop(self):
                """Set left and right pwm values to neutral"""
                self.set_state("Stop")
                self.left_pwm.set_neutral()
                self.right_pwm.set_neutral()

        def turn(self):
                """Start turning in place towards the next waypoint"""
                self.set_state("Turn")
                # negative angle difference -> goal angle < current angle -> turn right (clockwise)
                if self.angle_diff < 0:
                        # to turn right (clockwise) -> left forward, right backward
                        self.left_pwm.percentage = self.turn_initial_pwm
                        self.right_pwm.percentage = -self.turn_initial_pwm
                # positive angle difference -> goal angle > current angle -> turn left (counter-clockwise)
                elif self.angle_diff > 0:
                        # to turn left (counter-clockwise) -> left backward, right forward
                        self.left_pwm.percentage = -self.turn_initial_pwm
                        self.right_pwm.percentage = self.turn_initial_pwm
                # no angle difference -> goal angle = current angle -> do nothing
                else:
                        pass

        def straight(self):
                """Start moving straight towards the next waypoint."""
                self.set_state("Straight")
                self.left_pwm.value = self.straight_initial_pwm
                self.right_pwm.value = self.straight_initial_pwm

        # STATE MAINTENANCE

        def maintain_turn(self):
                # TODO Adjust 1 servo to turn the mower towards the next waypoint
                # negative angle difference -> goal angle < current angle -> turn right (clockwise)
                if self.angle_diff < -(self.turn_angle_tolerance / 2):
                        # to turn right (clockwise) -> left forward, right backward
                        pass
                        # Decide how much to adjust it (Maybe use Joel's PID error calc here)
                # positive angle difference -> goal angle > current angle -> turn left (counter-clockwise)
                elif self.angle_diff > (self.turn_angle_tolerance / 2):
                        # to turn left (counter-clockwise) -> left backward, right forward
                        pass
                        # Decide how much to adjust it (Maybe use Joel's PID error calc here)
                # no angle difference -> goal angle = current angle -> do nothing
                else:
                        pass

        def maintain_straight(self):
                # TODO Adjust 1 servo by small amount to correct the mower's direction
                # negative angle difference -> goal angle < current angle -> turn right (clockwise)
                if self.angle_diff < -(self.straight_angle_tolerance / 2):
                        # to turn right (clockwise) -> left forward, right backward
                        # self.left_pwm.percentage += 1
                        self.right_pwm.percentage = -self.straight_adjustment_pwm
                        # TODO Use Joel's PID error calc here
                # positive angle difference -> goal angle > current angle -> turn left (counter-clockwise)
                elif self.angle_diff > (self.straight_angle_tolerance / 2):
                        # to turn left (counter-clockwise) -> left backward, right forward
                        # self.left_pwm.percentage -= 1
                        self.right_pwm.percentage = self.straight_adjustment_pwm
                        # TODO Use Joel's PID error calc here
                # no angle difference -> goal angle = current angle -> reset to initial pwm
                else:
                        if self.right_pwm.value != self.straight_initial_pwm:
                                self.right_pwm.value = self.straight_initial_pwm
                # TODO Adjust both servos to maintain constant forward speed
                        # Use physics (velocity components) to figure out how much to adjust each servo

        # SUBSCRIBER CALLBACKS

        def odom_callback(self, msg: Odometry):
              self.current_odom = msg

        def is_autonomous_mode_callback(self, msg: Bool):
                # if autonomous to manual or manual to autonomous -> set servos to neutral
                # if self.is_autonomous_mode ^ msg.data:
                self.stop()
                self.is_autonomous_mode = msg.data

        def local_plan_callback(self, msg: Path):
                """Sets the goal pose to the middle pose in the path"""
                if len(msg.poses) == 0:
                        self.get_logger().warn("local plan path has no poses.")
                else:
                        self.goal_pose = msg.poses[1]
                        # # right now, get the mid point of the path
                        # mid_point = int(len(msg.poses)/2) # middle index
                        # self.goal_pose = msg.poses[mid_point] # this will always be the middle pose in the path


# MAIN

def main(args=None):
        rclpy.init(args=args)

        local_planner = LocalPlanner()

        rclpy.spin(local_planner)

        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()