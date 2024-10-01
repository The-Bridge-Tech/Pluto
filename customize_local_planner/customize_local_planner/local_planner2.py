"""
Local Planner 2.0
Author: Matthew Lauriault
Created: 10/1/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

# CALCULATION MODULES
import math

# HELPER MODULES
from .untilit import *


# PARAMETERS (must be declared even when using a YAML file)
DEFAULT_PARAMS = {
        # STRATEGY: MOVING STRAIGHT
        'moving_straight_kp': 2.0,
        'moving_straight_kd': 0.0,
        'moving_straight_ki': 0.0,
        'moving_straight_initial_pwm': 1500, # initial speed
        'moving_straight_angle_threshold': 15.0, # will only correct angle while moving straight if greater than this angle
        'moving_straight_forward_prediction_step': 1,

        # STRATEGY: TURNING
        'turning_kp': 3.0,
        'turning_kd': 0.7,
        'turning_ki': 0.5,
        'turning_prediction_step': 1,
        'turning_angle_tolerance': 5,

        # PWM
        'max_pwm': 1765,
        'min_pwm': 992,
        'neutral_pwm': 1376,

        # OTHER
        'autonomous_controller_frequency': 10,
        'distance_error_tolerance': 0.5,
        'local_plan_step_size': 1,
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
                # STRATEGY: MOVING STRAIGHT
                self.moving_straight_kp = load_param("moving_straight_kp").double_value
                self.moving_straight_kd = load_param("moving_straight_kd").double_value
                self.moving_straight_ki = load_param("moving_straight_ki").double_value
                self.moving_straight_initial_pwm = load_param("moving_straight_initial_pwm").integer_value
                # self.moving_straight_distance_tolerance = load_param("moving_straight_distance_tolerance").double_value
                self.moving_straight_angle_threshold = load_param("moving_straight_angle_threshold").double_value
                self.moving_straight_forward_prediction_step = load_param("moving_straight_forward_prediction_step").integer_value
                # STRATEGY: TURNING
                self.turning_kp = load_param("turning_kp").double_value
                self.turning_kd = load_param("turning_kd").double_value
                self.turning_ki = load_param("turning_ki").double_value
                self.turning_prediction_step = load_param("turning_prediction_step").integer_value
                self.turning_angle_tolerance = load_param("turning_angle_tolerance").integer_value
                # PWM
                self.max_pwm = load_param("max_pwm").integer_value
                self.min_pwm = load_param("min_pwm").integer_value
                self.neutral_pwm = load_param("neutral_pwm").integer_value
                # OTHER
                self.autonomous_controller_frequency = load_param("autonomous_controller_frequency").integer_value
                self.distance_error_tolerance = load_param("distance_error_tolerance").double_value
                self.local_plan_step_size = load_param("local_plan_step_size").integer_value

                # TIMER - for interval processing
                self.process_timer = self.create_timer(
                        1 / self.autonomous_controller_frequency, 
                        self.process
                )

                # PUBLISHERS
                self.left_pwm_publisher = self.create_publisher(
                        UInt32, 
                        "/steering_left", 
                        10
                )
                self.right_pwm_publisher = self.create_publisher(
                        UInt32, 
                        "/steering_right", 
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

                # VARIABLES
                self.state = "Stop"
                self.heading = None
                self.current_x = None
                self.current_y = None
                self.goal_x = None
                self.goal_y = None
                self.angle_diff = None
                self.distance_diff = None
                # set servos to neutral position
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
                # execute current state
                self.execute_state()


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
                        if not self.is_autonomous_mode:
                                self.set_state("Turn")
                        # TODO check if at last waypoint
                        return
                elif self.state == "Turn":
                        # If angle difference is within tolerance -> Straight
                        if abs(self.angle_diff) < self.turning_angle_tolerance:
                                self.set_state("Straight")
                        return
                elif self.state == "Straight":
                        # If within distance tolerance of goal position -> Stop
                        if self.distance_diff < self.distance_error_tolerance:
                                self.set_state("Stop")
                        return
                else:
                        self.get_logger().error(f"Invalid state: '{self.state}'.")
                        # Stop by default
                        self.set_state("Stop")
                        return

        def set_state(self, state: str):
                """Set next state."""
                self.get_logger().info(f"State: {self.state} -> {state}")
                self.state = state

        def execute_state(self):
                """Execute the function of the current state."""
                if self.state == "Stop":
                        self.stop()
                elif self.state == "Turn":
                        self.turn()
                elif self.state == "Straight":
                        self.straight()
                else:
                        self.get_logger().error(f"Invalid state: '{self.state}'.")
                

        # STATE FUNCTIONS

        def stop(self):
                """Publish neutral pwm values to the left and right servos."""
                self.publish_left_pwm(self.neutral_pwm)
                self.publish_right_pwm(self.neutral_pwm)
                self.get_logger().info("Servos set to neutral.")

        def turn(self):
                # TODO Adjust 1 servo to turn the mower towards the next waypoint
                        # Decide which servo to adjust
                        # Decide how much to adjust it (Maybe use Joel's PID error calc here)
                pass

        def straight(self):
                # TODO Adjust 1 servo by small amount to correct the mower's direction
                        # Decide which servo to adjust
                        # Decide how much to adjust it (Maybe use Joel's PID error calc here)
                # TODO Adjust both servos to maintain constant forward speed
                        # Use physics (velocity components) to figure out how much to adjust each servo
                pass



        # LOW-LEVEL HELPERS

        def publish_left_pwm(self, pwm: int):
               """Publish pwm value to left servo."""
               self.left_pwm_publisher.publish(UInt32(data=pwm))

        def publish_right_pwm(self, pwm: int):
               """Publish pwm value to right servo."""
               self.right_pwm_publisher.publish(UInt32(data=pwm))


        # SUBSCRIBER CALLBACKS

        def odom_callback(self, msg: Odometry):
              self.current_odom = msg

        def is_autonomous_mode_callback(self, msg: Bool):
                # if autonomous to manual or manual to autonomous -> set servos to neutral
                if self.is_autonomous_mode ^ msg.data:
                        self.stop()
                self.is_autonomous_mode = msg.data

        def local_plan_callback(self, loc_path: Path):
                """Sets the goal pose to the middle pose in the path"""
                if len(loc_path.poses) == 0:
                        self.get_logger().warn("no local path plan has been created")
                else:
                        # right now, get the mid point of the path
                        mid_point = int(len(loc_path.poses)/2) # middle index
                        self.goal_pose = loc_path.poses[mid_point] # this will always be the middle pose in the path
                        self.get_logger().info("local plan path has {0} poses left to navigate".format(len(loc_path.poses)))


# MAIN

def main(args=None):
        rclpy.init(args=args)

        local_planner = LocalPlanner()

        rclpy.spin(local_planner)

        local_planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()