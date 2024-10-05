# ROS MODULES
import rclpy
from rclpy.node import Node
import tf_transformations
from std_msgs.msg import UInt32, Bool
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import ParameterValue

# CALCULATION MODULES
import math
from math import atan2

# HELPER MODULES
from .controller import Controller
from .move_straight_pid_controller import MovingStraightPIDController
from .turning_pid_controller import TurningPIDController
from .stop_controller import Stop
from .untilit import *


# PARAMETERS (must be declared even when using a YAML file)
DEFAULT_PARAMS = {
    # STRATEGY: MOVING STRAIGHT
    'moving_straight_kp': 2.0,
    'moving_straight_kd': 0.0,
    'moving_straight_ki': 0.0,
    'moving_straight_initial_pwm': 1500, # initial speed
    'moving_straight_angle_tolerance': 15.0, # will only correct angle while moving straight if greater than this angle
    'moving_straight_forward_prediction_step': 1,

    # STRATEGY: TURNING
    'turning_kp': 3.0,
    'turning_kd': 0.7,
    'turning_ki': 0.5,
    'turning_initial_pwm_percentage': 10.0,
    'turning_prediction_step': 1,
    'turning_angle_tolerance': 5.0,

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
        self.moving_straight_angle_tolerance = load_param("moving_straight_angle_tolerance").double_value
        self.moving_straight_forward_prediction_step = load_param("moving_straight_forward_prediction_step").integer_value
        # STRATEGY: TURNING
        self.turning_kp = load_param("turning_kp").double_value
        self.turning_kd = load_param("turning_kd").double_value
        self.turning_ki = load_param("turning_ki").double_value
        self.turning_prediction_step = load_param("turning_prediction_step").integer_value
        # PWM
        self.max_pwm = load_param("max_pwm").integer_value
        self.min_pwm = load_param("min_pwm").integer_value
        self.neutral_pwm = load_param("neutral_pwm").integer_value
        # OTHER
        self.autonomous_controller_frequency = load_param("autonomous_controller_frequency").integer_value
        self.distance_error_tolerance = load_param("distance_error_tolerance").double_value
        self.local_plan_step_size = load_param("local_plan_step_size").integer_value

        # TIMER - for interval processing
        self.autonomous_controller_process_timer = self.create_timer(
            1 / self.autonomous_controller_frequency, 
            self.local_planner_processor
        )

        # PUBLISHERS
        self.left_wheel_pwm_publisher = self.create_publisher(
            UInt32, 
            "/steering_left", 
            10
        )
        self.right_wheel_pwm_publisher = self.create_publisher(
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
        self.coverage_area_end_pose_sub = self.create_subscription(
            PoseStamped,
            "coverage_area_end_pose",
            self.coverage_area_end_pose_callback,
            10,
        )
        self.coverage_area_end_pose: PoseStamped = None
        self.pure_pursuit_goal_pose_sub = self.create_subscription(
            PointStamped, 
            "/lookahead_point", 
            self.pure_pursuit_goal_pose_callback, 
            10
        )
        self.is_pure_pursuit_mode_sub = self.create_subscription(
            Bool, 
            "is_pure_pursuit_controller_mode", 
            self.is_pure_pursuit_mode_callback, 
            10
        )
        self.is_pure_pursuit_mode = True  # TODO: subscribe topic to update this

        # VARIABLES
        self.current_controller: Controller = None
        self.pose_to_navigate: PoseStamped = None

        # set servos to neutral position
        self.publish_neutral_pwm()

    
    # HELPERS

    def local_planner_processor(self):
        """Determines strategy -> sets controller based on strategy -> executes that controller's movement -> publishes left and right motor values"""
        # wait until /odometry/global and /local_plan have been subscribed
        if self.current_odom is None or self.pose_to_navigate is None:
            self.get_logger().info("Waiting for odom and local_plan data to be initialized.")
            return
        # wait for autonomous mode to be True
        if not self.is_autonomous_mode:
            self.get_logger().info("Waiting for autonomous mode.")
            return
        # update self.current_controller subclass (strategy) and return the difference between the following angles:
        #   current heading from odometry
        #   angle between current location and current goal pose
        angle_difference_in_degree = self.determine_controller_strategy()
        # update self.current_controller.left_pwm and .right_pwm
        self.current_controller.execute_movement(
            current_loc=self.current_odom, 
            pose_to_navigate=self.pose_to_navigate, 
            angle_difference_in_degree=angle_difference_in_degree
        )
        # publish self.current_controller.left_pwm and .right_pwm to '/steering_left' and '/steering_right' respectively
        self.publish_left_and_right_pwm()

    def determine_controller_strategy(self):
        """Determine the next strategy (straight, turn, or stop) based on the current angle difference.
        Then set the controller based on the strategy with set_controller_strategy()"""
        
        # get robot's current heading
        current_robot_heading = calculateEulerAngleFromOdometry(self.current_odom)
        
        # get robot's current position
        start_pose_x = self.current_odom.pose.pose.position.x
        start_pose_y = self.current_odom.pose.pose.position.y

        # get robot's goal position
        goal_pose_x = self.pose_to_navigate.pose.position.x
        goal_pose_y = self.pose_to_navigate.pose.position.y
    
        # calculate difference between current angle and goal angle
        angle_diff = angle_difference_in_degree(
            current_angle_in_degree = current_robot_heading,
            goal_position_x = goal_pose_x, 
            goal_position_y = goal_pose_y
        )
        
        # calculate distance between current position and goal position
        distance_diff  = math.dist( 
            [start_pose_x, start_pose_y], 
            [goal_pose_x, goal_pose_y]
        )

        self.get_logger().info(f"Angle difference: {angle_diff}\tCurrent Pos: ({start_pose_x}, {start_pose_y})\tGoal Pos: ({goal_pose_x}, {goal_pose_y})\tHeading: {current_robot_heading} \n")

        # If distance within error tolerance -> Stop
        if distance_diff < self.distance_error_tolerance:
            self.get_logger().info("Distance within error tolerance -> Stop")
            self.set_controller_strategy("Stop")
        # Angle difference above threshold -> PIDTurn
        elif abs(angle_diff) > self.moving_straight_angle_tolerance:
            self.get_logger().info("Angle difference above threshold -> PID Turning")
            self.set_controller_strategy("PIDTurn")
        # Distance and angle are good -> PIDStraight
        else:
            self.get_logger().info("PID Moving Straight")
            self.set_controller_strategy("PIDStraight")
        # Return the angle difference
        return angle_diff

    def set_controller_strategy(self, strategy: str):
        """Update the current controller subclass (MovingStraightPIDController, TurningPIDController, or Stop) based on the strategy"""
        if strategy == "PIDStraight":
            if not isinstance(self.current_controller, MovingStraightPIDController):
                self.current_controller = MovingStraightPIDController(
                    max_pwm=self.max_pwm,
                    min_pwm=self.min_pwm,
                    neutral_pwm=self.neutral_pwm,
                    kp=self.moving_straight_kp,
                    ki=self.moving_straight_ki,
                    kd=self.moving_straight_kd,
                    initial_pwm=self.moving_straight_initial_pwm,
                    logger=self.get_logger()
                )
        elif strategy == "PIDTurn":
            if not isinstance(self.current_controller, TurningPIDController):
                # Scale down the pwm range to 30%
                tuned_min_pwm = self.neutral_pwm - int((self.neutral_pwm - self.min_pwm) * 0.3)
                tuned_max_pwm = self.neutral_pwm + int((self.max_pwm - self.neutral_pwm) * 0.3)
                self.get_logger().info("The tuned max {0} and min {1}".format(tuned_max_pwm, tuned_min_pwm))
                self.current_controller = TurningPIDController(
                    max_pwm=tuned_max_pwm,
                    min_pwm=tuned_min_pwm,
                    neutral_pwm=self.neutral_pwm,
                    kp=self.turning_kp,
                    ki=self.turning_ki,
                    kd=self.turning_kd,
                    initial_pwm=self.neutral_pwm,
                    logger=self.get_logger(),
                )
        elif strategy == "Stop":
            if not isinstance(self.current_controller, Stop):
                self.current_controller = Stop(
                    neutral_pwm=self.neutral_pwm, 
                    logger=self.get_logger()
                )
        else:
            self.get_logger().warn(f"Strategy '{strategy}' is not provided")

    def publish_left_and_right_pwm(self):
        """Publishes left and right pwm values to the left and right servos."""
        # self.get_logger().info(type(self.current_controller.left_pwm()))
        left_pwm_input = self.current_controller.left_pwm
        right_pwm_input = self.current_controller.right_pwm

        self.left_wheel_pwm_publisher.publish(UInt32(data=left_pwm_input))
        self.right_wheel_pwm_publisher.publish(UInt32(data=right_pwm_input))

        self.get_logger().info(f"Current controller: {self.current_controller}")
        self.get_logger().info(f"left pwm: {left_pwm_input} right pwm: {right_pwm_input}")

    def publish_neutral_pwm(self):
        """Publish neutral pwm values to the left and right servos."""
        self.left_wheel_pwm_publisher.publish(UInt32(data=self.neutral_pwm))
        self.right_wheel_pwm_publisher.publish(UInt32(data=self.neutral_pwm))
        self.get_logger().info("Servos set to neutral.")



    # SUBSCRIBER CALLBACKS

    def odom_callback(self, global_odom: Odometry):
        self.current_odom: Odometry = global_odom

    def is_autonomous_mode_callback(self, msg: Bool):
        # if autonomous to manual or manual to autonomous -> set servos to neutral
        if self.is_autonomous_mode ^ msg.data:
            self.publish_neutral_pwm()
        self.is_autonomous_mode = msg.data


    def local_plan_callback(self, loc_path: Path):
        """Sets the goal pose to the middle pose in the path"""
        # NOTE: local plan includes the following locations:
        #   1. current location
        #   2. middle pose in the path
        if not self.is_pure_pursuit_mode:
            if len(loc_path.poses) == 0:
                self.get_logger().warn("no local path plan has been created")
            else:
                # right now, get the mid point of the path
                mid_point = int(len(loc_path.poses)/2) # middle index
                self.pose_to_navigate = loc_path.poses[mid_point] # this will always be the middle pose in the path
                self.get_logger().info("local plan path has {0} poses left to navigate".format(len(loc_path.poses)))
            # if len(loc_path.poses) <= self.local_plan_step_size - 1:
            #     self.get_logger().info(
            #         "local plan path only have {0} pose, smaller than configure path size. Thus, using the very first pose in path".format(len(loc_path.poses))
            #     )
            #     self.pose_to_navigate = loc_path.poses[0]
            # else:
            #     self.get_logger().info("local plan path has total {0} pose".format(len(loc_path.poses)))
            #     self.pose_to_navigate = loc_path.poses[self.local_plan_step_size - 1]

    def coverage_area_end_pose_callback(self, pose: PoseStamped):
        self.coverage_area_end_pose = pose

    def is_pure_pursuit_mode_callback(self, msg: Bool):
        self.is_pure_pursuit_mode = msg.data

    def pure_pursuit_goal_pose_callback(self, pose: PointStamped):
        if self.is_pure_pursuit_mode:
            self.pose_to_navigate = PoseStamped()
            goal_x = pose.point.x
            goal_y = pose.point.y

            #TODO: possibly don't need to calculate the angle
            euler_angle = atan2(goal_y, goal_x)

            #q = quaternion_from_euler(0, euler_angle, 0)
            self.pose_to_navigate.pose.position.x = goal_x
            self.pose_to_navigate.pose.position.y = goal_y
            q = tf_transformations.quaternion_from_euler(0.0,0.0,math.radians(euler_angle))
            self.pose_to_navigate.pose.orientation.x = q[0]
            self.pose_to_navigate.pose.orientation.y = q[1]
            self.pose_to_navigate.pose.orientation.z = q[2]
            self.pose_to_navigate.pose.orientation.w = q[3]


# MAIN

def main(args=None):
    rclpy.init(args=args)

    local_planner = LocalPlanner()

    rclpy.spin(local_planner)

    local_planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
