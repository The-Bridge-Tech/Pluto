import rclpy
from rclpy.node import Node

from .controller import Controller
from .move_straight_pid_controller import MovingStraightPIDController
from .turning_pid_controller import TurningPIDController
from .stop_controller import Stop
from .untilit import *
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from math import atan2, pi, sin, cos, atan
from std_msgs.msg import UInt32, Bool
import tf_transformations
import tf2_ros
# TODO: need to change to standard message type later
from std_msgs.msg import Float32

# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path

from .untilit import *


class LocalPlanner(Node):
    def __init__(self):
        super().__init__("Autonomous_Controller")

        self.declare_parameter("moving_straight_kp", 2.0)
        self.declare_parameter("moving_straight_kd", 0.3)
        self.declare_parameter("moving_straight_ki", 0.0)
        self.declare_parameter("moving_straight_initial_pwm", 1420)
        # self.declare_parameter("moving_straight_distance_tolerance", 0.3)
        self.declare_parameter("moving_straight_angle_threshold", 35.0)
        self.declare_parameter("moving_straight_forward_prediction_step", 1)

        self.declare_parameter("turning_kp", 2.0)
        self.declare_parameter("turning_kd", 0.0)
        self.declare_parameter("turning_ki", 0.0)
        self.declare_parameter("turning_prediction_step", 1)

        self.declare_parameter("max_pwm", 1765)
        self.declare_parameter("min_pwm", 992)
        self.declare_parameter("neutral_pwm", 1376)
        self.declare_parameter("autonomous_controller_frequency", 10)
        
        self.declare_parameter("error_distance_tolerance", 0.5)

        self.declare_parameter("local_plan_step_size", 1)

        # retrieve data from parameters, especially if loaded by yaml file later
        self.moving_straight_initial_pwm = (
            self.get_parameter("moving_straight_initial_pwm")
            .get_parameter_value()
            .integer_value
        )
        # self.moving_straight_distance_tolerance = self.get_parameter(
        #     "moving_straight_distance_tolerance").get_parameter_value().double_value
        self.moving_straight_kp = (
            self.get_parameter("moving_straight_kp").get_parameter_value().double_value
        )
        self.moving_straight_kd = (
            self.get_parameter("moving_straight_kd").get_parameter_value().double_value
        )
        self.moving_straight_ki = (
            self.get_parameter("moving_straight_ki").get_parameter_value().double_value
        )
        self.moving_straight_angle_threshold = (
            self.get_parameter("moving_straight_angle_threshold")
            .get_parameter_value()
            .double_value
        )
        self.moving_straight_forward_prediction_step = (
            self.get_parameter("moving_straight_forward_prediction_step")
            .get_parameter_value()
            .integer_value
        )

        self.turning_kp = (
            self.get_parameter("turning_kp").get_parameter_value().double_value
        )
        self.turning_kd = (
            self.get_parameter("turning_kd").get_parameter_value().double_value
        )
        self.turning_ki = (
            self.get_parameter("turning_ki").get_parameter_value().double_value
        )
        self.turning_prediction_step = (
            self.get_parameter("turning_prediction_step")
            .get_parameter_value()
            .integer_value
        )

        self.neutral_pwm = (
            self.get_parameter("neutral_pwm").get_parameter_value().integer_value
        )
        self.max_pwm = self.get_parameter("max_pwm").get_parameter_value().integer_value
        self.min_pwm = self.get_parameter("min_pwm").get_parameter_value().integer_value
        self.autonomous_controller_frequency = (
            self.get_parameter("autonomous_controller_frequency")
            .get_parameter_value()
            .integer_value
        )
        
        self.error_distance_tolerance = self.get_parameter("error_distance_tolerance").get_parameter_value().double_value
        self.local_plan_step_size = (
            self.get_parameter("local_plan_step_size")
            .get_parameter_value()
            .integer_value
        )
        
        #TODO:
        self.turning_pid_filter_counter = 0  #TODO NEED documentation later
        # some variable
        self.latestGlobalOdom: Odometry = None
        self.is_autonomous_state: Bool = Bool()
        self.is_autonomous_state.data = False
        self.coverage_area_end_pose: PoseStamped = None
        self.is_pure_pursuit_mode: Bool = Bool()
        self.is_pure_pursuit_mode.data = True  # TODO: subscribe topic to update this

        self.pose_to_navigate: PoseStamped = None

        # timer for local planner to do processing
        self.autonomous_controller_process_timer = self.create_timer(
            1 / self.autonomous_controller_frequency, self.local_planner_processor
        )

        # first, create a subscriber to odometry/global
        self.odometryGlobalSub = self.create_subscription(
            Odometry, "/odometry/global", self.globalOdometryCallback, 10
        )
        self.left_wheel_pwm_publisher = self.create_publisher(
            UInt32, "/steering_left", 10
        )
        self.right_wheel_pwm_publisher = self.create_publisher(
            UInt32, "/steering_right", 10
        )

        self.is_autonomous_state_sub = self.create_subscription(
            Bool, "is_autonomous_mode", self.is_autonomous_state_callback, 10
        )
        self.coverage_area_end_pose_sub = self.create_subscription(
            PoseStamped,
            "coverage_area_end_pose",
            self.coverage_area_end_pose_callback,
            10,
        )
        self.pure_pursuit_goal_pose_sub = self.create_subscription(
            PointStamped, "/lookahead_point", self.pure_pursuit_goal_pose_callback, 10
        )
        self.local_plan = self.create_subscription(
            Path, "local_plan", self.local_plan_callback, 10
        )
        self.is_pure_pursuit_mode_sub = self.create_subscription(
            Bool, "is_pure_pursuit_controller_mode", self.is_pure_pursuit_mode_callback, 10
        )
        self.forward_prediction_step_for_strategy_decision = 1
        self.waiting_for_initialization = True
        self.current_local_planner_controller: Controller = None

    def local_planner_processor(self):
        if (
            self.latestGlobalOdom == None
            or self.is_autonomous_state.data != True
            or self.pose_to_navigate == None
        ):
            pass
        else:
            self.determine_local_controller_strategy()

                
            self.current_local_planner_controller.execute_movement(
                    self.latestGlobalOdom, self.pose_to_navigate
                )
            self.publish_left_and_right_pwm()

    def determine_local_controller_strategy(self):
        plan_heading = calculateEulerAngleFromPoseStamped(self.pose_to_navigate)
        current_robot_heading = calculateEulerAngleFromOdometry(self.latestGlobalOdom)
        
        # turn the angles to 360 degree
        plan_heading = convert_to_0_360_degree(plan_heading)
        current_robot_heading = convert_to_0_360_degree(plan_heading)
        

        
        distance_difference = math.hypot(  (self.pose_to_navigate.pose.position.x - self.latestGlobalOdom.pose.pose.position.x),
                                         (self.pose_to_navigate.pose.position.y - self.latestGlobalOdom.pose.pose.position.y))
        direction, angle_error = determine_direction_enu(goal_angle=convert_to_0_360_degree(plan_heading), current_angle=convert_to_0_360_degree(current_robot_heading) )
        # check for angle tolerance
        # if distance_difference < self.error_distance_tolerance:
        #     self.get_logger().info("Stop due to within tolerance error distance")
        #     self.strategy_simple_factory("Stop")
        self.get_logger().info("current angle is {0} heading angle is {1}  The angle turning error is {2} the tolerance angle is {3} the robot is at {4} of the goal".format( current_robot_heading, plan_heading, angle_error, self.moving_straight_angle_threshold, direction))
        if abs(angle_error) > self.moving_straight_angle_threshold:
            self.get_logger().info("PID Turning Due to angle difference")
            self.strategy_simple_factory("PIDTurn")
        else:
            self.get_logger().info("PID moving straight")
            self.strategy_simple_factory("PIDStraight")

    def strategy_simple_factory(self, strategy: str):
        if strategy == "PIDStraight":
            if not isinstance(
                self.current_local_planner_controller, MovingStraightPIDController
            ):
                self.current_local_planner_controller = MovingStraightPIDController(
                    max_pwm=self.max_pwm,
                    min_pwm=self.min_pwm,
                    neutral_pwm=self.neutral_pwm,
                    kp=self.moving_straight_kp,
                    ki=self.moving_straight_ki,
                    kd=self.moving_straight_kd,
                    initial_pwm=self.moving_straight_initial_pwm,
                    forward_prediction_step=self.moving_straight_forward_prediction_step,
                    logger=self.get_logger(),
                )

        elif strategy == "PIDTurn":
            if not isinstance(
                self.current_local_planner_controller, TurningPIDController
            ):
                tuned_min_pwm = self.neutral_pwm -  int(  (self.neutral_pwm -  self.min_pwm)*0.3)
                tuned_max_pwm = self.neutral_pwm +  int(    (self.max_pwm - self.neutral_pwm )*0.3)
                self.get_logger().info("The tuned max {0} and min {1}".format(tuned_max_pwm, tuned_min_pwm))
                self.current_local_planner_controller = TurningPIDController(
                    max_pwm=tuned_max_pwm,
                    min_pwm=tuned_min_pwm,
                    neutral_pwm=self.neutral_pwm,
                    kp=self.turning_kp,
                    ki=self.turning_ki,
                    kd=self.turning_kd,
                    initial_pwm=self.neutral_pwm,
                    forward_prediction_step=self.turning_prediction_step,
                    logger=self.get_logger(),
                )
        elif strategy == "Stop":
            if not isinstance(self.current_local_planner_controller, Stop):
                self.current_local_planner_controller = Stop(
                    neutral_pwm=self.neutral_pwm, logger=self.get_logger()
                )
        else:
            self.get_logger().warn(
                "Strategy choice {0} is not provided".format(strategy)
            )

    def publish_left_and_right_pwm(self):
        # self.get_logger().info(type(self.current_local_planner_controller.left_pwm()))
        left_pwm_input = self.current_local_planner_controller.left_pwm()
        right_pwm_input = self.current_local_planner_controller.right_pwm()

        left_pwm = UInt32()
        left_pwm.data = left_pwm_input
        right_pwm = UInt32()
        right_pwm.data = right_pwm_input

        # left_pwm.data = roundPwmValue(
        #     max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=left_pwm_input
        # )
        # right_pwm.data = roundPwmValue(
        #     max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=right_pwm_input
        # )

        self.left_wheel_pwm_publisher.publish(left_pwm)
        self.right_wheel_pwm_publisher.publish(right_pwm)

        self.get_logger().info("local planner info: {0}".format(repr(self.current_local_planner_controller)))
        self.get_logger().info(
            "left pwm {0} right pwm {1}".format(left_pwm_input, right_pwm_input)
        )

    def globalOdometryCallback(self, global_odom: Odometry):
        self.latestGlobalOdom: Odometry = global_odom

    def is_autonomous_state_callback(self, state: Bool):
        self.is_autonomous_state = state

    def coverage_area_end_pose_callback(self, pose: PoseStamped):
        self.coverage_area_end_pose = pose
    def is_pure_pursuit_mode_callback(self, val: Bool):
        self.is_pure_pursuit_mode = val

    def pure_pursuit_goal_pose_callback(self, pose: PointStamped):
        if self.is_pure_pursuit_mode.data == True:
            self.pose_to_navigate = PoseStamped()
            goal_x = pose.point.x
            goal_y = pose.point.y

            euler_angle = calculate_heading_angle_between_two_position(
                self.latestGlobalOdom.pose.pose.position.x,
                self.latestGlobalOdom.pose.pose.position.y,
                goal_x,
                goal_y,
            )

            #q = quaternion_from_euler(0, euler_angle, 0)
            self.pose_to_navigate.pose.position.x = goal_x
            self.pose_to_navigate.pose.position.y = goal_y
            q = tf_transformations.quaternion_from_euler(0.0,0.0,euler_angle* (pi/180))
            self.pose_to_navigate.pose.orientation.x = q[0]
            self.pose_to_navigate.pose.orientation.y = q[1]
            self.pose_to_navigate.pose.orientation.z = q[2]
            self.pose_to_navigate.pose.orientation.w = q[3]
            
            # self.get_logger().info(
            #     "Calcuated angle is {0} the converted angle is {1} ".format(
            #         euler_angle,
            #         calculateEulerAngleFromPoseStamped(self.pose_to_navigate), 
            #     )
            # )

    def local_plan_callback(self, loc_path: Path):
        if self.is_pure_pursuit_mode.data == False:
            if len(loc_path.poses)  == 0:
                self.get_logger().warn("no local path plan has created")
            else:
                # right now, get the mid point of the path
                mid_point = int(len(loc_path.poses)/2)
                self.pose_to_navigate = loc_path.poses[mid_point]
                self.get_logger().info("local plan path has total {0} pose".format(len(loc_path.poses)))
            # if len(loc_path.poses) <= self.local_plan_step_size - 1:
            #     self.get_logger().info(
            #         "local plan path only have {0} pose, smaller than configure path size. Thus, using the very first pose in path".format(len(loc_path.poses))
            #     )
            #     self.pose_to_navigate = loc_path.poses[0]
            # else:
            #     self.get_logger().info("local plan path has total {0} pose".format(len(loc_path.poses)))
            #     self.pose_to_navigate = loc_path.poses[self.local_plan_step_size - 1]


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LocalPlanner()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
