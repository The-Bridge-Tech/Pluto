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
from std_msgs.msg import UInt32

# TODO: need to change to standard message type later
from std_msgs.msg import Float32

# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Pose
from nav_msgs.msg import Odometry, OccupancyGrid, Path

from .untilit import *


class LocalPlanner(Node):
    def __init__(self):
        super().__init__("pid_controller")

        self.declare_parameter("moving_straight_kp", 2.0)
        self.declare_parameter("moving_straight_kd", 0.3)
        self.declare_parameter("moving_straight_ki", 0.0)
        self.declare_parameter("moving_straight_initial_pwm", 1420)
        # self.declare_parameter("moving_straight_distance_tolerance", 0.3)
        self.declare_parameter("moving_straight_angle_threshold", 25)
        self.declare_parameter("moving_straight_forward_prediction_step", 1)

        self.declare_parameter("turning_kp", 2.0)
        self.declare_parameter("turning_kd", 0.0)
        self.declare_parameter("turning_ki", 0.0)
        self.declare_parameter("turning_prediction_step", 1)

        self.declare_parameter("max_pwm", 1765)
        self.declare_parameter("min_pwm", 992)
        self.declare_parameter("neutral_pwm", 1376)
        self.declare_parameter("local_planner_frequency", 10)

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
        self.local_planner_frequency = (
            self.get_parameter("local_planner_frequency")
            .get_parameter_value()
            .integer_value
        )

        # some variable
        self.latestGlobalOdom = None
        self.latestGlobalPath: Path = None

        # timer for local planner to do processing
        self.local_planner_process_timer = self.create_timer(
            1 / self.local_planner_frequency, self.local_planner_processor
        )

        self.goal_heading_angle_in_enu: float = None
        self.angleOffError: float = 0
        self.previousError = 0
        self.accumulateError = 0
        self.wheel_to_compensate: str = "none"
        # True if the robot is within a certain distance of current waypoint
        self.isWithinGoalDistance = False
        self.left_servo_pwm = self.moving_straight_initial_pwm
        self.right_servo_pwm = self.moving_straight_initial_pwm
        self.compensateValue: float = 0

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
        self.global_path_pose = self.create_subscription(
            Path, "planner/path", self.global_path_callback, 10
        )

        self.forward_prediction_step_for_strategy_decision = 1
        self.waiting_for_initialization = True
        self.current_local_planner_controller: Controller = None

    def local_planner_processor(self):
        if self.latestGlobalOdom == None or self.latestGlobalPath == None:
            self.get_logger().info("Waiting for initialization")
        else:
            self.waiting_for_initialization = False
        if self.waiting_for_initialization == True:
            # means still waiting for initialization
            pass
        else:
            self.determine_local_controller_strategy()
            self.current_local_planner_controller.execute_movement(
                self.latestGlobalOdom, self.latestGlobalPath
            )
            self.publish_left_and_right_pwm()

    def determine_local_controller_strategy(self):
        plan_heading = process_from_global_path(
            self.latestGlobalPath, self.forward_prediction_step_for_strategy_decision
        )
        current_robot_heading = calculateEulerAngleFromOdometry(self.latestGlobalOdom)
        angle_difference: float = abs(current_robot_heading - plan_heading)
        if len(self.latestGlobalPath.poses) < 2:
            self.get_logger().info(
                "Global path planning return with less than 2 way point!"
            )
            self.strategy_simple_factory("Stop")
        else:
            # check for angle tolerance
            if angle_difference > self.moving_straight_angle_threshold:
                self.get_logger().info("PID Turning Due to angle difference")
                self.strategy_simple_factory("PIDTurn")
            else:
                self.get_logger().info("PID moving straight")
                self.strategy_simple_factory("PIDStraight")
        
        # elif angle_difference >= self.moving_straight_angle_threshold:
        #     # 1. determine current robot heading

        #     self.get_logger().info("PID Turning")
        #     self.strategy_simple_factory("PIDTurn")
        # elif False:
        #     fasdfasdfd
        # else:
        #     self.get_logger().info("PID moving straight")
        #     self.strategy_simple_factory("PIDStraight")

    def strategy_simple_factory(self, strategy: str):
        if strategy == "PIDStraight" and not isinstance(
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
                forward_prediction=self.moving_straight_forward_prediction_step,
                logger=self.get_logger(),
            )

        elif strategy == "PIDTurn" and not isinstance(
            self.current_local_planner_controller, TurningPIDController
        ):
            self.current_local_planner_controller = TurningPIDController(
                max_pwm=self.max_pwm,
                min_pwm=self.min_pwm,
                neutral_pwm=self.neutral_pwm,
                kp=self.turning_kp,
                ki=self.turning_ki,
                kd=self.turning_kd,
                initial_pwm=self.neutral_pwm,
                forward_prediction_step=self.turning_prediction_step,
                logger=self.get_logger(),
            )
        elif strategy == "Stop" and not isinstance(
            self.current_local_planner_controller, Stop
        ):
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
        right_pwm = UInt32()

        left_pwm.data = roundPwmValue(
            max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=left_pwm_input
        )
        right_pwm.data = roundPwmValue(
            max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=right_pwm_input
        )

        self.left_wheel_pwm_publisher.publish(left_pwm)
        self.right_wheel_pwm_publisher.publish(right_pwm)

        self.get_logger().info(
            "left pwm {0} right pwm {1}".format(left_pwm_input, right_pwm_input)
        )

    def global_path_callback(self, global_path: Path):
        self.latestGlobalPath = global_path

    def globalOdometryCallback(self, global_odom: Odometry):
        self.latestGlobalOdom = global_odom


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
