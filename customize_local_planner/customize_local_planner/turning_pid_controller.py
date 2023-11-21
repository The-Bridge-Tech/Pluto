from .controller import Controller

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose
from .untilit import *


class TurningPIDController(Controller):

    def __init__(self, max_pwm: int, min_pwm:
                 int, neutral_pwm: int, kp: int, ki: int,
                 kd: int, initial_pwm: int,
                 forward_prediction_step: int, logger):
        super(TurningPIDController, self).__init__(logger)
   
        self.max_pwm = max_pwm
        self.min_pwm = min_pwm
        self.neutral_pwm = neutral_pwm
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.initial_pwm = initial_pwm

        self.forward_prediction_step: int = forward_prediction_step
        self.previous_error = 0
        self.accumulate_error = 0
        self.angle_off_error = 0

        self.left_value = 0
        self.right_value = 0

    def determine_error_(self, current_angle, goal_angle) -> float:
        error = goal_angle-current_angle
        return error

    def left_pwm(self):
        return self.left_value

    def right_pwm(self):
        return self.right_value

    def execute_movement(self, current_loc: Odometry, global_path: Path):
        goal_angle = process_from_global_path(
            global_path=global_path, forward_prediction_step=self.forward_prediction_step)
        current_angle = calculateEulerAngleFromOdometry(current_loc)
        error = self.determine_error_(
            current_angle=current_angle, goal_angle=goal_angle)

        self.previous_error = self.angle_off_error
        self.angle_off_error = error
        self.accumulate_error += error

        compensate_value = pidCalculation(
            self.kp, self.kd, self.ki, error, self.previous_error, self.accumulate_error)
        _, left, right = determine_Wheel_to_compensate_base_on_angle_error(
            angle_error=self.angle_off_error, init_pwm=self.initial_pwm, compensate_pwm=compensate_value)
        self.left_value = left
        self.right_value = right
