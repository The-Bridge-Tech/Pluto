from .controller import Controller

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
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


    def __repr__(self):
        return "TurningPID controller max_pwm: {0} min_pwm:{1} neutral_pwm:{2} left_pwm:{3} right_pwm:{4} kp:{5} kd:{6} ki:{7} angler_off_error:{8} previous_error:{9}  accumulate_error{10}".format(
            self.max_pwm, self.min_pwm, self.neutral_pwm, self.left_value, self.right_value, self.kp, self.kd, self.ki, self.angle_off_error, self.previous_error, self.accumulate_error
        )

    def left_pwm(self):
        return self.left_value

    def right_pwm(self):
        return self.right_value
    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped):
        goal_angle = convert_to_0_360_degree( calculateEulerAngleFromPoseStamped(pose_to_navigate))
        current_angle = convert_to_0_360_degree(calculateEulerAngleFromOdometry(current_loc))
        
        direction, error = determine_direction_enu(goal_angle=goal_angle, current_angle=current_angle)
        self.logger().info("Moving straight pid: direction {0}) error{1} ", direction, error)
        self.previous_error = self.angle_off_error
        self.angle_off_error = error
        #TODO: determine accumulate error laterself.accumulate_error += error

        compensate_value = pidCalculation(
            self.kp, self.kd, self.ki, self.angle_off_error, self.previous_error, self.accumulate_error)
        
        # only want the absolute compensation value
        compensate_value = abs(compensate_value)
        
        self.left_value = self.initial_pwm
        self.right_value = self.initial_pwm
        if direction == "none":
            pass
        elif direction == "right":
            self.right_value += compensate_value  # at the right of the goal, so turn left
        else:
            self.right_value -= compensate_value  # at the left of the goal, so turn right
        # now, round off those value
        self.left_value = self.initial_pwm
        self.right_value = int(roundPwmValue(max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=self.right_value))
