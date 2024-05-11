from .controller import Controller

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from .untilit import *


class MovingStraightPIDController(Controller):

    def __init__(self, max_pwm: int, min_pwm:
                 int, neutral_pwm: int, kp: int, ki: int,
                 kd: int, initial_pwm: int,
                 logger):
        
        super(MovingStraightPIDController, self).__init__(logger)
        self.max_pwm = max_pwm
        self.min_pwm = min_pwm
        self.neutral_pwm = neutral_pwm
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.initial_pwm = initial_pwm


    def __repr__(self):
        return "MovingStraight controller max_pwm: {0} min_pwm:{1} neutral_pwm:{2} left_pwm:{3} right_pwm:{4} kp:{5} kd:{6} ki:{7} angler_off_error:{8} previous_error:{9}  accumulate_error{10} direction{11} ".format(
            self.max_pwm, self.min_pwm, self.neutral_pwm, self.left_value, self.right_value, self.kp, self.kd, self.ki, self.angle_off_error, self.previous_error, self.accumulate_error, self.direction
        )

    def left_pwm(self) -> int:
        return self.left_value

    def right_pwm(self) -> int:
        return self.right_value

    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped):


        self.angle_error_calculation(current_loc=current_loc, pose_to_navigate=pose_to_navigate)
        
        compensate_value = pidCalculation(
            self.kp, self.kd, self.ki, self.angle_off_error, self.previous_error, self.accumulate_error)

        self.left_value = self.initial_pwm
        self.right_value = self.initial_pwm

        if(self.angle_off_error == 0):
            pass
        else:
            # tune the right side
            self.right_value += compensate_value
        # now, round off those value
        self.left_value = int(roundPwmValue(max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=self.left_value))
        self.right_value = int(roundPwmValue(max_pwm=self.max_pwm, min_pwm=self.min_pwm, pwm_value=self.right_value))
