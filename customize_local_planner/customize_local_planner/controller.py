# ROS MODULES
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.impl.rcutils_logger import RcutilsLogger

# HELPER MODULES
from .untilit import *


class Controller:
    
    def __init__(self, logger: RcutilsLogger) -> None:
        self.logger = logger
        self.direction: int = None
        self.angle_off_error: float = 0
        self.accumulate_error: float = 0
        self.previous_error: float = 0
        self.left_pwm: int = 0
        self.right_pwm: int = 0
    
    def angle_error_calculation(self, angle_difference_in_degree: float):      
        angle_error_from_goal = angle_difference_in_degree

        self.previous_error = self.angle_off_error
        self.angle_off_error = angle_error_from_goal
        self.accumulate_error += self.angle_off_error

    def pid_movement_algorithm(self, angle_difference_in_degree: float, kp: int, kd: int, ki: int, initial_pwm: int, max_pwm: int, min_pwm: int):
        # Calculate angle error
        self.angle_error_calculation(angle_difference_in_degree)
        # Calculate value to compensate the error in angle
        compensate_value = pidCalculation(
            kp=kp,
            kd=kd, 
            ki=ki, 
            error=self.angle_off_error, 
            previous_error=self.previous_error, 
            accumulate_error=self.accumulate_error 
        )
        # Initialize left and right pwm values
        self.left_pwm = initial_pwm
        self.right_pwm = initial_pwm
        # If there is angle off error, add the compensate value to the right value
        if(self.angle_off_error == 0):
            pass
        else:
            self.right_pwm += compensate_value
        
        # Set left and right pwm values
        self.left_pwm = int(roundPwmValue(
            max_pwm=max_pwm, 
            min_pwm=min_pwm, 
            pwm_value=self.left_pwm
        ))
        self.right_pwm = int(roundPwmValue(
            max_pwm=max_pwm, 
            min_pwm=min_pwm, 
            pwm_value=self.right_pwm
        ))

    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped, angle_difference_in_degree:float):
        pass