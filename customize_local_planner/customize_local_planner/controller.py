
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from .untilit import *
class Controller:
    def __init__(self, logger) -> None:
        self.logger = logger
        self.direction:int = None
        self.angle_off_error:float = 0
        self.accumulate_error:float = 0
        self.previous_error:float = 0
        self.left_value:int = 0
        self.right_value:int = 0
    
    def angle_error_calculation(self, current_loc:Odometry, pose_to_navigate:PoseStamped):
        # note: Odometry's angle is a absolute angle

        current_absolute_heading_angle = calculateEulerAngleFromOdometry(current_loc)
        
        angle_error_from_goal = relative_angle_between_two_position(
            start_position_x= current_loc.pose.pose.position.x,
            start_position_y=current_loc.pose.pose.position.y,
            start_position_angle= current_absolute_heading_angle,
            goal_position_x= pose_to_navigate.pose.position.x,
            goal_position_y=pose_to_navigate.pose.position.y
        )


        self.previous_error = self.angle_off_error
        self.angle_off_error = angle_error_from_goal
        self.accumulate_error += self.angle_off_error

    def pid_movement_algorithm(self, current_loc:Odometry, pose_to_navigate:PoseStamped,
                               kp:int,  kd:int,ki:int, initial_pwm:int, max_pwm:int, min_pwm:int):
        self.angle_error_calculation(current_loc=current_loc, pose_to_navigate=pose_to_navigate)
        
        compensate_value = pidCalculation(kp=kp,kd=kd, ki=ki, error=self.angle_off_error , previous_error=self.previous_error, accumulate_error=self.accumulate_error )

        self.left_value = initial_pwm
        self.right_value = initial_pwm
        if(self.angle_off_error == 0):
            pass
        else:
            self.right_value +=compensate_value
            
        self.left_value = int(roundPwmValue(max_pwm=max_pwm, min_pwm=min_pwm, pwm_value=self.left_value))
        self.right_value = int(roundPwmValue(max_pwm=max_pwm, min_pwm=min_pwm, pwm_value=self.right_value))

        

            
            

    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped):
        pass

    def left_pwm(self):
        return self.left_value

    def right_pwm(self):
        return self.right_value