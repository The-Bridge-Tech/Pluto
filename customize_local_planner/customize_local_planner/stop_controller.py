from .controller import Controller

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class Stop(Controller):

    def __init__(self,  neutral_pwm: int, logger):
        super(Stop, self).__init__(logger)
        self.neutral_pwm = neutral_pwm

        
    def left_pwm(self):
        return self.neutral_pwm

    def right_pwm(self):
        return self.neutral_pwm
    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped):
        self.logger.info("Stopping the robot")
