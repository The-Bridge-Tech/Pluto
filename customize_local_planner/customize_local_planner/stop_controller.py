from .controller import Controller

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class Stop(Controller):

    def __init__(self,  neutral_pwm: int, logger):
        super(Stop, self).__init__(logger)
        self.neutral_pwm = neutral_pwm

    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped):
        self.left_value = self.neutral_pwm
        self.right_value = self.neutral_pwm
        self.logger.info("Stopping the robot")

    def __repr__(self):
        return "Stop movement"  