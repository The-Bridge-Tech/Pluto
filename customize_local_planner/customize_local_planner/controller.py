
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
class Controller:
    def __init__(self, logger) -> None:
        self.logger = logger
    
    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped):
        pass

    def left_pwm(self):
        return 0

    def right_pwm(self):
        return 0