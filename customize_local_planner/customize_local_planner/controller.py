
from nav_msgs.msg import Odometry, Path
class Controller:
    def __init__(self, logger) -> None:
        self.logger = logger
    
    def execute_movement(self, current_loc: Odometry, global_path: Path):
        pass

    def left_pwm(self):
        return 0

    def right_pwm(self):
        return 0