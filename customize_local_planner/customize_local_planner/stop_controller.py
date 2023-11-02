from .controller import Controller

from nav_msgs.msg import Odometry, Path


class Stop(Controller):

    def __init__(self,  neutral_pwm: int, logger):
        super.__init__(self=self, logger=logger)
        self.neutral_pwm = neutral_pwm

        
    def left_pwm(self):
        return self.neutral_pwm

    def right_pwm(self):
        return self.neutral_pwm
    def execute_movement(self, current_loc: Odometry, global_path: Path):
        self.logger.info("Stopping the robot")
