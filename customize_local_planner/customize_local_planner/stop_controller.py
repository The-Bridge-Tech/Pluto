# ROS MODULES
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.impl.rcutils_logger import RcutilsLogger


# HELPER MODULES
from .controller import Controller


class Stop(Controller):

    def __init__(self,  neutral_pwm: int, logger: RcutilsLogger):
        super().__init__(logger)
        self.neutral_pwm = neutral_pwm

    def execute_movement(self, current_loc: Odometry, pose_to_navigate: PoseStamped, angle_difference_in_degree:float):
        self.left_value = self.neutral_pwm
        self.right_value = self.neutral_pwm
        self.logger.info("Stopping the robot")

    def __repr__(self):
        return "Stop movement"  