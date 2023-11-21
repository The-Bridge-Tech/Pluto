import numpy as np
import math
from math import atan2, pi, sin, cos, atan
from typing import Tuple
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped


def roundPwmValue(max_pwm, min_pwm,  pwm_value) -> float:
    if pwm_value > max_pwm:
        return max_pwm
    elif pwm_value < min_pwm:
        return min_pwm
    else:
        return round(pwm_value)


def calculateEulerAngleFromOdometry(odom: Odometry):
    # angle are returned in -180 to 180 degree
    rpy = euler_from_quaternion(odom.pose.pose.orientation)
    angle = rpy[2]*(180/pi)
    # if(angle < 0):
    #     angle +=360
    return angle


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# for the pid controllers
# assume forward_prediction_step is greater than 0
def process_from_global_path(global_path: Pose, forward_prediction_step: int):

    # look forward by certain pose index?

    future_way_point:PoseStamped  = None
    current_way_point: PoseStamped = global_path.poses[0]
    if len(global_path.poses) < forward_prediction_step:

        future_way_point = global_path.poses[len(global_path.poses)-1]
    else:
        # purpose skip waypoint at index 0, because it is the current position of the robot
        future_way_point = global_path.poses[forward_prediction_step]

    new_heading_angle = calculate_heading_angle_between_two_position(current_way_point.pose.position.x, current_way_point.pose.position.y,
                                                                     future_way_point.pose.position.x, future_way_point.pose.position.y)
    return new_heading_angle


def calculate_heading_angle_between_two_position(start_position_x, start_position_y, goal_position_x, goal_position_y):

    dx = goal_position_x - start_position_x
    dy = goal_position_y - start_position_y

    return math.atan2(dy, dx) * (180/pi)


def determine_Wheel_to_compensate_base_on_angle_error(angle_error: float, init_pwm:int, compensate_pwm:int)->Tuple[str, int, int]:

    left_servo_pwm = init_pwm
    right_servo_pwm =init_pwm
    compensate_info = ""
    if angle_error == 0:
        
        compensate_info = "none"
    if angle_error < 0.0:
        left_servo_pwm += abs(compensate_pwm)  # only care the absolute value
     
        # since is moving to left, need to compensate left to move faster, to correct it back
        compensate_info = "left"
    else:
        right_servo_pwm += abs(compensate_pwm)
        compensate_info = "right"
    return compensate_info, left_servo_pwm, right_servo_pwm

# function specific for PID Base controller
def pidCalculation(kp: int, kd: int, ki: int, error: float, previous_error: float, accumulate_error: float):
    # return self.moving_straight_kp * self.angleOffError + self.moving_straight_kd * (self.angleOffError-self.previousError) + self.moving_straight_ki*self.accumulateError
    return kp * error + kd * (error - previous_error) + ki*accumulate_error
