import numpy as np
import math
from math import atan2, pi, sin, cos, atan
from typing import Tuple
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Pose, PoseStamped
import tf_transformations

def roundPwmValue(max_pwm, min_pwm,  pwm_value) -> float:
    if pwm_value > max_pwm:
        return max_pwm
    elif pwm_value < min_pwm:
        return min_pwm
    else:
        return int(pwm_value)


def calculateEulerAngleFromOdometry(odom: Odometry):
    # angle are returned in -180 to 180 degree
    
    q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    rpy = tf_transformations.euler_from_quaternion(q)
    # angle = rpy[2]*(180/pi)
    # if(angle < 0):
    #     angle +=360
    return math.degrees(rpy[2])
    return angle

def calculateEulerAngleFromPoseStamped(pose: PoseStamped):
    q = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]
    rpy = tf_transformations.euler_from_quaternion(q)

    return math.degrees(rpy[2]) # only want the pitch 
    # # if(angle < 0):
    # #     angle +=360
    # return angle
# def quaternion_from_euler(ai, aj, ak):
#     ai /= 2.0
#     aj /= 2.0
#     ak /= 2.0
#     ci = math.cos(ai)
#     si = math.sin(ai)
#     cj = math.cos(aj)
#     sj = math.sin(aj)
#     ck = math.cos(ak)
#     sk = math.sin(ak)
#     cc = ci*ck
#     cs = ci*sk
#     sc = si*ck
#     ss = si*sk

#     q = np.empty((4, ))
#     q[0] = cj*sc - sj*cs
#     q[1] = cj*ss + sj*cc
#     q[2] = cj*cs - sj*sc
#     q[3] = cj*cc + sj*ss

#     return q


# def euler_from_quaternion(quaternion):
#     """
#     Converts quaternion (w in last place) to euler roll, pitch, yaw
#     quaternion = [x, y, z, w]
#     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
#     """
#     x = quaternion.x
#     y = quaternion.y
#     z = quaternion.z
#     w = quaternion.w

#     sinr_cosp = 2 * (w * x + y * z)
#     cosr_cosp = 1 - 2 * (x * x + y * y)
#     roll = np.arctan2(sinr_cosp, cosr_cosp)

#     sinp = 2 * (w * y - z * x)
#     pitch = np.arcsin(sinp)

#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw


# # for the pid controllers
# # assume forward_prediction_step is greater than 0
# def process_from_global_path(global_path: Pose, forward_prediction_step: int):

#     # look forward by certain pose index?

#     future_way_point:PoseStamped  = None
#     current_way_point: PoseStamped = global_path.poses[0]
#     if len(global_path.poses) < forward_prediction_step:

#         future_way_point = global_path.poses[len(global_path.poses)-1]
#     else:
#         # purpose skip waypoint at index 0, because it is the current position of the robot
#         future_way_point = global_path.poses[forward_prediction_step]

#     new_heading_angle = relative_angle_between_two_position(start_position_x=current_way_point.pose.position.x, 
#                                                             start_position_y=current_way_point.pose.position.y,
#                                                             start_position_angle=calculateEulerAngleFromPoseStamped(current_way_point),
#                                                             goal_position_x=future_way_point.pose.position.x, 
#                                                             goal_position_y=future_way_point.pose.position.y)
#     return new_heading_angle

def crossProductMag(vector1, vector2):
    vector1_x = vector1[0]
    vector1_y = vector1[1]
    vector2_x = vector2[0]
    vector2_y = vector2[1]
    return vector1_x*vector2_y - vector1_y*vector2_x
def dotProductMag(vector1, vector2):
    vector1_x = vector1[0]
    vector1_y = vector1[1]
    vector2_x = vector2[0]
    vector2_y = vector2[1]
    return vector1_x*vector2_x + vector1_y*vector2_y

def angle_difference_in_degree(current_angle_in_degree, goal_position_x, goal_position_y):
    angle_to_goal = math.degrees(atan2(goal_position_y, goal_position_x))

    return angle_to_goal- current_angle_in_degree

# def relative_angle_between_two_position(start_position_x, start_position_y, start_position_angle ,goal_position_x, goal_position_y):
#     # https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
#     # dx = goal_position_x - start_position_x
#     # dy = goal_position_y - start_position_y

#     # return math.degrees( math.atan2(dy, dx))

#     #https://wumbo.net/formulas/angle-between-two-vectors-2d/

#     # vector 1, represent the current heading from 0,0
#     # vector 2, build from the goal to start
#     print(math.radians(start_position_angle))
#     vector1 = (  cos( math.radians(start_position_angle)) ,  sin(  math.radians( start_position_angle) ) )
#     vector2 = (goal_position_x - start_position_x, goal_position_y - start_position_y)
#     print(vector1)
#     print(vector2)

#     #https://stackoverflow.com/questions/21483999/using-atan2-to-find-angle-between-two-vectors
    
#     return math.degrees(atan2(crossProductMag(vector1=vector1, vector2=vector2),
#                               dotProductMag(vector1=vector1, vector2=vector2)))
#     return math.degrees( atan2(vector2[1]*vector1[0] - vector2[0]*vector1[1],
#                                vector1[0]*vector2[0] + vector1[1]*vector2[1]  ))


# def determine_Wheel_to_compensate_base_on_angle_error(angle_error: float, init_pwm:int, compensate_pwm:int)->Tuple[str, int, int]:

#     left_servo_pwm = init_pwm
#     right_servo_pwm =init_pwm
#     compensate_info = ""
#     if angle_error == 0:
        
#         compensate_info = "none"
#     if angle_error < 0.0:
#         right_servo_pwm += abs(compensate_pwm)  # only care the absolute value
     
#         # since is moving to left, need to compensate left to move faster, to correct it back
#         compensate_info = "left"
#     else:
#         left_servo_pwm += abs(compensate_pwm)
#         compensate_info = "right"
#     return compensate_info, left_servo_pwm, right_servo_pwm

# function specific for PID Base strategy
def pidCalculation(kp: int, kd: int, ki: int, error: float, previous_error: float, accumulate_error: float):
    # return self.moving_straight_kp * self.angleOffError + self.moving_straight_kd * (self.angleOffError-self.previousError) + self.moving_straight_ki*self.accumulateError
    return kp * error + kd * (error - previous_error) + ki*accumulate_error

# def convert_to_0_360_degree(degree: float):
#    return (degree + 360) % 360

# def determine_direction_enu(goal_angle, current_angle):
#     """
#     Determine if the current angle is to the left or right of the goal angle in ENU coordinates.
    
#     Args:
#     - goal_angle (float): Goal angle in [0, 360] degrees.
#     - current_angle (float): Current angle in [0, 360] degrees.
    
#     Returns:
#     - str: "left", "right", or "at_goal" based on the relative position.
#     """
#     # Calculate the angular difference between goal and current angles
#     angular_difference = goal_angle - current_angle
    
#     # Ensure the difference is within the range [-180, 180] degrees
#     if angular_difference > 180:
#         angular_difference -= 360
#     elif angular_difference < -180:
#         angular_difference += 360
    
#     # Determine the direction based on the angular difference in ENU coordinates
#     if angular_difference == 0:
#         return ("none", angular_difference)
#     elif angular_difference > 0:
#         # robot is at the right of the goal
#         # so compensate left
#         return ("right", angular_difference)
#         return "clockwise (right)"
#     else:
#         # means at the left of the goal
#         # so compensate right
#         return ("left", angular_difference)
#         return "counter-clockwise (left)"

