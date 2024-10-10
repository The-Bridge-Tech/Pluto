"""
Conversion Helper Methods
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import tf_transformations

# CALCULATION MODULES
import numpy as np

# CONSTANTS
PWM_MIN = 992
PWM_NEUTRAL = 1376
PWM_MAX = 1765


def euler_to_quaternion(roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

def pwm_value_to_percentage(pwm: int) -> float:
        if pwm == PWM_NEUTRAL:
                return 0
        elif pwm < PWM_NEUTRAL:
                return ((pwm - PWM_NEUTRAL) / (PWM_NEUTRAL - PWM_MIN)) * 100
        elif pwm > PWM_NEUTRAL:
                return ((pwm - PWM_NEUTRAL) / (PWM_MAX - PWM_NEUTRAL)) * 100
        
def angle_from_odometry(odom: Odometry):
        """Angle is returned in the range -180 to 180 degrees"""
        q = [
                odom.pose.pose.orientation.x, 
                odom.pose.pose.orientation.y, 
                odom.pose.pose.orientation.z, 
                odom.pose.pose.orientation.w
        ]
        # rpy = [roll, pitch, yaw]
        rpy = tf_transformations.euler_from_quaternion(q)
        # rpy[2] = yaw (orientation around the vertical axis)
        return math.degrees(rpy[2])