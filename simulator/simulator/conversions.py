"""
Conversion Helper Methods
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
from nav_msgs.msg import Odometry
import tf_transformations

# CALCULATION MODULES
import math


def euler_to_quaternion(roll, pitch, yaw):
        roll /= 2.0
        pitch /= 2.0
        yaw /= 2.0
        ci = math.cos(roll)
        si = math.sin(roll)
        cj = math.cos(pitch)
        sj = math.sin(pitch)
        ck = math.cos(yaw)
        sk = math.sin(yaw)
        cc = ci * ck
        cs = ci * sk
        sc = si * ck
        ss = si * sk
        return [
                cj*sc - sj*cs, # x
                cj*ss + sj*cc, # y
                cj*cs - sj*sc, # z
                cj*cc + sj*ss  # w
        ]
        
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