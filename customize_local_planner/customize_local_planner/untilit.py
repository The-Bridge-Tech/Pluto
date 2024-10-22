"""
Conversion Helper Methods
Authors: Joel Du Shouyu & Matthew Lauriault
"""


# ROS MODULES
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf_transformations

# CALCULATION MODULES
import math
import numpy as np
from geodesy.utm import fromLatLong


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

def angle_from_imu(msg: Imu):
        """Angle is returned in the range -180 to 180 degrees"""
        q = [
            msg.orientation.x, 
            msg.orientation.y, 
            msg.orientation.z, 
            msg.orientation.w
        ]
        # rpy = [roll, pitch, yaw]
        rpy = tf_transformations.euler_from_quaternion(q)
        # rpy[2] = yaw (orientation around the vertical axis)
        return math.degrees(rpy[2])

def euler_to_quaternion(roll, pitch, yaw):
    """Converts [roll, pitch, yaw] to [x, y, z, w]"""
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

def quaternion_to_euler(quaternion):
    """Converts [x, y, z, w] to [roll, pitch, yaw]"""
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


# modify base on https://github.com/danielsnider/gps_goal
def calculate_goal_xy(origin_lat, origin_lon, goal_lat, goal_lon):
    """Convert GPS (lat & lon) to UTM (easting & northing) relative to origin point."""
    # Convert origin lat & lon to easting (x) & northing (y)
    origin_utm = fromLatLong(
        longitude = origin_lon, 
        latitude = origin_lat
    )
    # Convert goal lat & lon to easting (x) & northing (y)
    goal_utm = fromLatLong(
        longitude = goal_lon, 
        latitude = goal_lat
    )
    # x-distance between origin and goal
    dx = goal_utm.easting - origin_utm.easting
    # y-distance between origin and goal
    dy = goal_utm.northing - origin_utm.northing
    return (dx, dy)

def meters_to_gps_degrees(meters: float, latitude: float) -> float:
    """Approximate conversion from a distance in meters to gps degrees"""
    # 1 degree of latitude is approximately 111,320 meters
    lat_degree = meters / 111320
    # 1 degree of longitude varies with latitude
    lon_degree = meters / (111320 * np.cos(np.radians(latitude)))
    return max(lat_degree, lon_degree)  # Use the larger value to ensure the circle is visible

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calculate the distance in meters between 2 gps points."""
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    # Radius of the Earth in meters
    R = 6371000
    distance = R * c
    return distance
