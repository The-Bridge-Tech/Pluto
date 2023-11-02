

# create a function that get the relative x,y position of each gps point


# assume a given, know gps point represent the x,y in the map
from typing import List, Tuple
from geographiclib.geodesic import Geodesic

from math import sin, atan2, sqrt, cos
import math

import numpy as np
# import matplotlib.pyplot as plt



def angle_between_2_vector(x1, y1, x2, y2):
    # https://math.stackexchange.com/questions/317874/calculate-the-angle-between-two-vectors/1047350#1047350
    
    dot = x1*x2 + y1*y2      # Dot product between [x1, y1] and [x2, y2]
    det = x1*y2 - y1*x2      # Determinant
    angle = math.atan2(det, dot)  # atan2(y, x) or atan2(sin, cos)
    
    return angle 

def calculate_yaw_offset(point1X, point1Y, point2X, point2Y):
    straightX = 1;
    straightY = 0;

    y = (point2Y - point1Y) * -1;
    x = point2X - point1X;


    return angle_between_2_vector(straightY, straightX, y, x);



# modify base on https://github.com/danielsnider/gps_goal
def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
  # Calculate distance and azimuth between GPS points
  geod = Geodesic.WGS84  # define the WGS84 ellipsoid
  g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
  hypotenuse = distance = g['s12'] # access distance

  azimuth = g['azi1']


  # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
  # Convert azimuth to radians
  azimuth = math.radians(azimuth)
  x = adjacent = math.cos(azimuth) * hypotenuse
  y = opposite = math.sin(azimuth) * hypotenuse


  return x, y


def convert_gps_point_to_map_frame(origin_gps_data: Tuple[float, float], gps_coordinates: List[Tuple[float, float]]):
    gps_coordinate_in_map_frame :List[Tuple[float]]= []
    for point in gps_coordinates:
        
        gps_coordinate_in_map_frame.append(calc_goal(origin_gps_data[0], origin_gps_data[1], point[0], point[1]))
    
    return gps_coordinate_in_map_frame
# can either convert ENU to NED, or vice versa
def ENU_NED_CONVERSION(points):
    res=[]
    for p in points:
        res.append([p[1], p[0]])
    return res


import math


def generate_rectangle_area(lower_left, upper_left, width, decimal_accuracy = 3):
    
    # first round lower_left and upper_left
    lower_left = [round(lower_left[0], decimal_accuracy), round(lower_left[1], decimal_accuracy) ]
    upper_left = [round(upper_left[0], decimal_accuracy), round(upper_left[1], decimal_accuracy)]
    
    # 1. calculate the vector lower_left to upper_left
    
    left_lower_to_upper_vector_direction = [ upper_left[0] - lower_left[0], upper_left[1] - lower_left[1]]# x, y vector
    left_lower_to_upper_vector_offset = lower_left
    
    left_lower_to_upper_vector_mag = math.sqrt( left_lower_to_upper_vector_direction[0]**2+ left_lower_to_upper_vector_direction[1]**2  )
    left_lower_to_upper_unit_vector = [ left_lower_to_upper_vector_direction[0]/left_lower_to_upper_vector_mag, left_lower_to_upper_vector_direction[1]/left_lower_to_upper_vector_mag]
    
    left_lower_to_upper_angle_offset = math.atan2( left_lower_to_upper_vector_direction[1], left_lower_to_upper_vector_direction[0])# offset from the x, y -axis in cartesian grid
    
    
    # calculate vector of lower_right to lower_left
    left_lower_to_right_lower_angle_offset = left_lower_to_upper_angle_offset - (90 * (math.pi/180)) # since is clockwise 90 offset from the vector
    left_lower_to_right_lower_unit_vector = [cos(left_lower_to_right_lower_angle_offset) , sin(left_lower_to_right_lower_angle_offset)]
    left_lower_to_right_lower_vector_direction = [ left_lower_to_right_lower_unit_vector[0]* width, left_lower_to_right_lower_unit_vector[1]* width]
    left_lower_to_right_lower_vector_offset = lower_left
    
    lower_right = [ round(left_lower_to_right_lower_vector_direction[0] +left_lower_to_right_lower_vector_offset[0], decimal_accuracy ),
                   round(left_lower_to_right_lower_vector_direction[1] +left_lower_to_right_lower_vector_offset[1] , decimal_accuracy ),
                   ]
    
    # calculate vector of lower_right_to_upper_right
    right_lower_to_right_upper_angle_offset = left_lower_to_right_lower_angle_offset + (90 * (math.pi/180))
    right_lower_to_right_upper_unit_vector = [  cos(right_lower_to_right_upper_angle_offset), sin(right_lower_to_right_upper_angle_offset) ] 
    right_lower_to_right_upper_vector_direction = [  right_lower_to_right_upper_unit_vector[0] *  left_lower_to_upper_vector_mag, right_lower_to_right_upper_unit_vector[1] *  left_lower_to_upper_vector_mag ]
    right_lower_to_right_upper_Vector_offset = lower_right
    
    upper_right = [round(right_lower_to_right_upper_vector_direction[0] + right_lower_to_right_upper_Vector_offset[0], decimal_accuracy),
                   round(right_lower_to_right_upper_vector_direction[1] + right_lower_to_right_upper_Vector_offset[1],decimal_accuracy),
                   ]
    
    
    # calculate vector of upper_left to upper_right
    # use it for verification, to ensure correct calculation
    left_upper_to_right_upper_angle_offset = left_lower_to_upper_angle_offset - (90 * (math.pi/180))
    left_upper_to_right_upper_unit_vector = [ cos(left_upper_to_right_upper_angle_offset), sin(left_upper_to_right_upper_angle_offset)]
    left_upper_to_right_upper_vector_direction  =  [left_upper_to_right_upper_unit_vector[0]*width, left_upper_to_right_upper_unit_vector[1]*width]
    left_upper_to_right_upper_vector_offset  = upper_left
    
    upper_right2 = [round(left_upper_to_right_upper_vector_direction[0] + left_upper_to_right_upper_vector_offset[0],decimal_accuracy),
                    round(left_upper_to_right_upper_vector_direction[1] + left_upper_to_right_upper_vector_offset[1], decimal_accuracy),
                    
                    ]
    
    if( upper_right[0] == upper_right2[0]  and upper_right[1] == upper_right2[1]):
        return [lower_left, upper_left, lower_right, upper_right]
    else:
        return []
    
      

    
    

    
    


# gps_coord = [(34.84146125813767, -82.41180697265199), 
#              (34.84137966472769, -82.4116538358424),
#              (34.841335191235274, -82.41191397866223),
#              (34.84126384965586, -82.4117585774134)  ]
# gps_coord2 = [(34.84147118693877, -82.41181223236929),(34.84132967900098, -82.41191840846817),
#               (34.84126385924537, -82.41177056238641),(34.84137490010656, -82.41165924608055)]

# origin_data = (34.84135964592696, -82.41177886103307)


# lower_left_gps = (34.84133285149428, -82.41192955644298)
# upper_left_gps = (34.84147918512389, -82.41181254325562)



# x_y_coord = convert_gps_point_to_map_frame( origin_data, [lower_left_gps, upper_left_gps])

# x_y_coord = ENU_NED_CONVERSION(x_y_coord)

# res1 =  generate_rectangle_area(x_y_coord[0], x_y_coord[1], 25)
# res1 = ENU_NED_CONVERSION(res1)
# print(res1)

