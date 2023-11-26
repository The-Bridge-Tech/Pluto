# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .helperFunction import *
import tf2_ros
import sys
import rclpy
from rclpy.node import Node



from coverage_area_interface.srv import SelectSquare
from coverage_area_interface.srv import LoadMap
# include "rclcpp/rclcpp.hpp"

# This function is a stripped down version of the code in
# https://github.com/matthew-brett/transforms3d/blob/f185e866ecccb66c545559bc9f2e19cb5025e0ab/transforms3d/euler.py
# Besides simplifying it, this version also inverts the order to return x,y,z,w, which is

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


class CoveragePlannerClient(Node):

    def __init__(self):
        super().__init__('CoveragePlannerClient')

        # Initialize the transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # publisher to publish the final end goal of the coverage area
        #TODO: maybe turn this into a service topic later?
        self.final_goal_publisher = self.create_publisher(PoseStamped, "/coverage_area_end_pose",10)
        self.timer = self.create_timer(0.5,self.coverage_area_end_pose_timer )
        
        self.final_end_pose:PoseStamped = None
        
        # # # publish every 0.5 second
        # # timer_period = 0.1  # seconds
        # # self.timer = self.create_timer(
        # #     timer_period, self.publish_map_to_planner_transform)

        # # TODO: load the data from a yaml file later
        # # TODO: convert from gps to map_frame
        # # TODO: then somehow make sure / normalize the area to be a square/rectangle.
        # map_points = [(-1.0, 0.0), (2.0, 0.0),
        #               (-1.0, 2.0), (2.0, 2.0)]
        # # map_points = [(-1.5, 0.0), (2.0, 0.0),
        # #               (-1.5, -2.0), (2.0, -2.0)]
        # # map_points = [(0.0, 0.0), (1.0, -1.0),
        # #                         (-1.0, -1.0), (0.0, -2.0)]
        
        
        # #TODO: in the future, configure them to be parameter
        # self.origin_data = (34.84135964592696, -82.41177886103307)


        # self.lower_left_gps = (34.84133285149428, -82.41192955644298)
        # self.upper_left_gps = (34.84147918512389, -82.41181254325562)
        # # calculate those into map_frame x, y meters
        
        # x_y_coord = ENU_NED_CONVERSION (convert_gps_point_to_map_frame(self.origin_data, [self.lower_left_gps, self.upper_left_gps])) # convert from ENU to NED on the result
        # #map_points = ENU_NED_CONVERSION(generate_rectangle_area(x_y_coord[0], x_y_coord[1], 15)) # convert back from NED to ENU
        # print(map_points)
        #         # result order is 
        #         # lower_left, upper_left, lower_right, upper_right
        # self.select_area_client = self.create_client(SelectSquare, "/SelectSquare")


        # # 1. calculate the yaw offset base on one side
        # self.yaw_offset = calculate_yaw_offset(
        #     map_points[0][0], map_points[0][1], map_points[1][0], map_points[1][1])
        # # self.yaw_offset= 0
        # self.get_logger().info("The angle offset is" + str(self.yaw_offset))

        # # calculate the center x, and y
        # self.x_offset = (map_points[0][0] + map_points[1]
        #                  [0] + map_points[2][0] + map_points[3][0]) / 4
        # self.y_offset = (map_points[0][1] + map_points[1]
        #                  [1] + map_points[2][1] + map_points[3][1]) / 4

        # self.publish_map_to_planner_transform()  # publish once

        # # debug for transform
        # self.tf_buffer = Buffer()
        # self.tf_listener = TransformListener(self.tf_buffer, self)


        # while not self.select_area_client.wait_for_service(timeout_sec=5.0):
        #     self.get_logger().warn("SelectArea service is not available, trying again!")
        # self.req = SelectSquare.Request()
        
        # # put the info 
        # self.req.lower_left_x = map_points[0][0]
        # self.req.lower_left_y = map_points[0][1]
        # self.req.upper_left_x = map_points[1][0]
        # self.req.upper_left_y = map_points[1][1]
        # self.req.lower_right_x = map_points[2][0]
        # self.req.lower_right_y = map_points[2][1]
        # self.req.upper_right_x = map_points[3][0]
        # self.req.upper_right_y = map_points[3][1]
        # self.req.frame = "map"
        
        
        
        # server for load map request
        #self.load_map_service = self.create_service(LoadMap,"load_map_service", self.publish_coverage_info)
        
        


        # TODO: load the data from a yaml file later
        # TODO: convert from gps to map_frame
        # TODO: then somehow make sure / normalize the area to be a square/rectangle.
        map_points = [(-1.0, 0.0), (2.0, 0.0),
                      (-1.0, 2.0), (2.0, 2.0)]
        # map_points = [(-1.5, 0.0), (2.0, 0.0),
        #               (-1.5, -2.0), (2.0, -2.0)]
        # map_points = [(0.0, 0.0), (1.0, -1.0),
        #                         (-1.0, -1.0), (0.0, -2.0)]
        
        
        #TODO: in the future, configure them to be parameter
        self.origin_data = (34.84135964592696, -82.41177886103307)


        self.lower_left_gps = (34.84133285149428, -82.41192955644298)
        self.upper_left_gps = (34.84147918512389, -82.41181254325562)
        # calculate those into map_frame x, y meters
        
        x_y_coord = ENU_NED_CONVERSION (convert_gps_point_to_map_frame(self.origin_data, [self.lower_left_gps, self.upper_left_gps])) # convert from ENU to NED on the result
        #map_points = ENU_NED_CONVERSION(generate_rectangle_area(x_y_coord[0], x_y_coord[1], 15)) # convert back from NED to ENU
        print(map_points)
                # result order is 
                # lower_left, upper_left, lower_right, upper_right
        self.select_area_client = self.create_client(SelectSquare, "/SelectSquare")


        # 1. calculate the yaw offset base on one side
        self.yaw_offset = calculate_yaw_offset(
            map_points[0][0], map_points[0][1], map_points[1][0], map_points[1][1])
        # self.yaw_offset= 0
        self.get_logger().info("The angle offset is" + str(self.yaw_offset))

        # calculate the center x, and y
        self.x_offset = (map_points[0][0] + map_points[1]
                         [0] + map_points[2][0] + map_points[3][0]) / 4
        self.y_offset = (map_points[0][1] + map_points[1]
                         [1] + map_points[2][1] + map_points[3][1]) / 4

        self.publish_map_to_planner_transform()  # publish once

        # debug for transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        while not self.select_area_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("SelectArea service is not available, trying again!")
        self.req = SelectSquare.Request()
        
        # put the info 
        self.req.lower_left_x = map_points[0][0]
        self.req.lower_left_y = map_points[0][1]
        self.req.upper_left_x = map_points[1][0]
        self.req.upper_left_y = map_points[1][1]
        self.req.lower_right_x = map_points[2][0]
        self.req.lower_right_y = map_points[2][1]
        self.req.upper_right_x = map_points[3][0]
        self.req.upper_right_y = map_points[3][1]
        self.req.frame = "map"
        self.future = self.select_area_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        response_form_coverage= self.future.result()
        print(response_form_coverage.goal_end_pose)
        self.final_end_pose = response_form_coverage.goal_end_pose
        
        # self.future = cli.call_async(self.req)
        # rclpy.spin_until_future_complete(node2, self.future)
        
        # #TODO: load map base on request
        # res:PoseStamped = self.future.result().goal_end_pose
        # print(res)
        # response.goal_end_pose = res
        # self.get_logger().info("done")
        # # now, send the goal_end_pose as a co
        # return response
        
    def publish_map_to_planner_transform(self):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = "planner"

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.x_offset
        t.transform.translation.y = self.y_offset
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, self.yaw_offset)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def coverage_area_end_pose_timer(self):
        if self.final_end_pose != None:
            self.final_goal_publisher.publish(self.final_end_pose)

node2 = None
cli= None
def main():
    global cli, node2
    rclpy.init()
    node = CoveragePlannerClient()
    
    node2 = rclpy.create_node('add_two_ints_client')

    cli = node2.create_client(SelectSquare, "/SelectSquare")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
