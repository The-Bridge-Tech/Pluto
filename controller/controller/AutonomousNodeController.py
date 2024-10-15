#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

import time
from geographiclib.geodesic import Geodesic
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


from coverage_area_interface.srv import SelectSquare
from coverage_area_interface.srv import LoadMap
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np

import geopy.distance
from math import sin, cos, atan, sqrt, atan2
from .AutonomousNodeControllerHelperFunction import *


class AutonomousNodeController(Node):

    def __init__(self):
        super().__init__('autonomous_node_controller')

        self.autonomous_state_subscription = self.create_subscription(
            Bool, 'is_autonomous_mode', self.is_autonomous_state_listener, 1)
        self.is_autonomous_state_last_time = Bool()
        self.is_autonomous_state_last_time.data = False
        
        #self.request_to_load_map_client = self.create_client(LoadMap, "load_map_service")

        self.navigator = BasicNavigator()

        self.current_map_frame_odom = None
        self.map_frame_subscription = self.create_subscription(
            Odometry, "odometry/global", self.update_latest_map_odom, 10
        )
        self.initial_navigate_pose:PoseStamped = None

        self.detect_obstacle:Bool = Bool()  #TODO: topic later will update this
        self.detect_obstacle.data=False
        # Initialize the transform broadcaster to broadcast between "Map" and "Planner"
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.final_end_coverage_pose:PoseStamped=None
        self.final_start_coverage_pose:PoseStamped=None
        
        
        #TODO better method later
        self.hasMovedToOrigin = False  
        self.hasFinishedInitialization = False
        self.navigateTimer = self.create_timer(1, self.navigate_to_coverage_area_end_pose)
        self.is_pure_pursuit_mode_pub = self.create_publisher(Bool, "is_pure_pursuit_controller_mode", 10)

    def load_map_request(self):
        while not self.request_to_load_map_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = LoadMap.Request()
        self.req.map_index = 0
        self.future = self.request_to_load_map_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.final_end_coverage_pose = self.future.result().goal_end_pose
        print(self.final_end_coverage_pose)
        self.get_logger().info("done {0}".format(str(self.final_end_coverage_pose)))
        
    def update_latest_map_odom(self, message: Odometry):
        self.current_map_frame_odom = message

    def update_latest_current_gps_coordinate(self, message: NavSatFix):
        self.latest_gps_coor = message

    def is_autonomous_state_listener(self, mes: Bool):
        if (mes.data == True):
            if (self.is_autonomous_state_last_time.data == False):
                # ignoring
                self.is_autonomous_state_last_time.data = True
                if self.current_map_frame_odom != None:
                    self.generate_initial_pose()
                    self.initialize_all_navigation_lifecycle()
                    #self.load_map_request()
                    self.send_coverage_info_to_planner()
                    # self.final_end_coverage_pose =  self.send_coverage_info_to_planner()
                    self.hasFinishedInitialization = True

        else:
            self.is_autonomous_state_last_time.data = False

    def generate_initial_pose(self):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.current_map_frame_odom.pose.pose.position.x
        initial_pose.pose.position.y = self.current_map_frame_odom.pose.pose.position.y
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation = self.current_map_frame_odom.pose.pose.orientation
        self.initial_navigate_pose = initial_pose
        
    def initialize_all_navigation_lifecycle(self):

        
            # Set our demo's initial pose

        self.navigator.setInitialPose(self.initial_navigate_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.lifecycleStartup()
        time.sleep(10)  # wait for all node to finished

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()
    def uninitialized_all_navigation_lifecycle(self):
        self.navigator.lifecycleShutdown()
    def navigate_to_coverage_area_end_pose(self):
              
        if self.hasFinishedInitialization == False:
            pass
        else:
            if self.hasMovedToOrigin == False:
                
                mode =Bool()
                mode.data =False
                self.is_pure_pursuit_mode_pub.publish(mode)
                #1. first navigate to the point
                self.generate_initial_pose()
                path1 = self.navigator.getPath(self.initial_navigate_pose, self.final_start_coverage_pose, "Navfn")
                

                self.navigator.followPath(path=path1, controller_id="DWAFollower")
                

                self.navigator_result()
                self.hasMovedToOrigin = True #TODO: maybe change to a distance checker before saying is true
            else:
        
        
                self.generate_initial_pose()
                path = self.navigator.getPath(self.initial_navigate_pose,self.final_end_coverage_pose,"GridBased") # use default   #TODO: change later
                # smoothed_path = self.navigator.smoothPath(path=path)
                
                #TODO: later on add topic 
                if self.detect_obstacle.data == False:
                    mode =Bool()
                    mode.data =True
                    self.is_pure_pursuit_mode_pub.publish(mode)
 
                    self.navigator.followPath(path=path,controller_id="FollowPath")
                                   
                else:
                    mode =Bool()
                    mode.data =False
                    self.is_pure_pursuit_mode_pub.publish(mode)
                    self.navigator.followPath(path=path, controller_id="DWAFollower")
                    
                #self.navigator_result()

    def navigator_result(self):
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            pass
        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info(self.navigator.status)
            self.get_logger().info(result)
            self.get_logger().info('Goal has an invalid return status!')


    
    
    def send_coverage_info_to_planner(self):
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
        # self.future = self.select_area_client.call_async(self.req)
        # rclpy.spin_until_future_complete(self, self.future)
        # response_form_coverage= self.future.result()
        # print(response_form_coverage.goal_end_pose)
        # self.final_end_pose = response_form_coverage.goal_end_pose
        
        self.future = cli.call_async(self.req)
        rclpy.spin_until_future_complete(node2, self.future)
        
        #TODO: load map base on request
        res:PoseStamped = self.future.result().goal_end_pose
        print(res)
        self.final_end_coverage_pose = res
        self.final_start_coverage_pose = self.future.result().goal_start_pose

        
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
node2 = None
cli= None
def main():
    rclpy.init()
    global cli, node2
    autonomous_node_controller = AutonomousNodeController()
    node2 = rclpy.create_node('add_two_ints_client')

    cli = node2.create_client(SelectSquare, "/SelectSquare")
    rclpy.spin(autonomous_node_controller)
    # navigator = BasicNavigator()

    # #navigator.lifecycleShutdown()

    # # exit(0)
    autonomous_node_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
