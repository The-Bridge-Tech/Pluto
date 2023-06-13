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
import math 

"""
Basic navigation demo to go to pose.
"""



def calc_goal( origin_lat, origin_long, goal_lat, goal_long):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
    hypotenuse = distance = g['s12'] # access distance
    
    
    
    degree_to_rad = float(math.pi / 180.0)

    d_lat = (goal_lat - origin_lat) * degree_to_rad
    d_long = (goal_long - origin_long) * degree_to_rad

    a = pow(sin(d_lat / 2), 2) + cos(origin_lat * degree_to_rad) * cos(goal_lat * degree_to_rad) * pow(sin(d_long / 2), 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    km = 6367 * c
    mi = 3956 * c

    #self.get_logger().info("The distance from the origin to the goal is {:.3f} m.".format(distance))
    azimuth = 90- g['azi1'] #https://answers.ros.org/question/219182/how-to-determine-yaw-angle-from-gps-bearing/
    #self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
    # Convert azimuth to radians
    print(azimuth)
    azimuth = math.radians(azimuth)
    x = adjacent = math.cos(azimuth) * hypotenuse
    y = opposite = math.sin(azimuth) * hypotenuse
    #self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

    return x, y

import geopy.distance
from math import sin, cos, atan, sqrt, atan2
coords_1 = (34.841398, -82.411860)
coords_2 = (34.841323, -82.411724)

coord_list = [coords_1, coords_2]



class AutonomousNodeController(Node):

    def __init__(self):
        super().__init__('autonomous_node_controller')

        self.autonomous_state_subscription = self.create_subscription(
            Bool, 'is_autonomous_mode', self.is_autonomous_state_listener, 10)
        self.is_autonomous_state_last_time = Bool()
        self.is_autonomous_state_last_time.data = False
        self.navigator = None

        self.gps_coor_subscription = self.create_subscription(
            NavSatFix, "/fix", self.update_latest_current_gps_coordinate, 10
        )

        self.latest_gps_coor: NavSatFix = None

        self.current_map_frame_odom = None
        self.map_frame_subscription = self.create_subscription(
            Odometry, "odometry/global", self.update_latest_map_odom, 10
        )

    
    
    def calc_goal(self, origin_lat, origin_long, goal_lat, goal_long):
        # Calculate distance and azimuth between GPS points
        geod = Geodesic.WGS84  # define the WGS84 ellipsoid
        g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long) # Compute several geodesic calculations between two GPS points 
        hypotenuse = distance = g['s12'] # access distance
        
        
        
        degree_to_rad = float(math.pi / 180.0)

        d_lat = (goal_lat - origin_lat) * degree_to_rad
        d_long = (goal_long - origin_long) * degree_to_rad

        a = pow(sin(d_lat / 2), 2) + cos(origin_lat * degree_to_rad) * cos(goal_lat * degree_to_rad) * pow(sin(d_long / 2), 2)
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        km = 6367 * c
        mi = 3956 * c

        self.get_logger().info("The distance from the origin to the goal is {:.3f} m.".format(distance))
        azimuth = 90-g['azi1'] # yaw offset of ned to enu 
                            # https://answers.ros.org/question/219182/how-to-determine-yaw-angle-from-gps-bearing/
        self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

        # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
        # Convert azimuth to radians
        azimuth = math.radians(azimuth)
        x = adjacent = math.cos(azimuth) * hypotenuse
        y = opposite = math.sin(azimuth) * hypotenuse
        self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

        return x, y
    def update_latest_map_odom(self, message: Odometry):
        self.current_map_frame_odom = message

    def update_latest_current_gps_coordinate(self, message: NavSatFix):
        self.latest_gps_coor = message

    def is_autonomous_state_listener(self, mes: Bool):
        if (mes.data == True):
            if (self.is_autonomous_state_last_time.data == False):
                # ignoring
                self.is_autonomous_state_last_time.data = True
                self.get_logger().info("here1")
                self.MoveStraight()
        else:
            self.is_autonomous_state_last_time.data = False
            self.get_logger().info("ignoring")

        # if(mes.data == True):
        #     if(self.is_autonomous_state_last_time == False):
        #         self.is_autonomous_state_last_time.data = True
        #         self.get_logger().info("here1")
        #         self.navigateToTwoPoint()
        #         self.is_autonomous_state_last_time.data = False
        #     else:
        #         self.get_logger().info("ignoring")

    def navigateToGPSPoint(self):
        self.navigator = BasicNavigator()
        # Set our demo's initial pose
        # TODO: MAKE sure they all initalize first?
        time.sleep(5)
        
        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # initial_pose.pose.position.x = 0.0
        # initial_pose.pose.position.y = 0.0
        # initial_pose.pose.orientation.w = 1.0
        # initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(self.current_map_frame_odom.pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.lifecycleStartup()
        time.sleep(10)

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

        #
        
        for point in coord_list:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            #geopy.distance.geodesic(self.latest_gps_coor, point).
            
            x,y = self.calc_goal(self.latest_gps_coor.latitude, self.latest_gps_coor.longitude,point[0], point[1])
            goal_pose.pose.position.x =x
            goal_pose.pose.position.y = y
            goal_pose.pose.orientation.w = 1.0

            # sanity check a valid path exists
            # path = navigator.getPath(initial_pose, goal_pose)

            self.navigator.goToPose(goal_pose)
            self.navigator_result()

    def MoveStraight(self, distance=5.0):
        "This should be use as a test function"
        self.navigator = BasicNavigator()
        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0
        initial_pose.pose.orientation.w = 0.0
        self.navigator.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # navigator.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.lifecycleStartup()
        time.sleep(10)

        # If desired, you can change or load the map as well
        # navigator.changeMap('/path/to/map.yaml')

        # You may use the navigator to clear or obtain costmaps
        # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
        # global_costmap = navigator.getGlobalCostmap()
        # local_costmap = navigator.getLocalCostmap()

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        # geopy.distance.geodesic(coords_1, coords_2).m
        goal_pose.pose.position.x = distance
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.orientation.w = 1.0

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        self.navigator.goToPose(goal_pose)
        self.navigator_result()

    def navigator_result(self):
        i = 0
        while not self.navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + ' seconds.')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.navigator.cancelTask()

                # # Some navigation request change to demo preemption
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                #     goal_pose.pose.position.x = -3.0
                #     navigator.goToPose(goal_pose)

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

def main():
    rclpy.init()

    autonomous_node_controller = AutonomousNodeController()
    rclpy.spin(autonomous_node_controller)
    # navigator = BasicNavigator()

    # #navigator.lifecycleShutdown()

    # # exit(0)
    autonomous_node_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    x,y = calc_goal(coords_1[0], coords_1[1], coords_2[0], coords_2[1])
    print(x, y)
    main()
