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
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Bool
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

"""
Basic navigation demo to go to pose.
"""


import geopy.distance

coords_1 = (34.841386, -82.411834)
coords_2 = (34.841305, -82.411692)

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
            goal_pose.pose.position.x = geopy.distance.geodesic(self.latest_gps_coor, point).m
            goal_pose.pose.position.y = 0.0
            goal_pose.pose.orientation.w = 1.0

            # sanity check a valid path exists
            # path = navigator.getPath(initial_pose, goal_pose)

            self.navigator.goToPose(goal_pose)
            self.navigator_result()

    def MoveStraight(self, distance=3.0):
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
            # if feedback and i % 5 == 0:
            #     print('Estimated time of arrival: ' + '{0:.0f}'.format(
            #         Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
            #         + ' seconds.')

            #     # Some navigation timeout to demo cancellation
            #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
            #         self.navigator.cancelTask()

            #     # Some navigation request change to demo preemption
            #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #         goal_pose.pose.position.x = -3.0
            #         navigator.goToPose(goal_pose)

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
    main()
