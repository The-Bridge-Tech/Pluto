import rclpy
from rclpy.node import Node

from .controller import Controller
from .move_straight_pid_controller import MovingStraightPIDController
from .turning_pid_controller import TurningPIDController
from .stop_controller import Stop
from .untilit import *
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from math import atan2, pi, sin, cos, atan
from std_msgs.msg import UInt32, Bool
import tf_transformations
import tf2_ros
# TODO: need to change to standard message type later
from std_msgs.msg import Float32

# http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path






GPS_POINTs = [(34.841445002851714, -82.4118093380928), (34.841371505588576, -82.4116665085601),
              (34.84127181402163, -82.41174133570041), (34.84134081774882, -82.41188586505363)]
class PhaseOneDemo(Node):

    def __init__(self):
        super().__init__('PhaseOneDemo')
        self.tuning_local_plan_publisher = self.create_publisher(
            Path, "local_plan",  10
        )


        self.tuning_is_pure_pursuit_controller_mode_publisher = self.create_publisher(

            Bool, "is_pure_pursuit_controller_mode", 10
        )
        self.tuning_is_autonomous_mode_publisher = self.create_publisher(
            Bool, "is_autonomous_mode" , 10
        )


        self.odom_sub = self.create_subscription(
            Odometry, "odometry/global", self.globalOdometryCallback, 10
        )


        self.gps_sub = self.create_subscription(
            NavSatFix, "/fix", self.gps_fix_callback, 10
        )

        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.tuning_plan_publisher)

        self.latest_odom: Odometry = None
        self.initial_gps: NavSatFix = None

        self.pose_to_navigate = []
        self.current_pose_to_navigate_index = 0
    def tuning_plan_publisher(self):
        
        if self.latest_odom == None or self.initial_gps == None:
            self.get_logger().info("Waiting for odom and gps data to be initalized.")
            return 
        
        # when have data
        if len(self.pose_to_navigate) == 0:
            for gps_tuple in GPS_POINTs:
                (goa_x, goal_y) = calc_goal(origin_lat=self.initial_gps.latitude,
                          origin_long=self.initial_gps.longitude,
                          goal_lat=gps_tuple[0],
                          goal_long=gps_tuple[1]
                          )

                # self.get_logger().info(f"Distance to navigate to from origin{self.initial_gps.latitude}, {self.initial_gps.longitude} to 
                #                        goal{gps_tuple} with distance {goa_x},{goal_y}")
                self.pose_to_navigate.append((goa_x,goal_y))
                
        
        
        
        publishPath = Path()
        trueBool = Bool()
        trueBool.data=True
        falseBool = Bool()
        falseBool.data=False

        self.tuning_is_autonomous_mode_publisher.publish(trueBool)
        self.tuning_is_pure_pursuit_controller_mode_publisher.publish(falseBool)

        
        current_pose = self.latest_odom.pose.pose.position
        next_pose = self.pose_to_navigate[self.current_pose_to_navigate_index]
        distance_remainig_from_goal = math.dist(
            
            (current_pose.x, current_pose.y),
            (next_pose[0], next_pose[1])
        )

     
        currentPosePoseStamp = PoseStamped()
        currentPosePoseStamp.pose = self.latest_odom.pose.pose



        goalPosePoseStamp = PoseStamped()
        tempPose = Pose()

        # if(tuningStraight):


            
        #     tempPose.position.x = 5.0

        # else:
        #     # means turning pid
        #     # move 90 degree to north
        #     tempPose.orientation.x = 0.0
        #     tempPose.orientation.y = 0.0
        #     tempPose.orientation.z = 0.7071068
        #     tempPose.orientation.w = 0.7071068
        if(distance_remainig_from_goal < 0.3):
            if(self.current_pose_to_navigate_index +1 < len(self.pose_to_navigate)):
                self.current_pose_to_navigate_index +=1
                next_pose = self.pose_to_navigate[self.current_pose_to_navigate_index]
            else:
                self.get_logger().info("At the end of pose!!")
        
        tempPose.position.x = next_pose[0]
        tempPose.position.y = next_pose[1]

        

        goalPosePoseStamp.pose = tempPose
        publishPath.poses =  [currentPosePoseStamp,
                                    goalPosePoseStamp,goalPosePoseStamp,goalPosePoseStamp]

        self.tuning_local_plan_publisher.publish(publishPath)
    
    def gps_fix_callback(self, gps_data: NavSatFix):
        if self.initial_gps == None:
                self.initial_gps = gps_data

    def globalOdometryCallback(self, odom:Odometry):
        self.latest_odom = odom



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PhaseOneDemo()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()