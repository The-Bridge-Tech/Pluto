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
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path

from .untilit import *


tuningStraight=True
class PidTuningPublisher(Node):

    def __init__(self):
        super().__init__('PidTuningPublisher')
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

        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.tuning_plan_publisher)

        self.latest_odom = Odometry()
    def tuning_plan_publisher(self):
        
        global tuningStraight
        publishPath = Path()
        trueBool = Bool()
        trueBool.data=True
        falseBool = Bool()
        falseBool.data=False

        self.tuning_is_autonomous_mode_publisher.publish(trueBool)
        self.tuning_is_pure_pursuit_controller_mode_publisher.publish(falseBool)



        currentPosePoseStamp = PoseStamped()
        currentPosePoseStamp.pose = self.latest_odom.pose.pose
        goalPosePoseStamp = PoseStamped()
        tempPose = Pose()

        if(tuningStraight):


            
            tempPose.position.x = 5.0

        else:
            # means turning pid
            # move 90 degree to north
            tempPose.orientation.x = 0.0
            tempPose.orientation.y = 0.0
            tempPose.orientation.z = 0.7071068
            tempPose.orientation.w = 0.7071068

        goalPosePoseStamp.pose = tempPose
        publishPath.poses =  [currentPosePoseStamp,currentPosePoseStamp,
                                    goalPosePoseStamp,goalPosePoseStamp]

        self.tuning_local_plan_publisher.publish(publishPath)

    def globalOdometryCallback(self, odom:Odometry):
        self.latest_odom = odom

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PidTuningPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()