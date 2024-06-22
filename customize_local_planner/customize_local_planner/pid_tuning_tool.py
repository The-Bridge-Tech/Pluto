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


tuningStraight=False
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

        self.latest_odom = None
    def tuning_plan_publisher(self):
        

        if self.latest_odom == None:
            return
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


            
            tempPose.position.x = 5.0 + currentPosePoseStamp.pose.position.x

        else:
            # get a position 90 degree from the current position with positionx +=1

            current_heading = calculateEulerAngleFromOdometry(self.latest_odom)

            
            current_heading +=90 # move 90 degree turn

            self.get_logger().info(f"The heading after +90 is {current_heading}")
            tempPose.position.x = currentPosePoseStamp.pose.position.x+ math.cos(math.radians( current_heading))*1

            tempPose.position.y =  currentPosePoseStamp.pose.position.y+ math.sin( math.radians(current_heading))*1
            

        goalPosePoseStamp.pose = tempPose
        publishPath.poses =  [currentPosePoseStamp,
                                    goalPosePoseStamp,goalPosePoseStamp,goalPosePoseStamp]

        self.tuning_local_plan_publisher.publish(publishPath)

    def globalOdometryCallback(self, odom:Odometry):
        if self.latest_odom == None:
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