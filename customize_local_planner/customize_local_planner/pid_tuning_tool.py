# ROS MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path

# CALCULATION MODULES
import math

# HELPER MODULES
from .untilit import *

# GLOBALS
tuningStraight=False


class PidTuningPublisher(Node):

    def __init__(self):
        super().__init__('PidTuningPublisher')
        # Publishers
        self.tuning_local_plan_publisher = self.create_publisher(
            Path, 
            "local_plan", 
            10
        )
        self.tuning_is_pure_pursuit_controller_mode_publisher = self.create_publisher(
            Bool, 
            "is_pure_pursuit_controller_mode", 
            10
        )
        self.tuning_is_autonomous_mode_publisher = self.create_publisher(
            Bool, 
            "is_autonomous_mode", 
            10
        )
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 
            "odometry/global", 
            self.globalOdometryCallback, 
            10
        )
        # Timers
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.tuning_plan_publisher
        )
        # Vars
        self.latest_odom = None

    def tuning_plan_publisher(self):
        if self.latest_odom == None:
            return
        global tuningStraight
        publishPath = Path()

        msg = Bool()
        msg.data = True
        self.tuning_is_autonomous_mode_publisher.publish(msg)
        msg = Bool()
        msg.data = False
        self.tuning_is_pure_pursuit_controller_mode_publisher.publish(msg)
        
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
