import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

from math import cos, sin
import math
import numpy as np


def quaternion_from_euler(ai:float, aj:float, ak:float):
    """
    Convert Euler to Quaternion.
    See https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Broadcaster-Py.html
    
    Parameters
    ----------
    ai : float
        roll
    aj : float
        pitch
    ak : float
        yaw

    Returns
    -------
    list[float]

    """
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

class OdometryCalculation(Node):

    def __init__(self):
        super().__init__('odometry_calculation')

        self.define_parameters()
        self.PUBLISH_RATE = self.get_parameter('PUBLISH_RATE').get_parameter_value().integer_value
        self.X_ACCURACY = self.get_parameter('X_ACCURACY').get_parameter_value().double_value
        self.Y_ACCURACY = self.get_parameter('Y_ACCURACY').get_parameter_value().double_value
        self.Z_ACCURACY = self.get_parameter('Z_ACCURACY').get_parameter_value().double_value
        self.ROLL_ACCURACY = self.get_parameter('ROLL_ACCURACY').get_parameter_value().double_value
        self.PITCH_ACCURACY = self.get_parameter('PITCH_ACCURACY').get_parameter_value().double_value
        self.YAW_ACCURACY = self.get_parameter('YAW_ACCURACY').get_parameter_value().double_value
        self.VELOCITY_X_ACCURACY = self.get_parameter('VELOCITY_X_ACCURACY').get_parameter_value().double_value
        self.VELOCITY_Y_ACCURACY = self.get_parameter('VELOCITY_Y_ACCURACY').get_parameter_value().double_value
        self.VELOCITY_Z_ACCURACY = self.get_parameter('VELOCITY_Z_ACCURACY').get_parameter_value().double_value
        self.VELOCITY_ROLL_ACCURACY = self.get_parameter('VELOCITY_ROLL_ACCURACY').get_parameter_value().double_value
        self.VELOCITY_PITCH_ACCURACY = self.get_parameter('VELOCITY_PITCH_ACCURACY').get_parameter_value().double_value
        self.VELOCITY_YAW_ACCURACY = self.get_parameter('VELOCITY_YAW_ACCURACY').get_parameter_value().double_value
        
        time_period = 1/self.PUBLISH_RATE
        self.differential_raw_odometry_publisher = self.create_publisher(Odometry, 'differential_raw_odometry', 10)
        self.differential_raw_odometry_timer = self.create_timer(time_period, self.calculate_odometry_callback)
        self.differential_odometry = Odometry()
        
        self.x_accuracy = 0.3 
        self.y_accuracy = 0.3
        self.theta_accurcay = 0.1
        
        self.velocity_x_accuracy = 0.3
        self.velocity_y_accuracy = 0.3
        self.theta_velocity_accuracy = 0.1
        
        # subscribe to cmd_vel 
        self.cmd_vel_subscription = self.create_subscription(
            TwistStamped, 'differential_raw_twist', self.listen_cmd_vel_callback, 10
        )
        self.cmd_vel_subscription
        
        
        
        
        self.x = 0
        self.y = 0
        self.theta= 0
        self.last_twist = None
        self.current_twist = None


    
    
    def define_parameters(self)->None:
        self.declare_parameter('PUBLISH_RATE',30)
        self.declare_parameter('X_ACCURACY',0.3)
        self.declare_parameter('Y_ACCURACY',0.3)
        self.declare_parameter('Z_ACCURACY',0.0)
        self.declare_parameter('ROLL_ACCURACY',0.0)
        self.declare_parameter('PITCH_ACCURACY',0.0)
        self.declare_parameter('YAW_ACCURACY',0.1)
        self.declare_parameter('VELOCITY_X_ACCURACY',0.3)
        self.declare_parameter('VELOCITY_Y_ACCURACY',0.3)
        self.declare_parameter('VELOCITY_Z_ACCURACY',0)
        self.declare_parameter('VELOCITY_ROLL_ACCURACY',0.0)
        self.declare_parameter('VELOCITY_PITCH_ACCURACY',0.0)
        self.declare_parameter('VELOCITY_YAW_ACCURACY',0.1)
        
    
    def listen_cmd_vel_callback(self, message:TwistStamped)->None:
        if(self.last_twist is None):
            # initalize twist to be the same at first
            self.last_twist = message
            self.current_twist = message
        else:
            self.last_twist = self.current_twist
            self.current_twist = message
        

    def calculate_odometry_callback(self)->None:

        if(self.current_twist.header.stamp.sec == 0 and self.current_twist.header.stamp.nanosec == 0):
            #/clock is not ready yet
            return
        else:
            time_difference_in_second:float =(self.current_twist.header.stamp.sec + self.current_twist.header.stamp.nanosec/(10**9)  )
            - (self.last_twist.header.stamp.sec + self.last_twist.header.stamp.nanosec/(10**9) )

            if(time_difference_in_second >1 ):
                raise ValueError("Time difference between /differential_twist is greater than 1 seconds {0}-{1}  {2}-{3}  {4}", self.last_twist.header.stamp.sec, self.last_twist.header.stamp.nanosec,
                                 self.current_twist.header.stamp.sec, self.current_twist.header.stamp.nanosec, time_difference_in_second)
            
        
            
            delta_s = self.current_twist.twist.linear.x * time_difference_in_second
            delta_theta = self.current_twist.twist.angular.z * time_difference_in_second
            
            #https://github.com/Sollimann/CleanIt/blob/main/autonomy/src/slam/README.md
            
            self.x += delta_s * cos(self.theta + delta_theta/2)
            self.y += delta_s * sin(self.theta+ delta_theta/2)
            self.theta += delta_theta
            
        
            self.differential_odometry.header.stamp = self.current_twist.header.stamp
            self.differential_odometry.header.frame_id = "odom"
            self.differential_odometry.child_frame_id = "base_link"
            
            self.differential_odometry.pose.pose.position.x = self.x
            self.differential_odometry.pose.pose.position.y = self.y
            self.differential_odometry.pose.pose.position.z = 0
            
            
            quaternion = quaternion_from_euler(0,0, self.theta)
            self.differential_odometry.pose.pose.orientation.x = quaternion[0]
            self.differential_odometry.pose.pose.orientation.y = quaternion[1]
            self.differential_odometry.pose.pose.orientation.z = quaternion[2]
            self.differential_odometry.pose.pose.orientation.w = quaternion[3]
            
            self.differential_odometry.pose.covariance = [self.X_ACCURACY**2,   0.0,        0.0,                0.0,                    0.0,                        0.0,
                                                            0.0,    self.Y_ACCURACY**2,     0.0,                0.0,                    0.0,                        0.0,
                                                            0.0,                0.0,       self.Z_ACCURACY**2 , 0.0,                    0.0,                        0.0,
                                                            0.0,                0.0,        0.0,                self.ROLL_ACCURACY**2,  0.0,                        0.0,
                                                            0.0,                0.0,        0.0,                0.0,                    self.PITCH_ACCURACY**2,     0.0,
                                                            0.0,                0.0,        0.0,                0.0,                    0.0,                        self.YAW_ACCURACY**2]

            
            
            # the velocity parameter
            
            self.differential_odometry.twist.twist.linear.x = self.current_twist.twist.linear.x
            self.differential_odometry.twist.twist.linear.y = self.current_twist.twist.linear.y
            self.differential_odometry.twist.twist.linear.z = self.current_twist.twist.linear.z
            self.differential_odometry.twist.twist.angular.x = self.current_twist.twist.angular.x
            self.differential_odometry.twist.twist.angular.y = self.current_twist.twist.angular.y
            self.differential_odometry.twist.twist.angular.z = self.current_twist.twist.angular.z
            
            self.differential_odometry.twist.covariance = [self.VELOCITY_X_ACCURACY**2,     0.0,                             0.0,                           0.0,                                0.0,                                    0.0,
                                                            0.0,                            self.VELOCITY_Y_ACCURACY**2,     0.0,                           0.0,                                0.0,                                    0.0,
                                                            0.0,                            0.0,                            self.VELOCITY_Z_ACCURACY**2 ,   0.0,                                0.0,                                    0.0,
                                                            0.0,                            0.0,                             0.0,                           self.VELOCITY_ROLL_ACCURACY**2,     0.0,                                    0.0,
                                                            0.0,                            0.0,                             0.0,                           0.0,                                self.VELOCITY_PITCH_ACCURACY**2,        0.0,
                                                            0.0,                            0.0,                             0.0,                           0.0,                                0.0,                                    self.VELOCITY_YAW_ACCURACY**2]
            
            
            self.differential_raw_odometry_publisher.publish(self.differential_odometry)
            
                

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = OdometryCalculation()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()