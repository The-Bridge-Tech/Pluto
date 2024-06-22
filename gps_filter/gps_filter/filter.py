import rclpy
from rclpy.node import Node


from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix 
from nav_msgs.msg import Odometry


POSITION_COVARIANCE = 1.5 # 1.5 meter off for the worst case scenario
VELOCITY_COVARIANCE = 0.1 #TODO: is this correct?
NED_FORM = False

class GPSFilter(Node):

    def __init__(self):
        super().__init__('gps_filter')
        
        self.gps_filtered_message = self.create_publisher(NavSatFix, "fix/filtered",10)
        self.gps_velocity_odom = self.create_publisher(TwistWithCovarianceStamped, '/twist/gps_vel',10)
        
        self.gps_fix_sub = self.create_subscription(NavSatFix, "/fix", self.update_gps_fix_covariance,10)
        self.gps_vel_sub = self.create_subscription(TwistStamped, "/vel", self.generate_gps_vel,10)
        # timer_period = 0.1  # seconds
        # self.timer = self.create_timer(timer_period, self.message_filter_callback)
        # self.i = 0
        
        self.last_gps_fix :NavSatFix = NavSatFix()
        self.last_gps_twist : TwistStamped = TwistStamped()
        
        
    def update_gps_fix_covariance(self, message:NavSatFix):
        self.last_gps_fix = message
        self.last_gps_fix.position_covariance = [POSITION_COVARIANCE **2,   0.0,                                        0.0,
                                                 0.0,                       POSITION_COVARIANCE**2, 0.0,
                                                 0.0,                       0.0,                    POSITION_COVARIANCE**2]
        
        self.last_gps_fix.position_covariance_type= NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        
        self.gps_filtered_message.publish(self.last_gps_fix)
        
    def generate_gps_vel(self, message:TwistStamped ):
        
        tempTwistedWithCovariance = TwistWithCovarianceStamped()
        
        tempTwistedWithCovariance.header = message.header
        if(not NED_FORM):
            tempTwistedWithCovariance.twist.twist = message.twist
        else:
            tempx = message.twist.linear.x
            tempy = message.twist.linear.y
            tempTwistedWithCovariance.twist.twist.linear.x = tempx
            tempTwistedWithCovariance.twist.twist.linear.y = tempy
            
        
        tempTwistedWithCovariance.twist.covariance = [VELOCITY_COVARIANCE **2,      0.0,                    0.0,                    0.0,                    0.0,                    0.0,
                                                      0.0,                          VELOCITY_COVARIANCE**2, 0.0,                    0.0,                    0.0,                    0.0,
                                                      0.0,                          0.0,                    VELOCITY_COVARIANCE**2, 0.0,                    0.0,                    0.0,                        
                                                      0.0,                          0.0,                    0.0,                    0.0,                    0.0,                    0.0,
                                                      0.0,                          0.0,                    0.0,                    0.0,                    0.0,                    0.0,
                                                      0.0,                          0.0,                    0.0,                    0.0,                    0.0,                    0.0,
                                                      ]
        
        
        self.gps_velocity_odom.publish(tempTwistedWithCovariance)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GPSFilter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()