import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist, TwistStamped
from .HelpFunction import calculate_pwm_from_velocity2, calculate_velocity_from_pwm2
# # #initial state
# LEFT_NEUTRAL = 4555
# RIGHT_NEUTRAL = 6955
# RIGHT_MAX = 1000+RIGHT_NEUTRAL
# LEFT_MAX = 1000 + LEFT_NEUTRAL
# RIGHT_MIN = -1000 + RIGHT_NEUTRAL
# LEFT_MIN = -1000 + LEFT_NEUTRAL

# #TODO: need measurement
# WHEEL_RADIUS = 10 #In meters
# WHEEL_SEPARATION = 20# In meters

# # define a ration of pwm and velocity
# KNOW_VELOCITY = 1
# KNOW_PWM_LEFT = 4600
# KNOW_PWM_RIGHT = 7000



#Note: pwm value in servo are  50 hz per second

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.define_parameters()
        # call parameters 
        self.LEFT_NEUTRAL = self.get_parameter("LEFT_NEUTRAL").get_parameter_value().integer_value
        self.RIGHT_NEUTRAL = self.get_parameter("RIGHT_NEUTRAL").get_parameter_value().integer_value
        self.RIGHT_MAX = self.get_parameter("RIGHT_MAX").get_parameter_value().integer_value
        self.LEFT_MAX = self.get_parameter("LEFT_MAX").get_parameter_value().integer_value
        self.RIGHT_MIN = self.get_parameter("RIGHT_MIN").get_parameter_value().integer_value
        self.LEFT_MIN = self.get_parameter("LEFT_MIN").get_parameter_value().integer_value
        self.WHEEL_RADIUS = self.get_parameter("WHEEL_RADIUS").get_parameter_value().double_value
        self.WHEEL_SEPARATION = self.get_parameter("WHEEL_SEPARATION").get_parameter_value().double_value
        
        self.KNOW_LEFT_FULL_BACKWARD_SPEED = self.get_parameter('KNOW_LEFT_FULL_BACKWARD_SPEED').get_parameter_value().double_value
        self.KNOW_RIGHT_FULL_BACKWARD_SPEED = self.get_parameter('KNOW_RIGHT_FULL_BACKWARD_SPEED').get_parameter_value().double_value
        self.KNOW_LEFT_FULL_FORWARD_SPEED = self.get_parameter('KNOW_LEFT_FULL_FORWARD_SPEED').get_parameter_value().double_value
        self.KNOW_RIGHT_FULL_FORWARD_SPEED = self.get_parameter('KNOW_RIGHT_FULL_FORWARD_SPEED').get_parameter_value().double_value    
        self.PUBLISH_RATE = self.get_parameter('PUBLISH_RATE').get_parameter_value().integer_value
        
        self.right_server_publisher = self.create_publisher(UInt32, 'steering_right', 10)
        self.left_server_publisher = self.create_publisher(UInt32, 'steering_left', 10)        
        self.differential_raw_twist_publisher = self.create_publisher(TwistStamped, 'differential_raw_twist', 10)
        
        timer_period = 1/self.PUBLISH_RATE  # publish speed
        self.right_timer = self.create_timer(timer_period, self.right_servero_timer_callback)
        self.left_timer = self.create_timer(timer_period, self.left_servero_time_callback)
        self.differential_raw_twist_timer = self.create_timer(timer_period, self.differential_raw_twist_callback)
        self.i = 0
    
    
        self.new_left_pwm = UInt32()
        self.new_right_pwm = UInt32()
        self.differential_twist  = TwistStamped()
        self.differential_twist_frame_id = "differential_twist"
        


        # set the servo to neutral at startup
        self.new_left_pwm.data = self.LEFT_NEUTRAL
        self.new_right_pwm.data = self.RIGHT_NEUTRAL
        
        # subscribe to cmd_vel 
        self.cmd_vel_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listen_cmd_vel_callback, 10
        )
        self.cmd_vel_subscription
        
    def define_parameters(self)->None:
        self.declare_parameter('LEFT_NEUTRAL', 0)
        self.declare_parameter('RIGHT_NEUTRAL',0)
        self.declare_parameter('RIGHT_MAX', 0)
        self.declare_parameter('LEFT_MAX',0)
        self.declare_parameter('RIGHT_MIN',0)
        self.declare_parameter('LEFT_MIN',0)
        self.declare_parameter('WHEEL_RADIUS',0.4318)
        self.declare_parameter('WHEEL_SEPARATION',0.889)
        # self.declare_parameter('KNOW_VELOCITY',1)
        # self.declare_parameter('KNOW_PWM_LEFT',4600)
        # self.declare_parameter('KNOW_PWM_RIGHT', 7000)
        self.declare_parameter('KNOW_LEFT_FULL_BACKWARD_SPEED', -1.31065)
        self.declare_parameter('KNOW_RIGHT_FULL_BACKWARD_SPEED', -1.0847)
        self.declare_parameter('KNOW_LEFT_FULL_FORWARD_SPEED',1.514 )
        self.declare_parameter('KNOW_RIGHT_FULL_FORWARD_SPEED',1.5366)
        self.declare_parameter('PUBLISH_RATE',30)

    

    def listen_cmd_vel_callback(self, message:Twist):
        # do math to update the pwm
        
        # calculate left and right wheel's velocity base on 
            # 1. velocity in x
            # 2. velocity in z (yaw)
            # Note: we don't take velocity of y into consideration, because we simply can't move in the y direction
            
        # https://answers.ros.org/question/334022/how-to-split-cmd_vel-into-left-and-right-wheel-of-2wd-robot/
        # https://answers.ros.org/question/308340/exact-rotational-speed-of-a-wheel/
        
        velocity_x = message.linear.x
        velocity_yaw = message.angular.z
        
        # self.get_logger().info("velocity_x" + str(velocity_x))
        # self.get_logger().info("velocity_yaw" + str(velocity_yaw))
        
        #self.get_logger().info("Publish " + str(velocity_x) + " and " + str(velocity_yaw))
        
        # wheel_speed_left = ((velocity_x-(velocity_yaw*self.WHEEL_SEPARATION/2.0)) / self.WHEEL_RADIUS)
        # wheel_speed_right = ((velocity_x + (velocity_yaw*self.WHEEL_SEPARATION/2.0)) / self.WHEEL_RADIUS)
        
        # https://navigation.ros.org/setup_guides/odom/setup_odom.html
        wheel_speed_left = (2*velocity_x - velocity_yaw*self.WHEEL_SEPARATION)/2
        wheel_speed_right = 2*velocity_x -wheel_speed_left
        
        self.get_logger().info("wheel_speed_left" + str(wheel_speed_left))
        self.get_logger().info("wheel_speed_right" + str(wheel_speed_right))
        
        
        #self.get_logger().info("Left speed " + str(wheel_speed_left) + " and  Right speed" + str(wheel_speed_right))
        # Now, convert them to pwm values
        # since the servo is in linear. We assume that we are also in linear ratio.
        
        # temp_left_pwm =  int(calculate_pwm_from_velocity(wheel_speed_left, self.KNOW_VELOCITY, self.KNOW_PWM_LEFT)) 
        # temp_right_pwm =  int(calculate_pwm_from_velocity(wheel_speed_right, self.KNOW_VELOCITY, self.KNOW_PWM_RIGHT)) 
        
        
        # #self.get_logger().info("Left pwm " + str(temp_left_pwm) + " and  Right pwm" + str(temp_right_pwm))
        # self.new_left_pwm.data =  convert_negative_pwm_to_positive_pwm_value(self.LEFT_MAX, self.LEFT_NEUTRAL, self.LEFT_MIN, temp_left_pwm)
        
        # #int(self.calculate_pwm_from_velocity(wheel_speed_left, True))
        # self.new_right_pwm.data = convert_negative_pwm_to_positive_pwm_value(self.RIGHT_MAX, self.RIGHT_NEUTRAL, self.RIGHT_MIN, temp_right_pwm)
        # #int(self.calculate_pwm_from_velocity(wheel_speed_right, False))
        
        
        self.new_left_pwm.data =  int(calculate_pwm_from_velocity2(wheel_speed_left,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL))
        self.new_right_pwm.data = int(calculate_pwm_from_velocity2(wheel_speed_right,self.KNOW_RIGHT_FULL_FORWARD_SPEED, self.KNOW_RIGHT_FULL_BACKWARD_SPEED,self.RIGHT_MAX, self.RIGHT_MIN, self.RIGHT_NEUTRAL))
        
        #TODO: below might be unnecessary 
        if self.new_left_pwm.data > self.LEFT_MAX:
            self.new_left_pwm.data = self.LEFT_MAX
        elif self.new_left_pwm .data< self.LEFT_MIN:
            self.new_left_pwm.data = self.LEFT_MIN
        else:
            pass
        
        if self.new_right_pwm.data > self.RIGHT_MAX:
            self.new_right_pwm.data = self.RIGHT_MAX
        elif self.new_right_pwm.data < self.RIGHT_MIN:
            self.new_right_pwm.data = self.RIGHT_MIN
        else:
            pass
        
    def differential_raw_twist_callback(self):
        """
        Calculate differential odometry base on pwm value on both left and right servo. Publish result to /differential_raw_twist.
        """
        # 1. Calculate left and right wheel's velocity base on current pwm
        current_left_vel = calculate_velocity_from_pwm2(self.new_left_pwm.data,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL)
        current_right_vel = calculate_velocity_from_pwm2(self.new_right_pwm.data,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL)
        
        
        self.differential_twist.twist.linear.x = (current_right_vel + current_left_vel)/2
        self.differential_twist.twist.angular.z = (current_right_vel - current_left_vel)/self.WHEEL_SEPARATION
        
        
        # setting the header part
        self.differential_twist.header.stamp = self.get_clock().now().to_msg()
        self.differential_twist.header.frame_id = self.differential_twist_frame_id
        # 2. publish this odometry
        self.differential_raw_twist_publisher.publish(self.differential_twist)
        
    def right_servero_timer_callback(self):
        """
        Publish pwm value to '/steering_right'
        """
        self.right_server_publisher.publish(self.new_right_pwm)
        self.get_logger().info("Publish " + str(self.new_right_pwm.data) + " to right servero")
    
    def left_servero_time_callback(self):
        """
        Publish pwm value to '/steering_left'
        """
        self.left_server_publisher.publish(self.new_left_pwm)
        self.get_logger().info("Publish " + str(self.new_left_pwm.data) + " to left servero")


def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()