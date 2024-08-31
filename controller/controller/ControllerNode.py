"""
Gets interpreted joystick message from topic '/cmd_vel' (from JoystickInterpreter node) 
and publishes the corresponding servo values (topics '/steering_left' and '/steering_right')
"""


import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32, Bool
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

        # PARAMETERS
        self.define_parameters()
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
        
        # PUBLISHERS
        self.right_server_publisher = self.create_publisher(
            UInt32, 
            'steering_right', 
            10
        )
        self.left_server_publisher = self.create_publisher(
            UInt32, 
            'steering_left', 
            10
        )        
        #self.differential_raw_twist_publisher = self.create_publisher(TwistStamped, 'differential_raw_twist', 10)
        
        timer_period = 1/self.PUBLISH_RATE  # publish speed
        # self.right_timer = self.create_timer(timer_period, self.right_servero_timer_callback)
        # self.left_timer = self.create_timer(timer_period, self.left_servero_time_callback)
        #self.differential_raw_twist_timer = self.create_timer(timer_period, self.differential_raw_twist_callback)
        self.i = 0
    
        self.new_left_pwm = UInt32()
        self.new_right_pwm = UInt32()
        self.differential_twist  = TwistStamped()
        self.differential_twist_frame_id = "differential_twist"
        
        # set the servo to neutral at startup
        self.new_left_pwm.data = self.LEFT_NEUTRAL
        self.new_right_pwm.data = self.RIGHT_NEUTRAL

        # SUBSCRIBERS
        self.cmd_vel_sub = self.create_subscription(
            Twist, 
            'cmd_vel', 
            self.cmd_vel_callback, 
            10
        )
        self.is_autonomous_mode_sub = self.create_subscription(
            Bool, 
            "is_autonomous_mode", 
            self.is_autonomous_mode_callback, 
            10
        )
        self.is_autonomous_mode = False
    

    # HELPERS
    
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


    # SUBSCRIBER CALLBACKS

    def is_autonomous_mode_callback(self, msg: Bool):
        self.is_autonomous_mode = msg.data
        
    # interpret from cmd_vel to both left&right servo
    def cmd_vel_callback(self, msg: Twist):
        if not self.is_autonomous_mode:
            # do math to update the pwm
            
            # calculate left and right wheel's velocity base on 
                # 1. velocity in x
                # 2. velocity in z (yaw)
                # Note: we don't take velocity of y into consideration, because we simply can't move in the y direction
                
            #self.get_logger().info("I heard cmd_vel in controller")
            # https://answers.ros.org/question/334022/how-to-split-cmd_vel-into-left-and-right-wheel-of-2wd-robot/
            # https://answers.ros.org/question/308340/exact-rotational-speed-of-a-wheel/
            
            velocity_x = msg.linear.x
            velocity_yaw = msg.angular.z
            

            
            # https://navigation.ros.org/setup_guides/odom/setup_odom.html
            wheel_speed_left = (2*velocity_x - velocity_yaw*self.WHEEL_SEPARATION)/2
            wheel_speed_right = 2*velocity_x -wheel_speed_left
            

            
            new_calculated_left_pwm =  int(calculate_pwm_from_velocity2(wheel_speed_left,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL))
            new_calculated_right_pwm = int(calculate_pwm_from_velocity2(wheel_speed_right,self.KNOW_RIGHT_FULL_FORWARD_SPEED, self.KNOW_RIGHT_FULL_BACKWARD_SPEED,self.RIGHT_MAX, self.RIGHT_MIN, self.RIGHT_NEUTRAL))

            # Ensure pwm value fall within the limit
            if new_calculated_left_pwm > self.LEFT_MAX:
                new_calculated_left_pwm= self.LEFT_MAX
            elif new_calculated_left_pwm< self.LEFT_MIN:
                new_calculated_left_pwm = self.LEFT_MIN
            else:
                pass
            
            if new_calculated_right_pwm  > self.RIGHT_MAX:
                new_calculated_right_pwm  = self.RIGHT_MAX
            elif new_calculated_right_pwm  < self.RIGHT_MIN:
                new_calculated_right_pwm  = self.RIGHT_MIN
            else:
                pass
            
            # self.new_left_pwm.data = new_calculated_left_pwm
            # self.new_right_pwm.data = new_calculated_right_pwm
            if new_calculated_left_pwm != self.new_left_pwm.data or new_calculated_right_pwm != self.new_right_pwm.data:
                self.new_left_pwm.data = new_calculated_left_pwm
                self.new_right_pwm.data = new_calculated_right_pwm
                
                # Now, publish them out
                self.left_server_publisher.publish(self.new_left_pwm)
                self.right_server_publisher.publish(self.new_right_pwm)
            else:
                pass  # there is no point of updating the message then 
            
            # now, publish out the message to public
        else:
            pass  # do nothing at autonomous mode
            
    # def differential_raw_twist_callback(self):
    #     """
    #     Calculate differential twist base on pwm value on both left and right servo. Publish result to /differential_raw_twist.
        
    #     Note
    #     -----
    #     One might ask why did the function calcuate speed base on pwm value send to maestro, not base on value read from maestro.
    #     But if you read https://github.com/The-Bridge-Tech/Pluto/blob/devel/maestro_controller/maestro_controller/maestro.py#L123
    #     It shows that the maestro just return the recent received value. Thus, it does not make a difference in this case.
    #     """
    #     # 1. Calculate left and right wheel's velocity base on current pwm
    #     current_left_vel = calculate_velocity_from_pwm2(self.new_left_pwm.data,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL)
    #     current_right_vel = calculate_velocity_from_pwm2(self.new_right_pwm.data,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL)
        
        
    #     self.differential_twist.twist.linear.x = (current_right_vel + current_left_vel)/2
    #     self.differential_twist.twist.angular.z = (current_right_vel - current_left_vel)/self.WHEEL_SEPARATION
        
        
    #     # setting the header part
    #     self.differential_twist.header.stamp = self.get_clock().now().to_msg()
    #     self.differential_twist.header.frame_id = self.differential_twist_frame_id
    #     # 2. publish this odometry
    #     self.differential_raw_twist_publisher.publish(self.differential_twist)
        
    def right_servero_timer_callback(self):
        """Publish right servo value to '/steering_right'"""
        self.right_server_publisher.publish(self.new_right_pwm)
        #self.get_logger().info("Publish " + str(self.new_right_pwm.data) + " to right servero")
    
    def left_servero_time_callback(self):
        """Publish left servo value to '/steering_left'"""
        self.left_server_publisher.publish(self.new_left_pwm)
        #self.get_logger().info("Publish " + str(self.new_left_pwm.data) + " to left servero")


# MAIN

def main(args=None):
    rclpy.init(args=args)

    controller_node = ControllerNode()

    rclpy.spin(controller_node)

    controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()