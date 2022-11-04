import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt32
from geometry_msgs.msg import Twist

# #initial state
LEFT_NEUTRAL = 4555
RIGHT_NEUTRAL = 6955
RIGHT_MAX = 1000+RIGHT_NEUTRAL
LEFT_MAX = 1000 + LEFT_NEUTRAL
RIGHT_MIN = -1000 + RIGHT_NEUTRAL
LEFT_MIN = -1000 + LEFT_NEUTRAL

#TODO: need measurement
WHEEL_RADIUS = 10 #In meters
WHEEL_SEPARATION = 20# In meters

# define a ration of pwm and velocity
KNOW_VELOCITY = 1
KNOW_PWM_LEFT = 4600
KNOW_PWM_RIGHT = 7000



#Note: pwm value in servo are  50 hz per second

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.define_parameters()
        
        self.right_server_publisher = self.create_publisher(UInt32, 'steering_right', 10)
        self.left_server_publisher = self.create_publisher(UInt32, 'steering_left', 10)        
       
        timer_period = 0.1  # publish speed evey 0.1 seconds
        self.right_timer = self.create_timer(timer_period, self.right_servero_timer_callback)
        self.left_timer = self.create_timer(timer_period, self.left_servero_time_callback)
        self.i = 0
    
    
        self.new_left_pwm = UInt32()
        self.new_right_pwm = UInt32()
        
        
        # call parameters 
        self.LEFT_NEUTRAL = self.get_parameter("LEFT_NEUTRAL").get_parameter_value().integer_value
        self.RIGHT_NEUTRAL = self.get_parameter("RIGHT_NEUTRAL").get_parameter_value().integer_value
        self.RIGHT_MAX = self.get_parameter("RIGHT_MAX").get_parameter_value().integer_value
        self.LEFT_MAX = self.get_parameter("LEFT_MAX").get_parameter_value().integer_value
        self.RIGHT_MIN = self.get_parameter("RIGHT_MIN").get_parameter_value().integer_value
        self.LEFT_MIN = self.get_parameter("LEFT_MIN").get_parameter_value().integer_value
        self.WHEEL_RADIUS = self.get_parameter("WHEEL_RADIUS").get_parameter_value().integer_value
        self.WHEEL_SEPARATION = self.get_parameter("WHEEL_SEPARATION").get_parameter_value().integer_value
        self.KNOW_VELOCITY = self.get_parameter("KNOW_VELOCITY").get_parameter_value().integer_value
        self.KNOW_PWM_LEFT = self.get_parameter("KNOW_PWM_LEFT").get_parameter_value().integer_value
        self.KNOW_PWM_RIGHT = self.get_parameter("KNOW_PWM_RIGHT").get_parameter_value().integer_value
        
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
        self.declare_parameter('WHEEL_RADIUS',10)
        self.declare_parameter('WHEEL_SEPARATION',20)
        self.declare_parameter('KNOW_VELOCITY',1)
        self.declare_parameter('KNOW_PWM_LEFT',4600)
        self.declare_parameter('KNOW_PWM_RIGHT', 7000)

    
    def calculate_pwm_from_velocity(self, goal_velocity:int, is_left:bool)-> float:
        # we assume that the servo is linear
        """
        calculate servo's pwm base on given velocity

        Equations:
        
        known velocity        goal velocity
        ---------------   ==  ----------------
        know pwm value        goal pwm value
        
        Parameters
        ----------
        goal_velocity : int
        
        Returns
        -------
        int
        """ 
        if is_left:
            return (goal_velocity * self.KNOW_PWM_LEFT) / self.KNOW_VELOCITY
        else:
            return (goal_velocity * self.KNOW_PWM_RIGHT) / self.KNOW_VELOCITY
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
        
        wheel_speed_left = ((velocity_x-(velocity_yaw*self.WHEEL_SEPARATION/2.0)) / self.WHEEL_RADIUS)
        wheel_speed_right = ((velocity_x + (velocity_yaw*self.WHEEL_SEPARATION/2.0)) / self.WHEEL_RADIUS)
        
        # Now, convert them to pwm values
        # since the servo is in linear. We assume that we are also in linear ratio.
        self.new_left_pwm.data = int(self.calculate_pwm_from_velocity(wheel_speed_left, True))
        self.new_right_pwm.data = int(self.calculate_pwm_from_velocity(wheel_speed_right, False))
        
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