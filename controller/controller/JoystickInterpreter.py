
from operator import ne
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from .HelpFunction import  calculate_velocity_from_pwm


#from .ControllerNode import LEFT_NEUTRAL, RIGHT_NEUTRAL, RIGHT_MAX, LEFT_MAX, LEFT_MIN, RIGHT_MIN,WHEEL_RADIUS, WHEEL_SEPARATION, KNOW_VELOCITY, KNOW_PWM_RIGHT, KNOW_PWM_LEFT
#The joystick_interpret has two mode
# Mode 1: listen to 
    # axis 1 (for left wheel)
    # axis 4 (for right wheel)
# Mode 2:
    
    
    
# General setting
#TODO: change later

    
# settings for mode 1

# settings from teleop_twist
#TODO: change later


class JoystickInterpreter(Node):
    
    def __init__(self, mode:int) -> None:
        super().__init__("joystick_interpreter")
        self.mode  = mode

        self.define_parameters()
            # call parameters 

        self.last_joy_message = Joy()
        self.last_cmd_vel_joy_message = Twist()

        self.twist_msg = Twist()
        self.joy_subscription = self.create_subscription(
                Joy,'joy',self.joy_listerner,10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        cmd_vel_publisher_period =0.1 # seconds
        self.cmd_vel_timer = self.create_timer(cmd_vel_publisher_period, self.interpret_message)
        
        if mode == 1:
            # setup listerner for mode 1
            # this case, we listen to /joy topic
            # no more futher setup
            pass
        elif mode ==2:
            # setup listerner for mode 2
            self.cmd_joy_subscription = self.create_subscription(
                Twist,
            'cmd_vel_joy',
            self.cmd_vel_joy_listerner,
            10)


        else:
            raise ValueError("Unknown mode in joystick_interpret")
        
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
    
    def calculate_pwm_from_axis(self, axis: float, neutral, min, max):
        if axis == 0:
            return neutral
        elif axis > 0:
            return neutral + abs(axis) *(max-neutral)
            #return axis * (max-neutral)
        else:
            # when aixs <0
            return neutral  - abs(axis) * (neutral - min)
            #return abs(abs(axis) * (neutral-min)) - min
        
    def joy_listerner(self, message:Joy) ->None:
        
        self.last_joy_message = message
        
    def cmd_vel_joy_listerner(self, message:Twist) -> None:
        # x =  self.last_twist_message.linear.x
        # yaw = self.last_twist_message.angular.z
        #self.get_logger().info("Publish " + str(x) + " and " + str(yaw))
        self.last_cmd_vel_joy_message = message
            
        # x =  self.last_twist_message.linear.x
        # yaw = self.last_twist_message.angular.z
        # print(x, yaw)
        
    def interpret_message(self):
        if self.mode == 1:
            self._interpret_mode_one(self.last_joy_message)
        elif self.mode == 2:
            self._interpret_mode_two(self.last_cmd_vel_joy_message)
    def _interpret_mode_two(self, last_twist_message:Twist) -> None:
        # as of right now, just publish what we heard
        
        # final_velocity_x = (last_twist_message.linear.x/ MAX_X_VELOCITY) * (last_joy_message.axes[4]/ MAX_ALLOWED_X_VELOCITY)
        # final_velocity_yaw= (last_twist_message.angular.z/ MAX_YAW_VELOCITY) *(last_joy_message.axes[4]/ MAX_ALLOWED_YAW_VELOCITY)  
        
        # self.twist_msg.linear.x = final_velocity_x
        # self.twist_msg.angular.z = final_velocity_yaw
        # x =  last_twist_message.linear.x
        # yaw = last_twist_message.angular.z
        # self.get_logger().info("Publish " + str(x) + " and " + str(yaw))
        self.twist_msg = last_twist_message
        self.cmd_vel_publisher.publish(self.twist_msg)
        
    def _interpret_mode_one(self, message: Joy)->None:
             
        # only listen to axis 1 and axis 4
        # as left and right motor
        print(message.axes)
        left_axis_value = message.axes[1]   
        right_axis_value = message.axes[4]
        print(left_axis_value, right_axis_value)
        left_pwm = self.calculate_pwm_from_axis(left_axis_value, self.LEFT_NEUTRAL, self.LEFT_MIN, self.LEFT_MAX)
        right_pwm = self.calculate_pwm_from_axis(right_axis_value, self.RIGHT_NEUTRAL, self.RIGHT_MIN, self.RIGHT_MAX)
        
        print("left pwm " + str(left_pwm))
        print("right pwm" + str(right_pwm))
        
        # then conver to standard cmd_vel message
        left_velocity =  calculate_velocity_from_pwm(left_pwm, self.KNOW_VELOCITY, self.KNOW_PWM_LEFT, self.LEFT_MAX, self.LEFT_NEUTRAL, self.LEFT_MIN)
        right_velocity =  calculate_velocity_from_pwm(right_pwm, self.KNOW_VELOCITY, self.KNOW_PWM_RIGHT,self.RIGHT_MAX, self.RIGHT_NEUTRAL, self.RIGHT_MIN)
        
        print("Velocity left" + str(left_velocity))
        print("Velocity right" + str(right_velocity))
        
        velocity_x = ( self.WHEEL_RADIUS*left_velocity) + (self.WHEEL_RADIUS*right_velocity - self.WHEEL_RADIUS*left_velocity)/2
        velocity_yaw = (right_velocity*self.WHEEL_RADIUS - left_velocity*self.WHEEL_RADIUS) / self.WHEEL_SEPARATION
        

        
        self.twist_msg.linear.x = velocity_x
        self.twist_msg.angular.z = velocity_yaw
        self.cmd_vel_publisher.publish(self.twist_msg)
        
        



def main(args=None):

    rclpy.init(args=args)

    #TODO: modify later, right now is in mode 1
    joy_interpreter = JoystickInterpreter(mode=1)

    rclpy.spin(joy_interpreter)
  

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
