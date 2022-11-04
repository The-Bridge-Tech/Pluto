
from operator import ne
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from .HelpFunction import  calculate_velocity_from_pwm


from .ControllerNode import LEFT_NEUTRAL, RIGHT_NEUTRAL, RIGHT_MAX, LEFT_MAX, LEFT_MIN, RIGHT_MIN,WHEEL_RADIUS, WHEEL_SEPARATION, KNOW_VELOCITY, KNOW_PWM_RIGHT, KNOW_PWM_LEFT
#The joystick_interpret has two mode
# Mode 1: listen to 
    # axis 1 (for left wheel)
    # axis 4 (for right wheel)
    
    
    
# General setting
#TODO: change later

    
# settings for mode 1

# settings from teleop_twist
#TODO: change later
MAX_X_VELOCITY = 0.7
MAX_YAW_VELOCITY = 0.4
MAX_ALLOWED_X_VELOCITY = 5
MAX_ALLOWED_YAW_VELOCITY = 0.5

class JoystickInterpreter(Node):
    
    def __init__(self, mode:int, left_neutral:int, right_neutral:int, left_max, left_min, right_max, right_min, wheel_radius, wheel_separation, know_velocity, know_left_pwm, know_right_pwm) -> None:
        super().__init__("joystick_interpreter")
        self.mode  = mode
        self.left_neutral = left_neutral
        self.right_neutral = right_neutral
        self.left_max = left_max
        self.left_min = left_min
        self.right_max = right_max
        self.right_min = right_min
        self.wheel_radius = wheel_radius
        self.whee_separation = wheel_separation
        self.known_velocity = know_velocity
        self.know_left_pwm = know_left_pwm
        self.know_right_pwm = know_right_pwm
        
        
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
            self._interpret_mode_two,
            10)
   

        else:
            raise ValueError("Unknown modem in joystick_interpret")
    
    
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
        self.last_cmd_vel_joy_message = message
            
    
    def interpret_message(self):
        if self.mode == 1:
            self._interpret_mode_one(self.last_joy_message)
        elif self.mode == 2:
            self._interpret_mode_two(self.last_cmd_vel_joy_message, self.last_joy_message)
    def _interpret_mode_two(self, last_twist_message:Twist, last_joy_message:Joy) -> None:
        # as of right now, just publish what we heard
        
        # final_velocity_x = (last_twist_message.linear.x/ MAX_X_VELOCITY) * (last_joy_message.axes[4]/ MAX_ALLOWED_X_VELOCITY)
        # final_velocity_yaw= (last_twist_message.angular.z/ MAX_YAW_VELOCITY) *(last_joy_message.axes[4]/ MAX_ALLOWED_YAW_VELOCITY)  
        
        # self.twist_msg.linear.x = final_velocity_x
        # self.twist_msg.angular.z = final_velocity_yaw
        self.cmd_vel_publisher.publish(last_twist_message)
        
    def _interpret_mode_one(self, message: Joy)->None:
             
        # only listen to axis 1 and axis 4
        # as left and right motor
        print(message.axes)
        left_axis_value = message.axes[1]   
        right_axis_value = message.axes[4]
        print(left_axis_value, right_axis_value)
        left_pwm = self.calculate_pwm_from_axis(left_axis_value, self.left_neutral, self.left_min, self.left_max)
        right_pwm = self.calculate_pwm_from_axis(right_axis_value, self.right_neutral, self.right_min, self.right_max)
        
        print("left pwm " + str(left_pwm))
        print("right pwm" + str(right_pwm))
        
        # then conver to standard cmd_vel message
        left_velocity =  calculate_velocity_from_pwm(left_pwm, self.known_velocity, self.know_left_pwm)
        right_velocity =  calculate_velocity_from_pwm(right_pwm, self.known_velocity, self.know_right_pwm)
        
        print("Velocity left" + str(left_velocity))
        print("Velocity right" + str(right_velocity))
        
        velocity_x = ( self.wheel_radius*left_velocity) + (self.wheel_radius*right_velocity - self.wheel_radius*left_velocity)/2
        velocity_yaw = (right_velocity*self.wheel_radius - left_velocity*self.wheel_radius) / self.whee_separation
        

        
        self.twist_msg.linear.x = velocity_x
        self.twist_msg.angular.z = velocity_yaw
        self.cmd_vel_publisher.publish(self.twist_msg)
        
        



def main(args=None):

    rclpy.init(args=args)

    #TODO: modify later, right now is in mode 1
    joy_interpreter = JoystickInterpreter(mode=1, 
                                          left_neutral=LEFT_NEUTRAL, right_neutral=RIGHT_NEUTRAL, 
                                          left_max=LEFT_MAX, left_min=LEFT_MIN, 
                                          right_max=RIGHT_MAX, right_min=RIGHT_MIN, 
                                          wheel_radius=WHEEL_RADIUS, wheel_separation=WHEEL_SEPARATION, 
                                          know_velocity=KNOW_VELOCITY, know_left_pwm=KNOW_PWM_LEFT,
                                          know_right_pwm=KNOW_PWM_RIGHT)

    rclpy.spin(joy_interpreter)
  

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
