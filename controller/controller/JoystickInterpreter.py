"""
Interprets joystick message from topic '/joy' and publishes interpreted message to topic '/cmd_vel'
"""



from operator import ne
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from .HelpFunction import  calculate_velocity_from_pwm2
from std_msgs.msg import Bool

#from .ControllerNode import LEFT_NEUTRAL, RIGHT_NEUTRAL, RIGHT_MAX, LEFT_MAX, LEFT_MIN, RIGHT_MIN,WHEEL_RADIUS, WHEEL_SEPARATION, KNOW_VELOCITY, KNOW_PWM_RIGHT, KNOW_PWM_LEFT
#The joystick_interpret has two mode
# Mode 1: listen to 
    # axis 1 (for left wheel)
    # axis 4 (for right wheel)
# Mode 2:
    # axis 1 (for forward /backward)
    # axis 4 (for left/right)
    
    
# General setting
#TODO: change later

    
# settings for mode 1

# settings from teleop_twist
#TODO: change later


# CONSTANTS
BUTTONS = {
    'Y': 0,
    'B': 1,
    'A': 2,
    'X': 3,
    'L1': 4,
    'R1': 5,
    'L2': 6,
    'R2': 7,
    'SELECT': 8,
    'START': 9,
    'LEFT JOYSTICK PUSH IN': 10,
    'RIGHT JOYSTICK PUSH IN': 11,
    'HOME': 12,
}


class JoystickInterpreter(Node):
    
    def __init__(self, mode: int) -> None:
        super().__init__("joystick_interpreter")
        self.mode = mode # mode of joystick

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
        
        # SUBSCRIBERS
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        self.last_joy_message = Joy()
        
        # CMD_VEL
        # publish the command from joystick to cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            'cmd_vel', 
            10
        )
        cmd_vel_publisher_period =1/self.PUBLISH_RATE # seconds
        self.cmd_vel_timer = self.create_timer(
            cmd_vel_publisher_period, 
            self.interpret_message
        )
        
        # AUTONOMOUS MODE
        # publish change of state for 'A' button, which suppose to change back and forth between joystick and autonomous
        self.is_autonomous_mode_publisher = self.create_publisher(
            Bool, 
            'is_autonomous_mode', 
            1
        )
        self.is_autonomous_mode = False
        self.button_states = [0] * len(BUTTONS)

        # JOYSTICK MODE
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
                self.cmd_vel_joy_callback,
                10
            )
            self.last_cmd_vel_joy_message = Twist()
            self.twist_msg = Twist()
        else:
            raise ValueError("Unknown mode in joystick_interpreter")
        

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
        self.declare_parameter('KNOW_LEFT_FULL_BACKWARD_SPEED', -1.31065)
        self.declare_parameter('KNOW_RIGHT_FULL_BACKWARD_SPEED', -1.0847)
        self.declare_parameter('KNOW_LEFT_FULL_FORWARD_SPEED',1.514 )
        self.declare_parameter('KNOW_RIGHT_FULL_FORWARD_SPEED',1.5366)
        self.declare_parameter('PUBLISH_RATE',30)

    def calculate_pwm_from_axis(self, axis: float, neutral, min, max):
        if axis == 0:
            return neutral
        elif axis > 0:
            return neutral + abs(axis) *(max-neutral)
            #return axis * (max-neutral)
        else:
            # when axis <0
            return neutral  - abs(axis) * (neutral - min)
            #return abs(abs(axis) * (neutral-min)) - min
        
    def interpret_message(self):
        # only interpret if not in autonomous mode
        if not self.is_autonomous_mode:
            if self.mode == 1:
                self._interpret_mode_one(self.last_joy_message)
            elif self.mode == 2:
                self._interpret_mode_two(self.last_cmd_vel_joy_message)
        # else, means it is autonomous_mode now, simply ignore everything.
       
    def _interpret_mode_two(self, last_twist_msg:Twist) -> None:
        # as of right now, just publish what we heard
        
        # final_velocity_x = (last_twist_msg.linear.x/ MAX_X_VELOCITY) * (last_joy_message.axes[4]/ MAX_ALLOWED_X_VELOCITY)
        # final_velocity_yaw= (last_twist_msg.angular.z/ MAX_YAW_VELOCITY) *(last_joy_message.axes[4]/ MAX_ALLOWED_YAW_VELOCITY)  
        
        # self.twist_msg.linear.x = final_velocity_x
        # self.twist_msg.angular.z = final_velocity_yaw
        # x =  last_twist_msg.linear.x
        # yaw = last_twist_msg.angular.z
        #self.get_logger().info("Publish cmd vel")
        self.twist_msg = last_twist_msg
        self.cmd_vel_publisher.publish(self.twist_msg)
        
    def _interpret_mode_one(self, message: Joy)->None:
             
        # only listen to axis 1 and axis 4
        # as left and right motor
        #print(message.axes)
        left_axis_value = message.axes[1]   
        right_axis_value = message.axes[4]
        #print(left_axis_value, right_axis_value)
        left_pwm = self.calculate_pwm_from_axis(left_axis_value, self.LEFT_NEUTRAL, self.LEFT_MIN, self.LEFT_MAX)
        right_pwm = self.calculate_pwm_from_axis(right_axis_value, self.RIGHT_NEUTRAL, self.RIGHT_MIN, self.RIGHT_MAX)
        
        # self.get_logger().info('left pwm: ' + str(left_pwm))
        # self.get_logger().info('left forward: ' + str(self.KNOW_LEFT_FULL_FORWARD_SPEED))
        # self.get_logger().info('right forward: ' + str(self.KNOW_RIGHT_FULL_FORWARD_SPEED))
        # self.get_logger().info('left backward: ' + str(self.KNOW_LEFT_FULL_FORWARD_SPEED))
        # self.get_logger().info('right backward: ' + str(self.KNOW_RIGHT_FULL_BACKWARD_SPEED))
        
        # then conver to standard cmd_vel message
        left_velocity =  calculate_velocity_from_pwm2(left_pwm,self.KNOW_LEFT_FULL_FORWARD_SPEED, self.KNOW_LEFT_FULL_BACKWARD_SPEED, self.LEFT_MAX, self.LEFT_MIN, self.LEFT_NEUTRAL)
        right_velocity =  calculate_velocity_from_pwm2(right_pwm,self.KNOW_RIGHT_FULL_FORWARD_SPEED, self.KNOW_RIGHT_FULL_BACKWARD_SPEED, self.RIGHT_MAX, self.RIGHT_MIN, self.RIGHT_NEUTRAL)
        
        # self.get_logger().info('left velocity: ' + str(left_velocity))
        # self.get_logger().info('right velocity: ' + str(right_velocity))
        # print("Velocity left" + str(left_velocity))
        # print("Velocity right" + str(right_velocity))
        
        velocity_x = ( self.WHEEL_RADIUS*left_velocity) + (self.WHEEL_RADIUS*right_velocity - self.WHEEL_RADIUS*left_velocity)/2
        velocity_yaw = (right_velocity*self.WHEEL_RADIUS - left_velocity*self.WHEEL_RADIUS) / self.WHEEL_SEPARATION
        
        self.twist_msg.linear.x = velocity_x
        self.twist_msg.angular.z = velocity_yaw
        self.cmd_vel_publisher.publish(self.twist_msg)
        

    # BUTTON HELPERS

    def getButtonState(self, msg: Joy, button_name: str) -> int:
        return msg.buttons[BUTTONS[button_name]]
    
    def getLastButtonState(self, button_name: str) -> int:
        return self.button_states[BUTTONS[button_name]]
    
    def updateButtonStates(self, msg: Joy):
        self.button_states = msg.buttons

    def isButtonRelease(self, msg: Joy, button_name: str):
        return self.getLastButtonState(button_name) == 1 and self.getButtonState(msg, button_name) == 0


    # SUBSCRIBER CALLBACKS
        
    def joy_callback(self, msg: Joy) -> None:
        """Handle button presses. Only do action on button release."""
        # "A" button -> toggle autonomous mode
        if self.isButtonRelease(msg, 'A'):
            # toggle autonomous mode
            self.is_autonomous_mode = not self.is_autonomous_mode
            # publish the new state of the mode
            self.is_autonomous_mode_publisher.publish(Bool(data=self.is_autonomous_mode))
            self.get_logger().info(f'autonomous mode set to {self.is_autonomous_mode}')
        # Update button states
        self.updateButtonStates(msg)
        self.last_joy_message = msg

    def cmd_vel_joy_callback(self, message: Twist) -> None:
        # x =  self.last_twist_message.linear.x
        # yaw = self.last_twist_message.angular.z
        #self.get_logger().info("Publish " + str(x) + " and " + str(yaw))
        self.last_cmd_vel_joy_message = message
            
        # x =  self.last_twist_message.linear.x
        # yaw = self.last_twist_message.angular.z
        # print(x, yaw)

# MAIN

def main(args=None):
    rclpy.init(args=args)

    joy_interpreter = JoystickInterpreter(mode=2)

    rclpy.spin(joy_interpreter)

    joy_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
