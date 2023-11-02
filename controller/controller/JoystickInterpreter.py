
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


class JoystickInterpreter(Node):
    
    def __init__(self, mode:int) -> None:
        super().__init__("joystick_interpreter")
        self.mode  = mode # mode of joystick

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
        
        

        self.last_joy_message = Joy()
        self.last_cmd_vel_joy_message : Twist = Twist()

        self.twist_msg = Twist()
        self.joy_subscription = self.create_subscription(
                Joy,'joy',self.joy_listerner,10)
        
        
        
        
        # publish the command from joystick to cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        cmd_vel_publisher_period =1/self.PUBLISH_RATE # seconds
        self.cmd_vel_timer = self.create_timer(cmd_vel_publisher_period, self.interpret_message)
        
        # publish change of state for 'A' button, which suppose to change back and forth between joystick and autonomous
        self.autonomous_button_change_publisher = self.create_publisher(Bool, 'autonomous_button', 1)
        self.autonomous_button_counter=0 # 0 means did not turn on, 

        self.is_autonomous_mode_indicator = Bool()
    
        self.is_autonomous_mode_subscription = self.create_subscription(
            Bool, 'is_autonomous_mode', self.autonomous_mode_listerner,1)
        
        
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
            # when aixs <0
            return neutral  - abs(axis) * (neutral - min)
            #return abs(abs(axis) * (neutral-min)) - min
        
    def autonomous_mode_listerner(self, mes:Bool)->None:
        self.is_autonomous_mode_indicator = mes
        
        
    def joy_listerner(self, message:Joy) ->None:
        if(message.buttons[0] == 1):
            # self.get_logger().info("here")
            if self.autonomous_button_counter == 0:
                self.autonomous_button_counter = 1         
                # k  = Bool()
                # k.data=True
                # self.autonomous_button_change_publisher.publish(k)
                # self.autonomous_button_counter = 0
            # else:
            #     self.autonomous_button_counter += 1
        else:
            if self.autonomous_button_counter == 1:
                self.autonomous_button_counter = 0
                k  = Bool()
                k.data=True
                self.autonomous_button_change_publisher.publish(k)
        #self.get_logger().info("autonomousButtonIndicator" + str(self.autonomous_button_counter))
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
        
        # only interpret if is_autonomous_indicator == FAlse
        if self.is_autonomous_mode_indicator.data == False:
            if self.mode == 1:
                self._interpret_mode_one(self.last_joy_message)
            elif self.mode == 2:
                self._interpret_mode_two(self.last_cmd_vel_joy_message)
        # else, means it is autonomous_mode now, simply ignore everything.
       
            
    def _interpret_mode_two(self, last_twist_message:Twist) -> None:
        # as of right now, just publish what we heard
        
        # final_velocity_x = (last_twist_message.linear.x/ MAX_X_VELOCITY) * (last_joy_message.axes[4]/ MAX_ALLOWED_X_VELOCITY)
        # final_velocity_yaw= (last_twist_message.angular.z/ MAX_YAW_VELOCITY) *(last_joy_message.axes[4]/ MAX_ALLOWED_YAW_VELOCITY)  
        
        # self.twist_msg.linear.x = final_velocity_x
        # self.twist_msg.angular.z = final_velocity_yaw
        # x =  last_twist_message.linear.x
        # yaw = last_twist_message.angular.z
        #self.get_logger().info("Publish cmd vel")
        self.twist_msg = last_twist_message
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
        
        



def main(args=None):

    rclpy.init(args=args)

    #TODO: modify later, right now is in mode 1
    joy_interpreter = JoystickInterpreter(mode=2)

    rclpy.spin(joy_interpreter)
  

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_interpreter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()
