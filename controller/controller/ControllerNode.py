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
        # YAML File:
        #   Gas         pluto_launch/config/servos.yaml
        #   Electric    pluto_launch/config/servos_electric.yaml
        # PWM
        self.min_pwm = self.load_param_int("min_pwm")
        self.neutral_pwm = self.load_param_int("neutral_pwm")
        self.max_pwm = self.load_param_int("max_pwm")
        # WHEEL
        self.wheel_radius = self.load_param_double("wheel_radius")
        self.wheel_separation = self.load_param_double("wheel_separation")
        # MAX BACKWARD SPEED
        self.max_left_backward_speed = self.load_param_double('max_left_backward_speed')
        self.max_right_backward_speed = self.load_param_double('max_right_backward_speed')
        # MAX FORWARD SPEED
        self.max_left_forward_speed = self.load_param_double('max_left_forward_speed')
        self.max_right_forward_speed = self.load_param_double('max_right_forward_speed')    
        # OTHER
        self.publish_frequency = self.load_param_int('publish_frequency')   

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
        
        timer_period = 1 / self.publish_frequency  # publish speed
        # self.right_timer = self.create_timer(timer_period, self.right_servero_timer_callback)
        # self.left_timer = self.create_timer(timer_period, self.left_servero_time_callback)
        #self.differential_raw_twist_timer = self.create_timer(timer_period, self.differential_raw_twist_callback)
        self.i = 0
    
        self.new_left_pwm = UInt32()
        self.new_right_pwm = UInt32()
        self.differential_twist  = TwistStamped()
        self.differential_twist_frame_id = "differential_twist"
        
        # set the servo to neutral at startup
        self.new_left_pwm.data = self.neutral_pwm
        self.new_right_pwm.data = self.neutral_pwm

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
            1
        )
        self.is_autonomous_mode = False
    

    # HELPERS - PARAMETERS
    
    def load_param(self, param_name: str, init_value):
        self.declare_parameter(param_name, init_value)
        return self.get_parameter(param_name).get_parameter_value()
    
    def load_param_int(self, param_name: str) -> int:
        return self.load_param(param_name, 0).integer_value
    
    def load_param_double(self, param_name: str) -> float:
        return self.load_param(param_name, 0.0).double_value
    
    def load_param_bool(self, param_name: str) -> bool:
        return self.load_param(param_name, False).bool_value


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
            wheel_speed_left = (2*velocity_x - velocity_yaw*self.wheel_separation)/2
            wheel_speed_right = 2*velocity_x -wheel_speed_left
            

            
            new_calculated_left_pwm =  int(calculate_pwm_from_velocity2(wheel_speed_left,self.max_left_forward_speed, self.max_left_backward_speed, self.max_pwm, self.min_pwm, self.neutral_pwm))
            new_calculated_right_pwm = int(calculate_pwm_from_velocity2(wheel_speed_right,self.max_right_forward_speed, self.max_right_backward_speed,self.max_pwm, self.min_pwm, self.neutral_pwm))

            # Ensure pwm value fall within the limit
            if new_calculated_left_pwm > self.max_pwm:
                new_calculated_left_pwm= self.max_pwm
            elif new_calculated_left_pwm< self.min_pwm:
                new_calculated_left_pwm = self.min_pwm
            else:
                pass
            
            if new_calculated_right_pwm  > self.max_pwm:
                new_calculated_right_pwm  = self.max_pwm
            elif new_calculated_right_pwm  < self.min_pwm:
                new_calculated_right_pwm  = self.min_pwm
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
    #     current_left_vel = calculate_velocity_from_pwm2(self.new_left_pwm.data,self.max_left_forward_speed, self.max_left_backward_speed, self.max_pwm, self.min_pwm, self.neutral_pwm)
    #     current_right_vel = calculate_velocity_from_pwm2(self.new_right_pwm.data,self.max_left_forward_speed, self.max_left_backward_speed, self.max_pwm, self.min_pwm, self.neutral_pwm)
        
        
    #     self.differential_twist.twist.linear.x = (current_right_vel + current_left_vel)/2
    #     self.differential_twist.twist.angular.z = (current_right_vel - current_left_vel)/self.wheel_separation
        
        
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