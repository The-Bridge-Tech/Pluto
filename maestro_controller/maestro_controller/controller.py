# command to allow dmesg: sudo sysctl kernel.dmesg_restrict=0
# command to see serial : dmesg | grep tty
# get permission sudo chmod 666 /dev/ttyACM0



from typing import Tuple
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt32
from .maestro import Controller


# Note:
# In a standaridize servo, the whole duty cycle is 20 milli seconds (ms) /  50 hz
# The neutral position is 1500 microSeconds (1.5 ms) - a duty cycle of 7.5%


# Set speed of channel
# This option specifies the speed of the servo in units of 0.25 μs / (10 ms). For example, with a speed of 4, the position will change by at most 1 μs per 10 ms, or 100.00 μs/s.


# Set acceleration of channel
#. This option specifies the acceleration of the servo in units of (0.25 μs) / (10 ms) / (80 ms). For example, with an acceleration of 4, the speed of the servo will change by a maximum of 1250 μs/s every second.


class MaestroController(Node):
    def __init__(self):
        super().__init__('maestro_controller')
        
        self.servo_controller = Controller()
      
        self.steer_right_listerner = self.create_subscription(
            UInt32, 'steering_right', self.listen_right, 1
        )
        self.steer_left_listerner = self.create_subscription(
            UInt32, 'steering_left',self.listen_left, 1
        )
        self.initial_setup()
        self.get_logger().info('I heard: "%s"' % "Connect to maestro controller")

    def initial_setup(self):
        
        #TODO: determine the acceleration and velocity limit
        self.servo_controller.setAccel(0,4) 
        self.servo_controller.setAccel(1,4) 
        self.servo_controller.setSpeed(0,10)
        self.servo_controller.setSpeed(1,10)

    def listen_left(self, left_pwm: UInt32):
        # listen for left motor's pwm

        self.servo_controller.setTarget(0, left_pwm.data)
    
    def listen_right(self, right_pwm: UInt32):
        # listen for right motor's pwm

        self.servo_controller.setTarget(1, right_pwm.data)
            
    def convert_twist_to_wheel_velocity(self, goal_speed:Twist) -> Tuple[float, float]:
        vel_l = ((goal_speed.linear.x - (goal_speed.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
        vel_r = ((goal_speed.linear.x + (goal_speed.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

        return vel_l, vel_r  # Return in rpm
    def listen_cmd_vel_callback(self, goal_speed:Twist):
        wheel_rpm = self.convert_twist_to_wheel_velocity(goal_speed=goal_speed)

def main(args=None):
    rclpy.init(args=args)
    
    servo_controller = MaestroController()
    rclpy.spin(servo_controller)
    
    
    servo_controller.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()