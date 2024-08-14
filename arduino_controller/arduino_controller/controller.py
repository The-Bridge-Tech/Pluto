"""
Sends left and right motor pwm values to arduino.
Electric mower equivalent of maestro_controller for gas mower
"""


# command to allow dmesg: sudo sysctl kernel.dmesg_restrict=0
# command to see serial : dmesg | grep tty
# get permission sudo chmod 666 /dev/ttyACM0


# ROS MODULES
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt32
# ARDUINO MODULES
import serial


class ArduinoController(Node):

    def __init__(self):
        super().__init__('maestro_controller')
        
        self.serial_controller = serial.Serial(
            '/dev/ttyACM0', # serial port
            9600            # baud rate
        )
        
        # Subscribers
        self.steer_right_subscriber = self.create_subscription(
            UInt32, 
            'steering_right', 
            self.steer_right_callback, 
            1
        )
        self.steer_left_subscriber = self.create_subscription(
            UInt32, 
            'steering_left',
            self.steer_left_callback, 
            1
        )

        # Variables
        self.current_left_pwm = 120
        self.current_right_pwm = 120

        self.get_logger().info('I heard: "Connect to maestro controller"')

    
    # HELPERS

    def send_pwm_message(self):
        """Send current left and right pwm values to arduino via serial_controller."""
        message = f"{self.current_left_pwm} {self.current_right_pwm}\n"
        # Log the message
        self.get_logger().info('message: ' + message)
        # Send message to arduino
        self.serial_controller.write(message.encode('utf-8'))
        self.serial_controller.flushOutput()

    # def initial_setup(self):
    #     #TODO: determine the acceleration and velocity limit
    #     self.servo_controller.setAccel(0,4) 
    #     self.servo_controller.setAccel(1,4) 
    #     self.servo_controller.setSpeed(0,10)
    #     self.servo_controller.setSpeed(1,10)


    # SUBSCRIBER CALLBACKS

    def steer_left_callback(self, left_pwm: UInt32):
        """Update and send current left pwm value to arduino."""
        # Update the current left pwm value
        self.current_left_pwm = left_pwm.data
        # Send current pwm values to arduino
        self.send_pwm_message()

    def steer_right_callback(self, right_pwm: UInt32):
        """Update and send current right pwm value to arduino."""
        # Update the current right pwm value
        self.current_right_pwm = right_pwm.data
        # Send current pwm values to arduino
        self.send_pwm_message()

    # def convert_twist_to_wheel_velocity(self, goal_speed:Twist) -> Tuple[float, float]:
    #     vel_l = ((goal_speed.linear.x - (goal_speed.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
    #     vel_r = ((goal_speed.linear.x + (goal_speed.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

    #     return vel_l, vel_r  # Return in rpm
    # def listen_cmd_vel_callback(self, goal_speed:Twist):
    #     wheel_rpm = self.convert_twist_to_wheel_velocity(goal_speed=goal_speed)


# MAIN

def main(args=None):
    rclpy.init(args=args)
    controllerNode = ArduinoController()

    rclpy.spin(controllerNode)
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()