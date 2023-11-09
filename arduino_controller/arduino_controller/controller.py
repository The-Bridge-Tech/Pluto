# command to allow dmesg: sudo sysctl kernel.dmesg_restrict=0
# command to see serial : dmesg | grep tty
# get permission sudo chmod 666 /dev/ttyACM0



from typing import Tuple
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist 
from std_msgs.msg import UInt32
import serial



class ArduinoController(Node):
    def __init__(self):
        super().__init__('maestro_controller')
        
        self.serial_controller =serial.Serial('/dev/ttyACM0', 9600)  # Change 'COM1' to your specific serial port and set the baud rate accordingly

    
        self.steer_right_listerner = self.create_subscription(
            UInt32, 'steering_right', self.listen_right, 1
        )
        self.steer_left_listerner = self.create_subscription(
            UInt32, 'steering_left',self.listen_left, 1
        )
      
        self.get_logger().info('I heard: "%s"' % "Connect to maestro controller")

    # def initial_setup(self):
        
    #     #TODO: determine the acceleration and velocity limit
    #     self.servo_controller.setAccel(0,4) 
    #     self.servo_controller.setAccel(1,4) 
    #     self.servo_controller.setSpeed(0,10)
    #     self.servo_controller.setSpeed(1,10)

    def listen_left(self, left_pwm: UInt32):
        # listen for left motor's pwm
        message = "L{0}\n".format(left_pwm.data)  # Add a newline character to signal the end of the message
        
        # Send the message to the Arduino
        self.serial_controller.write(message.encode('utf-8'))


    def listen_right(self, right_pwm: UInt32):
        # listen for left motor's pwm
        message = "R{0}\n".format(right_pwm.data)  # Add a newline character to signal the end of the message

        # Send the message to the Arduino
        self.serial_controller.write(message.encode('utf-8'))

    # def convert_twist_to_wheel_velocity(self, goal_speed:Twist) -> Tuple[float, float]:
    #     vel_l = ((goal_speed.linear.x - (goal_speed.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)
    #     vel_r = ((goal_speed.linear.x + (goal_speed.angular.z * self.wheel_bias / 2.0)) / self.wheel_radius) * 60/(2*3.14159)

    #     return vel_l, vel_r  # Return in rpm
    # def listen_cmd_vel_callback(self, goal_speed:Twist):
    #     wheel_rpm = self.convert_twist_to_wheel_velocity(goal_speed=goal_speed)

def main(args=None):
    rclpy.init(args=args)
    controllerNode = ArduinoController()

    rclpy.spin(controllerNode)
    
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()