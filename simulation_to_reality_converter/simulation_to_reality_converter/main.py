import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry

class SimulationToRealityConverter(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.global_odom_pub = self.create_publisher(Odometry, '/odometry/global', 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom",self.odom_callback ,10)
    def odom_callback(self, odom:Odometry):
        self.global_odom_pub.publish(odom)
                


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimulationToRealityConverter()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()