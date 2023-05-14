import rclpy
from rclpy.node import Node


from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class FakeGpsPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #if(self.i%2 == 0):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.status.status=0
        msg.status.service=1
        msg.latitude=34.8413
        msg.longitude=-82.4117
        msg.altitude=278.299
        msg.position_covariance = [0.0169, 0.0, 0.0,
                                0.0, 0.0169, 0.0,
                                0.0, 0.0, 0.270]
        msg.position_covariance_type=1
        # else:
        #     msg = NavSatFix()
        #     msg.header.stamp = self.get_clock().now().to_msg()
        #     msg.header.frame_id = "gps_link"
        #     msg.status.status=0
        #     msg.status.service=1
        #     msg.latitude=34.831
        #     msg.longitude=-82.417
        #     msg.altitude=278.299
        #     msg.position_covariance = [0.0169, 0.0, 0.0,
        #                             0.0, 0.0169, 0.0,
        #                             0.0, 0.0, 0.2720]
        #     msg.position_covariance_type=1
        self.i+=1
        self.publisher_.publish(msg)
        
        

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = FakeGpsPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()