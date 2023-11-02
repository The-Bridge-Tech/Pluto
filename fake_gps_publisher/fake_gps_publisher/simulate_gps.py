import rclpy
from rclpy.node import Node


from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
gps_list = [(34.84136489243739, -82.41178985244608),(34.84131419440653, -82.41170652710007)]
class FakeGpsPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, '/fix', 10)
        timer_period = 1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        #if(self.i%2 == 0):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.status.status=0
        msg.status.service=1
        
        gps_coord = gps_list[self.i]
        msg.latitude=gps_coord[0]
        msg.longitude=gps_coord[1]
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
        self.i %= len(gps_list)
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