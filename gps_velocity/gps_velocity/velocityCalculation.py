import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import NavSatFix
from typing import List, Tuple
from geographiclib.geodesic import Geodesic

from math import sin, atan2, sqrt, cos
import math
import geopy.distance
from rclpy.qos import ReliabilityPolicy


class GPSVelocity(Node):

    def __init__(self):
        super().__init__('gps_velocity')
        self.publisher_ = self.create_publisher(
            TwistWithCovarianceStamped, '/velocity/gps', 10)

        # TODO: change to best effort strategy?
        self.gps_fix_sub = self.create_subscription(
            NavSatFix, "/fix", self.update_gps_velocity, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_gps_velocity)

        # the latest velocity calculation
        self.latest_twist: TwistWithCovarianceStamped = TwistWithCovarianceStamped()
        self.initialized: bool = False

        self.last_gps_coordinate: NavSatFix = None

    # # modify base on https://github.com/danielsnider/gps_goal
    # def calculate_gps_offset(self, origin_lat: float, origin_long: float, goal_lat: float, goal_long: float, unitOfMeasurement: str = "ENU") -> Tuple[float, float]:
    #     # Calculate distance and azimuth between GPS points
    #     geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    #     # Compute several geodesic calculations between two GPS points
    #     g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
    #     hypotenuse = distance = g['s12']  # access distance

    #     degree_to_rad = float(math.pi / 180.0)

    #     d_lat = (goal_lat - origin_lat) * degree_to_rad
    #     d_long = (goal_long - origin_long) * degree_to_rad

    #     a = pow(sin(d_lat / 2), 2) + cos(origin_lat * degree_to_rad) * \
    #         cos(goal_lat * degree_to_rad) * pow(sin(d_long / 2), 2)
    #     c = 2 * atan2(sqrt(a), sqrt(1 - a))
    #     km = 6367 * c
    #     mi = 3956 * c

    #     # self.get_logger().info("The distance from the origin to the goal is {:.3f} m.".format(distance))
    #     if unitOfMeasurement == "ENU":
    #         # https://answers.ros.org/question/219182/how-to-determine-yaw-angle-from-gps-bearing/
    #         azimuth = 90 - g['azi1']
    #     else:
    #         azimuth = g['azi1']
    #     # self.get_logger().info("The azimuth from the origin to the goal is {:.3f} degrees.".format(azimuth))

    #     # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
    #     # Convert azimuth to radians
    #     # print(azimuth)
    #     azimuth = math.radians(azimuth)
    #     x = adjacent = math.cos(azimuth) * hypotenuse
    #     y = opposite = math.sin(azimuth) * hypotenuse
    #     # self.get_logger().info("The translation from the origin to the goal is (x,y) {:.3f}, {:.3f} m.".format(x, y))

    #     return x, y

    def update_gps_velocity(self, message: NavSatFix):
        """
        Calculate velocity base on 2 gps points.

        Parameters
        ----------
        message : NavSatFix

        Note:
        ------
        The robot only can move in forward/backward (on the x-axis).
        Thus, the velocity of y will always set to be 0 (see publish_gps_velocity()).
        """
        if self.last_gps_coordinate == None:
            self.last_gps_coordinate = message
        else:
            # do velocity calculation
            # x, y = self.calculate_gps_offset(self.last_gps_coordinate.latitude, self.last_gps_coordinate.longitude,
                        
            #                                  message.latitude, message.longitude)
            # distance = sqrt(x*x + y*y)
            distance  = geopy.distance.geodesic( (self.last_gps_coordinate.latitude, self.last_gps_coordinate.longitude),
                                        (message.latitude, message.longitude)).km * 1000
            
            
            time_difference_in_second: float = (message.header.stamp.sec + message.header.stamp.nanosec/(
                10**9)) - (self.last_gps_coordinate.header.stamp.sec + self.last_gps_coordinate.header.stamp.nanosec/(10**9))

            self.latest_twist.twist.twist.linear.x = distance/time_difference_in_second
            
            self.last_gps_coordinate = message
  

    def publish_gps_velocity(self):
        if (self.initialized):
            self.latest_twist.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.latest_twist)
            
            
        else:
            # initialize all other data in self.latest_twist 
            
            self.latest_twist.twist.twist.linear.y = 0.0
            self.latest_twist.twist.twist.linear.z = 0.0
            self.latest_twist.twist.twist.angular.x = 0.0
            self.latest_twist.twist.twist.angular.y = 0.0
            self.latest_twist.twist.twist.angular.z = 0.0
            
            
            # the uncertainty in twist measurement
            
            #since the gps could be off by +/- 1 meters.
            # we make the inaccuracy of velocity x to be 0.5 meters/s
            
            
            self.latest_twist.twist.covariance = [
                0.5**2,   0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
            ]
            # although there is a tf transform between gps_link and base_link
            # however, the calculate_gps_offset has already convert yaw from NED to ENU
            self.latest_twist.header.frame_id="base_link"  
            self.initialized = True
            


def main(args=None):
    rclpy.init(args=args)

    gps_velocity = GPSVelocity()

    rclpy.spin(gps_velocity)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gps_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
