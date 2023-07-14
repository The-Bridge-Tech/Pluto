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
import geopy.distance

# modify base on https://github.com/danielsnider/gps_goal
def calc_goal(origin_lat, origin_long, goal_lat, goal_long):
    # Calculate distance and azimuth between GPS points
    geod = Geodesic.WGS84  # define the WGS84 ellipsoid
    # Compute several geodesic calculations between two GPS points
    g = geod.Inverse(origin_lat, origin_long, goal_lat, goal_long)
    hypotenuse = distance = g['s12']  # access distance

    azimuth = g['azi1']

    # Convert polar (distance and azimuth) to x,y translation in meters (needed for ROS) by finding side lenghs of a right-angle triangle
    # Convert azimuth to radians

    azimuth = math.radians(azimuth)

    x = adjacent = math.cos(azimuth) * hypotenuse
    y = opposite = math.sin(azimuth) * hypotenuse

    return x, y


class GPSVelocity(Node):

    def __init__(self):
        super().__init__('gps_velocity')
        self.enu_velocity_published = self.create_publisher(
            TwistWithCovarianceStamped, '/velocity/gps_enu', 10)
        self.ned_velocity_published = self.create_publisher(
            TwistWithCovarianceStamped, '/velocity/gps_ned', 10)
        self.nonholonomic_velocity_published = self.create_publisher(
            TwistWithCovarianceStamped, '/velocity/gps_nonholonomic', 10)

        # TODO: change to best effort strategy?
        self.gps_fix_sub = self.create_subscription(
            NavSatFix, "/fix", self.update_gps_velocity, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_gps_velocity)

        # the latest velocity calculation
        self.latest_enu_twist: TwistWithCovarianceStamped = TwistWithCovarianceStamped()
        self.latest_ned_twist: TwistWithCovarianceStamped = TwistWithCovarianceStamped()
        self.latest_nonholonomic_twist: TwistWithCovarianceStamped = TwistWithCovarianceStamped()
        self.initialized: bool = False

        self.last_gps_coordinate: NavSatFix = None

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

            # x_enu, y_enu = calc_goal(self.last_gps_coordinate.latitude, self.last_gps_coordinate.longitude,
            #                          message.latitude, message.longitude, True)
            
            x_ned, y_ned = calc_goal(self.last_gps_coordinate.latitude, self.last_gps_coordinate.longitude,
                                     message.latitude, message.longitude )
            #print(self.last_gps_coordinate)

            time_difference_in_second: float = (message.header.stamp.sec + message.header.stamp.nanosec/(
                10**9)) - (self.last_gps_coordinate.header.stamp.sec + self.last_gps_coordinate.header.stamp.nanosec/(10**9))
        
            #self.get_logger().info("X {0} y {1}  time {2}".format( x_ned, y_ned, time_difference_in_second))
            # self.latest_twist.twist.twist.linear.x = distance/time_difference_in_second

            self.latest_enu_twist.twist.twist.linear.x = y_ned/time_difference_in_second
            self.latest_enu_twist.twist.twist.linear.y = x_ned / time_difference_in_second

            self.latest_ned_twist.twist.twist.linear.x = x_ned/time_difference_in_second
            self.latest_ned_twist.twist.twist.linear.y = y_ned/time_difference_in_second


            distance = geopy.distance.geodesic((self.last_gps_coordinate.latitude, self.last_gps_coordinate.longitude), (message.latitude, message.longitude)).km
            distance *= 1000 # to meter
            self.latest_nonholonomic_twist.twist.twist.linear.x = distance/time_difference_in_second
            self.latest_nonholonomic_twist.twist.twist.linear.y = 0.0
                      
            self.last_gps_coordinate=  message

    def publish_gps_velocity(self):
        if (self.initialized and self.last_gps_coordinate is not None):

            self.latest_enu_twist.header.stamp = self.get_clock().now().to_msg()
            self.enu_velocity_published.publish(self.latest_enu_twist)

            self.latest_ned_twist.header.stamp = self.get_clock().now().to_msg()
            self.ned_velocity_published.publish(self.latest_ned_twist)

            self.latest_nonholonomic_twist.header.stamp = self.get_clock().now().to_msg()
            self.nonholonomic_velocity_published.publish(self.latest_nonholonomic_twist)
        else:
            # initialize all other data in self.latest_twist

            self.latest_enu_twist.twist.twist.linear.y = 0.0
            self.latest_enu_twist.twist.twist.linear.z = 0.0
            self.latest_enu_twist.twist.twist.angular.x = 0.0
            self.latest_enu_twist.twist.twist.angular.y = 0.0
            self.latest_enu_twist.twist.twist.angular.z = 0.0

            # the uncertainty in twist measurement

            # since the gps could be off by +/- 1 meters.
            # we make the inaccuracy of velocity x to be 0.5 meters/s

            self.latest_enu_twist.twist.covariance = [
                0.5**2,   0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.5**2,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
            ]
            self.latest_enu_twist.header.frame_id = "base_link"

            
            
            
            self.latest_ned_twist.twist.twist.linear.y = 0.0
            self.latest_ned_twist.twist.twist.linear.z = 0.0
            self.latest_ned_twist.twist.twist.angular.x = 0.0
            self.latest_ned_twist.twist.twist.angular.y = 0.0
            self.latest_ned_twist.twist.twist.angular.z = 0.0

            # the uncertainty in twist measurement

            # since the gps could be off by +/- 1 meters.
            # we make the inaccuracy of velocity x to be 0.5 meters/s

            self.latest_ned_twist.twist.covariance = [
                0.5**2,   0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.5**2,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
            ]
            # although there is a tf transform between gps_link and base_link
            # however, the calculate_gps_offset has already convert yaw from NED to ENU
            self.latest_ned_twist.header.frame_id = "gps_link"




            self.latest_nonholonomic_twist.twist.twist.linear.z = 0.0
            self.latest_nonholonomic_twist.twist.twist.angular.x = 0.0
            self.latest_nonholonomic_twist.twist.twist.angular.y = 0.0
            self.latest_nonholonomic_twist.twist.twist.angular.z = 0.0

            # the uncertainty in twist measurement

            # since the gps could be off by +/- 1 meters.
            # we make the inaccuracy of velocity x to be 0.5 meters/s

            self.latest_nonholonomic_twist.twist.covariance = [
                0.5**2,   0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.5**2,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,      0.0,  0.0,  0.0,  0.0,  0.0,
            ]
            self.latest_nonholonomic_twist.header.frame_id = "base_link"


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
    
    print(calc_goal(34.84136489243739, -82.41178985244608, 34.84131419440653, -82.41170652710007))
    # print(calc_goal(34.84136167706303, -82.4117820168262,34.841482893581954, -82.41171999716181,False))
    main()
