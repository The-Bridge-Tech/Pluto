"""
Logic Test Simulator
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
import rclpy.time_source
from std_msgs.msg import Header, Bool, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion, Vector3

# CALCULATION MODULES
import math

# HELPER MODULES
from .conversions import *

# CONSTANTS
BASE_GPS = (34.841400, -82.411743)

# PARAMETERS
PUBLISH_RATE = 10 # Hz (publishes / second)
INITIAL_HEADING = 127.0 # ° (-180° to 180°)
ANG_ACC_PER_PWM_PERCENT = 0.2 # (°/s^2) / pwm difference %
LINEAR_ACC_PER_PWM_PERCENT = 0.05 # (m/s^2) / pwm average %
FRICTION_COEFF = 0.05  # m/s^2 per m/s of velocity

# MOWER PROPERTIES (Husqvarna Z246)
MASS = 242.672 # kg (535 lbs)
TRACK_WIDTH = 0.9906 # m (39 in)
WIDTH = 1.11125 # m (43.75 in)
LENGTH = 1.905 # m (75 in)
MAX_ENGINE_POWER = 16032.55 # Watts (21.5 hp)
MAX_ENGINE_RPM = 3300 # ± 100 rpm
MAX_ENGINE_ANG_VEL = MAX_ENGINE_RPM * ((2*math.pi)/60)
MAX_LINEAR_VELOCITY = 2.90576 # m/s (6.5 mph)
MAX_TORQUE = 53.2 # Nm at 2200 rpm
MAX_TORQUE_RPM = 2200 # rpm

# PHYSICS QUANTITIES
RADIUS = TRACK_WIDTH / 2
MOMENT_OF_INERTIA = (1/12) * MASS * (LENGTH**2 + WIDTH**2)
COEFF_OF_FRICTION = 0.40 # 0.35 to 0.55 for dry grass
DRAG_COEFF = 0.9 # 0.7 to 1.1 for non-streamlined vehicles
GRAVITY = 9.81 # m/s^2
FRICTION_FORCE = COEFF_OF_FRICTION * MASS * GRAVITY # N


class LogicTester(Node):

        def __init__(self):
                super().__init__("logic_tester")

                # TIMERS
                self.publish_timer = self.create_timer(
                        1 / PUBLISH_RATE,
                        self.publish_sensor_data
                )

                # PUBLISHERS - SENSOR DATA
                self.gps_pub = self.create_publisher(
                        NavSatFix,
                        "/fix",
                        10
                )
                self.imu_pub = self.create_publisher(
                        Imu,
                        "/imu/data",
                        10
                )

                # PUBLISHERS - OTHER
                self.is_autonomous_mode_pub = self.create_publisher(
                        Bool, 
                        "is_autonomous_mode", 
                        1
                )

                # SUBSCRIBERS
                self.left_pwm_sub = self.create_subscription(
                        Float64,
                        "/steering_left/percentage",
                        self.left_pwm_callback,
                        10
                )
                self.left_pwm = 0
                self.right_pwm_sub = self.create_subscription(
                        Float64,
                        "/steering_right/percentage",
                        self.right_pwm_callback,
                        10
                )
                self.right_pwm = 0

                # VARIABLES
                self.counter = 0
                self.heading = INITIAL_HEADING
                self.x, self.y = lat_lon_to_utm(*BASE_GPS)
                self.angular_vel = 0.0
                self.linear_vel = 0.0

                self.vel_left = 0.0
                self.vel_right = 0.0


        # TIMER CALLBACKS

        def publish_sensor_data(self):
                """Simulate sensor data to observe logic in other nodes"""
                # publish initial gps
                if self.counter == self.seconds_to_counts(0):
                        self.publish_gps(*BASE_GPS)
                # publish initial heading (Stop -> Turn)
                elif self.counter == self.seconds_to_counts(0.5):
                        self.publish_heading(INITIAL_HEADING)
                # start autonomous mode (to allow some nodes to start subscribing)
                elif self.counter == self.seconds_to_counts(1.0):
                        self.publish_autonomous_mode(True)
                # re-publish initial gps
                elif self.counter == self.seconds_to_counts(1.5):
                        self.publish_gps(*BASE_GPS)
                # re-publish initial heading
                elif self.counter == self.seconds_to_counts(2):
                        self.publish_heading(INITIAL_HEADING)
 
                # publish gps & heading dynamically based on pwm values
                elif self.counter > self.seconds_to_counts(2):
                        # HEADING
                        # # convert pwm difference to angular displacement 
                        # rotation_pwm = self.right_pwm - self.left_pwm  # (positive = counter-clockwise, negative = clockwise)
                        # time_interval = 1 / PUBLISH_RATE                                        # t = 1/f
                        # angular_acc = ANG_ACC_PER_PWM_PERCENT * rotation_pwm                    # α = (α/pwm) * pwm
                        # angular_dec = FRICTION_COEFF * self.angular_vel                        # drag = bω
                        # self.angular_vel += (angular_acc - angular_dec) * time_interval        # Δω = αt
                        # angular_displacement = self.angular_vel * time_interval                 # θ = ωt
                        # # update heading
                        # self.heading += angular_displacement
                        # # publish new heading

                        # map PWM % to velocity of each wheel
                        self.vel_left = (self.left_pwm / 100.0) * MAX_LINEAR_VELOCITY
                        self.vel_right = (self.right_pwm / 100.0) * MAX_LINEAR_VELOCITY
                        # calculate linear and angular velocity of the mower
                        self.linear_vel = (self.vel_left + self.vel_right) / 2 # average
                        self.angular_vel = (self.vel_right - self.vel_left) / TRACK_WIDTH # difference / 2*radius
                        # calculate change in heading
                        delta_time = 1 / PUBLISH_RATE # t = 1/f
                        delta_theta = self.angular_vel * delta_time # Δθ = ωΔt
                        # update heading
                        self.heading += delta_theta
                        self.publish_heading(self.heading)

                        # GPS
                        # convert pwm average to linear displacement
                        # linear_pwm = (self.left_pwm + self.right_pwm) / 2       # (average of left and right)
                        # linear_acc = LINEAR_ACC_PER_PWM_PERCENT * linear_pwm    # a = (a/pwm) * pwm
                        # self.linear_vel += linear_acc * time_interval           # Δv = at
                        # theta = math.radians(self.heading)
                        # dx = self.linear_vel * time_interval * math.cos(theta)  # Δx = v*cos(θ)*t
                        # dy = self.linear_vel * time_interval * math.sin(theta)  # Δy = v*sin(θ)*t
                        # self.x += dx
                        # self.y += dy
                        # only publish every second
                        if self.counter % PUBLISH_RATE == 0:
                                # lat, lon = utm_to_lat_lon(self.x, self.y)
                                # self.publish_gps(lat, lon)
                                self.publish_gps(*BASE_GPS)
                        
                        # self.get_logger().info(f"alpha: {angular_acc} w: {self.angular_vel}") # a: {linear_acc} v: {self.linear_vel}")

                # update counter
                self.counter += 1


        # HELPERS - PUBLISHING

        def publish_gps(self, lat: float, lon: float):
                msg = NavSatFix(
                        header = Header(
                                stamp = self.get_clock().now().to_msg(),
                                frame_id = "gps_link"
                        ),
                        status = NavSatStatus(
                                status = 0,
                                service = 1
                        ),
                        latitude = lat,
                        longitude = lon,
                        altitude = 278.299,
                        position_covariance = [
                                0.0169, 0.0,    0.0,
                                0.0,    0.0169, 0.0,
                                0.0,    0.0,    0.270
                        ],
                        position_covariance_type = 1
                )
                self.gps_pub.publish(msg)
                self.get_logger().info(f"Published GPS: lat={lat}, lon={lon}")

        def publish_heading(self, angle: float):
                """Angle in degrees from -180 to 180"""
                quaternion = euler_to_quaternion(
                        roll = 0,
                        pitch = 0,
                        yaw = math.radians(angle)
                )
                msg = Imu(
                        header = Header(
                                stamp = self.get_clock().now().to_msg(),
                                frame_id = "imu_link"
                        ),
                        orientation = Quaternion(
                                x = quaternion[0],
                                y = quaternion[1],
                                z = quaternion[2],
                                w = quaternion[3],
                        ),
                        orientation_covariance = [
                                0.0324,    0.0,       0.0,
                                0.0,       0.0324,    0.0,
                                0.0,       0.0,       0.0324
                        ],
                        angular_velocity = Vector3(
                                x = 0.0,
                                y = 0.0,
                                z = math.radians(self.angular_vel)
                        ),
                        angular_velocity_covariance = [
                                0.04000000000000001,    0.0,                    0.0,
                                0.0,                    0.04000000000000001,    0.0,
                                0.0,                    0.0,                    0.04000000000000001
                        ],
                        linear_acceleration = Vector3(
                                x = 0.0,
                                y = 0.0,
                                z = 0.0
                        ),
                        linear_acceleration_covariance = [
                                0.32489999999999997,    0.0,                    0.0,
                                0.0,                    0.32489999999999997,    0.0,
                                0.0,                    0.0,                    0.32489999999999997
                        ]
                )
                self.imu_pub.publish(msg)
                self.get_logger().info(f"Published Heading: angle={angle}")

        def publish_autonomous_mode(self, is_autonomous_mode: bool):
                msg = Bool(
                        data = is_autonomous_mode
                )
                self.is_autonomous_mode_pub.publish(msg)
                self.get_logger().info(f"Published is_autonomous_mode: {is_autonomous_mode}")


        # HELPERS - TIMING

        def seconds_to_counts(self, seconds: int | float) -> int:
                # publish_rate = counts / second
                return round(PUBLISH_RATE * seconds)
        

        # SUBSCRIBER CALLBACKS

        def left_pwm_callback(self, msg: Float64):
                self.left_pwm = msg.data

        def right_pwm_callback(self, msg: Float64):
                self.right_pwm = msg.data

# MAIN

def main(args=None):
        rclpy.init(args=args)

        logic_tester = LogicTester()

        rclpy.spin(logic_tester)

        logic_tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
        main()