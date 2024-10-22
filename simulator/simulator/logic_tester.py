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
HEIGHT = 1.016 # m (40 in)
MAX_ENGINE_POWER = 16032.55 # Watts (21.5 hp)
MAX_ENGINE_RPM = 3300 # ± 100 rpm
MAX_ENGINE_ANG_VEL = MAX_ENGINE_RPM * ((2*math.pi)/60)
MAX_LINEAR_VELOCITY = 2.90576 # m/s (6.5 mph)
MAX_TORQUE = 53.2 # Nm at 2200 rpm
MAX_TORQUE_RPM = 2200 # rpm

# PHYSICS PARAMETERS
GRAVITY = 9.81 # m/s^2
COEFF_OF_FRICTION = 0.40 # 0.35 to 0.55 for dry grass
DRAG_COEFF = 0.9 # 0.7 to 1.1 for non-streamlined vehicles
AIR_DENSITY = 1.225  # kg/m^3 (standard air density at sea level)

# PHYSICS CALCULATIONS
MOMENT_OF_INERTIA = (1/12) * MASS * (LENGTH**2 + WIDTH**2) # kg*m^2
FRICTION_FORCE = COEFF_OF_FRICTION * MASS * GRAVITY # N
FRONTAL_AREA = WIDTH * HEIGHT # m^2
DRAG_FORCE = lambda linear_vel: 0.5 * DRAG_COEFF * AIR_DENSITY * FRONTAL_AREA * linear_vel**2 # N




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
                self.heading = INITIAL_HEADING  # ° (-180° to 180°)
                self.x0, self.y0 = lat_lon_to_utm(*BASE_GPS) # m
                self.x, self.y = 0.0, 0.0 # m
                self.angular_vel = 0.0 # rad/s
                self.linear_vel = 0.0 # m/s
                self.left_vel = 0.0 # m/s
                self.right_vel = 0.0 # m/s


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
                        # update physics quantities
                        self.update_physics()
                        # publish new heading
                        self.publish_heading(self.heading)
                        # only publish gps every second
                        if self.counter % PUBLISH_RATE == 0:
                                lon, lat = utm_to_lat_lon(
                                        self.x0 + self.x, 
                                        self.y0 + self.y
                                )
                                self.publish_gps(lat, lon)
                        # self.get_logger().info(f"w: {self.angular_vel} v: {self.linear_vel}")
                # update counter
                self.counter += 1

        
        # HELPERS - PHYSICS

        def update_physics(self):
                # VELOCITY
                # map PWM % to velocity of each wheel
                self.left_vel = (self.left_pwm / 100.0) * MAX_LINEAR_VELOCITY
                self.right_vel = (self.right_pwm / 100.0) * MAX_LINEAR_VELOCITY
                # calculate linear and angular velocity of the mower
                self.linear_vel = (self.left_vel + self.right_vel) / 2                  # average
                self.angular_vel = (self.right_vel - self.left_vel) / TRACK_WIDTH       # difference / 2*radius
                # calculate deceleration due to drag
                deceleration = DRAG_FORCE(self.linear_vel) / MASS       # a = F/m
                # apply deceleration to linear velocity
                delta_time = 1 / PUBLISH_RATE                   # Δt = 1/f
                self.linear_vel -= deceleration * delta_time    # Δv = aΔt
                # DIRECTION (HEADING)
                # calculate change in heading
                delta_theta = self.angular_vel * delta_time     # Δθ = ωΔt
                # update heading
                self.heading += math.degrees(delta_theta)
                # POSITION (GPS)
                # calculate velocity x & y components
                theta = math.radians(self.heading)
                vx = self.linear_vel * math.cos(theta)          # v_x = v*cos(θ)
                vy = self.linear_vel * math.sin(theta)          # v_y = v*sin(θ)
                # calculate displacement x & y components
                dx = vx * delta_time                            # Δx = v_x*t
                dy = vy * delta_time                            # Δy = v_y*t
                # update total displacement
                self.x += dx
                self.y += dy
                self.get_logger().info(f"x: {round(self.x, 3)}\t y: {round(self.y, 3)}\t v: {round(self.linear_vel, 3)}")


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