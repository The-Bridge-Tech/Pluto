"""
Logic Test Simulator
Author: Matthew Lauriault
Created: 10/9/24
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
import rclpy.time_source
from std_msgs.msg import Header, Bool, Float32
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion, Vector3
from geodesy import utm

# CALCULATION MODULES
import math

# HELPER MODULES
from customize_local_planner.conversions import *


class LogicTester(Node):

        def __init__(self):
                super().__init__("logic_tester")

                # PARAMETERS
                # load parameter values from YAML file (pluto_launch/config/logic_tester.yaml)
                # MOWER PHYSICAL PROPERTIES (Husqvarna Z246)
                self.MASS = self.load_param_double("MASS")
                self.WHEEL_SEPARATION = self.load_param_double("WHEEL_SEPARATION")
                self.WIDTH = self.load_param_double("WIDTH")
                self.LENGTH = self.load_param_double("LENGTH")
                self.HEIGHT = self.load_param_double("HEIGHT")
                # MOWER ENGINE PROPERTIES (Husqvarna Z246)
                self.MAX_POWER = self.load_param_double("MAX_POWER")
                self.MAX_RPM = self.load_param_double("MAX_RPM")
                self.MAX_TORQUE = self.load_param_double("MAX_TORQUE")
                self.MAX_TORQUE_RPM = self.load_param_double("MAX_TORQUE_RPM")
                self.MAX_LINEAR_VEL = self.load_param_double("MAX_LINEAR_VEL")
                self.MAX_LEFT_BACKWARD_VEL = self.load_param_double("MAX_LEFT_BACKWARD_VEL")
                self.MAX_RIGHT_BACKWARD_VEL = self.load_param_double("MAX_RIGHT_BACKWARD_VEL")
                self.MAX_LEFT_FORWARD_VEL = self.load_param_double("MAX_LEFT_FORWARD_VEL")
                self.MAX_RIGHT_FORWARD_VEL = self.load_param_double("MAX_RIGHT_FORWARD_VEL")
                # PHYSICS PARAMETERS
                self.GRAVITY = self.load_param_double("GRAVITY")
                self.COEFF_OF_FRICTION = self.load_param_double("COEFF_OF_FRICTION")
                self.DRAG_COEFF = self.load_param_double("DRAG_COEFF")
                self.AIR_DENSITY = self.load_param_double("AIR_DENSITY")
                # OTHER
                self.PUBLISH_RATE = self.load_param_double("PUBLISH_RATE")
                self.INITIAL_HEADING = self.load_param_double("INITIAL_HEADING")
                self.INITIAL_LATITUDE = self.load_param_double("INITIAL_LATITUDE")
                self.INITIAL_LONGITUDE = self.load_param_double("INITIAL_LONGITUDE")

                # PHYSICS CALCULATIONS
                self.MOMENT_OF_INERTIA = (1/12) * self.MASS * (self.LENGTH**2 + self.WIDTH**2) # kg*m^2
                self.FRICTION_FORCE = self.COEFF_OF_FRICTION * self.MASS * self.GRAVITY # N
                self.FRONTAL_AREA = self.WIDTH * self.HEIGHT # m^2
                self.DRAG_FORCE = lambda linear_vel: 0.5 * self.DRAG_COEFF * self.AIR_DENSITY * self.FRONTAL_AREA * linear_vel**2 # N

                # TIMERS
                self.publish_timer = self.create_timer(
                        1 / self.PUBLISH_RATE,
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
                        Float32,
                        "/steering_left/percentage",
                        self.left_pwm_callback,
                        10
                )
                self.left_pwm = 0
                self.right_pwm_sub = self.create_subscription(
                        Float32,
                        "/steering_right/percentage",
                        self.right_pwm_callback,
                        10
                )
                self.right_pwm = 0

                # VARIABLES
                self.counter = 0
                self.heading = self.INITIAL_HEADING  # ° (-180° to 180°)
                self.initial_utm = utm.fromLatLong(self.INITIAL_LATITUDE, self.INITIAL_LONGITUDE)
                self.x, self.y = 0.0, 0.0 # m
                self.angular_vel = 0.0 # rad/s
                self.linear_vel = 0.0 # m/s
                self.left_vel = 0.0 # m/s
                self.right_vel = 0.0 # m/s


        # HELPERS - PARAMETERS

        def load_param(self, param_name: str, init_value):
                self.declare_parameter(param_name, init_value)
                return self.get_parameter(param_name).get_parameter_value()
        
        def load_param_int(self, param_name: str) -> int:
                return self.load_param(param_name, 0).integer_value
        
        def load_param_double(self, param_name: str) -> float:
                return self.load_param(param_name, 0.0).double_value


        # TIMER CALLBACKS

        def publish_sensor_data(self):
                """Simulate sensor data to observe logic in other nodes"""
                # publish initial gps
                if self.counter == self.seconds_to_counts(0):
                        self.publish_gps(self.INITIAL_LATITUDE, self.INITIAL_LONGITUDE)
                # publish initial heading (Stop -> Turn)
                elif self.counter == self.seconds_to_counts(0.5):
                        self.publish_heading(self.INITIAL_HEADING)
                # start autonomous mode (to allow some nodes to start subscribing)
                elif self.counter == self.seconds_to_counts(1.0):
                        self.publish_autonomous_mode(True)
                # re-publish initial gps
                elif self.counter == self.seconds_to_counts(1.5):
                        self.publish_gps(self.INITIAL_LATITUDE, self.INITIAL_LONGITUDE)
                # re-publish initial heading
                elif self.counter == self.seconds_to_counts(2):
                        self.publish_heading(self.INITIAL_HEADING)

                # publish gps & heading dynamically based on pwm values
                elif self.counter > self.seconds_to_counts(2):
                        # update physics quantities
                        self.update_physics()
                        # publish new heading
                        self.publish_heading(self.heading)
                        # only publish gps every second
                        if self.counter % self.PUBLISH_RATE == 0:
                                new_utm = utm.UTMPoint(
                                        easting = self.initial_utm.easting + self.x,
                                        northing = self.initial_utm.northing + self.y,
                                        altitude = 278.299,
                                        zone = self.initial_utm.zone,
                                        band = self.initial_utm.band
                                )
                                new_gps = new_utm.toMsg()
                                self.publish_gps(new_gps.latitude, new_gps.longitude)
                        # self.get_logger().info(f"w: {self.angular_vel} v: {self.linear_vel}")
                # update counter
                self.counter += 1

        
        # HELPERS - PHYSICS

        def update_physics(self):
                # VELOCITY
                # map PWM % to velocity of each wheel
                self.left_vel = (self.left_pwm / 100.0) * (self.MAX_LEFT_FORWARD_VEL if self.left_pwm >= 0 else self.MAX_LEFT_BACKWARD_VEL)
                self.right_vel = (self.right_pwm / 100.0) * (self.MAX_RIGHT_FORWARD_VEL if self.right_pwm >= 0 else self.MAX_RIGHT_BACKWARD_VEL)
                # calculate linear and angular velocity of the mower
                self.linear_vel = (self.left_vel + self.right_vel) / 2                  # average
                self.angular_vel = (self.right_vel - self.left_vel) / self.WHEEL_SEPARATION       # difference / 2*radius
                # calculate deceleration due to drag
                deceleration = self.DRAG_FORCE(self.linear_vel) / self.MASS       # a = F/m
                # apply deceleration to linear velocity
                delta_time = 1 / self.PUBLISH_RATE                   # Δt = 1/f
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
                return round(self.PUBLISH_RATE * seconds)
        

        # SUBSCRIBER CALLBACKS

        def left_pwm_callback(self, msg: Float32):
                self.left_pwm = msg.data

        def right_pwm_callback(self, msg: Float32):
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