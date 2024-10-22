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
from customize_local_planner.conversions import *
from customize_local_planner.gps_plotter import BASE_GPS

# PARAMETERS (must be declared even when using a YAML file)
DEFAULT_PARAMS = {
        # MOWER PHYSICAL PROPERTIES (Husqvarna Z246)
        "MASS": 0.0,
        "WHEEL_SEPARATION": 0.0,
        "WIDTH": 0.0,
        "LENGTH": 0.0,
        "HEIGHT": 0.0,

        # MOWER ENGINE PROPERTIES (Husqvarna Z246)
        "MAX_POWER": 0.0,
        "MAX_RPM": 0.0,
        "MAX_TORQUE": 0.0,
        "MAX_TORQUE_RPM": 0.0,
        "MAX_LINEAR_VEL": 0.0,
        "MAX_LEFT_BACKWARD_VEL": 0.0,
        "MAX_RIGHT_BACKWARD_VEL": 0.0,
        "MAX_LEFT_FORWARD_VEL": 0.0,
        "MAX_RIGHT_FORWARD_VEL": 0.0,

        # PHYSICS PARAMETERS
        "GRAVITY": 0.0,
        "COEFF_OF_FRICTION": 0.0,
        "DRAG_COEFF": 0.0,
        "AIR_DENSITY": 0.0,

        # OTHER
        "PUBLISH_RATE": 0.0,
        "INITIAL_HEADING": 0.0,
}


class LogicTester(Node):

        def __init__(self):
                super().__init__("logic_tester")

                # PARAMETERS
                # declare all parameters with default values
                for name, value in DEFAULT_PARAMS.items():
                        self.declare_parameter(name, value)
                # load parameter values from YAML file (pluto_launch/config/logic_tester.yaml)
                load_param = lambda param_name: self.get_parameter(param_name).get_parameter_value()
                # MOWER PHYSICAL PROPERTIES (Husqvarna Z246)
                self.MASS = load_param("MASS").double_value
                self.WHEEL_SEPARATION = load_param("WHEEL_SEPARATION").double_value
                self.WIDTH = load_param("WIDTH").double_value
                self.LENGTH = load_param("LENGTH").double_value
                self.HEIGHT = load_param("HEIGHT").double_value
                # MOWER ENGINE PROPERTIES (Husqvarna Z246)
                self.MAX_POWER = load_param("MAX_POWER").double_value
                self.MAX_RPM = load_param("MAX_RPM").double_value
                self.MAX_TORQUE = load_param("MAX_TORQUE").double_value
                self.MAX_TORQUE_RPM = load_param("MAX_TORQUE_RPM").double_value
                self.MAX_LINEAR_VEL = load_param("MAX_LINEAR_VEL").double_value
                self.MAX_LEFT_BACKWARD_VEL = load_param("MAX_LEFT_BACKWARD_VEL").double_value
                self.MAX_RIGHT_BACKWARD_VEL = load_param("MAX_RIGHT_BACKWARD_VEL").double_value
                self.MAX_LEFT_FORWARD_VEL = load_param("MAX_LEFT_FORWARD_VEL").double_value
                self.MAX_RIGHT_FORWARD_VEL = load_param("MAX_RIGHT_FORWARD_VEL").double_value
                # PHYSICS PARAMETERS
                self.GRAVITY = load_param("GRAVITY").double_value
                self.COEFF_OF_FRICTION = load_param("COEFF_OF_FRICTION").double_value
                self.DRAG_COEFF = load_param("DRAG_COEFF").double_value
                self.AIR_DENSITY = load_param("AIR_DENSITY").double_value
                # OTHER
                self.PUBLISH_RATE = load_param("PUBLISH_RATE").double_value
                self.INITIAL_HEADING = load_param("INITIAL_HEADING").double_value

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
                self.heading = self.INITIAL_HEADING  # ° (-180° to 180°)
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
                        self.publish_heading(self.INITIAL_HEADING)
                # start autonomous mode (to allow some nodes to start subscribing)
                elif self.counter == self.seconds_to_counts(1.0):
                        self.publish_autonomous_mode(True)
                # re-publish initial gps
                elif self.counter == self.seconds_to_counts(1.5):
                        self.publish_gps(*BASE_GPS)
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