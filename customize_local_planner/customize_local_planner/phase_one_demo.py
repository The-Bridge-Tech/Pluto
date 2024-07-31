"""
Calculates tuning plan and publishes path to local_planner
"""


# ROS MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path
from custom_msgs.msg import WaypointMsg

# CALCULATION MODULES
import math

# HELPER MODULES
from .untilit import *

# CONSTANTS
GPS_POINTS = [
    (34.841433, -82.411767),    # front-right corner
    (34.841367,-82.411833),     # back-right corner
    (34.841283,  -82.411717),   # back-left corner
    (34.84135, -82.4117)        # front-left corner
]


class PhaseOneDemo(Node):

    def __init__(self):
        super().__init__('PhaseOneDemo')
        # Publishers
        self.tuning_local_plan_publisher = self.create_publisher(
            Path, 
            "local_plan",
            10
        )
        self.tuning_is_pure_pursuit_controller_mode_publisher = self.create_publisher(
            Bool, 
            "is_pure_pursuit_controller_mode", 
            10
        )
        self.tuning_is_autonomous_mode_publisher = self.create_publisher(
            Bool, 
            "is_autonomous_mode", 
            10
        )
        self.ping_publisher = self.create_publisher(
            WaypointMsg,
            "/waypoint_ping",
            10
        )
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 
            "odometry/global", 
            self.globalOdometryCallback, 
            10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, 
            "/fix", 
            self.gps_fix_callback, 
            10
        )
        # Timers
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(
            timer_period, 
            self.publish_tuning_plan
        )
        self.ready_to_ping = False
        ping_timer_period = 1 # seconds
        self.ping_timer = self.create_timer(
            ping_timer_period,
            self.publish_waypoint_ping
        )
        # Variables
        self.latest_odom: Odometry = None
        self.initial_gps: NavSatFix = None
        self.pose_to_navigate = []
        self.current_pose_to_navigate_index = 0

    def publish_tuning_plan(self):
        """Publish path based on the current goal pose to local_planner node."""
        if self.latest_odom == None or self.initial_gps == None:
            self.get_logger().info("Waiting for odom and gps data to be initalized.")
            return
        # (gps data is now available)
        # calculate goal poses (waypoints) from gps points (only runs the first time gps data is available)
        if len(self.pose_to_navigate) == 0:
            for gps_tuple in GPS_POINTS:
                goal_x, goal_y = calc_goal(
                    origin_lat=self.initial_gps.latitude,
                    origin_long=self.initial_gps.longitude,
                    goal_lat=gps_tuple[0],
                    goal_long=gps_tuple[1]
                )
                self.get_logger().info(f"""Distance to navigate to from origin({self.initial_gps.latitude}, {self.initial_gps.longitude}) to goal{gps_tuple} with distance ({goal_x},{goal_y})""")
                self.get_logger().info(f"Current gps is ({self.initial_gps.latitude}, {self.initial_gps.longitude}), distance is ({goal_x}, {goal_y}) from waypoint {len(self.pose_to_navigate) +1}")
                self.pose_to_navigate.append((goal_x, goal_y))
        
        publishPath = Path()

        msg = Bool()
        msg.data = True
        self.tuning_is_autonomous_mode_publisher.publish(msg)
        msg = Bool()
        msg.data = False
        self.tuning_is_pure_pursuit_controller_mode_publisher.publish(msg)

        current_pose = self.latest_odom.pose.pose.position # NOTE current position is determined from odometry
        next_pose = self.pose_to_navigate[self.current_pose_to_navigate_index]
        self.distance_remaining_from_goal = math.dist(
            (current_pose.x, current_pose.y),
            (next_pose[0], next_pose[1])
        )

        self.ready_to_ping = True

        currentPosePoseStamp = PoseStamped()
        currentPosePoseStamp.pose = self.latest_odom.pose.pose

        goalPosePoseStamp = PoseStamped()
        tempPose = Pose()

        # if(tuningStraight):
        #     tempPose.position.x = 5.0
        # else:
        #     # means turning pid
        #     # move 90 degree to north
        #     tempPose.orientation.x = 0.0
        #     tempPose.orientation.y = 0.0
        #     tempPose.orientation.z = 0.7071068
        #     tempPose.orientation.w = 0.7071068

        # UPDATE GOAL POSE WHEN REACHED
        if(self.distance_remaining_from_goal < 0.5): # (within 0.5 of goal - close enough)
            # If this is not the last pose to navigate
            if(self.current_pose_to_navigate_index + 1 < len(self.pose_to_navigate)):
                # Set the next pose in self.pose_to_navigate
                self.current_pose_to_navigate_index += 1
                next_pose = self.pose_to_navigate[self.current_pose_to_navigate_index]
                self.get_logger().info(f"Reached waypoint #{self.current_pose_to_navigate_index + 1}")
            else:
                self.get_logger().info("Reached last waypoint!")
        
        tempPose.position.x, tempPose.position.y = next_pose[0:2]

        goalPosePoseStamp.pose = tempPose
        publishPath.poses =  [
            currentPosePoseStamp,
            goalPosePoseStamp,
            goalPosePoseStamp,
            goalPosePoseStamp
        ]
        # PUBLISH PATH TO LOCAL_PLANNER
        self.tuning_local_plan_publisher.publish(publishPath)
    
    def gps_fix_callback(self, gps_data: NavSatFix):
        """Update GPS data."""
        if self.initial_gps == None:
            self.initial_gps = gps_data

    def globalOdometryCallback(self, odom: Odometry):
        """Update global odometry."""
        self.latest_odom = odom

    def publish_waypoint_ping(self):
        # wait until publish_tuning_plan() has created the data to publish
        if self.ready_to_ping:
            roll, pitch, yaw = euler_from_quaternion(self.latest_odom.pose.pose.orientation)
            yaw_degrees = math.degrees(yaw)
            ping = WaypointMsg(
                waypoint_number = self.current_pose_to_navigate_index+1,
                distance = self.distance_remaining_from_goal,
                yaw = yaw_degrees # orientation around the vertical axis
            )
            # log so that you can see realtime messages on monitor in phaseOne terminal
            lat = self.initial_gps.latitude
            long = self.initial_gps.longitude
            lat_str = f"{int(lat)}° {((lat - int(lat)) * 60)}"
            long_str = f"{int(long)}° {((long - int(long)) * 60)}"
            self.get_logger().info(f"GPS: {(lat_str, long_str)}\tHeading: {yaw_degrees}°\tDistance: {self.distance_remaining_from_goal}\tWaypoint #{self.current_pose_to_navigate_index+1}")
            self.ping_publisher.publish(ping)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PhaseOneDemo()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
