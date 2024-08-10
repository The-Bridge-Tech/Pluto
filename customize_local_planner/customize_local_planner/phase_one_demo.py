"""
Publishes path with goal poses to local_planner node
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
WAYPOINTS = [
        (34.841384, -82.411669),        # front-left corner
        (34.841254, -82.411731),        # back-left corner
        (34.841327, -82.411853),        # back-right corner
        (34.841434, -82.411776),        # front-right corner
        (34.841384, -82.411669),        # front-left corner (return to #1)
]
WAYPOINT_RADIUS = 1.0

class PhaseOneDemo(Node):

    def __init__(self):
        super().__init__('PhaseOneDemo')
        # Publishers
        self.tuning_plan_publisher = self.create_publisher(
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
        self.current_odom: Odometry = None
        self.initial_gps: NavSatFix = None
        self.current_gps: NavSatFix = None
        self.goal_poses = []
        self.pose_i = 0


    # HELPERS

    def calculate_goal_poses(self):
        """Convert GPS waypoints to xy coordinates (goal poses) relative to origin point."""
        for waypoint in WAYPOINTS:
                goal_x, goal_y = calc_goal(
                    origin_lat = self.initial_gps.latitude,
                    origin_long = self.initial_gps.longitude,
                    goal_lat = waypoint[0],
                    goal_long = waypoint[1]
                )
                self.get_logger().info(f"""Distance to navigate to from origin({self.initial_gps.latitude}, {self.initial_gps.longitude}) to goal{waypoint} with distance ({goal_x},{goal_y})""")
                self.get_logger().info(f"Current gps is ({self.initial_gps.latitude}, {self.initial_gps.longitude}), distance is ({goal_x}, {goal_y}) from waypoint {len(self.goal_poses)+1}")
                self.goal_poses.append((goal_x, goal_y))

    def update_goal_pose(self):
        """Update goal pose when current goal is reached."""
        # if within radius of goal -> update
        if(self.distance_from_goal < WAYPOINT_RADIUS):
            # If this is not the last pose to navigate
            if(self.pose_i + 1 < len(self.goal_poses)):
                # Set the next pose in self.goal_poses
                self.pose_i += 1
                self.get_logger().info(f"Reached waypoint #{self.pose_i + 1}")
            else:
                self.get_logger().info("Reached last waypoint!")

    def get_goal_pose(self) -> tuple:
        """Return the current goal pose."""
        return self.goal_poses[self.pose_i]
    
    def calculate_distance_from_goal(self):
        """Calculate the distance between the current position and the current goal pose."""
        # calculate distance remaining from goal using odom data
        current_pose = self.current_odom.pose.pose.position
        odom_distance_from_goal = math.dist(
            (current_pose.x, current_pose.y),
            self.get_goal_pose()
        )
        # calculate distance remaining from goal using gps data
        gps_distance_from_goal = haversine(
            lat1 = self.current_gps.latitude,
            lon1 = self.current_gps.longitude,
            lat2 = WAYPOINTS[self.pose_i][0],
            lon2 = WAYPOINTS[self.pose_i][1]
        )
        # select the calculation with the smallest result (will be most accurate in nearly every case)
        self.distance_from_goal = min(
            odom_distance_from_goal,
            gps_distance_from_goal
        )


    # PUBLISHER CALLBACKS

    def publish_tuning_plan(self):
        """Publish path based on the current goal pose to local_planner node."""
        if self.current_odom == None or self.initial_gps == None:
            self.get_logger().info("Waiting for odom and gps data to be initalized.")
            return
        # (gps data is now available)
        self.ready_to_ping = True
        # calculate goal poses (waypoints) from gps points (only runs the first time gps data is available)
        if len(self.goal_poses) == 0:
            self.calculate_goal_poses()

        # change mode from pure_pursuit_controller to autonomous
        self.tuning_is_autonomous_mode_publisher.publish(Bool(data=True))
        self.tuning_is_pure_pursuit_controller_mode_publisher.publish(Bool(data=False))

        # calculate the current distance from goal
        self.calculate_distance_from_goal()

        # CREATE PATH
        currentPoseStamp = PoseStamped(pose=self.current_odom.pose.pose)
        goalPose = Pose()

        # if(tuningStraight):
        #     goalPose.position.x = 5.0
        # else:
        #     # means turning pid
        #     # move 90 degree to north
        #     goalPose.orientation.x = 0.0
        #     goalPose.orientation.y = 0.0
        #     goalPose.orientation.z = 0.7071068
        #     goalPose.orientation.w = 0.7071068

        # Update goal pose
        self.update_goal_pose()
        # Configure goal pose stamp
        goalPose.position.x, goalPose.position.y = self.get_goal_pose()
        goalPoseStamp = PoseStamped(pose=goalPose)
        # Create path from poses
        path = Path(
            poses =  [
                currentPoseStamp,
                goalPoseStamp, # Why do we need 3 of the same goal poses in the path?
                goalPoseStamp,
                goalPoseStamp
            ]
        )
        # PUBLISH PATH TO LOCAL_PLANNER
        self.tuning_plan_publisher.publish(path)

    def publish_waypoint_ping(self):
        """Publish waypoint info to splunk_logger node and gps_plotter node."""
        # wait until publish_tuning_plan() has created the data to publish
        if self.ready_to_ping:
            roll, pitch, yaw = euler_from_quaternion(self.current_odom.pose.pose.orientation)
            yaw_degrees = math.degrees(yaw)
            ping = WaypointMsg(
                waypoint_number = self.pose_i+1,
                distance = self.distance_from_goal,
                yaw = yaw_degrees # orientation around the vertical axis
            )
            # log so that you can see realtime messages on monitor in phaseOne terminal
            lat = self.initial_gps.latitude
            long = self.initial_gps.longitude
            lat_str = f"{int(lat)}° {((lat - int(lat)) * 60)}"
            long_str = f"{int(long)}° {((long - int(long)) * 60)}"
            self.get_logger().info(f"GPS: {(lat_str, long_str)}\tHeading: {yaw_degrees}°\tDistance: {self.distance_from_goal}\tWaypoint #{self.pose_i+1}")
            self.ping_publisher.publish(ping)
    

    # SUBSCRIBER CALLBACKS
    
    def gps_fix_callback(self, gps_data: NavSatFix):
        """Set initial gps and update gps."""
        if self.initial_gps == None:
            self.initial_gps = gps_data
        self.current_gps = gps_data

    def globalOdometryCallback(self, odom: Odometry):
        """Update global odometry."""
        self.current_odom = odom


# MAIN

def main(args=None):
    rclpy.init(args=args)
    phase_one_demo = PhaseOneDemo()

    rclpy.spin(phase_one_demo)

    phase_one_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
