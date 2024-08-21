"""
Show a realtime map/plot of the current gps location and waypoints
Author: Matthew Lauriault
Created: 8/2/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry, Path
from custom_msgs.msg import WaypointMsg
# CALCULATION MODULES
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.image as mpimg
import matplotlib.patches as patches
from threading import Thread
import os

# HELPER MODULES
from .phase_one_demo import WAYPOINTS, WAYPOINT_RADIUS
from .untilit import meters_to_gps_degrees, haversine


# CONSTANTS
FENCE_GPS_POINTS=[
        (34.8414762, -82.4118085),      # front-right corner 
        (34.8413314, -82.4119220),      # back-right corner
        (34.8412136, -82.4116773),      # back-left corner
        (34.8413433, -82.4115738),      # front-left corner
]
MAP_IMAGE_POINTS = [
        (34.8414997, -82.4119735),      # top-left
        (34.8411603, -82.4114941)       # bottom-right
]
MAP_IMAGE_LOCATION = {
        "left":   MAP_IMAGE_POINTS[0][1],
        "right":  MAP_IMAGE_POINTS[1][1],
        "top":    MAP_IMAGE_POINTS[0][0],
        "bottom": MAP_IMAGE_POINTS[1][0],
}
PARENT_DIR = os.path.join(
        "src", 
        "Pluto", 
        "customize_local_planner", 
        "customize_local_planner"
)
MAP_IMAGE_DIR = os.path.join(
        PARENT_DIR, 
        "map3.png"
)


class GPSPlotter(Node):

        def __init__(self):
                super().__init__('gps_plotter')
                # Subscribe to gps topic to get initial gps data
                self.gps_subscriber = self.create_subscription(
                        NavSatFix, 
                        "/fix", 
                        self.gps_callback, 
                        10
                )
                self.currentGPS = None
                # For storing GPS coordinates
                self.latitudes = []
                self.longitudes = []
                # Subscribe to odometry topic
                self.odom_subscriber = self.create_subscription(
                        Odometry, 
                        "odometry/global", 
                        self.odom_callback, 
                        10
                )
                self.currentOdom = None
                # Subscribe to waypoint ping topic
                self.waypoint_ping_subscriber = self.create_subscription(
                        WaypointMsg,
                        "/waypoint_ping",
                        self.waypoint_ping_callback,
                        10
                )
                self.currentWaypointNumber = 1
                self.currentDistance = None
                # Timer to process current data (don't want to process each time data is received)
                process_timer_period = 0.3
                self.process_timer = self.create_timer(
                        process_timer_period, 
                        self.process
                )
                # PLOT
                # Load background image (map)
                self.img = mpimg.imread(MAP_IMAGE_DIR)
                self.img_extent = [
                        MAP_IMAGE_LOCATION["left"],
                        MAP_IMAGE_LOCATION["right"],
                        MAP_IMAGE_LOCATION["bottom"],
                        MAP_IMAGE_LOCATION["top"]
                ]  # [left, right, bottom, top] in plot coordinates
                # Turn on interactive mode
                plt.ion()
                # Create figure and axes
                self.fig, self.ax = plt.subplots()
                # Show the background image
                self.ax.imshow(self.img, extent=self.img_extent, aspect='auto')
                # Create plot for dynamic gps points (trail)
                self.scatter, = self.ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='blue', 
                        marker='o',
                        markersize=2,
                        label='Trail'
                )
                # Plot the fence corner points
                self.fence_corners_scatter = self.ax.scatter(
                        x = [p[1] for p in FENCE_GPS_POINTS],   # longitudes
                        y = [p[0] for p in FENCE_GPS_POINTS],   # latitudes
                        s = 3,                                  # marker-size
                        color='white', 
                        marker='o', 
                        label='Fence Corners'
                )
                # Plot the fence lines between the fence points
                # self.fence_lines, = self.ax.plot(
                #         [p[1] for p in FENCE_GPS_POINTS + [FENCE_GPS_POINTS[0]]],
                #         [p[0] for p in FENCE_GPS_POINTS + [FENCE_GPS_POINTS[0]]],
                #         color='red',
                # )
                # Plot the waypoints
                self.waypoints_scatter = self.ax.scatter(
                        x = [p[1] for p in WAYPOINTS],  # longitudes
                        y = [p[0] for p in WAYPOINTS],  # latitudes
                        s = 2,                          # marker-size
                        color='red', 
                        marker='o',
                        label='Waypoints'
                )
                # Add radius circles around each waypoint
                for waypoint in WAYPOINTS:
                        circle = patches.Circle(
                                (waypoint[1], waypoint[0]),  # (x=longitude, y=latitude)
                                meters_to_gps_degrees(WAYPOINT_RADIUS, waypoint[0]),  # Convert meter radius to degrees
                                edgecolor='red', 
                                facecolor='none', 
                                # linestyle='--',
                        )
                        self.ax.add_patch(circle)
                # ANIMATION
                self.animation = animation.FuncAnimation(
                        fig = self.fig, 
                        func = self.update_plot, 
                        interval=1000 # ms
                )
                # finish plot
                plt.legend()
                plt.xlabel('Longitude')
                plt.ylabel('Latitude')
                plt.title('Dynamic GPS Plotter')
        
        # HELPERS
        def getCurrentWaypoint(self) -> tuple:
                return WAYPOINTS[self.currentWaypointNumber - 1]

        # CALLBACKS
        def gps_callback(self, gps_msg: NavSatFix):
                """Update current GPS."""
                self.currentGPS = gps_msg
        def odom_callback(self, odom_msg: Odometry):
                """Update current odometry."""
                self.currentOdom = odom_msg
        def waypoint_ping_callback(self, ping_msg: WaypointMsg):
                """Update current waypoint number."""
                self.currentWaypointNumber = ping_msg.waypoint_number

        # INTERVAL UPDATES
        def update_plot(self, frame):
                """Update scatter plot with gps points."""
                self.scatter.set_data(self.longitudes, self.latitudes)
                self.ax.relim()
                self.ax.autoscale_view()
                # Redraw the figure canvas with the latest changes
                self.fig.canvas.draw()
                # Process any pending GUI events
                self.fig.canvas.flush_events()
                return self.scatter,
        def process(self):
                """Process on an interval (timer) rather than every time data is received (when the subscriber callbacks are called)."""
                # Wait until gps and odom data have been received
                if self.currentGPS is None or self.currentOdom is None:
                        self.get_logger().info("Waiting for odom and gps data to be initalized.")
                        return
                # Calculate distance from current gps to the current waypoint
                currentWaypoint = self.getCurrentWaypoint()
                self.currentDistance = haversine(
                        lat1 = self.currentGPS.latitude,
                        lon1 = self.currentGPS.longitude,
                        lat2 = currentWaypoint[0],
                        lon2 = currentWaypoint[1]
                )
                # (gps and odom data are available)
                self.get_logger().info(f'Lat: {self.currentGPS.latitude}\t Lon: {self.currentGPS.longitude}\t Distance: {self.currentDistance}\t Waypoint #{self.currentWaypointNumber}')
                # update latitudes and longitudes with current gps for plot
                self.latitudes.append(self.currentGPS.latitude)
                self.longitudes.append(self.currentGPS.longitude)

# MAIN
def main(args=None):
        rclpy.init(args=args)
        gps_plotter = GPSPlotter()

        # spin the node in a separate thread since plt.show() blocks and is not thread-safe
        node_spin_thread = Thread(
                target=rclpy.spin, 
                args=(gps_plotter,)
        )
        node_spin_thread.start()

        # Show the plot and keep it updated in the main thread
        plt.show(block=True)

        gps_plotter.destroy_node()
        rclpy.shutdown()
        node_spin_thread.join()


# When this file is run as a script
if __name__ == '__main__':
        main()

# bag: rosbag2_2024_08_01-17_14_31