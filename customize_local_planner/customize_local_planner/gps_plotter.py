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
import math

# HELPER MODULES
from .phase_one_demo import WAYPOINTS, WAYPOINT_RADIUS
from .untilit import meters_to_gps_degrees, haversine, calculateEulerAngleFromOdometry


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
HEADING_LINE_LENGTH = 0.000025


class GPSPlotter(Node):

        def __init__(self):
                super().__init__('gps_plotter')

                # SUBSCRIBERS
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
                self.lastWaypointNumber = 1
                self.currentWaypointNumber = 1
                self.currentDistance = None

                # TIMERS
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
                ]
                # Turn on interactive mode
                plt.ion()
                # Create figure and axes
                self.fig, self.ax = plt.subplots()
                # Show the background image
                self.ax.imshow(
                        self.img, 
                        extent=self.img_extent, 
                        aspect='auto'
                )
                # Create plot for dynamic gps points (trail)
                self.scatter = self.ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='blue', 
                        marker='o',
                        markersize=2,
                        label='Trail'
                )[0] # get the first and only item in the list returned by Axes.plot()
                self.current_position_scatter = self.ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='cyan', 
                        marker='+',
                        markersize=10,
                        markeredgewidth=2,
                        label='Current Position'
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Plot the fence corner points
                self.fence_corners_scatter = self.ax.scatter(
                        x = [p[1] for p in FENCE_GPS_POINTS],   # longitudes
                        y = [p[0] for p in FENCE_GPS_POINTS],   # latitudes
                        s = 5,                                  # marker-size
                        color='black', 
                        marker='o', 
                        label='Fence Corners'
                )
                # Plot the future waypoints
                self.future_waypoints_scatter = self.ax.plot(
                        [p[1] for p in WAYPOINTS[1:-1]],  # longitudes
                        [p[0] for p in WAYPOINTS[1:-1]],  # latitudes
                        'o',                              # only points, no lines
                        color='white', 
                        marker='o',
                        markersize=2,
                        label='Waypoints'
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Plot the current waypoint
                self.current_waypoint_scatter = self.ax.plot(
                        [WAYPOINTS[0][1]],  # longitude
                        [WAYPOINTS[0][0]],  # latitude
                        'o',                # only points, no lines
                        color='red', 
                        marker='o',
                        markersize=2,
                        label='Current Waypoint'
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Add radius circles around each waypoint
                self.waypoint_radius_circles = list[patches.Circle]()
                for waypoint in WAYPOINTS[:-1]:
                        circle = patches.Circle(
                                (waypoint[1], waypoint[0]),  # (x=longitude, y=latitude)
                                meters_to_gps_degrees(WAYPOINT_RADIUS, waypoint[0]),  # Convert meter radius to degrees
                                edgecolor='white', 
                                facecolor='none', 
                                # linestyle='--',
                        )
                        self.waypoint_radius_circles.append(circle)
                        self.ax.add_patch(circle)
                # Update the color of the radius circle around the current waypoint
                self.waypoint_radius_circles[0].set_edgecolor('red')
                # Initialize heading line
                self.heading_line = None
                # Allow the plot to be dynamic
                self.animation = animation.FuncAnimation(
                        fig = self.fig, 
                        func = self.update_plot, 
                        interval=1000 # ms
                )
                # Finish plot
                plt.legend()
                plt.xlabel('Longitude')
                plt.ylabel('Latitude')
                plt.title('Dynamic GPS Plotter')
        

        # HELPERS

        def getCurrentWaypoint(self) -> tuple:
                return WAYPOINTS[self.currentWaypointNumber - 1]
        
        def updateWaypoints(self):
                if self.currentWaypointNumber > self.lastWaypointNumber:
                        self.current_waypoint_scatter.set_data(
                                [self.getCurrentWaypoint()[1]], # longitude
                                [self.getCurrentWaypoint()[0]]  # latitude
                        )
                        self.future_waypoints_scatter.set_data(
                                [p[1] for p in WAYPOINTS[self.currentWaypointNumber:-1]],  # longitudes
                                [p[0] for p in WAYPOINTS[self.currentWaypointNumber:-1]],  # latitudes
                        )
                        for i, circle in enumerate(self.waypoint_radius_circles):
                                n = i + 1 # waypoint number
                                # if this belongs to the last reached waypoint
                                if n == self.lastWaypointNumber:
                                        circle.set_edgecolor('green')
                                # if this belongs to the current waypoint
                                elif n == self.currentWaypointNumber:
                                        circle.set_edgecolor('red')
                        self.lastWaypointNumber = self.currentWaypointNumber

        def getCurrentHeading(self) -> float:
                return calculateEulerAngleFromOdometry(self.currentOdom)
        
        def drawHeadingLine(self):
                # Remove previous heading line
                if self.heading_line:
                        self.heading_line.remove()
                # Get start point
                start_x = self.longitudes[-1] # current longitude
                start_y = self.latitudes[-1] # current latitude
                # Calculate x and y change based on the heading
                heading_rad = math.radians(self.getCurrentHeading())
                dx = HEADING_LINE_LENGTH * math.cos(heading_rad)
                dy = HEADING_LINE_LENGTH * math.sin(heading_rad)
                # Calculate end point by adding the change to start point
                end_x = start_x + dx
                end_y = start_y + dy
                # Draw the line indicating the heading direction
                self.heading_line, = self.ax.plot(
                        [start_x, end_x], 
                        [start_y, end_y], 
                        color='red',
                        linewidth=2
                )


        # CALLBACKS

        def gps_callback(self, gps_msg: NavSatFix):
                """Update current GPS."""
                self.currentGPS = gps_msg

        def odom_callback(self, odom_msg: Odometry):
                """Update current odometry."""
                self.currentOdom = odom_msg

        def waypoint_ping_callback(self, ping_msg: WaypointMsg):
                """Update current waypoint number."""
                if ping_msg.waypoint_number > self.currentWaypointNumber:
                        self.currentWaypointNumber = ping_msg.waypoint_number


        # INTERVAL UPDATES

        def update_plot(self, frame):
                """Update scatter plot with gps points."""
                # If there are dynamic gps points to plot
                if self.latitudes and self.longitudes:
                        # Update all points (trail)
                        self.scatter.set_data(
                                self.longitudes,
                                self.latitudes
                        )
                        # Update the last point (current position)
                        self.current_position_scatter.set_data(
                                self.longitudes[-1:], 
                                self.latitudes[-1:]
                        )
                        # If odom data is available (for heading)
                        if self.currentOdom:
                                # Re-draw heading line
                                self.drawHeadingLine()
                # Update the current waypoint
                self.updateWaypoints()
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
                currentWaypoint = self.getCurrentWaypoint()
                # Calculate distance from current gps to the current waypoint
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