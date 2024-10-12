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
from.gps_list import GPSList
from .phase_one_demo import WAYPOINTS, WAYPOINT_RADIUS
from .untilit import meters_to_gps_degrees, haversine, calculateEulerAngleFromOdometry


# CONSTANTS
BASE_GPS = (34.841400, -82.411743)
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

# PARAMETERS
PROCESS_RATE = 10 # Hz (times / second)
HEADING_LINE_LENGTH = 0.000025


class GPSPlotter(Node):

        def __init__(self):
                super().__init__('gps_plotter')

                # SUBSCRIBERS
                # Subscribe to original gps topic
                self.gps_sub = self.create_subscription(
                        NavSatFix, 
                        "/fix/filtered", 
                        self.gps_callback, 
                        10
                )
                self.currentGPS = None
                # For storing original GPS coordinates
                self.original_gps = GPSList()
                # Subscribe to offset gps topic
                self.offset_gps_sub = self.create_subscription(
                        NavSatFix, 
                        "/fix/offset", 
                        self.offset_gps_callback, 
                        10
                )
                self.currentOffsetGPS = None
                # For storing offset (corrected) GPS coordinates
                self.offset_gps = GPSList()
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
                self.process_timer = self.create_timer(
                        1 / PROCESS_RATE, 
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
                # Create plot for original gps points (trail)
                self.original_scatter = self.ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='purple', 
                        marker='o',
                        markersize=2,
                        label='Original Trail'
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Create plot for offset (corrected) gps points (trail)
                self.offset_scatter = self.ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='blue', 
                        marker='o',
                        markersize=2,
                        label='Offset Trail'
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
                # Plot the base point
                self.base_scatter = self.ax.scatter(
                        x = [BASE_GPS[1]],   # longitude
                        y = [BASE_GPS[0]],   # latitude
                        s = 7,               # marker-size
                        color='red', 
                        marker='o', 
                        label='Base Point'
                )
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
                for waypoint in WAYPOINTS:
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
                        interval = (1 / PROCESS_RATE) * 1000 # ms
                )
                # Only update plot if there is new data
                self.new_data = False
                self.new_offset_data = False
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
        
        def drawHeadingLine(self, current_lat: float, current_lon: float):
                # Remove previous heading line
                if self.heading_line:
                        self.heading_line.remove()
                # Get start point
                start_x = current_lon # current longitude
                start_y = current_lat # current latitude
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
        
        def isOutlier(self, lat: float, lon: float) -> bool:
                return (
                        lat > MAP_IMAGE_LOCATION["top"] or
                        lat < MAP_IMAGE_LOCATION["bottom"] or
                        lon > MAP_IMAGE_LOCATION["right"] or
                        lon < MAP_IMAGE_LOCATION["left"]
                )
        
        def updateDistance(self):
                """Update distance from current gps to the current waypoint"""
                currentWaypoint = self.getCurrentWaypoint()
                currentGPS = self.currentGPS if self.currentOffsetGPS is None else self.currentOffsetGPS
                self.currentDistance = haversine(
                        lat1 = currentGPS.latitude,
                        lon1 = currentGPS.longitude,
                        lat2 = currentWaypoint[0],
                        lon2 = currentWaypoint[1]
                )


        # CALLBACKS

        def gps_callback(self, msg: NavSatFix):
                """Update current GPS point"""
                # if new gps point is not an outlier
                if not self.isOutlier(msg.latitude, msg.longitude):
                        # update latitudes and longitudes with current gps for plot
                        self.original_gps.append(
                                lat = msg.latitude,
                                lon = msg.longitude
                        )
                self.currentGPS = msg

        def offset_gps_callback(self, msg: NavSatFix):
                """Update current GPS point with correction offset"""
                # if new gps point is not an outlier
                if not self.isOutlier(msg.latitude, msg.longitude):
                        # update offset latitudes and longitudes with current offset gps for plot
                        self.offset_gps.append(
                                lat = msg.latitude,
                                lon = msg.longitude
                        )
                self.currentOffsetGPS = msg

        def odom_callback(self, msg: Odometry):
                """Update current odometry."""
                self.currentOdom = msg

        def waypoint_ping_callback(self, msg: WaypointMsg):
                """Update current waypoint number."""
                if msg.waypoint_number > self.currentWaypointNumber:
                        self.currentWaypointNumber = msg.waypoint_number


        # INTERVAL UPDATES

        def update_plot(self, frame):
                """Update scatter plot with gps points."""
                # If there is new original gps data
                if self.original_gps.new:
                        # Update all original gps points (trail)
                        self.original_scatter.set_data(
                                self.original_gps.longitudes,
                                self.original_gps.latitudes
                        )
                        # If there are not offset gps points yet
                        if not self.currentOffsetGPS:
                                # Update the last point (current position)
                                self.current_position_scatter.set_data(
                                        self.original_gps.currentLon(), 
                                        self.original_gps.currentLat()
                                )
                        self.original_gps.update()
                # If there is new offset gps data
                if self.offset_gps.new:
                        # Update all offset (corrected) gps points (trail)
                        self.offset_scatter.set_data(
                                self.offset_gps.longitudes,
                                self.offset_gps.latitudes
                        )
                        # Update the last point (current position)
                        self.current_position_scatter.set_data(
                                self.offset_gps.currentLon(), 
                                self.offset_gps.currentLat()
                        )
                        self.offset_gps.update()
                # If odom data is available (for heading)
                if self.currentOdom:
                        # Determine which gps data to use to draw heading line
                        gps = self.original_gps if not self.currentOffsetGPS else self.offset_gps
                        # if there is gps data available yet
                        if len(gps) > 0:
                                # Re-draw heading line
                                self.drawHeadingLine(
                                        current_lat = gps.currentLat(),
                                        current_lon = gps.currentLon()
                                )
                # Update the current waypoint
                self.updateWaypoints()
                self.ax.relim()
                self.ax.autoscale_view()
                # Redraw the figure canvas with the latest changes
                self.fig.canvas.draw()
                # Process any pending GUI events
                self.fig.canvas.flush_events()
                return self.original_scatter,

        def process(self):
                # Wait until gps and odom data have been received
                if self.currentGPS is None or self.currentOdom is None:
                        self.get_logger().info("Waiting for odom and gps data to be initalized.")
                        return
                # Update distance from current gps to the current waypoint
                self.updateDistance()
                # (gps and odom data are available)
                self.get_logger().info(f'Lat: {self.currentGPS.latitude}\t Lon: {self.currentGPS.longitude}\t Distance: {round(self.currentDistance, 4)}\t Waypoint #{self.currentWaypointNumber}')


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