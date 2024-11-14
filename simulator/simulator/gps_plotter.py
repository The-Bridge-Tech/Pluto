"""
Show a realtime map/plot of the current gps location and waypoints
Author: Matthew Lauriault
Created: 8/2/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geodesy import utm
from nav_msgs.msg import Odometry
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
from .gps_list import GPSList
from customize_local_planner.phase_one_demo import WAYPOINTS
from customize_local_planner.conversions import *


# CONSTANTS
FENCE_GPS_POINTS = [
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
        "simulator", 
        "simulator"
)
MAP_IMAGE_DIR = os.path.join(
        PARENT_DIR, 
        "map3.png"
)

# PARAMETERS
PROCESS_RATE = 10 # Hz (times / second)
HEADING_LINE_LENGTH = 0.000025 * 2
WAYPOINT_RADIUS = 1.0 # should match local_planner's 'straight_distance_tolerance' parameter


class GPSPlotter(Node):

        def __init__(self):
                super().__init__('gps_plotter')

                # PARAMETERS
                # YAML File: pluto_launch/config/location.yaml
                self.base_lat = self.load_param_double("base_lat")
                self.base_lon = self.load_param_double("base_lon")

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
                # Subscribe to local_planner's position
                self.local_position_sub = self.create_subscription(
                        utm.GeoPoint, 
                        "/analysis/position", 
                        self.local_position_callback, 
                        10
                )
                self.currentLocalGPS = None
                # For storing local_planner's GPS coordinates
                self.local_gps = GPSList()
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
                # Create plot for local_planner's gps points (trail)
                self.local_scatter = self.ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='blue', 
                        marker='o',
                        markersize=2,
                        label='Local Trail'
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
                        x = [self.base_lon],   # longitude
                        y = [self.base_lat],   # latitude
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
                # Add radius circle around base pin
                base_radius_circle = patches.Circle(
                        (self.base_lon, self.base_lat),  # (x=longitude, y=latitude)
                        meters_to_gps_degrees(WAYPOINT_RADIUS, self.base_lat),  # Convert meter radius to degrees
                        edgecolor='white', 
                        facecolor='none', 
                        # linestyle='--',
                )
                self.ax.add_patch(base_radius_circle)
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
                self.new_local_data = False
                # Finish plot
                plt.legend()
                plt.xlabel('Longitude')
                plt.ylabel('Latitude')
                plt.title('Dynamic GPS Plotter')

        
        # HELPERS - PARAMETERS

        def load_param(self, param_name: str, init_value):
                self.declare_parameter(param_name, init_value)
                return self.get_parameter(param_name).get_parameter_value()
        
        def load_param_double(self, param_name: str) -> float:
                return self.load_param(param_name, 0.0).double_value
        

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
                return angle_from_odom(self.currentOdom)
        
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
                currentGPS = self.currentGPS if self.currentLocalGPS is None else self.currentLocalGPS
                self.currentDistance = haversine(
                        lat1 = currentGPS.latitude,
                        lon1 = currentGPS.longitude,
                        lat2 = currentWaypoint[0],
                        lon2 = currentWaypoint[1]
                )


        # SUBSCRIBER CALLBACKS

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

        def local_position_callback(self, msg: utm.GeoPoint):
                """Update current GPS point from local_planner"""
                # if new gps point is not an outlier
                if not self.isOutlier(msg.latitude, msg.longitude):
                        # update gps list for plot
                        self.local_gps.append(
                                lat = msg.latitude,
                                lon = msg.longitude
                        )
                self.currentLocalGPS = msg

        def odom_callback(self, msg: Odometry):
                """Update current odometry."""
                self.currentOdom = msg

        def waypoint_ping_callback(self, msg: WaypointMsg):
                """Update current waypoint number."""
                if msg.waypoint_number > self.currentWaypointNumber:
                        self.currentWaypointNumber = msg.waypoint_number


        # TIMER CALLBACKS

        def update_plot(self, frame):
                """Update scatter plot with gps points."""
                # If there is new original gps data
                if self.original_gps.new:
                        # Update all original gps points (trail)
                        self.original_scatter.set_data(
                                self.original_gps.longitudes,
                                self.original_gps.latitudes
                        )
                        # If there are not local gps points yet
                        if not self.currentLocalGPS:
                                # Update the last point (current position)
                                self.current_position_scatter.set_data(
                                        self.original_gps.currentLon(), 
                                        self.original_gps.currentLat()
                                )
                        self.original_gps.update()
                # If there is new local gps data
                if self.local_gps.new:
                        # Update all local gps points (trail)
                        self.local_scatter.set_data(
                                self.local_gps.longitudes,
                                self.local_gps.latitudes
                        )
                        # Update the last point (current position)
                        self.current_position_scatter.set_data(
                                self.local_gps.currentLon(), 
                                self.local_gps.currentLat()
                        )
                        self.local_gps.update()
                # If odom data is available (for heading)
                if self.currentOdom:
                        # Determine which gps data to use to draw heading line
                        gps = self.original_gps if not self.currentLocalGPS else self.local_gps
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