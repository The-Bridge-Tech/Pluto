"""
Show a realtime map/plot of the current odometry location
Author: Matthew Lauriault
Created: 8/23/24
"""


# ROS2 MODULES
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
# CALCULATION MODULES
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import math

# HELPER MODULES
from .conversions import angle_from_odometry


HEADING_LINE_LENGTH = 0.1


class OdomPlotter(Node):

        def __init__(self):
                super().__init__('odom_plotter')

                # SUBSCRIBERS
                # Subscribe to odometry topic
                self.odom_subscriber = self.create_subscription(
                        Odometry, 
                        "odometry/global", 
                        self.odom_callback, 
                        10
                )
                self.currentOdom = None
                # For storing odom coordinates
                self.odomXs = []
                self.odomYs = []

                # TIMERS
                # Timer to process current data (don't want to process each time data is received)
                process_timer_period = 0.1 # seconds
                self.process_timer = self.create_timer(
                        process_timer_period, 
                        self.process
                )

                # PLOT
                # Turn on interactive mode
                plt.ion()
                # Create figure and axes
                self.fig, self.ax = plt.subplots()
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
                self.heading_line = None
                # Allow the plot to be dynamic
                self.animation = animation.FuncAnimation(
                        fig = self.fig, 
                        func = self.update_plot, 
                        interval=process_timer_period*1000 # ms
                )
                # Ensure that x and y scales are always equal
                self.ax.set_aspect('equal', adjustable='datalim')
                # Finish plot
                plt.legend()
                plt.xlabel('x')
                plt.ylabel('y')
                plt.title('Dynamic Odometry Plotter')
        

        # HELPERS

        def getCurrentHeading(self) -> float:
                return angle_from_odometry(self.currentOdom)
        
        def drawHeadingLine(self):
                # Remove previous heading line
                if self.heading_line:
                        self.heading_line.remove()
                # Get start point
                start_x = self.odomXs[-1] # current x
                start_y = self.odomYs[-1] # current y
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

        def odom_callback(self, odom_msg: Odometry):
                """Update current odometry."""
                self.currentOdom = odom_msg


        # INTERVAL UPDATES

        def update_plot(self, frame):
                """Update scatter plot with odom xy points."""
                # If there are odom points to plot
                if self.odomXs and self.odomYs:
                        # Update all points (trail)
                        self.scatter.set_data(
                                self.odomXs,
                                self.odomYs
                        )
                        # Update the last point (current position)
                        self.current_position_scatter.set_data(
                                self.odomXs[-1:], 
                                self.odomYs[-1:]
                        )
                        # Re-draw heading line
                        self.drawHeadingLine()
                # Adjust x and y limits to see the data
                self.ax.relim()
                self.ax.autoscale_view()
                # Redraw the figure canvas with the latest changes
                self.fig.canvas.draw()
                # Process any pending GUI events
                self.fig.canvas.flush_events()
                return self.scatter,

        def process(self):
                """Process on an interval (timer) rather than every time data is received (when the subscriber callbacks are called)."""
                # Wait until odom data has been received
                if self.currentOdom is None:
                        self.get_logger().info("Waiting for odom data to be initalized.")
                        return
                # (gps and odom data are available)
                # get current x and y from odom data
                x = self.currentOdom.pose.pose.position.x
                y = self.currentOdom.pose.pose.position.y
                # add current x and y to lists for plot
                self.odomXs.append(x)
                self.odomYs.append(y)
                # log current x, y, and heading
                self.get_logger().info(f'({x}, {y}) at {self.getCurrentHeading()}')


# MAIN

def main(args=None):
        rclpy.init(args=args)
        odom_plotter = OdomPlotter()

        # spin the node in a separate thread since plt.show() blocks and is not thread-safe
        node_spin_thread = Thread(
                target=rclpy.spin, 
                args=(odom_plotter,)
        )
        node_spin_thread.start()

        # Show the plot and keep it updated in the main thread
        plt.show(block=True)

        odom_plotter.destroy_node()
        rclpy.shutdown()
        node_spin_thread.join()


# When this file is run as a script
if __name__ == '__main__':
        main()