"""
Dynamically Plot Servo PWM (Pulse-Width-Modulation) control values in real-time
Author: Matthew Lauriault
Created: 10/11/24
"""

# ROS2 MODULES
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# CALCULATION MODULES
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import time

# PARAMETERS
PROCESS_RATE = 10 # Hz (times / second)
PLOT_X_LIM = 60 # seconds
APPEND_CURRENT_VALUE_TIMEOUT = 2 # seconds


class PWMData:

        """Struct class for pwm plot data"""

        def __init__(self, initial_time: float = 0.0):
                self.clear()
                self.update()
                self.initial_time = initial_time

        def append(self, value: int | float):
                self.times.append(self.get_relative_time())
                self.values.append(float(value))
                self.new = True

        def append_current_value(self):
                # if there is no new data and 2 seconds have passed since last update
                if not self.new and self.get_seconds_since_last_update() > APPEND_CURRENT_VALUE_TIMEOUT:
                        self.append(self.values[-1])

        def update(self):
                self.new = False

        def clear(self):
                self.values = []
                self.times = []

        def get_relative_time(self) -> float:
                return time.time() - self.initial_time
        
        def get_seconds_since_last_update(self) -> float:
                return self.get_relative_time() - self.times[-1]


        def __len__(self) -> int:
                return len(self.values)


class PWMPlotter(Node):

        def __init__(self):
                super().__init__('pwm_plotter')

                # SUBSCRIBERS - INPUTS (CONTROL VARIABLES)
                self.left_pwm_sub = self.create_subscription(
                        Float64,
                        "/steering_left/percentage",
                        self.left_pwm_callback,
                        10
                )
                self.right_pwm_sub = self.create_subscription(
                        Float64,
                        "/steering_right/percentage",
                        self.right_pwm_callback,
                        10
                )

                # PLOT
                # Turn on interactive mode
                plt.ion()
                # Create figure and axes
                self.left_ax: plt.Axes = None
                self.right_ax: plt.Axes = None
                self.fig, (self.left_ax, self.right_ax) = plt.subplots(2, 1, figsize=(5, 9))
                # Configure left_pwm plot
                self.left_ax.set_title('Left PWM')
                self.left_ax.set_xlabel('Time (s)')
                self.left_ax.set_ylabel('PWM (%)')
                self.left_ax.set_xlim(0, PLOT_X_LIM)
                self.left_ax.set_ylim(-100, 100)
                self.left_ax.grid(True)
                self.left_ax.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
                # Configure right_pwm plot
                self.right_ax.set_title('Right PWM')
                self.right_ax.set_xlabel('Time (s)')
                self.right_ax.set_ylabel('PWM (%)')
                self.right_ax.set_xlim(0, PLOT_X_LIM)
                self.right_ax.set_ylim(-100, 100)
                self.right_ax.grid(True)
                self.right_ax.axhline(y=0, color='black', linestyle='-', linewidth=1.5)
                # Create plot for left_pwm
                self.left_pwm_plot = self.left_ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='orange', 
                        marker='o',
                        markersize=1
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Create plot for right_pwm
                self.right_pwm_plot = self.right_ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='purple', 
                        marker='o',
                        markersize=1
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Allow the plots to be dynamic
                self.animation = animation.FuncAnimation(
                        fig = self.fig, 
                        func = self.update_plot, 
                        interval = (1 / PROCESS_RATE) * 1000, # milliseconds - delay between frames
                        blit = True, # only redraw elements that have changed
                )

                # VARIABLES
                self.initial_time = time.time()
                self.left_pwm_data = PWMData(self.initial_time)
                self.right_pwm_data = PWMData(self.initial_time)

        # HELPERS

        def get_relative_time(self) -> float:
                return time.time() - self.initial_time

        # TIMER CALLBACKS

        def update_plot(self, frame):
                """Update left and right pwm plots"""
                # if there is left pwm data
                if len(self.left_pwm_data) > 0:
                        # add current left pwm value as needed
                        self.left_pwm_data.append_current_value()
                        # if there is new data to update
                        if self.left_pwm_data.new:
                                # update left pwm plot
                                self.left_pwm_plot.set_data(
                                        self.left_pwm_data.times, 
                                        self.left_pwm_data.values
                                )
                                # update left pwm data
                                self.left_pwm_data.update()
                                self.get_logger().info(f"time: {self.left_pwm_data.times[-1]}\t left pwm: {self.left_pwm_data.values[-1]}")
                # if there is right pwm data
                if len(self.right_pwm_data) > 0:
                        # add current right pwm value as needed
                        self.right_pwm_data.append_current_value()
                        # if there is new data to update
                        if self.right_pwm_data.new:
                                # update right pwm plot
                                self.right_pwm_plot.set_data(
                                        self.right_pwm_data.times, 
                                        self.right_pwm_data.values
                                )
                                # update right pwm data
                                self.right_pwm_data.update()
                                self.get_logger().info(f"time: {self.right_pwm_data.times[-1]}\t right pwm: {self.right_pwm_data.values[-1]}")
                # scroll the x-axis to the right as new data comes in
                if self.get_relative_time() > PLOT_X_LIM:
                        self.left_ax.set_xlim(
                                self.get_relative_time() - PLOT_X_LIM, 
                                self.get_relative_time()
                        )
                        self.right_ax.set_xlim(
                                self.get_relative_time() - PLOT_X_LIM, 
                                self.get_relative_time()
                        )
                # Redraw the figure canvas with the latest changes
                self.fig.canvas.draw()
                # Process any pending GUI events
                self.fig.canvas.flush_events()
                return self.left_pwm_plot, self.right_pwm_plot
        
        # SUBSCRIBER CALLBACKS

        def left_pwm_callback(self, msg: Float64):
                self.left_pwm_data.append(msg.data)

        def right_pwm_callback(self, msg: Float64):
                self.right_pwm_data.append(msg.data)

# MAIN

def main(args=None):
        rclpy.init(args=args)
        pwm_plotter = PWMPlotter()

        # spin the node in a separate thread since plt.show() blocks and is not thread-safe
        node_spin_thread = Thread(
                target=rclpy.spin, 
                args=(pwm_plotter,)
        )
        node_spin_thread.start()

        # Show the plot and keep it updated in the main thread
        plt.show(block=True)

        pwm_plotter.destroy_node()
        rclpy.shutdown()
        node_spin_thread.join()


# When this file is run as a script
if __name__ == '__main__':
        main()