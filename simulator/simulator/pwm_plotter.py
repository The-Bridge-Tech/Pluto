"""
Dynamically Plot PWM (Pulse-Width-Modulation) control values in real-time
Author: Matthew Lauriault
Created: 8/2/24
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


class PWMData:

        """Struct class for pwm plot data"""

        def __init__(self):
                self.clear()
                self.update()

        def append(self, value: int):
                self.times.append(time.time())
                self.values.append(value)
                self.new = True

        def update(self):
                self.new = False

        def clear(self):
                self.values = []
                self.times = []


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
                self.fig, (self.left_ax, self.right_ax) = plt.subplots(2, 1, figsize=(10, 8))
                # Create titles and labels for left_pwm plot
                self.left_ax.set_title('Left PWM')
                self.left_ax.set_xlabel('Time (s)')
                self.left_ax.set_ylabel('PWM (%)')
                # Create titles and labels for right_pwm plot
                self.right_ax.set_title('Right PWM')
                self.right_ax.set_xlabel('Time (s)')
                self.right_ax.set_ylabel('PWM (%)')
                # Create plot for left_pwm
                self.left_pwm_plot = self.left_ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='orange', 
                        marker='o',
                        markersize=2
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Create plot for right_pwm
                self.right_pwm_plot = self.right_ax.plot(
                        [], # initially empty
                        [], # initially empty
                        color='purple', 
                        marker='o',
                        markersize=2
                )[0] # get the first and only item in the list returned by Axes.plot()
                # Allow the plots to be dynamic
                self.animation = animation.FuncAnimation(
                        fig = self.fig, 
                        func = self.update_plot, 
                        interval = (1 / PROCESS_RATE) * 1000 # ms
                )

                # VARIABLES
                self.left_pwm_data = PWMData()
                self.right_pwm_data = PWMData()

        # TIMER CALLBACKS

        def update_plot(self, frame):
                """Update left and right pwm plots"""
                # if there is new left pwm data
                if self.left_pwm_data.new:
                        # update left pwm plot
                        self.left_pwm_plot.set_data(
                                self.left_pwm_data.times, 
                                self.left_pwm_data.values
                        )
                        self.left_pwm_data.update()
                # if there is new right pwm data
                if self.right_pwm_data.new:
                        # update right pwm plot
                        self.right_pwm_plot.set_data(
                                self.right_pwm_data.times, 
                                self.right_pwm_data.values
                        )
                        self.right_pwm_data.update()
                # update the plot limits and scale
                self.left_ax.relim()
                self.left_ax.autoscale_view()
                self.right_ax.relim()
                self.right_ax.autoscale_view()
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