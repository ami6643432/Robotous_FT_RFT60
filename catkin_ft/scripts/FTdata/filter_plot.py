#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from collections import deque
import numpy as np
from scipy.signal import firwin
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class RFTForceListener:
    def __init__(self):
        self.num_taps = 10
        self.cutoff_frequency = 0.2  # Normalized cutoff frequency
        self.coefficients = firwin(self.num_taps, self.cutoff_frequency)
        self.force_z_values = deque(maxlen=100)  # Stores the last 100 raw z force values
        self.filtered_force_values = deque(maxlen=100)  # Stores the last 100 filtered z force values
        self.current_z_force = 0.0
        rospy.init_node('rft_force_listener', anonymous=True)
        rospy.Subscriber("/RFT_FORCE", WrenchStamped, self.callback)
        
        # Start the plotting in a separate thread
        plot_thread = threading.Thread(target=self.start_plotting)
        plot_thread.daemon = True  # Allows the plot window to close automatically on script termination
        plot_thread.start()

        rospy.spin()

    def callback(self, data):
        # Update the current z force value
        self.current_z_force = data.wrench.force.z
        self.force_z_values.append(self.current_z_force)

        # Apply the FIR filter if enough data is collected
        if len(self.force_z_values) >= self.num_taps:
            filtered_force = np.dot(self.coefficients, list(self.force_z_values)[-self.num_taps:])
            self.filtered_force_values.append(filtered_force)

    def start_plotting(self):
        self.fig, self.ax = plt.subplots()
        self.raw_line, = self.ax.plot([], [], 'b-', label='Raw Force')
        self.filtered_line, = self.ax.plot([], [], 'r-', label='Filtered Force')
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(-100, 100)  # Set these limits based on expected force range
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Force (z)')
        self.ax.legend()
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        plt.show()

    def update_plot(self, frame):
        # Update raw force plot
        if len(self.force_z_values) > 0:
            self.raw_line.set_data(range(len(self.force_z_values)), list(self.force_z_values))
        
        # Update filtered force plot
        if len(self.filtered_force_values) > 0:
            self.filtered_line.set_data(range(len(self.filtered_force_values)), list(self.filtered_force_values))
        
        # Update axes limits dynamically
        current_length = max(len(self.force_z_values), len(self.filtered_force_values))
        self.ax.set_xlim(0, current_length)
        self.fig.canvas.draw()

        return self.raw_line, self.filtered_line

    def on_key_press(self, event):
        if event.key == 'escape':
            rospy.signal_shutdown('ESC pressed')
            plt.close(self.fig)  # Close the matplotlib figure

if __name__ == '__main__':
    listener = RFTForceListener()
