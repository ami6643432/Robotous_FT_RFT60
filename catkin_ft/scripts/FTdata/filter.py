#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from collections import deque
import numpy as np
from scipy.signal import firwin

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

        rospy.spin()

    def callback(self, data):
        # Update the current z force value
        self.current_z_force = data.wrench.force.z
        self.force_z_values.append(self.current_z_force)

        # Apply the FIR filter if enough data is collected
        if len(self.force_z_values) >= self.num_taps:
            filtered_force = np.dot(self.coefficients, list(self.force_z_values)[-self.num_taps:])
            self.filtered_force_values.append(filtered_force)

            # Log filtered force for verification
            rospy.loginfo("Filtered z force: %f", filtered_force)

if __name__ == '__main__':
    listener = RFTForceListener()
