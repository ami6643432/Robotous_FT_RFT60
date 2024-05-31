#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped
from collections import deque

class RFTForceListener:
    def __init__(self):
        self.force_z_values = deque(maxlen=20)  # Stores the last 20 z force values
        self.current_z_force = 0.0
        rospy.init_node('rft_force_listener', anonymous=True)
        rospy.Subscriber("/RFT_FORCE", WrenchStamped, self.callback)
        
        # Set up a timer to print the stored array every 10 seconds
        rospy.Timer(rospy.Duration(10), self.print_stored_array)
        
        rospy.spin()

    def callback(self, data):
        # Update the current z force value
        self.current_z_force = data.wrench.force.z
        
        # Add the z force value to the deque
        self.force_z_values.append(self.current_z_force)
        
        # Print the current z force value
        rospy.loginfo("Force z: %f", self.current_z_force)
        
        # Print the list of the last 20 z force values
        rospy.loginfo("Last 20 z force values: %s", list(self.force_z_values))
    
    def print_stored_array(self, event=None):
        # Print the stored array
        rospy.loginfo("Stored z force values: %s", list(self.force_z_values))

if __name__ == '__main__':
    listener = RFTForceListener()
