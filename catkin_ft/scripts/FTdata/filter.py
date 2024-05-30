#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped

def callback(data):
    # rospy.loginfo("Header:")
    # rospy.loginfo("  seq: %d", data.header.seq)
    # rospy.loginfo("  stamp: ")
    # rospy.loginfo("    secs: %d", data.header.stamp.secs)
    # rospy.loginfo("    nsecs: %d", data.header.stamp.nsecs)
    # rospy.loginfo("  frame_id: %s", data.header.frame_id)
    # rospy.loginfo("Wrench:")
    # rospy.loginfo("  force: ")
    # rospy.loginfo("    x: %f", data.wrench.force.x)
    # rospy.loginfo("    y: %f", data.wrench.force.y)
    # rospy.loginfo("    z: %f", data.wrench.force.z)
    # rospy.loginfo("  torque: ")
    # rospy.loginfo("    x: %f", data.wrench.torque.x)
    # rospy.loginfo("    y: %f", data.wrench.torque.y)
    # rospy.loginfo("    z: %f", data.wrench.torque.z)

    rospy.loginfo("Force z: %f", data.wrench.force.z)


def listener():
    rospy.init_node('rft_force_listener', anonymous=True)
    rospy.Subscriber("/RFT_FORCE", WrenchStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
