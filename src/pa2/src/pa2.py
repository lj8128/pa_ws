#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PaTwo:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        """
        raise NotImplementedError

    def follow_wall(self):
        """
        Main function that publishes movement commands to the robot based on its
        state.
        """
        raise NotImplementedError

if __name__ == '__main__':
    rospy.init_node('pa2')
    PaTwo().follow_wall()
