#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

class PaOne:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        raise NotImplementedError
        
    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        raise NotImplementedError

    def draw_square(self, side_length):
        """Moves the robot in a square with sides of `side_length`"""
        raise NotImplementedError

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        raise NotImplementedError
        
if __name__ == '__main__':
    rospy.init_node('pa1')
    PaOne().out_and_back(0.5)
    # PaOne().draw_square(0.3)
    # PaOne().move_in_a_circle(0.3)

