#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class PaFourSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
    
    def rotate_in_place(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.twist.angular.z = 0.1
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pa4_sim')
    PaFourSim().rotate_in_place()