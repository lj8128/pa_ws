#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import Twist

class PaFourSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def move_to_fiducial(self, fid_id):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                tf_base_to_fid = self.tf_buffer.lookup_transform(
                    'base_link',
                    f'fiducial_{fid_id}',
                    rospy.Time()).transform

                trans = tf_base_to_fid.translation

                self.twist.angular.z = -trans.y * 2

                self.cmd_vel_pub.publish(self.twist)
            except(tf2_ros.LookupException,
                   tf2_ros.ExtrapolationException,
                   tf2_ros.ConnectivityException):
                   continue

    def rotate_in_place(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            self.twist.angular.z = 0.1
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pa4_sim')
    PaFourSim().move_to_fiducial(1)