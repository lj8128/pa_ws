#!/usr/bin/env python3

import math
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class NavSim:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.cur_yaw = 0.0

    def odom_cb(self, msg):
        self.update_yaw(msg.pose.pose.orientation)

    def update_yaw(self, cur_orientation):
        orientations = [cur_orientation.x,
                cur_orientation.y,
                cur_orientation.z,
                cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        if yaw < 0:
            yaw = 2 * math.pi + yaw
        self.cur_yaw = yaw

    def find_ang_vel(self, base_vel, target_heading):
        return base_vel * (target_heading - self.cur_yaw)

    def move_to_pin(self, pin_id):
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

    def scan_for_fids(self):
        rate = rospy.Rate(10)
        
        target_yaw = math.pi * 2 * (9 / 10)

        while (not math.isclose(self.cur_yaw, target_yaw, abs_tol=0.008) and
               not rospy.is_shutdown()):
               self.twist.angular.z = self.find_ang_vel(0.5, target_yaw)
               self.cmd_vel_pub.publish(self.twist)
               print(self.cur_yaw)
               rate.sleep()

if __name__ == '__main__':
    rospy.init_node('nav_sim')

    nav = NavSim()
    nav.scan_for_fids()
    nav.move_to_pin(0)