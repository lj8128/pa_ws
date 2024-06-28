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
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.twist = Twist()
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

    def find_ang_vel(self, target_heading, base_vel):
        return base_vel * (target_heading - self.cur_yaw)

    #def move_to_pin(self, pin_id):
    #    rate = rospy.Rate(10)

    #    while not rospy.is_shutdown():
    #        try:
    #            tf_base_to_fid = self.tf_buffer.lookup_transform(
    #                'base_link',
    #                f'fiducial_{fid_id}',
    #                rospy.Time()).transform

    #            trans = tf_base_to_fid.translation

    #            self.twist.angular.z = -trans.y * 2

    #            self.cmd_vel_pub.publish(self.twist)
    #        except(tf2_ros.LookupException,
    #               tf2_ros.ExtrapolationException,
    #               tf2_ros.ConnectivityException):
    #               continue

    def turn_to_heading(self, target_yaw, base_vel):
        rate = rospy.Rate(10)
        while (
            not math.isclose(self.cur_yaw, target_yaw, abs_tol=0.008) and
            not rospy.is_shutdown()
            ):
               self.twist.angular.z = self.find_ang_vel(target_yaw, base_vel)
               self.cmd_vel_pub.publish(self.twist)
               rate.sleep()

    def scan_for_fids(self):
        target_yaw = math.pi * 2 * (9 / 10)
        self.turn_to_heading(target_yaw, 0.5)

    def match_pin_rotation(self, pin_id):
        pin_tf = self.tf_buffer.lookup_transform(
            f'pin_{pin_id}',
            f'pin_{pin_id}',
            rospy.Time()
        ).transform
        pin_quat = [
            pin_tf.rotation.x,
            pin_tf.rotation.y,
            pin_tf.rotation.z,
            pin_tf.rotation.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(pin_quat)
        self.turn_to_heading(yaw, 0.2) 

    def face_pin(self, pin_id):
        base_to_pin_transl = self.tf_buffer.lookup_transform(
            f'base_link',
            f'pin_{pin_id}',
            rospy.Time()
        ).transform.translation
        yaw = math.pi / 2 - math.atan(
            base_to_pin_transl.x / base_to_pin_transl.y
            )
        self.turn_to_heading(yaw, 0.2)

    def print_transl(self, pin_id):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                base_to_pin_transl = self.tf_buffer.lookup_transform(
                    'base_link',
                    f'pin_{pin_id}',
                    rospy.Time()
                    ).transform.translation
                rospy.loginfo(f'base_to_pin_{pin_id}_transl.x: {base_to_pin_transl.x}')
                rospy.loginfo(f'base_to_pin_{pin_id}_transl.y: {base_to_pin_transl.x}')
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ):
                rospy.loginfo("tf2_ros exception triggered!")
                continue
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('nav_sim')

    nav = NavSim()
    nav.scan_for_fids()
    nav.match_pin_rotation(0)
    nav.face_pin(0)
    # nav.print_transl(0)
    # nav.move_to_pin(0)