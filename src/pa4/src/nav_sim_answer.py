#!/usr/bin/env python3

import math
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

class NavSim:
    def __init__(self):
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.twist = Twist()
        self.total_dist = 0.0
        self.cur_yaw = 0.0

        # The value D such that, when the distance between the robot's base_link
        # frame's origin and the origin of the pin it's travelling towards
        # equals D meters, the robot should stop.
        self.HALT_DIST = 0.35
        
    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        cur_dist = msg.x
        cur_yaw = msg.y
        self.total_dist += cur_dist 
        self.cur_yaw = cur_yaw

    def find_ang_vel(self, target_heading, base_vel):
        """Helper to `turn_to_heading`."""
        return base_vel * (target_heading - self.cur_yaw)

    def turn_to_heading(self, target_yaw, base_vel):
        """
        Turns the robot to heading `target_yaw` with a base velocity of
        `base_vel`.
        """
        rate = rospy.Rate(10)
        while (
            not rospy.is_shutdown() and
            not math.isclose(self.cur_yaw, target_yaw, abs_tol=0.05)
            ):
               self.twist.angular.z = self.find_ang_vel(target_yaw, base_vel)
               self.cmd_vel_pub.publish(self.twist)
               rate.sleep()
        self.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def scan_for_fids(self):
        """
        Scans for fiducials by rotating in place. Note that the `mapper` node 
        does the actual mapping.
        """
        rospy.loginfo(f'---> Scanning for fiducials...')
        target_yaw = math.pi * 2 * (9 / 10)
        self.turn_to_heading(target_yaw, 0.5)

    def match_pin_rotation(self, pin_id):
        """
        Rotates the robot so that its `base_link` frame's orientation matches
        that of the target pin's frame.
        """
        rospy.loginfo(f'---> Matching orientation of pin_{pin_id}...') 
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
        """
        Rotates the robot so that it faces the target pin.
        """
        rospy.loginfo(f'---> Facing pin_{pin_id}...') 
        base_to_pin_transl = self.tf_buffer.lookup_transform(
            f'base_link',
            f'pin_{pin_id}',
            rospy.Time()
        ).transform.translation

        x_tr = base_to_pin_transl.x
        y_tr = base_to_pin_transl.y
        yaw = 0.0

        x_tr_abs = abs(x_tr)
        y_tr_abs = abs(y_tr)

        if not y_tr == 0:
            if x_tr > 0 and y_tr > 0:
                rospy.loginfo('x_tr: + // y_tr: +') 
                yaw = math.atan(y_tr_abs / x_tr_abs)
            elif x_tr > 0 and y_tr < 0:
                rospy.loginfo('x_tr: + // y_tr: -') 
                yaw = 2 * math.pi - math.atan(y_tr_abs / x_tr_abs)
            elif x_tr < 0 and y_tr < 0:
                rospy.loginfo('x_tr: - // y_tr: -') 
                yaw = math.pi + math.atan(y_tr_abs / x_tr_abs)
            elif x_tr < 0 and y_tr > 0:
                rospy.loginfo('x_tr: - // y_tr: +') 
                yaw = math.pi - math.atan(y_tr_abs / x_tr_abs)
            elif y_tr > 0:
                rospy.loginfo("x_tr: 0 // y_tr: +")
                yaw = math.pi / 2
            else:
                rospy.loginfo("x_tr: 0 // y_tr: -")
                yaw = 2 * math.pi - (math.pi / 2)
        elif x_tr < 0:
            rospy.loginfo("x_tr: - // y_tr: 0")
            yaw = math.pi

        rospy.loginfo(f'yaw: {yaw}')
        self.turn_to_heading(yaw, 0.2)

    def get_dist_base_to_pin(self, pin_id):
        """
        Returns the distance between the origin of the `base_link` frame and that
        of the target pin's frame.
        """
        base_to_pin_transl = self.tf_buffer.lookup_transform(
            f'base_link',
            f'pin_{pin_id}',
            rospy.Time()
        ).transform.translation

        return np.linalg.norm(
            np.array(
                [
                    base_to_pin_transl.x,
                    base_to_pin_transl.y,
                    base_to_pin_transl.z,
                ]
            )
        )

    def move_to_pin(self, pin_id):
        """
        Moves the robot to the target pin.
        """
        rospy.loginfo(f'---> Moving to pin_{pin_id}...')
        # Reset distance traveled
        self.total_dist = 0.0
        target_dist = self.get_dist_base_to_pin(pin_id) - self.HALT_DIST

        rate = rospy.Rate(10)

        while (
            not rospy.is_shutdown() and 
            not math.isclose(self.total_dist, target_dist, abs_tol=0.05)
        ):
            self.twist.linear.x = 0.05
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)

    def print_transl(self, pin_id):
        """
        For debugging. Prints the x and y translations from the `base_link` frame
        to the target pin's frame.
        """
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            try:
                base_to_pin_transl = self.tf_buffer.lookup_transform(
                    'base_link',
                    f'pin_{pin_id}',
                    rospy.Time()
                    ).transform.translation
                rospy.loginfo(f'base_to_pin_{pin_id}_transl.x: {base_to_pin_transl.x}')
                rospy.loginfo(f'base_to_pin_{pin_id}_transl.y: {base_to_pin_transl.y}')
            except (
                tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException
                ):
                continue
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('nav_sim')

    nav = NavSim()
    nav.scan_for_fids()
    target_pin_ids = [0, 1, 2, 3]
    for pin_id in target_pin_ids:
        nav.match_pin_rotation(pin_id)
        nav.face_pin(pin_id)
        nav.move_to_pin(pin_id)