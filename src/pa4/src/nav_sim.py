#!/usr/bin/env python3

import math
import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion

class NavSim:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.twist = Twist()
        self.traveled_dist = 0.0
        self.old_pose = None
        self.cur_yaw = 0.0

        # The value D such that, when the distance between the robot's base_link
        # frame's origin and the origin of the pin it's travelling towards
        # equals D meters, the robot should stop.
        self.HALT_DIST = 0.2 
        
    def odom_cb(self, msg):
        """Callback function for `odom_sub`."""
        cur_pose = msg.pose.pose
       
        # Update the distance traveled by the robot each time odom_cb is called,
        # by adding the distance between old_pose and cur_pose. 
        if self.old_pose is not None:
            self.traveled_dist += self.cur_dist(cur_pose.position)

        self.old_pose = cur_pose
        self.update_yaw(cur_pose.orientation)

    def cur_dist(self, cur_position):
        """
        Helper to `odom_cb`.
        Calculates the distance between `old_pose` and `cur_pose`.
        """
        x_diff = cur_position.x - self.old_pose.position.x
        y_diff = cur_position.y - self.old_pose.position.y
        return math.sqrt(x_diff ** 2 + y_diff ** 2)

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

    def turn_to_heading(self, target_yaw, base_vel):
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

    def get_dist_base_to_pin(self, pin_id):
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
        # Reset distance traveled
        self.traveled_dist = 0.0
        #-----------REFACTOR--------------------
        target_dist = self.get_dist_base_to_pin(pin_id)
        rospy.loginfo(f'raw_target_dist= {target_dist}')
        target_dist -= self.HALT_DIST
        rospy.loginfo(f'target_dist = {target_dist}')
        #-------------------------------

        rate = rospy.Rate(10)

        while (
            not rospy.is_shutdown() and 
            not math.isclose(self.traveled_dist, target_dist, abs_tol=0.05)
        ):
            rospy.loginfo(f'traveled_dist = {self.traveled_dist}')
            self.twist.linear.x = 0.05
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()
        self.twist.linear.x = 0.0
        self.cmd_vel_pub.publish(self.twist)

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
    nav.move_to_pin(0)
    # nav.print_transl(0)