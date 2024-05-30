#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PaOne:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_cb)
        self.old_pose = None
        self.traveled_dist = 0.0
        self.cur_yaw = None

    def _odom_cb(self, msg):
        cur_pose = msg.pose.pose
        
        if self.old_pose is not None:
            self.traveled_dist += self._cur_dist(cur_pose.position)
        else:
            self.old_pose = Pose()

        self.cur_yaw = self._find_yaw(cur_pose.orientation)
        self._update_pose(cur_pose)

    def _find_yaw(self, cur_orientation):
        orientations = [cur_orientation.x,
                cur_orientation.y,
                cur_orientation.z,
                cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientations)
        return yaw

    def _find_ang_vel(self, base_vel, target_heading):
        return base_vel * (target_heading - self.cur_yaw)

    def _cur_dist(self, cur_position):
        x_diff = cur_position.x - self.old_pose.position.x
        y_diff = cur_position.y - self.old_pose.position.y
        return math.sqrt(x_diff ** 2 + y_diff ** 2)

    def _update_pose(self, cur_pose):
        self.old_pose.position.x = cur_pose.position.x
        self.old_pose.position.y = cur_pose.position.y
        self.old_pose.position.z = cur_pose.position.z
        self.old_pose.orientation.x = cur_pose.orientation.x
        self.old_pose.orientation.y = cur_pose.orientation.y
        self.old_pose.orientation.z = cur_pose.orientation.z
        self.old_pose.orientation.w = cur_pose.orientation.w

    def out_and_back(self, target_dist, target_heading):
        dest_end_reached = False
        dest_start_reached = False
        turned_back = False
        done = False
        end_target_dist = target_dist * 2.2

        twist = Twist()

        rate = rospy.Rate(10)
        
        while not (done or rospy.is_shutdown()):
            if not dest_start_reached:
                twist.linear.x = 0.1
                if math.isclose(self.traveled_dist, target_dist, rel_tol=0.01):
                    twist.linear.x = 0
                    dest_start_reached = True
            elif dest_start_reached and not turned_back:
                twist.angular.z = self._find_ang_vel(0.5, target_heading) 
                if math.isclose(twist.angular.z, 0.0, abs_tol=0.01):
                    twist.angular.z = 0.0
                    turned_back = True
            elif not dest_end_reached:
                twist.linear.x = 0.1
                if math.isclose(self.traveled_dist, end_target_dist, rel_tol=0.01):
                    twist.linear.x = 0
                    dest_end_reached = True

            self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('pa1')
    PaOne().out_and_back(0.3, math.pi)

