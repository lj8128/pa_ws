#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

class PaOne:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.is_turning = False
        self.total_dist = 0.0
        self.cur_yaw = None

    def my_odom_cb(self, msg):
        """Callback function for `my_odom_sub`."""
        cur_dist = msg.x
        cur_yaw = msg.y
        self.total_dist += cur_dist 
        self.cur_yaw = cur_yaw

    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        dest_back_reached = False
        dest_out_reached = False
        turned_back = False
        end_target_dist = target_dist * 2

        twist = Twist()

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if not dest_out_reached:
                twist.linear.x = 0.1
                if math.isclose(self.total_dist, target_dist, rel_tol=0.01):
                    twist.linear.x = 0
                    dest_out_reached = True
            elif not turned_back:
                self.is_turning = True
                twist.angular.z = self.find_ang_vel(0.5, math.pi) 
                if math.isclose(twist.angular.z, 0.0, abs_tol=0.01):
                    twist.angular.z = 0.0
                    turned_back = True
                    self.is_turning = False
            elif not dest_back_reached:
                twist.linear.x = 0.1
                if math.isclose(self.total_dist, end_target_dist, rel_tol=0.01):
                    twist.linear.x = 0
                    dest_back_reached = True

            self.cmd_vel_pub.publish(twist)
            rate.sleep()


    def draw_square(self, side_length):
        """Moves the robot in a square with sides of `side_length`"""
        side_count = 1
        side_finished = False
        corner_turned = False
        
        twist = Twist()
        rate = rospy.Rate(10)

        while side_count < 5 and not rospy.is_shutdown():
            if not side_finished:
                twist.linear.x = 0.1
                if math.isclose(self.total_dist, side_length, abs_tol = 0.008):
                    side_finished = True
                    twist.linear.x = 0
                    self.total_dist = 0
            elif not corner_turned:
                self.is_turning = True
                cur_target_angle = math.pi / 2 * side_count
                twist.angular.z = self.find_ang_vel(0.5, cur_target_angle) 
                if math.isclose(twist.angular.z, 0.0, abs_tol=0.008):
                    twist.angular.z = 0
                    corner_turned = True
                    self.is_turning = False
            else:
                side_finished = False
                corner_turned = False
                side_count += 1

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def find_ang_vel(self, base_vel, target_heading):
        """Helper to `out_and_back` and `draw_square`."""
        return base_vel * (target_heading - self.cur_yaw)

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        linear_velocity = 0.3
        radius = r
        angular_velocity = linear_velocity / radius

        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pa1')
    # PaOne().out_and_back(1)
    # PaOne().draw_square(1)
    PaOne().move_in_a_circle(1)

