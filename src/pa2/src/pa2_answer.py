#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PaTwo:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.init_zones()
        self.init_states()

    def init_zones(self):
        """
        Initialize zone variables for the robot. Cf. `scan_cb` function below.
        """
        self.zone_0 = 0.0
        self.zone_1 = 0.0
        self.zone_2 = 0.0

    def init_states(self):
        """Initialize a map for the states of the robot.""" 
        self.states = {
                'no_walls': False,
                'wall_at_front': False,
                'wall_at_right_side': False,
                'wall_at_right_corner': False
                }

    def scan_cb(self, msg):
        """
        Callback function for `self.scan_sub`.
        Updates each zone variable to contain the minimum distance detected in
        each lidar zone.
        """
        self.zone_0 = min(msg.ranges[0:21] + msg.ranges[340:360])
        self.zone_1 = min(msg.ranges[260:340])
        self.zone_2 = min(msg.ranges[180:260])
        self.update_states()

    def update_states(self):
        """
        Updates the state the robot is in.
        """
        if self.wall_detected(self.zone_0):
            self.set_state('wall_at_front')
        elif (self.wall_detected(self.zone_1) and
                self.wall_detected(self.zone_2) and
                self.zone_1 > self.zone_2):
            self.set_state('wall_at_right_corner')
        elif self.wall_detected(self.zone_1):
            self.set_state('wall_at_right_side')
        elif self.wall_detected(self.zone_2):
            self.set_state('wall_at_right_corner')
        else:
            self.set_state('no_walls')

    def wall_detected(self, zone):
        """
        If the minimum distance in a lidar zone is `inf`, it means no walls have
        been detected in that zone.
        """
        return not math.isinf(zone)

    def set_state(self, state):
        """
        Set the argument state to `True` and the others to `False`.
        """
        for cur_state in self.states.keys():
            self.states[cur_state] = True if cur_state == state else False 

    def follow_wall(self):
        """
        Main function that publishes movement commands to the robot based on its
        state.
        """
        rate = rospy.Rate(10)
        twist = Twist()
        loop_counter = 0

        while not rospy.is_shutdown():
            if self.states['no_walls']:
                twist.linear.x = 0.2
            elif self.states['wall_at_front']:
                twist.angular.z = 0.2
            elif self.states['wall_at_right_corner']:
                twist.angular.z = -0.2
            elif self.states['wall_at_right_side']:
                twist.linear.x = 0.2
                twist.angular.z = self.pid(loop_counter)
            loop_counter += 1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def pid(self, loop_counter):
        """
        Helper function to follow_wall, used to maintain a distance from the wall
        with PID when robot is in the 'wall_at_right_side' state.
        """
         # desired setpoint (e.g. desired distance from wall)
        SP = 0.7
        # tuning constant for proportional control
        KP = 0.2
        # tuning constant for integral control
        KI = 0.2
        # tuning constant for derivative control
        KD = 0.2

        # error sum for integral control
        error_sum = 0.0
        # prev error value for derivative control
        prev_error = 0.0
        # constant used to update previous error only in certain intervals. 
        MOD = 20
        cur_error = SP - self.zone_1

        # Proportional Control
        pc = KP * cur_error

        # Integral Control
        error_sum += cur_error
        ic = KI * error_sum

        # Derivative Control
        dc = KD * (cur_error - prev_error)
        if loop_counter % MOD == 0:
            prev_error = cur_error

        # Sum of control values
        control_sum = pc + ic + dc

        return control_sum

if __name__ == '__main__':
    rospy.init_node('pa2')
    PaTwo().follow_wall()
