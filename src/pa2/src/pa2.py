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
        self.init_pid() 

    def scan_cb(self, msg):
        self.zone_0 = min(msg.ranges[0:21] + msg.ranges[340:360])
        self.zone_1 = min(msg.ranges[260:340])
        self.zone_2 = min(msg.ranges[180:260])
        self.update_states()

    def update_states(self):
        if self.zone_alert(self.zone_0):
            self.states['front_wall'] = True
            self.falsify_except('front_wall')
        elif (self.zone_alert(self.zone_1) and
                self.zone_alert(self.zone_2) and
                self.zone_1 > self.zone_2):
            self.states['right_corner_wall'] = True
            self.falsify_except('right_corner_wall')
        elif self.zone_alert(self.zone_1):
            self.states['side_wall'] = True
            self.falsify_except('side_wall')
        elif self.zone_alert(self.zone_2):
            self.states['right_corner_wall'] = True
            self.falsify_except('right_corner_wall')
        else:
            self.states['no_walls'] = True
            self.falsify_except('no_walls')

    def zone_alert(self, zone):
        return not math.isinf(zone)

    def falsify_except(self, cur_state):
        for state in self.states.keys():
            if state != cur_state:
                self.states[state] = False

    def init_zones(self):
        self.zone_0 = 0.0
        self.zone_1 = 0.0
        self.zone_2 = 0.0

    def init_states(self):
        self.states = {
                'no_walls': False,
                'front_wall': False,
                'side_wall': False,
                'right_corner_wall': False
                }

    def init_pid(self):
        # desired setpoint (e.g. desired distance from wall)
        self.sp = 0.7
        # tuning constant for proportional control
        self.KP = 0.3 
        # tuning constant for integral control
        self.KI = 0.0
        # tuning constant for derivative control
        self.KD = 0.0

        # error sum for integral control
        self.error_sum = 0.0

        # prev error value for derivative control
        self.prev_error = 0.0

    def pid_only(self):
        rate = rospy.Rate(10)
        MOD = 20
        loop_counter = 0

        while not rospy.is_shutdown():
            cur_error = self.sp - self.cur_3_oclock_dist

            # Proportional Control
            pc = self.KP * cur_error

            # Integral Control
            self.error_sum += cur_error
            ic = self.KI * self.error_sum

            # Derivative Control
            dc = self.KD * (cur_error - self.prev_error)
            if loop_counter % MOD == 0:
                self.prev_error = cur_error

            control_sum = pc + ic + dc 

            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = control_sum
            self.cmd_vel_pub.publish(twist)
            loop_counter += 1
            rate.sleep()
    
    def follow_wall(self):
        rate = rospy.Rate(10)

        twist = Twist()

        loop_counter = 0

        while not rospy.is_shutdown():
            if self.states['no_walls']:
                twist.linear.x = 0.2
            elif self.states['front_wall']:
                twist.angular.z = 0.2
            elif self.states['right_corner_wall']:
                twist.angular.z = -0.2
            elif self.states['side_wall']:
                self.pid(loop_counter, twist)
            loop_counter += 1
            rate.sleep()
            self.cmd_vel_pub.publish(twist)

    def pid(self, loop_counter, twist):
        MOD = 20
        cur_error = self.sp - self.zone_1

        # Proportional Control
        pc = self.KP * cur_error

        # Integral Control
        self.error_sum += cur_error
        ic = self.KI * self.error_sum

        # Derivative Control
        dc = self.KD * (cur_error - self.prev_error)
        if loop_counter % MOD == 0:
            self.prev_error = cur_error

        control_sum = pc + ic + dc 

        twist.linear.x = 0.2
        twist.angular.z = control_sum


if __name__ == '__main__':
    rospy.init_node('pa2')
    PaTwo().follow_wall()

