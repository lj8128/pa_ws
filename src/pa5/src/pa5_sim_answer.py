#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PaFiveSim:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.zone = [0, 0, 0]
        self.init_states()
        
    def init_states(self):
        """Initialize a map for the states of the robot."""
        self.states = {
                'go_straight': False,
                'turn_right': False,
                'turn_left': False,
                }

    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        ranges = list(msg.ranges)
        
        for i, r in enumerate(ranges):
            if r > 0.3:
                ranges[i] = float('inf')
             
        self.zone[0] = min(ranges[0:11] + ranges[330:360])
        self.zone[1] = min(ranges[280:330])
        self.zone[2] = min(ranges[180:280])
        self.update_states()

    def update_states(self):
        """ Updates the state the robot is in."""
        if self.wall_at_zone(0):
            self.set_state('turn_left')
        elif (self.wall_at_zone(1) or
              self.wall_at_zone(1) and self.wall_at_zone(2)
              and self.zone[1] < self.zone[2]):
            self.set_state('go_straight')
        elif (self.wall_at_zone(1) and self.wall_at_zone(2)
              and self.zone[1] >= self.zone[2] or
              self.wall_at_zone(2)):
            self.set_state('turn_right')
        else:
            self.set_state('go_straight')

    def wall_at_zone(self, zone_num): 
        """
        If the minimum distance in a lidar zone is `inf`, it means no walls have
        been detected in that zone.
        """
        return not math.isinf(self.zone[zone_num])

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

        while not rospy.is_shutdown():
            for state in self.states.keys():
                if self.states[state] == True:
                    rospy.loginfo(f'----> cur_state: {state}') 

            if self.states['go_straight']:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            elif self.states['turn_right']:
                twist.linear.x = 0.0
                twist.angular.z = -0.2
            elif self.states['turn_left']:
                twist.linear.x = 0.0
                twist.angular.z = 0.2
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('pa5_sim')
    PaFiveSim().follow_wall()
