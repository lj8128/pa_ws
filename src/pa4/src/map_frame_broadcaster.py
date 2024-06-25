#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import TransformStamped

class MapFrameBroadcaster:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        rate = rospy.Rate(10)

        map_tfs = TransformStamped()
        map_tfs.header.frame_id = 'odom'
        map_tfs.child_frame_id = 'map'

        rate = rospy.Rate(10.0)

        while not rospy.is_shutdown():
            try:
                odom_tf = self.tf_buffer.lookup_transform('odom', 'odom', rospy.Time()).transform
                map_tfs.header.stamp = rospy.Time.now()
                map_tfs.transform.translation = odom_tf.translation
                map_tfs.transform.rotation = odom_tf.rotation 

                self.tf_broadcaster.sendTransform(map_tfs)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                continue
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('map_frame_broadcaster')
    MapFrameBroadcaster()