#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import TransformStamped

class Mapper:

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.fiducial_ids = 

        rate = rospy.Rate(10)

        target_tfs = TransformStamped()
        target_tfs.header.frame_id = 'odom'
        target_tfs.child_frame_id = 'target_Z'

        rate = rospy.Rate(10.0)

        tf_not_set = True 

        while not rospy.is_shutdown():

            try:
                if tf_not_set:
                    odom_to_fid_tf = self.tf_buffer.lookup_transform('odom',
                                                            'fiducial_0',
                                                            rospy.Time()).transform
                    target_tfs.transform.translation.x = odom_to_fid_tf.translation.x
                    target_tfs.transform.translation.y = odom_to_fid_tf.translation.y
                    target_tfs.transform.translation.z = odom_to_fid_tf.translation.z
                    q = quaternion_from_euler(0.0, 0.0, 0.0)

                    target_tfs.transform.rotation.x = q[0]
                    target_tfs.transform.rotation.y = q[1]
                    target_tfs.transform.rotation.z = q[2]
                    target_tfs.transform.rotation.w = q[3]
                    tf_not_set = False

                target_tfs.header.stamp = rospy.Time.now()
                self.tf_broadcaster.sendTransform(target_tfs)

            except (tf2_ros.LookupException, tf2_ros.ExtrapolationException):
                continue
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mapper')
    Mapper()