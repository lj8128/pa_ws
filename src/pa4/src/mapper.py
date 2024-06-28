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
        self.pin_dict = {}

    def set_pins(self, fid_ids):
        for id in fid_ids:
            tfs = TransformStamped()
            tfs.header.frame_id = 'odom'
            tfs.child_frame_id = f'pin_{id}'
            self.pin_dict[id] = {'tfs': tfs, 'mapped': False}

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            for fid_id, pin in self.pin_dict.items():
                try:
                    if not pin['mapped']:
                        odom_to_fid_tf = self.tf_buffer.lookup_transform(
                            'odom',
                            f'fiducial_{fid_id}',
                            rospy.Time()).transform
                        pin['tfs'].transform.translation = odom_to_fid_tf.translation

                        q = quaternion_from_euler(0.0, 0.0, 0.0)
                        (pin['tfs'].transform.rotation.x,
                        pin['tfs'].transform.rotation.y,
                        pin['tfs'].transform.rotation.z,
                        pin['tfs'].transform.rotation.w) = q

                        pin['mapped'] = True

                    pin['tfs'].header.stamp = rospy.Time.now()
                    self.tf_broadcaster.sendTransform(pin['tfs'])

                except (
                    tf2_ros.LookupException,
                    tf2_ros.ExtrapolationException,
                    tf2_ros.ConnectivityException
                    ):
                    continue
            
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('mapper')
    mapper = Mapper()
    fid_ids = [0, 1, 2, 3]
    mapper.set_pins(fid_ids)
    mapper.run()
