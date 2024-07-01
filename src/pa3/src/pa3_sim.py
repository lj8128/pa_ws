#!/usr/bin/env python3
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class PaThreeSim:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def image_callback(self, msg):
        """Callback to `self.image_sub`."""
        raise NotImplementedError
    
rospy.init_node('line_follower_sim')
follower = PaThreeSim()
rospy.spin()