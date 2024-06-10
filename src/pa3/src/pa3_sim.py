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
        self.twist = Twist()

    def image_callback(self, msg):
        # Use the cv_bridge package to convert ROS sensor_msgs/Image messages 
        # into OpenCV2 images (cf. PRR p.197)
        image = self.bridge.imgmsg_to_cv2(msg)

        # Convert the BGR8 image into an HSV image (cf. PRR p.201)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Convert the HSV image to a binary image where all pixels that are in
        # the range [lower_yellow, upper_yellow] are white while those that are
        # not are black (cf. PRR p.203)
        lower_yellow = numpy.array([10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
       
        # Zero-out all pixels except those that are "in the 20-row portion of"
        # the mask "corresponding to the one-meter distance in front of the
        # Turtlebot" (PRR, p.205). 
        h, w, d = image.shape
        search_top = 3*h//4
        search_bot = 3*h//4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        
        # Calculate the centroid of the pixels in the mask that do not have a 
        # value of 0.
        M = cv2.moments(mask)
        if M['m00'] > 0:
             cx = int(M['m10']/M['m00'])
             cy = int(M['m01']/M['m00'])

             # Draw a circle on the centroid.
             cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

             # Configure and publish twist messages that have the Turtlebot
             # follow the centroid.
             err = cx - w/2
             self.twist.linear.x = 0.2
             self.twist.angular.z = -float(err) / 100
             self.cmd_vel_pub.publish(self.twist)
        
        # show the image (not the mask) on the window configured in the __init__
        # function.
        cv2.imshow("window", image)
        cv2.waitKey(3) 
    
rospy.init_node('line_follower_sim')
follower = PaThreeSim()
rospy.spin()