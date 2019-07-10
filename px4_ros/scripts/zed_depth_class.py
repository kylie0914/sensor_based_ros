#! /usr/bin/python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
#from zed_subs_class import *

class zed_depth:
    def __init__(self):

        self.left_img = None
        self.right_img = None
        self.depth = None
    #    rospy.init_node('zed_depth', anonymous=True)
        self.bridge_object = CvBridge()
        rospy.Subscriber("/zed/zed_node/depth/depth_registered",Image,self.depth_callback)

    def depth_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding='32FC1')
            self.depth=cv_image
        except CvBridgeError as e:
            print(e)
        cv2.imshow("depth",cv_image/3.8)
        cv2.waitKey(1)
