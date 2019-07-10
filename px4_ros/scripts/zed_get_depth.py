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
from zed_depth_class import *

if __name__ == '__main__': 
	try:
		detector = zed_depth()
		rospy.init_node('zed_depth', anonymous=True)
		frame_rate = rospy.Rate(10)
		while True:						
			frame_rate.sleep()
			img = detector.depth
			print(np.mean(img[30,336-50:336+50]))
			cv2.imshow('Disparity Map', img)
			k = cv2.waitKey(1)					
			if k == 27:   # esc key
				cv2.destroyAllWindow()	
				break  
	except rospy.ROSInterruptException:
		rospy.loginfo("Detector node terminated.")
				


