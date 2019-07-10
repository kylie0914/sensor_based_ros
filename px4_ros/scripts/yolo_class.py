#! /usr/bin/python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
import numpy as np


# ROS
class bounding_boxes:
	def __init__(self):
		self.object = None
		self.confidence = None
		self.img_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',
                                        BoundingBoxes, self.bbox_callback)
	def bbox_callback(self, data):
		self.object= data.bounding_boxes[0].Class
		self.confidence = data.bounding_boxes[0].probability
	#	print("object:", self.object, "prob.: ", self.confidence)
