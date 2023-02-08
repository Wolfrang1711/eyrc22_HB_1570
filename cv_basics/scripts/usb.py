#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (HB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:		    [ eYRC#HB#1570 ]
# Author List:		[ Pratik, Romala ]
# Filename:	 	    feedback.py
# Functions:		callback(), aruco_detection()
# Nodes:			Publishing node: /detected_aruco
#                   Subscribing node: /overhead_cam/image_raw

######################## IMPORT MODULES ##########################

import numpy				                            # If you find it required
import rospy 				
from sensor_msgs.msg import Image 	                    # Image is the message type for images in ROS
from cv_bridge import CvBridge  				        # Package to convert between ROS and OpenCV Images
import cv2				                                # OpenCV Library
import math				                                # If you find it required
from geometry_msgs.msg import Pose2D	                # Required to publish ARUCO's detected position & orientation
class ArucoFeedback():

	def __init__(self):

		# initialising node named "aruco_feedback_node"
		rospy.init_node('aruco_feedback_node') 

		# initialising publisher and subscriber of /detected_aruco and /overhead_cam/image_raw respectively
		self.aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
		rospy.Subscriber('usb_cam/image_rect', Image, self.callback)

		# declaring a Pose2D message
		self.aruco_msg = Pose2D()

		# initialising the required variables
		self.current_frame = None
		self.x = 0.0
		self.y = 0.0
		self.angle = 0.0

		# defining aruco dictionary and parameters
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		while not rospy.is_shutdown():

			# skipping empty frames
			if self.current_frame is None: 
				continue

			cv2.imshow('image',self.current_frame)
			
			
			# calling function for aruco detection
			self.aruco_detection()

			# updating and publishing pose values
			self.aruco_msg.x = self.x
			self.aruco_msg.y = self.y 
			self.aruco_msg.theta = self.angle

			self.aruco_publisher.publish(self.aruco_msg)

			cv2.waitKey(1)

	def callback(self, data):

		rospy.loginfo("receiving camera frame")

		# Bridge is Used to Convert ROS Image message to OpenCV image
		br = CvBridge()

		# Receiving raw image in a "grayscale" format and resizing image
		self.get_frame = br.imgmsg_to_cv2(data, desired_encoding="mono8")  
		self.current_frame = self.get_frame

       
				
	def aruco_detection(self):		
		
		# finding corners and aruco ids
		corner, ids, _ = cv2.aruco.detectMarkers(self.current_frame, self.arucoDict, parameters = self.arucoParams)	

		if len(corner) > 0:
			# flatten the Aruco IDs list
			ids = ids.flatten()

			for (markerCorner, _) in zip(corner, ids):

				# extracting the marker corner (which are always returned in top-left, top-right, bottom-right, and bottom-left order)
				corner = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = corner

				# converting each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				# computing the centre of the Aruco marker
				self.x = int((topLeft[0] + bottomRight[0]) / 2.0)
				self.y = int((topLeft[1] + bottomRight[1]) / 2.0)

				# finding midpoint of rightside of aruco marker
				(midx, midy) = int((topRight[0] + bottomRight[0]) / 2), int((topRight[1] + bottomRight[1]) / 2)
				
				# finding orientation
				self.angle = math.atan2((self.y - midy),(midx - self.x)) 		

if __name__ == '__main__':

	aruco = ArucoFeedback()
	
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
