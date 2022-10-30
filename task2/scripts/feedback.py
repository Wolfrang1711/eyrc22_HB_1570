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

# Team ID:		    [ eYRC#HB#1570  ]
# Author List:		[ Pratik, Romala ]
# Filename:	 	    feedback.py
# Functions:
# Nodes:		


######################## IMPORT MODULES ##########################

import numpy				                            # If you find it required
import rospy 				
from sensor_msgs.msg import Image 	                    # Image is the message type for images in ROS
from cv_bridge import CvBridge, CvBridgeError	        # Package to convert between ROS and OpenCV Images
import cv2				                                # OpenCV Library
import math				                                # If you find it required
from geometry_msgs.msg import Pose2D	                # Required to publish ARUCO's detected position & orientation
class feedback():

	def __init__(self):

		rospy.init_node('aruco_feedback_node') 

		self.aruco_publisher = rospy.Publisher('detected_aruco', Pose2D, queue_size=10)
		 
		rospy.Subscriber('overhead_cam/image_raw', Image, self.callback)

		self.aruco_msg = Pose2D()

		self.cX = 0.0
		self.cY = 0.0
		self.angle = 0.0

		self.corners = []

		self.current_frame = None

		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		while not rospy.is_shutdown():

			self.aruco_detection()
			
			rospy.spin()


	def callback(self, data):

		# Bridge is Used to Convert ROS Image message to OpenCV image
		br = CvBridge()
		rospy.loginfo("receiving camera frame")

		try:
			self.get_frame = br.imgmsg_to_cv2(data, desired_encoding="bgr8")  # Receiving raw image in a "grayscale" format
			self.current_frame = cv2.resize(self.get_frame, (500, 500), interpolation = cv2.INTER_LINEAR)
		except CvBridgeError as e:
			print(e)
			
		self.corners, self.ids, _ = cv2.aruco.detectMarkers(self.current_frame, self.arucoDict, parameters = self.arucoParams)	
				
	def aruco_detection(self):		

		if len(self.corners) > 0:
			# flatten the ArUco IDs list
			self.ids = self.ids.flatten()

			for (markerCorner, _) in zip(self.corners, self.ids):
				# extract the marker self.corners (which are always returned
				# in top-left, top-right, bottom-right, and bottom-left
				# order)
				self.corners = markerCorner.reshape((4, 2))
				(topLeft, topRight, bottomRight, bottomLeft) = self.corners
				# convert each of the (x, y)-coordinate pairs to integers
				topRight = (int(topRight[0]), int(topRight[1]))
				bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
				bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
				topLeft = (int(topLeft[0]), int(topLeft[1]))

				# draw the bounding box of the ArUCo detection
				cv2.line(self.current_frame, topLeft, topRight, (0, 0, 255), 1)
				cv2.line(self.current_frame, topRight, bottomRight, (0, 0, 255), 1)
				cv2.line(self.current_frame, bottomRight, bottomLeft, (0, 0, 255), 1)
				cv2.line(self.current_frame, bottomLeft, topLeft, (0, 0, 255), 1)
				# compute and draw the center (x, y)-coordinates of the
				# ArUco marker
				self.cX = int((topLeft[0] + bottomRight[0]) / 2.0)
				self.cY = int((topLeft[1] + bottomRight[1]) / 2.0)
				cv2.circle(self.current_frame, (self.cX, self.cY), 3, (0, 0, 255), -1)

				(midx, midy) = int((topRight[0] + topLeft[0]) / 2), int((topRight[1] + topLeft[1]) / 2)

				cv2.circle(self.current_frame, (midx,midy), 3, (0, 0, 255), -1)            
					
				robo_or = math.atan2(midx-self.cX, midy-self.cY) 
				self.angle = robo_or 


				self.aruco_msg.x = self.cX
				self.aruco_msg.y = self.cY
				self.aruco_msg.theta = self.angle

				self.aruco_publisher.publish(self.aruco_msg)

		# INSTRUCTIONS & HELP : 
		#	-> Use OpenCV to find ARUCO MARKER from the IMAGE
		#	-> You are allowed to use any other library for ARUCO detection, 
		#        but the code should be strictly written by your team and
		#	   your code should take image & publish coordinates on the topics as specified only.  
		#	-> Use basic high-school geometry of "TRAPEZOIDAL SHAPES" to find accurate marker coordinates & orientation :)
		#	-> Observe the accuracy of cv2.aruco detection & handle every possible corner cases to get maximum scores !

		############################################

			
if __name__ == '__main__':

	try:
		feedback()
	except rospy.ROSInterruptException:
		pass
