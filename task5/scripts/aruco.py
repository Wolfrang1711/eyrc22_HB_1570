#!/usr/bin/env python

# Team ID:		    [ eYRC#HB#1570 ]
# Author List:		[ Romala ]

import numpy as np				                       
import rospy 				
from sensor_msgs.msg import Image 
from std_msgs.msg import Int32	                   		# Image is the message type for images in ROS
from cv_bridge import CvBridge  				        # Package to convert between ROS and OpenCV Images
import cv2				                                # OpenCV Library
import math				                               
from task5.msg import aruco_data	            	    # Required to publish ARUCO's detected position & orientation

class ArucoFeedback():

	def __init__(self):

		# initialising node named "aruco_feedback_node"
		rospy.init_node('aruco_feedback_node') 

		# initialising publisher /detected_aruco
		self.aruco_publisher = rospy.Publisher('detected_aruco', aruco_data, queue_size=10)

		# initialising publisher /aruco15_status
		self.aruco_visibility = rospy.Publisher('aruco15_status', Int32, queue_size=10)

		# initialising subscriber /usb_cam/image_rect
		rospy.Subscriber('usb_cam/image_rect', Image, self.callback)

		# declaring data message type
		self.aruco_msg = aruco_data()
		self.aruco15_visual = Int32()

		# initialising the required variables
		self.final_frame = None
		self.x = 0.0
		self.y = 0.0
		self.angle = 0.0
		
		# defining coordinate points for arena detection
		self.topleftx, self.toplefty = 0, 0
		self.toprightx, self.toprighty = 0, 0
		self.bottomleftx, self.bottomlefty = 0, 0
		self.bottomrightx, self.bottomrighty = 0, 0

		# defining aruco dictionary and parameters
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
		self.arucoParams = cv2.aruco.DetectorParameters_create()

		while not rospy.is_shutdown():

			# skipping empty frames
			if self.final_frame is None: 
				continue
			
			# extraacting corner points of 4 corner aruco
			self.arena_rectify()
			
			# aruco 15 detection
			self.aruco_detection()

			# updating and publishing pose values
			self.aruco_msg.x = self.x
			self.aruco_msg.y = self.y 
			self.aruco_msg.theta = self.angle

			rospy.loginfo("Publishing Odometry")

			self.aruco_publisher.publish(self.aruco_msg)

			cv2.imshow('Frame',self.final_frame)
			cv2.waitKey(1)

	def callback(self, data):

		# bridge is Used to Convert ROS Image message to OpenCV image
		br = CvBridge()

		# receiving raw image in a "grayscale" format
		self.get_frame = br.imgmsg_to_cv2(data, desired_encoding="rgb8")

		# defining four corner points for perspective transform
		pts1 = np.float32([[self.topleftx, self.toplefty], [self.toprightx, self.toprighty],
                       [self.bottomleftx, self.bottomlefty], [self.bottomrightx, self.bottomrighty]])
		pts2 = np.float32([[0, 0], [500, 0],
						[0, 500], [500, 500]])
		
		# applying Perspective Transform Algorithm
		matrix = cv2.getPerspectiveTransform(pts1, pts2)
		self.final_frame = cv2.warpPerspective(self.get_frame, matrix, (500, 500))

		# show the current position and orientation of the bot on the frame
		cv2.putText(img = self.final_frame, text = str(self.x), org = (130, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (0, 0, 0), thickness = 2)
		cv2.putText(img = self.final_frame, text = str(self.y), org = (200, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (0, 0, 0), thickness = 2)
		cv2.putText(img = self.final_frame, text = str(round(self.angle,3)), org = (270, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (0, 0, 0), thickness = 2)	


	def arena_rectify(self):
		
		# finding corners and aruco ids
		corner, ids, _ = cv2.aruco.detectMarkers(self.get_frame, self.arucoDict, parameters = self.arucoParams)	

		if len(corner) > 0:

			# flatten the Aruco IDs list
			ids = ids.flatten()

			for (markerCorner, markerID) in zip(corner, ids):
			
			# extracting 4 corners of 4 arucos for perspective transform

				if(markerID == 4):

					# extracting the marker corners 
					corner = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corner
			
					# converting each of the (x, y)-coordinate pairs to integers
					topLeft = (int(topLeft[0]), int(topLeft[1]))
					self.topleftx = int(topLeft[0])
					self.toplefty = int(topLeft[1])

				if(markerID == 8):

					# extracting the marker corners
					corner = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corner
			
					# converting each of the (x, y)-coordinate pairs to integers
					topRight = (int(topRight[0]), int(topRight[1]))
					self.toprightx = int(topRight[0])
					self.toprighty = int(topRight[1])

				if(markerID == 10):

					# extracting the marker corners
					corner = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corner
			
					# converting each of the (x, y)-coordinate pairs to integers
					bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
					self.bottomrightx = int(bottomRight[0])
					self.bottomrighty = int(bottomRight[1])
			
				if(markerID == 12):

					# extracting the marker corners
					corner = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = corner
			
					# converting each of the (x, y)-coordinate pairs to integers
					bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
					self.bottomleftx = int(bottomLeft[0])
					self.bottomlefty = int(bottomLeft[1])

	def aruco_detection(self):		
		
		# finding corners and aruco ids
		corner, ids, _ = cv2.aruco.detectMarkers(self.final_frame, self.arucoDict, parameters = self.arucoParams)	

		if len(corner) > 0:

			# flatten the Aruco IDs list
			ids = ids.flatten()

			# aruco15 data ectraction
			for (markerCorner, markerID) in zip(corner, ids):

				if(markerID == 15):
					
					# for the bot to operate according to the detected aruco
					self.aruco15_visual = 1
					self.aruco_visibility.publish(self.aruco15_visual)
					
					# extracting the marker corners
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

				else:

					# for the bot to operate according to the detected aruco
					self.aruco15_visual = 0	
					self.aruco_visibility.publish(self.aruco15_visual)
			
if __name__ == '__main__':

	aruco = ArucoFeedback()
	
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
