#!/usr/bin/env python

'''
* Team Id : < HB_1570 >
* Author List : < Romala >
* Filename: < contours.py >
* Theme: < HB -- Specific to eYRC / eYRCPlus >
* Functions: < __init__(self), image_mode(self), function_mode(self) >
* Global Variables: < None >
'''

import numpy as np				                       
import rospy 				
from sensor_msgs.msg import Image 
from std_msgs.msg import Int32	                   		
from cv_bridge import CvBridge  				        
import cv2				                                
import math				                               
from task6.msg import aruco_data	            	    

class ArucoFeedback():

	'''
    * Function Name: < __init__ >
    * Input: < None >
    * Output: < None >
    * Logic: < >
    * Example Call: < called automatically by Operating System >
    '''

	def __init__(self):

		rospy.init_node('aruco_feedback_node') 

		self.aruco_publisher = rospy.Publisher('detected_aruco', aruco_data, queue_size=10)
		self.aruco_visibility = rospy.Publisher('aruco15_status', Int32, queue_size=10)

		rospy.Subscriber('usb_cam/image_rect', Image, self.callback)

		# declaring message types
		self.aruco_msg = aruco_data()
		self.aruco15_visual = Int32()

		# initialising variables
		self.get_frame = None
		self.x = 0
		self.y = 0
		self.angle = 0
		
		# defining aruco dictionary and parameters
		self.arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
		self.arucoParams = cv2.aruco.DetectorParameters_create()
		
		# extracting 4 required corners of the 4 corner aruco markers for perspective transform
		while(True):
			
			# skipping blank frames
			if self.get_frame is None: 
				continue

			# extracting corners and ids from aruco markers
			corner, ids, _ = cv2.aruco.detectMarkers(self.get_frame, self.arucoDict, parameters = self.arucoParams)	
				
			if len(corner) == 5:

				# flatten the Aruco IDs list
				ids = ids.flatten()

				for (markerCorner, markerID) in zip(corner, ids):

					if markerID == 4:
						markerCorner = markerCorner.reshape((4, 2))
						self.corner1 = markerCorner[0]

					if markerID == 8:
						markerCorner = markerCorner.reshape((4, 2))
						self.corner2 = markerCorner[1]

					if markerID == 10:
						markerCorner = markerCorner.reshape((4, 2))
						self.corner3 = markerCorner[2]

					if markerID == 12:
						markerCorner = markerCorner.reshape((4, 2))
						self.corner4 = markerCorner[3]
					
				break
				
		while not rospy.is_shutdown():

			# skipping empty frames
			if self.get_frame is None: 
				continue
			
			self.final_frame = self.frame_transform(self.get_frame)
			
			self.aruco_detection()
			
			rospy.loginfo("Publishing Odometry")

			# updating and publishing pose values
			self.aruco_msg.x = self.x
			self.aruco_msg.y = self.y 
			self.aruco_msg.theta = self.angle

			# publishing the data to /aruco_data topic
			self.aruco_publisher.publish(self.aruco_msg)

			cv2.imshow('Frame',self.final_frame)
			cv2.waitKey(1)
	
	'''
    * Function Name: < callback >
    * Input: < data - data recieved from ROS image message >
    * Output: < None >
    * Logic: < converts ROS message to OpenCV image >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 
	def callback(self, data):

		# bridge is Used to Convert ROS Image message to OpenCV image
		br = CvBridge()

		# receiving raw image in a "grayscale" format
		self.get_frame = br.imgmsg_to_cv2(data, desired_encoding="rgb8")

	'''
    * Function Name: < frame_transform >
    * Input: < original_frame - the image frame received >
    * Output: < None >
    * Logic: < performs perspective transform on the received image frame >
    * Example Call: < self.frame_transform(self.get_frame) >
    ''' 	
	def frame_transform(self, original_frame):
		
		# generating corner point matrix
		src_points = np.array([self.corner1, self.corner2, self.corner4, self.corner3], dtype=np.float32)
		dst_points = np.array([[0, 0], [500, 0],[0, 500], [500, 500]], dtype=np.float32)

		# generating perspective transform matrix
		M = cv2.getPerspectiveTransform(src_points, dst_points)

		# Apply the perspective transformation to the input image
		frame = cv2.warpPerspective(original_frame, M, (500, 500))

		return frame

	'''
    * Function Name: < aruco_detection >
    * Input: < None >
    * Output: < None >
    * Logic: < detects aruco marker with ID 15 to extract the bot's position and orientation >
    * Example Call: < self.aruco_detection >
    ''' 	
	def aruco_detection(self):

		# extracting corners and ids from aruco markers
		corner, ids, _ = cv2.aruco.detectMarkers(self.final_frame, self.arucoDict, parameters = self.arucoParams)	
			
		if len(corner) > 0:

			# flatten the Aruco IDs list
			ids = ids.flatten()

			for (markerCorner, markerID) in zip(corner, ids):
				
				if markerID == 15:
					
					# aruco15 marker visible
					aruco15_visual = 1
					self.aruco_visibility.publish(aruco15_visual)

					# extracting the marker corners
					markerCorner = markerCorner.reshape((4, 2))
					(topLeft, topRight, bottomRight, bottomLeft) = markerCorner
			
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

					# show the current position and orientation of the bot on the frame
					cv2.putText(img = self.final_frame, text = str(self.x), org = (130, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (0, 0, 0), thickness = 2)
					cv2.putText(img = self.final_frame, text = str(self.y), org = (200, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (0, 0, 0), thickness = 2)
					cv2.putText(img = self.final_frame, text = str(round(self.angle,3)), org = (270, 24), fontFace = cv2.FONT_HERSHEY_DUPLEX, fontScale = 0.8, color = (0, 0, 0), thickness = 2)	
	
		else:

				# aruco15 marker not visible
				aruco15_visual = 0
				self.aruco_visibility.publish(aruco15_visual)	

if __name__ == '__main__':

	aruco = ArucoFeedback()
	
	try:
		if not rospy.is_shutdown():
			rospy.spin()
	except rospy.ROSInterruptException as e:
		print(e)
