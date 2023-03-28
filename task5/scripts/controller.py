#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik, Romala ]

import rospy
import cv2

from geometry_msgs.msg import Pose2D		# Message type used for receiving aruco feedback
from std_msgs.msg import String				# Message type used for send velocity data

import math		                          
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

from feedback import ArucoFeedback

class controller:

	def __init__(self):

		# initialising node named "controller_node"
		rospy.init_node('controller_node')

		# initialising publisher to send velocity data
		self.move = rospy.Publisher('velocity_data', String, queue_size=10)

		# initialising subscriber of detected_aruco
		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		
		# initialising goalpoints array
		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []		                                                                                                                                                                                                                                                                                                           

		# initialising required variables
		self.hola_x = 0.0
		self.hola_y = 0.0 
		self.hola_theta = 0.0
		self.x = 0
		self.y = 0
		self.error_th = 0
		self.error_d = 0
		self.index = 0
		self.max_value = 0
		self.velocity = []

		# declaring thresholds
		self.dist_thresh = 2
		self.angle_thresh = 2 * (math.pi/180)

		# initialising PID variables
		self.last_error = 0
		self.intg = 0
		self.diff = 0
		self.prop = 0

		# initialising PID contsants
		self.kp_linear = 0.05
		self.kp_angular = 3.5
		self.ki = 0
		self.kd = 0

		# declaring data string 
		self.sending_data = String()

		# For maintaining control loop rate.
		rate = rospy.Rate(75)

		self.function_waypoints()
		
		# control loop
		while not rospy.is_shutdown():

			# loop till all goals are reached
			if self.index < len(self.theta_goals):

				# calculating Global Error from feedback
				self.global_error()

				# loop for moving and orienting till goal is reached
				if(self.error_d < self.dist_thresh and abs(self.error_th) < self.angle_thresh):

					# Stopping
					self.vr = 0.0
					self.vf = 0.0
					self.vl = 0.0

					# generating velocity list for 3 wheels and converting to string
					self.velocity = [self.vf, self.vl, self.vr]

					# converting velocity list to string
					self.sending_data = ','.join([str(e) for e in self.velocity])
					
					# publishing data to velocity_data topic
					self.move.publish(self.sending_data)

					print("Goal: ", self.x_goals[self.index], self.y_goals[self.index], self.theta_goals[self.index])

					# self.path.append((self.x_goals[self.index], self.y_goals[self.index]))
					
					print("Reached goal ", self.index)

					# Updating goals				
					self.index += 1

				else:

					# changing to robot frame by using Rotation Matrix 
					self.body_error()

					# calculating the required velocity of bot 
					self.balance_speed()
						
				# finding the required force vectors for individual wheels from it
					self.inverse_kinematics()

					# generating velocity list for 3 wheels 
					self.velocity = [self.vf, self.vl, self.vr]

					# clipping wheel velocity to a certain optimum limit
					self.clipper()

					# converting velocity list to string
					self.sending_data = ','.join([str(e) for e in self.velocity])

					# publishing data to velocity_data topic
					self.move.publish(self.sending_data)

				rate.sleep()

			else:

				print("All goals reached !!!")	

				# print(self.path)

				# print("Visualizing Path")
				# ArucoFeedback.visualize(self.path)

				exit()
		
	def function_waypoints(self):

		self.x_goals.clear()
		self.y_goals.clear()
		self.theta_goals.clear()

		N = 500
		scale = 1.5

		for i in range(N):

			t =  i*((2*math.pi)/N)

			# defining eqn
			x_eqn = 250*math.cos(t) 
			y_eqn = 125*math.sin(2*t)
			theta_eqn = (math.pi/4)*math.sin(t)

			x_eqn = x_eqn/scale + 250
			y_eqn = 250 - y_eqn/scale

			self.x_goals.append(int(x_eqn))
			self.y_goals.append(int(y_eqn))
			self.theta_goals.append(round(theta_eqn,3))   
					
	def aruco_feedback_Cb(self, msg):

        # taking the msg and updating the three variables
		self.hola_x = msg.x
		self.hola_y = msg.y
		self.hola_theta = msg.theta

	def global_error(self):

		# calculating error in global frame
		self.error_x = self.x_goals[self.index] - self.hola_x
		self.error_y = self.y_goals[self.index] - self.hola_y
		self.error_th = self.theta_goals[self.index] - self.hola_theta
		self.error_d = np.linalg.norm(np.array((self.error_x, self.error_y)) - np.array((0,0)))

	def body_error(self):

		# Calculating error in body frame		
		self.shifted_x = self.x_goals[self.index] - self.hola_x
		self.shifted_y = self.hola_y - self.y_goals[self.index] 
		self.x = self.shifted_x * math.cos(self.hola_theta) + self.shifted_y * math.sin(self.hola_theta)
		self.y = -self.shifted_x * math.sin(self.hola_theta) + self.shifted_y * math.cos(self.hola_theta)

	def clipper(self):
		
		# clipping range
		clip = 500

		# loop if velocity exceeds the range
		if (self.vf>clip or self.vl>clip or self.vr>clip):

			# finding max value in list
			self.max_value = max(self.velocity) 

			# normalizing the list
			for i, val in enumerate(self.velocity):
					self.velocity[i] = val/self.max_value

			# remapping the normalised velocities with proper clipping ratio
			self.velocity = [int(x * clip) for x in self.velocity]	
		
	def balance_speed(self):

		# Updating balanced speeds
		self.vel_x = self.PID(self.x, self.kp_linear)
		self.vel_y = self.PID(self.y, self.kp_linear)

		if (self.error_th > 3.14):
			self.vel_z = self.PID((self.error_th-6.28), self.kp_angular)
		elif (self.error_th < -3.14):
			self.vel_z = self.PID((self.error_th+6.28), self.kp_angular)
		else:
			self.vel_z = self.PID((self.error_th), self.kp_angular)

	def PID(self, error, kp):

		# implementing the PID controller to the error  
		self.prop = error
		self.intg = error + self.intg
		self.diff = error - self.last_error
		balance = (kp*self.prop) + (self.ki*self.intg) + (self.kd*self.diff)
		self.last_error = error
		return balance   	     
			
	def inverse_kinematics(self):
		
		# chassis radius and wheel radius respectively
		d = 0.175
		r = 0.029

		# inverse kinematics matrix derived through calculation
		mat1 = ([d, -1, 0],
				[d, 1/2, math.sqrt(3)/2],
				[d, 1/2, -math.sqrt(3)/2])

		# velocity matrix
		mat2 = ([self.vel_z],
				[self.vel_x],
				[self.vel_y])	

		# wheel force matrix
		res = (1/r) * np.dot(mat1,mat2)

		# converting output values to steps per second 
		res = (100/math.pi) * res

		# assigning force to respective wheels
		self.vf = int(res[0])
		self.vr = int(res[1])
		self.vl = int(res[2])

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException as e:
		print(e)