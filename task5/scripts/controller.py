#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik, Romala ]

import rospy

from geometry_msgs.msg import Pose2D		# Message type used for receiving feedback
from std_msgs.msg import String

import math		                            # If you find it useful
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

class controller:

	def __init__(self):

		# initialising node named "controller_node"
		rospy.init_node('controller_node')

		# initialising publisher of /right_wheel_force, /front_wheel_force, /left_wheel_force 
		self.move = rospy.Publisher('velocity_array', String, queue_size=10)

		# initialising subscriber of /detected_aruco and /task2_goals
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
		self.velocity = []

		# declaring thresholds
		self.dist_thresh = 0.05
		self.angle_thresh = 5 * (math.pi/180)

		# Initialising Kp values for the P Controller
		self.kp_linear = 5.0
		self.kp_angular = 35.0

		# declaring wrench message for 3 wheels 
		self.data_to_send = String()

		# For maintaining control loop rate.
		rate = rospy.Rate(100)
		
		# control loop
		while not rospy.is_shutdown():

			while self.index < len(self.theta_goals):

				# Calculating Global Error from feedback
				self.global_error()

				# Changing to robot frame by using Rotation Matrix 
				self.body_error()

				# Calculating the required velocity of bot 
				self.PID()
				
				# Finding the required force vectors for individual wheels from it
				self.inverse_kinematics()

				# Moving and Orienting till goal is reached
				if(self.error_d < self.dist_thresh and abs(self.error_th) < self.angle_thresh):
					
				# Applying appropriate force vectors

					# Stopping
					self.vr = 0.0
					self.vf = 0.0
					self.vl = 0.0

					self.velocity = [self.vr, self.vf, self.vl]
					self.data_to_send = ','.join([str(e) for e in self.velocity])

					self.move.publish(self.data_to_send)
					
					# Stopping for 1 sec
					rospy.sleep(1)

					# Updating goals				
					self.index += 1

				else:

					self.velocity = [self.vr, self.vf, self.vl]
					self.data_to_send = ','.join([str(e) for e in self.velocity])

					self.move.publish(self.data_to_send)

			print(self.velocity)
			rate.sleep()
		
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

	def PID(self):

		# implementing a P controller to react to the error with velocities in self.x, self.y and theta
		prop_x = self.x
		prop_y = self.y
		prop_th = self.error_th
		balance_x = (self.kp_linear*prop_x) 
		balance_y = (self.kp_linear*prop_y)
		balance_th = (self.kp_angular*prop_th)

		# Updating balanced speed
		self.vel_x = balance_x
		self.vel_y = balance_y
		self.vel_z = balance_th	 			

	def inverse_kinematics(self):

		d = 0.165
		r = 0.029

		# inverse kinematics matrix derived through calculation
		mat1 = ([-d, 1, 0],
		   		[-d, -1/2, -math.sqrt(3)/2],
				[-d, -1/2, math.sqrt(3)/2])

		# velocity matrix
		mat2 = ([self.vel_z],
				[self.vel_x],
				[self.vel_y])	

		# wheel force matrix
		res = (1/r) * np.dot(mat1,mat2)

		# assigning force to respective wheels
		self.vf = float(res[0])
		self.vr = float(res[1])
		self.vl = float(res[2])

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException:
		pass