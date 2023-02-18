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
		self.move = rospy.Publisher('velocity_data', String, queue_size=10)

		# initialising subscriber of /detected_aruco and /task2_goals
		rospy.Subscriber('detected_aruco',Pose2D,self.aruco_feedback_Cb)
		
		# initialising goalpoints array
		self.x_goals = [350,150,150,350]
		self.y_goals = [300,350,150,150]
		self.theta_goals = [0.785,2.355,-2.355,-0.785]		                                                                                                                                                                                                                                                                                                           

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
		self.dist_thresh = 1
		self.angle_thresh = 0.1 * (math.pi/180)

		# Initialising Kp values for the P Controller
		self.last_error = 0
		self.intg = 0
		self.diff = 0
		self.prop = 0
		self.kpl = 0.004
		self.kpa = 0.45
		self.ki = 0
		self.kd = 0

		# declaring wrench message for 3 wheels 
		self.data_to_send = String()

		# For maintaining control loop rate.
		rate = rospy.Rate(75)
		
		# control loop
		while not rospy.is_shutdown():

			while self.index < len(self.theta_goals):

				# Calculating Global Error from feedback
				self.global_error()

				# Changing to robot frame by using Rotation Matrix 
				self.body_error()

				# Calculating the required velocity of bot 
				self.balance()
				
				# Finding the required force vectors for individual wheels from it
				self.inverse_kinematics()

				# Moving and Orienting till goal is reached
				if(self.error_d < self.dist_thresh and abs(self.error_th) < self.angle_thresh):	
				# Applying appropriate force vectors

					# Stopping
					self.vr = 0.0
					self.vf = 0.0
					self.vl = 0.0

					self.velocity = [round(self.vf,3), round(self.vl,3), round(self.vr,3)]
					self.data_to_send = ','.join([str(e) for e in self.velocity])

					self.move.publish(self.data_to_send)

					# print(self.data_to_send)
					
					print("Reached goal ", self.index + 1)

					# Stopping for 1 sec
					rospy.sleep(1)

					# Updating goals				
					self.index += 1

				else:

					self.velocity = [round(self.vf,3), round(self.vl,3), round(self.vr,3)]
					self.data_to_send = ','.join([str(e) for e in self.velocity])

					self.move.publish(self.data_to_send)

					# print(self.data_to_send)

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
	
	def balance(self):

		# Updating balanced speed
		self.vel_x = self.PID(self.x, self.kpl)
		self.vel_y = self.PID(self.y, self.kpl)
		self.vel_z = self.PID(self.error_th, self.kpa)

	def PID(self, error, kp):

        # implementing the PID controller to the error  
		self.prop = error
		self.intg = error + self.intg
		self.diff = error - self.last_error
		balance = (kp*self.prop) + (self.ki*self.intg) + (self.kd*self.diff)
		self.last_error = error
		return balance   	     
			
	def inverse_kinematics(self):

		d = 0.165
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
		res = (100/math.pi) * (1/r) * np.dot(mat1,mat2)

		# assigning force to respective wheels
		self.vf = float(res[0])
		self.vr = float(res[1])
		self.vl = float(res[2])

if __name__ == "__main__":
	try:
		controller()
	except rospy.ROSInterruptException as e:
		print(e)