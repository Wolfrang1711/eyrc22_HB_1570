#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik, Romala ]

import rospy
import yaml

from task5.msg import aruco_data					# Message type used for receiving aruco feedback
from std_msgs.msg import String, Int32				# Message type used for send velocity data

import math		                          
import numpy as np

from tf.transformations import euler_from_quaternion	# Convert angles

class controller:

	def __init__(self):

		# initialising node named "controller_node"
		rospy.init_node('controller_node')

		# initialising publisher to send velocity data
		self.move = rospy.Publisher('velocity_data', String, queue_size=10)

		# initialising publisher to penStatus
		self.pen = rospy.Publisher('penStatus', Int32, queue_size=10)

		# initialising subscriber of detected_aruco
		rospy.Subscriber('detected_aruco', aruco_data, self.aruco_feedback_Cb)

		# initialising subscriber of contours
		rospy.Subscriber('contours', String, self.contours_feedback)

		# initialising subscriber of taskStatus
		rospy.Subscriber('taskStatus', Int32, self.taskstatus_feedback)

		# initialising subscriber of aruco15_status
		rospy.Subscriber('aruco15_status', Int32, self.aruco15_feedback)
	
		# initialising goalpoints array
		self.x_goals = []
		self.y_goals = []
		self.theta_goals = []
		self.goals = [self.x_goals, self.y_goals, self.theta_goals]	                                                                                                                                                                                                                                                                                                           

		# initialising required variables
		self.hola_x = 0
		self.hola_y = 0
		self.hola_theta = 0
		self.hola_position = []

		self.cnt_data = ''
		self.contours = []

		self.pen_x_start = []
		self.pen_y_start = []
		self.pen_x_end = []
		self.pen_y_end = []
		self.pen_goals = [self.pen_x_start, self.pen_y_start, self.pen_x_end, self.pen_y_end]

		self.index = 0
		self.task_status = 1
		self.aruco15_visual = 1

		self.velocity = []

		# declaring thresholds
		self.dist_thresh = 3
		self.angle_thresh = 2 * (math.pi/180)

		# initialising PID variables
		self.last_error = 0
		self.intg = 0
		self.diff = 0
		self.prop = 0

		# initialising PID contsants
		self.kp_linear = 0.03
		self.kp_angular = 5.0
		self.ki = 0
		self.kd = 0.00006
		self.clip_range = 500

		# declaring data message type
		self.sending_data = String()
		self.pen_data = Int32()

		# for maintaining control loop rate.
		rate = rospy.Rate(75)	  

		# control loop
		while not rospy.is_shutdown():

			# parses the given and returns a Python object
			self.contours = yaml.safe_load(self.cnt_data)

			if self.contours is None:
				continue
			
			# calling the waypoints generation function
			self.waypoints_generation(self.goals, self.contours, self.pen_goals)

			
			# loop till all goals are reached
			if self.index < len(self.theta_goals):

				self.hola_position = [self.hola_x, self.hola_y, self.hola_theta]

				# calculating Global Error from feedback
				angle_error, distance_error = self.global_error(self.goals, self.hola_position)

				# loop for moving and orienting till goal is reached
				if((distance_error <= self.dist_thresh and abs(angle_error) <= self.angle_thresh)):

					# Stopping
					self.vr = 0.0
					self.vf = 0.0
					self.vl = 0.0

					# generating velocity list for 3 wheels and converting to string
					self.velocity = [self.vf, self.vl, self.vr]

					# converting velocity list to string and publishing
					self.sending_data = ','.join([str(e) for e in self.velocity])
					self.move.publish(self.sending_data)

					print("Goal reached: ", self.x_goals[self.index], self.y_goals[self.index], self.theta_goals[self.index])

					# pen movement 
					self.pen_move(self.goals, self.pen_goals)

					# Updating goals				
					self.index += 1

				else:
					
					# changing to robot frame by using Rotation Matrix 
					error_x, error_y = self.body_error(self.goals, self.hola_position)

					# checking task status
					if not self.task_status:
						
						# moving the hola bot
						self.velocity = self.move_hola(error_x, error_y, angle_error)
						print(self.velocity)

					else:
						# Stopping
						self.vr = 0.0
						self.vf = 0.0
						self.vl = 0.0

						# generating velocity list for 3 wheels and converting to string
						self.velocity = [self.vf, self.vl, self.vr]


					# converting velocity list to string and publishing
					self.sending_data = ','.join([str(e) for e in self.velocity])
					self.move.publish(self.sending_data)

				rate.sleep()

			else:

				print("All goals reached !!!")
				exit()

	def aruco_feedback_Cb(self, msg):

        # taking the msg and updating the three variables
		self.hola_x = msg.x
		self.hola_y = msg.y
		self.hola_theta = msg.theta

	def contours_feedback(self, msg):

        # taking the msg and updating variable
		self.cnt_data = msg.data

	def taskstatus_feedback(self, msg):

        # taking the msg and updating variable
		self.task_status = msg.data	

	def aruco15_feedback(self, msg):

		# taking the msg and updating the variable
		self.aruco15_visual = msg.data 
	
	def waypoints_generation(self, goal_points, contours_points, pen_points):	

		# developing the goal points from the contours received
		goal_points[0].clear()
		goal_points[1].clear()
		goal_points[2].clear()

		pen_points[0].clear()
		pen_points[1].clear()
		pen_points[2].clear()
		pen_points[3].clear()

		for i in range(len(contours_points[0])):

			for j in range(len(contours_points[0][i])):

				goal_points[0].append(int(contours_points[0][i][j]))
				goal_points[1].append(int(contours_points[1][i][j]))
				goal_points[2].append(int(contours_points[2][i][j]))

			pen_points[0].append(int(contours_points[0][i][0]))
			pen_points[1].append(int(contours_points[1][i][0])) 
			pen_points[2].append(contours_points[0][i][len(contours_points[0][i])-1]) 
			pen_points[3].append(contours_points[1][i][len(contours_points[1][i])-1])         

		return goal_points, pen_points
	
	def global_error(self, goal_points, hola_pose):
	
		# calculating error in global frame
		error_x = goal_points[0][self.index] - hola_pose[0]
		error_y = goal_points[1][self.index] - hola_pose[1]
		error_theta = goal_points[2][self.index] - hola_pose[2]
		error_dist = np.linalg.norm(np.array((error_x, error_y)) - np.array((0,0)))

		return error_theta, error_dist

	def body_error(self, goal_points, hola_pose):

		# Calculating error in body frame		
		shifted_x = goal_points[0][self.index] - hola_pose[0]
		shifted_y = hola_pose[1] - goal_points[1][self.index] 
		err_x = shifted_x * math.cos(hola_pose[2]) + shifted_y * math.sin(hola_pose[2])
		err_y = -shifted_x * math.sin(hola_pose[2]) + shifted_y * math.cos(hola_pose[2])

		return err_x, err_y
	
	def move_hola(self, err_x, err_y, err_th):

		# Updating balanced speeds
		vel_x = self.PID(err_x, self.kp_linear)
		vel_y = self.PID(err_y, self.kp_linear)

		if (err_th > 3.14):
			vel_z = self.PID((err_th-6.28), self.kp_angular)
		elif (err_th < -3.14):
			vel_z = self.PID((err_th+6.28), self.kp_angular)
		else:
			vel_z = self.PID((err_th), self.kp_angular)

		vel_array = self.inverse_kinematics(vel_x, vel_y, vel_z)

		# loop if velocity exceeds the range
		if (abs(vel_array[0])>self.clip_range or abs(vel_array[1])>self.clip_range or abs(vel_array[2])>self.clip_range):

			vel_array = self.clipper(vel_array)	

		return vel_array

	def PID(self, error, kp):

		# implementing the PID controller to the error  
		self.prop = error
		self.intg = error + self.intg
		self.diff = error - self.last_error
		balance = (kp*self.prop) + (self.ki*self.intg) + (self.kd*self.diff)
		self.last_error = error

		return balance  

	def inverse_kinematics(self, vel_x, vel_y, vel_z):
		
		# chassis radius and wheel radius respectively
		d = 0.175
		r = 0.029

		# inverse kinematics matrix derived through calculation
		mat1 = ([d, -1, 0],
				[d, 1/2, math.sqrt(3)/2],
				[d, 1/2, -math.sqrt(3)/2])

		# velocity matrix
		mat2 = ([vel_z],
				[vel_x],
				[vel_y])	

		# wheel force matrix
		res = (1/r) * np.dot(mat1,mat2)

		# converting output values to steps per second 
		res = (100/math.pi) * res

		# assigning force to respective wheels
		v_front = int(res[0])
		v_right = int(res[1])
		v_left = int(res[2]) 

		wheel_vel = [v_front, v_left, v_right]	

		return wheel_vel

	def clipper(self, vel_array):

		# finding max value in list
		max_value = max([abs(x) for x in vel_array])

		# normalizing the list
		for i, val in enumerate(vel_array):
				vel_array[i] = val/max_value

		# remapping the normalised velocities with proper clipping ratio
		vel_array = [int(x * self.clip_range) for x in vel_array]	

		return vel_array	   
	
	def pen_move(self, goal_points, pen_points):

		# coordinating the pen up-down mechanism based on goal points reached
		for i in range(len(pen_points[0])):

			if(goal_points[0][self.index] == pen_points[0][i] and goal_points[1][self.index] == pen_points[1][i]):

				self.pen_data = 1
				print("Pen Down")
				self.pen.publish(self.pen_data)

				rospy.sleep(0.5)

			if(goal_points[0][self.index] == pen_points[2][i] and goal_points[1][self.index] == pen_points[3][i]):

				self.pen_data = 0
				print("Pen Up")
				self.pen.publish(self.pen_data)

				rospy.sleep(0.5)

if __name__ == "__main__":

	try:
		controller()
	except rospy.ROSInterruptException as e:
		print(e)