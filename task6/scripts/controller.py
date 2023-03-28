#!/usr/bin/env python

'''
* Team Id : < HB_1570 >
* Author List : < Pratik, Romala>
* Filename: < controller.py >
* Theme: < HB -- Specific to eYRC / eYRCPlus >
* Functions: < __init__(self), aruco_feedback_Cb(self, msg), contours_feedback(self, msg), taskstatus_feedback(self, msg),
  				aruco15_feedback(self, msg), waypoints_generation(self, goal_points, contours_points, pen_points), def global_error(self, goal_points, hola_pose),
  				body_error(self, goal_points, hola_pose), move_hola(self, err_x, err_y, err_th), def PID(self, error, kp), 
  				inverse_kinematics(self, vel_x, vel_y, vel_z), clipper(self, vel_array), pen_move(self, goal_points, pen_points) >
* Global Variables: < None >
'''

import rospy
import yaml
import math		                          
import numpy as np
from task6.msg import aruco_data					
from std_msgs.msg import String, Int32			
from tf.transformations import euler_from_quaternion	

class controller:

	'''
    * Function Name: < __init__ >
    * Input: < None >
    * Output: < None >
    * Logic: < initializing control loop to generate goals, calculate errors. generate and publish velocity data to move to goals on 
				satisfying various conditions then updating goals and doing the pen up/down mechanism >
    * Example Call: < called automatically by Operating System >
    '''

	def __init__(self):

		rospy.init_node('controller_node')

		self.move = rospy.Publisher('velocity_data', String, queue_size=10)
		self.pen = rospy.Publisher('penStatus', Int32, queue_size=10)

		rospy.Subscriber('detected_aruco', aruco_data, self.aruco_feedback_Cb)
		rospy.Subscriber('contours', String, self.contours_feedback)
		rospy.Subscriber('taskStatus', Int32, self.taskstatus_feedback)
		rospy.Subscriber('aruco15_status', Int32, self.aruco15_feedback)

		self.sending_data = String()
		self.pen_data = Int32()
	
		# initialising goalpoints variable - [[x goal], [y goal], [theta goal]]
		self.goals = [[],[],[]]	  
		# initialising pen up/down goal variable - [[pen down x goal], [pen down y goal], [pen up x goal], [pen up y goal]]
		self.pen_goals = [[],[],[],[]]                                                                                                                                                                                                                                                                                                        

		# initialising hola position variables
		self.hola_x = 0
		self.hola_y = 0
		self.hola_theta = 0
		self.hola_position = []

		self.cnt_data = ''
		self.contours = []

		self.velocity = [] 		

		# flag variables
		self.index = 0
		self.task_status = 1
		self.aruco15_visual = 1

		# declaring thresholds
		self.dist_thresh = 5
		self.angle_thresh = 5 * (math.pi/180)

		# initialising PID variables
		self.last_error = 0
		self.intg = 0
		self.diff = 0
		self.prop = 0

		# initialising PID contsants
		self.kp_linear = 0.02
		self.kp_angular = 5.5
		self.ki = 0
		self.kd = 0
		self.clip_range = 500

		rate = rospy.Rate(75)	  

		while not rospy.is_shutdown():

			# parses the given and returns a Python object
			self.contours = yaml.safe_load(self.cnt_data)

			if self.contours is None:
				continue
			
			# calling the waypoints generation function
			self.waypoints_generation(self.goals, self.contours, self.pen_goals)

			# loop till all goals are reached
			if self.index < len(self.goals[2]):

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

					# pen movement 
					self.pen_move(self.goals, self.pen_goals)

					# Updating goals				
					self.index += 1

				else:
					
					# changing to robot frame by using Rotation Matrix 
					error_x, error_y = self.body_error(self.goals, self.hola_position)

					# checking task status and if aruco deteced or not
					if self.task_status == 0 and self.aruco15_visual == 1:
						
						# moving the hola bot
						self.velocity = self.move_hola(error_x, error_y, angle_error)

						# converting velocity list to string and publishing
						self.sending_data = ','.join([str(e) for e in self.velocity])
						self.move.publish(self.sending_data)

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

	'''
    * Function Name: < aruco_feedback_Cb >
    * Input: < msg - data recieved from ROS subscriber callback >
    * Output: < None >
    * Logic: < recieves aruco position from /detected_aruco topic >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 
    
	def aruco_feedback_Cb(self, msg):

        # taking the msg and updating the three variables
		self.hola_x = msg.x
		self.hola_y = msg.y
		self.hola_theta = msg.theta
	
	'''
    * Function Name: < contours_feedback >
    * Input: < msg - data recieved from ROS subscriber callback >
    * Output: < None >
    * Logic: < recieves contours from /contours topic >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 	

	def contours_feedback(self, msg):

        # taking the msg and updating variable
		self.cnt_data = msg.data

	'''
    * Function Name: < taskstatus_feedback >
    * Input: < msg - data recieved from ROS subscriber callback >
    * Output: < None >
    * Logic: < recieves task status if task started or from /taskStatus topic >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 	

	def taskstatus_feedback(self, msg):

        # taking the msg and updating variable
		self.task_status = msg.data	

	'''
    * Function Name: < aruco15_feedback >
    * Input: < msg - data recieved from ROS subscriber callback >
    * Output: < None >
    * Logic: < recieves aruco15 visual status if aruco15 is detected or not from /aruco15_status topic >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 	

	def aruco15_feedback(self, msg):

		# taking the msg and updating the variable
		self.aruco15_visual = msg.data 

	'''
    * Function Name: < waypoints_generation >
    * Input: < goal_points - current goal recieved
			   contour_points - contour groups with co-ordinates
			   pen_points - pen up/down co-ordinates > 
    * Output: < goal_points - goals updated with contour co-ordinates of various groups
	 			pen_points - updated pen up/down co-ordinates  >
    * Logic: < updates the goals with contour co-ordinates meanwhile extractng first and last co-ordinate of every contour group for pen up/down 
	 			and adding homepose as start and end goal >
    * Example Call: < self.waypoints_generation(self.goals, self.contours, self.pen_goals) >
    ''' 	
	
	def waypoints_generation(self, goal_points, contours_points, pen_points):	

		# clearing the lists to store new values
		goal_points[0].clear()
		goal_points[1].clear()
		goal_points[2].clear()

		pen_points[0].clear()
		pen_points[1].clear()
		pen_points[2].clear()
		pen_points[3].clear()

		# storing contour co-ordinates as goals
		for i in range(len(contours_points[0])):

			for j in range(len(contours_points[0][i])):

				goal_points[0].append(int(contours_points[0][i][j]))
				goal_points[1].append(int(contours_points[1][i][j]))
				goal_points[2].append(int(contours_points[2][i][j]))

			# extractng first and last co-ordinate of every contour group for pen up/down 
			pen_points[0].append(int(contours_points[0][i][0]))
			pen_points[1].append(int(contours_points[1][i][0])) 
			pen_points[2].append(contours_points[0][i][len(contours_points[0][i])-1]) 
			pen_points[3].append(contours_points[1][i][len(contours_points[1][i])-1])         

		# adding homepose to start of goals
		goal_points[0][:0] = [250]
		goal_points[1][:0] = [250]
		goal_points[2][:0] = [0]

		# adding homepose to end of goals
		goal_points[0].append(250)
		goal_points[1].append(250)
		goal_points[2].append(0)

		return goal_points, pen_points

	'''
    * Function Name: < global_error >
    * Input: < goal_points - current goal recieved
			   hola_pose - odometry of hola in the camera frame > 
    * Output: < error_theta - error generated between the self.difference of current pose orientation and goal orientation
	 			error_dist - error generated between the self.difference of current pose and goal pose >
    * Logic: < generates error in angle by self.difference of current pose orientation and goal orientation and error in distance
	 			by self.difference of current pose and goal pose >
    * Example Call: < self.global_error(self.goals, self.hola_position) >
    ''' 
    	
	def global_error(self, goal_points, hola_pose):
	
		# calculating error in global frame
		error_x = goal_points[0][self.index] - hola_pose[0]
		error_y = goal_points[1][self.index] - hola_pose[1]
		error_theta = goal_points[2][self.index] - hola_pose[2]
		error_dist = np.linalg.norm(np.array((error_x, error_y)) - np.array((0,0)))

		return error_theta, error_dist

	'''
    * Function Name: < body_error >
    * Input: < goal_points - current goal recieved
			   hola_pose - odometry of hola in the camera frame > 
    * Output: < err_x - body error in x axis 
	 			err_y - body error in y axis >
    * Logic: < shifting and rotation of global axes with respect to robot's body frame axes to generate error in position 
				with respect to global co-ordinates >
    * Example Call: < self.body_error(self.goals, self.hola_position) >
    ''' 

	def body_error(self, goal_points, hola_pose):

		# shifting of global axes to robot's body frame axes		
		shifted_x = goal_points[0][self.index] - hola_pose[0]
		shifted_y = hola_pose[1] - goal_points[1][self.index] 

		# calculating error in position in robot's body frame with respect to global axes with use of rotation matrix
		err_x = shifted_x * math.cos(hola_pose[2]) + shifted_y * math.sin(hola_pose[2])
		err_y = -shifted_x * math.sin(hola_pose[2]) + shifted_y * math.cos(hola_pose[2])

		return err_x, err_y

	'''
    * Function Name: < move_hola >
    * Input: < err_x - error produced in body frame in the linear x
			   err_y - error produced in body frame in the linear y
			   err_z - error produced in body frame in the angular z >
    * Output: < vel_array - list of wheel velocity of 3 wheels  >
    * Logic: < converts the raw error in linear and angular axes to controller speeds passinf through PID then applying
	 			inverse kineamtics then clipping those upto a certain limit to those speeds to produce proper wheel velocities >
    * Example Call: < self.move_hola(error_x, error_y, angle_error) >
    ''' 
    	
	def move_hola(self, err_x, err_y, err_th):

		# updating balanced speeds in linear axis
		vel_x = self.PID(err_x, self.kp_linear)
		vel_y = self.PID(err_y, self.kp_linear)

		# updating balanced speeds in angular axis keeping all edge cases in mind
		if (err_th > 3.14):
			vel_z = self.PID((err_th-6.28), self.kp_angular)
		elif (err_th < -3.14):
			vel_z = self.PID((err_th+6.28), self.kp_angular)
		else:
			vel_z = self.PID((err_th), self.kp_angular)

		# applying inverse kinematics to produce wheel velocities
		vel_array = self.inverse_kinematics(vel_x, vel_y, vel_z)

		# if velocity exceeds the range, then clip it
		if (abs(vel_array[0])>self.clip_range or abs(vel_array[1])>self.clip_range or abs(vel_array[2])>self.clip_range):

			vel_array = self.clipper(vel_array)	

		return vel_array

	'''
    * Function Name: < PID >
    * Input: < error - error produced in body frame in the x,y linear and z angular axis
			   kp - linear and angular kp depending on use >
    * Output: < balance - balanced speed corrected through PID controller  >
    * Logic: < passes the calculated error in linear x,y and angular z through PID controller and produced controlled speeds >
    * Example Call: < self.PID(err_x, self.kp_linear) >
    ''' 
    
	def PID(self, error, kp):

		# implementing the PID controller to the error  
		self.prop = error
		self.intg = error + self.intg
		self.diff = error - self.last_error
		balance = (kp*self.prop) + (self.ki*self.intg) + (self.kd*self.diff)
		self.last_error = error

		return balance  
	
	'''
    * Function Name: < inverse_kinematics >
    * Input: < vel_x - linear velocity in x direction
			   vel_y - linear velocity in y direction
			   vel_z - angular velocity in z direction >
    * Output: < wheel_vel - list of wheel velocity of 3 wheels  >
    * Logic: < takes linear and angular velocity and passes it through inverse kineamtics equation, then converts it 
				into steps per second so that it can be passed in the stepper motor >
    * Example Call: < self.inverse_kinematics(vel_x, vel_y, vel_z) >
    ''' 

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

		# wheel velocity matrix
		res = (1/r) * np.dot(mat1,mat2)

		# converting output values to steps per second 
		res = (100/math.pi) * res

		# assigning velocities to respective wheels
		v_front = int(res[0])
		v_right = int(res[1])
		v_left = int(res[2]) 

		wheel_vel = [v_front, v_left, v_right]	

		return wheel_vel

	'''
    * Function Name: < clipper >
    * Input: < vel_array - list of wheel velocity of 3 wheels >
    * Output: < vel_array - list of clipped wheel velocity of 3 wheels at a certain limit  >
    * Logic: < compares current goal point with a list of points where the pen has to go up or down and executes the action accordingly >
    * Example Call: < self.pen_move(self.goals, self.pen_goals) >
    ''' 
    
	def clipper(self, vel_array):

		# finding max value in list
		max_value = max([abs(x) for x in vel_array])

		# normalizing the list
		for i, val in enumerate(vel_array):
				vel_array[i] = val/max_value

		# remapping the normalised velocities with proper clipping ratio
		vel_array = [int(x * self.clip_range) for x in vel_array]	

		return vel_array	   
	
	'''
    * Function Name: < pen_move >
    * Input: < goal_points - current goal recieved
	 		   pen_points - pen up/down co-ordinates >
    * Output: < None >
    * Logic: < compares current goal point with a list of points where the pen has to go up or down and executes the action accordingly >
    * Example Call: < self.pen_move(self.goals, self.pen_goals) >
    ''' 
    
	def pen_move(self, goal_points, pen_points):

		# coordinating the pen up-down mechanism based on goal points reached
		for i in range(len(pen_points[0])):
			
			# comaparing current goal with list of start points of contour groups for pen down
			if(goal_points[0][self.index] == pen_points[0][i] and goal_points[1][self.index] == pen_points[1][i]):
				
				# updating pen status and publishing data
				self.pen_data = 1
				print("Pen Down")
				self.pen.publish(self.pen_data)

				rospy.sleep(0.5)

			# comaparing current goal with list of end points of contour groups for pen up
			if(goal_points[0][self.index] == pen_points[2][i] and goal_points[1][self.index] == pen_points[3][i]):
				
				# updating pen status and publishing data
				self.pen_data = 0
				print("Pen Up")
				self.pen.publish(self.pen_data)

				rospy.sleep(0.5)

if __name__ == "__main__":

	try:
		controller()
	except rospy.ROSInterruptException as e:
		print(e)