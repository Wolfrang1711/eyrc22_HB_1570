#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    HolA Bot (HB) Theme (eYRC 2022-23)
*        		===============================================
*
*  This script should be used to implement Task 0 of HolA Bot (KB) Theme (eYRC 2022-23).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			eYRC#HB#1570
# Author List:		[Pratik Kumar Sahoo, Romala Mishra]
# Filename:			task_0.py
# Functions:		[callback(), move()]
# Nodes:		    Publishing - turtle1/cmd_vel
#                   Subscribing - turtle1/pose  


####################### IMPORT MODULES #######################
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
##############################################################

# Main Class
class main:

    def __init__(self):
        
        # Initializing Node
        rospy.init_node('turtlesim', anonymous=True)

        #Iniitializing Publisher and Subscriber
        self.publisher = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber('turtle1/pose', Pose, self.callback)
        
        # Assigning Twist message to a variable
        self.velocity = Twist()
        
        # Initialzing variables
        self.x = 0
        self.y = 0
        self.theta = 0 

        # Loop to run move function until the end
        while not rospy.is_shutdown():

            self.move()

    # Callback function to recieve data from Pose topic
    def callback(self, pose):

        self.x= round(pose.x, 2)
        self.y= round(pose.y, 2)
        self.theta= round(pose.theta, 2)

################# ADD GLOBAL VARIABLES HERE #################

##############################################################


################# ADD UTILITY FUNCTIONS HERE #################

    # Move funtion to execute the required tasks
    def move(self):

        rate = rospy.Rate(10)  #10hz

        # moving the turtle in a semi-circle
        if self.theta >= 0 and self.theta <= 3.14:

            self.velocity.linear.x = 1.0
            self.velocity.angular.z = 1.0

            print("\nMoving in Circle")
            print("X = ", self.x)
            print("Y = ", self.y)
            print("Theta = ", self.theta)

            self.publisher.publish(self.velocity) 

         
        elif self.theta <= 0 or self.theta >= 3.14:
            
            # Rotating the turtle in 90 degrees to face initial point
            if self.theta <= -1.57:

                self.velocity.linear.x = 0.0
                self.velocity.angular.z = 0.5

                print("\nRotating")
                print("X = ", self.x)
                print("Y = ", self.y)
                print("Theta = ", self.theta)

                self.publisher.publish(self.velocity)

            else:

                # Moving the turtle in straight line to initial point               
                if(self.x <= 5.54 and self.y >= 5.54):

                    self.velocity.linear.x = 1.0
                    self.velocity.angular.z = 0.0

                    print("\nMoving in Straigt Line")
                    print("X = ", self.x)
                    print("Y = ", self.y)
                    print("Theta = ", self.theta)

                    self.publisher.publish(self.velocity)

                # Stopping the turtle at initial point
                else:

                    self.velocity.linear.x = 0.0
                    self.velocity.angular.z = 0.0

                    print("\nStopping")
                    print("X = ", self.x)
                    print("Y = ", self.y)
                    print("Theta = ", self.theta)

                    self.publisher.publish(self.velocity)     
        
        rate.sleep()  

##############################################################


######### YOU ARE NOT ALLOWED TO MAKE CHANGES TO THIS PART #########
if __name__ == "__main__":
    try:
        print("------------------------------------------")
        print("         Python Script Started!!          ")
        print("------------------------------------------")
        main()

    except:
        print("------------------------------------------")
        traceback.print_exc(file=sys.stdout)
        print("------------------------------------------")
        sys.exit()

    finally:
        print("------------------------------------------")
        print("    Python Script Executed Successfully   ")
        print("------------------------------------------")
