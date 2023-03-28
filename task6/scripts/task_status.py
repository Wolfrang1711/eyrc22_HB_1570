#!/usr/bin/env python

'''
* Team Id : < HB_1570 >
* Author List : < Pratik >
* Filename: < task_status.py >
* Theme: < HB -- Specific to eYRC / eYRCPlus >
* Functions: < __init__(self), on_press(self, key), on_release(self, key) >
* Global Variables: < None >
'''

import rospy 
import os
from pynput.keyboard import Listener, KeyCode
from std_msgs.msg import Int32

class taskstatus:

    '''
    * Function Name: < __init__ >
    * Input: < None >
    * Output: < None >
    * Logic: < initializing control loop to publish task status data >
    * Example Call: < called automatically by Operating System >
    ''' 

    def __init__(self):

        rospy.init_node('taskstatus', anonymous=True) 

        self.taskStatus = rospy.Publisher('taskStatus', Int32, queue_size=10)
        
        self.status = Int32()

        # storing a random value as status so as to avoid unecessary start of task
        self.status.data = 10

        rate = rospy.Rate(75)

        print("Task Status\nPress W : Start Task\nPress S : End Task")

        # starting listener to recieve data on keypress and keyrelease 
        listener = Listener(self.on_press, self.on_release, suppress=False)
        listener.start()

        while listener.running and not rospy.is_shutdown():
            
            # publishing updated task status
            self.taskStatus.publish(self.status)

            rate.sleep()
    
    '''
    * Function Name: < on_press >
    * Input: < key - data recieved from keyboard keypress in system >
    * Output: < None >
    * Logic: < recieves what key is pressed in keyboard and update the task status based on the nature of keypress >
    * Example Call: < called automatically by ROS Listener >
    ''' 

    def on_press(self, key):
        
        # if w keys is pressed in keyboard
        if key == KeyCode(char='w'):

            self.status.data = 0
            print("Task Started")

        # if s keys is pressed in keyboard
        elif key == KeyCode(char='s'):

            self.status.data = 1
            print("Task Ended")

    '''
    * Function Name: < on_release >
    * Input: < key - data recieved from keyboard keypress in system >
    * Output: < None >
    * Logic: < deletes the last line in terminal on key release >
    * Example Call: < called automatically by ROS Listener >
    ''' 

    def on_release(self, key): 

        # deletes last line of input in terminal
        os.system('tput dl1') 
        
if __name__ == "__main__":

	try:
		taskstatus()
	except rospy.ROSInterruptException as e:
		print(e)       