#!/usr/bin/env python

# Team ID:		[ eYRC#HB#1570 ]
# Author List:	[ Pratik ]

import rospy 
import os
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import Int32

class taskstatus:

    def __init__(self):

        rospy.init_node('taskstatus', anonymous=True) 
        self.taskStatus = rospy.Publisher('taskStatus', Int32, queue_size=10)
        
        self.status = Int32()

        self.status.data = 10

        rate = rospy.Rate(75)

        print("Task Status\nPress W : Start Task\nPress S : End Task")
 
        listener = Listener(self.on_press, self.on_release, suppress=False)
        listener.start()

        while listener.running and not rospy.is_shutdown():

            self.taskStatus.publish(self.status)
            rate.sleep()

    def on_press(self, key):

        if key == KeyCode(char='w'):

            self.status.data = 0
            print("Task Started")

        elif key == KeyCode(char='s'):

            self.status.data = 1
            print("Task Ended")

    def on_release(self, key): 

        os.system('tput dl1') 
        
if __name__ == "__main__":

	try:
		taskstatus()
	except rospy.ROSInterruptException as e:
		print(e)       