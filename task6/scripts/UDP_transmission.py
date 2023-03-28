#!/usr/bin/env python

'''
* Team Id : < HB_1570 >
* Author List : < Pratik >
* Filename: < UDP_transmission.py >
* Theme: < HB -- Specific to eYRC / eYRCPlus >
* Functions: < __init__(self), data_feedback(self, msg), pen_feedback(self, msg) >
* Global Variables: < None >
'''

import rospy
from std_msgs.msg import String, Int32      
import socket                               

class transmitter:

    '''
    * Function Name: < __init__ >
    * Input: < None >
    * Output: < None >
    * Logic: < initializing control loop to start UDP communication and fuse velocity and pen data and send it via UDP communication >
    * Example Call: < called automatically by Operating System >
    ''' 

    def __init__(self):
        
        rospy.init_node('UDP_transmission_node')

        rospy.Subscriber('velocity_data', String, self.data_feedback)
        rospy.Subscriber('penStatus', Int32, self.pen_feedback)
        
        # initializing variables
        self.velocity_data = ''
        self.pen_data = 0
        recieved_data = ''
        data_packet = ''

        # IP address and port of UDP network 
        UDP_IP = "192.168.50.1"
        UDP_port = 44444

        rate = rospy.Rate(75)

        # creating a datagram socket for UDP commmunication
        self.UDP_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)

        while not rospy.is_shutdown():
            
            # converting to string and joining pen status to data to be sent
            self.pen_data = str(self.pen_data)
            recieved_data = ",".join([self.velocity_data, self.pen_data])

            # encoding the recieved data
            data_packet = str.encode(recieved_data)

            # sending data packets via UDP communication
            self.UDP_socket.sendto(data_packet, (UDP_IP,UDP_port))
            print("Sending Data: ", data_packet)

            rate.sleep()

    '''
    * Function Name: < data_feedback >
    * Input: < msg - data recieved from ROS subscriber callback >
    * Output: < None >
    * Logic: < recieves wheel velocity data from /velocity_data topic >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 

    def data_feedback(self, msg):

        self.velocity_data = msg.data  

    '''
    * Function Name: < pen_feedback >
    * Input: < msg - data recieved from ROS subscriber callback >
    * Output: < None >
    * Logic: < recieves pen up/down status data from /penStatus topic >
    * Example Call: < called automatically by ROS subscriber callback >
    ''' 

    def pen_feedback(self, msg):

        self.pen_data = msg.data    

if __name__ == "__main__":
        
	try:
		transmitter()
	except rospy.ROSInterruptException as e:
		print(e)


