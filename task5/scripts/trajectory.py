#!/usr/bin/env python

# Team ID:		    [ eYRC#HB#1570 ]
# Author List:		[ Romala ]

import cv2
import numpy as np
import rospy 	
from std_msgs.msg import String	
import math	                                        # Message type used for send velocity data
import argparse	

class Contours:

    def __init__(self):

        rospy.init_node('contours_node')

        self.contourPub = rospy.Publisher('contours', String, queue_size=10)

        parser = argparse.ArgumentParser(description="Select Mode: Function Mode or Image Mode")
        parser.add_argument("-f", "--mode", choices=["1", "2"], help="Select which mode to run")
        parser.add_argument("-r", "--resolution", help="Select resolution of image")

        self.args = parser.parse_args()

        self.cData = String()  

        self.frame = None
        self.n_list = []
        self.contour_ex_list, self.operating_list = [], []
        self.tempx, self.tempy, self.tempth = [], [], []
        self.xList , self.yList , self.thList, self.xListFinal , self.yListFinal , self.thListFinal, self.x, self.y, self.th= [] , [] , [] , [] , [] , [] , [] , [] , []      

        self.resolution = int(self.args.resolution)

        rate = rospy.Rate(75)

        if self.args.mode == "1":

            self.function_mode()
            print('starting function mode')

        elif self.args.mode == "2":

            self.img_mode()
            
            print('starting image mode')

        else:
            exit()
    
        while not rospy.is_shutdown():
            
            self.cData.data = str([self.xListFinal,self.yListFinal,self.thListFinal])

            rospy.loginfo("Publishing contours")

            # cv2.imshow('img', self.frame1)
            # cv2.waitKey(1)
            
            self.contourPub.publish(self.cData) 

            rate.sleep()          
        
    def img_mode(self):

        self.image = cv2.imread('/home/pratik/eyrc_ws/src/eyrc22_HB_1570/task5/scripts/snapchat.png')

        self.frame = cv2.resize(self.image, (500,500), interpolation=cv2.INTER_LINEAR)

        self.frame1 = self.frame.copy()

        self.img_gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        self.thresh = cv2.threshold(self.img_gray, 45, 255, cv2.THRESH_BINARY)[1]
        self.thresh = cv2.erode(self.thresh, None, iterations=2)
        self.thresh = cv2.dilate(self.thresh, None, iterations=2)

        contours, hierarchy = cv2.findContours(image=self.thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

        # for hierarchy
        self.h_list = hierarchy.tolist()

        for i in range(len(self.h_list[0])):
            x = self.h_list[0][i]
            self.contour_ex_list.append(x)

        mat = [item[3] for item in self.contour_ex_list]  
  

        for i in range(len(mat)):
             if mat[i]>0:
                  self.n_list.append(i)   

        #  for contours
        for i in range(len(self.n_list)):

            cnt = contours[self.n_list[i]]

            self.operating_list.append(cnt)
            
            tuple(self.operating_list) 

            cv2.drawContours(self.frame1, self.operating_list, -1, (0,255,0), 2)
        

        for i in self.operating_list: 

            self.xList.clear()    
            self.yList.clear()
            self.thList.clear()
            
            for j in i:

                    self.xList.append(j[0][0])
                    self.yList.append(j[0][1])
                    self.thList.append(0)

            self.x.append(str(self.xList))
            self.y.append(str(self.yList))   
            self.th.append(str(self.thList))   

        for i in range(len(self.x)):
            
            self.x[i] = eval(self.x[i])
            self.y[i] = eval(self.y[i])
            self.th[i] = eval(self.th[i])

        # print(self.x, self.y)    
    
        for i in range(len(self.x)):
          
            self.tempx.append(self.x[i][len(self.x[i])-1])
            self.tempy.append(self.y[i][len(self.y[i])-1])
            self.tempth.append(self.th[i][len(self.th[i])-1])

            # del self.x[i][len(self.x[i])-1]
            # del self.y[i][len(self.y[i])-1]
            # del self.th[i][len(self.th[i])-1]
             
        self.xListFinal = [sublist[::self.resolution] for sublist in self.x]
        self.yListFinal = [sublist[::self.resolution] for sublist in self.y]
        self.thListFinal = [sublist[::self.resolution] for sublist in self.th]

        # print(self.tempx, self.tempy)

        for i in range(len(self.tempx)):
         
            self.xListFinal[i].append(self.tempx[i])
            self.yListFinal[i].append(self.tempy[i])
            self.thListFinal[i].append(self.tempth[i])  

        # print(len(self.xListFinal[0]), len(self.yListFinal[0]))    

             
    def function_mode(self):

        self.xListFinal.clear()    
        self.yListFinal.clear()
        self.thListFinal.clear()

        scale_factor = 1.5

        for count in range(self.resolution):

            t =  count*((2*math.pi)/self.resolution)

            # defining eqn
            x_eqn = 250*math.cos(t) 
            y_eqn = 125*math.sin(2*t)
            theta_eqn = (math.pi/4)*math.sin(t)

            x_eqn = int(x_eqn/scale_factor + 250)
            y_eqn = int(250 - y_eqn/scale_factor)

            self.xList.append(x_eqn)
            self.yList.append(y_eqn)
            self.thList.append(round(theta_eqn,3))  

        self.xListFinal.append(self.xList)
        self.yListFinal.append(self.yList)
        self.thListFinal.append(self.thList)    

if __name__ == '__main__':

	try:
		Contours()
	except rospy.ROSInterruptException as e:
		print(e)
  









