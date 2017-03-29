#!/usr/bin/env python 
# ----------------------------------------------
# Way Points Publisher for ROS navigation stack
# ----------------------------------------------
# This code has been written by: Abdulrahman Renawi
# Any use of this material is allowed ONLY
# with proper citation. Edits CANNOT be done
# before seeking a permission.
# for inquiries and permissions:
# Contact email: abdulrahman.renawi@gmail.com
# ----------------------------------------------
# MTR Lab, American University of Sharjah.
# All rights are reserved.
# ----------------------------------------------

import rospy
from functions import *

rospy.init_node('WayPoints', anonymous=False)
rate = rospy.Rate(100) # 100hz

myrobot = robot('')

waypoints = [[10,0,0],[10,4,-1.57079632679],[0,4,3.14159265359],[0,0,0]]



errorhisx = []
errorhisy = []
errorhisyaw = []
errorhisWayPoints = []

paths = []

error = []


while not rospy.is_shutdown():	#press (Ctrl C) to exit 

	pos=myrobot.getPosition()
	#print pos

	for i in range (0,4):
		myrobot.sendGoal(waypoints[i])
		#suscribe to path and save it to paths:
			#for element in path:
			#paths.append(element)
			
			
		while not (myrobot.getState()==3):
		
			if myrobot.getState()==2:
				print ("The Goal has been PREEMPTED!")
				break
		
			if myrobot.getState()==4:
				print ("The Goal has been ABORTED!")
				break
			
			if myrobot.getState()==5:
				print ("The Goal has been REJECTED!")
				break
			if myrobot.getState()==5:
				print ("The Goal has been RECALLED!")
				break
			if rospy.is_shutdown():
				break
			
		thiserror = waypoints[i] - myrobot.getPosition()
		#print thiserror
		error.append(thiserror)
		print ("The Goal No. %d has been accomplished with errors (x ,y ,yaw):" % (i+1) )
		print error[i]
				
				
		if rospy.is_shutdown():
			break												
		
	break
	
	
	
