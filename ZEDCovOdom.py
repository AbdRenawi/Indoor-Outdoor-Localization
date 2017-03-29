#!/usr/bin/env python 
# ----------------------------------------------
# MIDG II serial read and ROS Publisher node
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
from numpy import inf
from copy import copy
import roslib,tf,rospy
from time import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped 
global newpos,pub,pubrviz,newnewpos,prev_x,prev_y,prev_z,prev_roll,prev_pitch,prev_yaw,prev_dt,dt,tt

newpos=Odometry()    	#message to be published as ZEDCovOdom msg
#newpos=PoseWithCovarianceStamped()
newnewpos=PoseStamped()


def getpos(msg):	# define the odometry msg 
    global newpos,pub,pubrviz,newnewpos,prev_x,prev_y,prev_z,prev_roll,prev_pitch,prev_yaw,prev_dt,dt,tt
    newpos.header = msg.header			#republish ZED header
    newnewpos.header = msg.header			#republish ZED header
    newpos.child_frame_id = msg.child_frame_id	#republish ZED child_frame_id
     
    
    newpos.pose.pose.position.x = msg.pose.pose.position.x 		#republish pose
    newpos.pose.pose.position.y = msg.pose.pose.position.y 
    newpos.pose.pose.orientation = msg.pose.pose.orientation 
    
    
    newnewpos.pose = msg.pose.pose		#republish pose
    
    quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)  #obtaining quaternion
    (roll,pitch,yaw)=tf.transformations.euler_from_quaternion(quaternion)  # quaternion to roll,pitch,yaw conversion   
    
    #feading ZED covariances (x, y, z, roll, pithc, yaw):
    for i in range(36):
    	#newpos.pose.covariance[i] = 0.05
    	newpos.pose.covariance[0] = 0.05
    	newpos.pose.covariance[7] = 0.05
    	newpos.pose.covariance[14] = 0.5
    	newpos.pose.covariance[21] = 0.1
    	newpos.pose.covariance[28] = 0.1
    	newpos.pose.covariance[35] = 0.1
    
    
      #get one measurement
    #if msg.header.seq < 2:
    #	prev_x = msg.pose.pose.position.x
    #	prev_y = msg.pose.pose.position.y
    #	prev_z = msg.pose.pose.position.z
    #		
    #	prev_roll = roll
    #	prev_pitch = pitch
    # 	prev_yaw = yaw
    # 	
    #	prev_dt = time()
    #	
    #	newpos.twist = msg.twist			#republish ZED twist in first value
    #
   
    #else:
    #differentiation:
	#dt = time() - prev_dt    
    	#prev_dt = time()
    
    	
    	#newpos.twist.twist.linear.x = (msg.pose.pose.position.x - prev_x)/dt
    	#newpos.twist.twist.linear.y = (msg.pose.pose.position.y - prev_y)/dt
    	#newpos.twist.twist.linear.z = (msg.pose.pose.position.z - prev_z)/dt
    	#
    	#newpos.twist.twist.angular.x = (roll - prev_roll)/dt
    	#newpos.twist.twist.angular.y = (pitch - prev_pitch)/dt
    	#newpos.twist.twist.angular.z = (yaw - prev_yaw)/dt
    	  
    	
    	#for next iteration: 
    	#prev_x = msg.pose.pose.position.x
    	#prev_y = msg.pose.pose.position.y 
    	#prev_z = msg.pose.pose.position.z 
    	#prev_roll = roll
    	#prev_pitch = pitch
    	#prev_yaw = yaw
    	
    
    
    
    #Publishing:
    pub.publish(newpos)				#publish the new Odometry with Convariances
    pubrviz.publish(newnewpos)
    
  
    
          

pub = rospy.Publisher('ZEDCovOdom', Odometry, queue_size=10)  	#define the publisher parameters
pubrviz = rospy.Publisher('ZEDPose', PoseStamped, queue_size=10)  	#define the publisher parameters


#pub = rospy.Publisher('ZEDCovOdom', Odometry, queue_size=10)  	#define the Subsriber parameters

rospy.init_node('zedCovarianceAdder', anonymous=False)		#define the ROS node
rospy.Subscriber('/zed/odom',Odometry,getpos)			#define the Subsriber parameters




while not rospy.is_shutdown():	#press (Ctrl C) to exit 
	pass   
    
    

    
   
