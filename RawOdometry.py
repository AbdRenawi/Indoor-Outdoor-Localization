#!/usr/bin/env python 
# ----------------------------------------------
# Raw Odometry Generator from Kobuki ticks msg
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

import rospy,math,time,copy,tf
from kobuki_msgs.msg import SensorState
from nav_msgs.msg import Odometry

#global LeftTick,RightTick,Ltick,Rtick,Theta,X,Y,OldTime
seq_ID = 0

PrevLTick = 0
PrevRTick = 0

Ltick = 0
Rtick = 0

LeftRevs = 0
RightRevs = 0

LeftDist = 0
RightDist = 0

Theta = 0
X = 0
Y = 0

oldright = 0
oldleft = 0

MultiplierL = 0
MultiplierR = 0

#global OldTime
global state
state = SensorState()
OldTime = time.time()

pos = Odometry()

def getstate(msg):
	global state
	#global LeftTick,RightTick,Ltick,Rtick,Theta,X,Y,OldTime
	state = msg
	
rospy.init_node('RawOdometry', anonymous=False)			#define the ROS node
rate = rospy.Rate(50) # 100Hz node communication spee
rospy.Subscriber('/mobile_base/sensors/core',SensorState,getstate)	#define the Subsriber parameters
pub = rospy.Publisher('raw_odometry', Odometry, queue_size=10)     #  velocity publisher definition

while not rospy.is_shutdown():	#press (Ctrl C) to exit 
	seq_ID+=1 #sequence_id increment
	current_time = rospy.Time.now()
	
	Ltick = state.left_encoder + 65535*MultiplierL
	Rtick = state.right_encoder + 65535*MultiplierR

	if (Ltick - PrevLTick > 6000):	#Building a continuous reading from the encoders:
		Ltick -= 65535
		MultiplierL -= 1
		#PrevLTick += 65535
	elif (Ltick - PrevLTick < -6000):
		Ltick += 65535
		MultiplierL += 1	
		#PrevLTick -= 65535
	if (Rtick - PrevRTick > 6000):
		Rtick -= 65535
		MultiplierR -= 1
		#PrevRTick += 65535
	elif (Rtick - PrevRTick < -6000):
		Rtick += 65535
		MultiplierR += 1
		#PrevRTick -= 65535
	
	#print "Ltick " ,Ltick, " - PrevLTick " ,PrevLTick," ---- Rtick " ,Rtick," - PrevRTick " ,PrevRTick
	
	#dt = time.time() - OldTime 				#Differentiate ticks to get angular Velocities
	#Vleft = ((Ltick - PrevLTick)/dt) * (math.pi/180.0)		#Left Velocity rad/s
	#Vright = ((Rtick - PrevRTick)/dt) * (math.pi/180.0)	#Right Velocity rad/s
	#OldTime = time.time()		
	
	#Vleft*=0.035			#Left Velocity in m/s Wheels radius = 35mm
	#Vright*=0.035			#Right Velocity in m/s
	
	#V = (Vright+Vleft)/2		#Average Robot Linear Speed
	#Omega = (Vright-Vleft)/0.230	#Average Robot Angular Speed L = 230mm
	
	#if V!=0:
	#	print V
	#	print Ltick
	#	print PrevLTick
	
	#PrevLTick = copy.copy(Ltick)			#Saving continuous ticks
	#PrevRTick = copy.copy(Rtick)			

	########## Runge-Kutta Numerical Integrator ##########
	#K00 = V*math.cos(Theta)
	#K01 = V*math.sin(Theta)
	#K02 = Omega
	#K10 = V*math.cos(Theta+dt*K02/2)
	#K11 = V*math.sin(Theta+dt*K02/2)
	#K12 = K02
	#K20 = K10
	#K21 = K11
	#K22 = K02
	#K30 = V*math.cos(Theta+dt*K02)
	#K31 = V*math.sin(Theta+dt*K02)
	#K32 = K02
	
	#X = X + (dt/6)*(K00 + 2*(K10+K20) + K30)
	#Y = Y + (dt/6)*(K01 + 2*(K11+K21) + K31)
	#Theta = Theta + (dt/6)*(K02 + 2*(K12+K22) + K32)
	
	
	######### Using Method from Siegwart Book ###############
	#LeftRevs = PrevLTick/2578.33		#Computing Revolutions 2578.33 ticks/wheel revolution
	#RightRevs = PrevRTick/2578.33	
	
	LeftDist = (Ltick/11.7)*10**-3	#Computing Distances 11.7 ticks/mm and converting to meters
	RightDist = (Rtick/11.7)*10**-3

	delta_left = LeftDist - oldleft
	delta_right = RightDist - oldright
	if delta_left>1 or delta_left<-1 or delta_right>1 or delta_right <-1:	
		print "WARNING!!""delta_left: ",delta_left,"delta_right: ",delta_right
	
	delta_s = (delta_left + delta_right)/2
	delta_theta = (delta_right - delta_left)/0.230
	delta_x = delta_s*math.cos(Theta+(delta_theta)/2.0)
	delta_y = delta_s*math.sin(Theta+(delta_theta)/2.0)
	
	X = X + delta_x
	Y = Y + delta_y
	Theta = Theta + delta_theta
	
	######
	PrevLTick = copy.copy(Ltick)			#Saving continuous ticks
	PrevRTick = copy.copy(Rtick)
	
	oldtheta = copy.copy(Theta)
	oldleft = copy.copy(LeftDist)
	oldright = copy.copy(RightDist)
	######################################################
	
	pos.header.frame_id = "odom"
	pos.header.seq = seq_ID
	pos.header.stamp = current_time
	pos.pose.pose.position.x = X
	pos.pose.pose.position.y = Y
	
	(qx,qy,qz,qw) = tf.transformations.quaternion_from_euler(0, 0, Theta)
	pos.pose.pose.orientation.x = qx
	pos.pose.pose.orientation.y = qy
	pos.pose.pose.orientation.z = qz
	pos.pose.pose.orientation.w = qw

	pub.publish(pos)
	
	#print 'loc. X:%.4f'% X,'-- Y:%.4f'% Y,'-- yaw:%.4f' % Theta
	
	rate.sleep()  #forced delay (if needed)
	
	
	
	
	
	
	
