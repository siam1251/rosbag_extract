#!/usr/bin/env python
"""
Sayem Mohammad Siam
December 20,2015
This programme subscribes different topics and write them into 
file synchronously
Input: You need to give the file name as argument where it will save all the topics
By default it will save in this path = "/home/siam/Dataset-UACampus/"+file_name

It will raise error if path doesn't exist
"""
import rospy
import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import std_msgs.msg as stdmsg
from nav_msgs.msg import Odometry
import sensor_msgs.msg as smsg
import geometry_msgs.msg as gmsg
from tf.transformations import euler_from_quaternion
from numpy.linalg import inv
import random
import math
import atexit
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#fig, ax = plt.subplots()
fig = plt.figure()
ax2 = fig.add_subplot(224, autoscale_on=True)
img_ax3 = fig.add_subplot(223, autoscale_on=True)
#img_ax3.set_xticks([]), img_ax3.set_yticks([])  # to hide tick values on X and Y axis

ax2.grid()

x1 = []*0       # x-array
y1 = []*0
ax1 = fig.add_subplot(211, autoscale_on=True)
ax1.grid()
w_range = 400
ax1.set_ylim([-w_range,w_range])
ax1.set_xlim([-w_range,w_range])

x = []*0
y = []*0
#odometry red color
line1, = ax1.plot(x1,y1,'r',lw=1, label="u1")
#filtered odom blue color
line3, = ax1.plot(x,y,'b',lw=1, label="u1")
x4 = []*0
y4 = []*0
#gps odom green color
line4, = ax1.plot(x,y,'g',lw=1, label="o/g")

x2 = []
y2 = []

line2, = ax2.plot(x2,y2,'g',lw=1, label="ekf")
#ax.axis("equal")


ax2.set_ylim([5933825-w_range,5933825 + w_range])
ax2.set_xlim([332550-w_range,332550+w_range])
#ax.spines['left'].set_position('center')

odom_data = Odometry()
def odometry_update(data):
	print"s"
def animate(i):
	global odom_data
	global x, y
	
	
	return line1
def animate2(i):
	global line2
	return line2
#Init only required for blitting to give a clean slate.



def save_image(data):
	global count, cnt
	cnt = cnt + 1
	if cnt%2 == 0:
		bridge = CvBridge()
		try:
		  cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError, e:
		  print e

		cv2.imwrite(path+ str('%04d'%(count))+'.jpg', cv_image)
		open(path+str('%04d'%(count))+'.txt','w').write('%s'%wheel_odom_data.pose.pose)
		open(path+gps_filtered, 'a').write(' %f %f %f\n' %( filtered_odom_x, filtered_odom_y, imu_theta))
		open(path+gps_odom, 'a').write(' %f %f %f\n' %( gps_odom_x, gps_odom_y, imu_theta))
		open(path+wheel_odom, 'a').write(' %f %f %f\n' %( wheel_odom_x, wheel_odom_y, wheel_th))
		open(path+wheel_odom_constraint, 'a').write(' %f %f %f\n' %( r, dx, dy))
		count = count + 1
		img_ax3.imshow(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
		
		# cv2.imshow("Image window", cv_image)
	#cv2.waitKey(3)


def update_gps_odom(data):
	global gps_odom_y, gps_odom_x
	odom_data = data
	if odom_data.pose.pose.position.x != 0 and odom_data.pose.pose.position.x != 0:
		x2.append(odom_data.pose.pose.position.x)
		y2.append(odom_data.pose.pose.position.y)
	#print odom_data.pose.pose.position.x
	#print odom_data.pose.pose.position.y
	gps_odom_x = data.pose.pose.position.x
	gps_odom_y = data.pose.pose.position.y
	line2.set_data(x2,y2)
	#gps_odom_file.write('%f %f\n' %(odom_data.pose.pose.position.x, odom_data.pose.pose.position.x))
	# global x, y
	# x.append(odom_data.pose.pose.position.x)
	# y.append(odom_data.pose.pose.position.y)
	
	# print odom_data.pose.pose.position.x
	# print odom_data.pose.pose.position.y
	# line.set_data(x,y)
	# print "s"

def update_imu(data):
	global imu_theta
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header.frame_id)
	euler = euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
	updateImu = True
	imu = euler[2]
	imu_cov = data.orientation_covariance[-1]
	imu_theta = imu#+2.5;
	#file.write('%f \n' %(imu))
i = 0
prev_pose = [0, 0, 0]
prev_pose_corrected = [0, 0, 0]
prev_dt = np.zeros(5)
def update_wheel_odom(data):

	global wheel_odom_x, wheel_odom_y, i, prev_pose, prev_pose_corrected, wheel_th, dx, dy, r
	global wheel_odom_data
	wheel_odom_data = data
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	# read orientation expressed in quaternion and compute Euler angle th_z 
	q = [0,0,0,1]
	q[0] = data.pose.pose.orientation.x
	q[1] = data.pose.pose.orientation.y
	q[2] = data.pose.pose.orientation.z
	q[3] = data.pose.pose.orientation.w

	# for how this equation works, refer to 
	# en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	# we deal with phi in the special case of q0 = q1 = 0

	th = math.atan2(2*q[2]*q[3], 1 - 2*q[2]*q[2])
	#th = imu_theta

	if i == 0:
	  # initialize intermeidate and historic pose variables
	  #pose = [x, y, th]
	  prev_pose = [x, y, th]
	  #prev_pose_corrected = [x, y, th]
	else:
	  # compute incremental changes in x, y and theta in world frame  
	  dxw = (x - prev_pose[0])
	  dyw = y - prev_pose[1]
	  dt = th - prev_pose[2]
	  b = 0.70
	  dxw = dxw
	  dyw = dyw
	  #dt = .8*d
	  
	  # update "pose" for next iteration
	  prev_pose = [x, y, th] 
	  # make sure the angular change doesn't jump
	  if dt > np.pi:
		dt -= 2*np.pi
	  elif dt < -np.pi:
		dt += 2*np.pi
	  dt = dt*.5
	  # if dt < 0:
	  # 	dt = .8*dt
	  # else:
	  # 	dt = 0.6*dt
	  #dt = .5*dt

	  # for i in range(len(prev_dt)-1,0,-1):
	  # 	prev_dt[i] = prev_dt[i-1]
	  # prev_dt[0] = dt
	  # if i > len(prev_dt):
	  # 	dt = np.mean(prev_dt)

	  # !!! this is robot-specific: our robot wheel odometry is not correct
	  # 0.47 works but any number close should be fine
	  
	 #  if dt > np.pi/2:
		# dt = np.pi/2
	 #  elif dt < -np.pi/2:
		# dt = -np.pi/2
	  # calculate linear change in x of local frame
	  #
	  dy = 0 		
	  # calculate linear change in x of local frame
	  r = math.sqrt(dxw*dxw + dyw*dyw)
	  dx = r*math.cos(dt)
	  dy = r*math.sin(dt) 


	  # finally, produce the correct pose
	  th = prev_pose_corrected[2] + dt 
	  x = prev_pose_corrected[0] + r*math.cos(th) 
	  y = prev_pose_corrected[1] + r*math.sin(th) 
	  #
	  prev_pose_corrected =[x, y, th]
	  

	
	
	#print odom_data.pose.pose.position.x
	#print odom_data.pose.pose.position.y
	i = i+1
	wheel_odom_x = x
	wheel_odom_y = y
	wheel_th = th
	x1.append(x)
	y1.append(y)
	line1.set_data(x1,y1)




def update_filtered_odom(data):
	global cur, count, prev
	global x, y,line3,filtered_odom_x, filtered_odom_y
	cur = count;
	x.append(data.pose.pose.position.x)
	y.append(data.pose.pose.position.y)
	filtered_odom_x = data.pose.pose.position.x;
	filtered_odom_y = data.pose.pose.position.y;
	prev = cur
	line3.set_data(x,y)

def update_odom_gps(data):

	global line3, x4, y4
	x4.append(data.pose.pose.position.x)
	y4.append(data.pose.pose.position.y)
	#print odom_data.pose.pose.position.x
	#print odom_data.pose.pose.position.y
	
	line4.set_data(x4,y4)
	

def update_control(data):
	global v
	tmp_x  = data.linear.x
	tmp_y  = data.linear.y
	v = math.sqrt(tmp_x*tmp_x + tmp_y*tmp_y)
def ekf_output(data):
	global x2, y2
	print data.x
	print data.y
	x2.append(data.x)
	y2.append(data.y)
	line2.set_data(x2, y2)
	#gps_filtered_file.write('%f %f\n' %(data.x, data.y))
def subscribe():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("/imu/data", smsg.Imu, update_imu)
	rospy.Subscriber("/odom1", Odometry, update_gps_odom) # gps data is converted to odometry 
	rospy.Subscriber("/husky_velocity_controller/odom", Odometry, update_wheel_odom)
	rospy.Subscriber("/odometry/filtered", Odometry, update_filtered_odom)
	rospy.Subscriber("/odometry/gps", Odometry, update_odom_gps)
	rospy.Subscriber("/husky_velocity_controller/cmd_vel", gmsg.Twist, update_control)
	rospy.Subscriber("/ekf", gmsg.Point, ekf_output)
	rospy.Subscriber("/camera/rgb/image_raw",Image,save_image)
	
#parameters
path = ""
imu_theta = 0
odom_x = 0
odom_y = 0
gps_filtered = "gps_filtered.txt"
gps_filtered_pose_constraint = 'gps_filtered_pose_constraint.txt'
filtered_odom_x = 0;
filtered_odom_y = 0;
wheel_odom_data = Odometry()
wheel_odom = 'wheel_odom.txt'
wheel_odom_constraint = 'wheel_odom_constraint.txt'
wheel_odom_y = 0
wheel_odom_x = 0
wheel_th = 0
r = 0
dx = 0
dy = 0
gps_odom = 'gps_odom.txt'
gps_odom_x = 0
gps_odom_y = 0
count = 1
cnt = 1





if __name__ == '__main__':
	subscribe()
	path = path +sys.argv[1] + '/'
	if not os.path.exists(path):
		os.makedirs(path)
	print path
	gps_odom_file = open(path+gps_odom, 'w')
	gps_filtered_file = open(path+ gps_filtered, 'w')
	wheel_odom_file = open(path+ wheel_odom, 'w')
	wheel_odom_constraint_file = open(path+wheel_odom_constraint, 'w')
	wheel_odom_constraint_file.close()
	wheel_odom_file.close()
	gps_filtered_file.close()
	gps_odom_file.close()
	
	simulation = animation.FuncAnimation(fig,animate, blit=False, interval=20, repeat=True)
	#simulation = animation.FuncAnimation(fig, animate2, blit=False, frames=2000, interval=20, repeat=True)
	#ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
	#    interval=25, blit=True)
	plt.show()
	rospy.spin() 
