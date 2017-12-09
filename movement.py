#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import time


#def callback(msg):
#	x = msg.pose.pose.position.x
#	y = msg.pose.pose.position.y	

def get_odom():
	rospy.init_node('talker',anonymous=True)
	msg = rospy.wait_for_message("/RosAria/pose",Odometry,timeout=10)
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	
	return [x,y]

def get_rssi():
	interface = os.popen("iwconfig").read()
    	interface = interface.split('  ')[0]
	wifi = os.popen("sudo iwlist " +interface+ " scanning | egrep 'Address|Signal'").read()
	wifi=wifi.split()
	 
	i=0
	aux=[]
	aux1=[]
	j=0
	for i in range(len(wifi)):
		if wifi[i] == 'Address:':
               	        aux1.append(wifi[i+1])
			j+=1
               	if wifi[i][0:5] == 'level':
			aux1.append(float(wifi[i][-3:len(wifi[i])]))
			j+=1
		if j==2:
			aux.append(aux1)
			aux1=[]
			j=0
	print aux
	
 	return aux

	#time.sleep(2)

def delta_odom(x_p,y_p):
	[x, y] = get_odom()
	d_x=x-x_p
	d_y=y-y_p

	return [d_x,d_y,x,y]
	

#def main():
#	rospy.init_node('movement')
#	[x,y]=get_odom()
#	#get_rssi()
#	print('x: {}, y: {}'.format(x,y))

#if __name__ == '__main__':
#	main()
