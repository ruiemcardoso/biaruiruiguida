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
	
	o_w = msg.pose.pose.orientation.w
	o_z = msg.pose.pose.orientation.z
	
	return [x,y,o_w,o_z]


def delta_odom(x_p,y_p):
	[x, y, o_w, o_z] = get_odom()
	d_x=x-x_p
	d_y=y-y_p

	return [d_x,d_y,x,y, o_w, o_z]
	

#def main():
#	rospy.init_node('movement')
#	[x,y]=get_odom()
#	#get_rssi()
#	print('x: {}, y: {}'.format(x,y))

#if __name__ == '__main__':
#	main()
