#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
	
	return data.data

def listener():
	rospy.init_node('listener',anonymous=True)
	cenas=rospy.Subscriber("rssi",String,callback)
	print("I heard %s" %cenas.data)
	rospy.spin()

if __name__ == '__main__':
	
	listener()
