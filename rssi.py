#!/usr/bin/env python

import rospy
import os
from std_msgs.msg import String

def get_rssi():
	interface = os.popen("iwconfig").read()
    	interface = interface.split('  ')[0]
	wifi = os.popen("sudo iwlist " +interface+ " scanning | egrep 'Address|Signal'").read() 
	return wifi

if __name__ == '__main__':
	rospy.init_node('rssi')
	rssi_pub=rospy.Publisher('rssi',String,queue_size=1)
	
	while not rospy.is_shutdown():
		rssi=get_rssi()
		print '%s' %rssi
		rssi_pub.publish(rssi)
