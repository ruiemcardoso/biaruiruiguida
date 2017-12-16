#!/usr/bin/env python

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from incicializ_particulas import *
from movement import *
from mapa import *
from rssi import *
import tf
import rospy
import time
from operator import itemgetter

def init_cloud():
	cloud=PoseArray()
	cloud.header.frame_id="map"
	return cloud
	
def init_w_part():
	w_part=Pose()
	return w_part

def init_particle(particulas):
	single_pose_msg=Pose()
	single_pose_msg.position.x=particulas[2]
	single_pose_msg.position.y=particulas[3]
	quat=tf.transformations.quaternion_from_euler(0,0,particulas[4])

	single_pose_msg.orientation.z=quat[2]
	single_pose_msg.orientation.w=quat[3]

	return single_pose_msg
	
def offset(particula,edges,mapa):
	[a,b]=edges[particula[0]][0:2]
	
	xa=mapa[a][0][1]
	ya=mapa[a][0][2]
	xb=mapa[b][0][1]
	yb=mapa[b][0][2]
	
	offset=abs((yb-ya)*particula[2]-(xb-xa)*particula[3]+xb*ya-yb*xa)/(((yb-ya)**2+(xb-xa)**2)**(1/2))

	return offset
	
def callback(data):
	wifi.append(data.data)
	
	
	
def move_particles(cloud,d_x,d_y,o_w,o_z,particulas,rssi,mapa,edges,declive,declive_perp,b,new_rssi):
	i=0
	
	for i in range (len(particulas)):
		#particulas[i][1] = 1.0/len(particulas)
         if d_x>0.01 and d_y>0.01:
             a=random.normalvariate(0,0.1)
         else:
              a=0
             
         particulas[i][2] += a+d_x
         particulas[i][3] += a+d_y
         particulas[i][5] = offset(particulas[i],edges,mapa)
         if new_rssi==1:
			particulas[i][1] = perceptual_model(particulas[i],mapa,edges,rssi)
	
	particulas=constrain(particulas, edges, mapa, declive, declive_perp, b)	
	particulas=sorted(particulas,key=itemgetter(1),reverse=True)
	#particulas[0][1]*=3
	wei_part=list(particulas[0])
 
	if (abs(d_x)>0.01 or abs(d_y)>0.01 and new_rssi==1):
		#print "faz resampling"
		particulas=resampling(particulas,mapa,edges,declive,b)
  
	for i in range(len(particulas)):
		if particulas[i][2]==wei_part[2] and particulas[i][3]==wei_part[3]:
			wei_part_temp=list(particulas[0])
			particulas[0]=list(wei_part)
			particulas[i]=list(wei_part_temp)
		new_rssi=0
		
	for i in range (len(particulas)):
		cloud[i].position.x = particulas[i][2]
		cloud[i].position.y = particulas[i][3]
		cloud[i].orientation.w = o_w
		cloud[i].orientation.z = o_z

	return particulas
	
def process_rssi(wifi):
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
	#print aux
	
 	return sorted(aux)

def talker(cloud,x,y,particulas,mapa,edges,declive,declive_perp,b,wifi,old_wifi,w_part):
	
	no_move=1	

	cloud_pub=rospy.Publisher('cloud_',PoseArray,queue_size=1)
	particle_pub=rospy.Publisher('w_particle',Pose,queue_size=1)
	w_part=init_particle(particulas[0])
	
	w_particle=w_part
	cloud_=cloud

	while not rospy.is_shutdown():
		cloud_pub.publish(cloud_)
		particle_pub.publish(w_particle)
		rospy.Subscriber('rssi',String,callback)
		new_wifi=wifi[len(wifi)-1]
		
		if new_wifi != old_wifi:
			print 'new'
			print new_wifi
			print '\n\n'
			print 'old'
			print old_wifi
			print '\n\n'
			new_rssi=1
			rssi=process_rssi(wifi[len(wifi)-1])
			print '\n entrou\n'
			old_wifi=new_wifi
			
		[d_x,d_y,x,y,o_w,o_z]=delta_odom(x,y)

		particulas=move_particles(cloud.poses,d_x,d_y,o_w,o_z,particulas,rssi,mapa,edges,declive,declive_perp,b,new_rssi)
		w_part.position.x=particulas[0][2]
		w_part.position.y=particulas[0][3]
		d_x=0
		d_y=0
		
        time.sleep(2)


def main():
	i=0	
	rospy.init_node('talker',anonymous=True)
	global wifi
	wifi=[]
	cenas=get_rssi()
	wifi.append(cenas)
	old_wifi='s'
	cloud=init_cloud()
	w_part=init_w_part()
	[vertices,edges,mapes,declive,declive_perp,b]=mapa()
	particulas=particle_init(mapes,edges,declive,b)
 

	for i in range (len(particulas)):
		single_pose_msg=init_particle(particulas[i])
		cloud.poses.append(single_pose_msg)
	
	[x,y,o_w,o_z]=get_odom()
	#print('x: {}, y: {}'.format(x,y))
	talker(cloud,x,y,particulas,mapes,edges,declive,declive_perp,b,wifi,old_wifi,w_part)

if __name__ == '__main__':
	main()
