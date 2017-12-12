#!/usr/bin/env python

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from incicializ_particulas import *
from movement import *
from mapa import *
import rospy
import os

def init_cloud():
	cloud=PoseArray()
	cloud.header.frame_id="map"
	return cloud

def init_particle(particulas):
	single_pose_msg=Pose()
	single_pose_msg.position.x=particulas[2]
	single_pose_msg.position.y=particulas[3]
	particle_w=particulas[1]
	cy=math.cos((particulas[4]*(2*math.pi))/(360)*0.5)
	sy=math.sin((particulas[4]*(2*math.pi))/(360)*0.5)
	cr=math.cos(0)
	sr=math.sin(0)
	cp=math.cos(0)
	sp=math.sin(0)

	angulo=(particulas[4]*(2*math.pi))/(360)

	single_pose_msg.orientation.z=sy*cr*cp-cy*sr*sp
	single_pose_msg.orientation.w=cy*cr*cp+sy*sr*sp

	return [single_pose_msg,angulo,particle_w]

def update_particle_orientation(d_x,d_y):
	particle=Pose()
	theta=math.atan2(d_y,d_x)
	print(theta)
	cy=math.cos(theta)
	sy=math.sin(theta)
	cr=math.cos(0)
	sr=math.sin(0)
	cp=math.cos(0)
	sp=math.sin(0)

	particle.orientation.z=sy*cr*cp-cy*sr*sp
	particle.orientation.w=cy*cr*cp+sy*sr*sp
	
	return particle
	
def offset(particula,edges):
	[a,b]=edges[particula[0]][0:2]
	xa=mapa[a][0][1]
	ya=mapa[a][0][2]
	xb=mapa[b][0][1]
	yb=mapa[b][0][2]
	
	offset=abs((yb-ya)*particula[2]-(xb-xa)*particula[3]+xb*ya-yb*xa)/(((yb-ya)**2+(xb-xa)**2)**(1/2))

	return offset	
	
def move_particles(cloud,d_x,d_y,orientation,particulas,rssi,mapa,edges,declive,declive_perp,b):
	i=0
	
	for i in range (len(particulas)):
		particulas[i][2] += random.normalvariate(0,0.05)+d_x
		particulas[i][3] += random.normalvariate(0,0.05)+d_y
		particulas[i][5] = offset(particula[i],edges)
		particulas[i][1] = perceptual_model(particulas[i],mapa,edges,rssi)
	
	particulas=constrain(particulas, edges, mapa, declive, declive_perp, b)	

	if (d_x>0.05 or d_y>0.05):
		particulas=resampling(particulas,mapa,edges,declive,b)
 

	for i in range (len(particulas)):
		
		cloud[i].position.x = particulas[i][2]
		cloud[i].position.y = particulas[i][3]
		#cloud[i].orientation.w += orientation.orientation.w
		#cloud[i].orientation.z += orientation.orientation.z

	return particulas
	
def talker(cloud,x,y,particulas,mapa,edges,declive,declive_perp,b):
	
	no_move=1	

	cloud_pub=rospy.Publisher('cloud_',PoseArray,queue_size=1)
	
	cloud_=cloud

	while not rospy.is_shutdown():
        	cloud_pub.publish(cloud_)
         	rssi=get_rssi()
		[d_x,d_y,x,y]=delta_odom(x,y)
		#if d_x ==0 and d_y ==0:
		orientation = Pose()
		orientation.orientation.z=0
		orientation.orientation.w=0
		no_move=0
 
		#if d_x !=0 or d_y !=0 and no_move != 1:
			#orientation=update_particle_orientation(d_x,d_y)
         	particulas=move_particles(cloud.poses,d_x,d_y,orientation,particulas,rssi,mapa,edges,declive,declive_perp,b)
         	rospy.sleep(1)


def main():
	i=0	
	
	cloud=init_cloud()
	[vertices,edges,mapes,declive,declive_perp,b]=mapa()
	particulas=particle_init(mapes,edges,declive,b)
	particle_w_vec=[None]*len(particulas)

	for i in range (len(particulas)):
		[single_pose_msg,angulo,particle_w]=init_particle(particulas[i])
		cloud.poses.append(single_pose_msg)
		particle_w_vec[i]=particle_w
		#angulos_list.append(angulo)
	
    	[x,y]=get_odom()
	#print('x: {}, y: {}'.format(x,y))
	talker(cloud,x,y,particulas,mapes,edges,declive,declive_perp,b)

if __name__ == '__main__':
	main()
