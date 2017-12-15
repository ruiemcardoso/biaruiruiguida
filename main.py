#!/usr/bin/env python

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from incicializ_particulas import *
from movement import *
from mapa import *
from rssi import *
import rospy
import time

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
	
	
	
def move_particles(cloud,d_x,d_y,orientation,particulas,rssi,mapa,edges,declive,declive_perp,b,new_rssi):
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
	
	if (abs(d_x)>0.01 or abs(d_y)>0.01 and new_rssi==1):
		#print "faz resampling"
		particulas=resampling(particulas,mapa,edges,declive,b)
		new_rssi=0
		
	for i in range (len(particulas)):
		cloud[i].position.x = particulas[i][2]
		cloud[i].position.y = particulas[i][3]
		#cloud[i].orientation.w += orientation.orientation.w
		#cloud[i].orientation.z += orientation.orientation.z

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

def talker(cloud,x,y,particulas,mapa,edges,declive,declive_perp,b,wifi,old_wifi):
	
	no_move=1	

	cloud_pub=rospy.Publisher('cloud_',PoseArray,queue_size=1)
	
	cloud_=cloud

	while not rospy.is_shutdown():
		cloud_pub.publish(cloud_)
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
			
		[d_x,d_y,x,y]=delta_odom(x,y)
		#if d_x ==0 and d_y ==0:
		orientation = Pose()
		orientation.orientation.z=0
		orientation.orientation.w=0
		no_move=0
 
		#if d_x !=0 or d_y !=0 and no_move != 1:
			#orientation=update_particle_orientation(d_x,d_y)
		
		particulas=move_particles(cloud.poses,d_x,d_y,orientation,particulas,rssi,mapa,edges,declive,declive_perp,b,new_rssi)
		
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
	talker(cloud,x,y,particulas,mapes,edges,declive,declive_perp,b,wifi,old_wifi)

if __name__ == '__main__':
	main()
