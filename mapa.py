# -*- coding: utf-8 -*-
"""
Created on Thu Oct 26 16:18:33 2017

@author: student

"""

#rosrun stage_ros stageros $(rospack find stage)/worlds/pioneer_walle.world
# rostopic echo  /robot_0/odom > try1.txt

#para correr o programa pelo terminal
#file = raw_input("Nome do ficheiro")
#F=open(file, 'r')


from media import *

def mapa():
	F=open("/home/rui/biaruiruiguida/1corr.txt","r")
	m=[F.readlines()]
	F.close()
	
	Q=open("/home/rui/biaruiruiguida/1corrwifi.txt","r")
	w=[Q.readlines()]
	Q.close()
	
	
	#As medições da odometria são guardadas na lista moves
	lastx=0
	lasty=0
	timestamp=0
	index=3
	vertices = []
	edges = []
	larg_corredor = 0.1
	index_vert=0
	
	while (index < len(m[0])):
	    if( timestamp < int(m[0][index][10:-1])):   #deteção de um novo timestamp
		timestamp = int(m[0][index][10:-1])
		
		existing = False
		#obtenção dos movimentos significativos
		if abs(lastx-float(m[0][index+7][9:-1])) >= larg_corredor or abs(lasty-float(m[0][index+8][9:-1])) >= larg_corredor:
		    lastx=float(m[0][index+7][9:-1])
		    lasty=float(m[0][index+8][9:-1])
	
		    #incialza a lista de vertices           
		    if index_vert==0:
		         vertices.append([timestamp, float(m[0][index+7][9:-1]), float(m[0][index+8][9:-1])])
	
		         index_vert +=1
		         last_vert=0
		    
		    else:
		    #verifica se coordenadas sao repetidas
		        j=0
		        while j<len(vertices):
		            if abs(lastx-vertices[j][1])<= larg_corredor and abs(lasty-vertices[j][2]) <= larg_corredor:
		                #é repetido
		                existing = True
		                rep = j
		                break
		            j += 1
		            
		                
		        if existing == False:
		            new_vertice = [timestamp, lastx, lasty]
		            vertices.append(new_vertice)
		            #if ([index_vert, index_vert] in edges) == False:
		                #edges.append([index_vert, index_vert])
		            if ([index_vert, last_vert] in edges) == False and ([last_vert, index_vert] in edges) == False:     
		                edges.append([index_vert, last_vert])
		                last_vert = index_vert
		                index_vert +=1
		     
		        
		    
		        else : 
		            #verifica se esta ligação ja existe
		            if ([last_vert, rep] in edges) == False and ([rep, last_vert] in edges) == False:
		                edges.append([last_vert, rep])
		                last_vert=rep
		                index_vert = len(vertices)
		         
		         
		         
		         
		         
	    index += 31
	
	
	
	wifi=[]
	aux=[]
	i=0
	timestamp=0
	x0=vertices[0][1]
	y0=vertices[0][2]
	
	while i < len(w[0]):
	    
	    if w[0][i][0:2] == '15':
		if timestamp < int(w[0][i][0:10]):
		    aux.append(int(w[0][i][0:10]))
		    timestamp=int(w[0][i][0:10])
		    i+=1
		    
		    while i<len(w[0]) and w[0][i][0:2] != '15':
		        
		        if w[0][i][20:27] == 'Address':
		            aux.append(w[0][i][29:46])
		            aux.append(w[0][i+1][48:51])
		        i+=2
	    
		wifi.append(aux)
		aux=[]
		i-=1
	    i +=1
	    
	    
	 
	
	
	
	
	mapa= []
	i=0
	flag=0
	timestamp=0
	aux1=[]
	aux2=[]
	aux3=[]
	aux4=[]
	aux5=[]
	aux6=[] 
	average=[]
	a=0
	while i<len(wifi):
	    j=0
	    aux= []
	    timestamp = wifi[i][0]
	
	    #recolha do timestamp e vertices
	    while i<len(wifi) and j<len(vertices) and timestamp >= vertices[j][0] :
		if timestamp == vertices[j][0] and len(aux)==0:
		    aux.append(timestamp)
		    aux.append(vertices[j][1])
		    aux.append(vertices[j][2])
		    flag=1
		    
		    #recolha dos aps e potências
		    a=2
		    while timestamp == vertices[j][0] and i<len(wifi) and len(wifi[i])>1 and a<=max(len(wifi[i-2]),len(wifi[i-1]),len(wifi[i]),len(wifi[i+1]),len(wifi[i+2])):
	
		        if (i+2) < len(wifi) and i>1:
		            while a<=max(len(wifi[i-2]),len(wifi[i-1]),len(wifi[i]),len(wifi[i+1]),len(wifi[i+2])):
		                
		                if a<len(wifi[i-2]):
		                    aux1.append([wifi[i-2][a-1],wifi[i-2][a]])
		                
		                if a<len(wifi[i-1]):    
		                    aux2.append([wifi[i-1][a-1],wifi[i-1][a]])
		                
		                if a<len(wifi[i]):
		                    aux3.append([wifi[i][a-1],wifi[i][a]])
		                
		                if a<len(wifi[i+1]):   
		                    aux4.append([wifi[i+1][a-1],wifi[i+1][a]])
		                
		                if a<len(wifi[i+2]):
		                    aux5.append([wifi[i+2][a-1],wifi[i+2][a]])    
		                a+=2
		                
		            aux6.append(sorted(aux1))
		            aux6.append(sorted(aux2))
		            aux6.append(sorted(aux3))
		            aux6.append(sorted(aux4))
		            aux6.append(sorted(aux5))  
		            
		            average=med_dev5(aux6)    
		    
		        elif i==0:    #primeiro timestamp - comparar SO com os dois T's seguintes
		            while a<=max(len(wifi[i]),len(wifi[i+1]),len(wifi[i+2])):
		                if a<len(wifi[i]):
		                    aux3.append([wifi[i][a-1],wifi[i][a]])
		                
		                if a<len(wifi[i+1]):   
		                    aux4.append([wifi[i+1][a-1],wifi[i+1][a]])
		                
		                if a<len(wifi[i+2]):
		                    aux5.append([wifi[i+2][a-1],wifi[i+2][a]])    
		                a+=2
		                
		            aux6.append(sorted(aux3))
		            aux6.append(sorted(aux4))
		            aux6.append(sorted(aux5))
		            average=med_dev3(aux6) 
		            print('primeiro')
		            
		        elif i==1:    #segundo timestamp - comparar com os dois T's seguintes e com o T anterior
		            while a<=max(len(wifi[i-1]),len(wifi[i]),len(wifi[i+1]),len(wifi[i+2])):
		                if a<len(wifi[i-1]):    
		                    aux2.append([wifi[i-1][a-1],wifi[i-1][a]])
		                
		                if a<len(wifi[i]):
		                    aux3.append([wifi[i][a-1],wifi[i][a]])
		                
		                if a<len(wifi[i+1]):   
		                    aux4.append([wifi[i+1][a-1],wifi[i+1][a]])
		                
		                if a<len(wifi[i+2]):
		                    aux5.append([wifi[i+2][a-1],wifi[i+2][a]])    
		                a+=2
		                
		            aux6.append(sorted(aux2))
		            aux6.append(sorted(aux3))
		            aux6.append(sorted(aux4))
		            aux6.append(sorted(aux5))
		            average=med_dev4(aux6) 
		            print('segundo')
		            
		        elif (i+2) == len(wifi):  #penultimo timestamp - comparar com os 2 T's anteriores e o T seguinte
		            while a<=max(len(wifi[i-2]),len(wifi[i-1]),len(wifi[i]),len(wifi[i+1])):
		                
		                if a<len(wifi[i-2]):
		                    aux1.append([wifi[i-2][a-1],wifi[i-2][a]])
		                
		                if a<len(wifi[i-1]):    
		                    aux2.append([wifi[i-1][a-1],wifi[i-1][a]])
		                
		                if a<len(wifi[i]):
		                    aux3.append([wifi[i][a-1],wifi[i][a]])
		                
		                if a<len(wifi[i+1]):   
		                    aux4.append([wifi[i+1][a-1],wifi[i+1][a]])
		                   
		                a+=2
		                
		            aux6.append(sorted(aux1))
		            aux6.append(sorted(aux2))
		            aux6.append(sorted(aux3))
		            aux6.append(sorted(aux4))
		            average=med_dev4(aux6) 
		            print('penultimo')
		         
		        elif (i+1) == len(wifi):  #ultimo timestamp - comparar SO com os 2 T's anteriores
		            while a<=max(len(wifi[i-2]),len(wifi[i-1]),len(wifi[i])):
		                
		                if a<len(wifi[i-2]):
		                    aux1.append([wifi[i-2][a-1],wifi[i-2][a]])
		                
		                if a<len(wifi[i-1]):    
		                    aux2.append([wifi[i-1][a-1],wifi[i-1][a]])
		                
		                if a<len(wifi[i]):
		                    aux3.append([wifi[i][a-1],wifi[i][a]])
		                
		                a+=2
		                
		            aux6.append(sorted(aux1))
		            aux6.append(sorted(aux2))
		            aux6.append(sorted(aux3))
		            average=med_dev3(aux6) 
		            print('ultimo')
		            
		         
		    
		else:
		    j+=1
		    if flag ==1:
		        
		        mapa.append([aux, average])
		        flag=0
		        aux=[]
		        aux1=[]
		        aux2=[]
		        aux3=[]
		        aux4=[]
		        aux5=[]
		        aux6=[]
		        average=[]
		    
	    i+=1
	    
	#new_vertices=[None]*len(mapa) 
	#counter=0  
	#for i in range(len(vertices)):
	#    for j in range(len(mapa)):
	#        if(vertices[i] == mapa[j][0]):
	#            new_vertices[counter] = vertices[i]
	#            counter +=1
	edges = []
	index_vert=0
	min_x=0
	max_x=0
	min_y=0
	max_y=0
	
	while (index_vert < len(mapa)):
	
	    existing = False
	    lastx=mapa[index_vert][0][1]
	    lasty=mapa[index_vert][0][2]
	
	    #incialza a lista de vertices           
	    if index_vert==0:
		 last_vert=0
	    
	    else:
	    #verifica se coordenadas sao repetidas
		j=0
		while j<len(mapa):
		    if abs(lastx-mapa[j][0][1])<= larg_corredor and abs(lasty-mapa[j][0][2]) <= larg_corredor:
		        #é repetido
		        existing = True
		        rep = j
		        break
		    j += 1
		    
		        
		if existing == False:
	
		    if ([index_vert, last_vert] in edges) == False and ([last_vert, index_vert] in edges) == False:     
		        edges.append([index_vert, last_vert])
		        last_vert = index_vert
		        index_vert +=1
	     
		
	    
		else : 
		    #verifica se esta ligação ja existe
		    if ([last_vert, rep] in edges) == False and ([rep, last_vert] in edges) == False:
		        edges.append([last_vert, rep])
		        last_vert=rep
		        
	    index_vert+=1
	   
	   
	   
	   
	   
	i=0
	declive=[None]*len(edges)
	b=[None]*len(edges)
	declive_perp=[None]*len(edges)
	for i in range(len(edges)):
	    #calcular declive (m) e ordenada na origem (b)
	    xa=mapa[edges[i][0]] [0][1]
	    ya=mapa[edges[i][0]] [0][2]
	    xb=mapa[edges[i][1]] [0][1]
	    yb=mapa[edges[i][1]] [0][2]
	
	    if xa==xb:
		xb=xb*0.999999  
	    
	    declive[i]=float((ya-yb))/(xa-xb)
	    
	    if declive[i]==0:
		declive[i]=0.0001
	    declive_perp[i]=float(-1)/declive[i]
	    b[i]=ya-float(declive[i])*xa

   	return [vertices,edges,mapa,declive,declive_perp,b]
        
