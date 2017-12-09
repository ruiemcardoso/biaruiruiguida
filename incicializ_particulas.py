# -*- coding: utf-8 -*-
"""
Created on Sat Nov 11 15:27:36 2017

@author: rui
"""

import random
import math
import numpy


def particle_init(mapa, edges,declive,b):
    N = 3*len(mapa)
    particulas = [None]*N
    
    for x in range (0, N):
        
        k = random.randint(0, len(edges)-1)
        
        aux1 = edges[k][0]
        aux2 = edges[k][1]
        
        xa = mapa[aux1][0][1]
        xb = mapa[aux2][0][1]
        
        xp=random.uniform(xa, xb)
        yp=declive[k]*xp+b[k]
        o=0
        
        if (aux1<aux2):
            ref = aux1
        else:
            ref = aux2
        a = xp-mapa[ref][0][1]
        aa =yp-mapa[ref][0][2]
        d = euc_norm([a,aa])        
        
        particulas[x] = [k, 1.0/N, xp, yp, random.uniform(0, 360), o, d]
        
    return particulas

    
def euc_norm(x):
    norm=0
    for i in range(len(x)):
        norm += x[i]**2
    return norm**(1.0/2)
    
    
def encontra_edge(mapa,edges,declive,declive_perp,b,part):
    i=0
    j=0
    modmin=[None]*len(part)
    modaux=[None]*len(edges)
    xpp=[None]*len(edges)
    ypp=[None]*len(edges)
    #partcopia=list(part)
    for j in range(len(part)):
        for i in range(len(edges)):
            b_perp=part[j][3]-declive_perp[i]*part[j][2]
            xpp[i]=float(b_perp-b[i])/(declive[i]-declive_perp[i])
            ypp[i]=declive_perp[i]*xpp[i]+b_perp        
        
            modaux[i]=euc_norm([part[j][2]-xpp[i],part[j][3]-ypp[i]])
        modmin[j]=modaux.index(min(modaux))
        part[j][0]=modmin[j]
        if (edges[modmin[j]][0] < edges[modmin[j]][1]):
            ref = edges[modmin[j]][0]
        else:
            ref = edges[modmin[j]][1]
        h = euc_norm(((xpp[modmin[j]]-mapa[ref][0][1]),(ypp[modmin[j]]-mapa[ref][0][2])))
        
        part[j][6] = h
        part[j][5] = min(modaux)
        part[j][2] = xpp[modmin[j]]
        part[j][3] = ypp[modmin[j]]
    return part

def constrain(particulas, edges, mapa, declive, declive_perp, b):
    wc_sum=0
    particulas=encontra_edge(mapa,edges,declive,declive_perp,b,particulas)
    
    for i in range(len(particulas)):
        larg_corredor=1 
        if particulas[i][5] > larg_corredor:
            particulas[i][1]=0.00000000000001
        
        wc_sum = wc_sum + particulas[i][1]
        
    sum_p=0
    for i in range(len(particulas)):
        particulas[i][1] = particulas[i][1]/(float(wc_sum))
            
    return particulas


def perceptual_model (particula,mapa,edges, medida):

    [x_part,y_part] = particula[2:4]
    
    [vert_i,vert_j]=edges[particula[0]]
    

    [x_i, y_i] = mapa[vert_i][0][1:3]
    [x_j, y_j] = mapa[vert_j][0][1:3]
    
    auxlvi=euc_norm([x_part-x_i,y_part-y_i])
    auxlvj=euc_norm([x_part-x_j,y_part-y_j])    
    auxvivj=euc_norm([x_j-x_i,y_j-y_i])    
    
    c1=0
    c2=0
    M=[]
    D=[]
    mac_ap=[]
    
    while (c1<len(mapa[vert_i][1]) and c2<len(mapa[vert_j][1]) ):
        
        if mapa[vert_i][1][c1][0] == mapa[vert_j][1][c2][0]:
            
            mac_ap.append(mapa[vert_j][1][c2][0])            
            M.append((auxlvj*mapa[vert_j][1][c2][1]+auxlvi*mapa[vert_i][1][c1][1])/auxvivj)
            D.append((auxlvj*mapa[vert_j][1][c2][2]+auxlvi*mapa[vert_i][1][c1][2])/auxvivj)
            c1 += 1
            c2 += 1
            
        elif mapa[vert_i][1][c1][0] < mapa[vert_j][1][c2][0]:
            
            mac_ap.append(mapa[vert_j][1][c2][0])
            M.append((auxlvj*mapa[vert_j][1][c2][1])/auxvivj)
            D.append((auxlvj*mapa[vert_j][1][c2][1]+auxlvi*(-50))/auxvivj)
            c1 += 1
            
        else:
            mac_ap.append(mapa[vert_i][1][c1][0])
            M.append((auxlvi*mapa[vert_i][1][c1][1])/auxvivj)
            D.append((auxlvi*mapa[vert_i][1][c1][1]+auxlvj*(-50))/auxvivj)
            c2 += 1
        
    i=0
    c1=0
    c2=0
    prob_sl=1
        
    if(auxlvi<auxlvj):
        d_xi=auxlvi
    else: 
        d_xi = auxlvj
    
    epsilon=0.1
        
        
    while(c2<len(medida) and c1<len(mac_ap)):
        if (mac_ap[c1] == medida[c2][0]):
            prob_sl = prob_sl*((2*epsilon/(math.sqrt(2*math.pi)*d_xi))*math.e**((medida[c2][1]-M[c1])**2/(2*D[c1]**2)))
            c1+=1
            c2+=1
        elif (mac_ap[c1] < medida[c2][0]):
            c1 +=1
        else: 
            c2 +=1            
            
    return prob_sl
            
    
def resampling(Xt):
    i=0
    aux=range(len(Xt))
    weig=Xt[0][1]
    weight=[None]*len(Xt)
    weigaux=[None]*len(Xt)
    weigaux_sum=0
    dif_weights = False
    soma=0.0
    for i in range(len(Xt)):
        weight[i]=Xt[i][1]
        soma = soma+weight[i]
        
    for i in range(len(Xt)):
        weigaux[i] = weight[i]/float(soma)
        weigaux_sum = weigaux_sum + float(weigaux[i]**2)
        
        if(weight[i]!=weig and dif_weights == False): #verifica se todos os pesos são iguais
            dif_weights = True
        weig = weight[i]
    
    n_ef= 1.0/weigaux_sum
        

    if n_ef<(len(Xt)/2.0) and dif_weights == True:  #se poucas partículas têm todo o peso é feito resampling
        
        i=0
        Xt_resampled=[None]*len(Xt)
        for i in range(len(Xt)):
            part=numpy.random.choice(aux,p=weigaux)
            Xt_resampled[i]=Xt[part]
            Xt_resampled[i][1]=1.0/len(Xt)
            
        Xt=Xt_resampled
            

    return Xt
    

    
def meas_model(particles,mapa,edges, medida):
    
    for i in range(len(particles)):  
        particles[i][1] = perceptual_model (particles[i],mapa,edges, medida)
        
    return particles

#xt = particle_init(mapa, edges, declive, b)
    
#xt = encontra_edge(mapa,edges,declive,declive_perp,b,xt)
