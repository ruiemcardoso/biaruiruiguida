# -*- coding: utf-8 -*-
"""
Created on Mon Nov 27 11:43:21 2017

@author: student
"""

import math

#aux6=[[['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-84'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-90'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:24:B2:2C:E8:8F', '-91'], ['00:27:19:C2:5D:12', '-64'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-54'], ['24:01:C7:91:5A:11', '-54'], ['24:01:C7:B9:E0:30', '-63'], ['24:01:C7:B9:E0:31', '-67'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-87'], ['70:4F:57:93:DF:8E', '-87'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-79'], ['B0:00:B4:20:4D:10', '-88'], ['B0:00:B4:20:4D:11', '-92'], ['B8:27:EB:C8:AB:3D', '-91'], ['BC:62:0E:3B:DB:50', '-88'], ['C4:6E:1F:2E:E8:40', '-86'], ['D0:BF:9C:3E:F4:76', '-93'], ['D4:6E:0E:3F:2F:36', '-93'], ['F0:F2:49:50:8D:C9', '-85']], [['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-84'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:24:B2:2C:E8:8F', '-91'], ['00:27:19:C2:5D:12', '-63'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-51'], ['24:01:C7:91:5A:11', '-51'], ['24:01:C7:B9:E0:30', '-69'], ['24:01:C7:B9:E0:31', '-68'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-88'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-84'], ['B0:00:B4:20:4D:10', '-89'], ['B0:00:B4:20:4D:11', '-91'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-86'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']], [['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:27:19:C2:5D:12', '-52'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-52'], ['24:01:C7:91:5A:11', '-51'], ['24:01:C7:B9:E0:30', '-69'], ['24:01:C7:B9:E0:31', '-66'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-86'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-77'], ['B0:00:B4:20:4D:10', '-89'], ['B0:00:B4:20:4D:11', '-91'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-87'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']], [['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-79'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-83'], ['00:27:19:C2:5D:12', '-54'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-56'], ['24:01:C7:91:5A:11', '-57'], ['24:01:C7:B9:E0:30', '-61'], ['24:01:C7:B9:E0:31', '-61'], ['24:09:95:1A:1F:80', '-93'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-89'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-76'], ['B0:00:B4:20:4D:10', '-88'], ['B0:00:B4:20:4D:11', '-93'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-87'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']], [['00:12:DA:CB:9A:A0', '-75'], ['00:12:DA:CB:9A:A1', '-76'], ['00:12:DA:CB:9A:A2', '-75'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-83'], ['00:27:19:C2:5D:12', '-58'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-53'], ['24:01:C7:91:5A:11', '-53'], ['24:01:C7:B9:E0:30', '-62'], ['24:01:C7:B9:E0:31', '-61'], ['24:09:95:1A:1F:80', '-93'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-90'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-78'], ['B0:00:B4:20:4D:10', '-88'], ['B0:00:B4:20:4D:11', '-93'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-92'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-87'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']]]

# implementação da matriz média e desvio padrão de potências:
                
                #b indica o timestamp (T-2, T-1, T, T+1, T+2) para o qual se está a olhar no momento
                #b_aux indica o ap actual no timestamp dado por b; este vai ser comparado c os aps dos outros timestamps
                #c indica o timestamp que está a ser comparado com o timestamp actual b
                #a indica o ap do timestamp c, que vai ser comparado com o ap dado por b_aux




def med_dev5(aux6):     #chamar quando se quer comparar um total de 5 timestamps

    a=0                
    media=[]  
    pointer=0
    for b in range(0,4):    #b=0, 1, 2 ou 3
        b_aux=0     
        a=0          
        a1=0
        a2=0
        a3=0
        a4=0 
                    
        while b_aux<len(aux6[b]):
            current=aux6[b][b_aux][0]
            vector=[]
            valor_media=0
            pointer=0
            vector.append(10**(((int(aux6[b][b_aux][1]))-30)/float(10)))
                      
            n=0
            while len(media)>0 and n<len(media):
                if media[n][0]==current: 
                    pointer=1
                n+=1
                    
            for c in range(b+1,5):      #c=1, 2, 3 ou 4, c indica o timestamp c/ q se está a comparar
                
                if pointer==0:
                    
                    if c==1:
                        a=a1
                    elif c==2:
                        a=a2
                    elif c==3:
                        a=a3
                    elif c==4:
                        a=a4
                        
                    while a<len(aux6[c]) and pointer==0:
                        
                        if current==aux6[c][a][0] and pointer==0:
                            vector.append(10**(((int(aux6[c][a][1]))-30)/float(10)))
                            
                        else:
                            if current < aux6[c][a][0]:
                                a=a-1
                                pointer =1
                  
                        a+=1
                                
                    if c==1:
                        a1=a
                    elif c==2:
                        a2=a
                    elif c==3:
                        a3=a
                    elif c==4:
                        a4=a    
                    pointer=0
                    
            if len(vector)>0 and pointer==0:    
                valor_media=float(sum(vector))/float(len(vector))
                j=0
                soma=10**(-27)
                dev_p=0
                while j<len(vector):
                    soma=soma + ((vector[j]-valor_media)**2)
                    j+=1
                valor_media= 30+10*math.log10(valor_media)
                dev_p=30+10*math.log10(math.sqrt(float(soma)/float(len(vector))))
                media.append([current, valor_media, dev_p])    
                 
            b_aux+=1
    
    a=0
    i=0
    b+=1
    flag=0
    while a < len(aux6[b]):
        while i<len(media):
            if aux6[b][a][0] == media[i][0]:
                flag=1
            i+=1
        if flag ==0:
                media.append([aux6[b][a][0], aux6[b][a][1], 30+10*math.log10(10**(-27))])
        a+=1
        i=0
        flag=0
        
    media=sorted(media)    
    return media
    
   
   
   
   
   
   
#aux6=[[['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-84'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-90'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:24:B2:2C:E8:8F', '-91'], ['00:27:19:C2:5D:12', '-64'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-54'], ['24:01:C7:91:5A:11', '-54'], ['24:01:C7:B9:E0:30', '-63'], ['24:01:C7:B9:E0:31', '-67'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-87'], ['70:4F:57:93:DF:8E', '-87'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-79'], ['B0:00:B4:20:4D:10', '-88'], ['B0:00:B4:20:4D:11', '-92'], ['B8:27:EB:C8:AB:3D', '-91'], ['BC:62:0E:3B:DB:50', '-88'], ['C4:6E:1F:2E:E8:40', '-86'], ['D0:BF:9C:3E:F4:76', '-93'], ['D4:6E:0E:3F:2F:36', '-93'], ['F0:F2:49:50:8D:C9', '-85']], [['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-84'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:24:B2:2C:E8:8F', '-91'], ['00:27:19:C2:5D:12', '-63'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-51'], ['24:01:C7:91:5A:11', '-51'], ['24:01:C7:B9:E0:30', '-69'], ['24:01:C7:B9:E0:31', '-68'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-88'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-84'], ['B0:00:B4:20:4D:10', '-89'], ['B0:00:B4:20:4D:11', '-91'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-86'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']], [['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:27:19:C2:5D:12', '-52'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-52'], ['24:01:C7:91:5A:11', '-51'], ['24:01:C7:B9:E0:30', '-69'], ['24:01:C7:B9:E0:31', '-66'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-86'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-77'], ['B0:00:B4:20:4D:10', '-89'], ['B0:00:B4:20:4D:11', '-91'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-87'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']], [['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-79'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-83'], ['00:27:19:C2:5D:12', '-54'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-56'], ['24:01:C7:91:5A:11', '-57'], ['24:01:C7:B9:E0:30', '-61'], ['24:01:C7:B9:E0:31', '-61'], ['24:09:95:1A:1F:80', '-93'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-89'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-76'], ['B0:00:B4:20:4D:10', '-88'], ['B0:00:B4:20:4D:11', '-93'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-87'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']]]
   
def med_dev4(aux6):     #chamar quando se quer comparar um total de 4 timestamps
    
    a=0                
    media=[]  
    pointer=0
    for b in range(0,3):    #b=0, 1 ou 2
        b_aux=0     
        a=0          
        a1=0
        a2=0
        a3=0
                    
        while b_aux<len(aux6[b]):
            current=aux6[b][b_aux][0]
            vector=[]
            valor_media=0
            pointer=0
            vector.append(10**(((int(aux6[b][b_aux][1]))-30)/float(10)))
                      
            n=0
            while len(media)>0 and n<len(media):
                if media[n][0]==current: 
                    pointer=1
                n+=1
                    
            for c in range(b+1,4):      #c=1, 2 ou 3, c indica o timestamp c/ q se está a comparar
                
                if pointer==0:
                    
                    if c==1:
                        a=a1
                    elif c==2:
                        a=a2
                    elif c==3:
                        a=a3
                        
                    while a<len(aux6[c]) and pointer==0:
                        
                        if current==aux6[c][a][0] and pointer==0:
                            vector.append(10**(((int(aux6[c][a][1]))-30)/float(10)))
                            
                        else:
                            if current < aux6[c][a][0]:
                                a=a-1
                                pointer =1
                  
                        a+=1
                                
                    if c==1:
                        a1=a
                    elif c==2:
                        a2=a
                    elif c==3:
                        a3=a   
                    pointer=0
                    
            if len(vector)>0 and pointer==0:    
                valor_media=float(sum(vector))/float(len(vector))
                j=0
                soma=10**(-27)
                dev_p=0
                while j<len(vector):
                    soma=soma + ((vector[j]-valor_media)**2)
                    j+=1
                valor_media= 30+10*math.log10(valor_media)
                dev_p=30+10*math.log10(math.sqrt(float(soma))/float(len(vector)))
                media.append([current, valor_media, dev_p])    
                 
            b_aux+=1
    
    media=sorted(media)    
    return media    
    
    
    
    
    
    
#aux6=[[['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-84'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-90'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:24:B2:2C:E8:8F', '-91'], ['00:27:19:C2:5D:12', '-64'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-54'], ['24:01:C7:91:5A:11', '-54'], ['24:01:C7:B9:E0:30', '-63'], ['24:01:C7:B9:E0:31', '-67'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-87'], ['70:4F:57:93:DF:8E', '-87'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-79'], ['B0:00:B4:20:4D:10', '-88'], ['B0:00:B4:20:4D:11', '-92'], ['B8:27:EB:C8:AB:3D', '-91'], ['BC:62:0E:3B:DB:50', '-88'], ['C4:6E:1F:2E:E8:40', '-86'], ['D0:BF:9C:3E:F4:76', '-93'], ['D4:6E:0E:3F:2F:36', '-93'], ['F0:F2:49:50:8D:C9', '-85']], [['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A1', '-84'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:24:B2:2C:E8:8F', '-91'], ['00:27:19:C2:5D:12', '-63'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-51'], ['24:01:C7:91:5A:11', '-51'], ['24:01:C7:B9:E0:30', '-69'], ['24:01:C7:B9:E0:31', '-68'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-88'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-84'], ['B0:00:B4:20:4D:10', '-89'], ['B0:00:B4:20:4D:11', '-91'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-86'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']], [['00:05:CA:7F:56:A8', '-91'], ['00:12:DA:CB:9A:A0', '-86'], ['00:12:DA:CB:9A:A2', '-81'], ['00:1D:AA:F0:6C:C0', '-93'], ['00:1E:7A:3B:D9:30', '-84'], ['00:23:CD:1A:02:2D', '-87'], ['00:27:19:C2:5D:12', '-52'], ['08:60:6E:BC:C2:60', '-91'], ['18:D6:C7:E8:C5:74', '-91'], ['24:01:C7:91:5A:10', '-52'], ['24:01:C7:91:5A:11', '-51'], ['24:01:C7:B9:E0:30', '-69'], ['24:01:C7:B9:E0:31', '-66'], ['26:0D:C2:4C:F3:5B', '-84'], ['30:B5:C2:62:74:51', '-86'], ['70:4F:57:93:DF:8E', '-87'], ['74:EA:3A:D3:B7:24', '-89'], ['A6:0D:AF:29:0D:62', '-91'], ['AA:6B:AD:59:6E:CC', '-77'], ['B0:00:B4:20:4D:10', '-89'], ['B0:00:B4:20:4D:11', '-91'], ['B0:C1:9E:07:52:1F', '-87'], ['B8:27:EB:C8:AB:3D', '-89'], ['BC:62:0E:3B:DB:50', '-81'], ['C4:6E:1F:2E:E8:40', '-87'], ['D0:BF:9C:3E:F4:76', '-90'], ['D4:6E:0E:3F:2F:36', '-91'], ['F0:F2:49:50:8D:C9', '-85']]]
   
def med_dev3(aux6):     #chamar quando se quer comparar um total de 3 timestamps
   
    a=0                
    media=[]  
    pointer=0
    for b in range(0,2):    #b=0 ou 1 
        b_aux=0     
        a=0          
        a1=0
        a2=0
                    
        while b_aux<len(aux6[b]):
            current=aux6[b][b_aux][0]
            vector=[]
            valor_media=0
            pointer=0
            vector.append(10**(((int(aux6[b][b_aux][1]))-30)/float(10)))
                      
            n=0
            while len(media)>0 and n<len(media):
                if media[n][0]==current: 
                    pointer=1
                n+=1
                    
            for c in range(b+1,3):      #c=1 ou 2 c indica o timestamp c/ q se está a comparar
                
                if pointer==0:
                    
                    if c==1:
                        a=a1
                    elif c==2:
                        a=a2
                        
                    while a<len(aux6[c]) and pointer==0:
                        
                        if current==aux6[c][a][0] and pointer==0:
                            vector.append(10**(((int(aux6[c][a][1]))-30)/float(10)))
                            
                        else:
                            if current < aux6[c][a][0]:
                                a=a-1
                                pointer =1
                  
                        a+=1
                                
                    if c==1:
                        a1=a
                    elif c==2:
                        a2=a
                    pointer=0
                    
            if len(vector)>0 and pointer==0:    
                valor_media=float(sum(vector))/float(len(vector))
                j=0
                soma=10**(-27)
                dev_p=0
                while j<len(vector):
                    soma=soma + ((vector[j]-valor_media)**2)
                    j+=1
                valor_media= 30+10*math.log10(valor_media)
                dev_p=30+10*math.log10(math.sqrt(float(soma))/float(len(vector)))
                media.append([current, valor_media, dev_p])    
                 
            b_aux+=1
    
    media=sorted(media)    
    return media    