import cv2
import math
import numpy as np
import matplotlib.path as mplPath
import matplotlib.pyplot as plt
import random

#SOLO SE OCUPA CAMBIAR ESTO
#esqu es el valor de cada esquina asi
#
#   3___4
#   |   |
#   1___2
#
esq1 =[8, 14]
esq2 =[29, 24]
esq3 =[9, 35]
esq4 =[18, 34]
it=50
#esta es la cordenada del patito
patito = [15, 32]
patitos = [15, 32]
xlar = []
ylar = []
data = []  # lista vacía
#aqui ya no se cambia
for i in range (it):
    x1 =esq2[0]
    x2 =esq1[0]
    x3 =esq4[0]
    y1 =esq2[1]
    y2 =esq1[1]
    y3 =esq4[1]

    #1 y 2
    a=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    #2 y 3
    b=np.sqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3))
    #3 y 1
    c=np.sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1))

    rad = np.arccos((a*a + b*b - c*c) / (2*a*b))
    θ1=rad*(180/math.pi)
    

    x3 = esq1[0]
    x2 =esq2[0]
    x1 =esq3[0]
    y3 =esq1[1]
    y2 =esq2[1]
    y1 =esq3[1]

    #1 y 2
    a=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    #2 y 3
    b=np.sqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3))
    #3 y 1
    c=np.sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1))
    rad = np.arccos((a*a + b*b - c*c) / (2*a*b))
    θ2=rad*(180/math.pi)
    

    #patito

    x1 =esq2[0]
    x2 =esq1[0]
    x3 =patito[0]
    y1 =esq2[1]
    y2 =esq1[1]
    y3 =patito[1]

    #1 y 2
    a=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    #2 y 3
    b=np.sqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3))
    #3 y 1
    c=np.sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1))

    rad = np.arccos((a*a + b*b - c*c) / (2*a*b))
    θp1=rad*(180/math.pi)
    

    x3 = esq1[0]
    x2 =esq2[0]
    x1 =patito[0]
    y3 =esq1[1]
    y2 =esq2[1]
    y1 =patito[1]

    #1 y 2
    a=np.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
    #2 y 3
    b=np.sqrt((x2-x3)*(x2-x3)+(y2-y3)*(y2-y3))
    #3 y 1
    c=np.sqrt((x3-x1)*(x3-x1)+(y3-y1)*(y3-y1))
    rad = np.arccos((a*a + b*b - c*c) / (2*a*b))
    θp2=rad*(180/math.pi)
    

    angr1=45*θp1/θ1
    
    angr2=45*θp2/θ2

    m1=np.tan(angr1*(math.pi/180))
    m2=-np.tan(angr2*(math.pi/180))
    #y-y1=m(x-x1
    #y=m1(x)
    #y=m2(x-30)
    y=m2*30
    print(y)
    #m1x=m2x-30
    #m2x-m1x=30
    #(m2-m1)(x)=30
    xrec=y/(m2-m1)
    yrec=m1*xrec
    print(xrec," , ",yrec)
    lista= [xrec, yrec]
    

    # Agregar elementos a la lista
    data.append(lista)
    xlar.append(data[i][0])
    ylar.append(data[i][1])
    patito[0]= (patito[0]+((random.choice([0,1,2,3,4,5,6,7,8,9,10]))/10))
    patito[1]= (patito[1]+((random.choice([0,1,2,3,4,5,6,7,8,9,10]))/10))


# Coordenadas x e y de los puntos

xesq=[0,30,0,30]
yesq=[0,0,30,30]
# Generar el gráfico
fig, ax = plt.subplots()
ax.scatter(xlar, ylar,c="r")
ax.scatter(xesq, yesq,c="g")
coords = np.column_stack((xlar,ylar))
ax.plot(coords[:,0], coords[:,1], c="r")
dt=0
for i in range (it-1):
    distancia = math.sqrt((xlar[i+1]- xlar[i])**2 + (ylar[i+1] - ylar[i])**2)
    dt=dt+distancia
print(dt)
plt.show()
print("AAAA")