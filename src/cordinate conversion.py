import numpy as np
import math as m
z_= 2 #distance of rogue drone from the camera or image
xi=np.array([[1],[0.1],[z_],[1]]) #input
f=0.5
K=np.array([[f, 0, 0, 0],[0, f, 0, 0],[0, 0, 1,0],[0,0,0,1]])
print(K) #cameramatrix
Ki=np.linalg.inv(K)
print(Ki)
Xc=np.dot(Ki,xi) # camera cordinate system
print(Xc)
theta=60 #robot cordinates
xr=12 #robot cordinates
yr=14 #robot cordinates
Tr=np.array([[np.cos(theta), -np.sin(theta), 0, xr],[np.sin(theta),np.cos(theta), 0, yr],[0, 0, 1,0],[0,0,0,1]])
X=np.dot(Tr,Xc)
print(X)