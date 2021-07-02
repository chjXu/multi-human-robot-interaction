import os
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import cv2
from mpl_toolkits.mplot3d import Axes3D

# 读取csv文件
date = pd.read_csv("../data/skeletons.csv",header=0)
dataset = date.values

data_num = 140
X = dataset[0:data_num,0]
Y = dataset[0:data_num,1]
Z = dataset[0:data_num,2]
T = np.arange(0,data_num,dtype=int)


#进行绘制
# plt.figure()
# plt.subplot(111)
# plt.plot(T,X)
# plt.show()


def KalmanFilter(z,  n_iter = 20):  
    #这里是假设A=1，H=1的情况  
      
    # intial parameters  
     
    sz = (n_iter,) # size of array   
      
    #Q = 1e-5 # process variance  
    Q = 1e-6 # process variance   
    # allocate space for arrays  
    xhat=np.zeros(sz)      # a posteri estimate of x  
    P=np.zeros(sz)         # a posteri error estimate  
    xhatminus=np.zeros(sz) # a priori estimate of x  
    Pminus=np.zeros(sz)    # a priori error estimate  
    K=np.zeros(sz)         # gain or blending factor  
      
    R = 0.1**2 # estimate of measurement variance, change to see effect  
      
    # intial guesses  
    xhat[0] = 0.0  
    P[0] = 1.0  
    A = 1
    H = 1

    for k in range(1,n_iter):  
        # time update  
        xhatminus[k] = A * xhat[k-1]  #X(k|k-1) = AX(k-1|k-1) + BU(k) + W(k),A=1,BU(k) = 0  
        Pminus[k] = A * P[k-1]+Q      #P(k|k-1) = AP(k-1|k-1)A' + Q(k) ,A=1  
      
        # measurement update  
        K[k] = Pminus[k]/( Pminus[k]+R ) #Kg(k)=P(k|k-1)H'/[HP(k|k-1)H' + R],H=1  
        xhat[k] = xhatminus[k]+K[k]*(z[k]-H * xhatminus[k]) #X(k|k) = X(k|k-1) + Kg(k)[Z(k) - HX(k|k-1)], H=1  
        P[k] = (1-K[k] * H) * Pminus[k] #P(k|k) = (1 - Kg(k)H)P(k|k-1), H=1  
    return xhat


xhat = KalmanFilter(X, n_iter=len(X))
yhat = KalmanFilter(Y, n_iter=len(Y))
zhat = KalmanFilter(Z, n_iter=len(Z))


plt.figure()
plt.suptitle("after,before")
plt.subplot(231)
plt.plot(T,xhat)
plt.subplot(232)
plt.plot(T,yhat)
plt.subplot(233)
plt.plot(T,zhat)
plt.subplot(234)
plt.plot(T,X)
plt.subplot(235)
plt.plot(T,Y)
plt.subplot(236)
plt.plot(T,Z)
plt.show()
# #
N = 200
fig_2 = plt.figure()
ax = fig_2.add_subplot((111),projection='3d')
figure = ax.plot(xhat[:data_num], yhat[:data_num], zhat[:data_num], c='r')
# ax.grid(False)
# ax.scatter(xhat[:N],yhat[:N],zhat[:N],c='r')
# ax.scatter(X[:N],Y[:N],Z[:N],c='b')
# #连线
# ax.plot(xhat[:N],yhat[:N],zhat[:N],c='b')
# ax.plot(X[:N],Y[:N],Z[:N],c='g')
plt.show()














