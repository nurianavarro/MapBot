# -*- coding: utf-8 -*-
from camaras import *
from orb import *
from slam import *
from robot import *
import cv2
import sim 
import numpy as np
import sys
import time
import matplotlib.pyplot as plt

simTime=60

if __name__ == "__main__":
    
    clientID = connect(19999)
                             
    if clientID != -1:
        print('Server is connected!')
    else:
        print('Server is unreachable!')
        sys.exit(0)
        
    retCode,camaraL=sim.simxGetObjectHandle(clientID,'Vision_sensorL',sim.simx_opmode_blocking)
    retCode,camaraR=sim.simxGetObjectHandle(clientID,'Vision_sensorR',sim.simx_opmode_blocking)
    retCode,ruedaL=sim.simxGetObjectHandle(clientID,'Motor_L',sim.simx_opmode_blocking)
    retCode,ruedaR=sim.simxGetObjectHandle(clientID,'Motor_R',sim.simx_opmode_blocking)
    time.sleep(0.5)
    
    mapbot = robot()
    x=0
    
    mapa = np.zeros((1,3))
    while(x <= simTime):
        mapbot, mapa = slam(mapbot, clientID, camaraL, camaraR, ruedaL, ruedaR, mapa)
        x += 1