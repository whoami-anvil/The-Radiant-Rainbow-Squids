# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 19:49:45 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import numpy as np
import BWSI_BuoyField

class BWSI_Camera(object):
    def __init__(self, visibility):
        self.__MAX_RANGE = visibility # maximum range camera can see
        self.__MAX_ANGLE = 110.0 # field of view of camera (+/- MAX_ANGLE degrees)
        self.__SENSOR_TYPE = 'ANGLE'
        
    def get_visible_buoys(self, pos, hdg, buoy_field):
        angle_left = np.mod(hdg-self.__MAX_ANGLE+360, 360)
        angle_right = np.mod(hdg+self.__MAX_ANGLE, 360)
        G, R = buoy_field.detectable_buoys(pos, 
                                           self.__MAX_RANGE, 
                                           angle_left,
                                           angle_right,
                                           self.__SENSOR_TYPE)
        
        for i in range(len(G)):
            G[i] = np.mod(G[i] - hdg + 360, 360)
            if G[i]>self.__MAX_ANGLE:
                G[i] = G[i] - 360.0
            if G[i]<-self.__MAX_ANGLE:
                G[i] = G[i] + 360.0
            
        for i in range(len(R)):
            R[i] = np.mod(R[i] - hdg + 360, 360)
            if R[i]>self.__MAX_ANGLE:
                R[i] = R[i] - 360.0
            if R[i]<-self.__MAX_ANGLE:
                R[i] = R[i] + 360.0
                
        return G, R
    
    
class BWSI_Laser(object):
    def __init__(self, visibility):
        self.__MAX_RANGE = visibility # maximum range camera can see
        self.__MAX_ANGLE = 85.0 # field of view of camera (+/- MAX_ANGLE degrees)
        self.__SENSOR_TYPE = 'RANGE_ANGLE'
        
    def get_visible_buoys(self, pos, hdg, buoy_field):
        angle_left = np.mod(hdg-self.__MAX_ANGLE+360, 360)
        angle_right = np.mod(hdg+self.__MAX_ANGLE, 360)
        G, R = buoy_field.detectable_buoys(pos, 
                                           self.__MAX_RANGE, 
                                           angle_left,
                                           angle_right,
                                           self.__SENSOR_TYPE)
                
        return G, R