#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 11:30:37 2021

@author: BWSI AUV Challenge Instructional Staff
"""
### JRE: for simulation only!

import sys
import pathlib
import datetime

import cv2

# For simulations
from BWSI_BuoyField import BuoyField
from BWSI_Sensor import BWSI_Camera


class ImageProcessor():
    def __init__(self, camera='PICAM', log_dir='./'):
        self.__camera_type = camera.upper()

        if self.__camera_type == 'SIM':
            self.__camera = BWSI_Camera(max_angle=31.1, visibility=30)
            self.__simField = None
            
        else:
            pass

        # create my save directory
        self.__image_dir = pathlib.Path(log_dir, 'frames')
        self.__image_dir.mkdir(parents=True, exist_ok=True)

    
    # ------------------------------------------------------------------------ #
    # Run an iteration of the image processor. 
    # The sim version needs the auv state ot generate simulated imagery
    # the PICAM does not need any auv_state input
    # ------------------------------------------------------------------------ #
    def run(self, auv_state=None):
        red = None
        green = None
        if auv_state['heading'] is not None:
            if (self.__camera_type == 'SIM'):
                if self.__simField is None:
                    self.__simField = BuoyField(auv_state['datum'])
                    config = {'nGates': 50,
                              'gate_spacing': 20,
                              'gate_width': 2,
                              'style': 'linear',
                              'max_offset': 5,
                              'heading': 120}
                    
                    self.__simField.configure(config)
                 
                image = self.__camera.get_frame(auv_state['position'], auv_state['heading'], self.__simField)

            elif self.__camera_type == 'PICAM':
                pass
        
            else:
                print(f"Unknown camera type: {self.__camera_type}")
                sys.exit(-10)
        
            # log the image
            fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
            cv2.imwrite(str(fn), image)
        
            # process and find the buoys!
        
        return red, green
