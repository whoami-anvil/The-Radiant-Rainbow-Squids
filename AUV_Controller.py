#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 12:05:08 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import numpy as np

class AUVController():
    def __init__(self):

        # initialize state information
        self.__heading = None
        self.__speed = None
        self.__rudder = None
        self.__rudder_prev = 0.0
        self.__position = None

        # assume we want to be going the direction we're going for now
        self.__desired_heading = None

    def initialize(self, auv_state):
        self.__heading = auv_state['heading']
        self.__speed = auv_state['speed']
        self.__rudder = auv_state['rudder']
        self.__position = auv_state['position']

        # assume we want to be going the direction we're going for now
        self.__desired_heading = auv_state['heading']

        #used for keeping track of times in AUV
        self.__time_list = []

    ### Public member functions
    def update_state (self, cmd, dt):
        #cmd is in BFNVG format
        #cmd comes from backseat

        delta_heading = self.__MAX_TURNING_RATE * (self.__rudder / self.__HARD_RUDDER_DEG) * (self.__speed_knots / self.__MAX_SPEED_KNOTS) * dt

		# adjust the delta heading based on rudder history
		# delta_heading += self.__rudder_hydro_effect()

		# adjust the delta heading based on rudder history
		# delta_heading += self.__rudder_hydro_effect()

        #update last known rudder state
        self.__rudder_prev = self.__rudder

		final_heading = np.mod(self.__heading + delta_heading + 360.0, 360.0)
		avg_heading = np.mod( self.__heading + delta_heading / 2.0 + 360.0, 360.0)

		# just march forward
		dx = self.__speed_mps * dt * np.sin(np.radians(avg_heading))
		dy = self.__speed_mps * dt * np.cos(np.radians(avg_heading))
		x = self.__position[0] + dx
		y = self.__position[1] + dy
		self.__position = (x,y)

		self.__heading = final_heading

def __select_command(self):
    # Unless we need to issue a command, we will return None
    turn_angle = None #for rudder
    rpm_speed = 750 #for RPM, 500RPM/knot, up to 5 knots

    # determine the angle between current and desired heading
    delta_angle = self.__desired_heading - self.__heading
    if delta_angle > 180: # angle too big, go the other way!
        delta_angle = delta_angle - 360
    if delta_angle < -180: # angle too big, go the other way!
        delta_angle = delta_angle + 360

    # how much do we want to turn the rudder
    ## Note: using STANDARD RUDDER only for now! A calculation here
    ## will improve performance!
    if np.abs(delta_angle) > 10:
        turn_angle = 15
    else:
        turn_angle = 5

    # which way do we have to turn
    if delta_angle>2: # need to turn to right!
        if self.__rudder >= 0: # rudder is turning the other way!
            pass
    elif delta_angle<-2: # need to turn to left!
        if self.__rudder <= 0: # rudder is turning the other way!
            turn_angle = -turn_angle
    else: #close enough!
        turn_angle = 0
    #adjust turn angle in comparison to previous rudder angle
    rudder_turn = self.__rudder_prev - turn_angle

    return rudder_turn, rpm_speed

    # return the desired heading to a public requestor
    def get_desired_heading(self):
        return self.__desired_heading


    ### Private member functions

    # calculate the heading we want to go to reach the gate center
    def __heading_to_position(self, gnext, rnext):
        # center of the next buoy pair
        gate_center = ((gnext[0]+rnext[0])/2.0, (gnext[1]+rnext[1])/2.0)

        # heading to gate_center
        tgt_hdg = np.mod(np.degrees(np.arctan2(gate_center[0]-self.__position[0],
                                               gate_center[1]-self.__position[1]))+360,360)

        return tgt_hdg

    def __heading_to_angle(self, gnext, rnext):
        # relative angle to the center of the next buoy pair
        relative_angle = (gnext[0] + rnext[0]) / 2.0

        # heading to center of the next buoy pair
        tgt_hdg = self.__heading + relative_angle

        return tgt_hdg

    def decide (self, auv_state, green_buoys, red_buoys, sensor_type = 'POSITION'):
        delta_rudder = 0
		new_engine_speed = 750 # RPM, as default

		# Z - AL Logic
        #decide rudder angles
        #figure out how to get it to move there based on its last rudder angle
		# update state information
        self.__heading = auv_state['heading']
        self.__speed = auv_state['speed']
        self.__rudder = auv_state['rudder']
        self.__position = auv_state['position']

        # determine what heading we want to go
        if sensor_type.upper() == 'POSITION': # known positions of buoys
            self.__desired_heading = self.__heading_to_position(green_buoys, red_buoys)
        elif sensor_type.upper() == 'ANGLE': # camera sensor
            self.__desired_heading = self.__heading_to_angle(green_buoys, red_buoys)

        # determine whether and what command to issue to desired heading
        delta_rudder, new_engine_speed = self.__select_command()

        return delta_rudder, new_engine_speed
