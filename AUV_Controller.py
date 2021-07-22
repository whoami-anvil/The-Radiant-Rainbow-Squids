#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 12:05:08 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import numpy as np
import utm as utm
import pynmea2

class AUVController ():

    def __init__ (self):

		# initialize state information
		self.__heading = None
		self.__speed = None
		self.__rudder = None
		self.__position = None
		self.__speed_mps = None
		self.__speed_knots = None

        # assume we want to be going the direction we're going for now
        self.__desired_heading = None

        #vehicle controller limits

        self.__HARD_RUDDER_DEG = 25
        self.__MAX_SPEED_KNOTS = 4
        self.__MAX_TURNING_RATE = 11.67

    def initialize (self, auv_state):


        self.__heading = auv_state['heading']
        self.__speed = auv_state['speed']
        self.__rudder = auv_state['rudder']
        self.__position = auv_state['position']

		self.__heading = auv_state['heading']
		self.__speed = auv_state['speed']
		self.__rudder = auv_state['rudder']
		self.__position = auv_state['position']

        # need to get speed meter/s and speed knots from AUV state
		self.__speed_mps = None
		self.__speed_knots = None
		self.__orig_lat_lon = None


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


		final_heading = np.mod(self.__heading + delta_heading + 360.0, 360.0)
		avg_heading = np.mod( self.__heading + delta_heading / 2.0 + 360.0, 360.0)

		# just march forward
		dx = self.__speed_mps * dt * np.sin(np.radians(avg_heading))
		dy = self.__speed_mps * dt * np.cos(np.radians(avg_heading))
		x = self.__position[0] + dx
		y = self.__position[1] + dy
		self.__position = (x,y)

		self.__heading = final_heading

    def decide (self, auv_state, green_buoys, red_buoys, sensor_type = 'POSITION'):

		### WIP ###

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
        cmd = self.__select_command()

        return cmd


    # return the desired heading to a public requestor
    def get_desired_heading (self):

        return self.__desired_heading

    def get_position (self):

    	return self.__position

    def get_current_heading (self):

        return self.__heading

    ### Private member functions

    # calculate the heading we want to go to reach the gate center
    def __heading_to_position (self, gnext, rnext):

        # center of the next buoy pair
        gate_center = ((gnext[0] + rnext[0]) / 2.0, (gnext[1] + rnext[1]) / 2.0)

        # heading to gate_center
        tgt_hdg = np.mod(np.degrees(np.arctan2(gate_center[0] - self.__position[0],
											   gate_center[1] - self.__position[1])) + 360, 360)

        return tgt_hdg

    def __heading_to_angle (self, gnext, rnext):

        # relative angle to the center of the next buoy pair
        relative_angle = (gnext[0] + rnext[0]) / 2.0

        # heading to center of the next buoy pair
        tgt_hdg = self.__heading + relative_angle

        return tgt_hdg

    # choose a command to send to the front seat
    def __select_command (self):

        # Unless we need to issue a command, we will return None
        cmd = None

        # determine the angle between current and desired heading
        delta_angle = self.__desired_heading - self.__heading

        if delta_angle > 180: # angle too big, go the other way!

            delta_angle = delta_angle - 360

        if delta_angle < -180: # angle too big, go the other way!

            delta_angle = delta_angle + 360

        turn_command = ""

        #using delta angle to calculate for better performance
        if self.desired_heading != self.heading:

            if np.abs(self.desired_heading - self.heading) > 10:

                turn_command = "STANDARD RUDDER"

            elif np.abs(self.desired_heading - self.heading) > 3:

                turn_command = "5 DEGREES RUDDER"

        # which way do we have to turn
        if delta_angle > 2: # need to turn to right!

            if self.__rudder >= 0: # rudder is turning the other way!

                cmd = f"RIGHT {turn_command}"

        elif delta_angle<-2: # need to turn to left!

            if self.__rudder <= 0: # rudder is turning the other way!

                cmd = f"LEFT {turn_command}"

        else: #close enough!

            cmd = "RUDDER AMIDSHIPS"

        return cmd
