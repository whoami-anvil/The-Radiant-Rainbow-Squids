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

		# initialize state information
		self.__heading = None
		self.__speed = None
		self.__rudder = None
		self.__position = None
		self.__speed_mps = None
		self.__speed_knots = None


        # assume we want to be going the direction we're going for now
        self.__desired_heading = None

    def initialize (self, auv_state):


        self.__heading = auv_state['heading']
        self.__speed = auv_state['speed']
        self.__rudder = auv_state['rudder']
        self.__position = auv_state['position']

		self.__heading = auv_state['heading']
		self.__speed = auv_state['speed']
		self.__rudder = auv_state['rudder']
		self.__position = auv_state['position']
		self.__speed_mps = None
		self.__speed_knots = None


        # assume we want to be going the direction we're going for now
        self.__desired_heading = auv_state['heading']


        #used for keeping track of times in AUV
        self.__time_list = []

    ### Public member functions
    def update_state(self, cmd, last_timestamp):
        print(cmd.latitude)
        print(cmd.longitude)
        print(cmd.heading)
        # self.__latlon = (cmd.latitude, cmd.longitude)
        # self.__time_list += cmd.time
        # self.__heading += cmd.heading

        self.__time_list += last_timestamp
        dt = last_timestamp - self.__time_list[-2]


        #get turning rate of vehicle
        turning_rate = 11.67 * (self.__rudder_position / self.__HARD_RUDDER_DEG) * (self.__speed_knots / self.__MAX_SPEED_KNOTS)
        speed_meters_per_second = self.__speed_knots * 0.514444
        heading_radians = np.radians(self.__heading) + np.radians((turning_rate * dt) / 2)
=======
	### Public member functions
	def update_state (self, cmd, dt):



		pass
>>>>>>> Stashed changes

        eastings = (speed_meters_per_second * dt) * np.sin(heading_radians)
        northings = (speed_meters_per_second * dt) * np.cos(heading_radians)

        self.__latlon = utm.to_latlon(self.__position[0] + self.__datum_position[0] + eastings, self.__position[1] + self.__datum_position[1] + northings, self.__datum_position[2], self.__datum_position[3])
        self.__position = (self.__position[0] + eastings, self.__position[1] + northings)
        self.__heading += turning_rate * dt


        #return None

    def decide (self, auv_state, green_buoys, red_buoys, sensor_type = 'POSITION'):

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
