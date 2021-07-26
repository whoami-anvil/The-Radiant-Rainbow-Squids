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
        self.__speed = 750
        self.__rudder = 0.0
		self.__rudder_prev = 0.0
        self.__position = None
		self.__speed_knots = None
		self.__speed_mps = None

        # assume we want to be going the direction we're going for now
        self.__desired_heading = None
		self.__time_list = []

    def initialize(self, auv_state):
        self.__heading = auv_state['heading']
        self.__position = auv_state['position']

        # assume we want to be going the direction we're going for now
        self.__desired_heading = auv_state['heading']

		self.__time_list = []

    ### Public member functions
	def update_state (self, auv_state):

		self.__heading = auv_state['heading']
		self.__position = auv_state['position']

		#used for keeping track of times in AUV
		self.__time_list.append(auv_state['last_fix_time'])

    def decide(self, auv_state, green_buoys, red_buoys, sensor_type='POSITION'):

        		#decide rudder angles
				#figure out how to get it to move there based on its last rudder angle
				# update state information
				self.__heading = auv_state['heading']
				self.__position = auv_state['position']

				# determine what heading we want to go

				print(f"Red Buoys: {red_buoys}\nGreen Buoys: {green_buoys}")
				if (red_buoys == None or green_buoys == None):

					return (self.__rudder_prev * -1), 750

				elif sensor_type.upper() == 'POSITION': # known positions of buoys

					print("Have seen bouys")

					self.__desired_heading = self.__heading_to_position(green_buoys, red_buoys)

				elif sensor_type.upper() == 'ANGLE': # camera sensor

					print("Have seen bouys")

					self.__desired_heading = self.__heading_to_angle(green_buoys, red_buoys)

				# determine whether and what command to issue to desired heading
				delta_rudder, new_engine_speed = self.__select_command()

				self.__rudder_prev += delta_rudder

				self.__speed = new_engine_speed
				self.__speed_knots = self.__speed / 250
				self.__speed_mps = self.__speed_knots * (1852 / 3600)

				return delta_rudder, new_engine_speed

    # return the desired heading to a public requestor
    def get_desired_heading(self):
        return self.__desired_heading


    ### Private member functions

    # calculate the heading we want to go to reach the gate center
    def __heading_to_position(self, gnext=None, rnext=None):
		"""
		with known buoy positions, return target heading
		"""
		# center of the next buoy pair
		tgt_hdg = self.__heading
		if gnext and rnext:
	        gate_center = ((gnext[0]+rnext[0])/2.0, (gnext[1]+rnext[1])/2.0)

	        # heading to gate_center
	        tgt_hdg = np.mod(np.degrees(np.arctan2(gate_center[0]-self.__position[0],
	                                               gate_center[1]-self.__position[1]))+360,360)

        return tgt_hdg

    def __heading_to_angle(self, gnext=None, rnext=None):
		"""
		with known buoy angles, return target heading
		"""
		tgt_hdg = self.__heading
		if gnext and rnext:
			# relative angle to the center of the next buoy pair
	        relative_angle = (gnext[0] + rnext[0]) / 2.0

	        # heading to center of the next buoy pair
	        tgt_hdg = self.__heading + relative_angle

        return tgt_hdg

    # choose a command to send to the front seat
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

		# with delta angle, match anything above zero to thirty degrees / 1.5 knots - anything above proportionally
		# scales to 2 knots
		if np.abs(delta_angle) > 10:

			turn_angle = 15
			rpm_speed = 1000

		else:

			turn_angle = 5
			rpm_speed = 750

		# which way do we have to turn
		if delta_angle > 2: # need to turn to right!

			if self.__rudder >= 0: # rudder is turning the other way!

				pass

		elif delta_angle < -2: # need to turn to left!

			if self.__rudder <= 0: # rudder is turning the other way!

				turn_angle = (turn_angle * -1)

		else: #close enough!

			turn_angle = 0

		#adjust turn angle in comparison to previous rudder angle
		rudder_turn = self.__rudder_prev - turn_angle

		return rudder_turn, rpm_speed
