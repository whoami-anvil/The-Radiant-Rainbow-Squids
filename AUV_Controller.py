#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 12:05:08 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import numpy as np

class AUVController ():

	def __init__ (self):
		"""
		intialize the state of the AUV
		"""

		# initialize state information
		self.__heading = None
		self.__speed = 0
		self.__rudder = 0
		self.__rudder_prev = 0
		self.__position = None
		self.__speed_knots = None
		self.__speed_mps = None
		self.__midpoint = None

		# assume we want to be going the direction we're going for now
		self.__desired_heading = None
		self.__time_list = []

	def initialize (self, auv_state):
		"""
		reinitialize auv state
		"""
		self.__heading = auv_state['heading']
		self.__position = auv_state['position']
		self.__speed_knots = self.__speed / 250
		self.__speed_mps = self.__speed_knots * (1852 / 3600)

		# assume we want to be going the direction we're going for now
		self.__desired_heading = auv_state['heading']

	### Public member functions
	def update_state (self, auv_state):
		"""
		update controller's heading, position, last updated time
		"""
		self.__heading = auv_state['heading']
		self.__position = auv_state['position']

		#used for keeping track of times in AUV
		self.__time_list.append(auv_state['last_fix_time'])

	def decide (self, auv_state, green_buoys, red_buoys, sensor_type = 'POSITION'):
		"""
		update desired heading and return new rudder angle and speed
		"""

		#decide rudder angles
		#figure out how to get it to move there based on its last rudder angle
		# update state information
		self.__heading = auv_state['heading']
		self.__position = auv_state['position']

		if not(green_buoys == None):

			green_buoys_translated = ((green_buoys[0] + auv_state['position'][0]), (green_buoys[1] + auv_state['position'][1]))

		else:

			green_buoys_translated = None

		if not(red_buoys == None):

			red_buoys_translated = ((red_buoys[0] + auv_state['position'][0]), (red_buoys[1] + auv_state['position'][1]))

		else:

			red_buoys_translated = None

		# determine what heading we want to go

		print(f"Red Buoys: {red_buoys}\nGreen Buoys: {green_buoys}")

		if (red_buoys_translated == None or green_buoys_translated == None):

			return 0, 750

		elif sensor_type.upper() == 'POSITION': # known positions of buoys

			print("Have seen buoys by position")

			self.__desired_heading = self.__heading_to_position(green_buoys, red_buoys)

		elif sensor_type.upper() == 'ANGLE': # camera sensor

			print("Have seen buoys by angle")

			self.__desired_heading = self.__heading_to_angle(green_buoys, red_buoys)

		# determine whether and what command to issue to desired heading
		new_rudder, new_engine_speed = self.__select_command()

		self.__rudder_prev = new_rudder

		self.__speed = new_engine_speed
		self.__speed_knots = self.__speed / 250
		self.__speed_mps = self.__speed_knots * (1852 / 3600)

		return new_rudder, new_engine_speed

	# return the desired heading to a public requestor
	def get_desired_heading (self):
		"""
		return the desired heading of the AUV
		"""

		return self.__desired_heading

	### Private member functions

	# calculate the heading we want to go to reach the gate center
	def __heading_to_position (self, gnext=None, rnext=None):
		"""
		DEPRECATED
		used to set desired_heading
		target heading becomes self.__desired_heading
		"""
		# center of the next buoy pair
		tgt_hdg = self.__heading

		if not(gnext == None and rnext == None):

			gate_center = ((gnext[0] + rnext[0]) / 2.0, (gnext[1] + rnext[1]) / 2.0)
			tgt_hdg = np.mod(np.degrees(np.arctan2(gate_center[0] - self.__position[0],
												   gate_center[1] - self.__position[1])) - 270, 360)

			print(f"Target heading: {tgt_hdg}\nHeading: {self.__heading}\nDelta Heading: {tgt_hdg - self.__heading}")

		elif not(gnext == None) and rnext == None:

			#if only one gate, set gate "center" to buoy location for temporary redirection
			self.__midpoint = (gnext[0], gnext[1])
			tgt_hdg = np.mod(np.degrees(np.arctan2(self.__midpoint[0] - self.__position[0],
												   self.__midpoint[1] - self.__position[1])) + 360,360)
		elif not(rnext == None) and gnext == None:

			self.__midpoint  = (rnext[0], rnext[1])
			tgt_hdg = np.mod(np.degrees(np.arctan2(self.__midpoint[0] - self.__position[0],
												   self.__midpoint[1] - self.__position[1])) + 360,360)

		else:

			tgt_hdg = self.__heading

		# else, do nothing and go straight
		# heading to gate_center

		return tgt_hdg

	def __heading_to_angle(self, gnext=None, rnext=None):
		"""
		return desired heading based on angle relative to the AUV
		"""
		#relative angle to the center of the next buoy pair
		if(rnext == None) and (gnext == None):
			tgt_hdg = np.mod(self.__heading + 360, 360)

		elif (rnext != None) and (gnext != None):

			print(rnext)

			print(gnext)
			relative_angle = (gnext + rnext) / 2.0
			tgt_hdg = np.mod(self.__heading + relative_angle + 360, 360)
		elif (rnext != None) and (gnext == None):
			#set tgt_hdg to heading of rnext
			relative_angle = rnext
			tgt_hdg = np.mod(self.__heading + relative_angle + 360, 360)

		#
		# if ((self.__heading + relative_angle) < 360):
		#   	tgt_hdg = self.__heading + relative_angle
		#
		# elif ((self.__heading + relative_angle) >= 360):
		#   	tgt_hdg = relative_angle - (360 - self.__heading)
		#
		# elif len(gnext)>0:
		#
		#   	tgt_hdg = self.__heading + gnext[0]
		#
		# elif len(rnext)>0:
		#
		#   	tgt_hdg = self.__heading + rnext[0]


		return tgt_hdg

	# choose a command to send to the front seat
	def __select_command(self):
		"""
		return the rudder turn angle and rpm speed based on difference in heading
		"""
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
		"""
		1000RPM/1 knot
		Max Angle: 25
		Max speed: 1000 RPM / 2 knots
		Min Angle: 0
		Min Speed: 750 RPM / 1.25 knots
		RPM Range 250 rpm
		Knot Range 0.75 knots
		"""

		# editing rudder speed
		if delta_angle > 0:

			rpm_speed = 750 + 250 / (1 + 250 * np.exp(-0.5 * delta_angle))

		elif delta_angle < 0:

			rpm_speed = 750 + 250 / (1 + 250 * np.exp(0.5 * delta_angle))

		else:

			rpm_speed = 750

		if np.abs(delta_angle) > 10:
		#
			turn_angle = 15
			rpm_speed = 1000
		#
		else:
		#
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
		rudder_turn = turn_angle

		return rudder_turn, rpm_speed
