#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 12:05:08 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import numpy as np
import datetime

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
		self.__order = None
		self.__search = False
		self.__search_direction = "left"
		self.__search_timer = 120

		# assume we want to be going the direction we're going for now
		self.__desired_heading = None
		self.__time_list = []

	def initialize (self, auv_state):
		"""
		reinitialize auv state
		"""
		self.__heading = auv_state['heading']
		self.__position = auv_state['position']
		self.__speed_knots = self.__speed / 500
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

	def decide (self, auv_state, green_buoys, red_buoys, order, sensor_type = 'POSITION'):

		"""
		update desired heading and return new rudder angle and speed
		"""

		#decide rudder angles
		#figure out how to get it to move there based on its last rudder angle
		# update state information
		self.__heading = auv_state['heading']
		self.__position = auv_state['position']

		if (self.__search):

			if (self.__search_timer == 0):

				return -100000000, -100000000

			if not((green_buoys != None) and (red_buoys != None)):

				if ((green_buoys == None) and (red_buoys == None)):

					#print(f"No buoys:\n\t{green_buoys}\n\t{red_buoys}")

					self.__search_timer -= 1

				if self.__search_direction == "left":

					return 25, 750

				elif self.__search_direction == None:

					return 0, 750

				else:

					return -25, 750

			else:

				self.__search = False
				self.__search_timer = 120

		if (order != None):

			self.__order = order
		# determine what heading we want to go

		if sensor_type.upper() == 'POSITION': # known positions of buoys

			### DEPRETIATED ###

			#print("Have seen buoys by position")

			self.__desired_heading = self.__heading_to_position(green_buoys, red_buoys)

		elif sensor_type.upper() == 'ANGLE': # camera sensor

			#print("Have seen buoys by angle")

			self.__desired_heading = self.__heading_to_angle(green_buoys, red_buoys)

		if (self.__search):

			if not((green_buoys != None) and (red_buoys != None)):

				if self.__search_direction == "left":

					return 25, 750

				else:

					return -25, 750

			else:

				self.__search = False

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
	def __heading_to_position (self, gnext = None, rnext = None):

		"""
		DEPRECATED
		used to set desired_heading
		target heading becomes self.__desired_heading
		"""

		# center of the next buoy pair
		tgt_hdg = self.__heading

		if not(gnext == None) and not(rnext == None):

			#print("Can see both")

			gate_center = ((gnext[0] + rnext[0]) / 2.0, (gnext[1] + rnext[1]) / 2.0)
			tgt_hdg = np.mod(np.degrees(np.arctan2(gate_center[0] - self.__position[0],
												   gate_center[1] - self.__position[1])) + 360, 360)

			#print(f"Target heading: {tgt_hdg}\nHeading: {self.__heading}\nDelta Heading: {tgt_hdg - self.__heading}")

		elif not(gnext == None) and rnext == None:

			#print("Can see Green")

			#if only one gate, set gate "center" to buoy location for temporary redirection
			self.__midpoint = (gnext[0], gnext[1])
			tgt_hdg = np.mod(np.degrees(np.arctan2(self.__midpoint[0] - self.__position[0],
												   self.__midpoint[1] - self.__position[1])) + 360,360)
		elif not(rnext == None) and gnext == None:

			#print("Can see Red")

			self.__midpoint  = (rnext[0], rnext[1])
			tgt_hdg = np.mod(np.degrees(np.arctan2(self.__midpoint[0] - self.__position[0],
												   self.__midpoint[1] - self.__position[1])) + 360,360)

		else:

			tgt_hdg = self.__heading

		# else, do nothing and go straight
		# heading to gate_center

		return tgt_hdg

	def __heading_to_angle(self, gnext = None, rnext = None):

		"""
		return desired heading based on angle relative to the AUV
		"""

		#relative angle to the center of the next buoy pair
		if (rnext == None) and (gnext == None):

			#print(f"No angle seen: {int(datetime.datetime.utcnow().timestamp())}")

			tgt_hdg = np.mod(self.__heading + 360, 360)

			self.__search = True

		elif (rnext != None) and (gnext != None):

			#print(f"Both angles seen: {int(datetime.datetime.utcnow().timestamp())}\n\tRed Angle: {rnext}\n\tGreen Angle: {gnext}")

			if ((self.__order == "gr") and (rnext <= gnext)):

				relative_angle = 90

			elif ((self.__order == "rg") and (gnext <= rnext)):

				relative_angle = 90

			else:

				relative_angle = (gnext + rnext) / 2.0

			tgt_hdg = np.mod(self.__heading + relative_angle + 360, 360)

		elif (rnext != None) and (gnext == None):

			#print(f"Only red seen: {int(datetime.datetime.utcnow().timestamp())}\n\tRed Angle: {rnext}")

			#print(f"Order: {self.__order}")

			#set tgt_hdg to heading of rnext
			if (self.__order == None):

				relative_angle = rnext

			elif (self.__order == "rg"):

				relative_angle = 90
				self.__search_direction = "right"

			else:

				relative_angle = -90
				self.__search_direction = "left"

			#print(f"Relative Angle is: {relative_angle}")

			tgt_hdg = np.mod(self.__heading + relative_angle + 360, 360)

			self.__search = True

			#print(f"Target is now: {tgt_hdg}")

		elif (gnext != None) and (rnext == None):

			#print(f"Only green seen: {int(datetime.datetime.utcnow().timestamp())}\n\tGreen Angle: {gnext}")

			#print(f"Order: {self.__order}")

			#set tgt_hdg to heading of rnext
			if (self.__order == None):

				relative_angle = gnext

			elif (self.__order == "gr"):

				relative_angle = 90
				self.__search_direction = "right"

			else:

				relative_angle = -90
				self.__search_direction = "left"

			#print(f"Relative Angle is: {relative_angle}")

			tgt_hdg = np.mod(self.__heading + relative_angle + 360, 360)
			self.__search = True

			#print(f"Target is now: {tgt_hdg}")

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

		#print(f"Delta Angle: {delta_angle}")

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
		rpm_speed = 750

		delta_angle *= (rpm_speed / 500)

		print(f"Delta Angle: {delta_angle}")
		#if delta_angle > 0:

		#	rpm_speed = int(np.round((1500 - (750 / (1 + 750 * np.exp(-0.5 * delta_angle)))), 0))

		#elif delta_angle < 0:

		#	rpm_speed = int(np.round((1500 - (750 / (1 + 750 * np.exp(0.5 * delta_angle)))), 0))

		#else:

		#	rpm_speed = 1500

		if np.abs(delta_angle) > 25:

			turn_angle = 25

		else:

			turn_angle = int(np.round(np.abs(delta_angle), 0))

		# which way do we have to turn
		if delta_angle < 0: # need to turn to right!

			turn_angle = (turn_angle * -1)

		elif delta_angle > 0: # need to turn to left!

			pass

		#adjust turn angle in comparison to previous rudder angle
		rudder_turn = turn_angle

		#print(f"New Rudder is: {rudder_turn}\nNew Speed is: {rpm_speed}")

		return rudder_turn, rpm_speed
