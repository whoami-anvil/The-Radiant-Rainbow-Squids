#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Jul 17 16:18:55 2021

This is the simulated Sandshark front seat


@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import csv
import time
import threading
import datetime

from AUV_Controller import AUVController
from Camera_Interpreter import Interpreter

from pynmea2 import pynmea2
import BluefinMessages
from Sandshark_Interface import SandsharkClient

reader = None
writer = None

class BackSeat ():

	# we assign the mission parameters on init
	def __init__ (self, host = 'localhost', port = 8000, warp = 1):

		# back seat acts as client
		self.__client = SandsharkClient(host = host, port = port)
		self.__current_time = time.time()
		self.__start_time = self.__current_time
		self.__warp = warp

		self.__autonomy = AUVController()
		self.__interpreter = Interpreter()

	def run (self):

		try:
			# connect the client
			client = threading.Thread(target=self.__client.run, args=())
			client.start()

			msg = BluefinMessages.BPLOG('ALL', 'ON')
			self.send_message(msg)

			### These flags are for the test code. Remove them after the initial test!
			engine_started = False
			turned = False

			while True:

				now = time.time()
				delta_time = (now - self.__current_time) * self.__warp

				self.send_status()
				self.__current_time += delta_time

				msgs = self.get_mail()

				if len(msgs) > 0:

					print("\nReceived from Frontseat:")

					for msg in msgs:

						print(f"{str(msg, 'utf-8')}")

				time.sleep(1 / self.__warp)

				picture = self.__interpreter.take_picture()

				self.__interpreter.calculate_values(picture)

				# Logging state

				writer.write([
					{'Timestamp (UTC)' : datetime.datetime.utcnow(),
					 'Position' : self.__autonomy.get_position(),
					 'Current Heading (deg)' : self.__autonomy.get_current_heading(),
					 'Desired Heading (deg)' : self.__autonomy.get_desired_heading(),
					 'Green Bouys' : self.__interpreter.get_green_bouy_positions(),
					 'Red Bouys' : self.__interpreter.get_red_bouy_positions(),
					 'Error': "None"}
				])

				### self.__autonomy.decide() probably goes here!

				rudder_command, engine_command = self.__autonomy.decide()

				### turn your output message into a BPRMB request!

				bluefin_message = self.convert_commands(rudder_command, engine_command)

				# ------------------------------------------------------------ #
				# ----This is example code to show commands being issued
				# ------------------------------------------------------------ #

				print(f"{self.__current_time - self.__start_time}")

				if not engine_started and (self.__current_time - self.__start_time) > 30:

					## We want to change the speed. For now we will always use the RPM (1500 Max)
					self.__current_time = time.time()

					# This is the timestamp format from NMEA: hhmmss.ss
					hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]

					cmd = f"BPRMB,{hhmmss},,,,750,0,1"

					# NMEA requires a checksum on all the characters between the $ and the *
					# you can use the BluefinMessages.checksum() function to calculate
					# and write it like below. The checksum goes after the *
					msg = f"${cmd}*{hex(BluefinMessages.checksum(cmd))[2:]}"
					self.send_message(msg)
					engine_started = True

				if not turned and (self.__current_time - self.__start_time) > 60:

					## We want to set the rudder position, use degrees plus or minus
					## This command is how much to /change/ the rudder position, not to
					## set the rudder
					self.__current_time = time.time()
					hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]

					cmd = f"BPRMB,{hhmmss},15,,,750,0,1"
					msg = f"${cmd}*{hex(BluefinMessages.checksum(cmd))[2:]}"
					self.send_message(msg)
					turned = True

				# ------------------------------------------------------------ #
				# ----End of example code
				# ------------------------------------------------------------ #

		except:

			self.__client.cleanup()
			client.join()

	def convert_commands (self, rudder_command, engine_command):



		pass

	def process_message (self, msg):

		# DEAL WITH INCOMING BFNVG MESSAGES AND USE THEM TO UPDATE THE
		# STATE IN THE CONTROLLER!

		cmd = pynmea2.parse(msg)

		self.__autonomy.update_state(cmd)

		pass

	def send_message (self, msg):

		print(f"sending message {msg}...")
		self.__client.send_message(msg)

	def send_status (self):

		#print("sending status...")
		self.__current_time = time.time()
		hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]
		msg = BluefinMessages.BPSTS(hhmmss, 1, 'BWSI Autonomy OK')
		self.send_message(msg)

	def get_mail (self):

		msgs = self.__client.receive_mail()

		return msgs

	def log_error (self, error_msg):

		writer.write([
			{'Timestamp (UTC)' : datetime.datetime.utcnow(),
			 'Position' : None,
			 'Current Heading (deg)' : None,
			 'Desired Heading (deg)' : None,
			 'Green Bouys' : None,
			 'Red Bouys' : None,
			 'Error': error_msg}
		])

		self.__current_time = time.time()
		hhmmss = datetime.datetime.fromtimestamp(self.__current_time).strftime('%H%M%S.%f')[:-4]
		msg = BluefinMessages.BPABT(hhmmss, error_msg)
		self.send_message(msg)

def main():

	if len(sys.argv) > 1:

		host = sys.argv[1]

	else:

		host = "localhost"

	if len(sys.argv) > 2:

		port = int(sys.argv[2])

	else:

		port = 8042

	log_file_name = f"mission_{datetime.datetime.now()}.csv"
	log_file_write = open(log_file_name, "w", encoding = 'UTF8', newline = '')
	log_file_read = open(log_file_name, "r", encoding = 'UTF8', newline = '')
	writer = csv.DictWriter(log_file_write, fieldnames = ['Timestamp (UTC)', 'Position', 'Current Heading (deg)', 'Desired Heading (deg)', 'Green Bouys', 'Red Bouys', 'Error'])
	writer.writeheader()
	reader = csv.DictReader(log_file_read)

	print(reader)
	print(len(reader))

	print(f"host = {host}, port = {port}")
	backseat = BackSeat(host = host, port = port)
	backseat.run()

if __name__ == '__main__':
	main()