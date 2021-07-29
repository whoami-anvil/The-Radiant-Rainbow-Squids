#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 11:30:37 2021

@author: BWSI AUV Challenge Instructional Staff
"""
### JRE: for simulation only!
### MDM: added Rasperry Pi V2 camera

import os
import sys
import pathlib
import datetime

import time
import numpy as np

if os.uname().nodename == 'auvpi':

	import picamera
	import picamera.array

import cv2
import matplotlib.pyplot as plt
# For simulations
from BWSI_BuoyField import BuoyField
from BWSI_Sensor import BWSI_Camera


class ImageProcessor():
	def __init__(self, camera='SIM', log_dir='./'):
		self.__camera_type = camera.upper()

		if self.__camera_type == 'SIM':
			self.__camera = BWSI_Camera(max_angle=31.1, visibility=50)
			self.__simField = None

		else:
			self.__camera = picamera.PiCamera()
			self.__camera.resolution = (640, 480)
			self.__camera.framerate = 24
			time.sleep(2) # camera warmup time
			self.__image = np.empty((480*640*3,), dtype=np.uint8)

		# create my save directory
		self.__image_dir = pathlib.Path(log_dir, 'frames')
		self.__image_dir.mkdir(parents=True, exist_ok=True)


	# ------------------------------------------------------------------------ #
	# Run an iteration of the image processor.
	# The sim version needs the auv state to generate simulated imagery
	# the PICAM does not need any auv_state input
	# ------------------------------------------------------------------------ #
	def px_to_mm (self, value, px, side = "W"):

		if (side == "W"):

			return (value - (0.5 * px)) * (3.68 / px)

		elif (side == "H"):

			return ((0.5 * px) - value) * (2.76 / px)

	def sensor_position_m (self, pix_x, pix_y, res_x, res_y):

		x = pix_x - (0.5 * res_x)
		y = (0.5 * res_y) - pix_y

		x_prop = (3.68 / 1000) / res_x
		y_prop = (2.76 / 1000) / res_y

		sensor_pos_x = x * x_prop
		sensor_pos_y = (y * y_prop)

		return (sensor_pos_x, sensor_pos_y)

	def sensor_position_px (self, pix_x, pix_y, res_x, res_y):

		x = pix_x - (0.5 * res_x)
		y = (0.5 * res_y) - pix_y

		return (x, y)

	def get_angles (self, x, y):

		f = (3.04 / 1000)

		hor_ang = np.degrees(np.arctan2(x, f))
		vert_ang = np.degrees(np.arctan2(y, f))

		return hor_ang, vert_ang

	def run(self, auv_state = None):

		red = None
		green = None
		order = None

		if auv_state['heading'] is not None:
			if (self.__camera_type == 'SIM'):
				# if it's the first time through, configure the buoy field
				if self.__simField is None:
					self.__simField = BuoyField(auv_state['datum'])
					config = {'nGates': 5,
							  'gate_spacing': 5,
							  'gate_width': 2,
							  'style': 'pool_1',
							  'max_offset': 5,
							  'heading': 0}

					self.__simField.configure(config)

				# synthesize an image
				image = self.__camera.get_frame(auv_state['position'], auv_state['heading'], self.__simField)

			elif self.__camera_type == 'PICAM':

				try:

					self.__camera.capture(self.__image, 'bgr')

				except:

					# restart the camera
# =============================================================================
#                     self.__camera = picamera.PiCamera()
#                     self.__camera.resolution = (640, 480)
#                     self.__camera.framerate = 24
# =============================================================================
					time.sleep(2) # camera warmup time

				image = self.__image.reshape((480, 640, 3))

			else:
				print(f"Unknown camera type: {self.__camera_type}")
				sys.exit(-10)

			# log the image
			fn = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}.jpg"
			fns = self.__image_dir / f"frame_{int(datetime.datetime.utcnow().timestamp())}_solved.jpg"
			cv2.imwrite(str(fn), image)

			# process and find the buoys!
			# loads image, resizes, and applies box filter
			img = np.flipud(cv2.imread(str(fn))).copy()
			img_brg_filtered = cv2.boxFilter(img, -1, (10,10))

			# puts image thresholds using RGB makes 3 boolean arrays depending on if pixel meets thresholds
			img_threshold_blue = np.logical_not(np.logical_and(img_brg_filtered[:, :, 0] >= 210, img_brg_filtered[:, :, 0] <= 255))
			img_threshold_green = np.logical_and(img_brg_filtered[:, :, 1] >= 197, img_brg_filtered[:, :, 1] <= 255)
			img_threshold_red = np.logical_and(img_brg_filtered[:, :, 2] >= 28, img_brg_filtered[:, :, 1] <= 140)

			# combines all three logical statements to find pixels that meet all three conditions
			walls = np.logical_and(img_threshold_blue, np.logical_not(img_threshold_green))
			img_green_buoys = img_threshold_green
			img_red_buoys = img_threshold_red

			#fig, ax = plt.subplots(3, 3)

			#ax[0, 0].imshow(img[:, :, 0], cmap = "Blues")
			#ax[0, 0].title.set_text("Blue")
			#ax[0, 1].imshow(img[:, :, 1], cmap = "Greens")
			#ax[0, 1].title.set_text("Green")
			#ax[0, 2].imshow(img[:, :, 2], cmap = "Reds")
			#ax[0, 2].title.set_text("Red")

			#ax[1, 0].imshow(img_brg_filtered[:, :, 0], cmap = "Blues")
			#ax[1, 0].title.set_text("Blue")
			#ax[1, 1].imshow(img_brg_filtered[:, :, 1], cmap = "Greens")
			#ax[1, 1].title.set_text("Green")
			#ax[1, 2].imshow(img_brg_filtered[:, :, 2], cmap = "Reds")
			#ax[1, 2].title.set_text("Red")

			#ax[2, 0].imshow(img_threshold_blue, cmap = "Blues")
			#ax[2, 0].title.set_text("Blue")
			#ax[2, 1].imshow(img_threshold_green, cmap = "Greens")
			#ax[2, 1].title.set_text("Green")
			#ax[2, 2].imshow(img_threshold_red, cmap = "Reds")
			#ax[2, 2].title.set_text("Red")

			#plt.show()

			#steps 13/14 of lab 15. looks at pixel intensity values of it's neighborhood and scales to 255
			contours_red = []
			contours_green = []
			img_points_red = cv2.boxFilter(img_red_buoys.astype(int), -1, (50, 50), normalize = False)
			img_points_green = cv2.boxFilter(img_green_buoys.astype(int), -1, (50, 50), normalize = False)

			if not(np.max(img_points_red) == 0):

				img_points_red_value_scaled = (img_points_red * (255 / np.max(img_points_red))).astype(np.uint8)
				thresh_red, img_out_red = cv2.threshold(img_points_red_value_scaled, 20, 255, cv2.THRESH_BINARY)
				contour_img_red, contours_red, hierarchy_red = cv2.findContours(img_out_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

			if not(np.max(img_points_green) == 0):

				img_points_green_value_scaled = (img_points_green * (255 / np.max(img_points_green))).astype(np.uint8)
				thresh_green, img_out_green = cv2.threshold(img_points_green_value_scaled, 20, 255, cv2.THRESH_BINARY)
				contour_img_green, contours_green, hierarchy_green = cv2.findContours(img_out_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

			# makes empty lists to stor info about each contour (like making table out of lists)
			contour_dims_red = []
			contour_centers_red = []
			contour_centers_unscaled_red = []

			contour_dims_green = []
			contour_centers_green = []
			contour_centers_unscaled_green = []

			for contour in contours_red:

				# finds dimensions of buoy using max recorded x-value and y-value
				x = self.px_to_mm(np.max([item[0][0] for item in contour]), img.shape[1], "W") - self.px_to_mm(np.min([item[0][0] for item in contour]), img.shape[1], "W")
				y = self.px_to_mm(np.max([item[0][1] for item in contour]), img.shape[0], "H") - self.px_to_mm(np.min([item[0][1] for item in contour]), img.shape[0], "H")
				contour_dims_red.append((x, y))

				# finds center of each contour
				center_x, center_y = self.sensor_position_px(int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour])), img.shape[1], img.shape[0])
				contour_centers_unscaled_red.append((int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour]))))
				contour_centers_red.append((center_x, center_y))

			for contour in contours_green:

				x = self.px_to_mm(np.max([item[0][0] for item in contour]), img.shape[1], "W") - self.px_to_mm(np.min([item[0][0] for item in contour]), img.shape[1], "W")
				y = self.px_to_mm(np.max([item[0][1] for item in contour]), img.shape[0], "H") - self.px_to_mm(np.min([item[0][1] for item in contour]), img.shape[0], "H")
				contour_dims_green.append((x, y))

				center_x, center_y = self.sensor_position_px(int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour])), img.shape[1], img.shape[0])
				contour_centers_unscaled_green.append((int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour]))))
				contour_centers_green.append((center_x, center_y))

			red_index = None

			if ((len(contour_dims_red) > 0) and (len(contour_dims_green) > 0)): # checks if we have at least 1 contour detected

				if len(contour_dims_green) > len(contour_dims_red):

					green_index = [item[0] for item in contour_dims_green].index(max([item[0] for item in contour_dims_green]))
					green_center_m_x = (((3.68 / 1000) / img.shape[1]) * contour_centers_green[green_index][0])
					green_center_m_y = (((2.76 / 1000) * img.shape[0]) / contour_centers_green[green_index][1])
					img = cv2.circle(img, contour_centers_unscaled_green[green_index], 5, (0, 255, 0), -1)
					green_angle_x, green_angle_y = self.get_angles(green_center_m_x, green_center_m_y)
					green = green_angle_x

					red_index = [np.abs(np.abs(item[0]) - contour_dims_green[green_index][0]) for item in contour_dims_red].index(min([np.abs(np.abs(item[0]) - contour_dims_green[green_index][0]) for item in contour_dims_red]))
					red_center_m_x = (((3.68 / 1000) / img.shape[1]) * contour_centers_red[red_index][0])
					red_center_m_y = (((2.76 / 1000) * img.shape[0]) / contour_centers_red[red_index][1])
					img = cv2.circle(img, contour_centers_unscaled_red[red_index], 5, (0, 0, 255), -1)
					red_angle_x, red_angle_y = self.get_angles(red_center_m_x, red_center_m_y)
					red = red_angle_x

				else:

					# for each contour finds the index of the smallest contour
					red_index = [item[0] for item in contour_dims_red].index(max([item[0] for item in contour_dims_red]))

					# finds angle to the buoys
					red_center_m_x = (((3.68 / 1000) / img.shape[1]) * contour_centers_red[red_index][0])
					red_center_m_y = (((2.76 / 1000) * img.shape[0]) / contour_centers_red[red_index][1])
					img = cv2.circle(img, contour_centers_unscaled_red[red_index], 5, (0, 0, 255), -1)
					red_angle_x, red_angle_y = self.get_angles(red_center_m_x, red_center_m_y)
					red = red_angle_x

					green_index = [np.abs(np.abs(item[0]) - contour_dims_red[red_index][0]) for item in contour_dims_green].index(min([np.abs(np.abs(item[0]) - contour_dims_red[red_index][0]) for item in contour_dims_green]))

					green_center_m_x = (((3.68 / 1000) / img.shape[1]) * contour_centers_green[green_index][0])
					green_center_m_y = (((2.76 / 1000) * img.shape[0]) / contour_centers_green[green_index][1])
					img = cv2.circle(img, contour_centers_unscaled_green[green_index], 5, (0, 255, 0), -1)
					green_angle_x, green_angle_y = self.get_angles(green_center_m_x, green_center_m_y)
					green = green_angle_x

			if len(contour_dims_green) > 0:

				green_index = [item[0] for item in contour_dims_green].index(max([item[0] for item in contour_dims_green]))
				green_center_m_x = (((3.68 / 1000) / img.shape[1]) * contour_centers_green[green_index][0])
				green_center_m_y = (((2.76 / 1000) * img.shape[0]) / contour_centers_green[green_index][1])
				img = cv2.circle(img, contour_centers_unscaled_green[green_index], 5, (0, 255, 0), -1)
				green_angle_x, green_angle_y = self.get_angles(green_center_m_x, green_center_m_y)
				green = green_angle_x

			if len(contour_dims_red) > 0:

				# for each contour finds the index of the smallest contour
				red_index = [item[0] for item in contour_dims_red].index(max([item[0] for item in contour_dims_red]))

				# finds angle to the buoys
				red_center_m_x = (((3.68 / 1000) / img.shape[1]) * contour_centers_red[red_index][0])
				red_center_m_y = (((2.76 / 1000) * img.shape[0]) / contour_centers_red[red_index][1])
				img = cv2.circle(img, contour_centers_unscaled_red[red_index], 5, (0, 0, 255), -1)
				red_angle_x, red_angle_y = self.get_angles(red_center_m_x, red_center_m_y)
				red = red_angle_x

			cv2.imwrite(str(fns), img)

			if (red != None) and (green != None):

				if (red < green):

					order = "rg"

				else:

					order = "gr"

		return red, green, order
