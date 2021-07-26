#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 22 11:30:37 2021

@author: BWSI AUV Challenge Instructional Staff
"""
### JRE: for simulation only!

import sys
import time
import pathlib
import datetime

import cv2

import numpy as np
import matplotlib.pyplot as plt

# For simulations
from BWSI_BuoyField import BuoyField
from BWSI_Sensor import BWSI_Camera
#from picamera import PiCamera


class ImageProcessor():

	def __init__(self, camera = 'PICAM', log_dir = './'):

		self.__camera_type = camera.upper()

		if self.__camera_type == 'SIM':

			self.__camera = BWSI_Camera(max_angle = 31.1, visibility = 30)
			self.__simField = None

		else:

			self.__camera = PiCamera()
			self.__camera.start_preview()

			time.sleep(2)

			pass

		# create my save directory
		self.__image_dir = pathlib.Path(log_dir, 'frames')
		self.__image_dir.mkdir(parents = True, exist_ok = True)

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

		hor_ang = np.degrees(np.arctan(x / f))
		vert_ang = np.degrees(np.arctan(y / f))

		return hor_ang, vert_ang

	# ------------------------------------------------------------------------ #
	# Run an iteration of the image processor.
	# The sim version needs the auv state ot generate simulated imagery
	# the PICAM does not need any auv_state input
	# ------------------------------------------------------------------------ #
	def run(self, auv_state = None):

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

				# log the image
				fn = f"{self.__image_dir}/{int(datetime.datetime.utcnow().timestamp())}.jpg"
				cv2.imwrite(str(fn), image)

			elif self.__camera_type == 'PICAM':

				fn = f"{self.__image_dir}/{int(datetime.datetime.utcnow().timestamp())}.jpg"
				self.__camera.capture(fn)

				pass

			else:

				print(f"Unknown camera type: {self.__camera_type}")
				sys.exit(-10)

			img = cv2.imread(fn)
			img = cv2.resize(img, (640, 480))
			img_brg_filtered = cv2.boxFilter(img, -1, (10,10))

			img_threshold_blue = np.logical_not(np.logical_and(img_brg_filtered[:, :, 0] >= 210, img_brg_filtered[:, :, 0] <= 255))
			img_threshold_green = np.logical_and(img_brg_filtered[:, :, 1] >= 140, img_brg_filtered[:, :, 1] <= 255)
			img_threshold_red = np.logical_and(img_brg_filtered[:, :, 2] >= 20, img_brg_filtered[:, :, 1] <= 140)

			img_red_buoys = np.logical_and(img_threshold_green, img_threshold_blue)
			img_green_buoys = np.logical_and(img_threshold_red, img_threshold_blue)

			fig, ax = plt.subplots(3, 3)

			ax[0, 0].imshow(img[:, :, 0], cmap = "Blues")
			ax[0, 0].title.set_text("Blue")
			ax[0, 1].imshow(img[:, :, 1], cmap = "Greens")
			ax[0, 1].title.set_text("Green")
			ax[0, 2].imshow(img[:, :, 2], cmap = "Reds")
			ax[0, 2].title.set_text("Red")

			ax[1, 0].imshow(img_brg_filtered[:, :, 0], cmap = "Blues")
			ax[1, 0].title.set_text("Blue")
			ax[1, 1].imshow(img_brg_filtered[:, :, 1], cmap = "Greens")
			ax[1, 1].title.set_text("Green")
			ax[1, 2].imshow(img_brg_filtered[:, :, 2], cmap = "Reds")
			ax[1, 2].title.set_text("Red")

			ax[2, 0].imshow(img_threshold_blue, cmap = "Blues")
			ax[2, 0].title.set_text("Blue")
			ax[2, 1].imshow(img_threshold_green, cmap = "Greens")
			ax[2, 1].title.set_text("Green")
			ax[2, 2].imshow(img_threshold_red, cmap = "Reds")
			ax[2, 2].title.set_text("Red")

			plt.show()

			img_points_red = cv2.boxFilter(img_red_buoys.astype(int), -1, (50, 50), normalize = False)
			img_points_red_value_scaled = (img_points_red * (255 / np.max(img_points_red))).astype(np.uint8)

			img_points_green = cv2.boxFilter(img_green_buoys.astype(int), -1, (50, 50), normalize = False)
			img_points_green_value_scaled = (img_points_green * (255 / np.max(img_points_green))).astype(np.uint8)

			thresh_red, img_out_red = cv2.threshold(img_points_red_value_scaled, 217, 255, cv2.THRESH_BINARY)
			thresh_green, img_out_green = cv2.threshold(img_points_green_value_scaled, 217, 255, cv2.THRESH_BINARY)

			contours_red, hierarchy_red = cv2.findContours(img_out_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
			contours_green, hierarchy_green = cv2.findContours(img_out_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

			contour_dims_red = []
			contour_centers_red = []
			contour_distances_red = []

			contour_dims_green = []
			contour_centers_green = []
			contour_distances_green = []

			for contour in contours_red:

				x, y = self.sensor_position_m(np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour]), np.max([item[0][1] for item in contour]) - np.min([item[0][1] for item in contour]), img.shape[1], img.shape[0])
				contour_dims_red.append((x, y))

				img = cv2.circle(img, (int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour]))), 5, (0, 0, 255), -1)
				center_x, center_y = self.sensor_position_px(int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour])), img.shape[1], img.shape[0])
				contour_centers_red.append((center_x, center_y))

				contour_distances_red.append(-1 * (((3.04 * 250 * img.shape[1]) / (np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour]) * 3.68)) / 1000))

			for contour in contours_green:

				x, y = self.sensor_position_m(np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour]), np.max([item[0][1] for item in contour]) - np.min([item[0][1] for item in contour]), img.shape[1], img.shape[0])
				contour_dims_green.append((x, y))

				img = cv2.circle(img, (int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour]))), 5, (0, 0, 255), -1)
				center_x, center_y = self.sensor_position_px(int(np.mean([item[0][0] for item in contour])), int(np.mean([item[0][1] for item in contour])), img.shape[1], img.shape[0])
				contour_centers_green.append((x, y))

				contour_distances_green.append(-1 * (((3.04 * 250 * img.shape[1]) / (np.max([item[0][0] for item in contour]) - np.min([item[0][0] for item in contour]) * 3.68)) / 1000))

			print(f"Dims Red: {contour_dims_red}\nDims Green: {contour_dims_green}\nCenters Red: {contour_centers_red}\nCenters Green: {contour_centers_green}\nDistances Red: {contour_distances_red}\nDistances Green: {contour_distances_green}")

			if len(contour_distances_red) > 0:

				red_index = contour_distances_red.index(min(contour_distances_red))
				red_angle_x, red_angle_y = self.get_angles(contour_centers_red[red_index][0], contour_centers_red[red_index][1])
				red = ((np.tan(np.radians(red_angle_x)) * contour_distances_red[red_index]), contour_distances_red[red_index])
				print(f"Red Angle: {red_angle_x}")

			if len(contour_distances_green) > 0:

				green_index = contour_distances_green.index(min(contour_distances_green))
				print(f"Green index: {green_index}")
				print(f"Green x: {contour_centers_green[green_index][0]}, Green y: {contour_centers_green[green_index][1]}")
				green_angle_x, green_angle_y = self.get_angles(contour_centers_green[green_index][0], contour_centers_green[green_index][1])
				green = ((np.tan(np.radians(green_angle_x)) * contour_distances_green[green_index]), contour_distances_green[green_index])
				print(f"Green Angle: {green_angle_x}")

			plt.imshow(img)
			plt.show()

		return red, green
