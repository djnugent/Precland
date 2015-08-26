#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
import cv2
import numpy as np

#for running outside of mavproxy
if __name__ == "__main__":
	import sys
	import os
	import inspect
	#Add script directory to path
	script_dir =  os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) # script directory
	script_dir = script_dir.replace('PrecisionLand_lib','')
	sys.path.append(script_dir)

	from Common.Flow_Camera import flow_cam
	from PrecisionLand_lib.PL_gui import PrecisionLandGUI as gui
	from Common.ImageRW import ImageWriter


#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_util import *


class Ring_Detector(object):



	def __init__(self):
		#load algorithm constants
		#how round a circle needs to be. Perfect circle = 1
		self.eccentricity = VN_config.get_float('algorithm', 'eccentricity', 0.8)
		#Minimum ratio while comparing contour area to ellipse area
		self.area_ratio = VN_config.get_float('algorithm','area_ratio', 0.8)
		#Minimum ratio between outer and inner circle area in a ring
		self.ring_ratio = VN_config.get_float('algorithm','ring_ratio', 0.8)
		#The smallest span of min max pixels that get enhanced(Largest range is 255, Smaller numbers make image suspitable to noise)
		self.min_range = VN_config.get_integer('algorithm', 'min_range', 10)
		#reduce the grayscale resoltion(steps) by this multipler( 1 is full res, 2 is half res, 4 is quarter res )
		self.res_reduction = VN_config.get_integer('algorithm', 'res_reduction',1)



	#analyze_frame_async - process an frame and look for a bullseye asynchronously
	#params -child_conn: child pipe connection
	#		-img: raw image to be processed
	#return -runtime: time in millis to process an image
	#		-center: tuple(x,y) of the objects position on; 'None' when no target
	#		-distance: distance in meters to the target; -1 when unable to calculate
	#		-targetEllipses: ellipses that compose the detected target 'None' when no target
	def analyze_frame_async(self, child_conn, img, frame_id):
		child_conn.send(self.analyze_frame(img,frame_id))


	#analyze_frame - process an frame and look for a bullseye
	#params -child_conn: child pipe connection
	#		-img: raw image to be processed
	#return -runtime: time in millis to process an image
	#		-center: tuple(x,y) of the objects position on; 'None' when no target
	#		-distance: distance in meters to the target; -1 when unable to calculate
	#		-targetEllipses: ellipses that compose the detected target 'None' when no target
	def analyze_frame(self, img, frame_id):
		#start timer
		start = current_milli_time()


		#check for a colored image
		if(len(img.shape)>2):
			#grayscale image
			img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		# Blur image
		blur = cv2.GaussianBlur(img,(3,3),0)
		#blur = cv2.medianBlur(img,3)

		#fix exposure, contrast, brightness
		filtered = self.balance(blur,self.ring_ratio, self.res_reduction)
		#filtered = blur
		cv2.imshow("filtered",filtered)


		#average brightness
		avg, null, null, null = cv2.mean(filtered)
		avg = int(avg)

		#canny edge detector
		filtered = cv2.Canny(filtered,avg/2,avg)
		#filtered = self.auto_canny(filtered)
		cv2.imshow('edges',filtered)

		#detect circles
		circles = self.detect_circles(filtered,self.eccentricity, self.area_ratio, 2)

		rings = []
		best_ring = None
		#turn circles into rings
		if len(circles) > 0:
			rings = self.detect_rings(circles,self.ring_ratio)

		#find smallest ring with orientation(if available)
		if len(rings) > 0:
			best_ring = self.best_target(rings)

		stop = current_milli_time()
		return (frame_id, stop-start,best_ring,rings)



	#detect_rings- Find circles that are nested inside of each other
	def detect_rings(self,rawCircles,ratio):
		size = len(rawCircles)
		rings = np.empty(size, object)
		ring_count = 0
		for i in xrange(0,size):
			for j in xrange(i, size):
				if i != j:
					circle1 = rawCircles[i]
					circle2 = rawCircles[j]
					#average major and minor axises
					radius1 = circle1.radius
					radius2 = circle2.radius

					distance = circle1.center.distance_to(circle2.center)

					#check if a circle is nested within another circle
					if distance < abs(radius1 - radius2):
						if (radius1 < radius2) and (radius1 * 1.0 /radius2 > ratio):
							#small circle, big circle
							rings[ring_count] = Ring(circle1, circle2)
							ring_count += 1
							break

						elif (radius1 > radius2) and (radius2 * 1.0 / radius1 > ratio):
							#small circle, big circle
							rings[ring_count] = Ring(circle2, circle1)
							ring_count += 1
							break
		#remove null objects
		rings  = np.resize(rings,ring_count)

		return rings


	def detect_circles(self, orig, eccentricity, area_ratio, min_radius = 0, max_radius = 500000):

		img = np.copy(orig)

		#locate contours
		contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		cv2.imshow("cont",img)

		#turn contours into circles
		circles = np.empty((len(contours)),object)
		circlesCnt = 0
		for i in xrange(0,len(contours)):
			contour = contours[i]

			circle = self.fit_circle(contour, eccentricity, area_ratio, min_radius, max_radius)
			if circle is not None:
				circles[circlesCnt] = circle
				circlesCnt += 1
		circles = np.resize(circles,circlesCnt)
		return circles




	def fit_circle(self, contour, eccentricity, area_ratio,min_radius = 0, max_radius = 500000):
		#convert to convex hull
		hull = cv2.convexHull(contour)
		min_area = math.pi * min_radius * min_radius
		max_area = math.pi * max_radius * max_radius
		contour_area = cv2.contourArea(hull)

		#check for a shape of a certain size and corner resolution
		if len(hull) > 4 and contour_area > min_area and contour_area < max_area:

			#fit an ellipse
			ellipse = cv2.fitEllipse(hull)
			#check for a circular ellipse
			if ellipse[1][0] * 1.0/ ellipse[1][1] > eccentricity:
				#compare area of raw hull vs area of ellipse to ellinate objects with corners
				e_area = (ellipse[1][0]/2.0) * (ellipse[1][1]/2.0) * math.pi
				c_area = cv2.contourArea(hull)
				if (c_area / e_area) > area_ratio:
					center = Point(int(ellipse[0][0]), int(ellipse[0][1]))
					radius = int((ellipse[1][0] + ellipse[1][0]) /4.0) #average  and diameter -> radius
					return Circle(center,radius,contour)
		return None

	#balance - improve contrast and adjust brightness in image
	#Created By: Tobias Shapinsky
	def balance(self,orig,min_range,res_reduction):

		img = np.copy(orig)
		#get min, max and range of image
		min_v = np.percentile(img,5)
		max_v = np.percentile(img,95)

		#clip extremes
		img.clip(min_v,max_v, img)

		#scale image so that brightest pixel is 255 and darkest is 0
		range_v = max_v - min_v
		if(range_v > min_range):
			img -= min_v
			img *= (255.0/(range_v))
			img /= res_reduction
			img *= res_reduction
			return img
		else:
			return np.zeros((img.shape[0],img.shape[1]), np.uint8)


	def auto_canny(image, sigma=0.33):
		# compute the median of the single channel pixel intensities
		v = np.median(image)

		# apply automatic Canny edge detection using the computed median
		lower = int(max(0, (1.0 - sigma) * v))
		upper = int(min(255, (1.0 + sigma) * v))
		edged = cv2.Canny(image, lower, upper)

		# return the edged image
		return edged


	#smallest oriented target
	def best_target(self, rings):
		min_rad = 99999999
		min_index = 0
		has_orient = False
		best_target = None
		for r in rings:
			if has_orient == False:
				if r.is_valid():
					best_target = r
					min_rad = best_target.radius
					has_orient = True
				elif r.radius < min_rad:
					best_target = r
					min_rad = best_target.radius

			elif has_orient == True:

				if r.is_valid() and r.radius < min_rad:
					best_target = r
					min_rad = best_target.radius

		return best_target


class Point(object):
	def __init__(self, x = None, y = None, tup = None):
		if tup is not None:
			self.x = tup[0]
			self.y = tup[1]
		else:
			self.x = x
			self.y = y

	def distance_to(self,other):
		return math.sqrt(math.pow(self.x - other.x,2)+ math.pow(self.y - other.y,2))

	def angle_to(self,other):
		#image coordinates
		diff_x = other.x - self.x
		diff_y = self.y -other.y
		return math.atan2(diff_x,diff_y) #NED coordinates

	def tuple(self):
		return (self.x,self.y)

	def __add__(self,other):
		return Point(self.x + other.x, self.y + other.y)

	def __sub__(self,other):
		return Point(self.x - other.x, self.y - other.y)

	def __str__(self):
		return "({0}, {1})".format(self.x,self.y)


class Circle(object):

	def __init__(self, center, radius, contour = None):
		self.center = center
		self.radius = radius
		self.contour = contour

	def __str__(self):
		return "Center: {0} Radius: {1}".format(self.center,self.radius)


class Ring(object):
	def __init__(self, inner_circle, outer_circle, center = None, radius = None, orientation = None):
		self.inner_circle = inner_circle
		self.outer_circle = outer_circle
		self.center = center
		self.radius = radius
		self.orientation = orientation
		self.calc_radius(0)
		self.calc_center(0)
		self.calc_orientation()

	def calc_orientation(self, bump_ratio = 0.9):
		inner_contour = self.inner_circle.contour
		min_radius = self.inner_circle.radius * 1.2
		closest_vertex = None
		actual_bump_ratio = 1

		#look for the point closest to the center
		for vertex in inner_contour:
			vertex = Point(tup = vertex[0])
			dist = vertex.distance_to(self.inner_circle.center)
			if dist < min_radius and (dist * 1.0 / self.inner_circle.radius) < bump_ratio:
				min_radius = dist
				closest_vertex = vertex

		if closest_vertex is not None:
			self.orientation = self.inner_circle.center.angle_to(closest_vertex)

	def calc_radius(self, type):
		if type == 0:
			self.radius = self.inner_circle.radius
		elif type == 1:
			self.radius = self.outer_circle.radius
		elif type == 2:
			self.radius = (self.inner_circle.radius + self.outer_circle.radius) / 2

	def calc_center(self, type):
		if type == 0:
			self.center = self.inner_circle.center
		elif type == 1:
			self.center = self.outer_circle.center
		elif type == 2:
			self.center = (self.inner_circle.center + self.outer_circle.center) / 2

	def get_angular_offset(self, img_width, img_height, hfov, vfov):
		x_pixel = self.center.x - (img_width/2.0)
		y_pixel = self.center.y - (img_height/2.0) #y-axis is inverted??? Works with arducopter

		#convert target location to angular radians
		x_angle = x_pixel * (hfov / img_width) * (math.pi/180.0)
		y_angle = y_pixel * (vfov / img_height) * (math.pi/180.0)
		return (x_angle,y_angle, self.orientation)

	def is_valid(self):
		return (self.orientation is not None)

	def __str__(self):
		return "Center: {0} Radius: {1} Orientation: {2} radians".format(self.center,self.radius,self.orientation)



if __name__ == "__main__":

	#cam = cv2.VideoCapture(0)
	cam = flow_cam
	#writer = ImageWriter("/home/daniel/test_footage_midday")
	detector = Ring_Detector()
	frame_id = 0
	if cam is not None:
		while True:
			ret, img = cam.read()
			if(img is not None and ret == True):
				results = detector.analyze_frame(img,frame_id)
				frame_id += 1

				#show results
				rend_Image = gui.add_ring_highlights(img, results[3],results[2])
				yaw , radius = None, None
				if results[2] is not None:
					radius = results[2].radius
					if results[2].is_valid():
						yaw = int(math.degrees(results[2].orientation ))

				status_text = '{0} Rings\n{1} ms\n{2} degs\n{3} radius\n{4} meters'.format(len(results[3]), results[1], yaw, radius, 0)
				rend_Image = gui.add_stats(rend_Image,status_text,5, 250)
				cv2.imshow('gui', rend_Image)
				cv2.waitKey(1)
				print status_text.replace('\n', ', ')

			else:
				print "failed to grab image"
