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


#COMMOM IMPORTS
from Common.VN_config import VN_config
from Common.VN_util import *


class TopCode(object):



	def __init__(self):
		#load algorithm constants
		#how round a circle needs to be. Perfect circle = 1
		self.eccentricity = VN_config.get_float('algorithm', 'eccentricity', 0.6)
		#Minimum ratio while comparing contour area to ellipse area
		self.area_ratio = VN_config.get_float('algorithm','area_ratio', 0.8)
		#Minimum ratio between outer and inner circle area in a ring
		self.ring_ratio = VN_config.get_float('algorithm','ring_ratio', 0.1)
		#The smallest span of min max pixels that get enhanced(Largest range is 255, Smaller numbers make image suspitable to noise)
		self.min_range = VN_config.get_integer('algorithm', 'min_range', 10)
		#reduce the grayscale resoltion(steps) by this multipler( 1 is full res, 2 is half res, 4 is quarter res )
		self.res_reduction = VN_config.get_integer('algorithm', 'res_reduction', 32)



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

		#cv2.imshow('norm',img)

		#check for a colored image
		if(len(img.shape)>2):
			#grayscale image
			img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)


		# Blur image
		blur = cv2.GaussianBlur(img,(3,3),0)

		#fix exposure, contrast, brightness
		filtered = self.balance(blur,self.ring_ratio, self.res_reduction)

		'''
		cv2.imshow("filtered",filtered)
		cv2.waitKey(1)

		#average brightness
		avg, null, null, null = cv2.mean(scal)
		avg = int(avg)

		#threshold
		ret,filter = cv2.threshold(img,thres,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)

		#dilate
		kernel = np.ones((5,5),np.uint8)
		filter = cv2.morphologyEx(filter,cv2.MORPH_CLOSE, kernel, borderType=cv2.BORDER_CONSTANT)
		cv2.imshow('thresh',filter)

		#canny edge detector
		edges = cv2.Canny(scal,avg/2,avg)
		cv2.imshow('edges',edges)
		'''
		#locate contours
		contours, hierarchy = cv2.findContours(filtered,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		#turn contours into circles
		circles = np.empty((len(contours)),object)
		circlesCnt = 0
		for i in xrange(0,len(contours)):
			contour = contours[i]

			circle = self.fit_circle(contour, self.eccentricity, self.area_ratio)
			if circle is not None:
				circles[circlesCnt] = circle
				circlesCnt += 1
		circles = np.resize(circles,circlesCnt)

		'''
		print "circles",circlesCnt


		#debug
		cimg = cv2.cvtColor(filtered,cv2.COLOR_GRAY2BGR)
		for cir in circles:
			cv2.circle(cimg, cir[0],cir[1], (0,0,255), thickness=4)

		rimg = cv2.cvtColor(filtered,cv2.COLOR_GRAY2BGR)
		'''
		rings = None
		#turn circles into rings
		if circlesCnt > 0:
			rings = self.detect_rings(circles,self.ring_ratio)

			'''
			print "rings",len(rings)

			#debug
			for ring in rings:
				cv2.circle(rimg,ring[0][0],ring[0][1],(255,0,0), thickness=4)
				cv2.circle(rimg,ring[1][0],ring[1][1],(255,0,0), thickness=4)
			cv2.imshow("rings",rimg)
			cv2.imshow("circles", cimg)
			cv2.waitKey(1)
			'''
		center = (-1,-1)
		if rings is not None and len(rings) > 0:
			for r in rings:
				center = center[0] + r[2][0], center[1] + r[2][1]
			center = center[0] / len(rings) , center[1] / len(rings)

		stop = current_milli_time()
		#print "time", stop-start
		return (frame_id, stop-start,center, 0, rings)


	#distCenters - distance between two circles
	def distCenters(self, circle1,circle2):
		#distance between centers
		distance = math.sqrt(math.pow((circle1[0][0]-circle2[0][0]),2) + math.pow((circle1[0][1] - circle2[0][1]),2))
		return distance

	#detectNested- return circles which are nested within other circles
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
					radius1 = circle1[1]
					radius2 = circle2[1]

					distance = self.distCenters(circle1,circle2)

					#check if a circle is nested within another circle
					if distance < abs(radius1 - radius2):
						if (radius1 < radius2) and (radius1 * 1.0 /radius2 > ratio):
							#small circle, big circle
							center = (circle1[0][0] + circle2[0][0])/2 , (circle1[0][1] + circle2[0][1])/2
							rings[ring_count] = (circle1, circle2,center)
							ring_count += 1
							break

						elif (radius1 > radius2) and (radius2 * 1.0 / radius1 > ratio):
							center = (circle1[0][0] + circle2[0][0])/2 , (circle1[0][1] + circle2[0][1])/2
							rings[ring_count] = (circle2, circle1,center)
							ring_count += 1
							break
		#remove null objects
		rings  = np.resize(rings,ring_count)

		return rings

	def fit_circle(self, contour, eccentricity, area_ratio):
		hull = cv2.convexHull(contour)

		#check for a shape
		if len(hull) > 4 and cv2.contourArea(hull) > 40:

			#fit an ellipse
			ellipse = cv2.fitEllipse(hull)
			#check for a circular ellipse
			if ellipse[1][0] * 1.0/ ellipse[1][1] > eccentricity:
				#compare area of raw hull vs area of ellipse to ellinate objects with corners
				e_area = (ellipse[1][0]/2.0) * (ellipse[1][1]/2.0) * math.pi
				c_area = cv2.contourArea(hull)
				if (c_area / e_area) > area_ratio:
					center = int(ellipse[0][0]), int(ellipse[0][1])
					radius = int((ellipse[1][0] + ellipse[1][0]) /4.0) #average  and diameter -> radius
					return (center, radius)
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


if __name__ == "__main__":

	#cam = cv2.VideoCapture(0)
	cam = flow_cam
	detector = TopCode()

	if cam is not None:
		while True:
			ret, img = cam.read()
			if(img is not None and ret == True):
				results = detector.analyze_frame(img)
				print results[1]

				rend_Image = gui.add_ring_highlights(img, results[3])
				#show results
				cv2.imshow('gui', rend_Image)
				cv2.waitKey(1)
				print ('RunTime: {0} Center: {1} Distance: {2} Raw Target: {3}'.format(results[0],results[1],results[2],results[3]))

			else:
				print "failed to grab image"
