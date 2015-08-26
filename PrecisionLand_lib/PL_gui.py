#!/usr/bin/python
import cv2
import numpy as np
import math

class PrecisionLandGUI(object):


	@classmethod
	#render_image - completes all gui operation according to config file
	def render_image(self, img, fps=None, target=None, distance=None, mode=None, vel=None, craftAttitude=None, locationCraft=None, locationTarget=None):
		pass

	@classmethod
	#add_predicted_location- place a marker on image where the target is suppossed to be based on GPS
	def add_predicted_location(self, img, locationCraft, locationTarget):
		pass

	@classmethod
	#add_artifical_horizon- place a bubble on image to indicate a line perpendicular to level
	def add_artifical_horizon(self, img, craftAttitude):
		pass

	@classmethod
	#add_mode_border- put a colored border around the image to signify the control the computer has
	def add_mode_border(self, mode):
		pass

	@classmethod
	#add_velocity_vectors- add vectors to show how the commands sent to the autopilot
	def add_velocity_vectors(self, vel):
		pass

	@classmethod
	#add_target_highlight- highlight the detected target, 1.the target center 2.each ring
	def add_target_highlights(self, image, target):
		#create a shallow copy of image
		img = copy(image)
		if(len(img.shape) < 3):
			img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

		if target is not None:
			for i in range(0,len(target)):
				cv2.ellipse(img,target[i],(0,255,0),2)
		return img


	@classmethod
	#add_target_highlight- highlight the detected target, 1.the target center 2.each ring
	def add_ring_highlights(self, image, rings = None, ring = None):
		#create a shallow copy of image
		img = np.copy(image)
		if(len(img.shape) < 3):
			img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)


		if rings is not None:
			for r in rings:
				cv2.circle(img,r.center.tuple(), r.inner_circle.radius,(255,0,0), thickness=1)
				cv2.circle(img,r.center.tuple(), r.outer_circle.radius,(255,0,0), thickness=1)
				if r.is_valid():
					x = r.center.x + int(r.inner_circle.radius * math.sin(r.orientation))
					y = r.center.y - int(r.inner_circle.radius * math.cos(r.orientation))
					cv2.line(img,r.center.tuple(),(x,y),(255,255,255), thickness=1 )

		if ring is not None:
			cv2.circle(img,ring.center.tuple(), ring.inner_circle.radius,(0,0,255), thickness=1)
			cv2.circle(img,ring.center.tuple(), ring.outer_circle.radius,(0,0,255), thickness=1)
			if r.is_valid():
				x = r.center.x + int(r.inner_circle.radius * math.sin(r.orientation))
				y = r.center.y - int(r.inner_circle.radius * math.cos(r.orientation))
				cv2.line(img,r.center.tuple(),(x,y),(255,255,255), thickness=1 )
		return img


	@classmethod
	#add_stats - adds a text to the image
	def add_stats(self,img, text,x0,y0):

		font = cv2.FONT_HERSHEY_SIMPLEX
		dy = 15
		for i, line in enumerate(text.split('\n')):
			y = y0 + i*dy
			cv2.putText(img,line,(x0,y), font, 0.45,(255,255,255),1)
		return img
