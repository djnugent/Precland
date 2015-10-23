#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
import cv2
import numpy as np
from cv_utils.helpers import *
from cv_utils.dataTypes import *
from cv_utils.benchmark import Benchmark

'''
Possible Optimisations:
speed up canny(fix image blur-> bilateral works well but is slow or write a better adaptive canny)
speed up circle detection
speed up ring detection
'''

perf = Benchmark("Ring_Detector")

class Ring_Detector(object):


	def __init__(self,config):
		#load algorithm constants
		#how round a circle needs to be. Perfect circle = 1
		self.eccentricity = config.get_float('algorithm', 'eccentricity', 0.8)
		#min radius of a circle(possibly make loosely dynamic)
		self.min_radius = config.get_integer('algorithm','min_radius', 9)
		#Minimum ratio while comparing contour area to ellipse area
		self.area_ratio = config.get_float('algorithm','area_ratio', 0.8)
		#Minimum ratio between outer and inner circle area in a ring
		self.min_ring_ratio = config.get_float('algorithm','min_ring_ratio', 0.7)
		#Maximum ratio between outer and inner circle area in a ring
		self.max_ring_ratio = config.get_float('algorithm', 'max_ring_ratio',0.9)


	#analyze_frame_async - process an frame and look for a bullseye asynchronously
	#params -child_conn: child pipe connection
	#		-img: raw image to be processed
	#return -runtime: time in millis to process an image
	#		-center: tuple(x,y) of the objects position on; 'None' when no target
	#		-distance: distance in meters to the target; -1 when unable to calculate
	#		-targetEllipses: ellipses that compose the detected target 'None' when no target
	def analyze_frame_async(self, child_conn, img, frame_id,timestamp,altitude):
		child_conn.send(self.analyze_frame(img,frame_id,timestamp,altitude))


	#analyze_frame - process an frame and look for a bullseye
	#params -child_conn: child pipe connection
	#		-img: raw image to be processed
	#return -runtime: time in millis to process an image        config = Config("precland","~/precland.cnf")

	#		-targetEllipses: ellipses that compose the detected target 'None' when no target
	def analyze_frame(self, img, frame_id,timestamp,altitude):
		#start timer
		start = int(time.time() * 1000)

		#check for a colored image
		if(len(img.shape)>2):
			#grayscale image
			img = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

		# Blur image
		perf.enter()
		blur = cv2.GaussianBlur(img,(5,5),0)
		#blur = cv2.bilateralFilter(img,9,75,75)
		perf.exit(function = 'blur')
		filtered = blur
		#cv2.imshow("blur",blur)

		#average brightness
		avg, null, null, null = cv2.mean(filtered)
		avg = int(avg)

		#canny edge detector
		perf.enter()
		filtered = cv2.Canny(filtered,avg/2,avg)
		#filtered = self.auto_canny(filtered)
		perf.exit(function = 'canny')
		canny = filtered
		#cv2.imshow("edge", canny)


		#detect circles
		perf.enter()
		circles = self.detect_circles(filtered,self.eccentricity, self.area_ratio, min_radius = self.min_radius)


		perf.exit(function = 'detect_circles')

		rings = []
		best_ring = None
		#turn circles into rings
		if len(circles) > 0:
			perf.enter()
			rings = self.detect_rings(circles,self.min_ring_ratio,self.max_ring_ratio)
			perf.exit(function = 'detect_rings')

		perf.enter()
		#find smallest ring with orientation(if available)
		min_code = 256
		for i in range(0,len(rings)):
			ring = rings[i]
			bal = ring.decode(blur)
			if ring.code < min_code:
				best_ring = ring
			cv2.imshow("roi{0}".format(i),bal)

		perf.exit(function = 'decodes')


		stop = int(time.time() * 1000)
		#perf.print_package("Ring_Detector")
		perf.clear()
		return ((frame_id,timestamp,altitude), stop-start,best_ring,rings)



	#detect_rings- Find circles that are nested inside of each other
	def detect_rings(self,rawCircles,min_ratio,max_ratio, no_overlap = True):
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


					#check if a circle is nested within another circle and is the correct size relative to the other
					if distance < abs(radius1 - radius2):
						new_ring = None
						if (radius1 < radius2) and (radius1 * 1.0 /radius2 > min_ratio) and (radius1 * 1.0 /radius2 < max_ratio):
							#small circle, big circle
							new_ring = Ring(circle1, circle2)

						elif (radius1 > radius2) and (radius2 * 1.0 / radius1 > min_ratio) and (radius2 * 1.0 / radius1 < max_ratio):
							#small circle, big circle
							new_ring = Ring(circle2, circle1)

					 	if new_ring is not None:
							#check for overlap if one or more rings already exists
							if no_overlap and ring_count > 0:
								bad_rings = []
								drop_new = False
								for k in range(0,len(rings)):
									exist_ring = rings[k]
									if exist_ring is None:
											break
									else:
										#if rings overlap then keep the one with smallest inner radius
										distance = exist_ring.center.distance_to(new_ring.center)
										if distance < (exist_ring.outer_circle.radius + new_ring.outer_circle.radius):
											if exist_ring.inner_circle.radius > new_ring.inner_circle.radius:
												bad_rings.append(k)
											else:
												drop_new = True
								#remove overlapping rings
								if not drop_new:
									#remove bad rings
									rings = np.delete(rings,bad_rings)
									ring_count -= len(bad_rings)
									#add new ring
									rings[ring_count] = new_ring
									ring_count += 1
								#else dont add new ring
								break;
							else:
								rings[ring_count] = new_ring
								ring_count += 1
							break

		#remove null objects
		rings  = np.resize(rings,ring_count)

		return rings


	def detect_circles(self, orig, eccentricity, area_ratio, min_radius = 0, max_radius = 500000):

		img = np.copy(orig)

		#locate contours
		contours, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

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
					return Circle(center,radius,contour,ellipse)
		return None





class Circle(object):

	def __init__(self, center, radius, contour = None, ellipse = None):
		self.center = center
		self.radius = radius
		self.contour = contour
		self.ellipse = ellipse

	def __str__(self):
		return "Center: {0} Radius: {1}".format(self.center,self.radius)


class Ring(object):
	def __init__(self, inner_circle, outer_circle, center = None, radius = None, orientation = None, code = None):
		self.inner_circle = inner_circle
		self.outer_circle = outer_circle
		self.center = center
		self.radius = radius
		self.orientation = orientation
		self.code = code
		self.calc_radius(0)
		self.calc_center(0)

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
		return Point3(x_angle,y_angle, self.orientation)

	def decode(self,img):

		#find border around contour and crop a ROI
		x,y,w,h = cv2.boundingRect(self.inner_circle.contour)
		img,origin = roi(img,self.inner_circle.center, Point(x = w, y = h))

		#Normilize image
		bal = balance(img,5,1)
		#adaptive threshold
		avg, null, null, null = cv2.mean(bal)
		avg = int(avg)
		ret,filtered = cv2.threshold(bal,avg,255,cv2.THRESH_BINARY)

		perf.enter()

		#extract shape,size,location of image
		ellipse = self.inner_circle.ellipse
		center = Point(tup = ellipse[0]) - origin
		size = Point(tup = ellipse[1])/2
		angle = np.radians(ellipse[2])

		steps = 72
		code_length = 6
		#scan image
		scan = np.zeros((steps),np.int8)
		for deg in range(0,steps):
			rad = math.radians(deg * 360.0 / steps)
			#scan an ellipse
			radius = size.x * size.y / np.sqrt(math.pow(size.y*np.sin(rad-angle),2) + math.pow(size.x*np.cos(rad-angle),2))

			scan_lines = (0.65,0.7,0.76,0.8)

			pix = 0.0
			for i in scan_lines:
				r = int(round(radius * i))
				cart = Point(x = r * np.sin(rad),y = -r * np.cos(rad))
				cart += center
				cart.x, cart.y = int(round(cart.x)), int(round(cart.y))
				cart.x, cart.y = max(0,cart.x), max(0,cart.y)
				cart.x, cart.y = min(cart.x,img.shape[1] - 1), min(cart.y,img.shape[0] - 1)
				pix += filtered.item(cart.y,cart.x) /255.0
				#cv2.circle(bal,cart.tuple(),0,color=(127))

			pix /= len(scan_lines)
			pix = int(round(pix))
			scan.itemset(deg,pix)


		perf.exit(function = 'scan')

		perf.enter()
		#denoise scan
		for i in range(0,steps):
			#compare value to neighbors
			depth = code_length / (3 * steps)
			low = (i - depth) % steps
			high = (i + depth) % steps + 1
			if low > high:
				scan.itemset(i,int(round(np.average(np.concatenate((scan[low:steps], scan[0:high]))))))
			else:
				scan.itemset(i,int(round(np.average(scan[low:high]))))



		perf.exit(function = 'denoise')

		if 0.6 > np.average(scan) > 0.4:
			perf.enter()
			#detect edges
			edges = []
			for i in range(0,steps):
				j = (i + 1)%steps
				a = scan.item(i)
				b = scan.item(j)
				if a == 1 and b == 0:
					edges.append((j,'falling'))
				if a == 0 and b == 1 :
					edges.append((j, 'rising'))


			perf.exit(function = 'detect edges')

			#detect segments
			segments = []
			if len(edges) == code_length:
				for i in range(0,code_length):
					a = edges[i]
					b = edges[(i+1)%code_length]
					if a[1] == 'rising':
						bit = 1
						orientation = a[0]
						length = None
						if a[0] > b[0]:
							length = (steps - a[0]) + b[0]
						else:
							length = b[0] - a[0]
					if a[1] == 'falling':
						bit = 0
						orientation = a[0]
						length = None
						if a[0] > b[0]:
							length = (steps - a[0]) + b[0]
						else:
							length = b[0] - a[0]
					segments.append((bit,orientation,length))

				perf.enter()
				#extract target orientation
				target_orientation = 0
				bit_shift = 0
				for i in range(0,len(segments)):
					seg = segments[i]
					bit,orient,length = seg
					#round up > 0.7
					m = math.modf(length* 8.0/steps)
					width = int(m[1]) + (1 if m[0] > 0.7 else 0)
					if width == 2 and bit == 1:
						self.orientation = orient * 360.0/steps
						bit_shift = i



				perf.exit(function = 'target_orientation')

				perf.enter()
				#read code
				bit_count = 7
				self.code = 0
				for i in range(0,len(segments)):
					seg = segments[(i + bit_shift) % len(segments)]
					bit,orientation,length = seg
					width = int(round(length * 8.0/steps))

					for j in range(0,width):
						self.code |= bit << abs(bit_count) #FIXME neg bitcount
						bit_count -= 1



				perf.exit(function = 'read_code')
				print "Code: {0}, Orient: {1} degs".format(self.code,self.orientation)

		return bal

	def is_valid(self):
		return (self.orientation is not None)

	def __str__(self):
		return "Center: {0} Radius: {1} Orientation: {2} radians".format(self.center,self.radius,self.orientation)

	# render_overlay- highlight the detected target
	def render_overlay(self, img, color):

		if(len(img.shape) < 3):
			img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

		#draw this ring
		cv2.circle(img,self.center.tuple(), self.inner_circle.radius,color, thickness=1)
		cv2.circle(img,self.center.tuple(), self.outer_circle.radius,color, thickness=1)
		if self.is_valid():
			x = self.center.x + int(self.inner_circle.radius * math.sin(self.orientation))
			y = self.center.y - int(self.inner_circle.radius * math.cos(self.orientation))
			cv2.line(img,self.center.tuple(),(x,y),(255,255,255), thickness=1 )




		return img


if __name__ == "__main__":
	import argparse
	from Flow_Camera import flow_cam
	from GUI import PrecisionLandGUI as gui
	from cv_utils.ImageRW import *
	from cv_utils.config import Config
	#parse arguments
	parser = argparse.ArgumentParser(description="Run Ring_Detector")
	#optional arguments
	parser.add_argument('-c', '--camera', default=0, help='Camera source: index, file, or stream')
	parser.add_argument('-i', '--input', default= None, help='use an image folder as an input instead of a camera input')
	parser.add_argument('-f', '--file', default='~/precland_default.cnf', help='load a config file other than the default')
	parser.add_argument('-w', '--write', default=False)

	args, unknown = parser.parse_known_args()


	#config file
	config = Config('Ring_Detector', args.file)

	#video writer
	if(args.write):
		new_file = args.input.replace('raw','gui')
		if(new_file == args.input):
			print "Overwrite protection"
			sys.exit()

		writer = ImageWriter(args.input.replace('raw','gui'))


	#video source
	cam = None

	if args.input is None:
		cam = cv2.VideoCapture(args.camera)
	else:
		cam = ImageReader(args.input)


	detector = Ring_Detector(config)
	frame_id = 0
	if cam is not None and cam.isOpened():
		while True:
			#print frame_id
			ret, img = cam.read()
			if(img is not None and ret == True):
				results = detector.analyze_frame(img,frame_id,0,0)
				frame_id += 1

				#unpack data
				frame_id, timestamp, altitude = results[0]
				best_ring = results[2]
				rings = results[3]

				#show results
				yaw , radius = None, None
				rend_Image = np.copy(img)
				if best_ring is not None:
					for r in rings:
						rend_Image = r.render_overlay(rend_Image,(255,0,0))
					rend_Image = best_ring.render_overlay(rend_Image,(0,0,255))
					radius = best_ring.radius
					if best_ring.is_valid():
						yaw = int(math.degrees(results[2].orientation))

				#overlay stats
				status_text = '{0} Rings\n{1} ms\n{2} degs\n{3} radius\n{4} meters'.format(len(results[3]), results[1], yaw, radius, 0)
				rend_Image = add_stats(rend_Image,status_text,5, 250)
				if(args.write):
					writer.write(rend_Image)

				cv2.imshow('gui', rend_Image)
				cv2.waitKey(1)
				print status_text.replace('\n', ', ')

			else:
				print "failed to grab image"
				break;
	else:
		print "No video source detected"
