#!/usr/bin/python
import time
import math
import numpy as np
from Common.VN_position_vector import PositionVector

''''
Class with various helper methods
'''




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

	def int(self):
		self.x = int(self.x)
		self.y = int(self.y)

	def float(self):
		self.x *= 1.0
		self.y *= 1.0

	def __add__(self,other):
		return Point(self.x + other.x, self.y + other.y)

	def __sub__(self,other):
		return Point(self.x - other.x, self.y - other.y)

	def __mul__(self,scalar):
		return Point(self.x * scalar, self.y * scalar)

	def __div__(self,scalar):
		return Point(self.x / scalar, self.y / scalar)

	def __getattr__(self,k):
		if k == 0:
			return x
		if k == 1:
			return y
		raise AttributeError


	def __str__(self):
		return "({0}, {1})".format(self.x,self.y)



class Point3(object):
	def __init__(self, x = None, y = None, z = None, tup = None):
		if tup is not None:
			self.x = tup[0]
			self.y = tup[1]
			self.z = tup[2]
		else:
			self.x = x
			self.y = y
			self.z = z

	def distance_to(self,other):
		return math.sqrt(math.pow(self.x - other.x,2)+ math.pow(self.y - other.y,2) + math.pow(self.z - other.z,2))

	def tuple(self):
		return (self.x,self.y,self.z)

	def __add__(self,other):
		return Point(self.x + other.x, self.y + other.y, self.z + other.z)

	def __sub__(self,other):
		return Point(self.x - other.x, self.y - other.y, self.z - other.z)

	def __getattr__(self,k):
		if k == 0:
			return x
		if k == 1:
			return y
		if k == 2:
			return z
		raise AttributeError

	def __str__(self):
		return "({0}, {1})".format(self.x,self.y)



# current_milli_time - current time in milliseconds
def current_milli_time():
	return int(time.time() * 1000)

# pixels_to_angle - converts a number of pixels into an angle in radians
def pixels_to_angle(num_pixels,fov,img_size):
    return num_pixels * math.radians(fov) / img_size

#shift_to_origin - make the origin of the image (imgwidth/2,imgheight/2)
def shift_to_origin(pt,width,height):
	return ((pt[0] - width/2.0),(-1*pt[1] + height/2.0))


#shift_to_image - make the origin of the image upper left corner
def shift_to_image(pt,width,height):
	return ((pt[0] + width/2),(-1*pt[1] + height/2.0))


# wrap_PI - wraps value between -2*PI ~ +2*PI (i.e. -360 ~ +360 degrees) down to -PI ~ PI (i.e. -180 ~ +180 degrees)
#angle should be in radians
def wrap_PI(angle):
	if (angle > math.pi):
		return (angle - (math.pi * 2.0))
	if (angle < -math.pi):
		return (angle + (math.pi * 2.0))
	return angle


# get_distance_from_pixels - returns distance to balloon in meters given number of pixels in image and expected 0.5m radius
#    size_in_pizels : diameter or radius of the object on the image (in pixels)
#    actual_size : diameter or radius of the object in meters
def get_distance_from_pixels(size_in_pixels, actual_size,fov,img_size):
	 # avoid divide by zero by returning 9999.9 meters for zero sized object
    if (size_in_pixels == 0):
        return 9999.9
    # convert num_pixels to angular size
    return actual_size / pixels_to_angle(size_in_pixels,fov,img_size)



# camera_to_global - take an angular offset from the center of belly mounted camera and turn it into a lat lon point
def camera_to_relative(angular_offset,location, attitude, alt_above_terrain,home):

	home_pos = PositionVector().get_from_location(home)
	veh_pos = PositionVector().get_from_location(location)

	#subtract vehicle lean angles
	x_rad = angular_offset.x - attitude.roll #radians
	y_rad = -angular_offset.y + attitude.pitch

	#rotate to earth-frame angles
	x_ef = y_rad*math.cos(attitude.yaw) - x_rad*math.sin(attitude.yaw) #radians
	y_ef = y_rad*math.sin(attitude.yaw) + x_rad*math.cos(attitude.yaw)
	ef_angle_to_target = Point(x_ef,y_ef)

    #get current altitude (constrained to no lower than 50cm)
	alt = max(alt_above_terrain, 0.5)

    #convert earth-frame angles to earth-frame position offset
	x = alt*math.tan(ef_angle_to_target.x) #meters
	y = alt*math.tan(ef_angle_to_target.y)
	z = 0;  #not used
	target_pos = PositionVector(x,y,z)

	#put in reference to home
	target_pos_relative = target_pos + veh_pos - home_pos

	return target_pos_relative


# camera_to_global - take an angular offset from the center of belly mounted camera and turn it into a lat lon point
def camera_to_global(angular_offset,location, attitude, alt_above_terrain):

	veh_pos = PositionVector().get_from_location(location)

	#subtract vehicle lean angles
	x_rad = angular_offset.x - attitude.roll #radians
	y_rad = -angular_offset.y + attitude.pitch

	#rotate to earth-frame angles
	x_ef = y_rad*math.cos(attitude.yaw) - x_rad*math.sin(attitude.yaw) #radians
	y_ef = y_rad*math.sin(attitude.yaw) + x_rad*math.cos(attitude.yaw)
	ef_angle_to_target = Point(x_ef,y_ef)

    #get current altitude (constrained to no lower than 50cm)
	alt = max(alt_above_terrain, 0.5)

    #convert earth-frame angles to earth-frame position offset
	x = alt*math.tan(ef_angle_to_target.x) #meters
	y = alt*math.tan(ef_angle_to_target.y)
	z = 0;  #not used
	target_pos = PositionVector(x,y,z)

	#put in reference to home
	target_pos_relative = target_pos + veh_pos
	target_pos_relative.z = 0

	return target_pos_relative.get_location()

# crop an image at a specific location.  Center is the center of the image, size is width/height, origin is the upper left of the image
def roi(img,center,size):

	#full image
	(height, width) = img.shape

	#crop image
	x0 = max(center.x - size.x/2,0)
	x1 = min(center.x + size.x/2,width)
	y0 = max(center.y - size.y/2,0)
	y1 = min(center.y + size.y/2,height)

	roi = img[y0:y1, x0:x1]
	origin = Point(center.x-size.x/2,center.y-size.y/2)

	return roi, origin

#balance - improve contrast and adjust brightness in image
#Created By: Tobias Shapinsky
def balance(orig,min_range,res_reduction):

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
		'''
		img /= res_reduction
		img *= res_reduction
		'''
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
