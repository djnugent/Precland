#!/usr/bin/python
import time
import math

''''
Class with various helper methods
'''


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