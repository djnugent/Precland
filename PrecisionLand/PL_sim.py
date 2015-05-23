#!/usr/bin/python
#SYSTEM IMPORTS
import math
import time
import cv2
import numpy as np


#COMMOM IMPORTS
from VisNav.Common.VN_config import VN_config
from VisNav.Common.VN_util import *
from VisNav.Common.VN_position_vector import PositionVector
from VisNav.Common.VN_vehicle_control import veh_control

#DRONEAPI IMPORTS
from droneapi.lib import VehicleMode, Location, Attitude


'''
TODO 
Intergrate config file

Long term enhancements: 
Add in textured/tiled background
dynamic exposure levels / frame rate 
Add google earth as background
'''



class PrecisionLandSimulator():


	def __init__(self):
		self.targetLocation = PositionVector()
		self.vehicleLocation = PositionVector()

		self.backgroundColor = (74,88,109)


		#load target
		filename = VN_config.get_string('simulator', 'target_location', '/home/dannuge/SmartCamera/target.jpg')
		target_size = VN_config.get_float('algorithm', 'outer_ring', 1.0)
		self.load_target(filename,target_size)


		#define camera
		self.camera_width = VN_config.get_integer('camera', 'width', 640)
		self.camera_height = VN_config.get_integer('camera', 'height', 640)
		self.camera_vfov = VN_config.get_float('camera', 'vertical-fov',72.42 )
		self.camera_hfov = VN_config.get_float('camera', 'horizontal-fov', 72.42)
		self.camera_fov = math.sqrt(self.camera_vfov**2 + self.camera_hfov**2)
		self.camera_frameRate = 30

	#load_target- load an image to simulate the target. Enter the actaul target size in meters(assuming the target is square)
	def load_target(self,filename, actualSize):
		self.target = cv2.imread(filename)
		self.target_width = self.target.shape[1]
		self.target_height = self.target.shape[0]


		self.actualSize = actualSize
		#scaling factor for real dimensions to simultor pixels
		self.pixels_per_meter = (self.target_height + self.target_width) / (2.0 * actualSize) 


	#set_target_location- give the target a gps location
	def set_target_location(self, location):
		self.targetLocation.set_from_location(location)


	#refresh_simulator - update vehicle position info necessary to simulate an image 
	def refresh_simulator(self, vehicleLocation, vehicleAttitude):
		#get gps location of vehicle
		self.vehicleLocation.set_from_location(vehicleLocation)

		self.vehicleAttitude = vehicleAttitude


	#main - code used to test the simulator. Must be run from sitl. Control vehicle in guided mode using arrow keys,r,t,q,e
	def main(self):
		veh_control.connect(local_connect())

		self.set_target_location(veh_control.get_location())

		while(veh_control.is_connected()):
		    location = veh_control.get_location()
		    attitude = veh_control.get_attitude()
		    
		    self.refresh_simulator(location,attitude)
		    frame = self.get_frame()
		    cv2.imshow('frame',frame)

		    key = cv2.waitKey(1)
		    print key

		    if key ==1113938:
		    	veh_control.set_velocity(2,0,0) #forward
		    elif key == 1113940:
		    	veh_control.set_velocity(-2,0,0) #backward
		    elif key == 1113937:
		    	veh_control.set_velocity(0,-2,0) #left
		    elif key ==1113939:
		    	veh_control.set_velocity(0,2,0) #right
		    elif(key == 1048690):
		    	yaw = math.degrees(attitude.yaw) #yaw left
		    	veh_control.set_yaw(yaw - 5)
		    elif(key == 1048692):
		    	yaw = math.degrees(attitude.yaw) #yaw right
		    	veh_control.set_yaw(yaw + 5)
		    elif(key == 1048677):
		    	veh_control.set_velocity(0,0,-2) #down
		    elif(key == 1048689):
		    	veh_control.set_velocity(0,0,2) #up
		    else:
				veh_control.set_velocity(0,0,0) #still


	#project_3D_to_2D - project a 3d point onto a 2d plane. Covert from world perspective to camera perspective
	def project_3D_to_2D(self,thetaX,thetaY,thetaZ, aX, aY,aZ, cX, cY, cZ, height, width, fov):
		dX = math.cos(-thetaY) * (math.sin(-thetaZ)*(cY-aY) + math.cos(-thetaZ)*(cX-aX)) - math.sin(-thetaY)*(aZ-cZ)
		dY = math.sin(-thetaX) * (math.cos(-thetaY)*(aZ-cZ) + math.sin(-thetaY)*(math.sin(-thetaZ)*(cY-aY) + math.cos(-thetaZ)*(cX-aX))) + math.cos(-thetaX)*(math.cos(-thetaZ)*(cY-aY) - math.sin(-thetaZ) * (cX-aX))
		dZ = math.cos(-thetaX) * (math.cos(-thetaY)*(aZ-cZ) + math.sin(-thetaY)*(math.sin(-thetaZ)*(cY-aY) + math.cos(-thetaZ)*(cX-aX))) - math.sin(-thetaX)*(math.cos(-thetaZ)*(cY-aY) - math.sin(-thetaZ) * (cX-aX))

		#veiwer position
		eX = 0
		eY = 0
		eZ = 1.0/math.tan(math.radians(fov)/2.0)

		#2D point
		bX = (dX - eX)*(eZ/dZ)
		bY = (dY - eY)*(eZ/dZ)

		#scaled to resolution
		sX = bX * width
		sY = bY * height

		return (sX,sY)


	#simulate_target - simulate an image given the target position[aX,aY,aZ](pixels)and camera position[cX,cY,cZ](pixels) and camera orientation
	def simulate_target(self,thetaX,thetaY,thetaZ, aX, aY, aZ, cX, cY, cZ, camera_height, camera_width, fov):
		img_width = self.target_width
		img_height = self.target_height

		#point maps
		corners = np.float32([[-img_width/2,img_height/2],[img_width/2 ,img_height/2],[-img_width/2,-img_height/2],[img_width/2, -img_height/2]])
		newCorners = np.float32([[0,0],[0,0],[0,0],[0,0]])


		#calculate projection for four corners of image
		for i in range(0,len(corners)):

			#shift to world
			x = corners[i][0] + cX - img_width/2.0
			y = corners[i][1] + cY - img_height/2.0


			#calculate perspective and position
			x , y = self.project_3D_to_2D(thetaX,thetaY,thetaZ, aY, aX, aZ, y, x, cZ,camera_height,camera_width,fov) 

			#shift to camera
			x , y = shift_to_image((x,y),camera_width,camera_height)
			newCorners[i] = x,y  


		#project image
		M = cv2.getPerspectiveTransform(corners,newCorners)
		sim = cv2.warpPerspective(self.target,M,(self.camera_width,self.camera_height),borderValue=self.backgroundColor)

		return sim



	#get_frame - retreive a simulated camera image
	def get_frame(self):
		start = current_milli_time()

		#distance bewteen camera and target in meters
		aX,aY,aZ = self.targetLocation.x, self.targetLocation.y, self.targetLocation.z
		cX,cY,cZ = self.vehicleLocation.x, self.vehicleLocation.y, self.vehicleLocation.z

		#camera angle
		thetaX = self.vehicleAttitude.pitch
		thetaY = self.vehicleAttitude.roll
		thetaZ = self.vehicleAttitude.yaw

		#convert distance bewtween camera and target in pixels
		aX = aX * self.pixels_per_meter
		aY = aY * self.pixels_per_meter
		aZ = aZ * self.pixels_per_meter
		cX = cX * self.pixels_per_meter
		cY = cY * self.pixels_per_meter
		cZ = cZ * self.pixels_per_meter
		

		#render image
		sim = self.simulate_target(thetaX,thetaY,thetaZ, aX, aY, aZ, cX, cY, cZ, self.camera_height, self.camera_width, self.camera_fov)
		
		#simulate framerate
		while(1000/self.camera_frameRate > current_milli_time() - start):
			pass
		return sim
	

sim = PrecisionLandSimulator()


if __name__ == "__builtin__":
	sim.main()



	



