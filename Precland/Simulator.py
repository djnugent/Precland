#!/usr/bin/python
#SYSTEM IMPORTS
import sys
from os.path import expanduser
import math
import time
import cv2
import numpy as np
from cv_utils.position_vector import PositionVector
from cv_utils.helpers import *
from cv_utils.transforms import *
from droneapi.lib import VehicleMode, Location, Attitude


'''
TODO
fix warped aspect ratios
'''



class PrecisionLandSimulator():


    def __init__(self,config):


        self.targetLocation = PositionVector()
        self.vehicleLocation = PositionVector()

        self.backgroundColor = (209,209,209)

        #define camera
        self.camera_width = config.get_integer('simulator','width',640)
        self.camera_height = config.get_integer('simulator','height',480)
        self.camera_hfov = config.get_float('camera','hfov',72.3)
        self.camera_vfov = config.get_float('camera','vfov',46)
        self.has_gimbal = config.get_boolean('camera','has_gimbal',False)


        #define environment
        self.simulator_framerate = config.get_integer('simulator','frame_rate',30)
        self.target_size = config.get_float('simulator','target_size',0.75)
        self.target_location = config.get_string('simulator','target_location','~/precland/Targets/mat_v1.jpg')


        self.camera_fov = math.sqrt(self.camera_vfov**2 + self.camera_hfov**2)
        self.last_update_time = 0

        self.load_target()

    #load_target- load an image to simulate the target. Enter the actaul target size in meters(assuming the target is square)
    def load_target(self):
        self.target = cv2.imread(expanduser(self.target_location))
        if self.target is None:
                print "Unable to load target image!"
                sys.exit(0)
        self.target_width = self.target.shape[1]
        self.target_height = self.target.shape[0]

        #scaling factor for real dimensions to simultor pixels
        self.pixels_per_meter = (self.target_height + self.target_width) / (2.0 * self.target_size)


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
            ret,frame = self.get_image()
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



    #get_image - retreive a simulated camera image
    def get_image(self):
        start = int(time.time() * 1000)

        #distance bewteen camera and target in meters
        aX,aY,aZ = self.targetLocation.x, self.targetLocation.y, self.targetLocation.z
        cX,cY,cZ = self.vehicleLocation.x, self.vehicleLocation.y, self.vehicleLocation.z

        #camera angle
        thetaX = self.vehicleAttitude.pitch
        thetaY = self.vehicleAttitude.roll
        thetaZ = self.vehicleAttitude.yaw

        if self.has_gimbal:
            thetaX, thetaY = 0,0

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
        #constrain framerate
        while self.simulator_framerate != 0 and (time.time() - self.last_update_time < (1.0 / self.simulator_framerate )):
            pass
        self.last_update_time = time.time()

        return True,sim




if __name__ == "__builtin__":
    #load config file
    config = Config("precland","~/precland_default.cnf")
    sim = PrecisionLandSimulator(config)
    sim.main()
